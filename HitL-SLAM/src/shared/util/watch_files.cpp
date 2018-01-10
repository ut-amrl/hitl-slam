#include <sys/types.h>
#include <sys/stat.h>

#include <fcntl.h>
#include <stdio.h>

#include "watch_files.h"


static const bool Debug = false;

/*
  [ References ]

  Inofity Documentation and FAQ
    http://inotify.aiken.cz/?section=inotify&page=doc&lang=en
    http://inotify.aiken.cz/?section=inotify&page=faq&lang=en
*/

//====================================================================//

time_t FileModTime(const char *filename)
{
  struct stat st;
  st.st_mtime = 0;
  stat(filename,&st);
  return(st.st_mtime);
}

void SetNonBlocking(int fd)
{
  int flags = fcntl(fd, F_GETFL, 0);
  if(flags < 0) flags = 0;
  fcntl(fd, F_SETFL, flags|O_NONBLOCK);
}

//====================================================================//

time_t ActiveFile::getFileModTime()
{
  return(FileModTime(name()));
}

//====================================================================//

bool WatchFiles::init()
{
  reset();

  inotify_fd = inotify_init();
  if(inotify_fd < 0) return(false);
  SetNonBlocking(inotify_fd);

  return(true);
}

void WatchFiles::reset()
{
  if(Debug){
    printf("WatchFiles: reset (num=%d)\n",num_watches);
  }

  if(isInited()){
    close(inotify_fd);
    inotify_fd = -1;
  }
  events.clear();
  num_watches=0;
}

bool WatchFiles::addWatch(Watch &w,const char *filename)
{
  // initialize if needed
  if(!isInited() && !init()) return(false);

  // remove existing watch if present
  if(w.parent) removeWatch(w);

  // add watch for file
  w.wd = inotify_add_watch(inotify_fd, filename, ModEvents);
  if(w.wd < 0) return(false);
  w.parent = this;
  num_watches++;

  if(Debug){
    printf("WatchFiles: watching \"%s\" (wd=%d)\n",
           filename,w.wd);
  }

  return(true);
}

bool WatchFiles::removeWatch(Watch &w)
{
  if(w.parent != this) return(false);

  if(Debug){
    printf("WatchFiles: removing wd=%d\n",w.wd);
  }

  int ret = inotify_rm_watch(inotify_fd, w.wd);
  w.parent = NULL;
  w.wd = -1;
  if(ret == 0) num_watches--;

  return(ret == 0);
}

uint32_t WatchFiles::calcEventMask(Watch &w)
{
  uint32_t mask = 0;
  for(unsigned i=0; i<events.size(); i++){
    if(events[i].wd == w.wd) mask |= events[i].mask;
  }
  return(mask);
}

int WatchFiles::getEvents()
{
  static const int BufSize = 512;
  char buf[BufSize];

  while(true){
    // get a buffer of events
    int nr = read(inotify_fd,buf,BufSize);
    if(Debug){
      printf("Read inotify: %d\n",nr);
    }
    if(nr < (int)sizeof(inotify_event)) break;
    
    // add them to our interal event queue
    int i=0;
    while(i < nr){
      const inotify_event &e = *(inotify_event*)(buf+i);
      events.push_back(e);
      i += sizeof(inotify_event) + e.len;

      if(Debug){
        printf("WatchFiles: wd=%d ev=0x%X",e.mask,e.wd);
        if(e.len) printf(" len=%d name=\"%s\"",e.len,e.name);
        printf("\n");
      }
    }
  }

  return(events.size());
}
