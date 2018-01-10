#ifndef _INCLUDED_WATCH_FILES_
#define _INCLUDED_WATCH_FILES_

#include <sys/inotify.h>

#include <vector>

#include <unistd.h>
#include <fstream>

#include "sstring.h"


class ActiveFile{
public:
  CharString name;
protected:
  time_t mod_time;
protected:
  time_t getFileModTime();
public:
  ActiveFile()
    {mod_time=0;}

  void init(const char *_name)
    {name=_name; mod_time=0;}
  const char *operator()()
    {return(name());}

  void invalidate()
    {mod_time=0;}
  bool isModified()
    {return(mod_time != getFileModTime());}
  void markAsRead()
    {mod_time = getFileModTime();}
};

class WatchFiles{
protected:
  static const uint32_t ModEvents = IN_CLOSE_WRITE|IN_MOVE_SELF;

public:
  class Watch{
  protected:
    WatchFiles *parent;
    int wd; // watch descriptor
  public:
    Watch()
      {parent=NULL; wd=-1;}
    Watch(const Watch &w)
      {parent=w.parent; wd=w.wd;}

    bool valid()
      {return(parent!=NULL && wd>=0);}
    bool watch(WatchFiles *_parent,const char *filename)
      {return(_parent && _parent->addWatch(*this,filename));}
    bool rewatch(const char *filename)
      {return(parent && parent->addWatch(*this,filename));}
    bool remove()
      {return(parent && parent->removeWatch(*this));}

    bool isFileModified()
      {return(parent && (parent->calcEventMask(*this) & ModEvents));}

    friend class WatchFiles;
  };

protected:
  std::vector<inotify_event> events;
  int inotify_fd;
  int num_watches;
public:
  WatchFiles()
    {inotify_fd=-1; num_watches=0;}
  ~WatchFiles()
    {reset();}

  bool init();
  void reset();
  bool isInited()
    {return(inotify_fd >= 0);}

  bool addWatch(Watch &w,const char *filename);
  bool removeWatch(Watch &w);
  uint32_t calcEventMask(Watch &w);

  int getEvents();
  int getNumEvents()
    {return(events.size());}
  void clearEvents()
    {events.clear();}
};

#endif
