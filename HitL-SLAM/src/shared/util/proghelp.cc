#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "timer.h"

#include "proghelp.h"


static bool *enable_run;
static int grace_time;


size_t CopyFile(FILE *source,FILE *dest)
{
  char buf[256];
  size_t len, total=0;

  while((len = fread(buf,sizeof(char),256,source)) > 0){
    fwrite(buf,sizeof(char),len,dest);
    total += len;
  }

  return(total);
}

void HandleStop(int i)
// Signal handler for breaks (Ctrl-C)
{
  // clear running flag, and set up alarm in case we've hung
  alarm(grace_time);
  if(enable_run) *enable_run = false;
}

void HandleAlarm(int i)
// Signal handler that forces an exit when the code hangs on Ctrl-C
{
  exit(0);
}

void InitHandleStop(bool *_enable_run,int _grace_time)
{
  enable_run = _enable_run;
  grace_time = (_grace_time > 0)? _grace_time : 1;

  // Connect the stop signals
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleAlarm);
}

bool Renice(int nice,bool verbose)
{
  int old_prio = getpriority(PRIO_PROCESS,0);
  setpriority(PRIO_PROCESS,0,nice);
  int new_prio = getpriority(PRIO_PROCESS,0);

  if(verbose && new_prio!=old_prio){
    printf("renice: %d -> %d\n",old_prio,new_prio);
  }
  return(new_prio == nice);
}

void GetDateStr(CharString &date)
{
  struct tm cur;
  GetDate(cur);
  date.printf("%04d%02d%02d-%02d%02d",
              1900+cur.tm_year, cur.tm_mon+1, cur.tm_mday,
              cur.tm_hour, cur.tm_min);
}

bool SetTimer(uint64_t interval)
{
  const uint64_t seconds = interval / static_cast<uint64_t>(1000000);
  const uint64_t useconds = interval % static_cast<uint64_t>(1000000);

  struct itimerval value;
  value.it_interval.tv_sec = seconds;
  value.it_interval.tv_usec = useconds;
  value.it_value.tv_sec = seconds;
  value.it_value.tv_usec = useconds;

  if (setitimer(ITIMER_REAL, &value, NULL) != 0) {
    perror("ERROR calling setitimer");
    return false;
  }
  return true;
}

bool SetTimerInterrupt(uint64_t interval, void (*callback)(int))
{
  if (!SetTimer(interval)) return false;

  struct sigaction sact;
  sigemptyset( &sact.sa_mask );
  sact.sa_flags = 0;
  sact.sa_handler = callback;

  if (sigaction(SIGALRM, &sact, NULL) != 0) {
    perror("ERROR calling sigaction");
    return false;
  }
  return true;
}

void TimerWait()
{
  sigset_t signal_set;
  sigemptyset(&signal_set);
  sigaddset(&signal_set, SIGALRM);
  int signal_number = 0;
  const int error = sigwait(&signal_set, &signal_number);
  if (error != 0 || signal_number != SIGALRM) {
    fprintf(stderr, "ERROR call sigwait\n");
  }
}

void CancelTimerInterrupts()
{
  struct itimerval value;
  int whichTimer = ITIMER_REAL;
  getitimer( whichTimer, &value );
  value.it_value.tv_sec = 0;
  value.it_value.tv_usec = 0;
  setitimer( whichTimer, &value, NULL );
}
