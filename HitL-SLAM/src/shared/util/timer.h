/*========================================================================
    Timer.h : Interval timers, cycle counters, and time utilities
  ------------------------------------------------------------------------
    Copyright (C) 1999-2002  James R. Bruce
    School of Computer Science, Carnegie Mellon University
  ------------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ========================================================================*/

#ifndef __TIMER_H__
#define __TIMER_H__

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#include <sys/time.h>
#include <time.h>

class Timer{
  timespec tv1,tv2;
public:
  void start()  {clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&tv1);}
  void stop()   {clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&tv2);}
  void end()    {stop();}
  double time() {return(double(tv2.tv_sec - tv1.tv_sec) + double(tv2.tv_nsec - tv1.tv_nsec) * 1.0E-9);}
  double timeMSec() {return(time() * 1.0E3);}
  double timeUSec() {return(time() * 1.0E6);}

  double interval(){
    double t;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&tv2);
    t = time();
    tv1 = tv2;
    return(t);
  }
  double midtime() {
    timespec tmp;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&tmp);
    return(double(tmp.tv_sec - tv1.tv_sec) + double(tmp.tv_nsec - tv1.tv_nsec) * 1.0E-9);
  }
};

class AccumulativeTimer{
  timespec tv1,tv2;
  double total;
public:
  AccumulativeTimer() {
    clear();
  }
  void clear() {
    total=0.0;
  }
  void start()  {clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&tv1);}
  void stop()
  {
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&tv2);
    total+=time();
  }
  double getTotal() {return total;}
  void end() {stop();}
  double time() {return(double(tv2.tv_sec - tv1.tv_sec) + double(tv2.tv_nsec - tv1.tv_nsec) * 1.0E-9);}
  double timeMSec() {return(time() * 1.0E3);}
  double timeUSec() {return(time() * 1.0E6);}

  double interval(){
    double t;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&tv2);
    t = time();
    tv1 = tv2;
    return(t);
  }
  double midtime() {
    timespec tmp;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&tmp);
    return(double(tmp.tv_sec - tv1.tv_sec) + double(tmp.tv_nsec - tv1.tv_nsec) * 1.0E-9);
  }
};

inline unsigned GetTimeUSec()
{
#ifdef Apertos
  struct SystemTime time;
  GetSystemTime(&time);
  return(time.seconds*1000000 + time.useconds);
#else
  timespec ts;
  clock_gettime(CLOCK_REALTIME,&ts);
  return(double(ts.tv_sec)*1000000.0 + double(ts.tv_nsec)*(1.0E-3));
#endif
}

inline double GetTimeSec()
{
#ifdef Apertos
  struct SystemTime time;
  GetSystemTime(&time);
  return((double)time.seconds + time.useconds*(1.0E-6));
#else
  timespec ts;
  clock_gettime(CLOCK_REALTIME,&ts);
  return(double(ts.tv_sec) + double(ts.tv_nsec)*(1.0E-9));
#endif
}

inline double GetProcessTimeSec()
{
  timespec ts;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&ts);
  return(double(ts.tv_sec) + double(ts.tv_nsec)*(1.0E-9));
}

inline void GetDate(struct tm &date)
{
  time_t t = time(NULL);
  localtime_r(&t,&date);
}

inline void Sleep(double sec)
{
  usleep((int)(sec * 1E6));
}

class FunctionTimer{
  const char *fname;
  double tStart, lapStart, tStop;
  int lapNum;
  double getTickCount() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC,&ts);
    return(double(ts.tv_sec) + double(ts.tv_nsec)*(1.0E-9));
  }

public:
  FunctionTimer(const char *_fname) : fname(_fname), tStop(0.0), lapNum(0) {
    tStart = lapStart = getTickCount();
  }

  void Lap(int lineNum) {
    tStop=getTickCount();
    printf("%s line %d lap %d: %12.3fus\n",
           fname, lineNum, lapNum,
           (tStop-lapStart)*1e6);
    lapNum++;
    lapStart=tStop;
  }

  ~FunctionTimer() {
    tStop=getTickCount();
    printf("%s: %0.6fms\n",fname,(tStop-tStart)*1000.0);
  }
};


#endif
