#include <stdint.h>

#include "sstring.h"

#ifndef __PROG_HELP_H__
#define __PROG_HELP_H__

void InitHandleStop(bool *_enable_run=NULL,int _grace_time=0);
bool Renice(int nice,bool verbose=true);

const char *GetProjectRoot();
void Usage(const char *program_path);

// get a string for the current date
void GetDateStr(CharString &date);

bool StrToInt(int &val,const char *str,int base=10);

// SetTimer: Used to set a POSIX itimer.
// Upon succesful execution, the signal SIGALRM will be generated for the
// calling process every @interval microseconds. On success returns true,
// and on failure prints the cause of error to stderr and returns false.
// Arguments:
// interval : interval of interrupt in micro-seconds
bool SetTimer(uint64_t interval);

// TimerWait: Block execution until SIGALRM is received, typically from
// a POSIX itimer previously set by SetTimer.
void TimerWait();

// SetTimerInterrupt: Used to set a POSIX itimer interrupt.
// Upon succesful execution, the signal SIGALRM will be generated for the
// calling process every @interval microseconds, and the @callback function
// will be called. On success returns true, and on failure prints the cause of
// error to stderr and returns false.
// Arguments:
// interval : interval of interrupt in micro-seconds
// callback : pointer to function used as callback
bool SetTimerInterrupt(uint64_t interval, void (*callback)(int));

// CancelTimerInterrupts: Used to Cancel all existing timer interrupts.
void CancelTimerInterrupts();

#endif
