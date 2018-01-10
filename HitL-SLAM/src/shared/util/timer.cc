#include "timer.h"

double GetCPUClockRateMHz()
{
#ifdef linux
  static double mhz = 0.0;
  if(mhz > 0.0) return(mhz);

  FILE *in = fopen("/proc/cpuinfo","rt");
  const int length=64;
  char buf[length],*s;

  if(in){
    while(fgets(buf,length,in)){
      if(strncmp(buf,"cpu MHz",7) == 0){
        s = strchr(buf,':');
        if(*s) s++;
        mhz = atof(s);
      }
    }
    // while(!feof(in) && fscanf(in,"cpu MHz%*s:%lf\n",&mhz)!=1);
    fclose(in);
  }
  // printf("mhz=%f\n",mhz);
  return(mhz);
#endif

#ifdef Apertos
  // Return constant, since supercore Aibos and the SDR are 400MHz
  // should eventually figure out how to actually get this properly
  return(400);
#endif

  // default, fail
  return(0.0);
}

double GetCPUClockPeriod()
{
  static double period = 0.0;
  if(period != 0.0) return(period);

  period = 1E-6 / GetCPUClockRateMHz();
  return(period);
}
