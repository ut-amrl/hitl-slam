//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
// Copyright 2016 joydeepb@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Timers to profile algorithm execution times.

#include <pthread.h>
#include <vector>

#ifndef __ALGORITHM_TIMER_H__
#define __ALGORITHM_TIMER_H__

namespace algorithm_timer {

class TimerCollection;

struct AlgorithmTimerMeasurement{
  AlgorithmTimerMeasurement(const char *name, const char* file_name) : 
      name(name), file_name(file_name) {}
  const char* name;
  const char* file_name;
  std::vector<int> line_numbers;
  std::vector<double> times;
};

class AlgorithmTimer{
  TimerCollection* collection_;
  AlgorithmTimerMeasurement measurement_;

public:

  AlgorithmTimer(TimerCollection& collection,
                 const char *name, 
                 const char* file_name ,
                 const int line_number);
  ~AlgorithmTimer();
  double GetTickCount();
  void Measure(const int line_number);
  void Lap(int line_number);
};


class TimerCollection{
public:
  TimerCollection();
  ~TimerCollection();
  void AddMeasurement(const AlgorithmTimerMeasurement& measurement);
private:
  pthread_mutex_t mutex_;
  std::vector<AlgorithmTimerMeasurement> measurements_;
};

}  // namespace algorithm_timer

#endif  // __ALGORITHM_TIMER_H__



