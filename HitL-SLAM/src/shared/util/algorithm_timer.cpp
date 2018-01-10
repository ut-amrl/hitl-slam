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

#include "algorithm_timer.h"
#include <pthread.h>
#include <stdio.h>
#include <time.h>
#include "pthread_utils.h"
#include "timer.h"

namespace algorithm_timer {

AlgorithmTimer::AlgorithmTimer(TimerCollection& collection,
                               const char *name, 
                               const char* file_name,
                               const int line_number) : 
    collection_(&collection),
    measurement_(name, file_name) {
  Measure(line_number);
}

double AlgorithmTimer::GetTickCount() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC,&ts);
  return(double(ts.tv_sec) + double(ts.tv_nsec)*(1.0E-9));
}

void AlgorithmTimer::Measure(const int line_number) {
    measurement_.line_numbers.push_back(line_number);
    measurement_.times.push_back(GetTickCount());
}

void AlgorithmTimer::Lap(int line_number) {
  Measure(line_number);
}

AlgorithmTimer::~AlgorithmTimer() {
  measurement_.times.push_back(GetTickCount());
  collection_->AddMeasurement(measurement_);
}

TimerCollection::TimerCollection() {
  pthread_mutex_init(&mutex_, NULL);
}

TimerCollection::~TimerCollection() {
  // Save all measurements to disk.
  // Print Summary.
  for (size_t i = 0; i < measurements_.size(); ++i) {
    printf("%s\n  %s:\n", measurements_[i].name, measurements_[i].file_name);
    for (size_t j = 0; j < measurements_[i].line_numbers.size(); ++j) {
      printf("  %d %f\n", 
             measurements_[i].line_numbers[j],
             measurements_[i].times[j]);
    }
  }
}

void TimerCollection::AddMeasurement(
    const AlgorithmTimerMeasurement& measurement) {
  ScopedLock lock(mutex_);
  measurements_.push_back(measurement);
}

}  // namespace algorithm_timer

