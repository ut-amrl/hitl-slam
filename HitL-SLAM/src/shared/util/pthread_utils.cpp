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
// Copyright 2013 joydeepb@ri.cmu.edu
// Robotics Institute, Carnegie Mellon University
//
// A simple ScopedLock to work with pthread mutexes.

#include <pthread.h>
#include <stdio.h>

#include "pthread_utils.h"

ScopedLock::ScopedLock(pthread_mutex_t& mutex) : mutex_(mutex) {
  pthread_mutex_lock(&mutex_);
}

ScopedLock::~ScopedLock() {
  pthread_mutex_unlock(&mutex_);
}

ScopedTryLock::ScopedTryLock(pthread_mutex_t& mutex) :
    locked_(false), checked_(false), mutex_(mutex) {
  locked_ = (pthread_mutex_trylock(&mutex_) == 0);
}

bool ScopedTryLock::Locked() {
  checked_ = true;
  return locked_;
}

ScopedTryLock::~ScopedTryLock() {
  // Make sure that the caller checked if the mutex was locked. If not,
  // such an occurrence will lead to concurrency problems, and is hence
  // reported as an error.
  assert(checked_);
  if (locked_) pthread_mutex_unlock(&mutex_);
}
