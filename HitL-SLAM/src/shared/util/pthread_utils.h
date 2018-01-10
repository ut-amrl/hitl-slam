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
// Helper classes to make thread safety management simpler.

#include <assert.h>
#include <glog/logging.h>
#include <pthread.h>

#ifndef PTHREAD_UTILS_H
#define PTHREAD_UTILS_H

class ScopedLock {
 public:
  explicit ScopedLock(pthread_mutex_t& mutex);
  ~ScopedLock();

 private:
  // Disallow default constructors and copy constructors.
  ScopedLock();
  ScopedLock(const ScopedLock&);
  void operator=(const ScopedLock&);

  pthread_mutex_t& mutex_;
};

class ScopedTryLock {
 public:
  explicit ScopedTryLock(pthread_mutex_t& mutex);
  ~ScopedTryLock();
  bool Locked();

 private:
  // Disallow default constructors and copy constructors.
  ScopedTryLock();
  ScopedTryLock(const ScopedLock&);
  void operator=(const ScopedLock&);

  bool locked_;
  bool checked_;
  pthread_mutex_t& mutex_;
};

template <typename T>
class ThreadSafe {
 public:
  // Default constructor.
  ThreadSafe() : locked_(false) {
    const int init_error = pthread_mutex_init(&mutex_, NULL);
    DCHECK(init_error == 0);
  }

  // Initialization constructor.
  explicit ThreadSafe(const T& value) : value_(value), locked_(false) {
    const int init_error = pthread_mutex_init(&mutex_, NULL);
    DCHECK(init_error == 0);
  }

  // Destructor.
  ~ThreadSafe() {
    // The mutex should not still be locked at the time of destruction.
    DCHECK(locked_ == false);
  }

  // Disabled copy constructor.
  ThreadSafe(const ThreadSafe<T>& other);

  // Disable assignment operator.
  void operator=(const ThreadSafe<T>& other);

  // Set the value of the underlying type in a thread-safe manner.
  template <typename T_Other>
  void Set(const T_Other& rvalue) {
    ScopedLock lock(mutex_);
    value_ = rvalue;
  }

  // Get a copy of the value of the underlying type in a thread-safe manner.
  T Get() const {
    T value;
    ScopedLock lock(mutex_);
    value = value_;
    return (value);
  }

  // Get a mutable reference to the value of the underlying type after locking
  // the protecting mutex. This is useful for the "Read" step of
  // "Read-Modify-Write" cases where the value neads to be read, updated, and
  // then set to its new value, all in an atomic section.
  T& GetLock() {
    const int lock_error = pthread_mutex_lock(&mutex_);
    DCHECK(lock_error == 0);
    locked_ = true;
    return (value_);
  }

  // Set the value of the underlying type and then unlock the protecting
  // mutex. This is useful for the "Write" step of "Read-Modify-Write" cases
  // where the value neads to be read, updated, and then set to its new value,
  // all in an atomic section.
  void SetUnlock(const T& value) {
    DCHECK(locked_);
    value_ = value;
    const int unlock_error = pthread_mutex_unlock(&mutex_);
    DCHECK(unlock_error == 0);
    locked_= false;
  }

  // Unlock the mutex in the event of an aborted "Read-Modify-Write" procedure
  // after it was previously locked by @GetLock().
  void Unlock() const {
    DCHECK(locked_);
    const int unlock_error = pthread_mutex_unlock(&mutex_);
    DCHECK(unlock_error == 0);
    locked_= false;
  }

 protected:
  T value_;
  mutable pthread_mutex_t mutex_;
  mutable bool locked_;
};

#endif  // PTHREAD_UTILS_H