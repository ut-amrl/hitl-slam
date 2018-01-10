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
// Copyright 2012 joydeepb@ri.cmu.edu
// Robotics Institute, Carnegie Mellon University
//
// Miscellaneous helper functions.

#include <assert.h>
#include <glog/logging.h>
#include <stdio.h>
#include <cstdarg>
#include <string>

#ifndef HELPERS_H
#define HELPERS_H

// Useful for debug printing.
#define DEBUG_PRINT_LINE \
    printf("%s:%d\n", __FILE__, __LINE__); \
    fflush(stdout);

// Return an std::string created using a printf-like syntax.
std::string StringPrintf(const char* format, ...)
    __attribute__((format(__printf__,1,2)));

// Returns an std::string of the result of executing a command @cmd.
std::string ExecuteCommand(const char* cmd);

// Print the execution backtrace of the program to the specified @file, which is
// stdout by default. For enhanced readability, the compiler should be invoked
// with the "-g -rdynamic" flags.
void PrintBackTrace(FILE* file = stdout);

// Returns truee iff the file specified exists in the file system.
bool FileExists(const std::string& file_name);

// A simple class wrapper for a file handle that automatically closes the file
// when the instance of the class is destroyed.
class ScopedFile {
  static const bool kDebug = false;
 public:
  // Constructor that inherits ownership of a previously opened file.
  explicit ScopedFile(FILE* fid) : fid_(fid) {}

  // Constructor that opens the specified file in the specified mode.
  ScopedFile(const std::string& file_name,
             const char* mode,
             bool print_error = false) :
      fid_(NULL) {
    Open(file_name, mode, print_error);
  }

  // Destructor.
  ~ScopedFile() {
    if (fid_ == NULL) return;
    const bool error = (fclose(fid_) != 0);
    if (kDebug) {
      printf("fclose success:%d\n", error);
      if (error) perror("Error closing file descriptor");
    }
  }

  // Open a file explicitly.
  void Open(const std::string& file_name,
             const char* mode,
             bool print_error = false) {
    if (fid_) fclose(fid_);
    fid_ = fopen(file_name.c_str(), mode);
    if (fid_ == NULL) {
      if (print_error) {
        const std::string error_string = "Error opening \"" + file_name + "\"";
        perror(error_string.c_str());
      }
    } else if (kDebug){
      printf("fopen: 0x%08X\n", fileno(fid_));
    }
  }

  // Getter for the underlying file handle.
  FILE* operator()() { return (fid_); }

  // Conversion operator to convert to FILE* type.
  operator FILE*&() { return (fid_); }

 private:
  // Disable the default constructor.
  ScopedFile();

  // Disable the copy constructor.
  ScopedFile(const ScopedFile& other);

  // Disable the assignment operator.
  const ScopedFile& operator=(const ScopedFile& other);

private:
  // The file handle owned by this instance.
  FILE* fid_;
};

#endif  // HELPERS_H
