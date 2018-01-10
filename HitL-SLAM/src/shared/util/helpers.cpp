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

#include "helpers.h"
#include <execinfo.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdarg>

std::string StringPrintf(const char* format, ...) {
  va_list al;
  int string_length = 0;
  char* buffer = NULL;

  va_start(al,format);
  string_length = vsnprintf(buffer,string_length,format,al);
  va_end(al);
  if (string_length == 0) return (std::string());
  buffer = reinterpret_cast<char*>(malloc((string_length + 1) * sizeof(char)));
  if (buffer == NULL) return (std::string());

  va_start(al,format);
  string_length = vsnprintf(buffer,string_length + 1,format,al);
  va_end(al);
  return (std::string(buffer));
}

std::string ExecuteCommand(const char* cmd) {
  FILE* pipe = popen(cmd, "r");
  if (!pipe) return "ERROR";
  char buffer[128];
  std::string result = "";
  while(!feof(pipe)) {
    if(fgets(buffer, sizeof(buffer), pipe) != NULL)
      result += buffer;
  }
  pclose(pipe);
  return result;
}

void PrintBackTrace(FILE* file) {
  void *trace[16];
  char **messages = (char **)NULL;
  int i, trace_size = 0;

  trace_size = backtrace(trace, 16);
  messages = backtrace_symbols(trace, trace_size);
  fprintf(file, "Stack Trace:\n");
  for (i=1; i<trace_size; ++i) {
    fprintf(file, "#%d %s\n", i - 1, messages[i]);
    std::string addr2line_command =
        StringPrintf("addr2line %p -e /proc/%d/exe", trace[i], getpid());
    fprintf(file, "%s", ExecuteCommand(addr2line_command.c_str()).c_str());
  }
}

bool FileExists(const std::string& file_name) {
  struct stat st;
  return(stat(file_name.c_str(), &st) == 0);
}
