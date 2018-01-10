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
/*!
\file    extensions.h
\brief   C++ Extensions
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <string>
#include "geometry.h"

///Returns a C - string denoting the type of the variable
///Returns a CustomExtensions::TypeIDs value denoting the type of the variable
#define TYPE_CHECK(Type) \
  const char* typeCheck(const Type p){ \
    return #Type; \
  } \
  int typeCheckID(const Type p){ \
    return CustomExtensions::t##Type; \
  }

namespace CustomExtensions{
  typedef enum {
    tuchar,
    tuint,
    tulong,
    tulonglong,
    tchar,
    tint,
    tlong,
    tlonglong,
    tfloat,
    tdouble,
    tvector2i,
    tvector2f,
    tvector2d,
    tvector3i,
    tvector3f,
    tvector3d,
    tline2i,
    tline2f,
    tline2d,
    tstring,
  }TypeIDs;

  typedef unsigned char uchar;
  typedef unsigned int uint;
  typedef unsigned long ulong;
  typedef long long longlong;
  typedef unsigned long long ulonglong;

  TYPE_CHECK(uchar)
  TYPE_CHECK(uint)
  TYPE_CHECK(ulong)
  TYPE_CHECK(ulonglong)
  TYPE_CHECK(char)
  TYPE_CHECK(int)
  TYPE_CHECK(long)
  TYPE_CHECK(longlong)
  TYPE_CHECK(double)
  TYPE_CHECK(float)
  TYPE_CHECK(string)
  TYPE_CHECK(vector2d)
  TYPE_CHECK(vector2f)
  TYPE_CHECK(vector2i)
  TYPE_CHECK(vector3d)
  TYPE_CHECK(vector3f)
  TYPE_CHECK(vector3i)
  TYPE_CHECK(line2d)
  TYPE_CHECK(line2f)
  TYPE_CHECK(line2i)
}




