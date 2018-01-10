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
\file    gvector.cpp
\brief   Utility functions for manipulation of the vector classes
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include "gvector.h"

namespace GVector {

#ifdef GVECTOR_SSE_OPTIMIZATIONS
template <> vector3d<float> vector3d<float>::transform(const matrix3d<float> &M) const
{
  float _x, _y, _z;
  __m128 v1, v2, rx, ry, rz;
  
  v1 = _mm_load_ps(&coefficients[0]);
  v2 = _mm_load_ps(&M.elements[0]);
  rx = _mm_dp_ps(v1,v2,0xF1);
  
  
  v1 = _mm_load_ps(&coefficients[0]);
  v2 = _mm_load_ps(&M.elements[4]);
  ry = _mm_dp_ps(v1,v2,0xF1);
  
  v1 = _mm_load_ps(&coefficients[0]);
  v2 = _mm_load_ps(&M.elements[8]);
  rz = _mm_dp_ps(v1,v2,0xF1);
  
  _mm_store_ss(&_x,rx);
  _mm_store_ss(&_y,ry);
  _mm_store_ss(&_z,rz);
  
  vector3d<float> q;
  q.set(_x,_y,_z);
  return q;
}

template <> float vector3d<float>::sqlength() const
{
  return sse_dot_product<float>(coefficients,coefficients,0x71);
}

template <> float vector3d<float>::dot(const vector3d<float> p) const
{
  return sse_dot_product<float>(coefficients,p.coefficients,0x71);
}

#endif

};
