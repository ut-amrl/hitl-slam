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
\file    triangle.h
\brief   C++ Interfaces: Triangle3d
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <float.h>
#include "geometry.h"

#ifndef TRIANGLE_H
#define TRIANGLE_H

#define TRIANGLE_TEMP template <class num> inline
#define TRIANGLE_FUNC Triangle3d<num>

template <class num> class Triangle3d;

template <class num>
class Triangle3d{
private:
  GVector::vector3d<num> p0,p1,p2;
  GVector::vector3d<num> normal;
  GVector::vector3d<num> centroid;
  num offset;
  
public:
  //Constructors, Setters
  Triangle3d();
  Triangle3d(GVector::vector3d<num> _p0, GVector::vector3d<num> _p1, GVector::vector3d<num> _p2);
  void set(GVector::vector3d<num> _p0, GVector::vector3d<num> _p1, GVector::vector3d<num> _p2);
  /// Calculate derived values (normal, centroid)
  void calcValues();
  
  //Common Math Functions related to 3D triangles
  /// Closest point on this triangle plane to p
  GVector::vector3d<num> closestPoint(GVector::vector3d<num> p);
  /// Closest distance from this triangle plane to p
  num closestDist(GVector::vector3d<num> p);
  
  /// Returns true if p lies alongside this triangle
  bool liesAlongside(GVector::vector3d<num> p);
  /// Return true if the triangle t intersect this triangle
  bool intersects(Triangle3d<num> t);
  
  //Getters
  inline const GVector::vector3d<num>& P0() const {return p0;}
  inline const GVector::vector3d<num>& P1() const {return p1;}
  inline const GVector::vector3d<num>& P2() const {return p2;}
  inline const GVector::vector3d<num>& Normal() const{return normal;}
  inline const GVector::vector3d<num>& Centroid() const{return centroid;}
  inline const num& Offset() const{return offset;}
  inline void ToString(char* str) const;
  
  //Operator overloads
  /// returns this triangle scaled by f
  Triangle3d<num> operator*(num f) const;
  /// returns this triangle scaled by 1/f
  Triangle3d<num> operator/(num f) const;
  /// scales this triangle by f
  Triangle3d<num> &operator*=(num f);
  /// scales this triangle by 1/f
  Triangle3d<num> &operator/=(num f);
  /// returns this triangle translated by vector v
  Triangle3d<num> operator+(GVector::vector3d<num> v) const;
  Triangle3d<num> operator-(GVector::vector3d<num> v) const;
  /// translates this triangle by vector v
  Triangle3d<num> &operator+=(GVector::vector3d<num> v);
  Triangle3d<num> &operator-=(GVector::vector3d<num> v);
};

typedef Triangle3d<double> triangle3d;
typedef Triangle3d<float> triangle3f;

TRIANGLE_TEMP
TRIANGLE_FUNC::Triangle3d()
{
  p0.zero();
  p1.zero();
  p2.zero();
  calcValues();
  //updateRequired = true;
}

TRIANGLE_TEMP
TRIANGLE_FUNC::Triangle3d(GVector::vector3d<num> _p0, GVector::vector3d<num> _p1, GVector::vector3d<num> _p2)
{
  p0 = _p0;
  p1 = _p1;
  p2 = _p2;
  calcValues();
  //updateRequired = true;
}

TRIANGLE_TEMP
void TRIANGLE_FUNC::ToString(char* str) const
{
  sprintf(str, "%.3f,%.3f,%.3f  %.3f,%.3f,%.3f  %.3f,%.3f,%.3f",V3COMP(p0),V3COMP(p1),V3COMP(p2));
}

TRIANGLE_TEMP
void TRIANGLE_FUNC::set(GVector::vector3d<num> _p0, GVector::vector3d<num> _p1, GVector::vector3d<num> _p2)
{
  Triangle3d(_p0, _p1, _p2);
}


TRIANGLE_TEMP
void TRIANGLE_FUNC::calcValues()
{
  centroid = (p0+p1+p2)/3.0;
  normal = (p2-p1).cross(p0-p1).norm();
  offset = -centroid.dot(normal);
  //updateRequired = false;
}

TRIANGLE_TEMP
num TRIANGLE_FUNC::closestDist(GVector::vector3d< num > p)
{
  return ( p.dot(normal) + offset );
}

TRIANGLE_TEMP
bool TRIANGLE_FUNC::intersects(Triangle3d< num > t)
{
  printf("Unimplemented function %s\n",__PRETTY_FUNCTION__);
  return false;
}

TRIANGLE_TEMP
bool TRIANGLE_FUNC::liesAlongside(GVector::vector3d< num > p)
{
  printf("Unimplemented function %s\n",__PRETTY_FUNCTION__);
  return false;
}

TRIANGLE_TEMP
GVector::vector3d< num > TRIANGLE_FUNC::closestPoint(GVector::vector3d< num > p)
{
  return ( p - normal*(normal.dot(p)+offset) );
}

#define TRIANGLE_UNARY_OPERATOR_SCALAR(op) \
  TRIANGLE_TEMP \
  Triangle3d<num>& TRIANGLE_FUNC::operator op (num f) \
  {                   \
    p0 = p0 op f;   \
    p1 = p1 op f;   \
    p2 = p2 op f;   \
    calcValues();     \
    return(*this);    \
  }

TRIANGLE_UNARY_OPERATOR_SCALAR(*=)
TRIANGLE_UNARY_OPERATOR_SCALAR(/=)

#define TRIANGLE_BINARY_OPERATOR_SCALAR(op) \
  TRIANGLE_TEMP \
  Triangle3d<num> TRIANGLE_FUNC::operator op (num f) const\
  {                   \
    GVector::vector3d<num> p0_ = p0 op f;   \
    GVector::vector3d<num> p1_ = p1 op f;   \
    GVector::vector3d<num> p2_ = p2 op f;   \
    return(Triangle3d<num>(p0_,p1_,p2_));    \
  }

TRIANGLE_BINARY_OPERATOR_SCALAR(*)
TRIANGLE_BINARY_OPERATOR_SCALAR(/)

#define TRIANGLE_BINARY_OPERATOR_VECTOR(op) \
  TRIANGLE_TEMP \
  Triangle3d<num> TRIANGLE_FUNC::operator op (GVector::vector3d<num> v) const\
  {                   \
    GVector::vector3d<num> p0_ = p0 op v;   \
    GVector::vector3d<num> p1_ = p1 op v;   \
    GVector::vector3d<num> p2_ = p2 op v;   \
    return(Triangle3d<num>(p0_,p1_,p2_));    \
  }

TRIANGLE_BINARY_OPERATOR_VECTOR(+)
TRIANGLE_BINARY_OPERATOR_VECTOR(-)
  
#define TRIANGLE_UNARY_OPERATOR_VECTOR(op) \
  TRIANGLE_TEMP \
  Triangle3d<num>& TRIANGLE_FUNC::operator op (GVector::vector3d<num> v) \
  {                   \
    p0 = p0 op v;   \
    p1 = p1 op v;   \
    p2 = p2 op v;   \
    calcValues();     \
    return(*this);    \
  }

TRIANGLE_UNARY_OPERATOR_VECTOR(+=)
TRIANGLE_UNARY_OPERATOR_VECTOR(-=)

#endif //TRIANGLE_H