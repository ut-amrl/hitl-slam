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
\file    line.h
\brief   C++ Interfaces: Line2d
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <float.h>
#include "geometry.h"

#ifndef LINE_H
#define LINE_H

#define LINE_TEM template <class num> inline
#define LINE_FUN Line2d<num>

#define LazyCaching
//using namespace GVector;
template <class num> class Line2d;

template <class num>
class Line2d{
private:
  /// End points of the line
  GVector::vector2d<num> p0,p1;
  /// Unit vector from p0 to p1 of the line
  mutable GVector::vector2d<num> dir;
  /// Direction of the line, rotated 90 degrees anticlockwise
  mutable GVector::vector2d<num> perp;
  /// Length of the line
  mutable num length;
  /// Used to keep track of lazy caching
  mutable bool updateRequired;
  /// Epsilon for computations
  static const num eps;

public:
  //Constructors, Setters
  Line2d();
  Line2d(const GVector::vector2d<num>& _p1);
  Line2d(const GVector::vector2d<num>& _p0, const GVector::vector2d<num>& _p1);
  Line2d(num p0x, num p0y, num p1x, num p1y);
  void set(const GVector::vector2d<num>& _p0,
           const GVector::vector2d<num>& _p1);
  void set(num p0x, num p0y, num p1x, num p1y);
  /// Calculate derived values (dir, perp, angle)
  void calcValues() const;

  //Common Math Functions related to lines
  /// Calculate closest distance of p from this line.
  num closestDistFromLine(const GVector::vector2d<num>& p, bool endcaps) const;

  /// Calculate distance of p along the direction rayDir from this line.
  num distFromLine(const GVector::vector2d< num >& p,
                   const GVector::vector2d< num >& rayDir, bool extendLine);

  /// Calculate distance of p along the direction angle rayDir from this line.
  num distFromLine1(const GVector::vector2d<num>& p,
                    num rayDir, bool extendLine);

  /// Returns a vector perpendicular to the line, starting from the line and
  /// terminating at p
  GVector::vector2d<num> perpFromLine(const GVector::vector2d<num>& p,
                                      bool endcaps);

  /// Returns true if p lies along side this line
  bool liesAlongside(const GVector::vector2d<num>& p, num margin = 0.0);

  /// Return true if the line l1 intersects this line.
  /// If touch==true then touching counts as intersection.
  bool intersects(const Line2d<num>& l1,
                  bool extendL0, bool extendL1, bool touch) const;

  /// Return true if the line joining p2 and p3 intersects this line.
  /// If touch==true then touching counts as intersection.
  bool intersects(const GVector::vector2d< num >& p2,
                  const GVector::vector2d< num >& p3,
                  bool extendL0, bool extendL1, bool touch) const;

  /// Return true if the ray v from origin intersects this line
  bool intersects(const GVector::vector2d<num>& v,
                  bool extendL0, bool extendL1) const {
    return intersects(Line2d<num>(v), extendL0, extendL1);
  }

  /// Returns true if the ray from p in the direction rayDir intersects the line
  bool intersects(const GVector::vector2d<num>& p,
                  const GVector::vector2d<num>& rayDir, bool touching) const;

  /// Returns true if the ray from p in the direction angle rayDir intersects the line
  bool intersects(const GVector::vector2d<num>& p, num rayDir) const;

  /// Checks if the current line intersects the line joining p2 and p3 and if
  /// so, returns the point of intersection.
  GVector::vector2d<num> intersectTest(const GVector::vector2d< num >& p2,
                                       const GVector::vector2d< num >& p3,
                                       bool &intersects, bool extendL0,
                                       bool extendL1);

  /// Return the point of intersection of two lines
  GVector::vector2d<num> intersection(const Line2d<num> &l1,
                                      bool extendL0, bool extendL1) const;

  /// Return the point of intersection of this line with the line joining p2 and p3
  GVector::vector2d<num> intersection(const GVector::vector2d< num >& p2,
                                      const GVector::vector2d< num >& p3,
                                      bool extendL0, bool extendL1) const;

  /// Rotates the line about point p by given angle
  Line2d<num> rotate(const GVector::vector2d<num>& p, num angle);

  /// Projects point p into the coordinate frame attached to the line segment
  /// where the x axis is in the direction of p0 to p1 of the line.
  GVector::vector2d<num> project_in(const GVector::vector2d<num>& p);

  /// Projects point p from the coordinate frame attached to the line segment
  /// to global coordinates.
  GVector::vector2d<num> project_out(const GVector::vector2d<num>& p);

  ///Returns the projection of p onto the line.
  GVector::vector2d<num> point_projection(
      const GVector::vector2d<num>& p) const;

  /// Returns a string formated representation of the line.
  char* ToString(char* str);

  //Getters
  inline const GVector::vector2d<num>& P0() const {return p0;}
  inline const GVector::vector2d<num>& P1() const {return p1;}

#ifdef LazyCaching
  inline const GVector::vector2d<num>& Dir() const {
    if(updateRequired) calcValues(); return dir;
  }

  inline const GVector::vector2d<num>& Perp() const {
    if(updateRequired) calcValues(); return perp;
  }

  inline const num Length() const {
    if(updateRequired) calcValues(); return length;
  }

  inline const num Angle() const {
    if(updateRequired) calcValues(); return dir.angle();
  }

#else

  inline GVector::vector2d<num> Dir() const {return (p1-p0).norm();}
  inline GVector::vector2d<num> Perp() const {return (p1-p0).perp().norm();}
  inline num Length() const {return (p1-p0).length();}
  inline num Angle() const {return (p1-p0).angle();}

#endif

  //Operator overloads
  /// returns this line scaled by f
  Line2d<num> operator*(num f) const;
  /// returns this line scaled by 1/f
  Line2d<num> operator/(num f) const;
  /// scales this line by f
  Line2d<num> &operator*=(num f);
  /// scales this line by 1/f
  Line2d<num> &operator/=(num f);
  /// returns this line translated by vector v
  Line2d<num> operator+(const GVector::vector2d<num>& v) const;
  Line2d<num> operator-(const GVector::vector2d<num>& v) const;
  /// translates this line by vector v
  Line2d<num> &operator+=(const GVector::vector2d<num>& v);
  Line2d<num> &operator-=(const GVector::vector2d<num>& v);
};

typedef Line2d<double> line2d;
typedef Line2d<float> line2f;
typedef Line2d<int> line2i;

template <class num> const num Line2d<num>::eps = num(1e-5);

LINE_TEM
LINE_FUN::Line2d()
{
  p0.zero();
  p1.zero();
#ifdef LazyCaching
  //calcValues();
  updateRequired = true;
#endif
}

LINE_TEM
LINE_FUN::Line2d(const GVector::vector2d< num >& _p1) : p1(_p1)
{
  p0.zero();
#ifdef LazyCaching
  //calcValues();
  updateRequired = true;
#endif
}

LINE_TEM
LINE_FUN::Line2d(const GVector::vector2d< num >& _p0,
                 const GVector::vector2d< num >& _p1) :
    p0(_p0), p1(_p1)
{
#ifdef LazyCaching
  //calcValues();
  updateRequired = true;
#endif
}

LINE_TEM
LINE_FUN::Line2d(num p0x, num p0y, num p1x, num p1y) :
    p0(p0x, p0y), p1(p1x, p1y)
{
#ifdef LazyCaching
  //calcValues();
  updateRequired = true;
#endif
}

LINE_TEM
void LINE_FUN::set(const GVector::vector2d<num>& _p0,
                   const GVector::vector2d<num>& _p1)
{
  p0 = _p0;
  p1 = _p1;
#ifdef LazyCaching
  //calcValues();
  updateRequired = true;
#endif
}

LINE_TEM
void LINE_FUN::set(num p0x, num p0y, num p1x, num p1y)
{
  p0.set(p0x, p0y);
  p1.set(p1x, p1y);
#ifdef LazyCaching
  //calcValues();
  updateRequired = true;
#endif
}

LINE_TEM
void LINE_FUN::calcValues() const
{
  length = (p1-p0).length();
  dir = (p1-p0)/length;
  perp = dir.perp();
  updateRequired = false;
}

LINE_TEM
num LINE_FUN::closestDistFromLine(const GVector::vector2d< num >& p,
                                  bool endcaps) const
{
#ifdef LazyCaching
  if(updateRequired) calcValues();
#else
  calcValues();
#endif
  num dist = fabs(perp.dot(p-p0));
  if (endcaps){
    num locationOnLine = dir.dot(p-p0);
    if(locationOnLine<0.0){
      return (p-p0).length();
    }else if(locationOnLine>length){
      return (p-p1).length();
    }
  }
  return dist;
}

LINE_TEM
num LINE_FUN::distFromLine(const GVector::vector2d< num >& p,
                           const GVector::vector2d< num >& rayDir,
                           bool extendLine)
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  num result = 0.0;
  if(intersects(p,rayDir) || extendLine){
    num sinTheta = dir.cross(rayDir.norm());
    result = fabs(perp.dot(p-p0)/sinTheta);
  }else{
    result = NAN;
  }
  if(result<0.0){
    printf("Fail2! %d %f %f %f\n",(intersects(p,rayDir) || extendLine)?1:0,dir.cross(rayDir.norm()), perp.dot(p-p0), result);
    fflush(stdout);
  }
  return result;
}

LINE_TEM
num LINE_FUN::distFromLine1(const GVector::vector2d< num >& p,
                            num rayDir, bool extendLine)
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  calcValues();
  GVector::vector2d<num> heading;
  heading.heading(rayDir);
  num result = 0.0;
  if(intersects(p,heading,false) || extendLine){
    num sinTheta = dir.cross(heading.norm());
    result = fabs(perp.dot(p-p0)/sinTheta);
  }else{
    result = NAN;
  }
  if(result<0.0){
    printf("Fail1! %d %f %f %f\n",(intersects(p,heading,false) || extendLine)?1:0,dir.cross(heading.norm()), perp.dot(p-p0), result);
    fflush(stdout);
  }
  return result;
}

LINE_TEM
GVector::vector2d<num> LINE_FUN::perpFromLine(const GVector::vector2d< num >& p,
                                              bool endcaps)
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  num d = dir.dot(p-p0);
  if((d>=0.0 && d<=length) || !endcaps){
    // Lies (alongside line), or (beyond line but using extension of line)
    return ( (p-p0) - dir*d );
  }else{
    // Beyond line and using endcaps
    if(d<0.0)
      return (p-p0);
    else
      return (p-p1);
  }
}

LINE_TEM
bool LINE_FUN::liesAlongside(const GVector::vector2d< num >& p,
                             num margin)
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  num location = (p-p0).dot(dir);
  return (location>-margin && location<length+margin);
}

LINE_TEM
bool LINE_FUN::intersects(const GVector::vector2d< num >& p2,
                          const GVector::vector2d< num >& p3,
                          bool extendL0, bool extendL1, bool touch) const
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  //GVector::vector2d<num> perp1 = perp;//(p1-p0).perp();
  //Check to see if lines intersect within their extents, or if it is okay to extend lines
  if(touch)
    return ( (extendL1 || (perp.dot(p2-p0)*perp.dot(p3-p0)<=eps)) && (extendL0 || ((p3-p2).perpdot(p0-p2)*(p3-p2).perpdot(p1-p2)<=eps)) );
  else
    return ( (extendL1 || (perp.dot(p2-p0)*perp.dot(p3-p0)<0.0)) && (extendL0 || ((p3-p2).perpdot(p0-p2)*(p3-p2).perpdot(p1-p2)<0.0)) );
}

LINE_TEM
GVector::vector2d<num> LINE_FUN::intersectTest(
    const GVector::vector2d< num >& p2,
    const GVector::vector2d< num >& p3,
    bool& intersects, bool extendL0, bool extendL1)
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  GVector::vector2d<num> dir2 = (p3-p2).norm();

  num den = dir.y*dir2.x - dir.x*dir2.y;
  num ua = (dir.y*(p0.x-p2.x) - dir.x*(p0.y-p2.y))/den;

  GVector::vector2d<num> p = p2 + ua*dir2;
  num position0 = dir.dot(p-p0);
  num position1 = dir2.dot(p-p2);

  intersects = true;
  if ((!extendL0 && (position0<0.0 || position0>length)) ||
      (!extendL1 && (sq(position1)>(p3-p2).sqlength()))) {
    intersects = false;
  }
  return p;
}

LINE_TEM
bool LINE_FUN::intersects(const Line2d< num >& l2,
                          bool extendL0, bool extendL1, bool touch) const
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  //GVector::vector2d<num> perp1 = perp;//(p1-p0).perp();
  //Check to see if lines intersect within their extents, or if it is okay to extend lines
  if(touch)
    return ( (extendL1 || (perp.dot(l2.P0()-p0)*perp.dot(l2.P1()-p0)<=eps)) && (extendL0 || (l2.Perp().dot(p0-l2.P0())*l2.Perp().dot(p1-l2.P0())<=eps)) );
  else
    return ( (extendL1 || (perp.dot(l2.P0()-p0)*perp.dot(l2.P1()-p0)<-eps)) && (extendL0 || (l2.Perp().dot(p0-l2.P0())*l2.Perp().dot(p1-l2.P0())<-eps)) );
}

LINE_TEM
bool LINE_FUN::intersects(
    const GVector::vector2d< num >& p,
    const GVector::vector2d< num >& rayDir, bool touching) const
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  GVector::vector2d<num> v0 = p0-p;
  GVector::vector2d<num> v1 = p1-p;
  if(v0.cross(v1)<0.0)
    swap(v0,v1);

  if(touching) {
    // Check if rayDir lies within the angle reqion between v0 and v1
    return ( v0.cross(rayDir)>=0.0 && v1.cross(rayDir) <= 0.0 );
  } else {
    // Check if rayDir lies within the angle reqion between v0 and v1
    return ( v0.cross(rayDir)>eps && v1.cross(rayDir) < -eps );
  }
}

LINE_TEM
bool LINE_FUN::intersects(const GVector::vector2d< num >& p,
                          num rayDir) const
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  GVector::vector2d<num> heading;
  heading.heading(rayDir);

  return intersects(p, heading,false);
}

LINE_TEM
GVector::vector2d<num> LINE_FUN::intersection(
    const GVector::vector2d<num>& p2, const GVector::vector2d<num>& p3,
    bool extendL0, bool extendL1) const
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  GVector::vector2d<num> dir2 = (p3-p2).norm();

  num den = dir.y*dir2.x - dir.x*dir2.y;
  num ua = (dir.y*(p0.x-p2.x) - dir.x*(p0.y-p2.y))/den;

  GVector::vector2d<num> p = p2 + ua*dir2;
  num position0 = dir.dot(p-p0);
  num position1 = dir2.dot(p-p2);

  if ((!extendL0 && (position0<0.0 || position0>length)) ||
      (!extendL1 && (sq(position1)>(p3-p2).sqlength()))) {
    return GVector::vector2d<num>(NAN,NAN);
  }
  return p;
}

LINE_TEM
GVector::vector2d<num> LINE_FUN::intersection(
    const Line2d< num >& l1, bool extendL0, bool extendL1) const
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  GVector::vector2d<num> p02 =l1.P0();
  GVector::vector2d<num> dir2 = l1.Dir();

  num den = dir.y*dir2.x - dir.x*dir2.y;
  num ua = (dir.y*(p0.x-p02.x) - dir.x*(p0.y-p02.y))/den;

  GVector::vector2d<num> p = p02 + ua*dir2;
  num position0 = dir.dot(p-p0);
  num position1 = dir2.dot(p-p02);

  if ((!extendL0 && (position0<0.0 || position0>length)) ||
      (!extendL1 && (position1<0.0 || position1>l1.Length()))) {
    return GVector::vector2d<num>(NAN,NAN);
  }
  return p;
}

LINE_TEM
Line2d<num> LINE_FUN::rotate(const GVector::vector2d<num>& p, num angle)
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  GVector::vector2d<num> _p0 = p + (p0-p).rotate(angle);
  GVector::vector2d<num> _p1 = p + (p1-p).rotate(angle);
  return Line2d<num>(_p0,_p1);
}

LINE_TEM
GVector::vector2d<num> LINE_FUN::project_in(const GVector::vector2d<num>& p)
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  return GVector::vector2d<num>(dir.dot(p-p0),perp.dot(p-p0));
}

LINE_TEM
GVector::vector2d<num> LINE_FUN::project_out(const GVector::vector2d<num>& p)
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  return (dir*p.x + perp*p.y);
}

LINE_TEM
GVector::vector2d<num> LINE_FUN::point_projection(
    const GVector::vector2d<num>& p) const
{
  #ifdef LazyCaching
  if(updateRequired) calcValues();
  #else
  calcValues();
  #endif
  return p - perp * (perp.dot(p - p0));
}

LINE_TEM
char* LINE_FUN::ToString(char* str)
{
  snprintf(str,64, "(%8.3f,%8.3f) - (%8.3f,%8.3f)",V2COMP(p0),V2COMP(p1));
  return str;
}

#define LINE_UNARY_OPERATOR_SCALAR(op) \
  LINE_TEM \
  Line2d<num>& LINE_FUN::operator op (num f) \
  {                   \
    p0 = p0 op f;   \
    p1 = p1 op f;   \
    updateRequired = true;     \
    return(*this);    \
  }

LINE_UNARY_OPERATOR_SCALAR(*=)
LINE_UNARY_OPERATOR_SCALAR(/=)

#define LINE_BINARY_OPERATOR_SCALAR(op) \
  LINE_TEM \
  Line2d<num> LINE_FUN::operator op (num f) const\
  {                   \
    GVector::vector2d<num> p0_ = p0 op f;   \
    GVector::vector2d<num> p1_ = p1 op f;   \
    return(Line2d<num>(p0_,p1_));    \
  }

LINE_BINARY_OPERATOR_SCALAR(*)
LINE_BINARY_OPERATOR_SCALAR(/)

#define LINE_BINARY_OPERATOR_VECTOR(op) \
  LINE_TEM \
  Line2d<num> LINE_FUN::operator op (const GVector::vector2d<num>& v) const\
  {                   \
    GVector::vector2d<num> p0_ = p0 op v;   \
    GVector::vector2d<num> p1_ = p1 op v;   \
    return(Line2d<num>(p0_,p1_));    \
  }

LINE_BINARY_OPERATOR_VECTOR(+)
LINE_BINARY_OPERATOR_VECTOR(-)

#define LINE_UNARY_OPERATOR_VECTOR(op) \
  LINE_TEM \
  Line2d<num>& LINE_FUN::operator op (const GVector::vector2d<num>& v) \
  {                   \
    p0 = p0 op v;   \
    p1 = p1 op v;   \
    updateRequired = true;     \
    return(*this);    \
  }

LINE_UNARY_OPERATOR_VECTOR(+=)
LINE_UNARY_OPERATOR_VECTOR(-=)

#endif //LINE_H