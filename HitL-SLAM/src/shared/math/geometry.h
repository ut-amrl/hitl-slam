#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#define USE_QUATERIONS        1
#define USE_MATRIX_FIXED_SIZE 0
#define USE_POSE              1

#include <stdio.h>
#include <stdint.h>

//==== Vector types ====//

#include "gvector.h"
#include "line.h"
#include "geomalgo.h"

typedef GVector::vector2d<double> vector2d;
typedef GVector::vector3d<double> vector3d;

typedef GVector::vector2d<float> vector2f;
typedef GVector::vector3d<float> vector3f;

typedef GVector::vector2d<int> vector2i;
typedef GVector::vector3d<int> vector3i;

struct vector2s{
  int16_t x,y;
};

struct vector3s{
  int16_t x,y,z;
};

//==== Some vector conversion functions ====//

template <class vector_a,class vector_b>
void vcopy2d(vector_a &dest,const vector_b &src)
{
  dest.x = src.x;
  dest.y = src.y;
}

template <class vector_a,class vector_b>
void vcopy3d(vector_a &dest,const vector_b &src)
{
  dest.x = src.x;
  dest.y = src.y;
  dest.z = src.z;
}

template <class vec_in>
inline vector2f vec2f(const vec_in &p)
{
  return(vector2f(p.x,p.y));
}

template <class vec_in>
inline vector2d vec2d(const vec_in &p)
{
  return(vector2d(p.x,p.y));
}

template <class vec_in>
inline vector3f vec3f(const vec_in &p)
{
  return(vector3f(p.x,p.y,p.z));
}

template <class vec_in>
inline vector3d vec3d(const vec_in &p)
{
  return(vector3d(p.x,p.y,p.z));
}

template <class vec_in>
inline vector2s vec2s(const vec_in &p)
{
  vector2s vs;
  vs.x = (int)rint(p.x);
  vs.y = (int)rint(p.y);
  return(vs);
}

template <class vec_in>
inline vector3s vec3s(const vec_in &p)
{
  vector3s vs;
  vs.x = (int)rint(p.x);
  vs.y = (int)rint(p.y);
  vs.z = (int)rint(p.z);
  return(vs);
}

//==== Angle conversion, and normally missing constant(s) ====//

#define RAD(deg) ((deg) * (M_PI / 180.0)) /* convert radians to degrees */
#define DEG(rad) ((rad) * (180.0 / M_PI)) /* convert degrees to radians */

#ifndef HUGE_VALF
#define HUGE_VALF (1E37)
#endif

#endif // __GEOMETRY_H__
