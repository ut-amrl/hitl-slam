/*========================================================================
    GVector.h : Simple vector class for 2D and 3D vectors
  ------------------------------------------------------------------------
    Copyright (C) 1999-2006  James R. Bruce
    Modified by Joydeep Biswas, 2009-2011
    School of Computer Science, Carnegie Mellon University
  ------------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ========================================================================*/

#ifndef __GVECTOR_H__
#define __GVECTOR_H__

#include <cmath>
#include "util.h"
#include <xmmintrin.h>
#include <smmintrin.h>

#define V3COMP(p) (p).x,(p).y,(p).z
#define V2COMP(p) (p).x,(p).y
#define pBool(var) (var)? ("true"):("false")

#define GVECTOR_SSE_OPTIMIZATIONS

#define VECTOR_TEM \
template <class num>

namespace GVector {

//=====================================================================//
//  Vector3D Class
//=====================================================================//

#define EPSILON (1.0E-10)

#ifdef GVECTOR_SSE_OPTIMIZATIONS
template <typename num>
inline num sse_sqrt(num xin)
{
  __m128d x = _mm_set_sd(double(xin));
  __m128d y = _mm_set_sd(double(xin));
  x = _mm_sqrt_sd(x,y);
  double retVal;
  _mm_store_sd(&retVal,x);
  return(retVal);
}

template <> inline float sse_sqrt<float>(float xin)
{
  __m128 x = _mm_set_ss((float) xin);
  x = _mm_sqrt_ss(x);
  float retVal;
  _mm_store_ss(&retVal,x);
  return(retVal);
}

template <> inline double sse_sqrt<double>(double xin)
{
  __m128d x = _mm_set_sd(xin);
  __m128d y = _mm_set_sd(xin);
  x = _mm_sqrt_sd(x,y);
  double retVal;
  _mm_store_sd(&retVal,x);
  return(retVal);
}

template <typename num>
inline num sse_dot_product(const num* fv1, const num* fv2, const int mask)
{
  __m128 v1 = _mm_set_ps(float(fv1[3]),float(fv1[2]),float(fv1[1]),float(fv1[0]));
  __m128 v2 = _mm_set_ps(float(fv2[3]),float(fv2[2]),float(fv2[1]),float(fv2[0]));
  __m128 dp = _mm_dp_ps(v1,v2,mask);
  float res;
  _mm_store_ss(&res,dp);
  return res;
}

template <>
inline float sse_dot_product(const float* fv1, const float* fv2, const int mask)
{
  __m128 v1 = _mm_load_ps(fv1);
  __m128 v2 = _mm_load_ps(fv2);
  __m128 dp = _mm_dp_ps(v1,v2,mask);
  float res;
  _mm_store_ss(&res,dp);
  return res;
}

template <>
inline double sse_dot_product(const double* fv1, const double* fv2, const int mask)
{
  __m128d v1 = _mm_load_pd(fv1);
  __m128d v2 = _mm_load_pd(fv2);
  __m128d dp1 = _mm_dp_pd(v1,v2,0x31);
  v1 = _mm_load_pd(&fv1[2]);
  v2 = _mm_load_pd(&fv2[2]);
  __m128d dp2 = _mm_dp_pd(v1,v2,mask);
  __m128d dp = _mm_add_sd(dp1,dp2);
  double res;
  _mm_store_sd(&res,dp);
  return res;
}

#endif

template <class num> inline num mysqrt(num x)
{
#ifdef GVECTOR_SSE_OPTIMIZATIONS
  return sse_sqrt<num>(x);
#else
  return sqrt(x);
#endif
}

template <class num> class vector3d;

template <class num>
class matrix3d{
public:
  union {
    struct{
      num m11;
      num m12;
      num m13;
      num m14;
      num m21;
      num m22;
      num m23;
      num m24;
      num m31;
      num m32;
      num m33;
      num m34;
      num m41;
      num m42;
      num m43;
      num m44;
    };
    num elements[16];
  }__attribute__ ((aligned (16)));

  void setxyzRotations(num rx, num ry, num rz);
  void xyzRotationAndTransformation(num rx, num ry, num rz, vector3d<num> t);
  matrix3d<num> operator*(const matrix3d< num > &M2) const;
  matrix3d<num> transpose() const;
};

template <class num> void matrix3d<num>::setxyzRotations(num rx, num ry, num rz)
{
  double cx = cos(rx);
  double sx = sin(rx);
  double cy = cos(ry);
  double sy = sin(ry);
  double cz = cos(rz);
  double sz = sin(rz);

  m11 = cz*cy;            m12 = -sz;        m13 = cz*sy;            m14 = 0.0;
  m21 = cx*sz*cy+sx*sy;   m22 = cx*cz;      m23 = cx*sz*sy-sx*cy;   m24 = 0.0;
  m31 = sx*sz*cy-cx*sy;   m32 = sx*cz;      m33 = cx*cy;            m34 = 0.0;
  m41 = 0.0;              m42 = 0.0;        m43 = 0.0;              m44 = 1.0;
}

template <class num> void matrix3d<num>::xyzRotationAndTransformation(num rx, num ry, num rz, vector3d< num > t)
{
  double cx = cos(rx);
  double sx = sin(rx);
  double cy = cos(ry);
  double sy = sin(ry);
  double cz = cos(rz);
  double sz = sin(rz);

  m11 = cz*cy;            m12 = -sz;        m13 = cz*sy;            m14 = t.x;
  m21 = cx*sz*cy+sx*sy;   m22 = cx*cz;      m23 = cx*sz*sy-sx*cy;   m24 = t.y;
  m31 = sx*sz*cy-cx*sy;   m32 = sx*cz;      m33 = cx*cy;            m34 = t.z;
  m41 = 0.0;              m42 = 0.0;        m43 = 0.0;              m44 = 1.0;
}

template <class num> matrix3d<num> matrix3d<num>::operator*(const matrix3d< num >& M2) const
{
  #define m1(i,j) m ## i ## j
  #define m2(i,j) M2.m ## i ## j
  #define m3(i,j) M3.m ## i ## j

  matrix3d<num> M3;

  m3(1,1) = m1(1,1)*m2(1,1) + m1(1,2)*m2(2,1) + m1(1,3)*m2(3,1) + m1(1,4)*m2(4,1);
  m3(1,2) = m1(1,1)*m2(1,2) + m1(1,2)*m2(2,2) + m1(1,3)*m2(3,2) + m1(1,4)*m2(4,2);
  m3(1,3) = m1(1,1)*m2(1,3) + m1(1,2)*m2(2,3) + m1(1,3)*m2(3,3) + m1(1,4)*m2(4,3);
  m3(1,4) = m1(1,1)*m2(1,4) + m1(1,2)*m2(2,4) + m1(1,3)*m2(3,4) + m1(1,4)*m2(4,4);

  m3(2,1) = m1(2,1)*m2(1,1) + m1(2,2)*m2(2,1) + m1(2,3)*m2(3,1) + m1(2,4)*m2(4,1);
  m3(2,2) = m1(2,1)*m2(1,2) + m1(2,2)*m2(2,2) + m1(2,3)*m2(3,2) + m1(2,4)*m2(4,2);
  m3(2,3) = m1(2,1)*m2(1,3) + m1(2,2)*m2(2,3) + m1(2,3)*m2(3,3) + m1(2,4)*m2(4,3);
  m3(2,4) = m1(2,1)*m2(1,4) + m1(2,2)*m2(2,4) + m1(2,3)*m2(3,4) + m1(2,4)*m2(4,4);

  m3(3,1) = m1(3,1)*m2(1,1) + m1(3,2)*m2(2,1) + m1(3,3)*m2(3,1) + m1(3,4)*m2(4,1);
  m3(3,2) = m1(3,1)*m2(1,2) + m1(3,2)*m2(2,2) + m1(3,3)*m2(3,2) + m1(3,4)*m2(4,2);
  m3(3,3) = m1(3,1)*m2(1,3) + m1(3,2)*m2(2,3) + m1(3,3)*m2(3,3) + m1(3,4)*m2(4,3);
  m3(3,4) = m1(3,1)*m2(1,4) + m1(3,2)*m2(2,4) + m1(3,3)*m2(3,4) + m1(3,4)*m2(4,4);

  m3(4,1) = m1(4,1)*m2(1,1) + m1(4,2)*m2(2,1) + m1(4,3)*m2(3,1) + m1(4,4)*m2(4,1);
  m3(4,2) = m1(4,1)*m2(1,2) + m1(4,2)*m2(2,2) + m1(4,3)*m2(3,2) + m1(4,4)*m2(4,2);
  m3(4,3) = m1(4,1)*m2(1,3) + m1(4,2)*m2(2,3) + m1(4,3)*m2(3,3) + m1(4,4)*m2(4,3);
  m3(4,4) = m1(4,1)*m2(1,4) + m1(4,2)*m2(2,4) + m1(4,3)*m2(3,4) + m1(4,4)*m2(4,4);

  return M3;
}

template <class num> matrix3d<num> matrix3d<num>::transpose() const
{
  matrix3d<num> M2 = *this;
  swap(M2.m12,M2.m21);
  swap(M2.m13,M2.m31);
  swap(M2.m14,M2.m41);
  swap(M2.m23,M2.m32);
  swap(M2.m24,M2.m42);
  swap(M2.m34,M2.m43);

  return M2;
}

template <class num>
class vector3d{
public:
  union {
    struct{
      num x;
      num y;
      num z;
      num w;
    };
    num coefficients[4];
  }__attribute__ ((aligned (16)));
  //num x,y,z;

  vector3d()
    {x=y=z=0.0;w=1.0;}
  vector3d(num nx,num ny,num nz)
    {x=nx; y=ny; z=nz; w=1.0;}
  vector3d(num nx,num ny,num nz,num nw)
    {x=nx; y=ny; z=nz; w=nw;}
  void set(num nx,num ny,num nz)
    {x=nx; y=ny; z=nz; w=1.0;}
  void set(num nx,num ny,num nz,num nw)
    {x=nx; y=ny; z=nz; w=nw;}
  void setAll(num nx)
    {x=y=z=nx; w=1.0;}
  void set(vector3d<num> p)
    {x=p.x; y=p.y; z=p.z; w = p.w;}
  void zero()
    {x=y=z=0; w=1.0;}

  vector3d<num> &operator=(const vector3d<num> p)
    {set(p); return(*this);}

  /// element accessor
  num &operator[](int idx)
    {return(coefficients[idx]);}
  const num &operator[](int idx) const
    {return(coefficients[idx]);}

  num length() const MustUseResult;
  num sqlength() const MustUseResult;
  vector3d<num> norm() const MustUseResult;
  vector3d<num> norm(const num len) const MustUseResult;
  void normalize();
  bool nonzero() const MustUseResult
    {return(x!=0 || y!=0 || z!=0);}

  num dot(const vector3d<num> p) const MustUseResult;
  vector3d<num> cross(const vector3d<num> p) const MustUseResult;

  vector3d<num> &operator+=(const vector3d<num> p);
  vector3d<num> &operator-=(const vector3d<num> p);
  vector3d<num> &operator*=(const vector3d<num> p);
  vector3d<num> &operator/=(const vector3d<num> p);

  vector3d<num> operator+(const vector3d<num> p) const;
  vector3d<num> operator-(const vector3d<num> p) const;
  vector3d<num> operator*(const vector3d<num> p) const;
  vector3d<num> operator/(const vector3d<num> p) const;

  vector3d<num> operator*(num f) const;
  vector3d<num> operator/(num f) const;
  vector3d<num> &operator*=(num f);
  vector3d<num> &operator/=(num f);

  vector3d<num> operator-() const;

  bool operator==(const vector3d<num> p) const;
  bool operator!=(const vector3d<num> p) const;
  bool operator< (const vector3d<num> p) const;
  bool operator> (const vector3d<num> p) const;
  bool operator<=(const vector3d<num> p) const;
  bool operator>=(const vector3d<num> p) const;

  vector3d<num> rotate(const vector3d<num> r,const double a) const;
  vector3d<num> transform(const matrix3d<num> &M) const;
  vector3d<num> rotate_x(const double a) const;
  vector3d<num> rotate_y(const double a) const;
  vector3d<num> rotate_z(const double a) const;

  //double shortest_angle(const vector3d<num> a,const vector3d<num> b);
  //vector3d<num> shortest_axis(const vector3d<num> a,const vector3d<num> b);

  bool finite() const MustUseResult
    {return(::finite(x) && ::finite(y) && ::finite(z));}

  void take_min(const vector3d<num> p);
  void take_max(const vector3d<num> p);
};

#ifdef GVECTOR_SSE_OPTIMIZATIONS
template <> float vector3d<float>::sqlength() const;

template <> float vector3d<float>::dot(const vector3d<float> p) const;

template <> vector3d<float> vector3d<float>::transform(const matrix3d<float> &M) const;
#endif

VECTOR_TEM num vector3d<num>::length() const
{
  return(mysqrt<num>(x*x + y*y + z*z));

}

VECTOR_TEM num vector3d<num>::sqlength() const
{
#ifdef GVECTOR_SSE_OPTIMIZATIONS
  __m128 p = _mm_set_ps((float)x,(float)y,(float)z,0.0f);
  p = _mm_mul_ps(p,p);
  __attribute__((aligned (16))) float x2[4];
  _mm_store_ps(x2,p);
  return(x2[3]+x2[2]+x2[1]);
#else
  return(x*x + y*y + z*z);
#endif
}

VECTOR_TEM vector3d<num> vector3d<num>::norm() const
{
#ifdef GVECTOR_SSE_OPTIMIZATIONS
  vector3d<num> p;
  num lInv = 1.0/(this->length());

  p.x = x * lInv;
  p.y = y * lInv;
  p.z = z * lInv;

  return(p);
#else
  vector3d<num> p;
  num l;

  l = mysqrt<num>(x*x + y*y + z*z);
  p.x = x / l;
  p.y = y / l;
  p.z = z / l;

  return(p);
#endif
}

VECTOR_TEM vector3d<num> vector3d<num>::norm(const num len) const
{
#ifdef GVECTOR_SSE_OPTIMIZATIONS
  vector3d<num> p;

  __m128 f = _mm_set1_ps(((float)len) / (this->length()));
  __m128 pm = _mm_set_ps((float)x,(float)y,(float)z,0.0f);
  pm = _mm_mul_ps(pm,f);
  __attribute__((aligned (16))) float x2[4];
  _mm_store_ps(x2,pm);
  p.x = x2[3];
  p.y = x2[2];
  p.z = x2[1];

  return(p);
#else
  vector3d<num> p;
  num f;

  f = len / mysqrt<num>(x*x + y*y + z*z);
  p.x = x * f;
  p.y = y * f;
  p.z = z * f;

  return(p);
#endif
}

VECTOR_TEM void vector3d<num>::normalize()
{
#ifdef GVECTOR_SSE_OPTIMIZATIONS
  __m128 f = _mm_set1_ps(1.0f / (this->length()));
  __m128 pm = _mm_set_ps((float)x,(float)y,(float)z,0.0f);
  pm = _mm_mul_ps(pm,f);
  __attribute__((aligned (16))) float x2[4];
  _mm_store_ps(x2,pm);
  x = x2[3];
  y = x2[2];
  z = x2[1];
  return;
#else
  num l;

  l = mysqrt<num>(x*x + y*y + z*z);
  x /= l;
  y /= l;
  z /= l;
#endif
}

VECTOR_TEM num vector3d<num>::dot(const vector3d<num> p) const
{
  return(x*p.x + y*p.y + z*p.z);
}

VECTOR_TEM num dot(const vector3d<num> a,const vector3d<num> b)
{
  return(a.x*b.x + a.y*b.y + a.z*b.z);
}

VECTOR_TEM num absdot(const vector3d<num> a,const vector3d<num> b)
{
  return(fabs(a.x*b.x) + fabs(a.y*b.y) + fabs(a.z*b.z));
}

VECTOR_TEM vector3d<num> vector3d<num>::cross(const vector3d<num> p) const
{
  vector3d<num> r;

  // right handed
  r.x = y*p.z - z*p.y;
  r.y = z*p.x - x*p.z;
  r.z = x*p.y - y*p.x;

  return(r);
}

VECTOR_TEM vector3d<num> cross(const vector3d<num> a,const vector3d<num> b)
{
  vector3d<num> r;

  // right handed
  r.x = a.y*b.z - a.z*b.y;
  r.y = a.z*b.x - a.x*b.z;
  r.z = a.x*b.y - a.y*b.x;

  return(r);
}

#define VECTOR3D_EQUAL_BINARY_OPERATOR(opr) \
  template <class num> \
  vector3d<num> &vector3d<num>::operator opr (const vector3d<num> p) \
  {                  \
    x opr p.x;   \
    y opr p.y;   \
    z opr p.z;   \
    return(*this);   \
  }

VECTOR3D_EQUAL_BINARY_OPERATOR(+=)
VECTOR3D_EQUAL_BINARY_OPERATOR(-=)
VECTOR3D_EQUAL_BINARY_OPERATOR(*=)
VECTOR3D_EQUAL_BINARY_OPERATOR(/=)

#define VECTOR3D_BINARY_OPERATOR(opr) \
  template <class num> \
  vector3d<num> vector3d<num>::operator opr (const vector3d<num> p) const \
  {                  \
    vector3d<num> r; \
    r.x = x opr p.x; \
    r.y = y opr p.y; \
    r.z = z opr p.z; \
    return(r);       \
  }

VECTOR3D_BINARY_OPERATOR(+)
VECTOR3D_BINARY_OPERATOR(-)
VECTOR3D_BINARY_OPERATOR(*)
VECTOR3D_BINARY_OPERATOR(/)

#define VECTOR3D_SCALAR_OPERATOR(opr) \
  template <class num> \
  vector3d<num> vector3d<num>::operator opr (const num f) const \
  {                  \
    vector3d<num> r; \
    r.x = x opr f;   \
    r.y = y opr f;   \
    r.z = z opr f;   \
    return(r);       \
  }

VECTOR3D_SCALAR_OPERATOR(*)
VECTOR3D_SCALAR_OPERATOR(/)

#define VECTOR3D_EQUAL_SCALAR_OPERATOR(opr) \
  template <class num> \
  vector3d<num> &vector3d<num>::operator opr (num f) \
  {                \
    x opr f;   \
    y opr f;   \
    z opr f;   \
    return(*this); \
  }

VECTOR3D_EQUAL_SCALAR_OPERATOR(*=)
VECTOR3D_EQUAL_SCALAR_OPERATOR(/=)

#define VECTOR3D_LOGIC_OPERATOR(opr,combine) \
  template <class num> \
  bool vector3d<num>::operator opr (const vector3d<num> p) const \
  {                            \
    return((x opr p.x) combine \
           (y opr p.y) combine \
           (z opr p.z));       \
  }

VECTOR3D_LOGIC_OPERATOR(==,&&)
VECTOR3D_LOGIC_OPERATOR(!=,||)

VECTOR3D_LOGIC_OPERATOR(< ,&&)
VECTOR3D_LOGIC_OPERATOR(> ,&&)
VECTOR3D_LOGIC_OPERATOR(<=,&&)
VECTOR3D_LOGIC_OPERATOR(>=,&&)

VECTOR_TEM vector3d<num> vector3d<num>::operator-() const
{
  vector3d<num> r;

  r.x = -x;
  r.y = -y;
  r.z = -z;

  return(r);
}

template<class num1, class num2> vector3d<num2> operator*(num1 f,const vector3d<num2> &a)
{
  vector3d<num2> r;

  r.x = f * a.x;
  r.y = f * a.y;
  r.z = f * a.z;

  return(r);
}

VECTOR_TEM vector3d<num> abs(vector3d<num> a)
{
  a.x = ::fabs(a.x);
  a.y = ::fabs(a.y);
  a.z = ::fabs(a.z);

  return(a);
}

VECTOR_TEM vector3d<num> min(vector3d<num> a,vector3d<num> b)
{
  vector3d<num> v;

  v.x = ::min(a.x,b.x);
  v.y = ::min(a.y,b.y);
  v.z = ::min(a.z,b.z);

  return(v);
}

VECTOR_TEM vector3d<num> max(vector3d<num> a,vector3d<num> b)
{
  vector3d<num> v;

  v.x = ::max(a.x,b.x);
  v.y = ::max(a.y,b.y);
  v.z = ::max(a.z,b.z);

  return(v);
}

VECTOR_TEM vector3d<num> bound(vector3d<num> v,num low,num high)
{
  v.x = ::bound(v.x,low,high);
  v.y = ::bound(v.y,low,high);
  v.z = ::bound(v.z,low,high);

  return(v);
}

// returns point rotated around axis <r> by <a> radians (right handed)
VECTOR_TEM vector3d<num> vector3d<num>::rotate(const vector3d<num> r,const double a) const
{
  vector3d<num> q;
  double s,c,t;

  s = sin(a);
  c = cos(a);
  t = 1 - c;

  q.x = (t * r.x * r.x + c      ) * x
      + (t * r.x * r.y - s * r.z) * y
      + (t * r.x * r.z + s * r.y) * z;

  q.y = (t * r.y * r.x + s * r.z) * x
      + (t * r.y * r.y + c      ) * y
      + (t * r.y * r.z - s * r.x) * z;

  q.z = (t * r.z * r.x - s * r.y) * x
      + (t * r.z * r.y + s * r.x) * y
      + (t * r.z * r.z + c      ) * z;

  return(q);
}

VECTOR_TEM vector3d<num> vector3d<num>::transform(const matrix3d< num >& M) const
{
  vector3d<num> q;
  q.x = M.m11*x + M.m12*y + M.m13*z + M.m14*w;
  q.y = M.m21*x + M.m22*y + M.m23*z + M.m24*w;
  q.z = M.m31*x + M.m32*y + M.m33*z + M.m34*w;

  return(q);
}

// returns point rotated around X axis by <a> radians (right handed)
VECTOR_TEM vector3d<num> vector3d<num>::rotate_x(const double a) const
{
  vector3d<num> q;
  double s,c;

  s = sin(a);
  c = cos(a);

  q.x = x;
  q.y = c*y + -s*z;
  q.z = s*y + c*z;

  return(q);
}

// returns point rotated around Y axis by <a> radians (right handed)
VECTOR_TEM vector3d<num> vector3d<num>::rotate_y(const double a) const
{
  vector3d<num> q;
  double s,c;

  s = sin(a);
  c = cos(a);

  q.x = c*x + s*z;
  q.y = y;
  q.z = -s*x + c*z;

  return(q);
}

// returns point rotated around Z axis by <a> radians (right handed)
VECTOR_TEM vector3d<num> vector3d<num>::rotate_z(const double a) const
{
  vector3d<num> q;
  double s,c;

  s = sin(a);
  c = cos(a);

  q.x = c*x + -s*y;
  q.y = s*x + c*y;
  q.z = z;

  return(q);
}

VECTOR_TEM double shortest_angle(const vector3d<num> a,const vector3d<num> b)
{
  return(acos(std::max(-1.0,std::min(1.0,dot(a,b)/(a.length()*b.length())))));
}

VECTOR_TEM vector3d<num> shortest_axis(const vector3d<num> a,const vector3d<num> b)
{
  return(cross(a,b).norm());
}


// set the vector to the minimum of its components and p's components
VECTOR_TEM void vector3d<num>::take_min(const vector3d<num> p)
{
  if(p.x < x) x = p.x;
  if(p.y < y) y = p.y;
  if(p.z < z) z = p.z;
}

// set the vector to the maximum of its components and p's components
VECTOR_TEM void vector3d<num>::take_max(const vector3d<num> p)
{
  if(p.x > x) x = p.x;
  if(p.y > y) y = p.y;
  if(p.z > z) z = p.z;
}

// returns distance between two points
VECTOR_TEM num dist(const vector3d<num> a,const vector3d<num> b)
{
  num dx,dy,dz;

  dx = a.x - b.x;
  dy = a.y - b.y;
  dz = a.z - b.z;

  return(mysqrt<num>(dx*dx + dy*dy + dz*dz));
}

VECTOR_TEM num distance(const vector3d<num> a,const vector3d<num> b)
{
  return(dist(a,b));
}

// returns square of distance between two points
VECTOR_TEM num sqdist(const vector3d<num> a,const vector3d<num> b)
{
  num dx,dy,dz;

  dx = a.x - b.x;
  dy = a.y - b.y;
  dz = a.z - b.z;

  return(dx*dx + dy*dy + dz*dz);
}

VECTOR_TEM num sqdistance(const vector3d<num> a,const vector3d<num> b)
{
  return(sqdist(a,b));
}

// returns distance from point p to line x0-x1
VECTOR_TEM num distance_to_line(const vector3d<num> x0,const vector3d<num> x1,const vector3d<num> p)
{
  // FIXME: this is probably broken
  vector3d<num> x;
  num t;

  t = ((p.x - x0.x) + (p.y - x0.y) + (p.z - x0.z)) / (x1.x + x1.y + x1.z);
  x = x0 + (x1 - x0) * t;

  return(distance(x,p));
}


//=====================================================================//
//  Vector2D Class
//=====================================================================//

template <class num>
class vector2d{
public:
  num x,y;

  vector2d()
    {}
  vector2d(num nx,num ny) : x(nx), y(ny)
    {}

  /// set the components of the vector
  void set(num nx,num ny)
    {x=nx; y=ny;}
  /// set the components of the vector to the same value
  void setAll(num nx)
    {x=y=nx;}
  /// set the components of the vector
  template <class vec> void set(vec p)
    {x=p.x; y=p.y;}
  /// zero all components of the vector
  void zero()
    {x=y=0;}

  /// copy constructor
  vector2d<num> &operator=(const vector2d<num>& p)
    {set(p); return(*this);}

  /// copy constructor from compatible classes
  template <class vec> vector2d<num> &operator=(vec p)
    {set(p.x,p.y); return(*this);}

  /// element accessor
  num &operator[](int idx)
    {return(((num*)this)[idx]);}
  const num &operator[](int idx) const
    {return(((num*)this)[idx]);}

  /// calculate Euclidean length
  num length() const MustUseResult;
  /// calculate squared Euclidean length (faster than length())
  num sqlength() const MustUseResult;
  /// calculate the clockwise angle from <1,0>
  num angle() const MustUseResult
    {return(atan2(y,x));}
  /// make a unit vector at given angle
  void heading(num angle)
    {x=cos(angle); y=sin(angle);}

  /// return a unit length vector in the same direction
  vector2d<num> norm() const MustUseResult;
  /// return a length 'len' vector in the same direction
  vector2d<num> norm(const num len) const MustUseResult;
  /// normalize to unit length in place
  void normalize();
  /// bound vector to a maximum length
  vector2d<num> bound(const num max_length) const MustUseResult;
  /// return if vector has any length at all
  bool nonzero() const MustUseResult
    {return(x!=0 || y!=0);}

  /// return dot product of vector with p
  template <class vec> num dot(const vec& p) const MustUseResult;
  /// return dot product of vector's perp with p, equivalent to (this->perp()).dot(p)
  num perpdot(const vector2d<num>& p) const MustUseResult;
  /// return z component of 3D cross product on 2D vectors.  right handed.
  num cross(const vector2d<num>& p) const MustUseResult;

  /// return the perpendicular of a vector (i.e. rotated 90 deg counterclockwise)
  vector2d<num> perp() const MustUseResult
    {return(vector2d(-y, x));}

  /// add a vector to the current element values
  vector2d<num> &operator+=(const vector2d<num>& p);
  /// subtract a vector from the current element values
  vector2d<num> &operator-=(const vector2d<num>& p);
  /// multiply (elementwise) a vector with the current element values
  vector2d<num> &operator*=(const vector2d<num>& p);
  /// divide (elementwise) a vector with the current element values
  vector2d<num> &operator/=(const vector2d<num>& p);

  /// return vector sum of this vector and p
  vector2d<num> operator+(const vector2d<num>& p) const;
  /// return vector difference of this vector and p
  vector2d<num> operator-(const vector2d<num>& p) const;
  /// return elementwise product of this vector and p
  vector2d<num> operator*(const vector2d<num>& p) const;
  /// return elementwise division of this vector by p
  vector2d<num> operator/(const vector2d<num>& p) const;

  /// return this vector scaled by f
  vector2d<num> operator*(const num f) const;
  /// return this vector scaled by 1/f
  vector2d<num> operator/(const num f) const;
  /// scale this vector by f
  vector2d<num> &operator*=(num f);
  /// scale this vector by 1/f
  vector2d<num> &operator/=(num f);

  /// negate vector (reflect through origin) <x,y> -> <-x,-y>
  vector2d<num> operator-() const;

  bool operator==(const vector2d<num>& p) const;
  bool operator!=(const vector2d<num>& p) const;
  bool operator< (const vector2d<num>& p) const;
  bool operator> (const vector2d<num>& p) const;
  bool operator<=(const vector2d<num>& p) const;
  bool operator>=(const vector2d<num>& p) const;

  /// return vector rotated by angle a
  vector2d<num> rotate(const double a) const MustUseResult;

  // vector2d<num> project(const vector2d<num> p) const;
  vector2d<num> project_in(const vector2d<num>& p) const MustUseResult;
  vector2d<num> project_out(const vector2d<num>& p) const MustUseResult;

  /// return true if both elements are finite, otherwise return false
  bool finite() const MustUseResult
    {return(::finite(x) && ::finite(y));}

  /// set the vector to the minimum of its components and p's components
  void take_min(const vector2d<num>& p);
  /// set the vector to the maximum of its components and p's components
  void take_max(const vector2d<num>& p);
};

VECTOR_TEM num vector2d<num>::length() const
{
  return(mysqrt<num>(x*x + y*y));
}

VECTOR_TEM num vector2d<num>::sqlength() const
{
  return(x*x + y*y);
}

VECTOR_TEM vector2d<num> vector2d<num>::norm() const
{
  vector2d<num> p;
  num l;

  l = mysqrt<num>(x*x + y*y);
  p.x = x / l;
  p.y = y / l;

  return(p);
}

VECTOR_TEM vector2d<num> vector2d<num>::norm(const num len) const
{
  vector2d<num> p;
  num f;

  f = len / mysqrt<num>(x*x + y*y);
  p.x = x * f;
  p.y = y * f;

  return(p);
}

VECTOR_TEM void vector2d<num>::normalize()
{
  num l;

  l = mysqrt<num>(x*x + y*y);
  x /= l;
  y /= l;
}

VECTOR_TEM vector2d<num> vector2d<num>::bound(const num max_length) const
{
  vector2d<num> p;
  num lsq,f;

  lsq = x*x + y*y;

  if(lsq < sq(max_length)){
    p.set(x,y);
  }else{
    f = max_length / mysqrt<num>(lsq);
    p.set(f*x,f*y);
  }

  return(p);
}

template <class num> template <class vec> num vector2d<num>::dot(const vec& p) const
{
  return(x*p.x + y*p.y);
}

VECTOR_TEM num vector2d<num>::perpdot(const vector2d<num>& p) const
// perp product, equivalent to (this->perp()).dot(p)
{
  return(x*p.y - y*p.x);
}

VECTOR_TEM num dot(const vector2d<num>& a,const vector2d<num>& b)
{
  return(a.x*b.x + a.y*b.y);
}

VECTOR_TEM num cosine(const vector2d<num>& a, const vector2d<num>& b)
// equivalent to dot(a.norm(),b.norm())
{
  num l;

  l = mysqrt<num>(a.x*a.x + a.y*a.y) * mysqrt<num>(b.x*b.x + b.y*b.y);

  return((a.x*b.x + a.y*b.y) / l);
}

VECTOR_TEM num vector2d<num>::cross(const vector2d<num>& p) const
{
  // right handed
  return(x*p.y - p.x*y);
}

// returns point rotated by <a> radians
VECTOR_TEM vector2d<num> vector2d<num>::rotate(const double a) const
{
  vector2d<num> q;
  double s,c;

  s = sin(a);
  c = cos(a);

  q.x = c*x + -s*y;
  q.y = s*x + c*y;

  return(q);
}

/* Depricated: replace "p.project(basis)" with "basis.project_out(p)"
/// returns vector projected onto (p, p.perp()) basis.
/// equivalent to q = p*x + p.perp()*y;
template <class num>
vector2d<num> vector2d<num>::project(const vector2d<num> p) const
{
  vector2d<num> q;

  q.x = p.x*x - p.y*y;
  q.y = p.y*x + p.x*y;

  return(q);
}
*/

/// takes a vector p in outer coordinate space and returns one
/// projected onto basis given by this,this.perp()
template <class num>
vector2d<num> vector2d<num>::project_in(const vector2d<num>& p) const
{
  vector2d<num> q;
  q.x = x*p.x + y*p.y; // q.x = this->dot(p);
  q.y = x*p.y - y*p.x; // q.y = this->perpdot(p);
  return(q);
}

/// takes a vector p in basis given by this,this.perp() and returns
/// one in the outer coordinate space
template <class num>
vector2d<num> vector2d<num>::project_out(const vector2d<num>& p) const
{
  vector2d<num> q;
  q.x = x*p.x - y*p.y;
  q.y = y*p.x + x*p.y;
  return(q);
}

#define VECTOR2D_EQUAL_BINARY_OPERATOR(opr) \
  template <class num> \
  vector2d<num> &vector2d<num>::operator opr (const vector2d<num>& p) \
  {                  \
    x opr p.x;   \
    y opr p.y;   \
    return(*this);   \
  }

VECTOR2D_EQUAL_BINARY_OPERATOR(+=)
VECTOR2D_EQUAL_BINARY_OPERATOR(-=)
VECTOR2D_EQUAL_BINARY_OPERATOR(*=)
VECTOR2D_EQUAL_BINARY_OPERATOR(/=)

#define VECTOR2D_BINARY_OPERATOR(opr) \
  template <class num> \
  vector2d<num> vector2d<num>::operator opr (const vector2d<num>& p) const \
  {                  \
    vector2d<num> r; \
    r.x = x opr p.x; \
    r.y = y opr p.y; \
    return(r);       \
  }

VECTOR2D_BINARY_OPERATOR(+)
VECTOR2D_BINARY_OPERATOR(-)
VECTOR2D_BINARY_OPERATOR(*)
VECTOR2D_BINARY_OPERATOR(/)

#define VECTOR2D_SCALAR_OPERATOR(opr) \
  template <class num> \
  vector2d<num> vector2d<num>::operator opr (const num f) const \
  {                  \
    vector2d<num> r;  \
    r.x = x opr f;   \
    r.y = y opr f;   \
    return(r);       \
  }

VECTOR2D_SCALAR_OPERATOR(*)
VECTOR2D_SCALAR_OPERATOR(/)

#define VECTOR2D_EQUAL_SCALAR_OPERATOR(opr) \
  template <class num> \
  vector2d<num> &vector2d<num>::operator opr (num f) \
  {                \
    x opr f;   \
    y opr f;   \
    return(*this); \
  }

VECTOR2D_EQUAL_SCALAR_OPERATOR(*=)
VECTOR2D_EQUAL_SCALAR_OPERATOR(/=)

#define VECTOR2D_LOGIC_OPERATOR(opr,combine) \
  template <class num> \
  bool vector2d<num>::operator opr (const vector2d<num>& p) const \
  {                            \
    return((x opr p.x) combine \
           (y opr p.y));       \
  }

VECTOR2D_LOGIC_OPERATOR(==,&&)
VECTOR2D_LOGIC_OPERATOR(!=,||)

VECTOR2D_LOGIC_OPERATOR(< ,&&)
VECTOR2D_LOGIC_OPERATOR(> ,&&)
VECTOR2D_LOGIC_OPERATOR(<=,&&)
VECTOR2D_LOGIC_OPERATOR(>=,&&)


template <class num>
inline vector2d<num> vector2d<num>::operator-() const
{
  vector2d<num> r;
  r.x = -x;
  r.y = -y;
  return(r);
}

template <class num1,class num2>
inline vector2d<num2> operator*(num1 f,const vector2d<num2>& a)
{
  vector2d<num2> r;

  r.x = f * a.x;
  r.y = f * a.y;

  return(r);
}

template <class num>
inline void vector2d<num>::take_min(const vector2d<num>& p)
{
  if(p.x < x) x = p.x;
  if(p.y < y) y = p.y;
}

template <class num>
inline void vector2d<num>::take_max(const vector2d<num>& p)
{
  if(p.x > x) x = p.x;
  if(p.y > y) y = p.y;
}

VECTOR_TEM vector2d<num> abs(vector2d<num> a)
{
  a.x = ::fabs(a.x);
  a.y = ::fabs(a.y);

  return(a);
}

VECTOR_TEM vector2d<num> min(const vector2d<num>& a, const vector2d<num>& b)
{
  vector2d<num> v;

  v.x = ::min(a.x,b.x);
  v.y = ::min(a.y,b.y);

  return(v);
}

VECTOR_TEM vector2d<num> max(const vector2d<num>& a, const vector2d<num>& b)
{
  vector2d<num> v;

  v.x = ::max(a.x,b.x);
  v.y = ::max(a.y,b.y);

  return(v);
}

VECTOR_TEM vector2d<num> bound(vector2d<num> v,num low,num high)
{
  v.x = ::bound(v.x,low,high);
  v.y = ::bound(v.y,low,high);

  return(v);
}

VECTOR_TEM num dist(const vector2d<num>& a,const vector2d<num>& b)
{
  num dx,dy;

  dx = a.x - b.x;
  dy = a.y - b.y;

  return(mysqrt<num>(dx*dx + dy*dy));
}

VECTOR_TEM num distance(const vector2d<num>& a,const vector2d<num>& b)
{
  return(dist(a,b));
}

// returns square of distance between two points
VECTOR_TEM num sqdist(const vector2d<num>& a,const vector2d<num>& b)
{
  num dx,dy;

  dx = a.x - b.x;
  dy = a.y - b.y;

  return(dx*dx + dy*dy);
}

VECTOR_TEM num sqdistance(const vector2d<num>& a,const vector2d<num>& b)
{
  return(sqdist(a,b));
}

}; // namespace vector

#endif
// __VECTOR_H__
