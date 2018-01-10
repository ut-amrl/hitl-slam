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
\file    quaternion_helper.h
\brief   Helpers for integrating Eigen Quaternions with GVector classes
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include "geometry.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#ifndef QUATERNION_HELPER_H
#define QUATERNION_HELPER_H

namespace GVector {
  
  template <class num> inline GVector::vector3d<num> quat_rotate(const Eigen::Quaternion<num> &quat, const GVector::vector3d<num> &vec) __attribute__ ((warn_unused_result));
  template <class num> inline GVector::vector3d<num> quat_rotate(const Eigen::Quaternion< num >& quat, const GVector::vector3d< num >& vec)
  {
    Eigen::Matrix<num,3,1> v(vec.x,vec.y,vec.z);
    v = quat*v;
    return GVector::vector3d<num>(v[0],v[1],v[2]);
  }
  
  template <class num> void quatToMatrix(const Eigen::Quaternion<num> &quat, GVector::matrix3d<num> &M)
  {
    Eigen::Matrix<num,3,3> mat;
    mat = quat.toRotationMatrix();
    M.m11 = mat(0,0);
    M.m12 = mat(0,1);
    M.m13 = mat(0,2);
    M.m14 = 0.0;
    M.m21 = mat(1,0);
    M.m22 = mat(1,1);
    M.m23 = mat(1,2);
    M.m24 = 0.0;
    M.m31 = mat(2,0);
    M.m32 = mat(2,1);
    M.m33 = mat(2,2);
    M.m34 = 0.0;
    M.m41 = 0.0;
    M.m42 = 0.0;
    M.m43 = 0.0;
    M.m44 = 1.0;
  }
  
  template <class num> void transformMatrix(const Eigen::Quaternion<num> &quat, const GVector::vector3d<num> &t, GVector::matrix3d<num> &M)
  {
    Eigen::Matrix<num,3,3> mat;
    mat = quat.toRotationMatrix();
    M.m11 = mat(0,0);
    M.m12 = mat(0,1);
    M.m13 = mat(0,2);
    M.m14 = t.x;
    M.m21 = mat(1,0);
    M.m22 = mat(1,1);
    M.m23 = mat(1,2);
    M.m24 = t.y;
    M.m31 = mat(2,0);
    M.m32 = mat(2,1);
    M.m33 = mat(2,2);
    M.m34 = t.z;
    M.m41 = 0.0;
    M.m42 = 0.0;
    M.m43 = 0.0;
    M.m44 = 1.0;
  }
  
  template <class num> Eigen::Quaternion<num> quat_construct(const GVector::vector3d<num> &axis, const num angle)
  {
    Eigen::Matrix<num,3,1> axisVector(axis.x,axis.y,axis.z);
    Eigen::AngleAxis<num> angleAxisForm(angle,axisVector);
    return Eigen::Quaternion<num>(angleAxisForm);
  }
  
  /// Generates a random rotation quaternion which rotates by a specified max angle, drawn from a Gaussian dsitribution or rotation angles.
  template <class num> Eigen::Quaternion<num> randRot(const num max_angle, const Eigen::Quaternion<num> &angle)
  {
    GVector::vector3d<num> randAxis(frand(-1.0,1.0),frand(-1.0,1.0),frand(-1.0,1.0));
    num randAngle = frand<num>(0.0,max_angle);
    if(randAngle>0.0 && randAxis.sqlength()>0.0){
      randAxis.normalize();
      return quat_construct<num>(randAxis,randAngle)*angle;
    }
    return angle;
  }
  
  /// Generates a random rotation quaternion which rotates by a specified max angle, drawn from a Gaussian dsitribution or rotation angles.
  template <class num> Eigen::Quaternion<num> randRot(const num &max_angle)
  {
    GVector::vector3d<num> axis(frand(-1.0,1.0),frand(-1.0,1.0),frand(-1.0,1.0));
    num angle = frand<num>(0.0,max_angle);
    if(angle>0.0 && axis.sqlength()>0.0){
      axis.normalize();
      return quat_construct<num>(axis,angle)*angle;
    }
    return Eigen::Quaternion<num>(1.0,0.0,0.0,0.0);
  }
};

#endif //QUATERNION_HELPER_H
