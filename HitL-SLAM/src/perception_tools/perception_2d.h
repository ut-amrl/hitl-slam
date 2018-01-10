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
// Copyright 2015 joydeepb@cs.umass.edu
// Computer Science Department, University of Massachusetts Amherst
//
// Data types and helper functions for perception in 2D.

#ifndef PERCEPTION_2D_H
#define PERCEPTION_2D_H

#include <math.h>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

namespace perception_2d {

template <typename num>
struct Pose2D {
  Eigen::Matrix<num, 2, 1> translation;
  num angle;

  // Default constructor, does not initialize anything.
  Pose2D() {}

  // Initialization contructor with an angle and Vector2f.
  Pose2D(const num& new_angle,
         const Eigen::Matrix<num, 2, 1>& new_translation) :
      translation(new_translation), angle(new_angle) {}

  // Templated constructor with an angle and templated 2D vector.
  template <typename T>
  Pose2D(const num& new_angle,
         const T& new_translation) :
      translation(new_translation.x, new_translation.y), angle(new_angle) {}

  // Initialization contructor with angle, x, y.
  Pose2D(const num& new_angle, const num& x, const num& y) :
      translation(x, y), angle(new_angle) {}

  // Initialization contructor from an Affine 2D transform.
  Pose2D(const Eigen::Transform<num, 2, Eigen::Affine>& affine) :
        translation(0, 0), angle(0) {
    Eigen::Rotation2D<num> rotation(0.0);
    rotation.fromRotationMatrix(affine.rotation());
    angle = rotation.angle();
    translation = affine.translation();
  }

  // Set the pose to identity.
  void Clear() {
    angle = static_cast<num>(0.0);
    translation.setZero();
  }

  // Set the pose to the specified state.
  void Set(const num& new_angle,
           const Eigen::Matrix<num, 2, 1>& new_translation) {
    angle = new_angle;
    translation = new_translation;
  }

  // Apply the pose @pose_other after this pose.
  void ApplyPose(const Pose2D<num>& pose_other) {
    const Eigen::Rotation2D<num> rotation(angle);
    translation += (rotation * pose_other.translation);
    angle = angle_mod(angle + pose_other.angle);
  }
};


typedef std::vector<Eigen::Vector2f> PointCloudf;
typedef std::vector<Eigen::Vector2f> NormalCloudf;
typedef Pose2D<float> Pose2Df;

// Generate normals from an ordered 2D point cloud by approximating the normal
// as being perpendicular to the tangent, which in turn is approximated by
// connecting each point with its neighbours.
void GenerateNormals(const float kMaxNormalPointDistance,
                     PointCloudf* point_cloud_ptr,
                     NormalCloudf* normal_cloud_ptr);

}  // namespace perception_2d

#endif  // PERCEPTION_2D_H
