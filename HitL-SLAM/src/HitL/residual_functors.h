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
// Residual functors for Ceres optimizations for robot localization and
// SLAM in 2D (3 Degrees of Freedom: x, y, theta).

#ifndef RESIDUAL_FUNCTORS_H
#define RESIDUAL_FUNCTORS_H

#include <cmath>
#include <ceres/jet.h>
#include <cmath>
#include <glog/logging.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <boost/concept_check.hpp>

#include "../shared/util/openmp_utils.h"
//#include "vector_localization/non_markov_localization.h"

namespace ceres {

template <typename T, int N> inline
Jet<T, N> fabs(const Jet<T, N>& f) {
  if (f.a < T(0.0)) {
    return (-f);
  }
  return f;
}

template <typename T, int N> inline
Jet<T, N> floor(const Jet<T, N>& f) {
  const T f_int = std::floor(f.a);
  const T f_total = f.a;
  return (f_int / f_total) * f;
}

}  // namespace ceres

namespace {

// sign
template <typename T> inline
T Sign(const T& x) {
  if (x < T(0.0)) return (T(-1.0));
  return (T(1.0));
}



//namespace vector_localization {

template <typename T> inline T sq(const T&x) { return x * x; }

struct LTSConstraint {
  LTSConstraint(const std::size_t _pose_index,
                const Eigen::Vector2f& _point,
                const Eigen::Vector2f& _line_normal,
                const Eigen::Vector2f& _line_dir,
                const Eigen::Vector2f& _line_p1,
                const Eigen::Vector2f& _line_p2,
                float _line_offset,
                float _std_dev,
                float _correlation_factor)
      : pose_index(_pose_index), point(_point), line_normal(_line_normal),
      line_dir(_line_dir), line_p1(_line_p1), line_p2(_line_p2),
      line_offset(_line_offset), std_dev(_std_dev),
      correlation_factor(_correlation_factor) {}

  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    // The first two elements in the pose array are the X,Y coordinates of the
    // robot.
    const Eigen::Matrix<T, 2, 1> pose_translation(pose[0], pose[1]);
    // The third element in the pose array is the rotation angle of the
    // robot.
    const Eigen::Rotation2D<T> pose_rotation(pose[2]);
    // Transform point from robot frame to global frame.
    const Eigen::Matrix<T, 2, 1> point_global_coords =
        (pose_rotation * point.cast<T>()) + pose_translation;
    // Type cast the line normal from a float type to "T" type.
    const Eigen::Matrix<T, 2, 1> line_normal_t = line_normal.cast<T>();
    // The signed distance d of a point p from a line is given by the equation
    // d = p.dot(line_normal) + line_offset.
    const T off_line_error =
        point_global_coords.dot(line_normal_t) + T(line_offset);
    const T pose_line_error =
        pose_translation.dot(line_normal_t) + T(line_offset);
    if (pose_line_error * off_line_error > T(0.0)) {
      // Observations are on the same side of the line as the robot.
      static const double kMaxObstacleError = 0.5;
      if (off_line_error < T(kMaxObstacleError) &&
          off_line_error > T(-kMaxObstacleError)) {
        residuals[0] = off_line_error / T(std_dev) * T(correlation_factor);
      } else {
        residuals[0] =
            T(kMaxObstacleError) / T(std_dev) * T(correlation_factor);
      }
    } else {
      // Observations are on opposide sides of the line as the robot: the robot
      // is seeing through walls.
      // Scale the error by the correlation factor to discount correlations
      // between points.
      residuals[0] = off_line_error / T(std_dev) *
          T(correlation_factor);
    }
    return true;
  }

  // The pose index from which this point was observed.
  const std::size_t pose_index;
  // The coordinates of the point in the robot local coordinate frame.
  const Eigen::Vector2f point;
  // The normal to the line (in global coordinates) of the map that this point
  // corresponds to.
  Eigen::Vector2f line_normal;
  // A vector parallel to the direction of the line.
  Eigen::Vector2f line_dir;
  // Endpoint 1 of the line segment.
  Eigen::Vector2f line_p1;
  // Endpoint 2 of the line segment.
  Eigen::Vector2f line_p2;
  // The offset (shortest distance from origin) of the line (in global
  // coordinates) of the map that this point corresponds to.
  float line_offset;
  // Standard deviation of observations.
  float std_dev;
  // Discounting constant to account for correlations between observed points.
  const float correlation_factor;
};

struct VisibilityConstraint {
  VisibilityConstraint(const std::size_t _pose_index,
                       const Eigen::Vector2f& _point,
                       const Eigen::Vector2f& _line_normal,
                       const Eigen::Vector2f& _line_dir,
                       const Eigen::Vector2f& _line_p1,
                       const Eigen::Vector2f& _line_p2,
                       float _line_offset,
                       float _std_dev,
                       float _correlation_factor)
      : pose_index(_pose_index), point(_point), line_normal(_line_normal),
      line_dir(_line_dir), line_p1(_line_p1), line_p2(_line_p2),
      line_offset(_line_offset), std_dev(_std_dev),
      correlation_factor(_correlation_factor) {}

  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    // The first two elements in the pose array are the X,Y coordinates of the
    // robot.
    const Eigen::Matrix<T, 2, 1> pose_translation(pose[0], pose[1]);
    // The third element in the pose array is the rotation angle of the
    // robot.
    const Eigen::Rotation2D<T> pose_rotation(pose[2]);
    // Transform point from robot frame to global frame.
    const Eigen::Matrix<T, 2, 1> point_global_coords =
        (pose_rotation * point.cast<T>()) + pose_translation;
    const Eigen::Matrix<T, 2, 1> line_p1_t = line_p1.cast<T>();
    const Eigen::Matrix<T, 2, 1> line_p2_t = line_p2.cast<T>();
    if ((line_p1_t - point_global_coords).dot(
        line_p2_t - point_global_coords) > 0.0) {
      // The point projected onto the line falls outside the line segment.
      residuals[0] = T(0.0);
      return  true;
    }
    // Type cast the line normal from a float type to "T" type.
    const Eigen::Matrix<T, 2, 1> line_normal_t = line_normal.cast<T>();
    // The signed distance d of a point p from a line is given by the equation
    // d = p.dot(line_normal) + line_offset .
    const T off_line_error =
        point_global_coords.dot(line_normal_t) + T(line_offset);
    const T pose_line_error =
        pose_translation.dot(line_normal_t) + T(line_offset);
    if (pose_line_error * off_line_error > T(0.0)) {
      // Observations are on the same side of the line as the robot.
      residuals[0] = T(0.0);
    } else {
      // Observations are on opposide sides of the line as the robot: the robot
      // is seeing through walls.
      // Scale the error by the correlation factor to discount correlations
      // between points.
      residuals[0] = off_line_error *
          T(correlation_factor) / T(std_dev);
    }
    return true;
  }

  // The pose index from which this point was observed.
  const std::size_t pose_index;
  // The coordinates of the point in the robot local coordinate frame.
  const Eigen::Vector2f point;
  // The normal to the line (in global coordinates) of the map that this point
  // corresponds to.
  Eigen::Vector2f line_normal;
  // A vector parallel to the direction of the line.
  Eigen::Vector2f line_dir;
  // Endpoint 1 of the line segment.
  Eigen::Vector2f line_p1;
  // Endpoint 2 of the line segment.
  Eigen::Vector2f line_p2;
  // The offset (shortest distance from origin) of the line (in global
  // coordinates) of the map that this point corresponds to.
  float line_offset;
  // Standard deviation of observations.
  float std_dev;
  // Discounting constant to account for correlations between observed points.
  const float correlation_factor;
};

struct VisibilityRelativeConstraint {
  VisibilityRelativeConstraint(
      const std::size_t _pose_index,
      const Eigen::Vector2f& _point,
      const Eigen::Vector2f& _line_normal,
      const Eigen::Vector2f& _line_dir,
      const Eigen::Vector2f& _line_p1,
      const Eigen::Vector2f& _line_p2,
      float _line_offset,
      float _std_dev,
      float _correlation_factor) :
      pose_index(_pose_index), point(_point), line_normal(_line_normal),
      line_dir(_line_dir), line_p1(_line_p1), line_p2(_line_p2),
      line_offset(_line_offset), std_dev(_std_dev),
      correlation_factor(_correlation_factor) {}

  template <typename T>
  bool operator()(T const* const* relative_pose_array, T* residuals) const {
    T pose[3] = {
        relative_pose_array[0][0],
        relative_pose_array[0][1],
        relative_pose_array[0][2]
    };
    for (size_t i = 1; i <= pose_index; ++i) {
      pose[0] = pose[0] + relative_pose_array[i][0];
      pose[1] = pose[1] + relative_pose_array[i][1];
      pose[2] = pose[2] + relative_pose_array[i][2];
    }
    // The first two elements in the pose array are the X,Y coordinates of the
    // robot.
    const Eigen::Matrix<T, 2, 1> pose_translation(pose[0], pose[1]);
    // The third element in the pose array is the rotation angle of the
    // robot.
    const Eigen::Rotation2D<T> pose_rotation(pose[2]);
    // Transform point from robot frame to global frame.
    const Eigen::Matrix<T, 2, 1> point_global_coords =
        (pose_rotation * point.cast<T>()) + pose_translation;
    const Eigen::Matrix<T, 2, 1> line_p1_t = line_p1.cast<T>();
    const Eigen::Matrix<T, 2, 1> line_p2_t = line_p2.cast<T>();
    if ((line_p1_t - point_global_coords).dot(
        line_p2_t - point_global_coords) > 0.0) {
      // The point projected onto the line falls outside the line segment.
      residuals[0] = T(0.0);
      return  true;
    }
    // Type cast the line normal from a float type to "T" type.
    const Eigen::Matrix<T, 2, 1> line_normal_t = line_normal.cast<T>();
    // The signed distance d of a point p from a line is given by the equation
    // d = p.dot(line_normal) + line_offset .
    const T off_line_error =
        point_global_coords.dot(line_normal_t) + T(line_offset);
    const T pose_line_error =
        pose_translation.dot(line_normal_t) + T(line_offset);
    if (pose_line_error * off_line_error > T(0.0)) {
      // Observations are on the same side of the line as the robot.
      residuals[0] = T(0.0);
    } else {
      // Observations are on opposide sides of the line as the robot: the robot
      // is seeing through walls.
      // Scale the error by the correlation factor to discount correlations
      // between points.
      residuals[0] = off_line_error *
          T(correlation_factor) / T(std_dev);
    }
    return true;
  }

  // The pose index from which this point was observed.
  const std::size_t pose_index;
  // The coordinates of the point in the robot local coordinate frame.
  const Eigen::Vector2f point;
  // The normal to the line (in global coordinates) of the map that this point
  // corresponds to.
  Eigen::Vector2f line_normal;
  // A vector parallel to the direction of the line.
  Eigen::Vector2f line_dir;
  // Endpoint 1 of the line segment.
  Eigen::Vector2f line_p1;
  // Endpoint 2 of the line segment.
  Eigen::Vector2f line_p2;
  // The offset (shortest distance from origin) of the line (in global
  // coordinates) of the map that this point corresponds to.
  float line_offset;
  // Standard deviation of observations.
  float std_dev;
  // Discounting constant to account for correlations between observed points.
  const float correlation_factor;
};

struct PointToLineGlobConstraint {
  PointToLineGlobConstraint(int _pose_index,
                            const std::vector<Eigen::Vector2f>& _points,
                            const std::vector<Eigen::Vector2f>& _line_normals,
                            const std::vector<float>& _line_offsets,
                            const std::vector<bool>& _valid,
                            float _std_dev,
                            float _correlation_factor)
      : pose_index(_pose_index),
      points(_points), line_normals(_line_normals), line_offsets(_line_offsets),
      correspondence_valid(_valid), std_dev(_std_dev),
      correlation_factor(_correlation_factor) {
    CHECK_EQ(correspondence_valid.size(), points.size());
  }

  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    T& residual = residuals[0];
    // The first two elements in the pose array are the X,Y coordinates of the
    // robot.
    const Eigen::Matrix<T, 2, 1> pose_translation(pose[0], pose[1]);
    // The third element in the pose array is the rotation angle of the
    // robot.
    const Eigen::Matrix<T, 2, 2> pose_rotation =
        Eigen::Rotation2D<T>(pose[2]).toRotationMatrix();
    std::vector<T> point_residuals(points.size(), T(0));
    OMP_PARALLEL_FOR
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (!correspondence_valid[i]) continue;
      // Transform point from robot frame to global frame.
      const Eigen::Matrix<T, 2, 1> point_global_coords =
          (pose_rotation * points[i].cast<T>()) + pose_translation;
      // Type cast the line normal from a float type to "T" type.
      const Eigen::Matrix<T, 2, 1> line_normal_t = line_normals[i].cast<T>();
      // The signed distance d of a point p from a line is given by the equation
      // d = p.dot(line_normal) + line_offset .
      const T off_line_error =
          point_global_coords.dot(line_normal_t) + T(line_offsets[i]);
      // Scale the error by the correlation factor to discount correlations
      // between points.
      point_residuals[i] = sq(off_line_error * T(correlation_factor) / T(std_dev));
      // residual = residual +
      //     sq(off_line_error * T(correlation_factor) / T(std_dev));
    }
    residual = std::accumulate(
        point_residuals.begin(), point_residuals.end(), T(0));
    /*
    if (residual != T(0.0)) {
      residual = ceres::sqrt(residual / static_cast<T>(points.size()));
    }
    */
    return true;
  }

  // Index number of the pose that this point was observed from.
  const int pose_index;
  // The coordinates of the point in the robot local coordinate frame.
  const std::vector<Eigen::Vector2f>& points;
  // The normal to the line (in global coordinates) of the map that this point
  // corresponds to.
  const std::vector<Eigen::Vector2f> line_normals;
  // The offset (shortest distance from origin) of the lines(in global
  // coordinates) of the map that the points correspond to.
  const std::vector<float> line_offsets;
  // Binary variable that indicates whether a valid corresponding line was
  // found for the corresponding point.
  const std::vector<bool> correspondence_valid;
  // Standard deviation of observations.
  const float std_dev;
  // Discounting constant to account for correlations between observed points.
  const float correlation_factor;
};

struct VisibilityGlobConstraint {
  VisibilityGlobConstraint(
      const std::size_t _pose_index,
      const std::vector<Eigen::Vector2f>& _points,
      const std::vector<Eigen::Vector2f>& _line_normals,
      const std::vector<Eigen::Vector2f>& _line_p1s,
      const std::vector<Eigen::Vector2f>& _line_p2s,
      const std::vector<float>& _line_offsets,
      float _std_dev,
      float _correlation_factor)
  : pose_index(_pose_index), points(_points), line_normals(_line_normals),
  line_p1s(_line_p1s), line_p2s(_line_p2s),
  line_offsets(_line_offsets), std_dev(_std_dev),
  correlation_factor(_correlation_factor) {}

  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    // The first two elements in the pose array are the X,Y coordinates of the
    // robot.
    const Eigen::Matrix<T, 2, 1> pose_translation(pose[0], pose[1]);
    // The third element in the pose array is the rotation angle of the
    // robot.
    const Eigen::Rotation2D<T> pose_rotation(pose[2]);
    std::vector<T> point_residuals(points.size(), T(0));
    OMP_PARALLEL_FOR
    for (size_t i = 0; i < points.size(); ++i) {
      // Transform point from robot frame to global frame.
      const Eigen::Matrix<T, 2, 1> point_global_coords =
      (pose_rotation * points[i].cast<T>()) + pose_translation;
      const Eigen::Matrix<T, 2, 1> line_p1_t = line_p1s[i].cast<T>();
      const Eigen::Matrix<T, 2, 1> line_p2_t = line_p2s[i].cast<T>();
      if ((line_p1_t - point_global_coords).dot(
          line_p2_t - point_global_coords) > 0.0) {
        // The point projected onto the line falls outside the line segment.
        continue;
      }
      // Type cast the line normal from a float type to "T" type.
      const Eigen::Matrix<T, 2, 1> line_normal_t = line_normals[i].cast<T>();
      // The signed distance d of a point p from a line is given by the equation
      // d = p.dot(line_normal) + line_offset .
      const T off_line_error =
      point_global_coords.dot(line_normal_t) + T(line_offsets[i]);
      const T pose_line_error =
      pose_translation.dot(line_normal_t) + T(line_offsets[i]);
      if (pose_line_error * off_line_error > T(0.0)) {
        // Observations are on the same side of the line as the robot.
        continue;
      } else {
        // Observations are on opposide sides of the line as the robot: the robot
        // is seeing through walls.
        // Scale the error by the correlation factor to discount correlations
        // between points.
        point_residuals[i] =
            sq(off_line_error * T(correlation_factor) / T(std_dev));
      }
    }
    residuals[0] = std::accumulate(
        point_residuals.begin(), point_residuals.end(), T(0));
    if (residuals[0] > T(0)) {
      residuals[0] = sqrt(residuals[0]);
    }
    /*
    CHECK_GT(residuals[0], T(-FLT_MIN));
    if (!ceres::IsFinite(sqrt(residuals[0]))) {
      std::cout << "ERROR res :" << residuals[0]
                << " sqrt(res) :" << sqrt(residuals[0]) << "\n";
    }
    residuals[0] = sqrt(residuals[0]);
    */
    CHECK(ceres::IsFinite(residuals[0]));
    return true;
  }

  // The pose index from which this point was observed.
  const std::size_t pose_index;
  // The coordinates of the point in the robot local coordinate frame.
  const std::vector<Eigen::Vector2f> points;
  // The normal to the line (in global coordinates) of the map that this point
  // corresponds to.
  const std::vector<Eigen::Vector2f> line_normals;
  // Endpoint 1 of the line segment.
  const std::vector<Eigen::Vector2f> line_p1s;
  // Endpoint 2 of the line segment.
  const std::vector<Eigen::Vector2f> line_p2s;
  // The offset (shortest distance from origin) of the line (in global
  // coordinates) of the map that this point corresponds to.
  const std::vector<float> line_offsets;
  // Standard deviation of observations.
  float std_dev;
  // Discounting constant to account for correlations between observed points.
  const float correlation_factor;
};

struct PointToLineRelativeConstraint {
  PointToLineRelativeConstraint(int _pose_index,
                            const std::vector<Eigen::Vector2f>& _points,
                            const std::vector<Eigen::Vector2f>& _line_normals,
                            const std::vector<float>& _line_offsets,
                            const std::vector<bool>& _valid,
                            float _std_dev,
                            float _correlation_factor)
      : pose_index(_pose_index),
      points(_points), line_normals(_line_normals), line_offsets(_line_offsets),
      correspondence_valid(_valid), std_dev(_std_dev),
      correlation_factor(_correlation_factor) {
    CHECK_EQ(correspondence_valid.size(), points.size());
  }
  template <typename T>
  bool operator()(T const* const* relative_pose_array, T* residuals) const {
    T pose[3] = {
        relative_pose_array[0][0],
        relative_pose_array[0][1],
        relative_pose_array[0][2]
    };
    for (size_t i = 1; i <= pose_index; ++i) {
      pose[0] = pose[0] + relative_pose_array[i][0];
      pose[1] = pose[1] + relative_pose_array[i][1];
      pose[2] = pose[2] + relative_pose_array[i][2];
    }
    T& residual = residuals[0];
    residual = T(0.0);
    // The first two elements in the pose array are the X,Y coordinates of the
    // robot.
    const Eigen::Matrix<T, 2, 1> pose_translation(pose[0], pose[1]);
    // The third element in the pose array is the rotation angle of the
    // robot.
    const Eigen::Matrix<T, 2, 2> pose_rotation =
        Eigen::Rotation2D<T>(pose[2]).toRotationMatrix();
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (!correspondence_valid[i]) continue;
      // Transform point from robot frame to global frame.
      const Eigen::Matrix<T, 2, 1> point_global_coords =
          (pose_rotation * points[i].cast<T>()) + pose_translation;
      // Type cast the line normal from a float type to "T" type.
      const Eigen::Matrix<T, 2, 1> line_normal_t = line_normals[i].cast<T>();
      // The signed distance d of a point p from a line is given by the equation
      // d = p.dot(line_normal) + line_offset .
      const T off_line_error =
          point_global_coords.dot(line_normal_t) + T(line_offsets[i]);
      // Scale the error by the correlation factor to discount correlations
      // between points.
      residual = residual +
          sq(off_line_error * T(correlation_factor) / T(std_dev));
    }
    if (residual != T(0.0)) {
      residual = ceres::sqrt(residual);
    }
    CHECK_NE(residuals[0], T(1e302));
    return true;
  }

  // Index number of the pose that this point was observed from.
  const size_t pose_index;
  // The coordinates of the point in the robot local coordinate frame.
  const std::vector<Eigen::Vector2f>& points;
  // The normal to the line (in global coordinates) of the map that this point
  // corresponds to.
  const std::vector<Eigen::Vector2f> line_normals;
  // The offset (shortest distance from origin) of the lines(in global
  // coordinates) of the map that the points correspond to.
  const std::vector<float> line_offsets;
  // Binary variable that indicates whether a valid corresponding line was
  // found for the corresponding point.
  const std::vector<bool> correspondence_valid;
  // Standard deviation of observations.
  const float std_dev;
  // Discounting constant to account for correlations between observed points.
  const float correlation_factor;
};

struct PointToLineConstraint {
  PointToLineConstraint(int _pose_index,
                        int _point_index,
                        const Eigen::Vector2f& _point,
                        const Eigen::Vector2f& _line_normal,
                        float _line_offset,
                        bool _valid,
                        float _std_dev,
                        float _correlation_factor)
      : pose_index(_pose_index), point_index(_point_index),
      point(_point), line_normal(_line_normal), line_offset(_line_offset),
      valid(_valid), std_dev(_std_dev),
      correlation_factor(_correlation_factor) {}

  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    // If the point to line constraint is not valid, set a constant value to
    // the residual. The Jacobian thus computed will not have any contribution
    // from this residual since it is independent of the parameter block.
    if (!valid) {
      residuals[0] = T(0.0);
      return true;
    }

    // The first two elements in the pose array are the X,Y coordinates of the
    // robot.
    const Eigen::Matrix<T, 2, 1> pose_translation(pose[0], pose[1]);
    // The third element in the pose array is the rotation angle of the
    // robot.
    const Eigen::Rotation2D<T> pose_rotation(pose[2]);
    // Transform point from robot frame to global frame.
    const Eigen::Matrix<T, 2, 1> point_global_coords =
        (pose_rotation * point.cast<T>()) + pose_translation;
    // Type cast the line normal from a float type to "T" type.
    const Eigen::Matrix<T, 2, 1> line_normal_t = line_normal.cast<T>();
    // The signed distance d of a point p from a line is given by the equation
    // d = p.dot(line_normal) + line_offset .
    const T off_line_error =
        point_global_coords.dot(line_normal_t) + T(line_offset);
    // Scale the error by the correlation factor to discount correlations
    // between points.
    residuals[0] = off_line_error * T(correlation_factor) / T(std_dev);
    return true;
  }

  // Index number of the pose that this point was observed from.
  const int pose_index;
  // Index number of the point within the point cloud observed from this pose
  // that this point to line constraint corresponds to.
  const int point_index;
  // The coordinates of the point in the robot local coordinate frame.
  const Eigen::Vector2f point;
  // The normal to the line (in global coordinates) of the map that this point
  // corresponds to.
  Eigen::Vector2f line_normal;
  // The offset (shortest distance from origin) of the line (in global
  // coordinates) of the map that this point corresponds to.
  float line_offset;
  // Binary variable that indicates whether a valid corresponding line was
  // found for this point.
  bool valid;
  // Standard deviation of observations.
  float std_dev;
  // Discounting constant to account for correlations between observed points.
  const float correlation_factor;
};

struct PointToObjectConstraint {
  PointToObjectConstraint(int _pose_index,
                          int _point_index,
                          const Eigen::Vector2f& _point,
                          const Eigen::Vector2f& _line_normal,
                          float _line_offset,
                          float _std_dev,
                          float _correlation_factor)
      : pose_index(_pose_index), point_index(_point_index),
      point(_point), line_normal(_line_normal), line_offset(_line_offset),
      std_dev(_std_dev), correlation_factor(_correlation_factor) {}

  template <typename T>
  bool operator()(const T* const robot_pose, const T* const object_pose,
                  T* residuals) const {
    // The first two elements in the pose array are the X,Y coordinates of the
    // robot.
    const Eigen::Matrix<T, 2, 1> robot_translation(
        robot_pose[0], robot_pose[1]);
    // Location of the object instance in the global frame.
    const Eigen::Matrix<T, 2, 1> object_translation(
        object_pose[0], object_pose[1]);
    // The third element in the pose array is the rotation angle of the
    // robot.
    const Eigen::Rotation2D<T> robot_rotation(robot_pose[2]);
    // Rotation to the object's local frame.
    const Eigen::Rotation2D<T> object_rotation(-object_pose[2]);
    // Transform point from robot frame to global frame.
    const Eigen::Matrix<T, 2, 1> point_global_coords =
        (robot_rotation * point.cast<T>()) + robot_translation;
    // Type cast the line normal from a float type to "T" type.
    const Eigen::Matrix<T, 2, 1> line_normal_t = line_normal.cast<T>();

    const Eigen::Matrix<T, 2, 1> point_local =
        object_rotation * (point_global_coords - object_translation);
    // The signed distance d of a point p from a line is given by the equation
    // d = p.dot(line_normal) + line_offset .
    const T off_line_error =
        point_local.dot(line_normal_t) + T(line_offset);
    // Scale the error by the correlation factor to discount correlations
    // between points.
    residuals[0] = off_line_error * T(correlation_factor) / T(std_dev);
    // residuals[0] = -sin(object_pose[2] - T(DEG(30.0)));
    // residuals[0] = T(0.0);
    return true;
  }

  // Index number of the pose that this point was observed from.
  const int pose_index;
  // Index number of the point within the point cloud observed from this pose
  // that this point to line constraint corresponds to.
  const int point_index;
  // The coordinates of the point in the robot local coordinate frame.
  const Eigen::Vector2f point;
  // The normal to the line (in global coordinates) of the map that this point
  // corresponds to.
  Eigen::Vector2f line_normal;
  // The offset (shortest distance from origin) of the line (in global
  // coordinates) of the map that this point corresponds to.
  float line_offset;
  // Standard deviation of observations.
  float std_dev;
  // Discounting constant to account for correlations between observed points.
  const float correlation_factor;
};

struct PointToPointConstraint {
  PointToPointConstraint(int _pose_index,
                         int _point_index,
                         const Eigen::Vector2f& _point,
                         const Eigen::Vector2f& _neighbor_point,
                         const Eigen::Vector2f& _neighbor_normal,
                         bool _valid,
                         float _std_dev,
                         float _correlation_factor)
    : pose_index(_pose_index), point_index(_point_index),
    point(_point), neighbor_point(_neighbor_point),
    neighbor_normal(_neighbor_normal),
    valid(_valid), std_dev(_std_dev), correlation_factor(_correlation_factor) {}

  template <typename T>
  bool operator()(const T* const pose,
                  const T* const neighbor_pose,
                  T* residuals) const {
    // If the point to line constraint is not valid, set a constant value to
    // the residual. The Jacobian thus computed will not have any contribution
    // from this residual since it is independent of the parameter block.
    if (!valid) {
      residuals[0] = T(0.0);
      return true;
    }

    // The first two elements in the pose array are the X,Y coordinates of the
    // robot.
    const Eigen::Matrix<T, 2, 1> pose_translation(pose[0], pose[1]);
    // The third element in the pose array is the rotation angle of the
    // robot.
    const Eigen::Rotation2D<T> pose_rotation(pose[2]);
    // Transform point from robot frame to global frame.
    Eigen::Matrix<T, 2, 1> point_global =
        (pose_rotation * point.cast<T>()) + pose_translation;

    // The first two elements in the pose array are the X,Y coordinates of the
    // robot.
    const Eigen::Matrix<T, 2, 1> neighbor_pose_translation(
      neighbor_pose[0], neighbor_pose[1]);
    // The third element in the pose array is the rotation angle of the
    // robot.
    const Eigen::Rotation2D<T> neighbor_pose_rotation(neighbor_pose[2]);
    // Transform neighbor point from robot frame to global frame.
    Eigen::Matrix<T, 2, 1> neighbor_point_global =
        (neighbor_pose_rotation * neighbor_point.cast<T>()) +
        neighbor_pose_translation;
    // Transform neighbor normal from robot frame to global frame.
    Eigen::Matrix<T, 2, 1> neighbor_normal_global =
        neighbor_pose_rotation * neighbor_normal.cast<T>();
    residuals[0] = neighbor_normal_global.dot(
        point_global - neighbor_point_global) * T(correlation_factor) /
        T(std_dev);
    return true;
  }

  // Index number of the pose that this point was observed from.
  const int pose_index;
  // Index number of the point within the point cloud observed from this pose
  // that this point to line constraint corresponds to.
  const int point_index;
  // The coordinates of the point in the robot local coordinate frame.
  const Eigen::Vector2f point;
  // The neighboring point in a different pose that this point corresponds to.
  Eigen::Vector2f neighbor_point;
  // The normal associated with the neighboring point that this point
  // corresponds to.
  Eigen::Vector2f neighbor_normal;
  // Binary variable that indicates whether a valid corresponding line was
  // found for this point.
  bool valid;
  // Standard deviation of observations.
  float std_dev;
  // Discounting constant to account for correlations between observed points.
  const float correlation_factor;
};


struct PointToPointGlobConstraint {
  PointToPointGlobConstraint(size_t _pose_index0,
                             size_t _pose_index1,
                             const std::vector<Eigen::Vector2f>& _points0,
                             const std::vector<Eigen::Vector2f>& _points1,
                             const std::vector<Eigen::Vector2f>& _normals0,
                             const std::vector<Eigen::Vector2f>& _normals1,
                             float _std_dev,
                             float _correlation_factor)
    : pose_index0(_pose_index0), pose_index1(_pose_index1),
    points0(_points0), points1(_points1),
    normals0(_normals0), normals1(_normals1),
    std_dev(_std_dev), correlation_factor(_correlation_factor) {
      CHECK_EQ(points0.size(), points1.size());
      CHECK_EQ(normals0.size(), normals1.size());
      CHECK_GT(points0.size(), 0);
    }

  template <typename T>
  bool operator()(const T* const pose0,
                  const T* const pose1,
                  T* residuals) const {
    const Eigen::Matrix<T, 2, 1> pose0_translation(pose0[0], pose0[1]);
    const Eigen::Matrix<T, 2, 1> pose1_translation(pose1[0], pose1[1]);
    const Eigen::Matrix<T, 2, 2> pose0_rotation =
        Eigen::Rotation2D<T>(pose0[2]).toRotationMatrix();
    const Eigen::Matrix<T, 2, 2> pose1_rotation =
        Eigen::Rotation2D<T>(pose1[2]).toRotationMatrix();

    T& residual0 = residuals[0];
    T& residual1 = residuals[1];
    std::vector<T> point_residuals0(points0.size());
    std::vector<T> point_residuals1(points0.size());
    OMP_PARALLEL_FOR
    for (size_t i = 0; i < points0.size(); ++i) {
      const Eigen::Matrix<T, 2, 1> p0_global =
          (pose0_rotation * points0[i].cast<T>()) + pose0_translation;
      const Eigen::Matrix<T, 2, 1> p1_global =
          (pose1_rotation * points1[i].cast<T>()) + pose1_translation;
      const Eigen::Matrix<T, 2, 1> n0_global =
          pose0_rotation * normals0[i].cast<T>();
      const Eigen::Matrix<T, 2, 1> n1_global =
          pose1_rotation * normals1[i].cast<T>();
      const Eigen::Matrix<T, 2, 1> delta_p = p1_global - p0_global;
      point_residuals0[i] =
          sq(T(n0_global.dot(delta_p)) * T(correlation_factor) / T(std_dev));
      point_residuals1[i] =
          sq(T(n1_global.dot(delta_p)) * T(correlation_factor) / T(std_dev));
    }
    residual0 = std::accumulate(
        point_residuals0.begin(), point_residuals0.end(), T(0));
    residual1 = std::accumulate(
      point_residuals1.begin(), point_residuals1.end(), T(0));

    if (residual0 != T(0.0)) {
      residual0 = ceres::sqrt(residual0 / static_cast<T>(points0.size()));
    }
    if (residual1 != T(0.0)) {
      residual1 = ceres::sqrt(residual1 / static_cast<T>(points0.size()));
    }

    return true;
  }

  // Index number of the source pose that these points were observed from.
  const int pose_index0;
  // Index number of the target pose that these points are related to.
  const int pose_index1;
  // The coordinates of the point in the pose0 local coordinate frame.
  const std::vector<Eigen::Vector2f> points0;
  // The coordinates of the point in the pose1 local coordinate frame.
  const std::vector<Eigen::Vector2f> points1;
  // The coordinates of the normal in the pose0 local coordinate frame.
  const std::vector<Eigen::Vector2f> normals0;
  // The coordinates of the normal in the pose1 local coordinate frame.
  const std::vector<Eigen::Vector2f> normals1;
  // Standard deviation of observations.
  float std_dev;
  // Discounting constant to account for correlations between observed points.
  const float correlation_factor;
};

struct PointToPointRelativeConstraint {
  PointToPointRelativeConstraint(
      size_t _pose_index0, size_t _pose_index1,
      const std::vector<Eigen::Vector2f>& _points0,
      const std::vector<Eigen::Vector2f>& _points1,
      const std::vector<Eigen::Vector2f>& _normals0,
      const std::vector<Eigen::Vector2f>& _normals1,
      float _std_dev, float _correlation_factor)
      : pose_index0(_pose_index0), pose_index1(_pose_index1),
      points0(_points0), points1(_points1),
      normals0(_normals0), normals1(_normals1),
      std_dev(_std_dev), correlation_factor(_correlation_factor) {
    CHECK_EQ(points0.size(), points1.size());
    CHECK_EQ(normals0.size(), normals1.size());
  }

  template <typename T>
  bool operator()(T const* const* relative_pose_array, T* residuals) const {
    T pose0[3] = {
        relative_pose_array[0][0],
        relative_pose_array[0][1],
        relative_pose_array[0][2]
    };
    T pose1[3] = {
        relative_pose_array[0][0],
        relative_pose_array[0][1],
        relative_pose_array[0][2]
    };
    for (size_t i = 1; i <= pose_index0; ++i) {
      pose0[0] = pose0[0] + relative_pose_array[i][0];
      pose0[1] = pose0[1] + relative_pose_array[i][1];
      pose0[2] = pose0[2] + relative_pose_array[i][2];
    }
    for (size_t i = 1; i <= pose_index1; ++i) {
      pose1[0] = pose1[0] + relative_pose_array[i][0];
      pose1[1] = pose1[1] + relative_pose_array[i][1];
      pose1[2] = pose1[2] + relative_pose_array[i][2];
    }
    const Eigen::Matrix<T, 2, 1> pose0_translation(pose0[0], pose0[1]);
    const Eigen::Matrix<T, 2, 1> pose1_translation(pose1[0], pose1[1]);
    const Eigen::Matrix<T, 2, 2> pose0_rotation =
        Eigen::Rotation2D<T>(pose0[2]).toRotationMatrix();
    const Eigen::Matrix<T, 2, 2> pose1_rotation =
        Eigen::Rotation2D<T>(pose1[2]).toRotationMatrix();

    T& residual0 = residuals[0];
    T& residual1 = residuals[1];
    residual0 = T(0.0);
    residual1 = T(0.0);
    for (size_t i = 0; i < points0.size(); ++i) {
      const Eigen::Matrix<T, 2, 1> p0_global =
          (pose0_rotation * points0[i].cast<T>()) + pose0_translation;
      const Eigen::Matrix<T, 2, 1> p1_global =
          (pose1_rotation * points1[i].cast<T>()) + pose1_translation;
      const Eigen::Matrix<T, 2, 1> n0_global =
          pose0_rotation * normals0[i].cast<T>();
      const Eigen::Matrix<T, 2, 1> n1_global =
          pose1_rotation * normals1[i].cast<T>();
      const Eigen::Matrix<T, 2, 1> delta_p = p1_global - p0_global;
      residual0 = residual0 +
          sq(n0_global.dot(delta_p) * T(correlation_factor) / T(std_dev));
      residual1 = residual1 +
          sq(n1_global.dot(delta_p) * T(correlation_factor) / T(std_dev));
    }
    if (residual0 != T(0.0)) {
      residual0 = ceres::sqrt(residual0);
    }
    if (residual1 != T(0.0)) {
      residual1 = ceres::sqrt(residual1);
    }
    CHECK_NE(residual0, T(1e302));
    CHECK_NE(residual1, T(1e302));
    return true;
  }

  // Index number of the source pose that these points were observed from.
  const size_t pose_index0;
  // Index number of the target pose that these points are related to.
  const size_t pose_index1;
  // The coordinates of the point in the pose0 local coordinate frame.
  const std::vector<Eigen::Vector2f> points0;
  // The coordinates of the point in the pose1 local coordinate frame.
  const std::vector<Eigen::Vector2f> points1;
  // The coordinates of the normal in the pose0 local coordinate frame.
  const std::vector<Eigen::Vector2f> normals0;
  // The coordinates of the normal in the pose1 local coordinate frame.
  const std::vector<Eigen::Vector2f> normals1;
  // Standard deviation of observations.
  float std_dev;
  // Discounting constant to account for correlations between observed points.
  const float correlation_factor;
};


struct RelativePoseConstraint {
  RelativePoseConstraint(
      size_t _pose_index0, size_t _pose_index1,
      const Eigen::Matrix2f& _axis_transform,
      float _radial_std_dev,
      float _tangential_std_dev,
      float _angular_std_dev,
      float _radial_translation,
      float _rotation) :
      pose_index0(_pose_index0),
      pose_index1(_pose_index1),
      axis_transform(_axis_transform),
      radial_std_dev(_radial_std_dev),
      tangential_std_dev(_tangential_std_dev),
      angular_std_dev(_angular_std_dev),
      radial_translation(_radial_translation),
      rotation(_rotation) {
    CHECK(std::isfinite(radial_translation));
    CHECK(std::isfinite(rotation));
    CHECK(std::isfinite(_axis_transform(0,0)));
    CHECK(std::isfinite(_axis_transform(0,1)));
    CHECK(std::isfinite(_axis_transform(1,0)));
    CHECK(std::isfinite(_axis_transform(1,1)));
    CHECK(std::isfinite(radial_std_dev));
    CHECK(std::isfinite(tangential_std_dev));
    CHECK(std::isfinite(angular_std_dev));
    CHECK_GT(radial_std_dev, 0.0);
    CHECK_GT(tangential_std_dev, 0.0);
    CHECK_GT(angular_std_dev, 0.0);
  }

  template <typename T>
  bool operator()(T const* const* relative_pose_array, T* residuals) const {
    T pose0[3] = {
        relative_pose_array[0][0],
        relative_pose_array[0][1],
        relative_pose_array[0][2]
    };
    T pose1[3] = {
        relative_pose_array[0][0],
        relative_pose_array[0][1],
        relative_pose_array[0][2]
    };
    for (size_t i = 1; i <= pose_index0; ++i) {
      pose0[0] = pose0[0] + relative_pose_array[i][0];
      pose0[1] = pose0[1] + relative_pose_array[i][1];
      pose0[2] = pose0[2] + relative_pose_array[i][2];
    }
    for (size_t i = 1; i <= pose_index1; ++i) {
      pose1[0] = pose1[0] + relative_pose_array[i][0];
      pose1[1] = pose1[1] + relative_pose_array[i][1];
      pose1[2] = pose1[2] + relative_pose_array[i][2];
    }
    // Compute translation difference between pose2 and pose1.
    Eigen::Matrix<T, 2, 1> translation(
        pose1[0] - pose0[0], pose1[1] - pose0[1]);
    // Transform relative pose to the reference frame of pose1.
    const Eigen::Rotation2D<T> pose_rotation(-pose0[2]);
    translation = pose_rotation * translation;
    // Get the matrix that transforms to the principal axes of the odometry
    // covariance.
    const Eigen::Matrix<T, 2, 2> axis_transform_t = axis_transform.cast<T>();
    translation = axis_transform_t * translation;
    residuals[0] =
        (translation.x() - T(radial_translation)) / T(radial_std_dev);
    residuals[1] =
        translation.y() / T(tangential_std_dev);
    T rotation_error = (pose1[2] - pose0[2] - T(rotation));
    if (false) {
      residuals[2] = T(0.0);
    } else {
      // CHECK_LT(angular_error, T(M_PI));
      // CHECK_GT(angular_error, T(-M_PI));
      residuals[2] = rotation_error / T(angular_std_dev);
    }
    CHECK_NE(residuals[0], T(1e302));
    CHECK_NE(residuals[1], T(1e302));
    CHECK_NE(residuals[2], T(1e302));
    return true;
  }

  // Index number of the source pose.
  const size_t pose_index0;
  // Index number of the target pose.
  const size_t pose_index1;
  const Eigen::Matrix2f axis_transform;
  const float radial_std_dev;
  const float tangential_std_dev;
  const float angular_std_dev;
  const float radial_translation;
  const float rotation;
};

struct AnchorConstraint {
  AnchorConstraint(float _x, float _y, float _std_dev) :
      x(_x), y(_y), std_dev(_std_dev) {}

  template <typename T>
  bool operator()(const T* const pose,
                  T* residuals) const {
    residuals[0] = (pose[0] - T(x)) / T(std_dev);
    residuals[1] = (pose[1] - T(y)) / T(std_dev);
    return true;
  }

  const float x;
  const float y;
  const float std_dev;
};

struct PoseConstraint {
  PoseConstraint(const Eigen::Matrix2f& _axis_transform,
                 float _radial_std_dev,
                 float _tangential_std_dev,
                 float _angular_std_dev,
                 float _radial_translation,
                 float _rotation) :
      axis_transform(_axis_transform),
      radial_std_dev(_radial_std_dev),
      tangential_std_dev(_tangential_std_dev),
      angular_std_dev(_angular_std_dev),
      radial_translation(_radial_translation),
      rotation(_rotation) {
    CHECK(std::isfinite(radial_translation));
    CHECK(std::isfinite(rotation));
    CHECK(std::isfinite(_axis_transform(0,0)));
    CHECK(std::isfinite(_axis_transform(0,1)));
    CHECK(std::isfinite(_axis_transform(1,0)));
    CHECK(std::isfinite(_axis_transform(1,1)));
    CHECK(std::isfinite(radial_std_dev));
    CHECK(std::isfinite(tangential_std_dev));
    CHECK(std::isfinite(angular_std_dev));
    CHECK_GT(radial_std_dev, 0.0);
    CHECK_GT(tangential_std_dev, 0.0);
    CHECK_GT(angular_std_dev, 0.0);
  }

  template <typename T>
  bool operator()(const T* const pose1,
                  const T* const pose2,
                  T* residuals) const {
    // Compute translation difference between pose2 and pose1.
    Eigen::Matrix<T, 2, 1> translation(pose2[0]-pose1[0], pose2[1]-pose1[1]);
    // Transform relative pose to the reference frame of pose1.
    const Eigen::Rotation2D<T> pose_rotation(-pose1[2]);
    translation = pose_rotation * translation;
    // covariance.
    const Eigen::Matrix<T, 2, 2> axis_transform_t = axis_transform.cast<T>();
    translation = axis_transform_t * translation;
    residuals[0] = (translation.x() - T(radial_translation)) / T(radial_std_dev);
    residuals[1] =  translation.y() / T(tangential_std_dev);
    //T rotation_error = (pose2[2] - pose1[2] - T(rotation));

    T angular_diff = pose2[2] - pose1[2];

    const T angular_error = ceres::atan2(sin(pose2[2] - pose1[2] - T(rotation)), 
                                         cos(pose2[2] - pose1[2] - T(rotation)));

//     if (angular_diff > T(M_PI)) {
//       std::cout << "BIG ANG ERROR" << std::endl;
//       angular_diff -= T(2.0 * M_PI);
//       std::cout << "using atan2: " << angular_error << std::endl;
//     }
//     else if (angular_diff < T(-M_PI)) {
//       std::cout << "BIG ANG ERROR" << std::endl;
//       angular_diff += T(2.0 * M_PI);
//       std::cout << "using atan2: " << angular_error << std::endl;
//     }
    
    
// //     const T angular_error = angular_diff - T(rotation);
//     
// //     const T angular_error = pose2[2] - pose1[2] - T(rotation);
// 
//     //     double ang_error_test = atan2(sin(pose2[2] - pose1[2] - double(rotation)), 
// // 				  cos(pose2[2] - pose1[2] - double(rotation)));
// //     if (ang_error_test > 1.0) { std::cout << "BIG ERROR: " << ang_error_test << std::endl; }
    residuals[2] = angular_error / T(angular_std_dev);
/*    residuals[0] = angular_error / T(angular_std_dev);*/

    return true;
  }

  const Eigen::Matrix2f axis_transform;
  const float radial_std_dev;
  const float tangential_std_dev;
  const float angular_std_dev;
  const float radial_translation;
  const float rotation;
};

struct CumulativePoseConstraint {
  CumulativePoseConstraint(float _radial_std_dev,
                           float _tangential_std_dev,
                           float _angular_std_dev,
                           float _radial_translation,
                           float _tangential_translation,
                           float _rotation) :
      radial_std_dev(_radial_std_dev),
      tangential_std_dev(_tangential_std_dev),
      angular_std_dev(_angular_std_dev),
      radial_translation(_radial_translation),
      tangential_translation(_tangential_translation),
      rotation(_rotation) {
    CHECK(std::isfinite(radial_std_dev));
    CHECK(std::isfinite(tangential_std_dev));
    CHECK(std::isfinite(angular_std_dev));
    CHECK_GT(radial_std_dev, 0.0);
    CHECK_GT(tangential_std_dev, 0.0);
    CHECK_GT(angular_std_dev, 0.0);
  }

  template <typename T>
  bool operator()(const T* const relative_pose,
                  T* residuals) const {
    residuals[0] =
        (relative_pose[0] - T(radial_translation)) / T(radial_std_dev);
    residuals[1] =
        (relative_pose[1] - T(tangential_translation)) / T(tangential_std_dev);
    residuals[2] =
        (relative_pose[2] - T(rotation)) / T(angular_std_dev);
    return true;
  }

  const float radial_std_dev;
  const float tangential_std_dev;
  const float angular_std_dev;

  const float radial_translation;
  const float tangential_translation;
  const float rotation;
};

struct CumulativePointToLineConstraint {
  CumulativePointToLineConstraint(int _pose_index,
                                  int _point_index,
                                  const Eigen::Vector2f& _point,
                                  const Eigen::Vector2f& _line_normal,
                                  float _line_offset,
                                  bool _valid,
                                  float _correlation_factor) :
      pose_index(_pose_index), point_index(_point_index),
      point(_point), line_normal(_line_normal), line_offset(_line_offset),
      valid(_valid), correlation_factor(_correlation_factor) {}

  template <typename T>
  void GetGlobalPose(const T* const relative_poses,
                     T* global_pose) {
    global_pose[0] = relative_poses[0];
    global_pose[1] = relative_poses[1];
    global_pose[2] = relative_poses[2];
    for (unsigned int i = 3; i < pose_index * 3; ++i) {
      const T &dx = relative_poses[i + 0];
      const T &dy = relative_poses[i + 1];
      const T &dr = relative_poses[i + 2];
      const T c = cos(global_pose[2]);
      const T s = sin(global_pose[2]);
      global_pose[0] = global_pose[0] + c * dx - s * dy;
      global_pose[1] = global_pose[1] + s * dx + c * dy;
      global_pose[2] = global_pose[2] + relative_poses[i + 2];
    }
  }

  template <typename T>
  bool operator()(const T* const relative_poses, T* residuals) const {
    // If the point to line constraint is not valid, set a constant value to
    // the residual. The Jacobian thus computed will not have any contribution
    // from this residual since it is independent of the parameter block.
    if (!valid) {
      residuals[0] = T(0.0);
      return true;
    }
    // Compute the global pose by summing up the relative poses.
    T global_pose[3] = { T(0.0), T(0.0), T(0.0) };
    GetGlobalPose(relative_poses, global_pose);

    // The first two elements in the pose array are the X,Y coordinates of the
    // robot.
    const Eigen::Matrix<T, 2, 1> pose_translation(
        global_pose[0], global_pose[1]);
    // The third element in the pose array is the rotation angle of the
    // robot.
    const Eigen::Rotation2D<T> pose_rotation(global_pose[2]);
    // Transform point from robot frame to global frame.
    const Eigen::Matrix<T, 2, 1> point_global_coords =
        (pose_rotation * point.cast<T>()) + pose_translation;
    // Type cast the line normal from a float type to "T" type.
    const Eigen::Matrix<T, 2, 1> line_normal_t = line_normal.cast<T>();
    // The signed distance d of a point p from a line is given by the equation
    // d = p.dot(line_normal) + line_offset .
    const T off_line_error =
        point_global_coords.dot(line_normal_t) + T(line_offset);
    // Scale the error by the correlation factor to discount correlations
    // between points.
    residuals[0] = off_line_error * T(correlation_factor);
    return true;
  }

  void UpdateCorrespondence(const Eigen::Vector2f& _line_normal,
                            float _line_offset,
                            bool _valid) {
    line_normal = _line_normal;
    line_offset = _line_offset;
    valid = _valid;
  }

  // Index number of the pose that this point was observed from.
  const unsigned int pose_index;
  // Index number of the point within the point cloud observed from this pose
  // that this point to line constraint corresponds to.
  const unsigned int point_index;
  // The coordinates of the point in the robot local coordinate frame.
  const Eigen::Vector2f point;
  // The normal to the line (in global coordinates) of the map that this point
  // corresponds to.
  Eigen::Vector2f line_normal;
  // The offset (shortest distance from origin) of the line (in global
  // coordinates) of the map that this point corresponds to.
  float line_offset;
  // Binary variable that indicates whether a valid corresponding line was
  // found for this point.
  bool valid;
  // Discounting constant to account for correlations between observed points.
  const float correlation_factor;
};


struct TestConstraint {
  TestConstraint() {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    const T K1 = static_cast<T>(50.0);

    T line_align_x = K1*((parameters[0]-parameters[2])*
                         (parameters[0]-parameters[2]));
    T line_align_y = K1*((parameters[1]-parameters[3])*
                         (parameters[1]-parameters[3]));

    residuals[0] = line_align_x;
    residuals[1] = line_align_y;

    return true;
  }
};










struct ColocationHumanImposedConstraint {
  ColocationHumanImposedConstraint(
    const double x_target,
    const double y_target,
    const double t_target) : x_target_(x_target), y_target_(y_target), t_target_(t_target) {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    const T Ks = static_cast<T>(1.0);
    //const T Kep = static_cast<T>(1000.0);
    const T Ka = static_cast<T>(1.0);
    
    T fitx = T(0.0);
    T fity = T(0.0);
    T fitt = T(0.0);
    
    fitx = x_target_ - parameters[0];
    fity = y_target_ - parameters[1];
    fitt = t_target_ - parameters[2];
    
    residuals[0] = Ks * fitx;
    residuals[1] = Ks * fity;
    residuals[2] = Ka * fitt;

    return true;
  }

  const double x_target_;
  const double y_target_;
  const double t_target_;
};

struct ColinearHumanImposedConstraint {
  ColinearHumanImposedConstraint(
    const double x_target,
    const double y_target,
    const double t_target,
    const double penalty_dir) : 
    
    x_target_(x_target),
    y_target_(y_target),
    t_target_(t_target),
    penalty_dir_(penalty_dir) {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    const T Ks = static_cast<T>(1.0);
    //const T Kep = static_cast<T>(1000.0);
    const T Ka = static_cast<T>(1.0);
    
    T x_pen_dir = T(cos(penalty_dir_));
    T y_pen_dir = T(sin(penalty_dir_));
    
    T fitxy = T(0.0);
    //T fity = T(0.0);
    T fitt = T(0.0);
    
    fitxy = x_pen_dir * (x_target_ - parameters[0]) + y_pen_dir * (y_target_ - parameters[1]);
    //fity = y_target_ - parameters[1];
    fitt = t_target_ - parameters[2];
    
    residuals[0] = Ks * fitxy;
    //residuals[1] = Ks * fity;
    residuals[1] = Ka * fitt;

    return true;
  }

  const double x_target_;
  const double y_target_;
  const double t_target_;
  const double penalty_dir_;
};

struct PerpendicularHumanImposedConstraint {
  PerpendicularHumanImposedConstraint(
    const double t_target) : t_target_(t_target) {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    const T Ka = static_cast<T>(1.0);
    //const T Kep = static_cast<T>(1000.0);

    T fitt = T(0.0);
    
    fitt = t_target_ - parameters[2];
    
    residuals[2] = Ka * fitt;

    return true;
  }

  const double t_target_;
};

struct ParallelHumanImposedConstraint {
  ParallelHumanImposedConstraint(
    const double t_target) : t_target_(t_target) {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    const T Ka = static_cast<T>(1.0);
    //const T Kep = static_cast<T>(1000.0);

    T fitt = T(0.0);
    
    fitt = t_target_ - parameters[2];
    
    residuals[2] = Ka * fitt;

    return true;
  }

  const double t_target_;
};

}  // namespace

//}
// namespace vector_localization

#endif  // RESIDUAL_FUNCTORS_H
