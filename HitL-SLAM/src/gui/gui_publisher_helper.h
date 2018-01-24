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
// Copyright 2013 joydeepb@ri.cmu.edu
// Robotics Institute, Carnegie Mellon University
//
// Helper Functions to draw primitives on the CoBot localization GUI.

#include <eigen3/Eigen/Dense>
#include <vector>
#include "vector_slam_msgs/LidarDisplayMsg.h"

#ifndef GUI_PUBLISHER_HELPER_H
#define GUI_PUBLISHER_HELPER_H

namespace cobot_gui {

// Some commonly used colors.
const uint32_t kColorRed = 0xFFFF0000;
const uint32_t kColorGreen = 0xFF00FF00;
const uint32_t kColorBlue = 0xFF0000FF;
const uint32_t kColorYellow = 0xFFFFFF00;
const uint32_t kColorBlack = 0xFF000000;
const uint32_t kColorWhite = 0xFFFFFFFF;

template <typename VectorType>
void DrawCircle(const VectorType& p, uint32_t color,
                vector_slam_msgs::LidarDisplayMsg* display_msg) {
  display_msg->circles_x.push_back(p.x);
  display_msg->circles_y.push_back(p.y);
  display_msg->circles_col.push_back(color);
}

template <typename VectorType>
void DrawLine(const VectorType& p0, const VectorType& p1, uint32_t color,
              vector_slam_msgs::LidarDisplayMsg* display_msg) {
  display_msg->lines_p1x.push_back(p0.x);
  display_msg->lines_p1y.push_back(p0.y);
  display_msg->lines_p2x.push_back(p1.x);
  display_msg->lines_p2y.push_back(p1.y);
  display_msg->lines_col.push_back(color);
}

template <typename ScalarType>
void DrawLine(const Eigen::Matrix<ScalarType, 2, 1>& p0,
              const Eigen::Matrix<ScalarType, 2, 1>& p1, uint32_t color,
              vector_slam_msgs::LidarDisplayMsg* display_msg) {
  display_msg->lines_p1x.push_back(p0.x());
  display_msg->lines_p1y.push_back(p0.y());
  display_msg->lines_p2x.push_back(p1.x());
  display_msg->lines_p2y.push_back(p1.y());
  display_msg->lines_col.push_back(color);
}

template <typename VectorType>
void DrawPoint(const VectorType& p, uint32_t color,
               vector_slam_msgs::LidarDisplayMsg* display_msg) {
  display_msg->points_x.push_back(p.x);
  display_msg->points_y.push_back(p.y);
  display_msg->points_col.push_back(color);
}

template <typename ScalarType>
void DrawPoint(const Eigen::Matrix<ScalarType, 2, 1>& p,
               uint32_t color,
               vector_slam_msgs::LidarDisplayMsg* display_msg) {
  display_msg->points_x.push_back(p.x());
  display_msg->points_y.push_back(p.y());
  display_msg->points_col.push_back(color);
}

template <typename VectorType>
void DrawText(const VectorType& loc,
              const std::string& text,
              uint32_t color,
              float text_height,
              bool text_in_window_coords,
              vector_slam_msgs::LidarDisplayMsg* display_msg) {
  display_msg->text_x.push_back(loc.x);
  display_msg->text_y.push_back(loc.y);
  display_msg->text_col.push_back(color);
  display_msg->text.push_back(text);
  display_msg->text_height.push_back(text_height);
  display_msg->text_in_window_coords.push_back(text_in_window_coords);
}

template <typename ScalarType>
void DrawText(const Eigen::Matrix<ScalarType, 2, 1>& loc,
              const std::string& text,
              uint32_t color,
              float text_height,
              bool text_in_window_coords,
              vector_slam_msgs::LidarDisplayMsg* display_msg) {
  display_msg->text_x.push_back(loc.x());
  display_msg->text_y.push_back(loc.y());
  display_msg->text_col.push_back(color);
  display_msg->text.push_back(text);
  display_msg->text_height.push_back(text_height);
  display_msg->text_in_window_coords.push_back(text_in_window_coords);
}

void ClearDrawingMessage(vector_slam_msgs::LidarDisplayMsg* display_msg) {
  display_msg->lines_p1x.clear();
  display_msg->lines_p1y.clear();
  display_msg->lines_p2x.clear();
  display_msg->lines_p2y.clear();
  display_msg->points_x.clear();
  display_msg->points_y.clear();
  display_msg->lines_col.clear();
  display_msg->points_col.clear();
  display_msg->circles_x.clear();
  display_msg->circles_y.clear();
  display_msg->circles_col.clear();
  display_msg->text.clear();
  display_msg->text_x.clear();
  display_msg->text_y.clear();
  display_msg->text_col.clear();
  display_msg->text_in_window_coords.clear();
  display_msg->windowSize = 1.0;
}






//TODO: fix below functions for general use




/*
void DrawGradients(const size_t start_pose, const size_t end_pose,
                   const vector<double>& gradients, uint32_t color) {

  cout << "drawing gradients" << endl;
  if (gradients.size() < poses.size()) {
    cout << "more poses than gradients" << endl;
    return;
  }
  cout << "start_pose: " << start_pose << endl;
  cout << "end_pose: " << end_pose << endl;
  for (size_t i = start_pose; i < end_pose; ++i) {
    const Vector2f location_gradient(gradients[3*i], gradients[3*i + 1]);
    const Vector2f pose_location = poses[i].translation;
    //const Rotation2Df pose_rotation(poses[j + 2]);
    // gradient DESCENT, so negatize the gradient to get direction of travel
    const Vector2f p2 = pose_location - location_gradient;
    //cout << (pose_location - p2).norm() << endl;
    DrawLine(pose_location, p2, color, &display_message_);
  }
}

void DrawPoseCovariance3D(const Vector2f& pose,
                          const Matrix2f covariance,
                          const float theta_var) {
  static const float kDTheta = RAD(15.0);
  Eigen::SelfAdjointEigenSolver<Matrix2f> solver;
  solver.compute(covariance);
  //cout << "cov mat" << covariance << endl;
  const Matrix2f eigenvectors = solver.eigenvectors();
  const Vector2f eigenvalues = solver.eigenvalues();
  float theta_color = (theta_var / max_theta_var)*255;
  uint32_t color = 0xFF808000 + int(theta_color);
  //cout << "theta_color" << theta_color << endl;
  //cout << "EV1 " << eigenvalues(0) << " EV2 " << eigenvalues(1) << endl;
  for (float a = 0; a < 2.0 * M_PI; a += kDTheta) {
    const Vector2f v1(cos(a) * sqrt(eigenvalues(0)),
                      sin(a) * sqrt(eigenvalues(1)));
    const Vector2f v2(cos(a + kDTheta) * sqrt(eigenvalues(0)),
                      sin(a + kDTheta) * sqrt(eigenvalues(1)));
    const Vector2f v1_global = eigenvectors.transpose() * v1 + pose;
    const Vector2f v2_global = eigenvectors.transpose() * v2 + pose;

    DrawLine(v1_global, v2_global, color, &display_message_);
    //DrawLine(v1_global, v2_global, kPoseCovarianceColor, &display_message_);
  }
}

void DrawStfs(
    const vector<VectorMapping::PointToPointGlobCorrespondence>&
        point_point_correspondences,
    const vector<double>& poses,
    const vector<vector<vector2f>>& point_clouds,
    const vector<NormalCloudf>& normal_clouds) {
  static const bool kDrawPoints = false;
  for (size_t i = 0; i < point_point_correspondences.size(); ++i) {
    const int pose_index0 = point_point_correspondences[i].pose_index0;
    const int pose_index1 = point_point_correspondences[i].pose_index1;
    DCHECK_EQ(point_point_correspondences[i].points0.size(),
              point_point_correspondences[i].points1.size());
    const Affine2f pose_tf0 =
        Translation2f(poses[3 * pose_index0 + 0], poses[3 * pose_index0 + 1]) *
        Rotation2Df(poses[3 * pose_index0 + 2]);
    const Affine2f pose_tf1 =
        Translation2f(poses[3 * pose_index1 + 0], poses[3 * pose_index1 + 1]) *
        Rotation2Df(poses[3 * pose_index1 + 2]);
    for (size_t j = 0; j < point_point_correspondences[i].points0.size(); ++j) {
      const Vector2f p0 = pose_tf0 * point_point_correspondences[i].points0[j];
      const Vector2f p1 = pose_tf1 * point_point_correspondences[i].points1[j];
      if (kDrawPoints) {
        DrawPoint(p0, cobot_gui::kColorBlue, &display_message_);
        DrawPoint(p1, cobot_gui::kColorBlue, &display_message_);
      }
      DrawLine(p0, p1, kStfCorrespondenceColor, &display_message_);
    }
  }
}

void DrawObservations(
    const size_t start_pose, const size_t end_pose,
    const vector<double>& poses,
    const vector<vector< vector2f> >& point_clouds,
    const std::vector< NormalCloudf >& normal_clouds) {
  static const bool kDisplayTangents = false;
  for (size_t i = start_pose; i <= end_pose; ++i) {
    const vector<vector2f> &point_cloud = point_clouds[i];
    const vector<Vector2f> &normal_cloud = normal_clouds[i];
    const vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
    const float pose_angle = poses[3 * i + 2];
    const Rotation2Df pose_rotation(pose_angle);
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      const vector2f& point = point_cloud[j].rotate(pose_angle) + pose_location;
      if (kDisplayTangents) {
        const Vector2f normal_e = pose_rotation * normal_cloud[j];
        const vector2f normal(normal_e.x(), normal_e.y());
        const vector2f tangent = 0.05 * normal.perp();
        DrawLine(point + tangent,
                 point - tangent,
                 kStfPointColor,
                 &display_message_);
      }
      DrawPoint(point, kStfPointColor, &display_message_);
    }
  }
}
*/

/*
void DrawPoseCovariance(const Vector2f& pose, const Matrix2f& covariance) {
  static const float kDTheta = RAD(15.0);
  Eigen::SelfAdjointEigenSolver<Matrix2f> solver;
  solver.compute(covariance);
  const Matrix2f eigenvectors = solver.eigenvectors();
  const Vector2f eigenvalues = solver.eigenvalues();
  for (float a = 0; a < 2.0 * M_PI; a += kDTheta) {
    const Vector2f v1(cos(a) * sqrt(eigenvalues(0)),
                      sin(a) * sqrt(eigenvalues(1)));
    const Vector2f v2(cos(a + kDTheta) * sqrt(eigenvalues(0)),
                      sin(a + kDTheta) * sqrt(eigenvalues(1)));
    const Vector2f v1_global = eigenvectors.transpose() * v1 + pose;
    const Vector2f v2_global = eigenvectors.transpose() * v2 + pose;
    DrawLine(v1_global, v2_global, kPoseCovarianceColor, &display_message_);
  }
}
*/

/*
void DrawPoses(const size_t start_pose, const size_t end_pose,
    const vector<Pose2Df>& odometry_poses, const vector<double>& poses,
    const vector<Matrix2f>& covariances) {
  static const bool kDrawCovariances = false;
  Vector2f pose_location_last(0.0, 0.0);
  float pose_angle_last = 0.0;
  const bool valid_covariances = (covariances.size() > end_pose);
*/ 

 //double L = 0.5;
  //double l = 0.1;

 // for (size_t i = start_pose; i <= end_pose; ++i) {
 //   const Vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
/*
    //draw arrows for poses here
    Vector2f arr(L*cos(poses[3*i+2]), L*sin(poses[3*i+2]));
    arr += pose_location;
    Vector2f w1(l*cos(poses[3*i+2] + 5.0*3.14159/6.0), l*sin(poses[3*i+2] +
5.0*3.14159/6.0));
    Vector2f w2(l*cos(poses[3*i+2] - 5.0*3.14159/6.0), l*sin(poses[3*i+2] -
5.0*3.14159/6.0));
    Vector2f W1 = w1 + arr;
    Vector2f W2 = w2 + arr;

    DrawLine(pose_location, arr, kOdometryColor, &display_message_);
    DrawLine(W1, arr, kOdometryColor, &display_message_);
    DrawLine(W2, arr, kOdometryColor, &display_message_);
    */
/*
   if (i > start_pose) {
      DrawLine(pose_location, pose_location_last, kTrajectoryColor,
               &display_message_);
      const Vector2f odometry =
          Rotation2Df(pose_angle_last) *
          Rotation2Df(-odometry_poses[i - 1].angle) *
          (odometry_poses[i].translation - odometry_poses[i - 1].translation);
      DrawLine(pose_location, Vector2f(pose_location_last + odometry),
               kOdometryColor, &display_message_);
      if (kDrawCovariances && valid_covariances) {
        DrawPoseCovariance(pose_location, covariances[i]);
      }
    }
    pose_location_last = pose_location;
    pose_angle_last = poses[3 * i + 2];
  }
}
*/




}  // namespace cobot_gui

#endif  // GUI_PUBLISHER_HELPER_H
