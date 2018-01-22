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
// Main entry point for non-Markov localization.

#include <stdint.h>
#include <stdio.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pthread.h>
#include <queue>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <termios.h>
#include <time.h>
#include <utility>
#include <vector>

#include "eigen_helper.h"
#include "helpers.h"
//#include "../vmapping/vector_mapping.h"
#include "../../extern_libraries/CImg/CImg.h"
#include "glog/logging.h"
#include "ceres/ceres.h"
#include "cobot_msgs/CobotEventsMsg.h"
#include "cobot_msgs/CobotLocalizationMsg.h"
#include "cobot_msgs/CobotOdometryMsg.h"
#include "cobot_msgs/GuiKeyboardEvent.h"
#include "cobot_msgs/GuiMouseMoveEvent.h"
#include "cobot_msgs/GuiMouseClickEvent.h"
#include "cobot_msgs/LidarDisplayMsg.h"
#include "cobot_msgs/LocalizationGuiCaptureSrv.h"
#include "cobot_msgs/LocalizationMsg.h"
#include "cobot_msgs/CobotLocalizationSrv.h"
#include "../gui/gui_publisher_helper.h"
#include "nav_msgs/Odometry.h"
#include "../perception_tools/perception_2d.h"
#include "popt_pp.h"
#include "proghelp.h"
#include "pthread_utils.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "../shared/math/geometry.h"
#include "../shared/util/configreader.h"
#include "sensor_msgs/LaserScan.h"
#include "timer.h"
#include "util.h"
//#include "../map/vector_map.h"
#include "residual_functors.h"

#include "EMinput.h"
#include "ApplyExplicitCorrection.h"
#include "Backprop.h"
#include "JointOptimization.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using cobot_gui::ClearDrawingMessage;
using cobot_gui::DrawCircle;
using cobot_gui::DrawLine;
using cobot_gui::DrawPoint;
using cobot_gui::DrawText;
using vector_slam_msgs::CobotEventsMsg;
using vector_slam_msgs::CobotLocalizationSrv;
using Eigen::Affine2d;
using Eigen::Affine2f;
using Eigen::Matrix2d;
using Eigen::Matrix2f;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Perp2;
using Eigen::Rotation2Dd;
using Eigen::Rotation2Df;
using Eigen::ScalarCross;
using Eigen::Translation2d;
using Eigen::Translation2f;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector3d;
using Eigen::DistanceToLineSegment;
using perception_2d::GenerateNormals;
using perception_2d::NormalCloudf;
using perception_2d::PointCloudf;
using perception_2d::Pose2Df;
using ros::ServiceServer;
using ros::Subscriber;
using std::cout;
using std::endl;
using std::pair;
using std::queue;
using std::size_t;
using std::sort;
using std::string;
using std::vector;


using vector_localization::EMInput;
using vector_localization::AppExpCorrect;
using vector_localization::Backprop;
using vector_localization::JointOpt;





vector<float> prev_ceres_cost;

// gradients from ceres solver
std::vector<double> ceres_gradients_;

// sparse jacobian representation from ceres
ceres::CRSMatrix ceres_jacobian_;
bool jacobian_init_ = false;

// number of residuals due to human constraints
int num_hc_residuals_;

// For visualization
float max_theta_var = 0.0;

vector<double> message_timestamps_;

// pose estimates
vector<Pose2Df> poses;

// original relative odometry
//vector<perception_2d::Pose2Df> orig_odom_;

// in case human wants to undo last constraint
vector<Pose2Df> prev_poses;
int added_human_constraints = 0;

// point clouds in robot frame
vector<PointCloudf> init_point_clouds;
vector<NormalCloudf> normal_clouds;

//covariances
vector<Matrix3d> covariances_;
vector<Matrix3d> prev_covariances_;

// human constraints
vector<vector<VectorMapping::HumanConstraint>> human_constraints_;

// points selected by user during HitL
vector<Vector2f> selected_points_;
CorrectionType correction_type_ = CorrectionType::kUnknownCorrection;
CorrectionType pending_correction_type_ = CorrectionType::kUnknownCorrection;
bool loop_corrections_on_ = false;

// Name of the topic on Cobot's software stack that laser data is published on.
static const string kCobotLaserTopic("/Cobot/Laser");
// Name of the topic on Cobot's software stack that Kinect scan data is
// published on.
static const string kKinectScanTopic("/Cobot/Kinect/Scan");
// Name of the topic in a standardized data bag files that Kinect scan data is
// published on.
static const string kStandardKinectScanTopic("kinect_scan");
// Name of the topic on Cobot's software stack that odometry is published on.
static const string kCobotOdometryTopic("/Cobot/Odometry");
// Name of the topic in a standardized data bag files that laser data is
// published on.
static const string kStandardLaserTopic("laser");
// Name of the topic in a standardized data bag files that odometry is
// published on.
static const string kStandardOdometryTopic("odom");
// Name of the topic in a standardized data bag files that location reset
// commands are published on.
static const string kStandardSetLocationTopic("set_location");
// Name of the map to localize the robot on.
const string kMapName("EmptyMap");
// Robot's starting location.
Vector2f kStartingLocation = Vector2f(0.0, 0.0);
// Robot's starting angle.
float kStartingAngle = 0.0;
// Uncertainty of translation in the direction of travel.
float kRadialTranslationUncertainty = 0.05;
// Uncertainty of translation perpendicular to the direction of travel.
float kTangentialTranslationUncertainty = 0.05;
// Uncertainty of rotation in radians after moving 1 radian.
float kAngleUncertainty = 0.05;
// Scaling constant to correct for error in angular odometry.
float kOdometryRotationScale = 1.0;
// Scaling constant to correct for error in translation odometry.
float kOdometryTranslationScale = 1.0;
// Minimum distance of observed points from the robot.
float kMinPointCloudRange = 0.2;
// Maximum distance of observed points from the robot.
float kMaxPointCloudRange = 6.0;
// Maximum distance between adjacent points to use for computation of normals.
float kMaxNormalPointDistance = 0.03;
// Indicates whether the bag file being read is a standardized data bag file.
bool kStandardizedData = false;
// Angular margin from the scan area boundary to ignore laser readings.
float kAngularMargin = 0.0;

char* bag_file = NULL;

char* stfs_file = NULL;

char* log_file = NULL;

bool lost_poses = true;

int correction_number = 0;

// Mutex to ensure only a single relocalization call is made at a time.
pthread_mutex_t relocalization_mutex_ = PTHREAD_MUTEX_INITIALIZER;

// The path of the cobot_linux ROS stack.
static const string kCobotStackPath(ros::package::getPath("cobot_linux"));

// The directory where all the maps are stored.
static const string kMapsDirectory(kCobotStackPath + "/../maps");

// Index of test set. This will determine which file the results of the test are
// saved to.
int test_set_index_ = -1;

// Indicates that a statistical test is being run, and to save the localization
// results to the file with the specified index.
int statistical_test_index_ = -1;

// The fraction of additive odometry noise for statistical tests.
double odometry_additive_noise_ = 0.05;

//static const uint32_t kTrajectoryColor = 0x6FFF0000;
static const uint32_t kTrajectoryColor = 0x7F000000;
static const uint32_t kPoseCovarianceColor = 0xFF808080;
static const uint32_t kOdometryColor = 0x70FF0000;
// static const uint32_t kTrajectoryColor = 0xFFC0C0C0;
static const uint32_t kLtfCorrespondenceColor = 0x7FFF7700;
static const uint32_t kLtfPointColor = 0xFFFF7700;
static const uint32_t kStfPointColor = 0xFFFF5500;
static const uint32_t kStfCorrespondenceColor = 0x7F994CD9;
static const uint32_t kDfPointColor  = 0x7F37B30C;
static const uint32_t kObjectColor = 0xFF56C4C3;

bool run_ = true;
int debug_level_ = -1;

// Indicates that /Cobot/Kinect/Scan should be used instead of
// /Cobot/Laser, to localize using the Kinect sensor instead of the laser
// rangefinder.
bool use_kinect_ = false;

// Indicates that the user will provide an estimate of an object instance via
// the localization_gui.
bool get_object_instance_ = false;

// Display message for drawing debug vizualizations on the localization_gui.
vector_slam_msgs::LidarDisplayMsg display_message_;

// Service client to ask localization_gui to save images.
ros::ServiceClient gui_capture_client;

// ROS publisher to publish display_message_ to the ROS topic
// Cobot/VectorLocalization/Gui
ros::Publisher display_publisher_;

// ROS publisher to the CoBot events topic.
ros::Publisher events_publisher_;

// ROS publisher to publish the latest robot localization estimate to
// Cobot/Localization
ros::Publisher localization_publisher_;

// ROS subscriber to the localization gui mouse move events
ros::Subscriber mouse_move_subscriber_;

// ROS subscriber to the localization gui mouse click events
ros::Subscriber mouse_click_subscriber_;

// ROS subscriber to /Cobot/VectorLocalization/GuiKeyboardEvents
ros::Subscriber keyboard_event_subscriber_;

// Parameters and settings for Non-Markov Localization.
VectorMapping::VectorMappingOptions localization_options_;

// Parameters for learning HitL SLAM


// Main class instance for Non-Markov Localization.
VectorMapping localization_(kMapsDirectory);


EMInput em_input_;

AppExpCorrect app_exp_corr_;

Backprop backprop_;

JointOpt joint_opt_;



// Relocalization interface.
//vector_localization::Relocalization relocalization_(kMapsDirectory);

// The last observed laser scan, used for auto localization.
sensor_msgs::LaserScan last_laser_scan_;

// Parameters used for relocalization.
//VectorLocalization2D::LidarParams relocalization_lidar_params_;

// File name of the final image of all observations from all poses with their
// classifications.
char* save_image_file = NULL;

// Directory where images of every episode at every timestep will be saved.
char* episode_images_path = NULL;

// Determines whether STFS will be saved for later object mapping or not.
bool save_stfs_ = false;

// WatchFiles to track changes to config files.
WatchFiles watch_files_;

// Config reader for localization options.
ConfigReader config_((kCobotStackPath + "/").c_str());

// Corrections to angle-dependent errors of the laser scanner.
vector<float> laser_corrections_;

// Resolution of the laser corrections lookup table.
float laser_corrections_resolution_ = 0.0;

// Boolean flag to indicate use of laser scanner corrections.
bool use_laser_corrections_ = false;

// Flag to write LTF observations to disk for error correction computation.
bool save_ltfs_ = false;

























































void DrawImageLine(const vector2f& p0, const vector2f& p1,
              const vector2f& origin, const float& scale,
              const uint8_t* color,
              cimg_library::CImg<uint8_t>* image) {
  static const int kThickness = 2;
  vector2f p0_transformed = (p0 - origin) / scale;
  vector2f p1_transformed = (p1 - origin) / scale;
  const int height = image->height();
  for (int dx = -kThickness; dx < kThickness; ++dx) {
    for (int dy = -kThickness; dy < kThickness; ++dy) {
      image->draw_line(p0_transformed.x + dx,
                       height - 1 - p0_transformed.y + dy,
                       p1_transformed.x + dx,
                       height - 1 - p1_transformed.y + dy,
                       color);
    }
  }
}

Eigen::MatrixXd ConvertJ(ceres::CRSMatrix& jacobian) {

  int num_rows = jacobian.num_rows;

  cout << "ceres jacobian rows: " << num_rows << endl;
  // Convert the sparse matrix to a dense matrix
  Eigen::MatrixXd denseJacobian = Eigen::MatrixXd::Zero(num_rows, jacobian.num_cols);
  if (num_rows > 0) {
    for(size_t k = 0; k < jacobian.rows.size() - 1; ++k) {
      size_t row = jacobian.rows[k];
      size_t nextRow = jacobian.rows[k + 1];
      for(size_t l = row; l < nextRow; ++l) {
        int column = jacobian.cols[l];
        double value = jacobian.values[l];
          //cout << row << " " << column << " " << value << endl;
          denseJacobian(k,column) = value;
      }
    }
  }
  //denseJacobian = Eigen::MatrixXd(jacobianMatrix);
  // Calculate j.transpose j
  //Eigen::MatrixXd jTj = denseJacobian.transpose() * denseJacobian;

  return denseJacobian;
}


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

void DrawPoses(const size_t start_pose, const size_t end_pose,
    const vector<Pose2Df>& odometry_poses, const vector<double>& poses,
    const vector<Matrix2f>& covariances) {
  static const bool kDrawCovariances = false;
  Vector2f pose_location_last(0.0, 0.0);
  float pose_angle_last = 0.0;
  const bool valid_covariances = (covariances.size() > end_pose);
  //double L = 0.5;
  //double l = 0.1;

  for (size_t i = start_pose; i <= end_pose; ++i) {
    const Vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
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























void DisplayPoses(const vector<Pose2Df>& poses,
                  const vector<PointCloudf>& point_clouds,
                  const vector<NormalCloudf>& normal_clouds,
                  const char* save_image_file) {
//   if (debug_level_ < 0) return;
  
  ClearDrawingMessage(&display_message_);
  
  static const int kSkipLaserScans = 1;
  CHECK_EQ(poses.size(), point_clouds.size());
  CHECK_GT(poses.size(), 0);

     
  //float max_cost = 1.0;  
  /*
  float max_cost = 0.0;
  vector<float> cc = localization_.GetCeresCost();
  if (cc.size() >= poses.size()) {
    for (size_t i = 0; i < cc.size();  ++i) {
        max_cost = max(cc[i], max_cost);
    }
  }
  else {
    max_cost = 1.0;
  }
  
  uint32_t cost_color;
  if (cc.size() >= poses.size()) {
    cost_color = int((cc[i]/max_cost)*255.0);
  }
  else {
    cost_color = 0x80;
  }

  //cout << "cost color: " << cost_color << endl;
  cost_color = 0xFF800080 + cost_color*16*16;
*/
 
  
  if (covariances_.size() >= poses.size()) {
    for (size_t i = 0; i < poses.size();  ++i) {
      max_theta_var = max(float(covariances_[i](2,2)), max_theta_var);
    }
  }
  else {
     max_theta_var = 1.0;
  }

  
  // Add robot localization trajectory.
  for (size_t i = 0; i < poses.size() - 1; ++i) {
    // color / draw covariances
    if (covariances_.size() > i) {
      Eigen::Matrix2f single_cov;
      single_cov(0,0) = float(covariances_[i](0,0));
      single_cov(0,1) = float(covariances_[i](0,1));
      single_cov(1,0) = float(covariances_[i](1,0));
      single_cov(1,1) = float(covariances_[i](1,1));
      float theta_var = covariances_[i](2,2);
      DrawPoseCovariance3D(poses[i].translation, single_cov, theta_var);
    }
    // draw trajectory
    DrawLine(poses[i].translation, poses[i + 1].translation, kTrajectoryColor, &display_message_);

    //draw arrows for poses here
    double L = 0.5;
    double l = 0.1;
    Vector2f arr(L*cos(poses[i].angle), L*sin(poses[i].angle));

    arr += poses[i].translation;
    Vector2f w1(l*cos(poses[i].angle + 5.0*3.14159/6.0),
                l*sin(poses[i].angle + 5.0*M_PI/6.0));
    Vector2f w2(l*cos(poses[i].angle - 5.0*3.14159/6.0),
                l*sin(poses[i].angle - 5.0*3.14159/6.0));
    Vector2f W1 = w1 + arr;
    Vector2f W2 = w2 + arr;
    // Assign color based on if the pose is part of ANY human constraint
    bool hc = false;
    for (size_t j = 0; j < human_constraints_.size(); ++j) {
      for (size_t k = 0; k < human_constraints_[j].size(); ++k) {
        if (human_constraints_[j][k].anchor_pose_id == int(i) ||
            human_constraints_[j][k].constrained_pose_id == int(i)) {
          hc = true;
        }
      }
    }
    if (hc) {
      DrawLine(poses[i].translation, arr, 0xFF8080FF, &display_message_);
      DrawLine(W1, arr, 0xFF8080FF, &display_message_);
      DrawLine(W2, arr, 0xFF8080FF, &display_message_);
    }
    else {
      DrawLine(poses[i].translation, arr, 0xFF000000, &display_message_);
      DrawLine(W1, arr, 0xFF000000, &display_message_);
      DrawLine(W2, arr, 0xFF000000, &display_message_);
    }
  }
  Eigen::MatrixXd jacobian_;
//   Eigen::MatrixXd jacobian_ = ConvertJ(ceres_jacobian_);
  // Draw all the gradients, and color them by residual origin
//   if (jacobian_init_ && jacobian_.rows() > (poses.size()-1)*3 -1 && false) { // break up all types (odom, hc, stfs)
//     cout << "rows: " << jacobian_.rows() << endl;
//     cout << "cols: " << jacobian_.cols() << endl;
if (false) {
    
    CHECK_GT(jacobian_.rows(), (poses.size()-1)*3 -1);
    //cout << "jacobian rows inside draw: " << jacobian.rows() << endl;
    uint32_t odom_color = 0xFF0000FF;
    uint32_t hc_color = 0xFF00FF00;
    uint32_t stfs_color = 0xFFFF00FF;
    int num_cols = jacobian_.cols();
    int num_odom_rows = (poses.size()-1)*3;
    int num_hc_rows = num_hc_residuals_;
    int num_stf_rows = jacobian_.rows() - (num_odom_rows + num_hc_rows);
    CHECK_EQ(jacobian_.rows(), num_odom_rows + num_hc_rows + num_stf_rows);
    
    vector<double> col_sums;
    // Odometry
    for (int i = 0; i < num_cols; ++i) {
      double col_sum = 0.0;
      for (int j = 0; j < num_odom_rows; ++j) { //just the odometry
        col_sum += jacobian_(j,i);
      }
      col_sums.push_back(col_sum);
    }
    // draw the gradients
    for (size_t i = 0; i < poses.size(); ++i) {
      Vector2f jacobian_dir(col_sums[3 * i], col_sums[3 * i + 1]);
      const Vector2f pose_location = poses[i].translation;
      const Vector2f p2 = pose_location - jacobian_dir;
      DrawLine(pose_location, p2, odom_color, &display_message_);
    }
    col_sums.clear();
    std::cout << "odom viz" << std::endl;
    // human constraints
    for (int i = 0; i < num_cols; ++i) {
      double col_sum = 0.0;
      for (int j = num_odom_rows; j < (num_hc_rows + num_odom_rows); ++j) { //just the human constraints
        col_sum += jacobian_(j,i);
      }
      col_sums.push_back(col_sum);
    }
    // draw the gradients
    for (size_t i = 0; i < poses.size(); ++i) {
      Vector2f jacobian_dir(col_sums[3 * i], col_sums[3 * i + 1]);
      const Vector2f pose_location = poses[i].translation;
      const Vector2f p2 = pose_location - jacobian_dir;
      DrawLine(pose_location, p2, hc_color, &display_message_);
    }
    col_sums.clear();
    std::cout << "odom viz" << std::endl;
    // short term features
    for (int i = 0; i < num_cols; ++i) {
      double col_sum = 0.0;
      for (int j = (num_hc_rows + num_odom_rows); j < jacobian_.rows(); ++j) { //just the stfs
        col_sum += jacobian_(j,i);
      }
      col_sums.push_back(col_sum);
    }
    // draw the gradients
    for (size_t i = 0; i < poses.size(); ++i) {
      Vector2f jacobian_dir(col_sums[3 * i], col_sums[3 * i + 1]);
      const Vector2f pose_location = poses[i].translation;
      const Vector2f p2 = pose_location - jacobian_dir;
      DrawLine(pose_location, p2, stfs_color, &display_message_);
    }
    std::cout << "passed viz" << std::endl;
  }
  else if (ceres_gradients_.size() > 0 && false) { // default condition. draw sum of all gradients
    uint32_t color = 0xFFFF0000;
    DrawGradients(0, poses.size()-1, ceres_gradients_, color);
  }

  
  
//   human_constraints_.clear();
//   ScopedFile numfid("numHC.txt", "r");
//   ScopedFile hcfid("HumanConstraints.txt", "r");
//   if (numfid != NULL && hcfid != NULL) {
//     LoadHumanConstraints();
//   }
// 
//   if (human_constraints_.size() > 2) {
// 
//   // Convert laser scans to WORLD FRAME and add them to display message.
//   //for (int i = 0; i < 2; ++i) {
//   for (size_t i = 0; i < human_constraints_[0].pose_obs_ids_a.size(); ++i) {
//     const int pose_idx = human_constraints_[0].pose_obs_ids_a[i].first;
//     const PointCloudf& point_cloud = point_clouds[pose_idx];
//     const Rotation2Df rotation(poses[pose_idx].angle);
//     const Vector2f pose_location = poses[pose_idx].translation;
//     uint32_t color;
//     if (i == 0) {
//       color = 0xFFFF0000;
//     }
//     else {
//       color = 0xFFFF7700;
//     }
//     //uint32_t color = kLtfPointColor;
//     for (size_t j=0; j<human_constraints_[0].pose_obs_ids_a[i].second.size(); ++j ) {
//       const int obs_idx = human_constraints_[0].pose_obs_ids_a[i].second[j];
//       const Vector2f p(rotation * point_cloud[obs_idx] + pose_location);
//  
//       DrawPoint(p, color, &display_message_);
//     }
//   }
//   //for (int i = 0; i < 2; ++i) {
//   for (size_t i = 0; i < human_constraints_[0].pose_obs_ids_b.size(); ++i) {
//     const int pose_idx = human_constraints_[0].pose_obs_ids_b[i].first;
//     const PointCloudf& point_cloud = point_clouds[pose_idx];
//     const Rotation2Df rotation(poses[pose_idx].angle);
//     const Vector2f pose_location = poses[pose_idx].translation;
//     uint32_t color;
//     if (i == 0) {
//       color = 0xFF0000FF;
//     }
//     else {
//       color = 0xFF00FFFF;
//     }
//     //uint32_t color = kLtfPointColor;
//     for (size_t j=0; j<human_constraints_[0].pose_obs_ids_b[i].second.size(); ++j ) {
//       const int obs_idx = human_constraints_[0].pose_obs_ids_b[i].second[j];
//       const Vector2f p(rotation * point_cloud[obs_idx] + pose_location);
//       DrawPoint(p, color, &display_message_);
//     }
//   }
//   }
//   else {

  // Convert laser scans to WORLD FRAME and add them to display message.
  for (size_t i = 0; i < point_clouds.size(); i += kSkipLaserScans) {
    const PointCloudf& point_cloud = point_clouds[i];
    const Rotation2Df rotation(poses[i].angle);
    // const Vector2f& pose_location = poses[i].translation;
    const Vector2f pose_location = poses[i].translation;
//     std::cout << "have poses in display" << std::endl;
    for (unsigned int j = 0; j < point_cloud.size(); ++j) {
//       if (j%200 == 0) { std::cout << "have points in display" << std::endl; }
      const Vector2f p(rotation * point_cloud[j] + pose_location);
      uint32_t color = kLtfPointColor;
      DrawPoint(p, color, &display_message_);
      // DrawLine(p, pose_location, 0x3Fc0c0c0, &display_message_);
    }
//   }
  }


  if (save_image_file != NULL) {
    const string filename = string(save_image_file) + ".bag";
    rosbag::Bag bag(filename, rosbag::bagmode::Write);
    bag.write("/Cobot/VectorLocalization/Gui",
              ros::Time::now(),
              display_message_);
    bag.close();
  }
  PublishDisplay();
}

































void CorrespondenceCallback(
    const vector<double>& poses,
    const vector<vector< vector2f> >& point_clouds,
    const vector< NormalCloudf >& normal_clouds,
    const vector<VectorMapping::PointToPointGlobCorrespondence>&
        point_point_correspondences,
    const vector<double>& gradients,
    const vector<Matrix2f>& covariances,
    const vector<Pose2Df>& odometry_poses,
    const size_t start_pose,
    const size_t end_pose) {
  static const bool kDisplayStfCorrespondences = true;
  uint32_t color = 0xFFFF0000;
  CHECK_EQ(poses.size(), point_clouds.size() * 3);
  ClearDrawingMessage(&display_message_);
  DrawPoses(start_pose, end_pose, odometry_poses, poses, covariances);
  DrawGradients(start_pose, end_pose, gradients, color);
  DrawObservations(start_pose, end_pose, poses, point_clouds, normal_clouds);
  if (kDisplayStfCorrespondences) {
    DrawStfs(point_point_correspondences, poses, point_clouds, normal_clouds);
  }
  display_publisher_.publish(display_message_);
}




pair<Vector2f, cimg_library::CImg<float>> ConstructSingleSDF(
                          perception_2d::PointCloudf point_cloud,
                                            const Vector2f source,
                                            const float pose_angle) {
  float image_resolution = 0.05; //meters
  float image_border = 0.0; //meters
  const float pixel_half_width = sqrt(2.0) * image_resolution;
  const float laser_angular_resolution = M_PI * (270.0 / 1024.0) / 180.0;

  float min_x(FLT_MAX), min_y(FLT_MAX);
  float max_x(-FLT_MAX), max_y(-FLT_MAX);
  for (size_t k = 0; k < point_cloud.size() ; ++k) {
    const Vector2f point = Rotation2Df(pose_angle) * point_cloud[k] + source;
    min_x = min(min_x, point[0]);
    max_x = max(max_x, point[0]);
    min_y = min(min_y, point[1]);
    max_y = max(max_y, point[1]);
  }

  const float width = max_x - min_x + 2.0 * image_border;
  const float height = max_y - min_y + 2.0 * image_border;
  const unsigned int image_width = ceil(width / image_resolution);
  const unsigned int image_height = ceil(height /image_resolution);

  cimg_library::CImg<float> sdf_weights_image(image_width, image_height);
  const Vector2f image_origin(min_x - image_border, min_y - image_border);

  float eps = 0.04; //meters

  OMP_PARALLEL_FOR
  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      sdf_weights_image(x, y) = 0.0;
    }
  }

  OMP_PARALLEL_FOR
  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      for (size_t k = 0; k < point_cloud.size(); ++k) {
        const Vector2f& point = Rotation2Df(pose_angle)*point_cloud[k] + source;
        Vector2f line_dir = (point - source);
        line_dir.normalize();
        const Vector2f line_perp = Perp2(line_dir);
        const Vector2f pixel_loc= image_origin+image_resolution*Vector2f(x,y);

        const bool within_angle_tolerance =
            (fabs(line_perp.dot(point-pixel_loc)) / (point - source).norm()
            < 0.5 * laser_angular_resolution);

        const bool along_viewing_ray =
            (fabs(line_perp.dot(pixel_loc - point)) < pixel_half_width);
        if (!along_viewing_ray && !within_angle_tolerance) continue;

        // Locations farther than the observed point along the viewing ray
        // from the source have
        // a negative SDF value since they are "occupied".
        const float sdf_value = line_dir.dot(point - pixel_loc);

        //only revise values in front of or near range reading
        if (sdf_value <= eps) {
          sdf_weights_image(x,y) += 1.0;
        }
      }
    }
  }

  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      if (sdf_weights_image(x, y) > 0.0) {
        sdf_weights_image(x,y) = 255.0;
      }
    }
  }

  pair<Vector2f, cimg_library::CImg<float>> sdf = std::make_pair(image_origin,
                                                            sdf_weights_image);
  return sdf;
}

cimg_library::CImg<float> NormalizeSDF(
                             pair<Vector2f, cimg_library::CImg<float>> SDF_a,
                             pair<Vector2f, cimg_library::CImg<float>> SDF_b) {

  cimg_library::CImg<float> normalized_sdf;

  //create new master SdfObject with proper size
  float O0x = SDF_a.first(0);
  float O0y = SDF_a.first(1);
  float O1x = SDF_b.first(0);
  float O1y = SDF_b.first(1);

  int h0 = SDF_a.second.height();
  int h1 = SDF_b.second.height();
  int w0 = SDF_a.second.width();
  int w1 = SDF_b.second.width();

  float res = 0.05; //meters
  float new_height_wc =
                     max(max(O0y+h0*res,O0y+h1*res),max(O1y+h0*res,O1y+h1*res));
  float new_width_wc =
                     max(max(O0x+w0*res,O0x+w1*res),max(O1x+w0*res,O1x+w1*res));
  Vector2f new_origin(min(O0x, O1x), min(O0y, O1y));

  const unsigned int image_height = ceil(fabs(new_height_wc-new_origin(1))/res);
  const unsigned int image_width = ceil(fabs(new_width_wc - new_origin(0))/res);

  int ix_master = round((O0x - new_origin(0))/res);
  int iy_master = round((O0y - new_origin(1))/res);

  int ix_add = round((O1x - new_origin(0))/res);
  int iy_add = round((O1y - new_origin(1))/res);

  cimg_library::CImg<float> sdf_weights_image(image_width, image_height);

  OMP_PARALLEL_FOR
  for (size_t x=0; x<image_width; x++) {
    for (size_t y=0; y<image_height; y++) {
      sdf_weights_image(x,y) = 0.0;
    }
  }

  OMP_PARALLEL_FOR
  for (int x=0; x<w0; x++) {
    for (int y=0; y<h0; y++) {
      sdf_weights_image(x+ix_master,y+iy_master) += SDF_a.second(x,y);
    }
  }

  OMP_PARALLEL_FOR
  for (int x=0; x<w1; x++) {
    for (int y=0; y<h1; y++) {
      sdf_weights_image(x+ix_add,y+iy_add) += SDF_b.second(x,y);
    }
  }
  sdf_weights_image = sdf_weights_image/2.0;
  return sdf_weights_image;
}

void AnimateCeres(vector<pair<Vector2f, Vector2f> > lines,
                  vector<Vector2f> points) {

  cout << "animating poses!!!!" << endl;
  ClearDrawingMessage(&display_message_);
  for (size_t i = 0; i < lines.size(); ++i) { //lines
    DrawLine(lines[i].first, lines[i].second, 0xFFFF00FF, &display_message_);
  }
  for (size_t i = 0; i < points.size(); ++i) { //points
    DrawPoint(points[i], kStfPointColor, &display_message_);
  }
  display_publisher_.publish(display_message_);
  //exit(1);
  Sleep(2);
}

void EvaluateConsistency2(vector<Pose2Df> poses,
                       std::vector<perception_2d::PointCloudf> point_clouds) {

  int EnMLHistoryLength = 10;

  float laser_range = 10.0;
  CHECK_EQ(poses.size()%3, 0);
  int num_poses = poses.size() / 3;

  cimg_library::CImg<float> inconsistency(num_poses, num_poses);
  OMP_PARALLEL_FOR
  for (int x = 0; x < num_poses; ++x) {
    for (int y = 0; y < num_poses; ++y) {
      inconsistency(x, y) = 0.0;
    }
  }

  vector<pair<Vector2f, cimg_library::CImg<float>>> all_sdfs;
  OMP_PARALLEL_FOR
  for (int i = 0; i < num_poses; i++) {
    const vector<Vector2f> point_cloud = point_clouds[i];
    const Vector2f source = poses[i].translation;
    const float pose_angle = poses[i].angle;
    pair<Vector2f, cimg_library::CImg<float>> sdf;
    sdf = ConstructSingleSDF(point_cloud, source, pose_angle);
    all_sdfs.push_back(sdf);
    if (i%20 == 0) {
      cout << "outer" << i << endl;
    }
  }
  cout << "PASSED" << endl;
  for (int i = 0; i < num_poses; i++) {
    //for (int j = i+1; j < num_poses; j++) {
    for (int j = i + EnMLHistoryLength - 1; j < num_poses; j++) {
      const Vector2f source_a = poses[i].translation;
      const Vector2f source_b = poses[j].translation;
      //cout << " " << endl;
      //if ((source_a - source_b).norm() < (2*laser_range - 4.0)) {
      if ((source_a - source_b).norm() < laser_range) {
        float pairwise_inconsistency = 0.0;
        cimg_library::CImg<float> merged;
        merged = NormalizeSDF(all_sdfs[i], all_sdfs[j]);
        for (int x = 0; x < merged.width(); ++x) {
          for (int y = 0; y < merged.height(); ++y) {
            if (merged(x,y) > 0.0 && merged(x,y) < 255.0)  {
              pairwise_inconsistency += 1.0;
            }
          }
        }
        inconsistency(i,j) = pairwise_inconsistency;
        inconsistency(j,i) = pairwise_inconsistency;
      }
    }
  }
  //TODO tag iteration
  //TODO: pick a better norm for OT hist
  //cout << "before finding max" << endl;
  float max_incon = 0.0;
  for (int x = 0; x < num_poses; ++x) {
    for (int y = 0; y < num_poses; ++y) {
      max_incon = max(max_incon, inconsistency(x, y));
    }
  }

  cout << "normalized" << endl;
  for (int x = 0; x < num_poses; ++x) {
    for (int y = 0; y < num_poses; ++y) {
      inconsistency(x, y) = (inconsistency(x,y) / max_incon) * 255.0;
      //cout << inconsistency(x, y) << endl;
    }
  }
  int bins[10];
  for (int i = 0; i < 10; ++i) {
    bins[i] = 0;
  }
  for (int x = 0; x < num_poses; ++x) {
    for (int y = 0; y < num_poses; ++y) {
      bins[int(inconsistency(x,y) / 25.5)] ++;
    }
  }

  for (int i = 0; i < 10; ++i) {
    cout << bins[i] << endl;
  }

  const string consistency_image_file =
      StringPrintf("consistency%d.png", correction_number);
  inconsistency.save_png(consistency_image_file.c_str());
  correction_number++;
}






















//TODO: modify replay for new architecture

void ReplayLog() {
  bool log_entry_found = false;
  if (current_replay_index < int(logged_input.size())) {
    while (!log_entry_found && current_replay_index<int(logged_input.size())) {
      if (!logged_input[current_replay_index].undone) {


        //TODO: move to seperate function
        std::vector<std::vector<Eigen::Vector2f>> world_frame_point_clouds;
        for (size_t i = 0; i < init_point_clouds.size(); ++i) {
          const Rotation2Df pose_rotation(poses[i].angle);
          const Affine2f pose_transform = Translation2f(poses[i].translation) * pose_rotation;
          std::vector<Eigen::Vector2f> single_point_cloud;
          for (unsigned int j = 0; j < init_point_clouds[i].size(); ++j) {
            const Vector2f p = pose_transform * init_point_clouds[i][j];
            single_point_cloud.push_back(p);
          }
          world_frame_point_clouds.push_back(single_point_cloud);
        }
      
      
      
      //NOTE: maybe use this for evaluation.... maybe
      //if (localization_options_.EvaluateConsistency2 != NULL) {
      //  localization_options_.EvaluateConsistency2(poses, init_point_clouds);
      //}

      
        log_entry_found = true;
        SingleInput current_sim = logged_input[current_replay_index];
      
        //bool debug = false;
        int init_hc_size = human_constraints_.size();
        prev_poses = poses;
        prev_covariances_ = covariances_;
        selected_points_ = current_sim.input_points;
        correction_type_ = current_sim.type_of_constraint;


//TODO: change these to call constructor with all relavent args instead of assigning one by one 
      
        em_input_.local_version_point_clouds_ = world_frame_point_clouds;
        em_input_.selected_points_ = selected_points_;
        em_input_.correction_type_ = correction_type_;
      
        em_input_.Run();
      
        std::vector<int> corrected_poses = em_input_.corrected_poses_;
        std::vector<int> anchor_poses = em_input_.anchor_poses_;
        std::pair<int, int> backprop_bounds = em_input_.backprop_bounds_;
        std::vector<Eigen::Vector2f> new_selected_points = em_input_.selected_points_;

	std::cout << "made it here" << std::endl;
	
//       ClearDrawingMessage(&display_message_);
//       DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
        if (backprop_bounds.first >= 0 && backprop_bounds.second >= 1) {
          app_exp_corr_.correction_type_ = correction_type_;
          app_exp_corr_.selected_points_ = new_selected_points;
          app_exp_corr_.corrected_poses_ = corrected_poses;
          app_exp_corr_.anchor_poses_ = anchor_poses;
          app_exp_corr_.poses_ = poses;
     
          app_exp_corr_.Run();

          poses = app_exp_corr_.poses_;
          Eigen::Vector3f correction = app_exp_corr_.correction_;
          std::vector<VectorMapping::HumanConstraint> new_human_constraints;
          new_human_constraints = app_exp_corr_.new_human_constraints_;
          human_constraints_.push_back(new_human_constraints);
      
  //       ClearDrawingMessage(&display_message_);
  //       DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
        //std::cout << "BP bounds: " << backprop_bounds.first << ", " << backprop_bounds.second << std::endl;
      
          backprop_.poses_ = poses;
          backprop_.correction_ = correction;
          backprop_.backprop_bounds_ = backprop_bounds;
          backprop_.d3_covariances_ = covariances_;
      
          backprop_.Run();
      
          covariances_ = backprop_.d3_covariances_;
          poses = backprop_.poses_;

  //       ClearDrawingMessage(&display_message_);
  //       DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
          ceres_gradients_.clear();
          jacobian_init_ = false;
        
          for (size_t i = 0; i < poses.size(); ++i) { 
            float angle = poses[i].angle;
            //std::cout << angle << " ::: " << atan2(sin(angle), cos(angle)) << std::endl;
            poses[i].angle = atan2(sin(angle), cos(angle));
          }
        
          //joint_opt_.orig_odom_ = orig_odom_;
          joint_opt_.poses_ = poses;
          joint_opt_.robot_frame_point_clouds_ = init_point_clouds;
          joint_opt_.robot_frame_normal_clouds_ = normal_clouds;
          joint_opt_.d3_covariances_ = covariances_;
          joint_opt_.human_constraints_ = human_constraints_;
                  
          joint_opt_.Run();

          poses = joint_opt_.poses_;
          ceres_gradients_ = joint_opt_.gradients_;
          ceres_jacobian_ = joint_opt_.ceres_jacobian_;
          jacobian_init_ = true;
          num_hc_residuals_ = joint_opt_.num_hc_residuals_;
          std::cout << "num hc residuals: " << num_hc_residuals_ << std::endl;
      
          ClearDrawingMessage(&display_message_);
          DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
          int final_hc_size = human_constraints_.size();
          added_human_constraints = final_hc_size - init_hc_size;
          std::cout << "added constraints: " << added_human_constraints << std::endl;



//         vector<float> cc = localization_.GetCeresCost();
//         CHECK_EQ(cc.size(), poses.size());
//         float total_cost = 0.0;
//         for (size_t i = 0; i < cc.size(); ++i) {
//           cout << "pose: " << i << "cost: " << cc[i] << endl;
//           total_cost += cc[i];
//         }
//         cout << "total cost: " << total_cost << endl;


//           prev_covariances_ = covariances_;
//           covariances_ = localization_.GetPoseCovariances();
          cout << "size of covariances_: " << covariances_.size() << endl;

          cout << "completed cycle" << endl;
        }
      
        current_replay_index++;
      }
      else {
        cout << "No more inputs to replay!" << endl;
      }
    }
  }
}












bool IsValidCorrectionType(const CorrectionType& type) {
  if (type == CorrectionType::kPointCorrection ||
      type == CorrectionType::kLineSegmentCorrection ||
      type == CorrectionType::kCornerCorrection ||
      type == CorrectionType::kColinearCorrection ||
      type == CorrectionType::kPerpendicularCorrection ||
      type == CorrectionType::kParallelCorrection) {
    return true;
  }
  return false;
}

void AddCorrectionPoints(const vector_slam_msgs::GuiMouseClickEvent& msg) {
  const CorrectionType correction_input_type =
      static_cast<CorrectionType>(msg.modifiers);
  const Vector2f p1(msg.mouse_down.x, msg.mouse_down.y);
  const Vector2f p2(msg.mouse_up.x, msg.mouse_up.y);
  if (correction_input_type != pending_correction_type_ &&
      IsValidCorrectionType(correction_input_type)) {
    const auto name_index = static_cast<size_t>(correction_input_type);
    cout << "Correction mode: "
         << CorrectionTypeNames[name_index]
         << endl;
    // Start a new correction type.
    selected_points_.clear();
    // All corrections need the first point.
    selected_points_.push_back(p1);
    if (correction_input_type != CorrectionType::kPointCorrection) {
      // Except for the point correction, all other corrections require
      // the second point.
      selected_points_.push_back(p2);
    }
    pending_correction_type_ = correction_input_type;
    correction_type_ = CorrectionType::kUnknownCorrection;
  } else {
    switch (correction_input_type) {
      case CorrectionType::kPointCorrection : {
        cout << "Points" << endl;
        if (correction_type_ == CorrectionType::kUnknownCorrection) {
          selected_points_.push_back(p1);
          correction_type_ = CorrectionType::kPointCorrection;
          pending_correction_type_ = CorrectionType::kUnknownCorrection;
        }
      } break;
      case CorrectionType::kLineSegmentCorrection : {
        cout << "Lines" << endl;
        if (correction_type_ == CorrectionType::kUnknownCorrection) {
          selected_points_.push_back(p1);
          selected_points_.push_back(p2);
          correction_type_ = CorrectionType::kLineSegmentCorrection;
          pending_correction_type_ = CorrectionType::kUnknownCorrection;
        }
      } break;
      case CorrectionType::kCornerCorrection : {
        cout << "Corners" << endl;
        if (pending_correction_type_ == CorrectionType::kCornerCorrection &&
            selected_points_.size() < 6) {
          selected_points_.push_back(p1);
          selected_points_.push_back(p2);
        } else if (correction_type_ == CorrectionType::kUnknownCorrection) {
          selected_points_.push_back(p1);
          selected_points_.push_back(p2);
          correction_type_ = CorrectionType::kCornerCorrection;
          pending_correction_type_ = CorrectionType::kUnknownCorrection;
        }
      } break;
      case CorrectionType::kColinearCorrection : {
        cout << "Colinear" << endl;
        if (correction_type_ == CorrectionType::kUnknownCorrection) {
          selected_points_.push_back(p1);
          selected_points_.push_back(p2);
          correction_type_ = CorrectionType::kColinearCorrection;
          pending_correction_type_ = CorrectionType::kUnknownCorrection;
        }
      } break;
      case CorrectionType::kPerpendicularCorrection : {
        cout << "Perpendicular" << endl;
        if (correction_type_ == CorrectionType::kUnknownCorrection) {
          selected_points_.push_back(p1);
          selected_points_.push_back(p2);
          correction_type_ = CorrectionType::kPerpendicularCorrection;
          pending_correction_type_ = CorrectionType::kUnknownCorrection;
        }
      } break;
      case CorrectionType::kParallelCorrection : {
        cout << "Parallel" << endl;
        if (correction_type_ == CorrectionType::kUnknownCorrection) {
          selected_points_.push_back(p1);
          selected_points_.push_back(p2);
          correction_type_ = CorrectionType::kParallelCorrection;
          pending_correction_type_ = CorrectionType::kUnknownCorrection;
        }
      } break;
      default : {
        // Do nothing.
      } break;
    }
  }
}

size_t VerifyUserInput(vector<PointCloudf>* temp_point_clouds) {
  size_t points_verified = 0;
  float local_select_thresh = 0.05;
  bool seen;
  for (size_t i=0; i<selected_points_.size(); i++) {
    seen = false;
    for (size_t j=0; j<temp_point_clouds->size(); j++) {
      for (size_t k=0; k<temp_point_clouds[0][j].size(); k++) {
        if ((temp_point_clouds[0][j][k] - selected_points_[i]).norm() <
              local_select_thresh) {
          points_verified ++;
          seen = true;
          break;
        }
      }
      if (seen) {
        break;
      }
    }
  }

  if (selected_points_[0] == selected_points_[1] ||
      selected_points_[2] == selected_points_[3]) {
    points_verified = 0;
  }
  return points_verified;
}




void ResetCorrectionInputs() {
  selected_points_.clear();
  pending_correction_type_ = CorrectionType::kUnknownCorrection;
  correction_type_ = CorrectionType::kUnknownCorrection;
}




















void LoopCloseCallback(const vector_slam_msgs::GuiMouseClickEvent& msg) {
  //static const bool kDebug = false;
  if (msg.modifiers == 0x06) {
    loop_corrections_on_ = !loop_corrections_on_;
    cout << "Loop corrections: " << loop_corrections_on_ << endl;
  }

  if (loop_corrections_on_) {
    AddCorrectionPoints(msg);
  }

  else if (correction_type_ != CorrectionType::kUnknownCorrection) {
    // Not in corrections mode, and there is a valid correction type.
    vector<PointCloudf> temp_point_clouds = init_point_clouds;

    CHECK_EQ(temp_point_clouds.size(), poses.size());
    CHECK_EQ(temp_point_clouds.size(), init_point_clouds.size());

    for (size_t i=0; i<init_point_clouds.size(); i++) {
      const Rotation2Df rotation(poses[i].angle);
      const Translation2f translation(poses[i].translation);
      const Affine2f pose_transform = translation * rotation;
      for (size_t j=0; j < init_point_clouds[i].size(); j++) {
        temp_point_clouds[i][j] = pose_transform * init_point_clouds[i][j];
      }
    }

    //NOTE: temp_point_clouds = world frame
    //NOTE: init_point_clouds = robot frame
    //NOTE: normal_clouds = robot frame
    
    size_t points_verified = VerifyUserInput(&temp_point_clouds);
    cout << "points verified: " << points_verified << endl;

    if (points_verified == selected_points_.size()) {

      //TODO: move to seperate function
      std::vector<std::vector<Eigen::Vector2f>> world_frame_point_clouds;
      for (size_t i = 0; i < init_point_clouds.size(); ++i) {
        const Rotation2Df pose_rotation(poses[i].angle);
        const Affine2f pose_transform = Translation2f(poses[i].translation) * pose_rotation;
        std::vector<Eigen::Vector2f> single_point_cloud;
        for (unsigned int j = 0; j < init_point_clouds[i].size(); ++j) {
          const Vector2f p = pose_transform * init_point_clouds[i][j];
          single_point_cloud.push_back(p);
        }
        world_frame_point_clouds.push_back(single_point_cloud);
      }
      
      
      
      //NOTE: maybe use this for evaluation.... maybe
      //if (localization_options_.EvaluateConsistency2 != NULL) {
      //  localization_options_.EvaluateConsistency2(poses, init_point_clouds);
      //}

      
      
      
      prev_poses = poses;
      prev_covariances_ = covariances_;
      //bool debug = false;
      int init_hc_size = human_constraints_.size();
      SingleInput current_input;
      current_input.type_of_constraint = correction_type_;
      current_input.input_points = selected_points_;
      current_input.undone = 0;
      input_history.push_back(current_input);
      printf("correction type: %d\n", input_history.back().type_of_constraint);

      
      
      
//TODO: change these to call constructor with all relavent args instead of assigning one by one 
      
      em_input_.local_version_point_clouds_ = world_frame_point_clouds;
      em_input_.selected_points_ = selected_points_;
      em_input_.correction_type_ = correction_type_;
      
      em_input_.Run();
      
      std::vector<int> corrected_poses = em_input_.corrected_poses_;
      std::vector<int> anchor_poses = em_input_.anchor_poses_;
      std::pair<int, int> backprop_bounds = em_input_.backprop_bounds_;
      std::vector<Eigen::Vector2f> new_selected_points = em_input_.selected_points_;

//       ClearDrawingMessage(&display_message_);
//       DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
      if (backprop_bounds.first >= 0 && backprop_bounds.second >= 1) {
        app_exp_corr_.correction_type_ = correction_type_;
        app_exp_corr_.selected_points_ = new_selected_points;
        app_exp_corr_.corrected_poses_ = corrected_poses;
        app_exp_corr_.anchor_poses_ = anchor_poses;
        app_exp_corr_.poses_ = poses;
     
        app_exp_corr_.Run();

        poses = app_exp_corr_.poses_;
        Eigen::Vector3f correction = app_exp_corr_.correction_;
        std::vector<VectorMapping::HumanConstraint> new_human_constraints;
        new_human_constraints = app_exp_corr_.new_human_constraints_;
        human_constraints_.push_back(new_human_constraints);
      
  //       ClearDrawingMessage(&display_message_);
  //       DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
        //std::cout << "BP bounds: " << backprop_bounds.first << ", " << backprop_bounds.second << std::endl;
      
        backprop_.poses_ = poses;
        backprop_.correction_ = correction;
        backprop_.backprop_bounds_ = backprop_bounds;
        backprop_.d3_covariances_ = covariances_;
      
        backprop_.Run();
      
        covariances_ = backprop_.d3_covariances_;
        poses = backprop_.poses_;

  //       ClearDrawingMessage(&display_message_);
  //       DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
        ceres_gradients_.clear();
        jacobian_init_ = false;
        
        for (size_t i = 0; i < poses.size(); ++i) { 
          float angle = poses[i].angle;
          //std::cout << angle << " ::: " << atan2(sin(angle), cos(angle)) << std::endl;
	  poses[i].angle = atan2(sin(angle), cos(angle));
        }
        
        //joint_opt_.orig_odom_ = orig_odom_;
        joint_opt_.poses_ = poses;
        joint_opt_.robot_frame_point_clouds_ = init_point_clouds;
        joint_opt_.robot_frame_normal_clouds_ = normal_clouds;
        joint_opt_.d3_covariances_ = covariances_;
        joint_opt_.human_constraints_ = human_constraints_;
                  
        joint_opt_.Run();

        poses = joint_opt_.poses_;
        ceres_gradients_ = joint_opt_.gradients_;
        ceres_jacobian_ = joint_opt_.ceres_jacobian_;
        jacobian_init_ = true;
        num_hc_residuals_ = joint_opt_.num_hc_residuals_;
	std::cout << "num hc residuals: " << num_hc_residuals_ << std::endl;
      
        ClearDrawingMessage(&display_message_);
        DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
        int final_hc_size = human_constraints_.size();
        added_human_constraints = final_hc_size - init_hc_size;
        std::cout << "added constraints: " << added_human_constraints << std::endl;
      }
    }
    
    cout << "completed cycle" << endl;
    num_completed_cycles++;

    ResetCorrectionInputs();
  }
  else {
    ResetCorrectionInputs();
  }

}


