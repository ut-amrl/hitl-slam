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


//TODO: 
//      save files
//      load config
//      viz / gui
//      log logic
//      replay logic
//      undo logic
//      load odom correctly
//      load odom quick fix



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
//#include "../../extern_libraries/CImg/CImg.h"
#include "glog/logging.h"
#include "ceres/ceres.h"
#include "vector_slam_msgs/CobotEventsMsg.h"
#include "vector_slam_msgs/CobotLocalizationMsg.h"
#include "vector_slam_msgs/CobotOdometryMsg.h"
#include "vector_slam_msgs/GuiKeyboardEvent.h"
#include "vector_slam_msgs/GuiMouseMoveEvent.h"
#include "vector_slam_msgs/GuiMouseClickEvent.h"
#include "vector_slam_msgs/LidarDisplayMsg.h"
#include "vector_slam_msgs/LocalizationGuiCaptureSrv.h"
#include "vector_slam_msgs/LocalizationMsg.h"
#include "vector_slam_msgs/CobotLocalizationSrv.h"
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

#include "HitLSLAM.h"


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
//using EnmlMaps::PersistentObject;
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


char* pose_graph_file_ = NULL;

char* log_file_ = NULL;


// Display message for drawing debug vizualizations on the localization_gui.
vector_slam_msgs::LidarDisplayMsg display_message_;

// Service client to ask localization_gui to save images.
//ros::ServiceClient gui_capture_client;

// ROS publisher to publish display_message_ to the ROS topic
// Cobot/VectorLocalization/Gui
ros::Publisher display_publisher_;

// ROS publisher to the CoBot events topic.
ros::Publisher events_publisher_;

// ROS publisher to publish the latest robot localization estimate to
// Cobot/Localization
//ros::Publisher localization_publisher_;

// ROS subscriber to the localization gui mouse move events
ros::Subscriber mouse_move_subscriber_;

// ROS subscriber to the localization gui mouse click events
ros::Subscriber mouse_click_subscriber_;

// ROS subscriber to /Cobot/VectorLocalization/GuiKeyboardEvents
ros::Subscriber keyboard_event_subscriber_;

// Instance of HitLSLAM class used for running all HitLSLAM computations 
HitLSLAM hitl_slam_session_;

// Flag for whether or not to process gui input as a correction
bool correction_mode_on_ = false;




























static const uint32_t kTrajectoryColor = 0x7F000000;
static const uint32_t kPoseCovarianceColor = 0xFF808080;
static const uint32_t kOdometryColor = 0x70FF0000;
static const uint32_t kLtfCorrespondenceColor = 0x7FFF7700;
static const uint32_t kLtfPointColor = 0xFFFF7700;
static const uint32_t kStfPointColor = 0xFFFF5500;
static const uint32_t kStfCorrespondenceColor = 0x7F994CD9;
static const uint32_t kDfPointColor  = 0x7F37B30C;
static const uint32_t kObjectColor = 0xFF56C4C3;










double total_runtime = 0.0;
int num_completed_cycles = 0;
int num_total_constraints = 0;
vector<SingleInput> input_history;
vector<SingleInput> logged_input;
int current_replay_index = 0;
int correction_number = 0;










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
// Angular margin from the scan area boundary to ignore laser readings.
float kAngularMargin = 0.0;









//TODO: figure out decent config setup
// WatchFiles to track changes to config files.
//WatchFiles watch_files_;

// Config reader for localization options.
//ConfigReader config_((kCobotStackPath + "/").c_str());


/*

bool LoadConfiguration(VectorMapping::VectorMappingOptions* options) {
  if (!config_.readFiles()) return false;

  ConfigReader::SubTree c(config_,"VectorMapping");
  bool error = false;
  error = error || !c.getReal("radial_translation_uncertainty",
                              kRadialTranslationUncertainty);
  error = error || !c.getReal("tangential_translation_uncertainty",
                              kTangentialTranslationUncertainty);
  error = error || !c.getReal("angle_uncertainty", kAngleUncertainty);
  error = error || !c.getReal("odometry_translation_scale",
                              kOdometryTranslationScale);
  error = error || !c.getReal("odometry_rotation_scale",
                              kOdometryRotationScale);
  error = error || !c.getReal("min_point_cloud_range", kMinPointCloudRange);
  error = error || !c.getReal("max_point_cloud_range", kMaxPointCloudRange);
  error = error || !c.getReal("max_normal_point_distance",
                              kMaxNormalPointDistance);
#ifdef NDEBUG
  error = error || !c.getInt("num_threads", options->kNumThreads);
#else
  options->kNumThreads = 1;
#endif

  options->kMinRange = kMinPointCloudRange;
  options->kMaxRange = kMaxPointCloudRange;

  error = error || !c.getReal("robot_laser_offset.x",
                              options->sensor_offset.x());
  error = error || !c.getReal("robot_laser_offset.y",
                              options->sensor_offset.y());
  error = error || !c.getReal("min_rotation", options->minimum_node_rotation);
  error = error || !c.getReal("min_translation",
                              options->minimum_node_translation);
  error = error || !c.getInt("max_correspondences_per_point",
                             options->kMaxCorrespondencesPerPoint);
  error = error || !c.getReal("laser_std_dev",
                              options->kLaserStdDev);
  error = error || !c.getReal("point_correlation_factor",
                              options->kPointPointCorrelationFactor);
  error = error || !c.getReal("odometry_radial_stddev_rate",
                              options->kOdometryRadialStdDevRate);
  error = error || !c.getReal("odometry_tangential_stddev_rate",
                              options->kOdometryTangentialStdDevRate);
  error = error || !c.getReal("odometry_angular_stddev_rate",
                              options->kOdometryAngularStdDevRate);
  error = error || !c.getReal("odometry_translation_min_stddev",
                              options->kOdometryTranslationMinStdDev);
  error = error || !c.getReal("odometry_translation_max_stddev",
                              options->kOdometryTranslationMaxStdDev);
  error = error || !c.getReal("odometry_rotation_min_stddev",
                              options->kOdometryAngularMinStdDev);
  error = error || !c.getReal("odometry_rotation_max_stddev",
                              options->kOdometryAngularMaxStdDev);
  error = error || !c.getReal("point_match_threshold",
                              options->kPointMatchThreshold);
  error = error || !c.getReal("max_stf_angle_error",
                              options->kMaxStfAngleError);
  error = error || !c.getInt("pose_increment",
                              options->kPoseIncrement);
  error = error || !c.getInt("max_history",
                             options->kMaxHistory);
  error = error || !c.getInt("max_solver_iterations",
                             options->max_solver_iterations);
  error = error || !c.getInt("max_repeat_iterations",
                             options->kMaxRepeatIterations);
  error = error || !c.getInt("num_repeat_iterations",
                             options->kNumRepeatIterations);
  error = error || !c.getUInt("min_episode_length",
                              options->kMinEpisodeLength);
  error = error || !c.getUInt("num_skip_readings",
                             options->num_skip_readings);
  error = error || !c.getReal("angle_margin",
                              kAngularMargin);
  error = error || !c.getReal("max_update_period",
                              options->max_update_period);
  //error = error || !relocalization_lidar_params_.LoadFromConfig(&config_);

  return !error;
}

*/












//Load point clouds into ROBOT FRAME
void loadPoseGraph(bool placeholder_for_existing_loop_closure_constraints,
                   vector<Pose2Df>* odometry_ptr,
                   vector<PointCloudf>* point_cloud_ptr,
                   vector<NormalCloudf>* normal_cloud_ptr, 
                   vector<Pose2Df>* poses_ptr,
                   vector<Matrix3f>* covariances_ptr) {

  vector<PointCloudf> point_clouds = *point_cloud_ptr;
  vector<NormalCloudf> normal_clouds = *normal_cloud_ptr;
  vector<Pose2Df> poses = *poses_ptr;
  vector<Matrix3f> covariances = *covariances_ptr;
  PointCloudf single_point_cloud;
  NormalCloudf single_normal_cloud;
  
  CHECK_EQ(point_clouds.size(),0);
  CHECK_EQ(normal_clouds.size(),0);
  CHECK_EQ(poses.size(),0);
  CHECK_EQ(covariances.size(),0);
  
  ScopedFile fid(pose_graph_file_, "r");
  if(fid() == NULL) {
    cout << "ERROR: Unable to open specified pose-graph file: " << pose_graph_file_ << endl;
    exit(1);
  }
  
  static const int kNumFields = 16;
  Vector2f point(0, 0);
  Vector2f normal(0, 0);
  Pose2Df pose(0, 0, 0);
  Matrix3f one_cov = Matrix3f::Zero();
  



  //TODO: need to add original odom

  //TODO: maybe this can be done better

  while (fscanf(fid(), "%f,%f,%f, %f,%f, %f,%f,"
                       "%f,%f,%f,%f,%f,%f,%f,%f,%f,\n",
    &(pose.translation.x()), &(pose.translation.y()), &(pose.angle),
    &(point.x()), &(point.y()), &(normal.x()), &(normal.y()),
    &(one_cov(0,0)), &(one_cov(0,1)), &(one_cov(0,2)),
    &(one_cov(1,0)), &(one_cov(1,1)), &(one_cov(1,2)),
    &(one_cov(2,0)), &(one_cov(2,1)), &(one_cov(2,2))) == kNumFields) {

    if (poses.size() == 0) {
      poses.push_back(pose);
      //cout << covariances.size() << "\n" << one_cov << endl;
      covariances.push_back(one_cov);
      single_point_cloud.push_back(point);
      single_normal_cloud.push_back(normal);
    }
    else if (pose.translation != poses[poses.size()-1].translation ||
             pose.angle != poses[poses.size()-1].angle) {
      const Rotation2Df R(-poses[poses.size()-1].angle);
      const Vector2f pose_location = -poses[poses.size()-1].translation;
      for (size_t i = 0; i < single_point_cloud.size(); i++) {
          single_point_cloud[i] = R*(single_point_cloud[i] + pose_location);
          single_normal_cloud[i] = R*(single_normal_cloud[i] + pose_location);
      }
      point_clouds.push_back(single_point_cloud);
      normal_clouds.push_back(single_normal_cloud);
      single_point_cloud.clear();
      single_normal_cloud.clear();
      poses.push_back(pose);
      covariances.push_back(one_cov);
      single_point_cloud.push_back(point);
      single_normal_cloud.push_back(normal);
    }
    else {
      single_point_cloud.push_back(point);
      single_normal_cloud.push_back(normal);
    }
  }
  if (single_point_cloud.size() != 0) {
    const Rotation2Df R(-poses[poses.size()-1].angle);
    const Vector2f pose_location = -poses[poses.size()-1].translation;
    for (size_t i = 0; i < single_point_cloud.size(); i++) {
      single_point_cloud[i] = R*(single_point_cloud[i] + pose_location);
      single_normal_cloud[i] = R*(single_normal_cloud[i] + pose_location);
    }
    point_clouds.push_back(single_point_cloud);
    normal_clouds.push_back(single_normal_cloud);
  }
}





void ClearDisplay() {
  ClearDrawingMessage(&display_message_);
  display_publisher_.publish(display_message_);
}

void PublishDisplay() {
  display_publisher_.publish(display_message_);
  ClearDrawingMessage(&display_message_);
}




//TODO: finish, plus make as much of this repeated code into functions living
//      in gui_publisher_helper.h (or something like that)


void DisplayPoses() {
  ClearDrawingMessage(&display_message_);

  vector<Pose2Df> poses = hitl_slam_session_.getPoses();
  vector<Matrix3f> covariances = hitl_slam_session_.getCovariances();
  vector<PointCloudf> point_clouds = hitl_slam_session_.getWorldFrameScans();
  
  CHECK_EQ(poses.size(), point_clouds.size());
  CHECK_GT(poses.size(), 0);

  //float max_cost = 1.0;  
  float max_theta_var = 0.0;
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


  if (covariances.size() >= poses.size()) {
    for (size_t i = 0; i < poses.size();  ++i) {
      max_theta_var = max(float(covariances[i](2,2)), max_theta_var);
    }
  }
  else {
     max_theta_var = 1.0;
  }


  // Add robot localization trajectory.
  for (size_t i = 0; i < poses.size() - 1; ++i) {
    // color / draw covariances
    /*
    if (covariances.size() > i) {
      Eigen::Matrix2f single_cov;
      single_cov(0,0) = float(covariances[i](0,0));
      single_cov(0,1) = float(covariances[i](0,1));
      single_cov(1,0) = float(covariances[i](1,0));
      single_cov(1,1) = float(covariances[i](1,1));
      float theta_var = covariances[i](2,2);
      DrawPoseCovariance3D(poses[i].translation, single_cov, theta_var);
    }*/
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
    /*
    for (size_t j = 0; j < human_constraints_.size(); ++j) {
      for (size_t k = 0; k < human_constraints_[j].size(); ++k) {
        if (human_constraints_[j][k].anchor_pose_id == int(i) ||
            human_constraints_[j][k].constrained_pose_id == int(i)) {
          hc = true;
        }
      }
    }*/
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

/*
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
*/


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

  // Draw point clouds
  for (size_t i = 0; i < point_clouds.size(); ++i) {
    const PointCloudf& point_cloud = point_clouds[i];
    for (unsigned int j = 0; j < point_cloud.size(); ++j) {
      uint32_t color = kLtfPointColor;
      DrawPoint(point_cloud[j], color, &display_message_);
    }
  }
  PublishDisplay();
}





































/*
void GetCovarianceFromRelativePose(const Vector2f& relative_location,
                                   const float& relative_angle,
                                   Matrix3f* covariance) {
  covariance->setZero();
  if (relative_location.norm() > FLT_MIN) {
    const Vector2f radial_direction(relative_location.normalized());
    const Vector2f tangential_direction(Rotation2Df(M_PI_2) * radial_direction);
    Matrix2f eigenvectors;
    eigenvectors.leftCols<1>() = radial_direction;
    eigenvectors.rightCols<1>() = tangential_direction;
    Matrix2f eigenvalues;
    eigenvalues.setZero();
    eigenvalues(0, 0) = kRadialTranslationUncertainty;
    eigenvalues(1, 1) = kTangentialTranslationUncertainty;
    covariance->block<2, 2>(0, 0) =
        eigenvectors * eigenvalues * (eigenvectors.transpose());
  }
  (*covariance)(2, 2) = kAngleUncertainty * fabs(relative_angle);
}
*/


































/*

void SaveLoggedPoses(const string& filename,
                     const vector<Pose2Df>& logged_poses,
                     const vector<double>& timestamps) {
  ScopedFile fid(filename, "w");
  for (size_t i = 0; i < logged_poses.size(); ++i) {
    fprintf(fid(), "%f %f %f %f\n",
            timestamps[i],
            logged_poses[i].translation.x(),
            logged_poses[i].translation.y(),
            logged_poses[i].angle);
  }
}

//transform point clouds and save them in WORLD FRAME
void SaveStfsandCovars(
    const string& map_name,
    const vector<Pose2Df>& poses,
    const vector<PointCloudf>& point_clouds,
    const vector<NormalCloudf>& normal_clouds,
    const string& bag_file,
    double timestamp) {
  static const bool kDisplaySteps = false;
  const string stfs_and_covars_file = bag_file + ".stfs.covars";
  ScopedFile fid(stfs_and_covars_file, "w");
  fprintf(fid(), "%s\n", map_name.c_str());
  fprintf(fid(), "%lf\n", timestamp);
  VectorMap map(map_name, kMapsDirectory, true);
  if (kDisplaySteps) {
    ClearDrawingMessage(&display_message_);
    nonblock(true);
  }
  // edit to save partial trajectories
  for (size_t i = 0; i < point_clouds.size(); ++i) {
    const Rotation2Df pose_rotation(poses[i].angle);
    const Affine2f pose_transform =
        Translation2f(poses[i].translation) * pose_rotation;
    //cout << i << "\n" << covariances[i] << endl;
    //printf("%f\n", covariances[i](0,0));
    for (unsigned int j = 0; j < point_clouds[i].size(); ++j) {
      const Vector2f p = pose_transform * point_clouds[i][j];
      if (kDisplaySteps) {
        DrawLine(p, poses[i].translation, 0x1FC0C0C0, &display_message_);
      }
      const vector2f p_g(p.x(), p.y());

      const Vector2f n = pose_rotation * normal_clouds[i][j];
      fprintf(fid(), "%.4f,%.4f,%.4f,%.4f,%.4f, %.4f,%.4f,"
              "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",
          poses[i].translation.x(), poses[i].translation.y(),
          poses[i].angle, p.x(), p.y(), n.x(), n.y(),
          covariances[i](0,0), covariances[i](0,1), covariances[i](0,2),
          covariances[i](1,0), covariances[i](1,1), covariances[i](1,2),
          covariances[i](2,0), covariances[i](2,1), covariances[i](2,2));
      if (kDisplaySteps) {
        DrawPoint(p, kStfPointColor, &display_message_);
      }
    }
    if (kDisplaySteps) {
      PublishLocation(kMapName, poses[i].translation.x(),
                      poses[i].translation.y(), poses[i].angle);
      PublishDisplay();
      while (kbhit() == 0) {
        Sleep(0.02);
      }
      fgetc(stdin);
      printf("\r");
      fflush(stdout);
      const string file_name = StringPrintf(
          "poses/%04d.png", static_cast<int>(i));
      vector_slam_msgs::LocalizationGuiCaptureSrv::Request req;
      vector_slam_msgs::LocalizationGuiCaptureSrv::Response res;
      req.filename = string(file_name);
      printf("Saving image to %s\n", req.filename.c_str());
      gui_capture_client.call(req, res);
    }
  }
  if (kDisplaySteps) {
    PublishDisplay();
  }
}
*/

















//TODO: load log file - make work

/*

vector<SingleInput> LoadLogFile(const string& log_file) {
  vector<SingleInput> logged_input;
  string full_line;
  string line;

  std::fstream stream(log_file);

  getline(stream, full_line);
  double total_time = stod(full_line);
  cout << "total_time: " << total_time << endl;
  getline(stream, full_line);
  int num_entries = stoi(full_line);
  getline(stream, full_line);
  int num_constraints = stoi(full_line);
  cout << "num constriants: " << num_constraints << endl;

  for (int i = 0; i < num_entries; ++i) {
    if (getline(stream, full_line)) {
      SingleInput one_input;
      std::stringstream iss;
      iss << full_line;
      getline(iss, line, ',');
      int constraint_type = stoi(line);
      getline(iss, line, ',');
      int undone = stoi(line);
      //cout << constraint_type << endl;
      //cout << undone << endl;
//       one_input.type_of_constraint = constraint_type;
      one_input.undone = undone;

      vector<Vector2f> logged_selected_points;
      Vector2f new_point;
      int num_selected_points = 0;
      if (constraint_type == 1) { //colocation
       num_selected_points = 2;
       one_input.type_of_constraint = CorrectionType::kPointCorrection;
      }
      else if (constraint_type == 3) { // Corners
        num_selected_points = 8;
        one_input.type_of_constraint = CorrectionType::kCornerCorrection;
      }
      else if (constraint_type == 2 || constraint_type == 4 ||
               constraint_type == 5 || constraint_type == 7) { // other
        num_selected_points = 4;
        if (constraint_type == 2) {
          one_input.type_of_constraint = CorrectionType::kLineSegmentCorrection;
        }
        else if (constraint_type == 4) {
          one_input.type_of_constraint = CorrectionType::kColinearCorrection;
        }
        else if (constraint_type == 5) {
          one_input.type_of_constraint = CorrectionType::kPerpendicularCorrection;
        }
        else if (constraint_type == 7) {
          one_input.type_of_constraint = CorrectionType::kParallelCorrection;
        }
      }
      else { // garbage
        one_input.type_of_constraint = CorrectionType::kUnknownCorrection;
        cout << "ERROR: unidentified constraint type read in log file." << endl;
      }
      for (int j = 0; j < num_selected_points; ++j) {
        getline(stream, full_line);
        std::stringstream instream;
        instream << full_line;
        getline(instream, line, ',');
        float px = stof(line);
        getline(instream, line, ',');
        float py = stof(line);
        new_point(0) = px;
        new_point(1) = py;
        logged_selected_points.push_back(new_point);
      }
      one_input.input_points = logged_selected_points;
      logged_input.push_back(one_input);

      cout << constraint_type << ", " << one_input.undone << endl;
      for (size_t j = 0; j < one_input.input_points.size(); ++j) {
        Vector2f p = one_input.input_points[j];
        cout << p(0) << ", " << p(1) << endl;
      }
    }
    else {
      break;
    }
  }

  return logged_input;
}

*/


























//TODO make this work


/*

void LogActivity() {
  clock_gettime(CLOCK_MONOTONIC, &ms2);
  total_runtime =  (ms2.tv_sec - ms1.tv_sec) +
                  ((ms2.tv_nsec - ms1.tv_nsec) / 1000000000.00);
  cout << "total runtime: " << total_runtime << endl;
  if (log_file == NULL) {
    num_total_constraints = human_constraints_.size();

    std::stringstream ss;
    ss.str(bag_name);
    string item;
    vector<string> elems;
    while (getline(ss, item, '.')) {
      elems.push_back(item);
    }
    bag_name = elems[0];

    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    string year = std::to_string((now->tm_year + 1900));
    string mon = std::to_string((now->tm_mon + 1));
    string day = std::to_string(now->tm_mday);
    string hour = std::to_string(now->tm_hour);
    string min = std::to_string(now->tm_min);
    string sec = std::to_string(now->tm_sec);

    string log_date = year +"-"+ mon +"-"+ day +"-"+ hour +"-"+ min +"-"+ sec;
    string log_name = bag_name + "_logged_" + log_date + ".log";

    ScopedFile fid(log_name, "w");
    fprintf(fid(), "%.4f \n", total_runtime);
    fprintf(fid(), "%d \n", num_completed_cycles);
    fprintf(fid(), "%d \n", num_total_constraints);
    for (size_t i = 0; i < input_history.size(); ++i) {
      fprintf(fid(), "%d, %d\n", input_history[i].type_of_constraint,
                                input_history[i].undone);
      cout << "writing undone status: " << input_history[i].undone << endl;
      for (size_t j = 0; j < input_history[i].input_points.size(); ++j) {
        fprintf(fid(), "%.4f, %.4f\n", input_history[i].input_points[j](0),
                                     input_history[i].input_points[j](1));
      }
    }
  }
}

*/
















void MouseMoveCallback(const vector_slam_msgs::GuiMouseMoveEvent& msg) {
  //ClearDrawingMessage(&display_message_);
  //DrawCircle(msg.location,
  //           cobot_gui::kColorRed,
   //          &display_message_);
  //PublishDisplay();
}


bool MouseDragged(const vector_slam_msgs::GuiMouseClickEvent& msg) {
  const int sq_distance_moved =
      sqrt(msg.mouse_down.x - msg.mouse_up.x) +
      sqrt(msg.mouse_down.y - msg.mouse_up.y);
  return (sq_distance_moved > 1.0);
}










void KeyboardRequestCallback(const vector_slam_msgs::GuiKeyboardEvent& msg) {
  // Code for 'p' key, for Provide correction
  if (msg.keycode == 0x50) {
    correction_mode_on_ = !correction_mode_on_;
    cout << "In correction mode: " << correction_mode_on_ << endl;
    if (!correction_mode_on_) {
      hitl_slam_session_.Run();
    }
  }
  //TODO finish
  /*
  else if (msg.keycode == 0x55) { // key code 85, 'u' for undo
    cout << "undo" << endl;
    cout << (log_file == NULL) << endl;
    if (log_file == NULL) {
      poses = prev_poses;
      covariances = prev_covariances_;
      input_history.back().undone = 1;
//       cout << (added_human_constraints > 0) << endl;
      if (added_human_constraints > 0) {
        cout << "removing constraints" << endl;
        for (int i = 0; i < added_human_constraints; ++i) {
          human_constraints_.pop_back();
        }
        added_human_constraints = 0;
      }
      ClearDrawingMessage(&display_message_);
      DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
    }
    else {
      cout << "undo not allowed in log replay mode." << endl;
    }
  }
  else if (msg.keycode == 0x56) { //key code 86, 'v' for save
    cout << "Are you sure you want to save? (y/n)" << endl;
    string s;
    std::cin >> s;
    if (s == "y") {
      cout << "time to save!" << endl;
      message_timestamps_.push_back(0.0);
      CHECK_GT(message_timestamps_.size(), 0);
    //SaveLoggedPoses(string(bag_file) + ".poses", poses, message_timestamps_);
      SaveLoggedPoses("test.poses", poses, message_timestamps_);
      if (save_stfs_) {
        SaveStfsandCovars(kMapName,
                poses,
                init_point_clouds,
                normal_clouds,
                "test.poses",
                message_timestamps_.front());
        SaveStfs(kMapName,
                poses,
                init_point_clouds,
                normal_clouds,
                "test.poses",
                message_timestamps_.front());
      }
      cout << "time to ctrl-C!" << endl;
    }
  }
  else if (msg.keycode == 0x4C) { //key code 76, 'l' for log
    cout << "step ahead log" << endl;
//     cout << "not currently available" << endl;
    ReplayLog();
  }*/
}


void MouseClickCallback(const vector_slam_msgs::GuiMouseClickEvent& msg) {
  if (correction_mode_on_) {
    Vector2f mouse_down(msg.mouse_down.x, msg.mouse_down.y);
    Vector2f mouse_up(msg.mouse_up.x, msg.mouse_up.y);
    
    hitl_slam_session_.addCorrectionPoints(msg.modifiers, mouse_down, mouse_up);
  }
}













//Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  if (log_file_ == NULL) {
    //TODO: make this work
    //LogActivity();
  }
  printf("\nTerminating.\n");
  exit(0);
}

// Signal handler bad signals. Print a backtrace and quit.
void HandleTerminalError(int i) {
  printf("\nSignal intercepted, exiting.\n");
  PrintBackTrace();
  exit(0);
}

int main(int argc, char** argv) {
  //InitHandleStop(&run_, 0);
  signal(SIGINT, HandleStop);
  signal(SIGALRM, HandleStop);
  signal(SIGABRT, HandleTerminalError);
  signal(SIGSEGV, HandleTerminalError);

  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();

  static struct poptOption options[] = {
    { "load-pose-graph", 'P', POPT_ARG_STRING, &pose_graph_file_, 1,
        "Load existing pose-graph file", "STRING"},
    { "load-recording", 'L', POPT_ARG_STRING, &log_file_, 1,
        "Load existing log file", "STRING"},
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}

  //TODO figure out a good config system
  //config_.init(watch_files_);
  //config_.addFile("../robot.cfg");
  //config_.addFile("config/vector_mapping.cfg");
  //config_.addFile("config/localization_parameters.cfg");
  //CHECK(LoadConfiguration(&localization_options_));




  // Node setup
  string node_name = "Human-in-the-Loop SLAM";
  ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  ros::NodeHandle ros_node;

  // Publishers
  display_publisher_ =
      ros_node.advertise<vector_slam_msgs::LidarDisplayMsg>(
      "VectorSLAM/VectorLocalization/Gui",1,true);
  //localization_publisher_ =
  //    ros_node.advertise<vector_slam_msgs::CobotLocalizationMsg>(
  //    "VectorSLAM/Localization", 1, true);
  //gui_capture_client =
  //    ros_node.serviceClient<vector_slam_msgs::LocalizationGuiCaptureSrv>(
  //    "VectorLocalization/Capture");
 
  // Subscribers
  mouse_click_subscriber_ =
      ros_node.subscribe("VectorSLAM/VectorLocalization/GuiMouseClickEvents",
                         1, MouseClickCallback);
  mouse_move_subscriber_ =
      ros_node.subscribe("VectorSLAM/VectorLocalization/GuiMouseMoveEvents",
                         1, MouseMoveCallback);
  keyboard_event_subscriber_ = 
      ros_node.subscribe("VectorSLAM/VectorLocalization/GuiKeyboardEvents",
                         1, KeyboardRequestCallback);

  
  
  if (pose_graph_file_ == NULL && log_file_ == NULL) {
    fprintf(stderr,
        "ERROR: Must Specify pose-graph file (-P), or log file (-L)\n");
    return 1;
  }
  else if (pose_graph_file_ != NULL) {
    cout << "got pose graph" << endl;
    
    vector<Pose2Df> odom;
    vector<PointCloudf> rob_frame_pcs;
    vector<NormalCloudf> norm_clouds;
    vector<Matrix3f> covars;
    vector<Pose2Df> poses;
    
    loadPoseGraph(true, &odom, &rob_frame_pcs, &norm_clouds, &poses, &covars);

    hitl_slam_session_.init(odom, rob_frame_pcs, norm_clouds, covars, poses);
  }
  else if (log_file_ != NULL) {
    cout << "got log file" << endl;
    //TODO: make log replay work
    //BatchLocalize(keyframes, max_laser_poses,
    //              !disable_stfs, time_skip, return_initial_poses);
  }

  ros::spin();

  return 0;
}
