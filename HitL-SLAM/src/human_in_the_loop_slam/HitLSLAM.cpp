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

//#include "eigen_helper.h"
//#include "helpers.h"
//#include "../vmapping/vector_mapping.h"
#include "../../extern_libraries/CImg/CImg.h"
#include "glog/logging.h"
#include "ceres/ceres.h"
//#include "vector_slam_msgs/CobotEventsMsg.h"
//#include "vector_slam_msgs/CobotLocalizationMsg.h"
//#include "vector_slam_msgs/CobotOdometryMsg.h"
//#include "vector_slam_msgs/GuiKeyboardEvent.h"
//#include "vector_slam_msgs/GuiMouseMoveEvent.h"
//#include "vector_slam_msgs/GuiMouseClickEvent.h"
//#include "vector_slam_msgs/LidarDisplayMsg.h"
//#include "vector_slam_msgs/LocalizationGuiCaptureSrv.h"
//#include "vector_slam_msgs/LocalizationMsg.h"
//#include "vector_slam_msgs/CobotLocalizationSrv.h"
#include "../gui/gui_publisher_helper.h"
#include "nav_msgs/Odometry.h"
#include "../perception_tools/perception_2d.h"
//#include "popt_pp.h"
//#include "proghelp.h"
//#include "pthread_utils.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "../shared/math/geometry.h"
#include "../shared/util/configreader.h"
#include "sensor_msgs/LaserScan.h"
//#include "timer.h"
//#include "util.h"
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


