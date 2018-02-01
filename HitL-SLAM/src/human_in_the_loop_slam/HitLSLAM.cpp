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
//#include "../gui/gui_publisher_helper.h"
#include "nav_msgs/Odometry.h"
#include "../perception_tools/perception_2d.h"
//#include "popt_pp.h"
//#include "proghelp.h"
//#include "pthread_utils.h"
#include "../shared/math/geometry.h"
#include "../shared/util/configreader.h"
#include "sensor_msgs/LaserScan.h"
//#include "timer.h"
//#include "util.h"

#include "residual_functors.h"
#include "HitLSLAM.h"

#include "EMinput.h"
#include "ApplyExplicitCorrection.h"
#include "Backprop.h"
#include "JointOptimization.h"

//using cobot_gui::ClearDrawingMessage;
//using cobot_gui::DrawCircle;
//using cobot_gui::DrawLine;
//using cobot_gui::DrawPoint;
//using cobot_gui::DrawText;
using Eigen::Affine2d;
using Eigen::Affine2f;
using Eigen::Matrix2d;
using Eigen::Matrix2f;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
//using Eigen::Perp2;
using Eigen::Rotation2Dd;
using Eigen::Rotation2Df;
//using Eigen::ScalarCross;
using Eigen::Translation2d;
using Eigen::Translation2f;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector3d;
//using Eigen::DistanceToLineSegment;
using perception_2d::GenerateNormals;
using perception_2d::NormalCloudf;
using perception_2d::PointCloudf;
using perception_2d::Pose2Df;
using std::cout;
using std::endl;
using std::pair;
using std::queue;
using std::size_t;
using std::sort;
using std::string;
using std::vector;

// TODO: 
//       deal with reading from input log for replay
//       get covar from ceres (JO)
 
void HitLSLAM::init(const vector<Pose2Df> odom,
                    const vector<PointCloudf> rob_frame_pcs,
                    const vector<NormalCloudf> norm_clouds,
                    vector<Matrix3f> covars,
                    vector<Pose2Df> poses) {

  odometry_ = odom;
  prev_poses_ = poses;
  poses_ = poses;
  ROBOT_FRAME_point_clouds_ = rob_frame_pcs;
  normal_clouds_ = norm_clouds;
  prev_covariances_ = covars;
  covariances_ = covars;
  WORLD_FRAME_point_clouds_.resize(ROBOT_FRAME_point_clouds_.size());
  for (size_t i=0; i<ROBOT_FRAME_point_clouds_.size(); i++) {
    WORLD_FRAME_point_clouds_[i].resize(ROBOT_FRAME_point_clouds_[i].size());
  }
  transformPointCloudsToWorldFrame();
}

vector<Pose2Df> HitLSLAM::getPoses() {
  return poses_;
}

vector<Matrix3f> HitLSLAM::getCovariances() {
  return covariances_;
}

vector<PointCloudf> HitLSLAM::getWorldFrameScans() {
  return WORLD_FRAME_point_clouds_;
}

vector<SingleInput> HitLSLAM::getInputHistory() {
  return input_history_;
}

//TODO: define getter todo from header

bool HitLSLAM::isValidCorrectionType(const CorrectionType& type) {
  if (type == CorrectionType::kPointCorrection ||
      type == CorrectionType::kLineSegmentCorrection ||
      type == CorrectionType::kColinearCorrection ||
      type == CorrectionType::kPerpendicularCorrection ||
      type == CorrectionType::kParallelCorrection) {
    return true;
  }
  return false;
}

void HitLSLAM::addCorrectionPoints(const uint32_t type,
                                   const Eigen::Vector2f mouse_down,
                                   const Eigen::Vector2f mouse_up) {
  CorrectionType correction_type =  static_cast<CorrectionType>(type);
  if (correction_type == CorrectionType::kUnknownCorrection) { return; }
  else if (correction_type != pending_correction_type_ &&
                     isValidCorrectionType(correction_type)) { 
    const auto name_index = static_cast<size_t>(correction_type);
    cout << "Correction mode: " << CorrectionTypeNames[name_index] << endl;
    // Start a new correction type.
    selected_points_.clear();
    // All corrections need the first point.
    selected_points_.push_back(mouse_down);
    if (correction_type != CorrectionType::kPointCorrection) {
      // Except for the point correction, all other corrections require
      // the second point.
      selected_points_.push_back(mouse_up);
    }
    pending_correction_type_ = correction_type;
    //correction_type_ = CorrectionType::kUnknownCorrection;
  } 
  else {
    switch (correction_type) {
      case CorrectionType::kPointCorrection : {
        cout << "Points" << endl;
        selected_points_.push_back(mouse_down);
        correction_type_ = CorrectionType::kPointCorrection;
        pending_correction_type_ = CorrectionType::kUnknownCorrection;
      } break;
      case CorrectionType::kLineSegmentCorrection : {
        cout << "Lines" << endl;
        selected_points_.push_back(mouse_down);
        selected_points_.push_back(mouse_up);
        correction_type_ = CorrectionType::kLineSegmentCorrection;
        pending_correction_type_ = CorrectionType::kUnknownCorrection;
      } break;
      case CorrectionType::kColinearCorrection : {
        cout << "Colinear" << endl;
        selected_points_.push_back(mouse_down);
        selected_points_.push_back(mouse_up);
        correction_type_ = CorrectionType::kColinearCorrection;
        pending_correction_type_ = CorrectionType::kUnknownCorrection;
      } break;
      case CorrectionType::kPerpendicularCorrection : {
        cout << "Perpendicular" << endl;
        selected_points_.push_back(mouse_down);
        selected_points_.push_back(mouse_up);
        correction_type_ = CorrectionType::kPerpendicularCorrection;
        pending_correction_type_ = CorrectionType::kUnknownCorrection;
      } break;
      case CorrectionType::kParallelCorrection : {
        cout << "Parallel" << endl;
        selected_points_.push_back(mouse_down);
        selected_points_.push_back(mouse_up);
        correction_type_ = CorrectionType::kParallelCorrection;
        pending_correction_type_ = CorrectionType::kUnknownCorrection;
      } break;
      default : {
        // Do nothing.
      } break;
    }
    const auto name_index = static_cast<size_t>(correction_type);
    cout << "Assigning correction mode: " << CorrectionTypeNames[name_index] << endl;
    correction_type_ = correction_type;
  }
}

size_t HitLSLAM::verifyUserInput() {
  size_t points_verified = 0;
  float local_select_thresh = 0.05;
  bool seen;
  for (size_t i=0; i<selected_points_.size(); i++) {
    seen = false;
    for (size_t j=0; j<WORLD_FRAME_point_clouds_.size(); j++) {
      for (size_t k=0; k<WORLD_FRAME_point_clouds_[j].size(); k++) {
        if ((WORLD_FRAME_point_clouds_[j][k] - selected_points_[i]).norm() < local_select_thresh) {
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

void HitLSLAM::transformPointCloudsToWorldFrame() {
  for (size_t i=0; i<ROBOT_FRAME_point_clouds_.size(); i++) {
    const Rotation2Df rotation(poses_[i].angle);
    const Translation2f translation(poses_[i].translation);
    const Affine2f pose_transform = translation * rotation;
    for (size_t j=0; j < ROBOT_FRAME_point_clouds_[i].size(); j++) {
      WORLD_FRAME_point_clouds_[i][j] = pose_transform * ROBOT_FRAME_point_clouds_[i][j];
    }
  }
}

void HitLSLAM::resetCorrectionInputs() {
  selected_points_.clear();
  pending_correction_type_ = CorrectionType::kUnknownCorrection;
  correction_type_ = CorrectionType::kUnknownCorrection;
}

bool HitLSLAM::undo() {
  if (input_history_.size() == 0) {
    cout << "Nothing to undo." << endl;
    return false;
  }
  if (input_history_.back().undone) {
    cout << "Already undone to max depth (1)." << endl;
    return false;
  }
  poses_ = prev_poses_;
  covariances_ = prev_covariances_;
  input_history_.back().undone = 1;
  human_constraints_.pop_back();
  return true;
}

void HitLSLAM::replayLog(const SingleInput logged_input) {
  correction_type_ = logged_input.type_of_constraint;
  selected_points_ = logged_input.input_points;
  size_t points_verified = verifyUserInput();
  cout << "points verified: " << points_verified << endl;

  if (points_verified == selected_points_.size()) {

    prev_poses_ = poses_;
    prev_covariances_ = covariances_;
    //SingleInput current_input;
    //current_input.type_of_constraint = correction_type_;
    //current_input.input_points = selected_points_;
    //current_input.undone = 0;
    //input_history_.push_back(current_input);
    //printf("correction type: %d\n", input_history_.back().type_of_constraint);
      
    em_input_.local_version_point_clouds_ = WORLD_FRAME_point_clouds_;
    em_input_.selected_points_ = selected_points_;
    em_input_.correction_type_ = correction_type_;
    
    em_input_.Run();

    std::vector<int> corrected_poses = em_input_.corrected_poses_;
    std::vector<int> anchor_poses = em_input_.anchor_poses_;
    std::pair<int, int> backprop_bounds = em_input_.backprop_bounds_;
    std::vector<Eigen::Vector2f> new_selected_points = em_input_.selected_points_;
      
    if (backprop_bounds.first >= 0 && backprop_bounds.second >= 1) {
      app_exp_corr_.correction_type_ = correction_type_;
      app_exp_corr_.selected_points_ = new_selected_points;
      app_exp_corr_.corrected_poses_ = corrected_poses;
      app_exp_corr_.anchor_poses_ = anchor_poses;
      app_exp_corr_.poses_ = poses_;
   
      app_exp_corr_.Run();
    
      poses_ = app_exp_corr_.poses_;
      Eigen::Vector3f correction = app_exp_corr_.correction_;
      vector<HumanConstraint> new_human_constraints;

      new_human_constraints = app_exp_corr_.new_human_constraints_;
      human_constraints_.push_back(new_human_constraints);
      
      //cout << "BP bounds: " << backprop_bounds.first << ", " << backprop_bounds.second << endl;
      
      backprop_.poses_ = poses_;
      backprop_.correction_ = correction;
      backprop_.backprop_bounds_ = backprop_bounds;
      backprop_.covariances_ = covariances_;
      
      backprop_.Run();
      
      covariances_ = backprop_.covariances_;
      poses_ = backprop_.poses_;

      //ceres_gradients_.clear();
      //jacobian_init_ = false;
        
      // make sure angles are all within the same 2*pi range
      for (size_t i = 0; i < poses_.size(); ++i) { 
        float angle = poses_[i].angle;
        //std::cout << angle << " ::: " << atan2(sin(angle), cos(angle)) << std::endl;
        poses_[i].angle = atan2(sin(angle), cos(angle));
      }
        
      //joint_opt_.orig_odom_ = odometry_;
      joint_opt_.poses_ = poses_;
      joint_opt_.robot_frame_point_clouds_ = ROBOT_FRAME_point_clouds_;
      joint_opt_.robot_frame_normal_clouds_ = normal_clouds_;
      joint_opt_.covariances_ = covariances_;
      joint_opt_.human_constraints_ = human_constraints_;
                  
      joint_opt_.Run();

      poses_ = joint_opt_.poses_;

      //ceres_gradients_ = joint_opt_.gradients_;
      //ceres_jacobian_ = joint_opt_.ceres_jacobian_;
      //jacobian_init_ = true;
      //num_hc_residuals_ = joint_opt_.num_hc_residuals_;
      //std::cout << "num hc residuals: " << num_hc_residuals_ << std::endl;
      
      //ClearDrawingMessage(&display_message_);
      //DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
    }
  }
    
  cout << "completed cycle" << endl;
  num_completed_cycles++;

  resetCorrectionInputs();
}





void HitLSLAM::Run() {
  if (selected_points_.size() == 0 || 
      pending_correction_type_ != CorrectionType::kUnknownCorrection) {
    cout << "Incomplete correction specification" << endl;
    return;
  } 
  size_t points_verified = verifyUserInput();
  cout << "points verified: " << points_verified << endl;

  if (points_verified == selected_points_.size()) {

    prev_poses_ = poses_;
    prev_covariances_ = covariances_;
    SingleInput current_input;
    //printf("correction type: %d\n", static_cast<size_t>(correction_type_));
    current_input.type_of_constraint = correction_type_;
    current_input.input_points = selected_points_;
    current_input.undone = 0;
    input_history_.push_back(current_input);
    printf("correction type: %d\n", input_history_.back().type_of_constraint);
      
    em_input_.local_version_point_clouds_ = WORLD_FRAME_point_clouds_;
    em_input_.selected_points_ = selected_points_;
    em_input_.correction_type_ = correction_type_;
    
    em_input_.Run();

    std::vector<int> corrected_poses = em_input_.corrected_poses_;
    std::vector<int> anchor_poses = em_input_.anchor_poses_;
    std::pair<int, int> backprop_bounds = em_input_.backprop_bounds_;
    std::vector<Eigen::Vector2f> new_selected_points = em_input_.selected_points_;
      
    if (backprop_bounds.first >= 0 && backprop_bounds.second >= 1) {
      app_exp_corr_.correction_type_ = correction_type_;
      app_exp_corr_.selected_points_ = new_selected_points;
      app_exp_corr_.corrected_poses_ = corrected_poses;
      app_exp_corr_.anchor_poses_ = anchor_poses;
      app_exp_corr_.poses_ = poses_;
   
      app_exp_corr_.Run();
    
      poses_ = app_exp_corr_.poses_;
      Eigen::Vector3f correction = app_exp_corr_.correction_;
      vector<HumanConstraint> new_human_constraints;

      new_human_constraints = app_exp_corr_.new_human_constraints_;
      human_constraints_.push_back(new_human_constraints);
      
      //cout << "BP bounds: " << backprop_bounds.first << ", " << backprop_bounds.second << endl;
      
      backprop_.poses_ = poses_;
      backprop_.correction_ = correction;
      backprop_.backprop_bounds_ = backprop_bounds;
      backprop_.covariances_ = covariances_;
      
      backprop_.Run();
      
      covariances_ = backprop_.covariances_;
      poses_ = backprop_.poses_;

      //ceres_gradients_.clear();
      //jacobian_init_ = false;
        
      // make sure angles are all within the same 2*pi range
      for (size_t i = 0; i < poses_.size(); ++i) { 
        float angle = poses_[i].angle;
        //std::cout << angle << " ::: " << atan2(sin(angle), cos(angle)) << std::endl;
        poses_[i].angle = atan2(sin(angle), cos(angle));
      }
        
      //joint_opt_.orig_odom_ = odometry_;
      joint_opt_.poses_ = poses_;
      joint_opt_.robot_frame_point_clouds_ = ROBOT_FRAME_point_clouds_;
      joint_opt_.robot_frame_normal_clouds_ = normal_clouds_;
      joint_opt_.covariances_ = covariances_;
      joint_opt_.human_constraints_ = human_constraints_;
                  
      joint_opt_.Run();

      poses_ = joint_opt_.poses_;

      //ceres_gradients_ = joint_opt_.gradients_;
      //ceres_jacobian_ = joint_opt_.ceres_jacobian_;
      //jacobian_init_ = true;
      //num_hc_residuals_ = joint_opt_.num_hc_residuals_;
      //std::cout << "num hc residuals: " << num_hc_residuals_ << std::endl;
      
      //ClearDrawingMessage(&display_message_);
      //DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
    }
    else {
      // TODO: error message about backprop
      //cout << "Erroro: " << endl;
    }
  }
  else {
    cout << "User input was not verified as close to observations" << endl;
  }

  transformPointCloudsToWorldFrame(); 
  cout << "completed cycle" << endl;
  num_completed_cycles++;

  resetCorrectionInputs();
}



