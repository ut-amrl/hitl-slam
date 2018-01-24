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
// Implementation of non-Markov Localization.

#include "ApplyExplicitCorrection.h"

#include <dirent.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/SVD>
#include <fstream>
#include <glog/logging.h>
#include <utility>
#include <vector>
#include <boost/concept_check.hpp>

//#include "ceres/ceres.h"
//#include "ceres/dynamic_autodiff_cost_function.h"
#include "../map/vector_map.h"
#include "../perception_tools/perception_2d.h"
#include "../shared/math/eigen_helper.h"
#include "../shared/math/util.h"
#include "../shared/util/helpers.h"
#include "../shared/util/pthread_utils.h"
#include "../shared/util/timer.h"
//#include "vector_localization/residual_functors.h"
//#include <vectorparticlefilter.h>
#include "../vmapping/vector_mapping.h"
// #define ENABLE_TIMING

#ifdef ENABLE_TIMING
  #define TIME_FUNCTION FunctionTimer ft(__FUNCTION__);
#else
  #define TIME_FUNCTION ;
#endif

//using ceres::AutoDiffCostFunction;
//using ceres::DynamicAutoDiffCostFunction;
//using ceres::IterationCallback;
//using ceres::IterationSummary;
using Eigen::Affine2f;
using Eigen::DistanceToLineSegment;
using Eigen::Matrix2f;
using Eigen::Matrix2d;
using Eigen::Matrix3f;
using Eigen::Matrix3d;
using Eigen::Rotation2Df;
using Eigen::ScalarCross;
using Eigen::Vector2f;
using Eigen::Vector2d;
using Eigen::Vector3f;
using Eigen::Vector3d;
//using EnmlMaps::PersistentObject;
using perception_2d::NormalCloudf;
using perception_2d::PointCloudf;
using perception_2d::Pose2Df;
using std::cout;
using std::endl;
using std::make_pair;
using std::max;
using std::pair;
using std::size_t;
using std::string;
using std::vector;
using vector_localization::CorrectionType;
using vector_localization::CorrectionTypeNames;
//using vector_localization::LTSConstraint;
//using vector_localization::PointToLineConstraint;
//using vector_localization::PoseConstraint;

typedef Eigen::Translation<float, 2> Translation2Df;

namespace vector_localization {

AppExpCorrect::AppExpCorrect() {}

AppExpCorrect::~AppExpCorrect() {}

/*
int AppExpCorrect::AddPointToPointHumanConstraint(
                                     vector<Vector2f> selected_points,
                                     vector<CorrectionPair>* corrections) {
  //determine which poses are affected by inputs, and which constraints to apply
  CHECK_EQ(selected_points.size(), 2);
  vector<pair<double, pair <int, int>>> dst_to_POIs;
  for (size_t i=0; i<selected_points.size(); i++) {
    IntPair obs_id = make_pair(-1, -1);
    pair<double, IntPair > closest_obs =
        make_pair(std::numeric_limits<double>::infinity(), obs_id);
    dst_to_POIs.push_back(closest_obs);
  }
  for (size_t i=0; i<robot_frame_point_clouds_.size(); i++) {
    for (size_t j=0; j<robot_frame_point_clouds_[i].size(); j++) {
      for (size_t k=0; k<dst_to_POIs.size(); k++) {
        double dst = (selected_points[k] -
                      robot_frame_point_clouds_[i][j]).norm();
        if (dst_to_POIs[k].first > dst) {
          dst_to_POIs[k].first = dst;
          dst_to_POIs[k].second.first = i;
          dst_to_POIs[k].second.second = j;
        }
      }
    }
  }
  if (dst_to_POIs[0].second.first < dst_to_POIs[1].second.first) {//switch order
    pair<double, pair <int, int>> temp = dst_to_POIs[0];
    dst_to_POIs[0] = dst_to_POIs[1];
    dst_to_POIs[1] = temp;
  }

  const Vector2f& A =
      robot_frame_point_clouds_[dst_to_POIs[0].second.first]
      [dst_to_POIs[0].second.second];
  const Vector2f& B =
      robot_frame_point_clouds_[dst_to_POIs[1].second.first]
      [dst_to_POIs[1].second.second];
  const Vector2f partial_correction = B - A;
  const Eigen::Vector3f C(partial_correction(0), partial_correction(1), 0.0);
  CorrectionPair new_correction = make_pair(dst_to_POIs[0].second.first, C);
  corrections->push_back(new_correction);
  return dst_to_POIs[1].second.first;
}
*/

void AppExpCorrect::AddLineToLineHumanConstraint(vector<CorrectionPair>*
                                               corrections) {
  CHECK_EQ(selected_points_.size(), 4);
  //determine transformation between lines
  Vector2f cmA = (selected_points_[1] + selected_points_[0])/2.0;
  Vector2f cmB = (selected_points_[3] + selected_points_[2])/2.0;

  Vector2f A(selected_points_[1] - selected_points_[0]);
  A.normalize();
  Vector2f B(selected_points_[3] - selected_points_[2]);
  B.normalize();
  double theta = acos(A.dot(B));

  Eigen::Vector3f A3(A(0), A(1), 0);
  Eigen::Vector3f B3(B(0), B(1), 0);
  if (A3.cross(B3)(2) < 0.0) { //A cross B
    theta = -theta;
  }
  // rotation matrix R(theta)
  const Rotation2Df R(theta);


  for (size_t i = 0; i < corrected_poses_.size(); ++i) {
    int pose_index = corrected_poses_[i];
    Vector2f p0(poses_[pose_index].translation);
    Vector2f p1 = cmB + R*(p0 - cmA);
    Vector2f T = p1-p0;
    Eigen::Vector3f c(T(0), T(1), theta);
    CorrectionPair new_correction = make_pair(pose_index, c);
    corrections->push_back(new_correction);
  }
}
/*
void AppExpCorrect::AddCornerToCornerHumanConstraint(
                                        vector<Vector2f> selected_points,
                                        const vector<Pose2Df>& poses,
                                        vector<CorrectionPair>* corrections) {
  CHECK_EQ(selected_points.size(), 8);
  //transforming A to B
  Vector2f pA1 = selected_points[0];
  Vector2f pA2 = selected_points[1];
  Vector2f pA3 = selected_points[2];
  Vector2f pA4 = selected_points[3];

  Vector2f pB1 = selected_points[4];
  Vector2f pB2 = selected_points[5];
  Vector2f pB3 = selected_points[6];
  Vector2f pB4 = selected_points[7];

  Vector2f cmA = (pA1 + pA2 + pA3 + pA4)/4.0;
  Vector2f cmB = (pB1 + pB2 + pB3 + pB4)/4.0;
  Eigen::Matrix2f H =
      (pA1 - cmA) * (pB1 - cmB).transpose() +
      (pA2 - cmA) * (pB2 - cmB).transpose() +
      (pA3 - cmA) * (pB3 - cmB).transpose() +
      (pA4 - cmA) * (pB4 - cmB).transpose();

  Eigen::JacobiSVD<Eigen::Matrix2f> svd(
      H, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix2f R = svd.matrixV() * svd.matrixU().transpose();

  double theta = acos(R(0,0)); //radians
  if (R(1,0) < 0) { //make sure theta has the right sign
    theta = -theta;
  }


  for (size_t i = 0; i < corrected_poses_.size(); ++i) {
    int pose_index = corrected_poses_[i];
    Vector2f p0(poses[pose_index].translation);
    Vector2f p1 = cmB + R*(p0 - cmA);
    Vector2f T = p1-p0;
    Eigen::Vector3f c(T(0), T(1), theta);
    CorrectionPair new_correction = make_pair(pose_index, c);
    corrections->push_back(new_correction);
  }
}
*/

void AppExpCorrect::AddColinearHumanConstraint(vector<CorrectionPair>*
                                               corrections) {
  CHECK_EQ(selected_points_.size(), 4);
  //determine transformation between lines
  const Vector2f cmA = 0.5 * (selected_points_[1] + selected_points_[0]);
  const Vector2f cmB = 0.5 * (selected_points_[3] + selected_points_[2]);
  const Vector2f A = (selected_points_[1] - selected_points_[0]).normalized();
  const Vector2f B = (selected_points_[3] - selected_points_[2]).normalized();
  const float theta =
      (ScalarCross(A, B) >= 0.0) ? (acos(A.dot(B))) : (-acos(A.dot(B)));
  const Rotation2Df R(theta);

  // project A onto B
  const float alpha = (cmA - cmB).dot(B);
  // new_cmA is colinear with B's endpoints
  const Vector2f new_cmA = cmB + alpha * B;



  for (size_t i = 0; i < corrected_poses_.size(); ++i) {
    int pose_index = corrected_poses_[i];
    Vector2f p0(poses_[pose_index].translation);
    Vector2f p1 = new_cmA + R*(p0 - cmA);
    Vector2f T = p1-p0;
    Eigen::Vector3f c(T(0), T(1), theta);
    CorrectionPair new_correction = make_pair(pose_index, c);
    corrections->push_back(new_correction);
  }
}

void AppExpCorrect::AddPerpendicularHumanConstraint(vector<CorrectionPair>*
                                                        corrections) {
  CHECK_EQ(selected_points_.size(), 4);
  //determine transformation between lines
  Vector2f cmA = (selected_points_[1] + selected_points_[0])/2.0;
  Vector2f A(selected_points_[1] - selected_points_[0]);
  A.normalize();
  Vector2f B(selected_points_[3] - selected_points_[2]);
  B.normalize();

  Eigen::Vector3f A3(A(0), A(1), 0);
  Eigen::Vector3f B3(B(0), B(1), 0);

  double theta = 0.0;
  //A cross B
  if (A3.cross(B3)(2) < 0.0) { theta = -acos(A.dot(B)); }
  else { theta = acos(A.dot(B)); }

  if (theta == M_PI/2.0 || theta == - M_PI/2.0) { theta = 0.0; }
  else if (theta > 0.0) { theta = -(-theta + M_PI/2.0); }
  else { theta = -(-theta - M_PI/2.0); }

  // Rotation matrix R(theta)
  const Rotation2Df R(theta);

  for (size_t i = 0; i < corrected_poses_.size(); ++i) {
    int pose_index = corrected_poses_[i];
    Vector2f p0(poses_[pose_index].translation);
    Vector2f p1 = cmA + R*(p0 - cmA);
    Vector2f T = p1-p0;
    Eigen::Vector3f c(T(0), T(1), theta);
    CorrectionPair new_correction = make_pair(pose_index, c);
    corrections->push_back(new_correction);
  }
}

void AppExpCorrect::AddParallelHumanConstraint(
                                    vector<CorrectionPair>* corrections) {
  CHECK_EQ(selected_points_.size(), 4);
  //determine transformation between lines
  const Vector2f cmA = 0.5 * (selected_points_[1] + selected_points_[0]);
  const Vector2f A = (selected_points_[1] - selected_points_[0]).normalized();
  const Vector2f B = (selected_points_[3] - selected_points_[2]).normalized();
  const float theta =
      (ScalarCross(A, B) >= 0.0) ? (acos(A.dot(B))) : (-acos(A.dot(B)));
  const Rotation2Df R(theta);


  for (size_t i = 0; i < corrected_poses_.size(); ++i) {
    int pose_index = corrected_poses_[i];
    Vector2f p0(poses_[pose_index].translation);
    Vector2f p1 = cmA + R*(p0 - cmA);
    Vector2f T = p1 - p0;
    Eigen::Vector3f c(T(0), T(1), theta);
    CorrectionPair new_correction = make_pair(pose_index, c);
    corrections->push_back(new_correction);
  }
}

void AppExpCorrect::CalculateExplicitCorrections(vector<CorrectionPair>*
                                                    corrections) {
  switch (correction_type_) {
    case CorrectionType::kPointCorrection : {
      cout << "not currently supported" << endl;
      //CHECK_EQ(selected_points.size(), 2);
      //backprop_start = AddPointToPointHumanConstraint(selected_points,
      //                                                corrections);
    } break;
    case CorrectionType::kLineSegmentCorrection : {
      CHECK_EQ(selected_points_.size(), 4);
      AddLineToLineHumanConstraint(corrections);
    } break;
    case CorrectionType::kCornerCorrection : {
      cout << "not currently supported" << endl;
      //CHECK_EQ(selected_points.size(), 8);
      //AddCornerToCornerHumanConstraint(selected_points, poses, corrections);
    } break;
    case CorrectionType::kColinearCorrection : {
      CHECK_EQ(selected_points_.size(), 4);
      AddColinearHumanConstraint(corrections);
    } break;
    case CorrectionType::kPerpendicularCorrection : {
      CHECK_EQ(selected_points_.size(), 4);
      AddPerpendicularHumanConstraint(corrections);
    } break;
    case CorrectionType::kParallelCorrection : {
      CHECK_EQ(selected_points_.size(), 4);
      AddParallelHumanConstraint(corrections);
    } break;
    default : {
      fprintf(stderr, "Error: Unknown correction HiTL correction mode.\n");
    }
  }
}

// currently we do not take advantage of this info, just a place holder for now
void AppExpCorrect::FindContiguousGroups(size_t min_poses, size_t max_poses,
                                         vector<CorrectionPair>* corrections,
                                         vector<vector<CorrectionPair>>*
                                         contiguous_corrections) {

  // figure which poses belong to which contiguous groups
  vector<CorrectionPair> one_group;
  for (auto i = min_poses; i <= max_poses; ++i) {
    bool in_group = false;
    int pose_index;
    for (size_t j = 0; j < corrections->size(); j++) {
      if (corrections[0][j].first == static_cast<int>(i)) {
        in_group = true;
        pose_index = j;
      }
    }

    if (in_group) {
      one_group.push_back(corrections[0][pose_index]);
    } else if (one_group.size() != 0) {
      contiguous_corrections->push_back(one_group);
      one_group.clear();
    }
  }
  if (one_group.size() != 0) {
    contiguous_corrections->push_back(one_group);
  }
}

int AppExpCorrect::ApplyExplicitCorrections(size_t i,
                                            vector<vector<CorrectionPair>>*
                                            contiguous_corrections) {

  for (size_t j = 0; j < contiguous_corrections[0][i].size(); ++j) {
    poses_[contiguous_corrections[0][i][j].first].translation.x() +=
                                      contiguous_corrections[0][i][j].second(0);
    poses_[contiguous_corrections[0][i][j].first].translation.y() +=
                                      contiguous_corrections[0][i][j].second(1);
    poses_[contiguous_corrections[0][i][j].first].angle +=
                                      contiguous_corrections[0][i][j].second(2);
  }

  int last_pose = contiguous_corrections[0][0].back().first;
  cout << "last group! group number " << i << endl;
  cout << "last pose number " << last_pose << endl;

  Vector3f last_correction = contiguous_corrections[0][i].back().second;
  for (size_t k = last_pose + 1; k < poses_.size(); ++k) {
    //calculate implicit new position for all poses beyond constraint
    poses_[k].angle += last_correction(2);
    Vector2f ab = poses_[k].translation - poses_[last_pose].translation;
    Rotation2Df rot = Rotation2Df(last_correction(2));
    Vector2f new_ab = rot*ab;
    poses_[k].translation = poses_[last_pose].translation + new_ab +
                             Vector2f(last_correction(0), last_correction(1));
  }
  return last_pose;
}

Vector3f AppExpCorrect::AppExpCorrections() {

  Vector3f C;
  size_t min_poses = 0;
  size_t max_poses = poses_.size()-1;

  //store corrections resulting from user adjustments to relative observations
  vector<CorrectionPair> corrections;
  cout << "Calculate explicit corrections" << endl;
  CalculateExplicitCorrections(&corrections);

  cout << "Finding contiguous groups" << endl;
  vector<vector<CorrectionPair>> contiguous_corrections;
  FindContiguousGroups(min_poses, max_poses, &corrections,
                                     &contiguous_corrections);

  //impose changes for each contiguous group.
  for (size_t i = 0; i < contiguous_corrections.size(); i++) {
    if (i == 0) { //TODO: handle multiple contiguous groups properly
      C = contiguous_corrections[i][0].second;
      //apply explicit pose corrections within one contiguous group
      cout << "Applying explicit constraints" << endl;
      int end_exp_corr;
      end_exp_corr = ApplyExplicitCorrections(i, &contiguous_corrections);
      cout << "End of expicit corrections: " << end_exp_corr << endl;
    }
  }
  return C;
}

void AppExpCorrect::calculateConstraintTargets() {
  new_human_constraints_.clear();
  Eigen::Vector2f correction_direction = selected_points_[3] - selected_points_[2];
  //std::cout << "SELECTED POINTS:\n" << selected_points_[3] << selected_points_[2] << std::endl;
  float correction_angle = atan2(correction_direction(1), correction_direction(0));
//   std::cout << "ANCHOR / CONNECTED POSES:" << std::endl;
//   for (size_t i = 0; i < anchor_poses_.size(); ++i) {
//     std::cout << anchor_poses_[i] << std::endl;
//   }
//   for (size_t i = 0; i < corrected_poses_.size(); ++i) {
//     std::cout << corrected_poses_[i] << std::endl;
//   }
  for (size_t i = 0; i < anchor_poses_.size(); ++i) {
    float anchor_angle = poses_[anchor_poses_[i]].angle;
    float rel_pen_dir = atan2(sin(correction_angle - anchor_angle), 
                              cos(correction_angle - anchor_angle)) + M_PI/2.0;
    Eigen::Vector2f anchor_loc = poses_[anchor_poses_[i]].translation;
    for (size_t j = 0; j < corrected_poses_.size(); ++j) {
      Eigen::Vector2f pose_loc = poses_[corrected_poses_[j]].translation;
      float pose_angle = poses_[corrected_poses_[j]].angle;
   
      Eigen::Vector2f pose_rel = pose_loc - anchor_loc;
      Eigen::Vector2f p = Eigen::Vector2f(cos(anchor_angle), sin(anchor_angle));
      Eigen::Vector2f n = Eigen::Vector2f(-p(1), p(0));
      
      float para_dist = p.dot(pose_rel);
      float perp_dist = n.dot(pose_rel);
      float angle_dist = atan2(sin(pose_angle - anchor_angle), cos(pose_angle - anchor_angle));
      
      HumanConstraint single_constraint;
      single_constraint.constraint_type = correction_type_;
      single_constraint.anchor_pose_id = anchor_poses_[i];
      single_constraint.constrained_pose_id = corrected_poses_[j];
      single_constraint.delta_angle = angle_dist;
      single_constraint.delta_parallel = para_dist;
      single_constraint.delta_perpendicular = perp_dist;
      single_constraint.relative_penalty_dir = rel_pen_dir;
      new_human_constraints_.push_back(single_constraint);
    }
  }
}

void AppExpCorrect::Run() {
  correction_ = AppExpCorrections();
  calculateConstraintTargets();
}

}  // namespace vector_localization
