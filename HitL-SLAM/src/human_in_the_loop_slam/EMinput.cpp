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

#include "EMinput.h"

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
#include "../shared/math/eigen_helper.h"
//#include <fstream>
//#include <glog/logging.h>
//#include <utility>
#include <vector>
//#include <boost/concept_check.hpp>

#include "ceres/ceres.h"
#include "ceres/dynamic_autodiff_cost_function.h"
//#include "map/vector_map.h"
#include "../perception_tools/perception_2d.h"
//#include "shared/math/eigen_helper.h"
//#include "shared/math/util.h"
//#include "shared/util/helpers.h"
//#include "shared/util/pthread_utils.h"
//#include "shared/util/timer.h"
//#include "vector_localization/residual_functors.h"
//#include <vectorparticlefilter.h>
//#include "vector_mapping.h"

#include "human_constraints.h"

// #define ENABLE_TIMING

#ifdef ENABLE_TIMING
  #define TIME_FUNCTION FunctionTimer ft(__FUNCTION__);
#else
  #define TIME_FUNCTION ;
#endif

using ceres::AutoDiffCostFunction;
using ceres::IterationCallback;
using ceres::IterationSummary;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using Eigen::Affine2f;
using Eigen::DistanceToLineSegment;
using Eigen::Matrix2f;
using Eigen::Matrix2d;
using Eigen::Matrix3f;
using Eigen::Matrix3d;
using Eigen::Rotation2Df;
//using Eigen::ScalarCross;
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
//using vector_localization::CorrectionType;
//using vector_localization::CorrectionTypeNames;

typedef Eigen::Translation<float, 2> Translation2Df;


//namespace vector_localization {

EMInput::EMInput() {}

EMInput::~EMInput() {}

struct segDistResidualEM {
  segDistResidualEM(double px, double py, double cmx, double cmy,
                  double len, double N) : px_(px), py_(py), cmx_(cmx),
                                          cmy_(cmy), len_(len), N_(N) {}
  template <typename T> bool operator()(const T* const theta, T* residual)
                                        const {
        T partial_res;
        Eigen::Matrix<T, 2, 1> alpha(cos(theta[0]), sin(theta[0]));
        alpha.normalize();
        T p1[2];
        T p2[2];
        p1[0] = T(cmx_) + (T(len_))*T(alpha[0]);
        p1[1] = T(cmy_) + (T(len_))*T(alpha[1]);
        p2[0] = T(cmx_) - (T(len_))*T(alpha[0]);
        p2[1] = T(cmy_) - (T(len_))*T(alpha[1]);
        T t = ((T(px_)-p1[0])*(p2[0]-p1[0])+(T(py_)-p1[1])*(p2[1]-p1[1])) /
               (pow(p2[0]-p1[0],2)+pow(p2[1]-p1[1],2));

//centroid1 = sqrt(pow(T(cmx_)-p1[0],2)+pow(T(cmy_)-p1[1],2)); //dist p1 to cm
//centroid2 = sqrt(pow(T(cmx_)-p2[0],2)+pow(T(cmy_)-p2[1],2)); //dist p2 to cm
    if (t < 0.0) {// Beyond the 'p1' end of the segment
      partial_res = sqrt(pow(T(px_)-p1[0],2)+pow(T(py_)-p1[1],2));
    }
    else if (t > 1.0) { // Beyond the 'p2' end of the segment
      partial_res = sqrt(pow(T(px_)-p2[0],2)+pow(T(py_)-p2[1],2));
    }
    else {
      T projx = p1[0] + t*(p2[0]-p1[0]);  // Projection falls on the segment
      T projy = p1[1] + t*(p2[1]-p1[1]);  // Projection falls on the segment
      partial_res = sqrt(pow((T(px_) - projx),2) + pow((T(py_) - projy),2));
    }
    residual[0] = partial_res; // + T(10.0)*centroid1/(N_) +
                               // T(10.0)*centroid2/(N_);
    return true;
  }
  private:
  const double px_;
  const double py_;
  const double cmx_;
  const double cmy_;
  const double len_;
  const double N_;
 };


vector<Vector2f> EMInput::SegFitEM(double* p1, double* p2, double* cm, double*
data, int size) {
  vector<Vector2f> fit;
  Vector2f ep1;
  Vector2f ep2;
  double icm[2];
  icm[0] = (p1[0] + p2[0]) / 2.0;
  icm[1] = (p1[1] + p2[1]) / 2.0;
  double hy = sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
  double ad = fabs(p1[0] - p2[0]);
  double theta[1];
  theta[0] = acos(ad/hy);
  Problem problem;

  for (int i = 0; i<size; ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<segDistResidualEM, 1, 1>(
        new segDistResidualEM(data[2*i], data[2*i + 1], icm[0], icm[1], hy/2.0,
                       double(size))), NULL, theta);
  }

  Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  Vector2d alpha(cos(theta[0]), sin(theta[0]));
  alpha.normalize();

  ep1[0] = icm[0] + (hy/2.0)*alpha[0];
  ep1(1) = icm[1] + (hy/2.0)*alpha[1];
  ep2(0) = icm[0] - (hy/2.0)*alpha[0];
  ep2(1) = icm[1] - (hy/2.0)*alpha[1];

  fit.push_back(ep1);
  fit.push_back(ep2);
  return fit;
}



void EMInput::AutomaticEndpointAdjustment() {
  //automatic endpoint adjustment
  // TODO this needs to handle "corner" entries
  for (size_t k = 0; k < selected_points_.size()/2; k++) {
    double thresh = 0.05;
    double adjustment1 = 2*thresh;
    double adjustment2 = 2*thresh;
    while (adjustment1 > thresh || adjustment2 > thresh) {
      Vector2f cm(0.0, 0.0);
      int num_inliers = 0;
      vector<Vector2f> inlier_set;
      double threshold = 0.03; //meters
      for (size_t i=0; i<local_version_point_clouds_.size(); i++) {
        for (size_t j=0; j < local_version_point_clouds_[i].size(); j++) {
          double dst = DistanceToLineSegment(selected_points_[2*k],
                                             selected_points_[2*k+1],
                                             local_version_point_clouds_[i][j]);
          if (dst < threshold) {
            num_inliers++;
            inlier_set.push_back(local_version_point_clouds_[i][j]);
            cm += local_version_point_clouds_[i][j];
          }
        }
      }
    
//       std::cout << selected_points_[2*k][0] << ", " << selected_points_[2*k][1] << std::endl;
//       std::cout << selected_points_[2*k+1][0] << ", " << selected_points_[2*k+1][1] << std::endl; 
//       std::cout << "NUM INLIERS: " << num_inliers << std::endl;
    
      cm = cm / double(num_inliers);
      //cout << "num inliers: " << num_inliers << endl;
      double CM[2];
      CM[0] = double(cm(0));
      CM[1] = double(cm(1));
      double P1[2];
      double P2[2];
      P1[0] = double(selected_points_[2*k][0]);
      P1[1] = double(selected_points_[2*k][1]);
      P2[0] = double(selected_points_[2*k+1][0]);
      P2[1] = double(selected_points_[2*k+1][1]);
      double data[2*num_inliers];
      for (int j = 0; j < num_inliers; j++) {
        data[2*j] = double(inlier_set[j][0]);
        data[2*j+1] = double(inlier_set[j][1]);
      }

      vector<Vector2f> new_endpoints = SegFitEM(P1, P2, CM, data, num_inliers);
      adjustment1 = (selected_points_[2*k] - new_endpoints[0]).norm();
      adjustment2 = (selected_points_[2*k+1] - new_endpoints[1]).norm();
      //cout << "adj1: " << adjustment1 << endl;
      //cout << "adj2: " << adjustment2 << endl;
      selected_points_[2*k] = new_endpoints[0];
      selected_points_[2*k+1] = new_endpoints[1];
    }
  } //end automatic end point adjustment
}


void EMInput::SetCorrectionRelations(vector<pair<int, vector<int>>>
                                           first_poses_obs,
                                           vector<pair<int, vector<int>>>
                                           second_poses_obs) {
  corrected_poses_.clear();
  anchor_poses_.clear();
  for (size_t i = 0; i < first_poses_obs.size(); ++i) {
    corrected_poses_.push_back(first_poses_obs[i].first);
    //std::cout << "corr pose: " << first_poses_obs[i].first << std::endl;
  }
  for (size_t i = 0; i < second_poses_obs.size(); ++i) {
    anchor_poses_.push_back(second_poses_obs[i].first);
    //std::cout << "anch pose: " << second_poses_obs[i].first << std::endl;
  }
}

double EMInput::distToLineSeg(Vector2f p1, Vector2f p2, Vector2f p) {
  float t = (p-p1).dot(p2-p1)/(p2-p1).dot(p2-p1);
  if (t < 0.0) {
    return double(sqrt((p-p1).dot(p-p1)));  // Beyond the 'v' end of the segment
  }
  else if (t > 1.0) {
    return double(sqrt((p-p2).dot(p-p2)));  // Beyond the 'w' end of the segment
  }
  Vector2f proj = p1 + t*(p2-p1);  // Projection falls on the segment
  return double(sqrt((p-proj).dot(p-proj)));
}

pair<vector<pair<int, vector<int>>>, vector<pair<int, vector<int>>>>
  EMInput::EstablishObservationSets() {

  vector<pair<int, vector<int>>> first_poses_observations;
  vector<pair<int, vector<int>>> second_poses_observations;

  double threshold = 0.03; //3cm 'pill-shaped' envelope around line
  for (size_t i=0; i<local_version_point_clouds_.size(); i++) {
    vector<int> first_obs;
    vector<int> second_obs;
    for (size_t j=0; j<local_version_point_clouds_[i].size(); j++) {
      // first selection
      const double dst_to_first_selection = distToLineSeg(
                                                 selected_points_[0],
                                                 selected_points_[1],
                                    local_version_point_clouds_[i][j]);
      if (dst_to_first_selection < threshold) {
        first_obs.push_back(j);
      }
      // second selection
      const double dst_to_second_selection = distToLineSeg(
                                                      selected_points_[2],
                                                      selected_points_[3],
                                          local_version_point_clouds_[i][j]);
      if (dst_to_second_selection < threshold) {
        second_obs.push_back(j);
      }
    } // end obs

    if (first_obs.size() > 5) {
      pair<int, vector<int>> selection = make_pair(i, first_obs);
      first_poses_observations.push_back(selection);
    }
    if (second_obs.size() > 5) {
      pair<int, vector<int>> selection = make_pair(i, second_obs);
      second_poses_observations.push_back(selection);
    }
  } // end pose
  pair<vector<pair<int, vector<int>>>, vector<pair<int, vector<int>>>>
      selected_poses = make_pair(first_poses_observations,
                                 second_poses_observations);
  return selected_poses;
}

void EMInput::OrderAndFilterUserInput() {
  int backprop_start = 0;
  int backprop_end = 0;
  CHECK_EQ(selected_points_.size(), 4);
  pair<vector<pair<int, vector<int>>>, vector<pair<int, vector<int>>>>
      selected_poses;
  selected_poses = EstablishObservationSets();

  vector<int> first_selection_poses;
  vector<int> second_selection_poses;
  for (size_t i = 0; i < selected_poses.first.size(); ++i) {
    first_selection_poses.push_back(selected_poses.first[i].first);
  }
  for (size_t i = 0; i < selected_poses.second.size(); ++i) {
    second_selection_poses.push_back(selected_poses.second[i].first);
  }

  size_t second_start = selected_points_.size()/2;
  int first_selection_min_pose;
  int first_selection_max_pose;
  int second_selection_min_pose;
  int second_selection_max_pose;

  vector<int> overlaps;
  for (size_t i = 0; i < second_selection_poses.size(); ++i) {
    for (size_t j = 0; j < first_selection_poses.size(); ++j) {
      if (second_selection_poses[i] == first_selection_poses[j]) {
        cout << "overlap pose: " << first_selection_poses[j] << endl;
        overlaps.push_back(first_selection_poses[j]);
      }
    }
  }

  if (overlaps.size() == first_selection_poses.size() &&
      overlaps.size() == second_selection_poses.size()) {
    cout << "ERROR: complete selection overlap." << endl;
    backprop_start = -1;
    backprop_end = -1;
  }
  else if (overlaps.size() == first_selection_poses.size()) {
    for (size_t i = 0; i < overlaps.size(); ++i) {
      second_selection_poses.erase(std::remove(second_selection_poses.begin(),
      second_selection_poses.end(),overlaps[i]),second_selection_poses.end());
    }
  }
  else if (overlaps.size() == second_selection_poses.size()) {
    for (size_t i = 0; i < overlaps.size(); ++i) {
      first_selection_poses.erase(std::remove(first_selection_poses.begin(),
        first_selection_poses.end(), overlaps[i]),first_selection_poses.end());
    }
  }
  else if (overlaps.size() > 0) {
    for (size_t i = 0; i < overlaps.size(); ++i) {
      first_selection_poses.erase(std::remove(first_selection_poses.begin(),
        first_selection_poses.end(), overlaps[i]), first_selection_poses.end());
      second_selection_poses.erase(std::remove(second_selection_poses.begin(),
        second_selection_poses.end(),overlaps[i]),second_selection_poses.end());
    }
  }
  first_selection_min_pose = first_selection_poses[0];
  first_selection_max_pose = first_selection_poses.back();
  second_selection_min_pose = second_selection_poses[0];
  second_selection_max_pose = second_selection_poses.back();

  vector<pair<int, vector<int>>> selected_poses_new_first;
  vector<pair<int, vector<int>>> selected_poses_new_second;
  pair<vector<pair<int, vector<int>>>, vector<pair<int, vector<int>>>>
      selected_poses_new;
  for (size_t i = 0; i < first_selection_poses.size(); ++i) {
    for (size_t j = 0; j < selected_poses.first.size(); ++j) {
      if (first_selection_poses[i] == selected_poses.first[j].first) {
        selected_poses_new_first.push_back(selected_poses.first[j]);
      }
    }
  }
  for (size_t i = 0; i < second_selection_poses.size(); ++i) {
    for (size_t j = 0; j < selected_poses.second.size(); ++j) {
      if (second_selection_poses[i] == selected_poses.second[j].first) {
        selected_poses_new_second.push_back(selected_poses.second[j]);
      }
    }
  }
  selected_poses_new = make_pair(selected_poses_new_first,
                                 selected_poses_new_second);

  if (first_selection_min_pose > second_selection_max_pose) { //user was good

    SetCorrectionRelations(selected_poses_new.first,
                           selected_poses_new.second);
    backprop_start = second_selection_max_pose + 1;
    backprop_end = first_selection_min_pose - 1;
  }
  else if (first_selection_max_pose < second_selection_min_pose) { //bad user
    vector<Vector2f> reordered_points;
    for (size_t i = second_start; i < selected_points_.size(); ++i) {
      reordered_points.push_back(selected_points_[i]);
    }
    for (size_t i = 0; i < second_start; ++i) {
      reordered_points.push_back(selected_points_[i]);
    }
    CHECK_EQ(reordered_points.size(), selected_points_.size());
    for (size_t i = 0; i < reordered_points.size(); ++i) {
      selected_points_[i] = reordered_points[i];
    }

    SetCorrectionRelations(selected_poses_new.second,
                           selected_poses_new.first);
    backprop_start = first_selection_max_pose + 1;
    backprop_end = second_selection_min_pose - 1;
  }
  else {
    cout << "ERROR: selection overlap." << endl;
    backprop_start = -1;
    backprop_end = -1;
  }

//   Rotation2Df rot = Rotation2Df(M_PI/2.0);
//   new_human_constraint_.len_a =(selected_points_[0]-selected_points_[1]).norm();
//   new_human_constraint_.len_b =(selected_points_[2]-selected_points_[3]).norm();
//   new_human_constraint_.cm_a = (selected_points_[0] + selected_points_[1])/2.0;
//   new_human_constraint_.cm_b = (selected_points_[2] + selected_points_[3])/2.0;
//   Vector2f normal_a = (selected_points_[1]-selected_points_[0]);
//   normal_a.normalize();
//   new_human_constraint_.n_a = rot*normal_a;
//   Vector2f normal_b = (selected_points_[3]-selected_points_[2]);
//   normal_b.normalize();
//   new_human_constraint_.n_b = rot*normal_b;

  backprop_bounds_.first = backprop_start;
  backprop_bounds_.second = backprop_end;
}

void EMInput::Run() {
  // Adjust theta for input lines. Write optimal fit into selected_points_.
  std::cout << "adjusting" << std::endl;
  AutomaticEndpointAdjustment();
  if (correction_type_ != CorrectionType::kPointCorrection && 
      correction_type_ != CorrectionType::kCornerCorrection) {
    //backprop start is the anchor pose for COP-SLAM
    //inside OrderAndFilterUserInput(), the correct sequence for
    //selected_points_ is determined, and all globals related to human
    //constraints are filled out.
    OrderAndFilterUserInput();
  }
  else {
    cout << "point to point and corner to corner not supported yet." << endl;
  }
}

//}  // namespace vector_localization
