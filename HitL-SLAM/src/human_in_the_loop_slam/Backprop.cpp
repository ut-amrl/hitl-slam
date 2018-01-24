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

#include "Backprop.h"

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

#include "ceres/ceres.h"
#include "ceres/dynamic_autodiff_cost_function.h"
//#include "../../map/vector_map.h"
#include "./../perception_tools/perception_2d.h"
#include "../../shared/math/eigen_helper.h"
#include "../../shared/math/util.h"
#include "../../shared/util/helpers.h"
#include "../../shared/util/pthread_utils.h"
#include "../../shared/util/timer.h"
//#include "vector_localization/residual_functors.h"
//#include <vectorparticlefilter.h>
//#include "../../vmapping/vector_mapping.h"

// #define ENABLE_TIMING

#ifdef ENABLE_TIMING
  #define TIME_FUNCTION FunctionTimer ft(__FUNCTION__);
#else
  #define TIME_FUNCTION ;
#endif

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
//using vector_localization::LTSConstraint;
//using vector_localization::PointToLineConstraint;
//using vector_localization::PoseConstraint;

typedef Eigen::Translation<float, 2> Translation2Df;

//namespace vector_localization {

Backprop::Backprop() {}

Backprop::~Backprop() {}

void Backprop::BackPropagateError() {
  int min_poses = backprop_bounds_.first;
//   int max_poses = backprop_bounds_.second + 1;
  int max_poses = backprop_bounds_.second;

  Vector2f destination = poses_[max_poses].translation +
                        Vector2f(correction_(0), correction_(1));

  float destination_rot_variance = 0.0001; //radians
  float destination_trans_variance = 0.001; //meters
  vector<float> rot_sigmas;
  vector<float> trans_sigmas;
  for (size_t i = 0; i < covariances_.size(); ++i) {
    rot_sigmas.push_back(covariances_[i](2,2));
    trans_sigmas.push_back((covariances_[i](0,0) +
                            covariances_[i](1,1))/2.0);
  }

  vector<float> rot_weights;
  vector<float> trans_weights;
  float sum_of_rot_var = 0.0;
  float sum_of_trans_var = 0.0;
  //tabulate variances
  for (int i = min_poses; i <= max_poses; ++i) {
    sum_of_rot_var += rot_sigmas[i];
    sum_of_trans_var += trans_sigmas[i];
  }
  //Fuse destination variance with current final pose variance
  sum_of_rot_var += destination_rot_variance;
  sum_of_trans_var += destination_trans_variance;

  //calculate weights
  for (int i = min_poses; i <= max_poses; ++i) {
    rot_weights.push_back(rot_sigmas[i]/sum_of_rot_var);
    trans_weights.push_back(trans_sigmas[i]/sum_of_trans_var);
  }

  //initialize alphas
  CHECK_EQ(rot_weights.size(), trans_weights.size());

  // calculate beta for variance updates
  float rot_beta = 1/(1 + (rot_sigmas[max_poses - 1] /
                               destination_rot_variance));
  float trans_beta = 1/(1 + (trans_sigmas[max_poses - 1] /
                                 destination_trans_variance));

  //update fused rotation uncertainty
  rot_sigmas[max_poses] = rot_sigmas[max_poses - 1] *
                              destination_rot_variance /
                              (rot_sigmas[max_poses - 1] +
                              destination_rot_variance);
  //update fused translation uncertainty
  trans_sigmas[max_poses] = trans_sigmas[max_poses - 1] *
                              destination_trans_variance /
                              (trans_sigmas[max_poses - 1] +
                              destination_trans_variance);

  cout << "trans_beta: " << trans_beta << endl;
  cout << "rot_beta: " << rot_beta << endl;

  // update covariance estimates
  for (int i = min_poses; i < max_poses; ++i) {
    covariances_[i](0,0) *= trans_beta;
    covariances_[i](0,1) *= trans_beta;
    covariances_[i](1,0) *= trans_beta;
    covariances_[i](1,1) *= trans_beta;

    covariances_[i](0,2) *= rot_beta;
    covariances_[i](0,2) *= rot_beta;
    covariances_[i](2,0) *= rot_beta;
    covariances_[i](2,1) *= rot_beta;

    covariances_[i](2,2) *= rot_beta;
  }

  //APPLY ROTATION
  float theta = correction_(2);
  float delta_theta;
  //figure out new pose locations
  for (int i = min_poses; i < max_poses; ++i) {
    delta_theta = rot_weights[i - min_poses] * theta;
    const Affine2f post_i_correction = Translation2Df(poses_[i].translation) *
                                       Rotation2Df(delta_theta) *
                                       Translation2Df(-poses_[i].translation);
    poses_[i].angle += delta_theta;
    for (auto k = i + 1; k <= max_poses; ++k) {
      poses_[k].angle += delta_theta;
      poses_[k].translation = (post_i_correction * poses_[k].translation);
    }
  }

  //APPLY TRANSLATION
  Vector2f trans = destination - poses_[max_poses].translation;

  Vector2f delta_trans;
  //figure out new pose locations
  for (int i = min_poses; i < max_poses; ++i) {
    delta_trans = trans_weights[i - min_poses] * trans;
    for (auto k = i + 1; k <= max_poses; ++k) {
      poses_[k].translation += delta_trans;
    }
  }
}

void Backprop::Run() {

  if (backprop_bounds_.first < backprop_bounds_.second) {
    BackPropagateError();
  }
  else {
    std::cout << "No poses between bounds. Cannot run backprop." << std::endl;
  }
}


//}  // namespace vector_localization
