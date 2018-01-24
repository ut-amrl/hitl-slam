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
// Interface for non-Markov Localization.

#ifndef BACKPROP_H
#define BACKPROP_H

//#include <pthread.h>
//#include <semaphore.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>

//#include "ceres/ceres.h"
//#include "geometry.h"
//#include "kdtree.h"
#include "../../perception_tools/perception_2d.h"
//#include "pthread_utils.h"
//#include "vector_map.h"
//#include "vector_mapping.h"



//namespace vector_localization {

class Backprop {
 public:

  explicit Backprop();
  virtual ~Backprop();

  void Run();
  
  std::pair<int, int> backprop_bounds_;

  // 3x3 Covariance matrix for each pose.
  std::vector<Eigen::Matrix3d> d3_covariances_;
  
  // Poses of every node.
  std::vector<perception_2d::Pose2Df> poses_;

  // correction to be fixed via COP-SLAM
  Eigen::Vector3f correction_;

 private:

  void LoadMapState();

  void SaveMapState();

  void SaveBounds();

  void LoadBackprop();

  void BackPropagateError();

};

//}  // namespace vector_localization

#endif  // backprop_H
