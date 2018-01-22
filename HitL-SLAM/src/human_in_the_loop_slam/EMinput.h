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

#ifndef EMINPUT_H
#define EMINPUT_H

#include <pthread.h>
#include <semaphore.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>

//#include "ceres/ceres.h"
//#include "geometry.h"
//#include "kdtree.h"
//#include "perception_tools/perception_2d.h"
//#include "pthread_utils.h"
//#include "vector_map.h"
#include "../vmapping/vector_mapping.h"

namespace vector_localization {

class EMInput {
 public:

  explicit EMInput();
  virtual ~EMInput();

  void Run();
    
  // points input by the user to specify a correction
  std::vector<Eigen::Vector2f> selected_points_;
    
  // Local version of point clouds for every pose... WORLD FRAME
  std::vector<std::vector<Eigen::Vector2f>> local_version_point_clouds_;
  
  // pose ids for poses to be moved during explicit corrections
  std::vector<int> corrected_poses_;
 
  // pose ids for poses which anchor loop constraints
  std::vector<int> anchor_poses_;

  // bounds between which to use COP-SLAM TODO: inclusive?
  std::pair<int, int> backprop_bounds_;
  
  vector_localization::CorrectionType correction_type_;
  
 private:

  void OrderAndFilterUserInput();

  std::pair<std::vector<std::pair<int, std::vector<int>>>,
            std::vector<std::pair<int, std::vector<int>>>>
      EstablishObservationSets();

  void SetCorrectionRelations(std::vector<std::pair<int, std::vector<int>>>
                              first_poses_obs,
                              std::vector<std::pair<int, std::vector<int>>>
                              second_poses_obs);

  void AutomaticEndpointAdjustment();

  std::vector<Eigen::Vector2f> SegFitEM(double* p1, double* p2, double* cm,
                                                    double* data, int size);

  double distToLineSeg(Eigen::Vector2f p1,
                       Eigen::Vector2f p2,
                       Eigen::Vector2f p);

  // Local version of normal clouds for every pose... WORLD FRAME
  std::vector<std::vector<Eigen::Vector2f>> local_version_normal_clouds_;
  
};

}  // namespace vector_localization

#endif  // EMINPUT_H
