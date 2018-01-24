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

#ifndef APPLYEXPLICITCORRECTION_H
#define APPLYEXPLICITCORRECTION_H

//#include <pthread.h>
//#include <semaphore.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>

//#include "ceres/ceres.h"
//#include "geometry.h"
//#include "kdtree.h"
#include "../perception_tools/perception_2d.h"
//#include "pthread_utils.h"
//#include "vector_map.h"
//#include "../vmapping/vector_mapping.h"

#include "human_constraints.h"

//namespace vector_localization {

class AppExpCorrect {
 public:

  typedef std::pair<int,int> IntPair;
  typedef std::pair<Eigen::Vector2f, Eigen::Vector2f> Vector2fPair;
  typedef std::pair<IntPair, Vector2fPair> ConstraintPair;
  typedef std::pair<int, Eigen::Vector3f> CorrectionPair;

  explicit AppExpCorrect();
  virtual ~AppExpCorrect();

  void Run();

  CorrectionType correction_type_;

  std::vector<Eigen::Vector2f> selected_points_;

  // list of indices of corrected poses
  std::vector<int> corrected_poses_;
  
  std::vector<int> anchor_poses_;

  // Poses of every node.
  std::vector<perception_2d::Pose2Df> poses_;
  
  Eigen::Vector3f correction_;
  
  std::vector<HumanConstraint> new_human_constraints_;
  
 private:
  void SaveMapState();
  void LoadMapState();

  void LoadCorrections();
  void LoadCorrectings();
  void SaveBackprop(Eigen::Vector3f C);
  void SaveHumanConstraints();

  Eigen::Vector3f AppExpCorrections();

  int AddPointToPointHumanConstraint(
         std::vector<Eigen::Vector2f> selected_points,
         std::vector<CorrectionPair>* corrections);

  void AddLineToLineHumanConstraint(
         std::vector<CorrectionPair>* corrections);

  void AddCornerToCornerHumanConstraint(
         std::vector<Eigen::Vector2f> selected_points,
         const std::vector<perception_2d::Pose2Df>& poses,
         std::vector<CorrectionPair>* corrections);

  void AddColinearHumanConstraint(
         std::vector<CorrectionPair>* corrections);

  void AddPerpendicularHumanConstraint(
         std::vector<CorrectionPair>* corrections);

  void AddParallelHumanConstraint(
         std::vector<CorrectionPair>* corrections);

  void AddHumanCorrections(std::vector<perception_2d::Pose2Df>* poses,
                           const std::vector<Eigen::Vector2f> selected_points,
                           const int mode,
                           std::vector<HumanConstraint>&
                           human_constraints);

  void CalculateExplicitCorrections(std::vector<CorrectionPair>* corrections);


  void FindContiguousGroups(size_t min_poses, size_t max_poses,
                            std::vector<CorrectionPair>* corrections,
                            std::vector<std::vector<CorrectionPair>>*
                            contiguous_corrections);

  int ApplyExplicitCorrections(size_t i,
                               std::vector<std::vector<CorrectionPair>>*
                               contiguous_corrections);

  void calculateConstraintTargets();

};

//}  // namespace vector_localization

#endif  // app exp corr _H
