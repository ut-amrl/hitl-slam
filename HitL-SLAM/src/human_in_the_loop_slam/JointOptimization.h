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

#ifndef JOINTOPT_H
#define JOINTOPT_H

#include <pthread.h>
#include <semaphore.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>

#include "ceres/ceres.h"
#include "geometry.h"
#include "../perception_tools/kdtree.h"
#include "../perception_tools/perception_2d.h"
#include "pthread_utils.h"
//#include "../map/vector_map.h"
#include "../episodic_non_markov_localization/vector_mapping.h"

#include <CImg.h>

//#include "../../extern_libraries/CImg/CImg.h"

//namespace vector_localization {

#include "human_constraints.h"

class JointOpt {
 public:

  explicit JointOpt();
  virtual ~JointOpt();

  void Run();

  // Clears the current list of poses and associated observations.
  void ClearPoses();

  std::vector<Eigen::Matrix3d> GetPoseCovariances();

  std::vector<float> GetCeresCost();
  
  // Poses of every node.
  std::vector<perception_2d::Pose2Df> poses_;
  
  // Point clouds for every pose node, in Eigen::Vector2f format. ROBOT FRAME
  std::vector<perception_2d::PointCloudf> robot_frame_point_clouds_;
  
  // Normal clouds for every pose node, in Eigen::Vector2f format. ROBOT FRAME
  std::vector<perception_2d::NormalCloudf> robot_frame_normal_clouds_;
  
  // 3x3 Covariance matrix for each pose.
  std::vector<Eigen::Matrix3d> d3_covariances_;
 
  // vector of vectors of pose to pose constraints
  std::vector<std::vector<HumanConstraint>> human_constraints_;
  
  // original relative odometry transforms
  //std::vector<perception_2d::Pose2Df> orig_odom_;
  
  // gradients from ceres
  std::vector<double> gradients_;

  // ceres Jacobian
  ceres::CRSMatrix ceres_jacobian_;
  
  // number of residuals from human constraints
  int num_hc_residuals_;
  
 private:

   void LoadOdom();

   void CopyParams();

   void SetParams();

   //bool LoadConfiguration(VectorMapping::VectorMappingOptions* options);

   void ConvertPointClouds();

   void HumanConstraintsInnerCopy();

   void CopyTempLaserScans();

   void WriteGradients();

   void WriteCorrespondences();

   void WriteJacobian(std::string jacobian_file);

   void WriteHumanResiduals();

   void WriteHCFitLines();

   Eigen::MatrixXd ConvertJ(ceres::CRSMatrix& jacobian);

  // Add a new pose node along with the associated observations. Returns the
  // number of pose nodes in use at the moment.
  void AddPose(const perception_2d::PointCloudf& point_cloud,
               const perception_2d::NormalCloudf& normal_cloud,
               const perception_2d::Pose2Df& relative_pose);

  // Find point to point STF correspondences for every point from every pose
  // to every other pose with overlapping scans. The points will be read from
  // the member variable point_clouds_, and the results of the correspondence
  // matching will be stored in the member variable
  // point_point_correspondences_.
  void FindSTFCorrespondences(const std::size_t min_poses,
                              const std::size_t max_poses);

  // Find correspondences between successive observations.
  void FindVisualOdometryCorrespondences(int min_poses, int max_poses);

  // Add point to point STF constraints based on previously computed
  // correspondences stored in the member variable point_point_correspondences_.
  // For each constraint, a new residual will be created and added to the
  // point_point_constraints_ member variable. A new autodiff residual will
  // be created from every constraint and added to the Ceres @problem.
  void AddSTFConstraints(ceres::Problem* problem);

  // Reset the global pose array from @start to @end (both inclusive), using
  // relative poses computed from @poses.
  void ResetGlobalPoses(const std::size_t start, const std::size_t end,
                        const std::vector<perception_2d::Pose2Df>& poses);

  // Add pose constraints based on odometry between successive poses. A new
  // autodiff residual will be created from every constraint and added to
  // the Ceres @problem.
  void AddOdometryConstraints(ceres::Problem* problem);

  std::pair<std::vector<std::pair<int, std::vector<int>>>,
            std::vector<std::pair<int, std::vector<int>>>>
      EstablishObservationSets(bool corners,
                               std::vector<Eigen::Vector2f>*
                               selected_points_ptr);

  double distToLineSeg(Eigen::Vector2f p1,
                       Eigen::Vector2f p2,
                       Eigen::Vector2f p);

  void SetCorrectionRelations(std::vector<std::pair<int, std::vector<int>>>
                              first_poses_obs,
                              std::vector<std::pair<int, std::vector<int>>>
                              second_poses_obs);

  void AddLoopConstraint(float scale,
                         ceres::Problem* problem);

  void AddHumanConstraints(ceres::Problem* problem);
  
  //int FindLoopClosurePose(const std::vector<Eigen::Vector2f>& selected_points,
    //                      bool corners);

  std::vector<Eigen::Vector2f> SegFit(double* p1, double* p2, double* cm,
                                                  double* data, int size);

  void TryCorrCallback(std::vector<double> gradients,
                       std::vector<Eigen::Matrix2f> covariances,
                       size_t min_poses, size_t max_poses);

  void CalculateConsistency();

  void AnimateCeres(std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >
                    lines,
                    std::vector<Eigen::Vector2f> points);

  ceres::TerminationType SolveHumanConstraints();

  ceres::TerminationType PostHumanOptimization(int min_pose, int max_pose);

   // Make a copy of the point clouds in vector2f format for ray casting
  // with VectorMap.
  void ConvertPointClouds(
      const std::vector<perception_2d::PointCloudf >& point_clouds);

  // Compute the relative transform to transform a point in the pose with the
  // index @source to the pose with the index @target.
  Eigen::Affine2f RelativePoseTransform(unsigned int source,
                                        unsigned int target);

  // Recomputes the relative pose array from the absolute pose array.
  void RecomputeRelativePoses();

  void BuildKDTrees();

  // Indicates that the update thread and batch localization (if running) should
  // keep running. It gets set to false by the destructor, or if the instance
  // is explicitly asked to terminate.
  bool terminate_;

  // Local version of point clouds for every pose... WORLD FRAME
  std::vector<std::vector<Eigen::Vector2f>> world_frame_point_clouds_;


  // List of point to point constraints obtained by matching observed points
  // from overlapping poses.
  std::vector<vector_localization::VectorMapping::PointToPointCorrespondence>
                       point_point_correspondences_;

  // List of point to point constraints obtained by matching observed points
  // from overlapping poses.
  std::vector<vector_localization::VectorMapping::PointToPointGlobCorrespondence>
                       point_point_glob_correspondences_;

  // List of point to point STF constraints.
  std::vector<vector_localization::PointToPointConstraint*> point_point_constraints_;

  // Array containing the 3DOF poses being optimized.
  std::vector<double> pose_array_;

  double last_cost_;

  int backprop_start_;

  int backprop_end_;

  // KD Trees of point clouds.
  std::vector<KDTree<float, 2>* > kdtrees_;

  // Point clouds for every pose node, in GVector::vector2f format.
  std::vector<std::vector<vector2f>> point_clouds_g_;

  // Array containing the 3DOF pose of the initial pose of the episode, and the
  // relative poses of every subsequent pose.
  std::vector<double> relative_pose_array_;

    // Options pertaining to non-Markov Localization.
  vector_localization::VectorMapping::VectorMappingOptions localization_options_;

  std::vector<Eigen::Vector2f> temp_inliers_;

  std::vector<float> ceres_cost_;
  
  cimg_library::CImg<float>* info_mat_;

};

//}  // namespace vector_localization

#endif  // jointopt_H
