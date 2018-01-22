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

#ifndef VECTOR_MAPPING_H
#define VECTOR_MAPPING_H

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
//#include <libfreenect2/src/opencl_depth_packet_processor.cl>

namespace vector_localization {

struct LTSConstraint;
struct PointToLineConstraint;
struct PointToPointConstraint;
struct VisibilityConstraint;
struct VisibilityGlobConstraint;



class VectorMapping {
 public:

  typedef std::pair<int,int> IntPair;
  typedef std::pair<Eigen::Vector2f, Eigen::Vector2f> Vector2fPair;
  typedef std::pair<IntPair, Vector2fPair> ConstraintPair;
  typedef std::pair<int, Eigen::Vector3f> CorrectionPair;

  // Struct for storing human added geometric constraints
 


//   struct HumanConstraint {
//     // Type of constraint
//     //int constraint_type;
//     // pose / observation indices
//     std::vector<std::pair<std::vector<std::pair<double, double>>, int>> obs_a;
//     std::vector<std::pair<std::vector<std::pair<double, double>>, int>> obs_b;
//     // pose / obs ids
//     std::vector<std::pair<int, std::vector<int>>> pose_obs_ids_a;
//     std::vector<std::pair<int, std::vector<int>>> pose_obs_ids_b;
//     // pose absolute indices
//     std::vector<int> x_a;
//     std::vector<int> x_b;
//     // line segment lengths
//     double len_a;
//     double len_b;
//     // cm estimates
//     Eigen::Vector2f cm_a;
//     Eigen::Vector2f cm_b;
//     // normal estimates
//     Eigen::Vector2f n_a;
//     Eigen::Vector2f n_b;
//     // correction mode
//     int mode;
//   };


  // Struct to keep track of the correspondence between a pair of observed
  // points, each point being from a different pose.
  struct PointToPointCorrespondence {
    // Index of the source pose.
    std::size_t source_pose;
    // Index of the point in the source pose for this correspondence.
    std::size_t source_point;
    // Index of the target pose.
    std::size_t target_pose;
    // Index of the point in the target pose for this correspondence.
    std::size_t target_point;
  };

  // Struct to keep track of the correspondence between STFs from a pair of
  // poses.
  struct PointToPointGlobCorrespondence {
    // Index number of the source pose that these points were observed from.
    std::size_t pose_index0;
    // Index number of the target pose that these points are related to.
    std::size_t pose_index1;
    // The index of the points from pose0.
    std::vector<size_t> points0_indices;
    // The index of the points from pose1.
    std::vector<size_t> points1_indices;
    // The coordinates of the point in the pose0 local coordinate frame.
    std::vector<Eigen::Vector2f> points0;
    // The coordinates of the point in the pose1 local coordinate frame.
    std::vector<Eigen::Vector2f> points1;
    // The coordinates of the normal in the pose0 local coordinate frame.
    std::vector<Eigen::Vector2f> normals0;
    // The coordinates of the normal in the pose1 local coordinate frame.
    std::vector<Eigen::Vector2f> normals1;
  };

  struct VectorMappingOptions {
    // Minimum distance that the robot should move before considering a new pose
    // for non-Markov localization.
    float minimum_node_translation;

    // Minimum angle that the robot should turn before considering a new pose
    // for non-Markov localization.
    float minimum_node_rotation;

    // The maximum refresh period that non-Markov Localization will ignore
    // non-zero odometry updates over. If the robot has moved within this much
    // time, and even if the motion is smaller than minimum_node_translation and
    // minimum_node_rotation, a pose update will be forced.
    double max_update_period;
    float kMinRange;
    float kMaxRange;
    float kPointPointCorrelationFactor;
    float kOdometryRadialStdDevRate;
    float kOdometryTangentialStdDevRate;
    float kOdometryAngularStdDevRate;
    float kOdometryTranslationMinStdDev;
    float kOdometryTranslationMaxStdDev;
    float kOdometryAngularMinStdDev;
    float kOdometryAngularMaxStdDev;
    float kPointMatchThreshold;
    float kLaserStdDev;
    float kMaxStfAngleError;
    unsigned int num_skip_readings;
    unsigned int kMinEpisodeLength;
    int kMaxRepeatIterations;
    int kNumRepeatIterations;
    int kMaxCorrespondencesPerPoint;
    int kNumThreads;
    int kPoseIncrement;
    int kMaxHistory;
    int max_solver_iterations;
    bool use_visual_odometry;
    bool log_poses;
    // Location of the sensor with respect to the robot center. Assumes that
    // the sensor is forward-facing w.r.t. the robot.
    Eigen::Vector2f sensor_offset;
    void (*CorrespondenceCallback)(
      const std::vector<double>& poses,
      const std::vector<std::vector<vector2f>>& point_clouds,
      const std::vector<perception_2d::NormalCloudf>& normal_clouds,
      const std::vector<PointToPointGlobCorrespondence>&
          point_point_correspondences,
      const std::vector<double>& gradients,
      const std::vector<Eigen::Matrix2f>& covariances,
      //const std::vector<Eigen::Matrix3f>& covariances,
      const std::vector<perception_2d::Pose2Df>& odometry_poses,
      const size_t start_pose,
      const size_t end_pose);

   void (*EvaluateConsistency)(const std::vector<double>& poses,
                       std::vector<perception_2d::PointCloudf>& point_clouds);

   void (*EvaluateConsistency2)(std::vector<perception_2d::Pose2Df> poses,
                       std::vector<perception_2d::PointCloudf> point_clouds);

    void (*AnimateCeres)(
      std::vector<std::pair<Eigen::Vector2f,Eigen::Vector2f>> lines,
                                     std::vector<Eigen::Vector2f> points);

    VectorMappingOptions() : log_poses(false), CorrespondenceCallback(NULL) {}
  };

  explicit VectorMapping(const std::string& maps_directory);
  virtual ~VectorMapping();

  // Notifies concurrently running threads, including the update thread and
  // BatchLocalize() to terminate.
  void Terminate();

  bool BatchLocalize(const VectorMappingOptions& options,
                     const std::string& map_name,
                     const std::vector<perception_2d::PointCloudf>&
                        point_clouds,
                     const std::vector<perception_2d::NormalCloudf>&
                        normal_clouds,
                     const bool debug, const bool return_initial_poses,
                     std::vector<perception_2d::Pose2Df>* poses);

  std::vector<HumanConstraint> BatchLocalizeHotStart(
      const VectorMappingOptions& options,
      const std::string& map_name,
      const std::vector<perception_2d::PointCloudf>& point_clouds,
      const std::vector<perception_2d::NormalCloudf>& normal_clouds,
      const bool debug,
      std::vector<perception_2d::Pose2Df>* poses,
      std::vector<Eigen::Vector2f> selected_points,
      const int mode,
      std::vector<Eigen::Matrix3d> covars,
      std::vector<HumanConstraint> human_constraints);


  // Clears the current list of poses and associated observations.
  void ClearPoses();

  // Returns the latest pose estimate based on all odometry messages observed so
  // far.
  perception_2d::Pose2Df GetLatestPose() const;

  // Return the latest pose of the MLE optimized node.
  perception_2d::Pose2Df GetLastMLEPose() const;

  // Return the last computed lost metric.
  float GetLostMetric() const;

  // Returns copies of vectors of the poses, point clouds, and pending poses.
  bool GetNodeData(
      std::vector<perception_2d::Pose2Df>* poses,
      std::vector<perception_2d::PointCloudf>* point_clouds,
      std::vector<perception_2d::Pose2Df>* pending_poses,
      perception_2d::Pose2Df* latest_pose) const;

  // Account for odometry update.
  void OdometryUpdate(const float dx, const float dy, const float d_theta);

  // Account for observation update.
  void SensorUpdate(
      const perception_2d::PointCloudf& point_cloud,
      const perception_2d::NormalCloudf& normal_cloud);

  // Set localization options.
  void SetOptions(const VectorMappingOptions& options);

  // Set initial MLE pose.
  void Initialize(const perception_2d::Pose2Df& pose, const std::string&
map_name);

  // Returns the current map name.
  std::string GetCurrentMapName() const;

  // Returns true when the background MLE solver is running.
  bool RunningSolver() const;

  // Returns a copy of the logged poses.
  std::vector<perception_2d::Pose2Df> GetLoggedPoses() const;

  // Returns the logged history of episode lengths.
  std::vector<int> GetLoggedEpisodeLengths() const;

  std::vector<Eigen::Matrix3d> GetPoseCovariances();

  std::vector<float> GetCeresCost();

 private:
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
  void FindVisualOdometryCorrespondences(const std::size_t min_poses,
                                         const std::size_t max_poses);

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
  void AddPoseConstraints(const std::size_t min_poses,
                          const std::size_t max_poses,
                          const std::vector<perception_2d::Pose2Df>& poses,
                          ceres::Problem* problem);

  int AddPointToPointHumanConstraint(
         std::vector<Eigen::Vector2f> selected_points,
         std::vector<CorrectionPair>* corrections);

  void AddLineToLineHumanConstraint(
         std::vector<Eigen::Vector2f> selected_points,
         const std::vector<perception_2d::Pose2Df>& poses,
         std::vector<CorrectionPair>* corrections);

  void AddCornerToCornerHumanConstraint(
         std::vector<Eigen::Vector2f> selected_points,
         const std::vector<perception_2d::Pose2Df>& poses,
         std::vector<CorrectionPair>* corrections);

  void AddColinearHumanConstraint(
         std::vector<Eigen::Vector2f> selected_points,
         const std::vector<perception_2d::Pose2Df>& poses,
         std::vector<CorrectionPair>* corrections);

  void AddPerpendicularHumanConstraint(
         std::vector<Eigen::Vector2f> selected_points,
         const std::vector<perception_2d::Pose2Df>& poses,
         std::vector<CorrectionPair>* corrections);

  void AddParallelHumanConstraint(
         std::vector<Eigen::Vector2f> selected_points,
         const std::vector<perception_2d::Pose2Df>& poses,
         std::vector<CorrectionPair>* corrections);

  int OrderAndFilterUserInput(const std::vector<perception_2d::Pose2Df>& poses,
                              bool corners,
                              std::vector<Eigen::Vector2f>*
                              selected_points_ptr);

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
                              second_poses_obs,
                              const std::vector<perception_2d::Pose2Df>& poses);


  void AddLoopConstraint(const std::vector<Eigen::Matrix3d>& D3_covariances,
                         const std::vector<perception_2d::Pose2Df>& poses,
                         float scale,
                         ceres::Problem* problem);

  void AddHumanConstraints(const std::vector<HumanConstraint>&
                           human_constraints,
                           const std::vector<perception_2d::Pose2Df>& poses,
                           ceres::Problem* problem);

  void AddHumanCorrections(std::vector<perception_2d::Pose2Df>* poses,
                           const std::vector<Eigen::Vector2f> selected_points,
                           const int mode,
                           std::vector<HumanConstraint>& human_constraints);

  int CalculateExplicitCorrections(std::vector<Eigen::Vector2f>
                                   selected_points,
                                   const std::vector<perception_2d::Pose2Df>&
                                   poses,
                                   int mode,
                                   std::vector<CorrectionPair>* corrections);

  void CopyTempLaserScans(const std::vector<perception_2d::Pose2Df>* poses);

  void FindContiguousGroups(size_t min_poses, size_t max_poses,
                            std::vector<CorrectionPair>* corrections,
                            std::vector<std::vector<CorrectionPair>>*
                            contiguous_corrections);

  int FindLoopClosurePose(const std::vector<Eigen::Vector2f>& selected_points,
                          bool corners);

  void BackPropagatePoseCorrections(std::vector<Eigen::Matrix3d>* covars,
                                    size_t temp_min_poses,
                                    size_t temp_max_poses,
                                    Eigen::Vector3f C,
                                    std::vector<perception_2d::Pose2Df>* poses);

  int ApplyExplicitCorrections(size_t i,
                               std::vector<std::vector<CorrectionPair>>*
                               contiguous_corrections,
                               std::vector<perception_2d::Pose2Df>* poses);

  std::vector<Eigen::Vector2f> SegFit(double* p1, double* p2, double* cm,
                                                  double* data, int size);

  std::vector<Eigen::Vector2d> InitLine(const std::vector<Eigen::Vector2d>*
                                                                inliers_ptr);

  void ConvertObsToInliers(
    const std::vector<std::pair<std::vector<std::pair<double, double>>, int>>*
    obs_ptr,
    const std::vector<perception_2d::Pose2Df>& poses,
    const std::vector<int> pose_ids,
    std::vector<Eigen::Vector2d>* inliers);

  void UpdateCovariances(size_t num_poses,
                         std::vector<Eigen::Matrix3d>* covars,
                         std::vector<Eigen::Matrix3d>* D3_covariances);

  void TryCorrCallback(std::vector<double> gradients,
                       std::vector<Eigen::Matrix2f> covariances,
                       std::vector<perception_2d::Pose2Df>* poses,
                       size_t min_poses, size_t max_poses);

  void CalculateConsistency();

  void AnimateCeres(std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >
                    lines,
                    std::vector<Eigen::Vector2f> points);

  std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> RefitConstraints(
                               std::vector<HumanConstraint>* human_constraints);

  ceres::TerminationType SolveHumanConstraints(
                                 std::vector<Eigen::Matrix3d>* D3_covariances,
                                 std::vector<perception_2d::Pose2Df>* poses,
                                 std::vector<HumanConstraint>*
                                 human_constraints);

  ceres::TerminationType PostHumanOptimization(
                                 std::vector<Eigen::Matrix3d>* D3_covariances,
                                 std::vector<perception_2d::Pose2Df>* poses,
                                 std::vector<HumanConstraint>*
                                 human_constraints,
                                 int min_pose,
                                 int max_pose);

  // Make a copy of a single point cloud in vector2f format
  void ConvertPointCloud(const perception_2d::PointCloudf& point_cloud,
                         std::vector<vector2f>* point_cloud_converted) const;

  // Make a copy of the point clouds in vector2f format for ray casting
  // with VectorMap.
  void ConvertPointClouds(
      const std::vector<perception_2d::PointCloudf >& point_clouds);

  // Buld KD Trees for every pose with the provided @point_clouds and
  // @normal_clouds.
  void BuildKDTrees(
      const std::vector<perception_2d::PointCloudf>& point_clouds,
      const std::vector<perception_2d::NormalCloudf>& normal_clouds);

  // Compute the relative transform to transform a point in the pose with the
  // index @source to the pose with the index @target.
  Eigen::Affine2f RelativePoseTransform(unsigned int source,
                                        unsigned int target);

  // Add the pending pose nodes along with the observations to the vectors of
  // current poses, point clouds, and normal clouds.
  void AddPendingPoseNodes();

  // Returns the number of observed points and number of valid LTF constraints
  // for the pose node entry indicated by @node_index.
  void CountConstraints(std::size_t node_index,
                        std::size_t* num_points_ptr,
                        std::size_t* num_ltf_constraints_ptr);

  // Recomputes the relative pose array from the absolute pose array.
  void RecomputeRelativePoses();

  // Recomputes the absolute pose array from the relative pose array.
  void RecomputeAbsolutePoses();

  // Computes an error metric that idicates how "lost" the EnML estimate is.
  void ComputeLostMetric();

  // The error metric of how lost the EnML estimate is.
  float lost_metric_;

  // Mutex to control access to pose nodes and point clouds. When locked, it
  // means that an update is concurrently running in the background. When
  // unlocked, it means that the pose nodes and point clouds are safe to be
  // modified, and there is no update running concurrently. See the
  // @update_semaphore_ for documentation of the order in which this mutex and
  // the @update_semaphore_ should be updated.
  mutable pthread_mutex_t update_mutex_;

  // Semaphore to control the producer - consumer nature of non-Markov
  // Localization. The semaphore is incremented when it's necessary to run the
  // update in the background thread, and decremented by the update thread once
  // done.
  //
  // Producer / Consumer model:
  // The producer is the thread that adds new pose data, the consumer is the
  // background thread that does the actual pose update.
  //
  // Once new sensor information is available to the producer:
  //  1. The producer first tries to lock the @update_mutex_ to get exclusive
  //     write-access to the pose nodes data structures.
  //  2. If the lock was succesful, it then adds the new data to the pose, point
  //     clouds and normal clouds arrays, and increments the @update_semaphore_.
  //  3. Finally it unlocks the @update_mutex_.
  //
  // The consumer:
  //  1. Waits on a background thread until the @update_semaphore_ is
  //     incremented by the producer.
  //  2. It then waits for a lock on the @update_mutex_ to get exclusive
  //     write-access to the pose nodes data structures.
  //  3. It then updates the MLE pose estimates by running non-Markov
  //     Localization.
  //  4. Finally it unlocks the @update_mutex_.
  sem_t update_semaphore_;

  // Indicates that the update thread and batch localization (if running) should
  // keep running. It gets set to false by the destructor, or if the instance
  // is explicitly asked to terminate.
  bool terminate_;

  // The vector map on which localization is being performed.
  //VectorMap vector_map_;

  // The accumulated translation and rotation of the robot since the last pose
  // node.
  perception_2d::Pose2Df pending_relative_pose_;

  // The sum of the magnitude of translations of the robot since the the last
  // pose node.
  float pending_translation_;

  // The sum of the magnitude of rotations of the robot since the the last
  // pose node.
  float pending_rotation_;

  // Point clouds for every pose node, in GVector::vector2f format.
  std::vector<std::vector<vector2f>> point_clouds_g_;

  // Point clouds for every pose node, in Eigen::Vector2f format. ROBOT FRAME
  std::vector<perception_2d::PointCloudf> point_clouds_e_;

  // Normal clouds for every pose node, in Eigen::Vector2f format. ROBOT FRAME
  std::vector<perception_2d::NormalCloudf> normal_clouds_e_;

  // Local version of point clouds for every pose... WORLD FRAME
  std::vector<std::vector<Eigen::Vector2f>> local_version_point_clouds_;

  // Poses of every node.
  std::vector<perception_2d::Pose2Df> poses_;

  // 3x3 Covariance matrix for each pose.
  std::vector<Eigen::Matrix3d> D3_covariances_;

  // list of relative observations for selected poses
  std::vector<std::pair<std::vector<std::pair<double, double>>, int>>
  corrected_observations_;

  // list of relative observations for selected poses
  std::vector<std::pair<std::vector<std::pair<double, double>>, int>>
  correcting_observations_;

  std::vector<std::pair<int, std::vector<int>>> corrected_pose_obs_;
  std::vector<std::pair<int, std::vector<int>>> correcting_pose_obs_;

  // list of indices of corrected poses
  std::vector<int> corrected_poses_;

  // list of indices of correcting poses
  std::vector<int> correcting_poses_;

  // KD Trees of point clouds.
  std::vector<KDTree<float, 2>* > kdtrees_;

  // List of point to point constraints obtained by matching observed points
  // from overlapping poses.
  std::vector<PointToPointCorrespondence> point_point_correspondences_;

  // List of point to point constraints obtained by matching observed points
  // from overlapping poses.
  std::vector<PointToPointGlobCorrespondence> point_point_glob_correspondences_;

  // List of point to point STF constraints.
  std::vector<PointToPointConstraint*> point_point_constraints_;

  // Array containing the 3DOF poses being optimized.
  std::vector<double> pose_array_;

  // array containing the parameters for human constraints
  // [cm_ax, cm_ay, cm_bx, cm_by, n_ax, n_ay, n_bx, n_by, ...]
  std::vector<double> hc_params_array_;

  // Array containing the 3DOF pose of the initial pose of the episode, and the
  // relative poses of every subsequent pose.
  std::vector<double> relative_pose_array_;

  // Latest pose estimate of the latest MLE optimized pose node.
  ThreadSafe<perception_2d::Pose2Df> latest_mle_pose_;

  // The pose of the latest pending pose node, relative to the latest MLE
  // optimized pose node, @latest_pose_.
  ThreadSafe<perception_2d::Pose2Df> latest_pending_pose_;

  // Pending point clouds in Eigen::Vector2f format, that yet need to be added
  // to the MLE pose array.
  std::vector<perception_2d::PointCloudf> pending_point_clouds_e_;

  // Pending normal clouds in Eigen::Vector2f format, that yet need to be added
  // to the MLE pose array.
  std::vector<perception_2d::NormalCloudf> pending_normal_clouds_e_;

  // Pending relative poses of every node, that yet need to be added to the MLE
  // pose array.
  std::vector<perception_2d::Pose2Df> pending_relative_poses_;

  // The history of logged poses generated by non-Markov localization. The poses
  // are logged if localization_options_.log_poses is set to true, and are
  // saved by the TrimEpisode function when the poses are removed from the
  // latest non-Markov episode.
  ThreadSafe<std::vector<perception_2d::Pose2Df>> logged_poses_;

  // The history of the episode lengths for every MLE update.
  std::vector<int> logged_episode_lengths_;

  // Options pertaining to non-Markov Localization.
  VectorMappingOptions localization_options_;

  // The last time a pose update was performed.
  double t_last_update_;

  double last_cost_;

  std::vector<Eigen::Vector2f> temp_inliers_;

  std::vector<float> ceres_cost_;

  double test_array_[4];
};

}  // namespace vector_localization

#endif  // VECTOR_MAPPING_H
