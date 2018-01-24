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

#include "../../vmapping/vector_mapping.h"

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

#include "ros/package.h"
#include "../../shared/util/configreader.h"

#include "ceres/ceres.h"
#include "ceres/dynamic_autodiff_cost_function.h"
#include "../../map/vector_map.h"
#include "../../perception_tools/perception_2d.h"
#include "../../shared/math/eigen_helper.h"
#include "../../shared/math/util.h"
#include "../../shared/util/helpers.h"
#include "../../shared/util/pthread_utils.h"
#include "../../shared/util/timer.h"
//#include "vector_localization/residual_functors.h"
//#include <vectorparticlefilter.h>
#include "residual_functors.h"

#include "JointOptimization.h"
//#include <libfreenect2/src/opencl_depth_packet_processor.cl>

// #define ENABLE_TIMING

#ifdef ENABLE_TIMING
  #define TIME_FUNCTION FunctionTimer ft(__FUNCTION__);
#else
  #define TIME_FUNCTION ;
#endif

using ceres::AutoDiffCostFunction;
using ceres::DynamicAutoDiffCostFunction;
using ceres::IterationCallback;
using ceres::IterationSummary;
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


namespace {

static const bool kUseRelativeConstraints = false;
static const size_t kDynamicDiffStride = 4;

// The path of the cobot_linux ROS stack.
static const string kCobotStackPath(ros::package::getPath("cobot_linux"));

// Config reader for localization options.
ConfigReader config_((kCobotStackPath + "/").c_str());

// Parameters used for relocalization.
//VectorLocalization2D::LidarParams relocalization_lidar_params_;

// Uncertainty of translation in the direction of travel.
float kRadialTranslationUncertainty = 0.05;
// Uncertainty of translation perpendicular to the direction of travel.
float kTangentialTranslationUncertainty = 0.05;
// Uncertainty of rotation in radians after moving 1 radian.
float kAngleUncertainty = 0.05;
// Scaling constant to correct for error in angular odometry.
float kOdometryRotationScale = 1.0;
// Scaling constant to correct for error in translation odometry.
float kOdometryTranslationScale = 1.0;

// Maximum distance between adjacent points to use for computation of normals.
float kMaxNormalPointDistance = 0.03;
// Angular margin from the scan area boundary to ignore laser readings.
float kAngularMargin = 0.0;
// Minimum distance of observed points from the robot.
float kMinPointCloudRange = 0.2;
// Maximum distance of observed points from the robot.
float kMaxPointCloudRange = 6.0;


void SetSolverOptions(
    const vector_localization::VectorMappingOptions&
        localization_options,
    ceres::Solver::Options* options_ptr) {
  ceres::Solver::Options& solver_options = *options_ptr;

  solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  solver_options.minimizer_type = ceres::TRUST_REGION;

  // solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  solver_options.minimizer_progress_to_stdout = false;
  solver_options.num_threads = localization_options.kNumThreads;
  solver_options.num_linear_solver_threads = localization_options.kNumThreads;
  solver_options.max_num_iterations =
      localization_options.max_solver_iterations;
  solver_options.function_tolerance = 0.000001;
  // solver_options.initial_trust_region_radius = 0.5;
  // solver_options.max_trust_region_radius = 2.0;
  solver_options.update_state_every_iteration = true;
}

class SRLCallback : public ceres::IterationCallback {
 public:
  SRLCallback(const vector<double>& poses,
              const vector<vector2f>& point_cloud_g,
              const PointCloudf& point_cloud_e,
              const NormalCloudf& normal_cloud,
              const vector<LTSConstraint*>& constraints,
              void (*callback)(
                  const vector<double>& poses,
                  const vector<vector2f>& point_cloud_g,
                  const PointCloudf& point_cloud_e,
                  const NormalCloudf& normal_cloud,
                  const vector<LTSConstraint*>& constraints)) :
      poses_(poses), point_cloud_g_(point_cloud_g),
      point_cloud_e_(point_cloud_e), normal_cloud_(normal_cloud),
      constraints_(constraints), callback_(callback) {
  }

  virtual ceres::CallbackReturnType operator()(
      const IterationSummary& summary) {
    callback_(poses_, point_cloud_g_, point_cloud_e_, normal_cloud_,
              constraints_);
    return ceres::SOLVER_CONTINUE;
  }

  const vector<double>& poses_;
  const vector<vector2f>& point_cloud_g_;
  const PointCloudf& point_cloud_e_;
  const NormalCloudf& normal_cloud_;
  const vector<LTSConstraint*>& constraints_;

  void (*callback_)(
      const vector<double>& poses,
      const vector<vector2f>& point_cloud_g,
      const PointCloudf& point_cloud_e,
      const NormalCloudf& normal_cloud,
      const vector<LTSConstraint*>& constraints);
};


}  // namespace

namespace vector_localization {

JointOpt::JointOpt() {}
JointOpt::~JointOpt() {}

bool JointOpt::LoadConfiguration(VectorMappingOptions* options) {
  if (!config_.readFiles()) return false;

  ConfigReader::SubTree c(config_,"VectorMapping");
  bool error = false;
  error = error || !c.getReal("radial_translation_uncertainty",
                              kRadialTranslationUncertainty);
  error = error || !c.getReal("tangential_translation_uncertainty",
                              kTangentialTranslationUncertainty);
  error = error || !c.getReal("angle_uncertainty", kAngleUncertainty);
  error = error || !c.getReal("odometry_translation_scale",
                              kOdometryTranslationScale);
  error = error || !c.getReal("odometry_rotation_scale",
                              kOdometryRotationScale);
  error = error || !c.getReal("min_point_cloud_range", kMinPointCloudRange);
  error = error || !c.getReal("max_point_cloud_range", kMaxPointCloudRange);
  error = error || !c.getReal("max_normal_point_distance",
                              kMaxNormalPointDistance);
#ifdef NDEBUG
  error = error || !c.getInt("num_threads", options->kNumThreads);
#else
  options->kNumThreads = 1;
#endif

  options->kMinRange = kMinPointCloudRange;
  options->kMaxRange = kMaxPointCloudRange;

  error = error || !c.getReal("robot_laser_offset.x",
                              options->sensor_offset.x());
  error = error || !c.getReal("robot_laser_offset.y",
                              options->sensor_offset.y());
  error = error || !c.getReal("min_rotation", options->minimum_node_rotation);
  error = error || !c.getReal("min_translation",
                              options->minimum_node_translation);
  error = error || !c.getInt("max_correspondences_per_point",
                             options->kMaxCorrespondencesPerPoint);
  error = error || !c.getReal("laser_std_dev",
                              options->kLaserStdDev);
  error = error || !c.getReal("point_correlation_factor",
                              options->kPointPointCorrelationFactor);
  error = error || !c.getReal("odometry_radial_stddev_rate",
                              options->kOdometryRadialStdDevRate);
  error = error || !c.getReal("odometry_tangential_stddev_rate",
                              options->kOdometryTangentialStdDevRate);
  error = error || !c.getReal("odometry_angular_stddev_rate",
                              options->kOdometryAngularStdDevRate);
  error = error || !c.getReal("odometry_translation_min_stddev",
                              options->kOdometryTranslationMinStdDev);
  error = error || !c.getReal("odometry_translation_max_stddev",
                              options->kOdometryTranslationMaxStdDev);
  error = error || !c.getReal("odometry_rotation_min_stddev",
                              options->kOdometryAngularMinStdDev);
  error = error || !c.getReal("odometry_rotation_max_stddev",
                              options->kOdometryAngularMaxStdDev);
  error = error || !c.getReal("point_match_threshold",
                              options->kPointMatchThreshold);
  error = error || !c.getReal("max_stf_angle_error",
                              options->kMaxStfAngleError);
  error = error || !c.getInt("pose_increment",
                              options->kPoseIncrement);
  error = error || !c.getInt("max_history",
                             options->kMaxHistory);
  error = error || !c.getInt("max_solver_iterations",
                             options->max_solver_iterations);
  error = error || !c.getInt("max_repeat_iterations",
                             options->kMaxRepeatIterations);
  error = error || !c.getInt("num_repeat_iterations",
                             options->kNumRepeatIterations);
  error = error || !c.getUInt("min_episode_length",
                              options->kMinEpisodeLength);
  error = error || !c.getUInt("num_skip_readings",
                             options->num_skip_readings);
  error = error || !c.getReal("angle_margin",
                              kAngularMargin);
  error = error || !c.getReal("max_update_period",
                              options->max_update_period);
  //error = error || !relocalization_lidar_params_.LoadFromConfig(&config_);

  return !error;
}

Affine2f JointOpt::RelativePoseTransform(
    unsigned int source, unsigned int target) {
  const Affine2f source_tf = Translation2Df(
      pose_array_[3 * source], pose_array_[3 * source + 1]) *
      Rotation2Df(pose_array_[3 * source + 2]);
  const Affine2f target_tf = Translation2Df(
        pose_array_[3 * target], pose_array_[3 * target + 1]) *
        Rotation2Df(pose_array_[3 * target + 2]);
  return target_tf.inverse(Eigen::Affine) * source_tf;
}

/*
class PoseToObjectConstraint : public ceres::SizedCostFunction<1, 3, 3> {
 public:
  PoseToObjectConstraint(
      const vector<Vector2f>& points,
      const PersistentObject& object,
      size_t instance_id) :
      points_(points), object_(object), instance_id_(instance_id) {}

  bool Evaluate(double const* const* parameters,
                double* residuals, double** jacobians) const {
    // double const* const robot_pose = parameters[0];
    // double const* const object_pose = parameters[1];

    // Compose the pose transform from the robot reference frame to the
    // object grid reference frame

    // Compute residual as sum of occupancy values for all points tranformed
    // by robot pose, on to object grid transformed by object pose.

    // For any non-NULL jacobian pointers, return the partial jacobians.

    return true;
  }

 private:
  const vector<Vector2f> points_;
  const PersistentObject& object_;
  const size_t instance_id_;
};*/

vector<Matrix3d> JointOpt::GetPoseCovariances() {
  return d3_covariances_;
}

vector< float > JointOpt::GetCeresCost() {
  return ceres_cost_;
}

/*
void JointOpt::TryCorrCallback(vector<double> gradients,
                                    vector<Matrix2f> covariances,
                                    vector<Pose2Df>* poses,
                                    size_t min_poses, size_t max_poses) {

  if (localization_options_.CorrespondenceCallback != NULL) {
    localization_options_.CorrespondenceCallback(
        pose_array_,
        point_clouds_g_,
        normal_clouds_e_,
        point_point_glob_correspondences_,
        gradients,
        covariances,
        *poses,
        min_poses,
        max_poses);
  }
}*/

void JointOpt::CalculateConsistency() {
  if (localization_options_.EvaluateConsistency != NULL) {
    localization_options_.EvaluateConsistency(pose_array_,
robot_frame_point_clouds_);
  }
}

void JointOpt::AnimateCeres(vector<pair<Vector2f, Vector2f> > lines,
                                 vector<Vector2f> points) {
  if (localization_options_.AnimateCeres != NULL) {
    localization_options_.AnimateCeres(lines, points);
  }
}

void JointOpt::CopyParams() {
  // Copy over the optimized poses.
  cout << "begin copy" << endl;
  cout << "pose_array_ size: " << pose_array_.size() << endl;
  for (size_t i = 0; i < poses_.size(); ++i) {
    const int j = 3 * i;
    poses_[i].translation.x() = pose_array_[j + 0];
    poses_[i].translation.y() = pose_array_[j + 1];
    poses_[i].angle = angle_mod(pose_array_[j + 2]);
  }
}

void JointOpt::SetParams() {
  pose_array_.clear();
  CHECK_EQ(pose_array_.size(), 0);
  pose_array_.resize(poses_.size() * 3);
  for (size_t i = 0; i < poses_.size(); ++i) {
    const int j = 3 * i;
    pose_array_[j + 0] = poses_[i].translation.x();
    pose_array_[j + 1] = poses_[i].translation.y();
    pose_array_[j + 2] = poses_[i].angle;
  }
}

void JointOpt::CopyTempLaserScans() {
  world_frame_point_clouds_.resize(robot_frame_point_clouds_.size());
  for (size_t i = 0; i < robot_frame_point_clouds_.size(); ++i) {
    const PointCloudf& point_cloud = robot_frame_point_clouds_[i];
    world_frame_point_clouds_[i].resize(point_cloud.size());
  }
  // put laser scans in world frame... as they are displayed in the gui
  for (size_t i = 0; i < robot_frame_point_clouds_.size(); i++) {
    const Rotation2Df rotation(poses_[i].angle);
    const Vector2f pose_location = poses_[i].translation;
    for (size_t j = 0; j < robot_frame_point_clouds_[i].size(); ++j) {
      world_frame_point_clouds_[i][j]=(rotation*robot_frame_point_clouds_[i][j]
                                     + pose_location);
    }
  }
}

void JointOpt::ConvertPointClouds() {
  point_clouds_g_.resize(robot_frame_point_clouds_.size());
  for (size_t i = 0; i < robot_frame_point_clouds_.size(); ++i) {
    const PointCloudf& point_cloud = robot_frame_point_clouds_[i];
    point_clouds_g_[i].resize(point_cloud.size());
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      point_clouds_g_[i][j] = vector2f(point_cloud[j].x(), point_cloud[j].y());
    }
  }
}

void JointOpt::FindVisualOdometryCorrespondences(int min_poses, int max_poses) {
  const float min_cosine_angle = cos(localization_options_.kMaxStfAngleError);
  const size_t poses_end = min(
      static_cast<size_t>(max_poses + 1), point_clouds_g_.size());
  if (int(poses_end) < min_poses + 1) return;
  vector<vector<PointToPointCorrespondence>>
                   pose_correspondences(poses_end - min_poses - 1);
  //OMP_PARALLEL_FOR
  for (size_t i = min_poses; i < poses_end - 1; ++i) {
    PointToPointCorrespondence correspondence;
    correspondence.source_pose = i;
    correspondence.target_pose = i + 1;
    Affine2f source_to_target_tf = RelativePoseTransform(i, i + 1);
    for (size_t k = 0; k < point_clouds_g_[i].size(); ++k) {
      correspondence.source_point = k;
      const Vector2f point(source_to_target_tf*robot_frame_point_clouds_[i][k]);
      const Vector2f normal =
          Rotation2Df(pose_array_[3 * i + 2 + 3] - pose_array_[3 * i + 2]) *
          robot_frame_normal_clouds_[i][k];
      KDNodeValue<float, 2> neighbor_point;
      const float closest_distance = kdtrees_[i + 1]->FindNearestPoint(
          point, localization_options_.kPointMatchThreshold, &neighbor_point);
      const float cosine_angle = neighbor_point.normal.dot(normal);
      if (closest_distance < localization_options_.kPointMatchThreshold &&
          cosine_angle > min_cosine_angle) {
        // Valid point to point match found
        correspondence.target_point = neighbor_point.index;
        pose_correspondences[i - min_poses].push_back(correspondence);
      }
    }
  }
  for (size_t i = 0; i < pose_correspondences.size(); ++i) {
    point_point_correspondences_.insert(
        point_point_correspondences_.end(),
        pose_correspondences[i].begin(), pose_correspondences[i].end());
  }
}

void JointOpt::RecomputeRelativePoses() {
  relative_pose_array_.resize(pose_array_.size());
  // The pose array size must be a multiple of 3, since each pose has 3-DOF.
  CHECK_EQ((pose_array_.size() % 3), 0);
  // The first 3-DOF pose value is absolute.
  relative_pose_array_[0] = pose_array_[0];
  relative_pose_array_[1] = pose_array_[1];
  relative_pose_array_[2] = pose_array_[2];
  for (size_t i = 3; i < pose_array_.size(); ++i) {
    relative_pose_array_[i] = pose_array_[i] - pose_array_[i - 3];
  }
}

void JointOpt::ResetGlobalPoses(
    const size_t start, const size_t end,
    const vector< Pose2Df >& poses) {
  CHECK_LT(end, poses.size());
  for (size_t i = start; i <= end; ++i) {
    const Vector2f& p0 = poses[i - 1].translation;
    const Vector2f& p1 = poses[i].translation;
    const double dr = angle_mod(poses[i].angle - poses[i - 1].angle);
    const Vector2f dp = Rotation2Df(-poses[i - 1].angle) * (p1 - p0);
    const Vector2f p1_new =
        Vector2f(pose_array_[3 * (i - 1) + 0], pose_array_[3 * (i - 1) + 1]) +
        Rotation2Df(pose_array_[3 * (i - 1) + 2]) * dp;
    pose_array_[3 * i + 0] = p1_new.x();
    pose_array_[3 * i + 1] = p1_new.y();
    pose_array_[3 * i + 2] = pose_array_[3 * (i - 1) + 2] + dr;
  }
}

double JointOpt::distToLineSeg(Vector2f p1, Vector2f p2, Vector2f p) {
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


void JointOpt::BuildKDTrees() {
  CHECK_EQ(robot_frame_point_clouds_.size(), robot_frame_normal_clouds_.size());
  kdtrees_.resize(robot_frame_point_clouds_.size(), NULL);
  const unsigned int num_point_clouds = robot_frame_point_clouds_.size();

  //OMP_PARALLEL_FOR
  for (size_t i = 0; i < num_point_clouds; ++i) {
    const PointCloudf& point_cloud = robot_frame_point_clouds_[i];
    const NormalCloudf& normal_cloud = robot_frame_normal_clouds_[i];
    CHECK_EQ(point_cloud.size(), normal_cloud.size());
    vector<KDNodeValue<float, 2>> values(point_cloud.size());
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      values[j].index = j;
      values[j].point = point_cloud[j];
      values[j].normal = normal_cloud[j];
    }
    if (values.size() > 0) {
      kdtrees_[i] = new KDTree<float, 2>(values);
    }
    else {
      kdtrees_[i] = new KDTree<float, 2>();
    }
  }
}

void JointOpt::AddSTFConstraints(ceres::Problem* problem) {
  TIME_FUNCTION
  cout << "stfs size: " << point_point_glob_correspondences_.size() << endl;
  for (size_t i = 0; i < point_point_glob_correspondences_.size(); ++i) {
    PointToPointGlobCorrespondence& correspondence =
        point_point_glob_correspondences_[i];

    DCHECK_NE(correspondence.pose_index0, correspondence.pose_index1);
    PointToPointGlobConstraint* constraint = new PointToPointGlobConstraint(
        correspondence.pose_index0, correspondence.pose_index1,
        correspondence.points0, correspondence.points1,
        correspondence.normals0, correspondence.normals1,
        localization_options_.kLaserStdDev,
        localization_options_.kPointPointCorrelationFactor);
    problem->AddResidualBlock(
        new AutoDiffCostFunction<PointToPointGlobConstraint, 2, 3, 3>(
            constraint), NULL,
            &(pose_array_[3 * correspondence.pose_index0]),
            &(pose_array_[3 * correspondence.pose_index1]));
  }
}

void JointOpt::FindSTFCorrespondences(
    const size_t min_poses, const size_t max_poses) {
  static const size_t kMinInterPoseCorrespondence = 10;
  const float min_cosine_angle = cos(localization_options_.kMaxStfAngleError);
  // const float kMaxPoseSqDistance = sq(100.0);
  static const int kPointMatchSeparation = 1;
  const size_t poses_end = min(
      static_cast<size_t>(max_poses + 1), point_clouds_g_.size());
  CHECK_GT(pose_array_.size(), poses_end - 1);
  vector<vector<PointToPointGlobCorrespondence>>
pose_point_correspondences(
      poses_end - min_poses);
  point_point_glob_correspondences_.clear();
  //cout << "here" << endl;
  OMP_PARALLEL_FOR
  for (size_t i = min_poses; i < poses_end; ++i) {
    vector<int> point_corrspondences(point_clouds_g_[i].size(), 0);
    // const Vector2f source_pose_location(
    //     pose_array_[3 * i], pose_array_[3 * i + 1]);
    for (size_t j = min_poses ; j < poses_end; j += kPointMatchSeparation) {
      if (i == j) continue;
      PointToPointGlobCorrespondence correspondence;
      vector<size_t> source_point_indices;
      correspondence.pose_index0 = i;
      correspondence.pose_index1 = j;
      size_t num_stfs = 0;
      const Vector2f target_pose_location(
          pose_array_[3 * j], pose_array_[3 * j + 1]);
      // Ignore poses with insufficent overlap.
      // if ((target_pose_location - source_pose_location).squaredNorm() >
      //     kMaxPoseSqDistance) continue;
      Affine2f source_to_target_tf = RelativePoseTransform(i, j);
      for (size_t k = 0; k < point_clouds_g_[i].size();
           k += localization_options_.num_skip_readings) {
        // Ignore points that already have a valid correspondence: they do not
        // need to be considered as STF objects.
        if (point_corrspondences[k] >=
            localization_options_.kMaxCorrespondencesPerPoint) {
          continue;
        }

        const Vector2f point(source_to_target_tf *
                                           robot_frame_point_clouds_[i][k]);
        const Vector2f normal =
            Rotation2Df(pose_array_[3 * j + 2] - pose_array_[3 * i + 2]) *
            robot_frame_normal_clouds_[i][k];
        KDNodeValue<float, 2> neighbor_point;
        const float closest_distance = kdtrees_[j]->FindNearestPointNormal(
            point, localization_options_.kPointMatchThreshold, &neighbor_point);
        const float cosine_angle = neighbor_point.normal.dot(normal);
        if (closest_distance < localization_options_.kPointMatchThreshold &&
            cosine_angle > min_cosine_angle) {
          // Valid point to point match found
          correspondence.points0_indices.push_back(k);
          correspondence.points1_indices.push_back(neighbor_point.index);
          correspondence.points0.push_back(robot_frame_point_clouds_[i][k]);
          correspondence.points1.push_back(
              robot_frame_point_clouds_[j][neighbor_point.index]);
          correspondence.normals0.push_back(robot_frame_normal_clouds_[i][k]);
          correspondence.normals1.push_back(
              robot_frame_normal_clouds_[j][neighbor_point.index]);
          source_point_indices.push_back(k);
	  
	  info_mat_[0](i, j) = 255.0;
	  info_mat_[0](j, i) = 255.0;
	  
          ++num_stfs;
          ++point_corrspondences[k];
        }
      }
      if (correspondence.points0.size() > kMinInterPoseCorrespondence) {
        pose_point_correspondences[i - min_poses].push_back(correspondence);
      }
    }
  }
  for (size_t i = 0; i < pose_point_correspondences.size(); ++i) {
    point_point_glob_correspondences_.insert(
        point_point_glob_correspondences_.end(),
        pose_point_correspondences[i].begin(),
        pose_point_correspondences[i].end());
  }
}











// void JointOpt::AddOdometryConstraints(ceres::Problem* problem) {
//   static const bool debug = false;
//   TIME_FUNCTION
//   static const float kEpsilon = 1e-6;
//   //float odom_std_dev_weight = 100.0;
//   float odom_std_dev_weight = 1.0;
//   cout << "poses size: " << poses_.size() << endl;
//   cout << "odom size: " << orig_odom_.size() << endl;
//   for (size_t i = 1; i < orig_odom_.size(); ++i) {
//     // Create a new odometry constraint residual from the odometry constraint
//     // between pose i and pose i-1, and add it to the Ceres problem.
//     const Vector2f translation(poses_[i].translation - poses_[i-1].translation);
//     Vector2f radial_direction;
//     Vector2f tangential_direction;
//     float radial_translation = 0.0;
//     if (fabs(translation.x()) < kEpsilon && fabs(translation.y()) < kEpsilon) {
//       radial_direction = Vector2f(cos(orig_odom_[i].angle), sin(orig_odom_[i].angle));
//     }
//     else {
//       radial_direction = Vector2f((Rotation2Df(orig_odom_[i - 1].angle) * translation).normalized());
//       radial_translation = translation.norm();
//     }
//     
//     float rotation = orig_odom_[i].angle;
//     //std::cout << orig_odom_[i].angle << std::endl;
//     float trial = atan2(sin(pose_array_[(3 * i) + 2] - pose_array_[(3 * (i - 1)) + 2] - orig_odom_[i].angle),
// 			cos(pose_array_[(3 * i) + 2] - pose_array_[(3 * (i - 1)) + 2] - orig_odom_[i].angle));
//     if (fabs(pose_array_[(3 * i) + 2] - pose_array_[(3 * (i - 1)) + 2] - orig_odom_[i].angle) > 1.0) {
//       std::cout << "poses: " << i -1 << ", " << i << std::endl;
//       std::cout << orig_odom_[i].angle << std::endl;
//       std::cout << pose_array_[(3 * (i - 1)) + 2] << ", " << pose_array_[(3 * i) + 2] << std::endl;
//       std::cout << "REG: " << pose_array_[(3 * i) + 2] - pose_array_[(3 * (i - 1)) + 2] - orig_odom_[i].angle << std::endl;
//       std::cout << "TRIAL: " << trial << std::endl;
//     }
//     Matrix2f axis_transform;
//     tangential_direction = Vector2f(Rotation2Df(M_PI_2) * radial_direction);
//     //rotation = angle_mod(poses_[i].angle - poses_[i-1].angle);
//     axis_transform.block<1, 2>(0, 0) = radial_direction.transpose();
//     axis_transform.block<1, 2>(1, 0) = tangential_direction.transpose();
// 
// 
// //     cout << "rad_dir: " << radial_direction << endl;
// //     cout << "tan dir: " << tangential_direction << endl;
// //     cout << "rotation: " << rotation << endl;
// //     cout << "translation: " << radial_translation << endl;
// 
//     const float radial_std_dev = bound<float>(
//         localization_options_.kOdometryRadialStdDevRate * radial_translation,
//         localization_options_.kOdometryTranslationMinStdDev,
//         localization_options_.kOdometryTranslationMaxStdDev);
//     const float tangential_std_dev = bound<float>(
//         localization_options_.kOdometryTangentialStdDevRate * radial_translation,
//         localization_options_.kOdometryTranslationMinStdDev,
//         localization_options_.kOdometryTranslationMaxStdDev);
//     const float angular_std_dev = bound<float>(
//         localization_options_.kOdometryAngularStdDevRate * fabs(rotation),
//         localization_options_.kOdometryAngularMinStdDev,
//         localization_options_.kOdometryAngularMaxStdDev);
// 
//     std::cout << localization_options_.kOdometryAngularMinStdDev << " ::: "
//               << angular_std_dev << std::endl << " ::: "
//               << localization_options_.kOdometryAngularMaxStdDev << std::endl;
// 	      
//     problem->AddResidualBlock(
//       //new AutoDiffCostFunction<PoseConstraint, 3, 3, 3>(
//       new AutoDiffCostFunction<PoseConstraint, 2, 3, 3>(
// //       new AutoDiffCostFunction<PoseConstraint, 1, 3, 3>(
//           new PoseConstraint(axis_transform,
//                              radial_std_dev*odom_std_dev_weight,
//                              tangential_std_dev*odom_std_dev_weight,
//                              angular_std_dev*odom_std_dev_weight,
//                              radial_translation,
//                              rotation)),
//           NULL,
//           &(pose_array_[3 * i - 3]),
//           &(pose_array_[3 * i]));
//   }
//   problem->SetParameterBlockConstant(&pose_array_[0]);
// }



void JointOpt::AddOdometryConstraints(ceres::Problem* problem) {
  static const bool debug = false;
  TIME_FUNCTION
  static const float kEpsilon = 1e-6;
  
//   std::cout << "top of loop" << std::endl;
//   std::cout << "odom_size" << orig_odom_.size() << std::endl;
  for (size_t i = 1; i < poses_.size(); ++i) {
    info_mat_[0](i - 1, i) = 255.0;
    info_mat_[0](i, i - 1) = 255.0;
    // Create a new pose constraint residual from the odometry constraint
    // between pose i and pose i-1, and add it to the Ceres problem.
    const Vector2f translation(poses_[i].translation - poses_[i-1].translation);
    Vector2f radial_direction;
    Vector2f tangential_direction;
    float rotation;
    Matrix2f axis_transform;
    float radial_translation = 0.0;
    if (fabs(translation.x()) < kEpsilon && fabs(translation.y()) < kEpsilon) {
      radial_direction = Vector2f(cos(poses_[i].angle), sin(poses_[i].angle));
      tangential_direction = Vector2f(Rotation2Df(M_PI_2) * radial_direction);
      rotation = angle_mod(poses_[i].angle - poses_[i-1].angle);
      axis_transform.block<1, 2>(0, 0) = radial_direction.transpose();
      axis_transform.block<1, 2>(1, 0) = tangential_direction.transpose();
      radial_translation = 0.0;
    } else {
      radial_direction = Vector2f(
          (Rotation2Df(-poses_[i - 1].angle) * translation).normalized());
      tangential_direction = Vector2f(Rotation2Df(M_PI_2) * radial_direction);
      rotation = angle_mod(poses_[i].angle - poses_[i-1].angle);
      axis_transform.block<1, 2>(0, 0) = radial_direction.transpose();
      axis_transform.block<1, 2>(1, 0) = tangential_direction.transpose();
      radial_translation = translation.norm();
    }
    const float radial_std_dev = bound<float>(
        localization_options_.kOdometryRadialStdDevRate * radial_translation,
        localization_options_.kOdometryTranslationMinStdDev,
        localization_options_.kOdometryTranslationMaxStdDev);
    const float tangential_std_dev = bound<float>(
        localization_options_.kOdometryTangentialStdDevRate *
        radial_translation,
        localization_options_.kOdometryTranslationMinStdDev,
        localization_options_.kOdometryTranslationMaxStdDev);
    const float angular_std_dev = bound<float>(
        localization_options_.kOdometryAngularStdDevRate * fabs(rotation),
        localization_options_.kOdometryAngularMinStdDev,
        localization_options_.kOdometryAngularMaxStdDev);
    
//     std::cout << localization_options_.kOdometryAngularMinStdDev << " ::: "
//               << angular_std_dev << " ::: "
//               << localization_options_.kOdometryAngularMaxStdDev << std::endl;
    
    
//     if (debug) {
//       printf("Adding pose constraint %d:%d @ 0x%lx : 0x%lx\n",
//              static_cast<int>(i - 1), static_cast<int>(i),
//              reinterpret_cast<uint64_t>(&(pose_array_[3 * i - 3])),
//              reinterpret_cast<uint64_t>(&(pose_array_[3 * i])));
//     }
//     if (kUseRelativeConstraints) {
//       RelativePoseConstraint* constraint = new RelativePoseConstraint(
//           i - 1, i, axis_transform, radial_std_dev, tangential_std_dev,
//           angular_std_dev, radial_translation, rotation);
//       DynamicAutoDiffCostFunction<RelativePoseConstraint, kDynamicDiffStride>*
//           cost_function = new DynamicAutoDiffCostFunction<
//               RelativePoseConstraint, kDynamicDiffStride>(constraint);
//       vector<double*> parameter_blocks;
//       for (size_t j = 0; j <= i; ++j) {
//         cost_function->AddParameterBlock(3);
//         parameter_blocks.push_back(relative_pose_array_.data() + 3 * j);
//       }
//       cost_function->SetNumResiduals(3);
//       problem->AddResidualBlock(cost_function, NULL, parameter_blocks);
//     } else {
      problem->AddResidualBlock(
        new AutoDiffCostFunction<PoseConstraint, 3, 3, 3>(
          new PoseConstraint(axis_transform, radial_std_dev, tangential_std_dev,
                            angular_std_dev, radial_translation, rotation)),
          NULL, &(pose_array_[3 * i - 3]), &(pose_array_[3 * i]));
    // }
  }
  problem->SetParameterBlockConstant(&pose_array_[0]);
}






// void JointOpt::AddLoopConstraint(float scale, ceres::Problem* problem) {
//   CHECK_EQ(poses_.size(), d3_covariances_.size());
//   static const float kEpsilon = 1e-6;
// 
//   ceres_cost_.clear();
//   for (size_t i = 0; i < poses_.size(); ++i) {
//     ceres_cost_.push_back(0.0);
//   }
// 
//   for (size_t i=1; i<poses_.size(); i++) {
//     const Vector2f translation(poses_[i].translation - poses_[i-1].translation);
//     Vector2f radial_direction;
//     Vector2f tangential_direction;
//     Matrix2f axis_transform;
//     float radial_translation;
//     float rotation;
//     if (fabs(translation.x()) < kEpsilon && fabs(translation.y()) < kEpsilon) {
//       radial_direction = Vector2f(cos(poses_[i].angle), sin(poses_[i].angle));
//       radial_translation = 0.0;
//     } else {
//       radial_direction = Vector2f((Rotation2Df(-poses_[i-1].angle) *
//                                    translation).normalized());
//       radial_translation = translation.norm();
//     }
//     tangential_direction = Vector2f(Rotation2Df(M_PI_2) * radial_direction);
//     rotation = (poses_[i].angle - poses_[i-1].angle);
//     axis_transform.block<1, 2>(0, 0) = radial_direction.transpose();
//     axis_transform.block<1, 2>(1, 0) = tangential_direction.transpose();
// 
//     Eigen::Matrix2d trans_cov;
//     trans_cov << d3_covariances_[i-1](0,0),
//                  d3_covariances_[i-1](0,1),
//                  d3_covariances_[i-1](1,0),
//                  d3_covariances_[i-1](1,1);
// 
//     Vector2f dir1;
//     Vector2f dir2;
//     Eigen::EigenSolver<Matrix2d> es;
//     es.compute(trans_cov, true);
//     double lambda1 = es.eigenvalues()[0].real();
//     double lambda2 = es.eigenvalues()[1].real();
//     double sigma1 = sqrt(5.991*lambda1);
//     double sigma2 = sqrt(5.991*lambda2);
//     dir1(0) = es.eigenvectors().col(0)(0).real();
//     dir1(1) = es.eigenvectors().col(0)(1).real();
//     dir2(0) = es.eigenvectors().col(1)(0).real();
//     dir2(1) = es.eigenvectors().col(1)(1).real();
//     dir1.normalize();
//     dir2.normalize();
//     dir1 *= sigma1;
//     dir2 *= sigma2;
// 
//     float r_std_dev = float(sqrt(pow(radial_direction.dot(dir1),2) +
//         pow(radial_direction.dot(dir2),2)));
//     float t_std_dev = float(sqrt(pow(tangential_direction.dot(dir1),2) +
//         pow(tangential_direction.dot(dir2),2)));
//     float a_std_dev = float(sqrt(d3_covariances_[i-1](2,2)));
// 
//     float radial_std_dev = bound<float>(
//       r_std_dev,
//       localization_options_.kOdometryTranslationMinStdDev,
//       localization_options_.kOdometryTranslationMaxStdDev);
//     float tangential_std_dev = bound<float>(
//       t_std_dev,
//       localization_options_.kOdometryTranslationMinStdDev,
//       localization_options_.kOdometryTranslationMaxStdDev);
//     float angular_std_dev = bound<float>(
//       a_std_dev,
//       localization_options_.kOdometryAngularMinStdDev,
//       localization_options_.kOdometryAngularMaxStdDev);
// 
// 
//     if (!std::isfinite(radial_std_dev) || radial_std_dev < localization_options_.kOdometryTranslationMinStdDev) {
//       cout << "radial std dev is not finite or is too low: "
//            << radial_std_dev
//            << endl;
//       cout << "setting radial std dev to: "
//            << localization_options_.kOdometryTranslationMinStdDev
//            << endl;
//       radial_std_dev = localization_options_.kOdometryTranslationMinStdDev;
//     }
//     if (!std::isfinite(tangential_std_dev)) {
//       //cout << "tangential std dev is not finite: "
//       //     << tangential_std_dev
//       //     << endl;
//       //cout << "setting tangential std dev to: "
//       //     << localization_options_.kOdometryTranslationMinStdDev
//       //     << endl;
//       tangential_std_dev = localization_options_.kOdometryTranslationMinStdDev;
//     }
//     if (!std::isfinite(angular_std_dev)) {
//       //cout << "angular std dev is not finite: "
//       //     << angular_std_dev
//       //     << endl;
//       //cout << "setting angular std dev to: "
//       //     << localization_options_.kOdometryAngularMinStdDev
//       //     << endl;
//       angular_std_dev = localization_options_.kOdometryAngularMinStdDev;
//     }
//     if (!std::isfinite(radial_translation)) {
//       cout << "radial translation is not finite: "
//            << radial_translation
//            << endl;
//       //cout << "setting radial translation to: "
//       //     << 0.0
//       //     << endl;
//       radial_translation = 0.0;
//     }
//     //cout << "min rad stdev: " <<
// //localization_options_.kOdometryTranslationMinStdDev << endl;
//   //  cout << scale*radial_std_dev << endl;
//     problem->AddResidualBlock(
//         new AutoDiffCostFunction<PoseConstraint, 3, 3, 3>(
//             new PoseConstraint(axis_transform,
//                                scale * radial_std_dev,
//                                scale * tangential_std_dev,
//                                scale * angular_std_dev,
//                                radial_translation,
//                                rotation)),
//             NULL,
//             &(pose_array_[3 * (i-1)]),
//             &(pose_array_[3 * i]));
// 
//   }
// }












void JointOpt::AddHumanConstraints(ceres::Problem* problem) {
  //TODO: support other correction types
  //TODO: (possibly) refactor hc representation so we dont need to check the type inside the nested loop
  num_hc_residuals_ = 0;
  // for (size_t i = 0; i < human_constraints_.size() - 1; ++i) {
  //int prev_id = -1;
  for (size_t i = 0; i < human_constraints_.size(); ++i) {
    for (size_t j = 0; j < human_constraints_[i].size(); ++j) {
      HumanConstraint constraint = human_constraints_[i][j];
      info_mat_[0](constraint.anchor_pose_id, constraint.constrained_pose_id) = 255.0;
      info_mat_[0](constraint.constrained_pose_id, constraint.anchor_pose_id) = 255.0;
      if (constraint.constraint_type == vector_localization::CorrectionType::kLineSegmentCorrection) {
        Eigen::Vector2f anchor_pose_loc = poses_[constraint.anchor_pose_id].translation;
        float anchor_pose_angle = poses_[constraint.anchor_pose_id].angle;
        Eigen::Vector2f para_dir = Eigen::Vector2f(cos(anchor_pose_angle), sin(anchor_pose_angle));
        Eigen::Vector2f perp_dir = Eigen::Vector2f(-para_dir(1), para_dir(0));
        Eigen::Vector2f target_loc = anchor_pose_loc + 
                                     constraint.delta_parallel * para_dir +
                                     constraint.delta_perpendicular * perp_dir;
        float t_angle = anchor_pose_angle + constraint.delta_angle;
        float target_angle = atan2(sin(t_angle), cos(t_angle));
// 	float target_angle = poses_[constraint.constrained_pose_id].angle;
// 	if (i == 1) {
// 	  std::cout << poses_[constraint.constrained_pose_id].angle << " ::: " << target_angle << std::endl;
// 	}
        problem->AddResidualBlock(
            new AutoDiffCostFunction<ColocationHumanImposedConstraint, 3, 3>(
              new ColocationHumanImposedConstraint(double(target_loc(0)), 
                                                   double(target_loc(1)), 
                                                   double(target_angle))), 
                                                   NULL, 
                                                   &(pose_array_[3 * constraint.constrained_pose_id]));
        num_hc_residuals_ += 3;
      }
      else if (constraint.constraint_type == vector_localization::CorrectionType::kColinearCorrection) {
        Eigen::Vector2f anchor_pose_loc = poses_[constraint.anchor_pose_id].translation;
        float anchor_pose_angle = poses_[constraint.anchor_pose_id].angle;
        Eigen::Vector2f para_dir = Eigen::Vector2f(cos(anchor_pose_angle), sin(anchor_pose_angle));
        Eigen::Vector2f perp_dir = Eigen::Vector2f(-para_dir(1), para_dir(0));
        Eigen::Vector2f target_loc = anchor_pose_loc + 
                                     constraint.delta_parallel * para_dir +
                                     constraint.delta_perpendicular * perp_dir;
        float t_angle = anchor_pose_angle + constraint.delta_angle;
        float target_angle = atan2(sin(t_angle), cos(t_angle));
        float penalty_dir = anchor_pose_angle + constraint.relative_penalty_dir;
        problem->AddResidualBlock(
            new AutoDiffCostFunction<ColinearHumanImposedConstraint, 2, 3>(
              new ColinearHumanImposedConstraint(double(target_loc(0)), 
                                                 double(target_loc(1)), 
                                                 double(target_angle),
                                                 double(penalty_dir))), NULL, 
                                                 &(pose_array_[3 * constraint.constrained_pose_id]));
        num_hc_residuals_ += 2;
      }
      else if (constraint.constraint_type == vector_localization::CorrectionType::kPerpendicularCorrection) {
        float anchor_pose_angle = poses_[constraint.anchor_pose_id].angle;
        float t_angle = anchor_pose_angle + constraint.delta_angle;
        float target_angle = atan2(sin(t_angle), cos(t_angle));
        problem->AddResidualBlock(
            new AutoDiffCostFunction<PerpendicularHumanImposedConstraint, 1, 1>(
              new PerpendicularHumanImposedConstraint(double(target_angle))), NULL, 
                                                   &(pose_array_[3 * constraint.constrained_pose_id + 2]));
        num_hc_residuals_ += 1;
      }
      else if (constraint.constraint_type == vector_localization::CorrectionType::kParallelCorrection) {
        float anchor_pose_angle = poses_[constraint.anchor_pose_id].angle;
        float t_angle = anchor_pose_angle + constraint.delta_angle;
        float target_angle = atan2(sin(t_angle), cos(t_angle));
        problem->AddResidualBlock(
            new AutoDiffCostFunction<ParallelHumanImposedConstraint, 1, 1>(
              new ParallelHumanImposedConstraint(double(target_angle))), NULL, 
                                                   &(pose_array_[3 * constraint.constrained_pose_id + 2]));
        num_hc_residuals_ += 1;
      }
    }
  }
}









ceres::TerminationType JointOpt::SolveHumanConstraints() {
  cout << "Begin solving problem" << endl;
  ceres::Solver::Options solver_options;
//   solver_options.use_nonmonotonic_steps = true;
  solver_options.minimizer_progress_to_stdout = true;
  //solver_options.min_relative_decrease = 0.00001;
  solver_options.max_num_iterations = 100;

  //solver_options.update_state_every_iteration = true;
  //SetSolverOptions(localization_options_, &solver_options);
  ceres::Problem problem;

//   cout << "Adding loop constraints" << endl;
  //AddLoopConstraint(odom_std_dev_weight, &problem);
  
  //TODO: compare original odom to poses
  cout << "Adding odom constraints" << endl;
  AddOdometryConstraints(&problem);
  
  //TODO: try geomfit again
  cout << "Adding human constraints" << endl;
  AddHumanConstraints(&problem);

  std::cout << "num poses: " << poses_.size() << std::endl; 
  
  cout << "Solving problem" << endl;
  ceres::Solver::Summary summary;
  cout << "iter\t  cost\t      cost_change\t  |gradient|\t  |step|  "
       << "tr_ratio\t tr_radius\t   ls_iter    \titer_time" << endl;
  ceres::Solve(solver_options, &problem, &summary);
  //cout << summary.BriefReport() << endl;
  cout << summary.FullReport() << endl;



  
  
  
  

//   int num_residuals = problem.NumResiduals();
//   cout << "num residuals: " << num_residuals << endl;
// 
//   vector<double> res;
//   vector<double>* residuals = &res;
//   for (int i = 0; i < num_residuals; ++i) {
//     residuals->push_back(0);
//   }
//   
// 
//   ceres::Problem::EvaluateOptions evalOptions = ceres::Problem::EvaluateOptions();
// 
//   gradients_.clear();
//   gradients_.resize(pose_array_.size());
//   for (size_t i = 0; i < pose_array_.size(); ++i) {
//     gradients_.push_back(0.0);
//   }
//   problem.Evaluate(evalOptions, NULL, residuals, &gradients_, &ceres_jacobian_);
  
  if (summary.termination_type == ceres::FAILURE || summary.termination_type == ceres::USER_FAILURE) {
    cout << "\nCeres failure\n" << endl;
    exit(1);
  }

  else if (summary.termination_type == ceres::CONVERGENCE ||
           summary.termination_type == ceres::USER_SUCCESS ||
           summary.termination_type == ceres::NO_CONVERGENCE) {

    cout << "\nCeres DID IT! Save this incremental progress now!\n" << endl;
  }
   
//   cout << "Solve finished running... evaluating result" << endl;

  return summary.termination_type;
}

















ceres::TerminationType JointOpt::PostHumanOptimization(int min_pose,
                                                       int max_pose) {
  FunctionTimer timer(__FUNCTION__);
  float odom_std_dev_weight = 1.0;

  timer.Lap(__LINE__);
  cout << "Performing post-HiTL optimization\n";
  ceres::Solver::Options solver_options;
  //solver_options.min_relative_decrease = 0.00001;

  SetSolverOptions(localization_options_, &solver_options);
  solver_options.minimizer_progress_to_stdout = true;
  solver_options.max_num_iterations = 100;
  // this time add STF constraints
  ceres::Problem problem;
  timer.Lap(__LINE__);
  cout << "Adding loop constraints" << endl;
  //AddLoopConstraint(odom_std_dev_weight, &problem);
  //AddOdometryConstraints(&problem);


  //AddHumanConstraints(&problem);


  cout << "Finding correspondences" << endl;
  timer.Lap(__LINE__);
  cout << "min_pose" << min_pose << endl;
  cout << "max_pose" << max_pose << endl;
  //FindVisualOdometryCorrespondences(min_pose, max_pose);
  //FindSTFCorrespondences(min_pose, max_pose);


  FindVisualOdometryCorrespondences(0, pose_array_.size()/3 - 1);
  FindSTFCorrespondences(0, pose_array_.size()/3 - 1);
  cout << "Add STF constraints" << endl;
  timer.Lap(__LINE__);
  AddSTFConstraints(&problem);




  problem.SetParameterBlockConstant(&pose_array_[0]);




  timer.Lap(__LINE__);
  cout << "Solving post-HitL problem" << endl;
  ceres::Solver::Summary summary;
  cout << "iter\t  cost\t      cost_change\t  |gradient|\t  |step|  "
       << "tr_ratio\t tr_radius\t   ls_iter    \titer_time" << endl;

  ceres::Solve(solver_options, &problem, &summary);
  //cout << summary.BriefReport() << endl;
  cout << summary.FullReport() << endl;


  /*
  //vector<Matrix2f> covariances;

  ceres::CRSMatrix jacobian;
  // Evaluate Jacobians
  ceres::Problem::EvaluateOptions evaluate_options;
  evaluate_options.num_threads = localization_options_.kNumThreads;
  for (size_t i = 0; i < pose_array_.size() / 3; ++i) {
     evaluate_options.parameter_blocks.push_back(&(pose_array_[3 * i]));
  }
  for (size_t i = 0; i < 2.0*(human_constraints_.size() - 1); ++i) {
    evaluate_options.parameter_blocks.push_back(&(hc_params_array_[4 * i]));
  }
  problem.Evaluate(evaluate_options, NULL, NULL, &gradients_, &jacobian);
  CHECK_EQ(gradients_.size(), pose_array_.size() +
                              8*(human_constraints_.size()-1));

*/




  int num_residuals = problem.NumResiduals();
  cout << "num residuals: " << num_residuals << endl;

  //double residuals[num_residuals];
  //double residuals = NULL;
  vector<double> res;
  vector<double>* residuals = &res;
  for (int i = 0; i < num_residuals; ++i) {
    residuals->push_back(0);
  }


  ceres::Problem::EvaluateOptions evalOptions = ceres::Problem::EvaluateOptions();

  gradients_.clear();
  gradients_.resize(pose_array_.size());
  //problem.Evaluate(evalOptions, NULL, residuals, NULL, &ceres_jacobian);
  problem.Evaluate(evalOptions, NULL, residuals, &gradients_, &ceres_jacobian_);


  return summary.termination_type;
}














void JointOpt::WriteCorrespondences() {
  //TODO glob or not glob????
  /*
  ScopedFile fid("PointPointCorr.txt", "w");
  for (size_t i = 0; i < point_point_correspondences_.size(); ++i) {
    const Rotation2Df pose_rotation(poses_[i].angle);
    const Affine2f pose_transform =
        Eigen::Translation2f(poses_[i].translation) * pose_rotation;
    for (unsigned int j = 0; j < robot_frame_point_clouds_[i].size(); ++j) {
      const Vector2f p = pose_transform * robot_frame_point_clouds_[i][j];
      const Vector2f n = pose_rotation *robot_frame_normal_clouds_[i][j];
      fprintf(fid(), "%.4f,%.4f,%.4f,%.4f,%.4f, %.4f,%.4f,"
              "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",
      poses_[i].translation.x(), poses_[i].translation.y(),
      poses_[i].angle, p.x(), p.y(), n.x(), n.y(),
      d3_covariances_[i](0,0), d3_covariances_[i](0,1), d3_covariances_[i](0,2),
      d3_covariances_[i](1,0), d3_covariances_[i](1,1), d3_covariances_[i](1,2),
      d3_covariances_[i](2,0), d3_covariances_[i](2,1),d3_covariances_[i](2,2));
    }
  }*/
  cout << "not ready yet..." << endl;
}


void JointOpt::Run() {
  // WatchFiles to track changes to config files.
  WatchFiles watch_files_;
  config_.init(watch_files_);
  config_.addFile("../robot.cfg");
  config_.addFile("config/vector_mapping.cfg");
  config_.addFile("config/localization_parameters.cfg");
  CHECK(LoadConfiguration(&localization_options_));

  ConvertPointClouds();
  CopyTempLaserScans();
  
  if (kdtrees_.size() == 0) { BuildKDTrees(); }

  SetParams();
  
  std::cout << "set params" << std::endl;
  
  cimg_library::CImg<float> info_base(3074, 3074, 1, 1, 0);

//   info_mat_(int(poses_.size()), int(poses_.size()), 1, 1, 0);
  std::cout << poses_.size() << std::endl;
  for (size_t i = 0; i < poses_.size(); ++i) {
    for (size_t j = 0; j < poses_.size(); ++j) {
      info_base(i, j) = 0.0;
    }
  }
  
  info_mat_ = &info_base;
  std::cout << info_mat_->height() << std::endl;
  std::cout << info_mat_->width() << std::endl;
  
  std::cout << "setup complete" << std::endl;
  
//   ceres::TerminationType term_type = ceres::CONVERGENCE;
//   if (human_constraints_.size() > 1) {
//     term_type = SolveHumanConstraints();
//   }
  
  ceres::TerminationType term_type = SolveHumanConstraints();

//   std::cout << "NHCR INSIDE: " << num_hc_residuals_ << std::endl;
  
  if (term_type == ceres::FAILURE || term_type == ceres::USER_FAILURE) {
    cout << "\nCeres failure\n" << endl;
    exit(1);
  }

  else if (term_type == ceres::CONVERGENCE ||
           term_type == ceres::USER_SUCCESS ||
           term_type == ceres::NO_CONVERGENCE) {

    cout << "\nCeres DID IT! Save this incremental progress now!\n" << endl;

    CopyParams();
    //WriteCorrespondences();

/*

    SetParams();
    //ceres::TerminationType post_opt_term_type = ceres::NO_CONVERGENCE;
    ceres::TerminationType post_opt_term_type =
                      PostHumanOptimization(backprop_start_,backprop_end_);

    if (post_opt_term_type == ceres::FAILURE ||
        post_opt_term_type == ceres::USER_FAILURE) {
      cout << "\nCeres failure\n" << endl;
      exit(1);
    }

    else if (post_opt_term_type == ceres::CONVERGENCE ||
             post_opt_term_type == ceres::USER_SUCCESS ||
             post_opt_term_type == ceres::NO_CONVERGENCE) {

      cout << "\nCeres DID IT (again)! Save progress now!\n" << endl;
      CopyParams();
    }
    */
  }
  
//   if (human_constraints_.size() == 16) {
//   std::cout << "finding stfs" << std::endl;
//   FindVisualOdometryCorrespondences(0, pose_array_.size()/3 - 1);
//   FindSTFCorrespondences(0, pose_array_.size()/3 - 1);
//   }
  const string infomat_image_file = StringPrintf("info_mat.png");
  info_mat_->save_png(infomat_image_file.c_str());
  //WriteCorrespondences();

}

}  // namespace vector_localization
