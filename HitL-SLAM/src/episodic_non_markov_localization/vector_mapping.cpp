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

#include "vector_mapping.h"

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
#include "../../map/vector_map.h"
#include "../../perception_tools/perception_2d.h"
#include "../../shared/math/eigen_helper.h"
#include "../../shared/math/util.h"
#include "../../shared/util/helpers.h"
#include "../../shared/util/pthread_utils.h"
#include "../../shared/util/timer.h"
//#include "vector_localization/residual_functors.h"
//#include <vectorparticlefilter.h>
#include "../../HitL/residual_functors.h"
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

void SetSolverOptions(
    const vector_localization::VectorMapping::VectorMappingOptions&
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




VectorMapping::VectorMapping(const string& maps_directory) :
    lost_metric_(0),
    terminate_(false),
    vector_map_(maps_directory.c_str()),
    t_last_update_(0) {
  CHECK_EQ(sem_init(&update_semaphore_, 0, 0), 0);
  CHECK_EQ(pthread_mutex_init(&update_mutex_, NULL), 0);
}

VectorMapping::~VectorMapping() {
  Terminate();
}

void VectorMapping::Terminate() {
  if (!terminate_) {
    terminate_ = true;
    CHECK_EQ(sem_post(&update_semaphore_), 0);
  }
}

Affine2f VectorMapping::RelativePoseTransform(
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
};
*/

vector<Matrix3d> VectorMapping::GetPoseCovariances() {
  return D3_covariances_;
}

vector< float > VectorMapping::GetCeresCost() {
  return ceres_cost_;
}

void VectorMapping::UpdateCovariances(size_t num_poses,
                                      vector<Matrix3d>* covars,
                                      vector<Matrix3d>* D3_covariances) {
  //copy current covariances
  D3_covariances_ = *covars;
  //make local copy
  *D3_covariances = D3_covariances_;
  if (D3_covariances->size() == 0) {
    Matrix3d cov_placeholder = Matrix3d::Identity();
    cov_placeholder = cov_placeholder*0.1;
    for (size_t i = 0; i < num_poses; ++i) {
      D3_covariances->push_back(cov_placeholder);
    }
  }
}

void VectorMapping::TryCorrCallback(vector<double> gradients,
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
}

void VectorMapping::CalculateConsistency() {
  if (localization_options_.EvaluateConsistency != NULL) {
    localization_options_.EvaluateConsistency(pose_array_, point_clouds_e_);
  }
}

void VectorMapping::AnimateCeres(vector<pair<Vector2f, Vector2f> > lines,
                                 vector<Vector2f> points) {
  if (localization_options_.AnimateCeres != NULL) {
    localization_options_.AnimateCeres(lines, points);
  }
}

void VectorMapping::FindVisualOdometryCorrespondences(
    const size_t min_poses, const size_t max_poses) {
  const float min_cosine_angle = cos(localization_options_.kMaxStfAngleError);
  const size_t poses_end = min(
      static_cast<size_t>(max_poses + 1), point_clouds_g_.size());
  if (poses_end < min_poses + 1) return;
  vector<vector<PointToPointCorrespondence>> pose_correspondences(
      poses_end - min_poses - 1);
  //OMP_PARALLEL_FOR
  for (size_t i = min_poses; i < poses_end - 1; ++i) {
    PointToPointCorrespondence correspondence;
    correspondence.source_pose = i;
    correspondence.target_pose = i + 1;
    Affine2f source_to_target_tf = RelativePoseTransform(i, i + 1);
    for (size_t k = 0; k < point_clouds_g_[i].size(); ++k) {
      correspondence.source_point = k;
      const Vector2f point(source_to_target_tf * point_clouds_e_[i][k]);
      const Vector2f normal =
          Rotation2Df(pose_array_[3 * i + 2 + 3] - pose_array_[3 * i + 2]) *
          normal_clouds_e_[i][k];
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

void VectorMapping::AddSTFConstraints(ceres::Problem* problem) {
  TIME_FUNCTION
  for (size_t i = 0; i < point_point_glob_correspondences_.size(); ++i) {
    PointToPointGlobCorrespondence& correspondence =
        point_point_glob_correspondences_[i];
    if (kUseRelativeConstraints) { // hardcoded false for right now
      CHECK_NE(correspondence.pose_index0, correspondence.pose_index1);
      PointToPointRelativeConstraint* constraint =
          new PointToPointRelativeConstraint(
          correspondence.pose_index0, correspondence.pose_index1,
          correspondence.points0, correspondence.points1,
          correspondence.normals0, correspondence.normals1,
          localization_options_.kLaserStdDev,
          1.0 / static_cast<double>(correspondence.points0.size()));
          //localization_options_.kPointPointCorrelationFactor);
      DynamicAutoDiffCostFunction<PointToPointRelativeConstraint,
          kDynamicDiffStride>* cost_function =
          new DynamicAutoDiffCostFunction<PointToPointRelativeConstraint,
              kDynamicDiffStride>(constraint);
      vector<double*> parameter_blocks;
      for (size_t j = 0;
           j <= max(correspondence.pose_index0, correspondence.pose_index1);
           ++j) {
        cost_function->AddParameterBlock(3);
        parameter_blocks.push_back(relative_pose_array_.data() + 3 * j);
      }
      cost_function->SetNumResiduals(2);
      problem->AddResidualBlock(cost_function, NULL, parameter_blocks);
    } else {
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
}

void VectorMapping::FindSTFCorrespondences(
    const size_t min_poses, const size_t max_poses) {
  static const size_t kMinInterPoseCorrespondence = 10;
  const float min_cosine_angle = cos(localization_options_.kMaxStfAngleError);
  // const float kMaxPoseSqDistance = sq(100.0);
  static const int kPointMatchSeparation = 1;
  const size_t poses_end = min(
      static_cast<size_t>(max_poses + 1), point_clouds_g_.size());
  CHECK_GT(pose_array_.size(), poses_end - 1);
  vector<vector<PointToPointGlobCorrespondence>> pose_point_correspondences(
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

        const Vector2f point(source_to_target_tf * point_clouds_e_[i][k]);
        const Vector2f normal =
            Rotation2Df(pose_array_[3 * j + 2] - pose_array_[3 * i + 2]) *
            normal_clouds_e_[i][k];
        KDNodeValue<float, 2> neighbor_point;
        const float closest_distance = kdtrees_[j]->FindNearestPointNormal(
            point, localization_options_.kPointMatchThreshold, &neighbor_point);
        const float cosine_angle = neighbor_point.normal.dot(normal);
        if (closest_distance < localization_options_.kPointMatchThreshold &&
            cosine_angle > min_cosine_angle) {
          // Valid point to point match found
          correspondence.points0_indices.push_back(k);
          correspondence.points1_indices.push_back(neighbor_point.index);
          correspondence.points0.push_back(point_clouds_e_[i][k]);
          correspondence.points1.push_back(
              point_clouds_e_[j][neighbor_point.index]);
          correspondence.normals0.push_back(normal_clouds_e_[i][k]);
          correspondence.normals1.push_back(
              normal_clouds_e_[j][neighbor_point.index]);
          source_point_indices.push_back(k);
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

void VectorMapping::AddLoopConstraint(
    const std::vector<Eigen::Matrix3d>& D3_covariances,
    const std::vector<Pose2Df>& poses,
    float scale,
    ceres::Problem* problem) {
  CHECK_EQ(poses.size(), D3_covariances.size());
  static const float kEpsilon = 1e-6;


  ceres_cost_.clear();
  for (size_t i = 0; i < poses.size(); ++i) {
    ceres_cost_.push_back(0.0);
  }

  for (size_t i=1; i<poses.size(); i++) {
    const Vector2f translation(poses[i].translation - poses[i-1].translation);
    Vector2f radial_direction;
    Vector2f tangential_direction;
    Matrix2f axis_transform;
    float radial_translation;
    float rotation;
    if (fabs(translation.x()) < kEpsilon && fabs(translation.y()) < kEpsilon) {
      radial_direction = Vector2f(cos(poses[i].angle), sin(poses[i].angle));
      radial_translation = 0.0;
    } else {
      radial_direction = Vector2f((Rotation2Df(-poses[i-1].angle) *
                                   translation).normalized());
      radial_translation = translation.norm();
    }
    tangential_direction = Vector2f(Rotation2Df(M_PI_2) * radial_direction);
    rotation = (poses[i].angle - poses[i-1].angle);
    axis_transform.block<1, 2>(0, 0) = radial_direction.transpose();
    axis_transform.block<1, 2>(1, 0) = tangential_direction.transpose();

    Eigen::Matrix2d trans_cov;
    trans_cov << D3_covariances[i-1](0,0),
                 D3_covariances[i-1](0,1),
                 D3_covariances[i-1](1,0),
                 D3_covariances[i-1](1,1);

    Vector2f dir1;
    Vector2f dir2;
    Eigen::EigenSolver<Matrix2d> es;
    es.compute(trans_cov, true);
    double lambda1 = es.eigenvalues()[0].real();
    double lambda2 = es.eigenvalues()[1].real();
    double sigma1 = sqrt(5.991*lambda1);
    double sigma2 = sqrt(5.991*lambda2);
    dir1(0) = es.eigenvectors().col(0)(0).real();
    dir1(1) = es.eigenvectors().col(0)(1).real();
    dir2(0) = es.eigenvectors().col(1)(0).real();
    dir2(1) = es.eigenvectors().col(1)(1).real();
    dir1.normalize();
    dir2.normalize();
    dir1 *= sigma1;
    dir2 *= sigma2;

    float r_std_dev = float(sqrt(pow(radial_direction.dot(dir1),2) +
        pow(radial_direction.dot(dir2),2)));
    float t_std_dev = float(sqrt(pow(radial_direction.dot(dir1),2) +
        pow(radial_direction.dot(dir2),2)));
    float a_std_dev = float(sqrt(D3_covariances[i-1](2,2)));

    float radial_std_dev = bound<float>(
      r_std_dev,
      localization_options_.kOdometryTranslationMinStdDev,
      localization_options_.kOdometryTranslationMaxStdDev);
    float tangential_std_dev = bound<float>(
      t_std_dev,
      localization_options_.kOdometryTranslationMinStdDev,
      localization_options_.kOdometryTranslationMaxStdDev);
    float angular_std_dev = bound<float>(
      a_std_dev,
      localization_options_.kOdometryAngularMinStdDev,
      localization_options_.kOdometryAngularMaxStdDev);


    if (!std::isfinite(radial_std_dev)) {
      //cout << "radial std dev is not finite: "
      //     << radial_std_dev
      //     << endl;
      //cout << "setting radial std dev to: "
      //     << localization_options_.kOdometryTranslationMinStdDev
      //     << endl;
      radial_std_dev = localization_options_.kOdometryTranslationMinStdDev;
    }
    if (!std::isfinite(tangential_std_dev)) {
      //cout << "tangential std dev is not finite: "
      //     << tangential_std_dev
      //     << endl;
      //cout << "setting tangential std dev to: "
      //     << localization_options_.kOdometryTranslationMinStdDev
      //     << endl;
      tangential_std_dev = localization_options_.kOdometryTranslationMinStdDev;
    }
    if (!std::isfinite(angular_std_dev)) {
      //cout << "angular std dev is not finite: "
      //     << angular_std_dev
      //     << endl;
      //cout << "setting angular std dev to: "
      //     << localization_options_.kOdometryAngularMinStdDev
      //     << endl;
      angular_std_dev = localization_options_.kOdometryAngularMinStdDev;
    }
    if (!std::isfinite(radial_translation)) {
      //cout << "radial translation is not finite: "
      //     << radial_translation
      //     << endl;
      //cout << "setting radial translation to: "
      //     << 0.0
      //     << endl;
      radial_translation = 0.0;
    }
    problem->AddResidualBlock(
        new AutoDiffCostFunction<PoseConstraint, 3, 3, 3>(
            new PoseConstraint(axis_transform,
                               scale * radial_std_dev,
                               scale * tangential_std_dev,
                               scale * angular_std_dev,
                               radial_translation,
                               rotation)),
            NULL,
            &(pose_array_[3 * (i-1)]),
            &(pose_array_[3 * i]));

    // calc init cost
    // Compute translation difference between pose2 and pose1.
    Eigen::Vector2f trans4cost = poses[i].translation - poses[i-1].translation;

    // Transform relative pose to the reference frame of pose1.
    const Eigen::Rotation2D<float> pose_rotation(-poses[i-1].angle);
    trans4cost = pose_rotation * trans4cost;
    // covariance.
    trans4cost = axis_transform * trans4cost;

    float r0 = (trans4cost(0) - radial_translation) / radial_std_dev;
    float r1 =  trans4cost(1) / tangential_std_dev;
    float r2 = (poses[i].angle - poses[i-1].angle - rotation) / angular_std_dev;
    ceres_cost_[i] += (r0*r0 + r1*r1 + r2*r2);
  }
}

struct segDistResidual {
  segDistResidual(double px, double py, double cmx, double cmy,
                  double len, double N) : px_(px), py_(py), cmx_(cmx),
                                          cmy_(cmy), len_(len), N_(N) {}
  template <typename T> bool operator()(const T* const theta, T* residual)
                                        const {

    Eigen::Matrix<T, 2, 1> normal(-sin(theta[0]), cos(theta[0]));

    Eigen::Matrix<T, 2, 1> alpha(cos(theta[0]), sin(theta[0]));
    alpha.normalize();

    Eigen::Matrix<T, 2, 1> point_vec(T(px_) - (T(cmx_) + (T(len_))*T(alpha[0])),
                                    T(py_) - (T(cmy_) + (T(len_))*T(alpha[1])));

    residual[0] = normal.dot(point_vec);
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

vector<Vector2f> VectorMapping::SegFit(double* p1, double* p2, double* cm,
                                        double* data, int size) {
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
  ceres::Problem problem;

  for (int i = 0; i<size; ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<segDistResidual, 1, 1>(
        new segDistResidual(data[2*i], data[2*i + 1], icm[0], icm[1], hy/2.0,
                       double(size))), NULL, theta);
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

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

// vector<pair<Vector2f, Vector2f>> VectorMapping::RefitConstraints(
//                                                        vector<HumanConstraint>*
//                                                        human_constraints_ptr) {
//   //automatic endpoint adjustment
//   vector<HumanConstraint> human_constraints = *human_constraints_ptr;
//   // TODO this needs to handle "corner" entries
//   vector<pair<Vector2f, Vector2f>> new_P_sets;
//   vector<pair<Vector2f, Vector2f>> new_P_sets_a;
//   vector<pair<Vector2f, Vector2f>> new_P_sets_b;
// 
// 
//   for (size_t k = 0; k < human_constraints.size(); k++) {
//     int num_inliers = 0;
//     vector<Vector2f> inlier_set;
//     HumanConstraint H = human_constraints[k];
//     for (size_t i=0; i<H.pose_obs_ids_a.size(); i++) {
//       vector<pair<int, vector<int>>> po = H.pose_obs_ids_a;
//       for (size_t j=0; j<po[i].second.size(); ++j) {
//           inlier_set.push_back(
//             local_version_point_clouds_[po[i].first][po[i].second[j]]);
//           num_inliers++;
// 
//           temp_inliers_.push_back(
//             local_version_point_clouds_[po[i].first][po[i].second[j]]);
//       }
//     }
// 
//     //compute centroid, scatter matrix components
//     Eigen::Matrix2f scatter = Eigen::Matrix2f::Zero();
//     Vector2f p_bar(0.0, 0.0);
//     for (size_t i = 0; i < inlier_set.size(); ++i) {
//       p_bar += inlier_set[i];
//       scatter += inlier_set[i]*inlier_set[i].transpose();
//     }
//     p_bar = p_bar/float(num_inliers);
// 
//     Vector2f line_dir;
//     Vector2f dir1;
//     Vector2f dir2;
//     Eigen::EigenSolver<Matrix2f> es;
//     es.compute(scatter, true);
//     float lambda1 = float(es.eigenvalues()[0].real());
//     float lambda2 = float(es.eigenvalues()[1].real());
//     dir1(0) = es.eigenvectors().col(0)(0).real();
//     dir1(1) = es.eigenvectors().col(0)(1).real();
//     dir2(0) = es.eigenvectors().col(1)(0).real();
//     dir2(1) = es.eigenvectors().col(1)(1).real();
//     if (fabs(lambda1) > fabs(lambda2)) {
//       line_dir = dir1;
//     }
//     else {
//       line_dir = dir2;
//     }
//     line_dir.normalize();
//     float max_t = 0.0;
//     float min_t = 0.0;
//     Vector2f max_t_point;
//     Vector2f min_t_point;
//     for (size_t i = 0; i < inlier_set.size(); ++i) {
//       float t = (inlier_set[i] - p_bar).dot(line_dir);
//       if (t > max_t) {
//         max_t = t;
//         max_t_point = inlier_set[i];
//       }
//       else if (t < min_t) {
//         min_t = t;
//         min_t_point = inlier_set[i];
//       }
//     }
//     line_dir = max_t_point - min_t_point;
//     line_dir.normalize();
//     Vector2f ep1 = p_bar + min_t*line_dir;
//     Vector2f ep2 = p_bar + max_t*line_dir;
// 
// 
//     double CM[2];
//     CM[0] = double(p_bar(0));
//     CM[1] = double(p_bar(1));
//     double P1[2];
//     double P2[2];
//     P1[0] = double(ep1(0));
//     P1[1] = double(ep1(1));
//     P2[0] = double(ep2(0));
//     P2[1] = double(ep2(1));
//     double data[2*num_inliers];
//     for (int j = 0; j < num_inliers; j++) {
//       data[2*j] = double(inlier_set[j][0]);
//       data[2*j+1] = double(inlier_set[j][1]);
//     }
// 
//     vector<Vector2f> new_endpoints = SegFit(P1, P2, CM, data, num_inliers);
//     Vector2f new_line_dir = new_endpoints[1] - new_endpoints[0];
//     new_line_dir.normalize();
// 
//     max_t = 0.0;
//     min_t = 0.0;
//     for (size_t i = 0; i < inlier_set.size(); ++i) {
//       float t = (inlier_set[i] - p_bar).dot(new_line_dir);
//       max_t = max(max_t, t);
//       min_t = min(min_t, t);
//     }
//     ep1 = p_bar + min_t*new_line_dir;
//     ep2 = p_bar + max_t*new_line_dir;
// 
//     Vector2f cm = (ep1 + ep2)/2.0;
// 
//     Vector2f nhat = Rotation2Df(M_PI/2.0)*new_line_dir;
// 
//     //Vector2f nhat = Rotation2Df(M_PI/2.0)*line_dir;
//     pair<Vector2f, Vector2f> new_pair = make_pair(cm, nhat);
//     new_P_sets_a.push_back(new_pair);
// 
// 
// 
//   }
// 
//   for (size_t k = 0; k < human_constraints.size(); k++) {
//     int num_inliers = 0;
//     vector<Vector2f> inlier_set;
//     HumanConstraint H = human_constraints[k];
//     for (size_t i=0; i<H.pose_obs_ids_b.size(); i++) {
//       vector<pair<int, vector<int>>> po = H.pose_obs_ids_b;
//       for (size_t j=0; j<po[i].second.size(); ++j) {
//           inlier_set.push_back(
//             local_version_point_clouds_[po[i].first][po[i].second[j]]);
//           num_inliers++;
// 
//           temp_inliers_.push_back(
//             local_version_point_clouds_[po[i].first][po[i].second[j]]);
//       }
//     }
// 
//     //compute centroid, scatter matrix components
//     Eigen::Matrix2f scatter = Eigen::Matrix2f::Zero();
//     Vector2f p_bar(0.0, 0.0);
//     for (size_t i = 0; i < inlier_set.size(); ++i) {
//       p_bar += inlier_set[i];
//       scatter += inlier_set[i]*inlier_set[i].transpose();
//     }
//     p_bar = p_bar/float(num_inliers);
// 
//     Vector2f line_dir;
//     Vector2f dir1;
//     Vector2f dir2;
//     Eigen::EigenSolver<Matrix2f> es;
//     es.compute(scatter, true);
//     float lambda1 = float(es.eigenvalues()[0].real());
//     float lambda2 = float(es.eigenvalues()[1].real());
//     dir1(0) = es.eigenvectors().col(0)(0).real();
//     dir1(1) = es.eigenvectors().col(0)(1).real();
//     dir2(0) = es.eigenvectors().col(1)(0).real();
//     dir2(1) = es.eigenvectors().col(1)(1).real();
//     if (fabs(lambda1) > fabs(lambda2)) {
//       line_dir = dir1;
//     }
//     else {
//       line_dir = dir2;
//     }
//     line_dir.normalize();
//     float max_t = 0.0;
//     float min_t = 0.0;
//     Vector2f max_t_point;
//     Vector2f min_t_point;
//     for (size_t i = 0; i < inlier_set.size(); ++i) {
//       float t = (inlier_set[i] - p_bar).dot(line_dir);
//       if (t > max_t) {
//         max_t = t;
//         max_t_point = inlier_set[i];
//       }
//       else if (t < min_t) {
//         min_t = t;
//         min_t_point = inlier_set[i];
//       }
//     }
//     line_dir = max_t_point - min_t_point;
//     line_dir.normalize();
//     Vector2f ep1 = p_bar + min_t*line_dir;
//     Vector2f ep2 = p_bar + max_t*line_dir;
// 
// 
// 
//     double CM[2];
//     CM[0] = double(p_bar(0));
//     CM[1] = double(p_bar(1));
//     double P1[2];
//     double P2[2];
//     P1[0] = double(ep1(0));
//     P1[1] = double(ep1(1));
//     P2[0] = double(ep2(0));
//     P2[1] = double(ep2(1));
//     double data[2*num_inliers];
//     for (int j = 0; j < num_inliers; j++) {
//       data[2*j] = double(inlier_set[j][0]);
//       data[2*j+1] = double(inlier_set[j][1]);
//     }
// 
//     vector<Vector2f> new_endpoints = SegFit(P1, P2, CM, data, num_inliers);
//     Vector2f new_line_dir = new_endpoints[1] - new_endpoints[0];
//     new_line_dir.normalize();
// 
//     max_t = 0.0;
//     min_t = 0.0;
//     for (size_t i = 0; i < inlier_set.size(); ++i) {
//       float t = (inlier_set[i] - p_bar).dot(new_line_dir);
//       max_t = max(max_t, t);
//       min_t = min(min_t, t);
//     }
//     ep1 = p_bar + min_t*new_line_dir;
//     ep2 = p_bar + max_t*new_line_dir;
// 
//     Vector2f cm = (ep1 + ep2)/2.0;
// 
//     Vector2f nhat = Rotation2Df(M_PI/2.0)*line_dir;
//     //Vector2f nhat = Rotation2Df(M_PI/2.0)*new_line_dir;
// 
//     pair<Vector2f, Vector2f> new_pair = make_pair(cm, nhat);
//     new_P_sets_b.push_back(new_pair);
// 
//         cout << "pbar B:\n" << p_bar << endl;
//   }
// 
//   CHECK_EQ(new_P_sets_a.size(), new_P_sets_b.size());
//   for (size_t i = 0; i < new_P_sets_a.size(); ++i) {
//     new_P_sets.push_back(new_P_sets_a[i]);
//     new_P_sets.push_back(new_P_sets_b[i]);
//   }
//   return new_P_sets;
// } //end automatic end point adjustment

// void VectorMapping::AddHumanConstraints(
//     const vector<HumanConstraint>& human_constraints,
//     const vector<Pose2Df>& poses,
//     ceres::Problem* problem) {
// 
//   //hc_params_array_.clear();
//   vector<pair<Vector2f, Vector2f> > lines;
//   vector<Vector2f> points;
//    //cout << "STILL POST OPT" << endl;
//   for (size_t i = 0; i < human_constraints.size(); ++i) {
// 
//     Vector2f line_dir = Rotation2Df(-M_PI/2.0) * human_constraints[i].n_a;
//     Vector2f ep1 = human_constraints[i].cm_a +
//                   (human_constraints[i].len_a/2.0) * line_dir;
//     Vector2f ep2 = human_constraints[i].cm_a -
//                   (human_constraints[i].len_a/2.0) * line_dir;
//     pair<Vector2f, Vector2f> new_line = make_pair(ep1, ep2);
//     lines.push_back(new_line);
// 
// 
//     line_dir = Rotation2Df(-M_PI/2.0) * human_constraints[i].n_b;
//     ep1 = human_constraints[i].cm_b + (human_constraints[i].len_b/2.0) *
//                                                         line_dir;
//     ep2 = human_constraints[i].cm_b - (human_constraints[i].len_b/2.0) *
//                                                         line_dir;
//     new_line = make_pair(ep1, ep2);
//     lines.push_back(new_line);
// 
//   }
// 
//   points = temp_inliers_;
//   cout << "points size PRE: " << points.size() << endl;
//   temp_inliers_.clear();
//  // AnimateCeres(lines, points);
// 
//   //if (human_constraints.size() > 1) {
//    // exit(1);
//   //}
//   cout << "hc params size: " << hc_params_array_.size() << endl;
//   cout << "PRE OPT HC ARRAY" << endl;
//   for (size_t i = 0; i < human_constraints.size()-1; ++i) {
//   //for (size_t i = 0; i < 1; ++i) {
// 
// 
//     hc_params_array_[8 * i] = double(human_constraints[i].cm_a(0));
//     hc_params_array_[8 * i + 1] = double(human_constraints[i].cm_a(1));
//     hc_params_array_[8 * i + 2] = double(human_constraints[i].n_a(0));
//     hc_params_array_[8 * i + 3] = double(human_constraints[i].n_a(1));
//     hc_params_array_[8 * i + 4] = double(human_constraints[i].cm_b(0));
//     hc_params_array_[8 * i + 5] = double(human_constraints[i].cm_b(1));
//     hc_params_array_[8 * i + 6] = double(human_constraints[i].n_b(0));
//     hc_params_array_[8 * i + 7] = double(human_constraints[i].n_b(1));
//     //cout << "hc params size: " << hc_params_array_.size() << endl;
// 
// 
//     //for (size_t j = 0; j < hc_params_array_.size(); ++j) {
//     for (size_t j = 0; j < 8; ++j) {
//       cout << hc_params_array_[8*i + j] << endl;
//       //cout << test_array_[j] << endl;
//     }
//     if (human_constraints[i].mode == 2 || human_constraints[i].mode == 4 ||
//         human_constraints[i].mode == 5 || human_constraints[i].mode == 6) {
// 
//       vector<double*> parameter_blocks_a;
//       vector<double*> parameter_blocks_b;
//       CHECK_EQ(human_constraints[i].obs_a.size(),
//                human_constraints[i].x_a.size());
//       CHECK_EQ(human_constraints[i].obs_b.size(),
//                human_constraints[i].x_b.size());

//       //Ra
//       LineFitHumanImposedConstraint* constraint_a =
//           new LineFitHumanImposedConstraint(human_constraints[i].obs_a,
//                                             human_constraints[i].len_a);
// 
//       DynamicAutoDiffCostFunction<
//           LineFitHumanImposedConstraint, kDynamicDiffStride>*
//       cost_function_a = new DynamicAutoDiffCostFunction<
//           LineFitHumanImposedConstraint, kDynamicDiffStride>(constraint_a);
// 
//       cost_function_a->AddParameterBlock(4);
//       parameter_blocks_a.push_back(&(hc_params_array_[8 * i]));
// 
//       for (size_t j = 0; j < human_constraints[i].x_a.size(); ++j) {
//         cost_function_a->AddParameterBlock(3);
//         int pose_index = human_constraints[i].x_a[j];
//         parameter_blocks_a.push_back(&(pose_array_[3 * pose_index]));
//       }
// 
//       cost_function_a->SetNumResiduals(1);
//       cout << "parameter_blocks_a size: " << parameter_blocks_a.size() << endl;
//       problem->AddResidualBlock(cost_function_a, NULL, parameter_blocks_a);
// 
//       //parameter_blocks_a.clear();
// 
// 
//       //Rb
//       LineFitHumanImposedConstraint* constraint_b =
//           new LineFitHumanImposedConstraint(human_constraints[i].obs_b,
//                                             human_constraints[i].len_b);
// 
//       DynamicAutoDiffCostFunction<
//           LineFitHumanImposedConstraint, kDynamicDiffStride>*
//       cost_function_b = new DynamicAutoDiffCostFunction<
//           LineFitHumanImposedConstraint, kDynamicDiffStride>(constraint_b);
// 
//       cost_function_b->AddParameterBlock(4);
//       parameter_blocks_b.push_back(&(hc_params_array_[8 * i + 4]));
// 
//       for (size_t j = 0; j < human_constraints[i].x_b.size(); ++j) {
//         cost_function_b->AddParameterBlock(3);
//         int pose_index = human_constraints[i].x_b[j];
//         parameter_blocks_b.push_back(&(pose_array_[3 * pose_index]));
//       }
// 
//       cost_function_b->SetNumResiduals(1);
//       cout << "parameter_blocks size: " << parameter_blocks_b.size() << endl;
//       problem->AddResidualBlock(cost_function_b, NULL, parameter_blocks_b);

//       //Rp
//       if (human_constraints[i].mode == 2) { //TODO: corner, point
//         problem->AddResidualBlock(
//           new AutoDiffCostFunction<ColocationHumanImposedConstraint, 5, 4, 4>(
//             new ColocationHumanImposedConstraint()),
//                 NULL, &(hc_params_array_[8 * i]), &(hc_params_array_[8*i + 4]));
//       }
//       else if (human_constraints[i].mode == 4) { // colinear
//         problem->AddResidualBlock(
//           new AutoDiffCostFunction<ColinearHumanImposedConstraint, 4, 4, 4>(
//             new ColinearHumanImposedConstraint(human_constraints[i].len_a)),
//                 NULL, &(hc_params_array_[8 * i]), &(hc_params_array_[8*i + 4]));
//       }
//       else if (human_constraints[i].mode == 5) { // perpendicular
//         problem->AddResidualBlock(
//          new AutoDiffCostFunction<PerpendicularHumanImposedConstraint, 3, 4, 4>(
//             new PerpendicularHumanImposedConstraint(human_constraints[i].len_a))
//               , NULL, &(hc_params_array_[8 * i]), &(hc_params_array_[8*i + 4]));
//       }
//       else if (human_constraints[i].mode == 7) { //parallel
//           problem->AddResidualBlock(
//             new AutoDiffCostFunction<ParallelHumanImposedConstraint, 3, 4, 4>(
//               new ParallelHumanImposedConstraint(human_constraints[i].len_a)),
//                 NULL, &(hc_params_array_[8 * i]), &(hc_params_array_[8*i + 4]));
//       }
//     }
//     else {
//       cout << "mode not recognized / not supported yet." << endl;
//     }
// 
//   }
// }

void VectorMapping::AddPoseConstraints(
    const size_t min_poses, const size_t max_poses,
    const std::vector<Pose2Df>& poses,
    ceres::Problem* problem) {
  static const bool debug = false;
  TIME_FUNCTION
  static const float kEpsilon = 1e-6;
  for (size_t i = min_poses + 1; i <= max_poses && i < poses.size(); ++i) {
    // Create a new pose constraint residual from the odometry constraint
    // between pose i and pose i-1, and add it to the Ceres problem.
    const Vector2f translation(poses[i].translation - poses[i-1].translation);
    Vector2f radial_direction;
    Vector2f tangential_direction;
    float rotation;
    Matrix2f axis_transform;
    float radial_translation = 0.0;
    if (fabs(translation.x()) < kEpsilon && fabs(translation.y()) < kEpsilon) {
      radial_direction = Vector2f(cos(poses[i].angle), sin(poses[i].angle));
      tangential_direction = Vector2f(Rotation2Df(M_PI_2) * radial_direction);
      rotation = angle_mod(poses[i].angle - poses[i-1].angle);
      axis_transform.block<1, 2>(0, 0) = radial_direction.transpose();
      axis_transform.block<1, 2>(1, 0) = tangential_direction.transpose();
      radial_translation = 0.0;
    }
    else {
      radial_direction = Vector2f(
          (Rotation2Df(-poses[i - 1].angle) * translation).normalized());
      tangential_direction = Vector2f(Rotation2Df(M_PI_2) * radial_direction);
      rotation = angle_mod(poses[i].angle - poses[i-1].angle);
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

    if (debug) {
      printf("Adding pose constraint %d:%d @ 0x%lx : 0x%lx\n",
             static_cast<int>(i - 1), static_cast<int>(i),
             reinterpret_cast<uint64_t>(&(pose_array_[3 * i - 3])),
             reinterpret_cast<uint64_t>(&(pose_array_[3 * i])));
    }
    if (kUseRelativeConstraints) {
      RelativePoseConstraint* constraint = new RelativePoseConstraint(
          i - 1, i, axis_transform, radial_std_dev, tangential_std_dev,
          angular_std_dev, radial_translation, rotation);
      DynamicAutoDiffCostFunction<RelativePoseConstraint, kDynamicDiffStride>*
          cost_function = new DynamicAutoDiffCostFunction<
              RelativePoseConstraint, kDynamicDiffStride>(constraint);
      vector<double*> parameter_blocks;
      for (size_t j = 0; j <= i; ++j) {
        cost_function->AddParameterBlock(3);
        parameter_blocks.push_back(relative_pose_array_.data() + 3 * j);
      }
      cost_function->SetNumResiduals(3);
      problem->AddResidualBlock(cost_function, NULL, parameter_blocks);
    }
    else {
      problem->AddResidualBlock(
        new AutoDiffCostFunction<PoseConstraint, 3, 3, 3>(
            new PoseConstraint(axis_transform,
                               radial_std_dev,
                               tangential_std_dev,
                               angular_std_dev,
                               radial_translation,
                               rotation)),
            NULL,
            &(pose_array_[3 * i - 3]),
            &(pose_array_[3 * i]));
      if (i == min_poses + 1) {
        problem->SetParameterBlockConstant(&pose_array_[3*(i-1)]);
      }
    }
  }
}

void VectorMapping::ConvertPointCloud(
    const PointCloudf& point_cloud, vector<vector2f>* point_cloud_converted_ptr)
    const {
  vector<vector2f>& point_cloud_converted = *point_cloud_converted_ptr;
  point_cloud_converted.resize(point_cloud.size());
  for (size_t j = 0; j < point_cloud.size(); ++j) {
    point_cloud_converted[j] =
      vector2f(point_cloud[j].x(), point_cloud[j].y());
  }
}

void VectorMapping::ConvertPointClouds(
  const vector<PointCloudf>& point_clouds) {
  point_clouds_g_.resize(point_clouds.size());
  for (size_t i = 0; i < point_clouds.size(); ++i) {
    const PointCloudf& point_cloud = point_clouds[i];
    point_clouds_g_[i].resize(point_cloud.size());
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      point_clouds_g_[i][j] = vector2f(point_cloud[j].x(), point_cloud[j].y());
    }
  }
}

void VectorMapping::BuildKDTrees(
    const vector< PointCloudf >& point_clouds,
    const vector< NormalCloudf >& normal_clouds) {
  CHECK_EQ(point_clouds.size(), normal_clouds.size());
  kdtrees_.resize(point_clouds.size(), NULL);
  const unsigned int num_point_clouds = point_clouds.size();
  OMP_PARALLEL_FOR
  for (size_t i = 0; i < num_point_clouds; ++i) {
    const PointCloudf& point_cloud = point_clouds[i];
    const NormalCloudf& normal_cloud = normal_clouds[i];
    CHECK_EQ(point_cloud.size(), normal_cloud.size());
    vector<KDNodeValue<float, 2>> values(point_cloud.size());
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      values[j].index = j;
      values[j].point = point_cloud[j];
      values[j].normal = normal_cloud[j];
    }
    if (values.size() > 0)
      kdtrees_[i] = new KDTree<float, 2>(values);
    else
      kdtrees_[i] = new KDTree<float, 2>();
  }
}

void VectorMapping::ResetGlobalPoses(
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

Eigen::MatrixXd CalculateJTJ(const ceres::CRSMatrix& jacobian) {

  int num_rows = jacobian.num_rows;
  // Convert the sparse matrix to a dense matrix

  // Convert the CRS Matrix to an eigen matrix
  Eigen::MatrixXd denseJacobian = Eigen::MatrixXd::Zero(num_rows,
      jacobian.num_cols);
  for(size_t k = 0; k < jacobian.rows.size() - 1; ++k) {
    size_t row = jacobian.rows[k];
    size_t nextRow = jacobian.rows[k + 1];
    for(size_t l = row; l < nextRow; ++l) {
      int column = jacobian.cols[l];
      double value = jacobian.values[l];
        //cout << row << " " << column << " " << value << endl;
        denseJacobian(k,column) = value;
    }
  }

  //denseJacobian = Eigen::MatrixXd(jacobianMatrix);
  // Calculate j.transpose j
  Eigen::MatrixXd jTj = denseJacobian.transpose() * denseJacobian;

  return jTj;
}

Eigen::MatrixXd ConvertJ(const ceres::CRSMatrix& jacobian) {

  int num_rows = jacobian.num_rows;
  // Convert the sparse matrix to a dense matrix

  // Convert the CRS Matrix to an eigen matrix
  Eigen::MatrixXd denseJacobian = Eigen::MatrixXd::Zero(num_rows,
      jacobian.num_cols);
  for(size_t k = 0; k < jacobian.rows.size() - 1; ++k) {
    size_t row = jacobian.rows[k];
    size_t nextRow = jacobian.rows[k + 1];
    for(size_t l = row; l < nextRow; ++l) {
      int column = jacobian.cols[l];
      double value = jacobian.values[l];
        //cout << row << " " << column << " " << value << endl;
        denseJacobian(k,column) = value;
    }
  }

  //denseJacobian = Eigen::MatrixXd(jacobianMatrix);
  // Calculate j.transpose j
  //Eigen::MatrixXd jTj = denseJacobian.transpose() * denseJacobian;

  return denseJacobian;
}


void VectorMapping::SetCorrectionRelations(vector<pair<int, vector<int>>>
                                           first_poses_obs,
                                           vector<pair<int, vector<int>>>
                                           second_poses_obs,
                                           const vector<Pose2Df>& poses) {

  corrected_pose_obs_.clear();
  correcting_pose_obs_.clear();
  corrected_pose_obs_ = first_poses_obs;
  correcting_pose_obs_ = second_poses_obs;

  corrected_poses_.clear();
  correcting_poses_.clear();
  for (size_t i = 0; i < first_poses_obs.size(); i++) {
    corrected_poses_.push_back(first_poses_obs[i].first);
  }
  for (size_t i = 0; i < second_poses_obs.size(); i++) {
    correcting_poses_.push_back(second_poses_obs[i].first);
  }

  corrected_observations_.clear();
  correcting_observations_.clear();
  for (size_t i = 0; i < first_poses_obs.size(); i++) {
    vector<int> obs_index = first_poses_obs[i].second;
    vector<pair<double, double>> relative_transforms;
    int pose_index = first_poses_obs[i].first;
    for (size_t j = 0; j < first_poses_obs[i].second.size(); ++j) {
      Vector2f obs = local_version_point_clouds_[pose_index][obs_index[j]];
      Vector2f to_obs = obs - poses[pose_index].translation;
      double alpha = double(to_obs.norm());
      to_obs.normalize();
      Vector2f pose_dir(cos(poses[pose_index].angle),
                        sin(poses[pose_index].angle));

      double theta = double(acos(to_obs.dot(pose_dir)));
      Eigen::Vector3f to_obs3(to_obs(0), to_obs(1), 0);
      Eigen::Vector3f pose_dir3(pose_dir(0), pose_dir(1), 0);
      if (pose_dir3.cross(to_obs3)(2) < 0.0) { //pose_dir cross to_obs
        theta = -theta;
      }
      if (isnan(theta)) { // sometimes object is dead-ahead
        theta = 0.0;
        cout << "THETA WAS NAN" << endl;
      }
      if (isnan(alpha)) {
        cout << "ALPHA WAS NAN" << endl;
      }
      pair<double, double> trans = make_pair(alpha, theta);
      relative_transforms.push_back(trans);
    }
    pair<vector<pair<double, double>>, int> pose_obs_pair =
        make_pair(relative_transforms, i);
    corrected_observations_.push_back(pose_obs_pair);
  }

  for (size_t i = 0; i < second_poses_obs.size(); i++) {
    vector<int> obs_index = second_poses_obs[i].second;
    vector<pair<double, double>> relative_transforms;
    int pose_index = second_poses_obs[i].first;
    for (size_t j = 0; j < second_poses_obs[i].second.size(); ++j) {
      Vector2f obs = local_version_point_clouds_[pose_index][obs_index[j]];
      Vector2f to_obs = obs - poses[pose_index].translation;
      double alpha = double(to_obs.norm());
      to_obs.normalize();
      Vector2f pose_dir(cos(poses[pose_index].angle),
                        sin(poses[pose_index].angle));

      double theta = double(acos(to_obs.dot(pose_dir)));
      Eigen::Vector3f to_obs3(to_obs(0), to_obs(1), 0);
      Eigen::Vector3f pose_dir3(pose_dir(0), pose_dir(1), 0);
      if (pose_dir3.cross(to_obs3)(2) < 0.0) { //pose_dir cross to_obs
        theta = -theta;
      }
      if (isnan(theta)) { // sometimes object is dead-ahead
        theta = 0.0;
        cout << "THETA WAS NAN" << endl;
      }
      if (isnan(alpha)) {
        cout << "ALPHA WAS NAN" << endl;
      }
      pair<double, double> trans = make_pair(alpha, theta);
      relative_transforms.push_back(trans);
    }
    pair<vector<pair<double, double>>, int> pose_obs_pair =
        make_pair(relative_transforms, i);
    correcting_observations_.push_back(pose_obs_pair);
  }

}



double VectorMapping::distToLineSeg(Vector2f p1, Vector2f p2, Vector2f p) {
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
  VectorMapping::EstablishObservationSets(bool corners, vector<Vector2f>*
                                                        selected_points_ptr) {

  vector<Vector2f> selected_points = *selected_points_ptr;
  vector<pair<int, vector<int>>> first_poses_observations;
  vector<pair<int, vector<int>>> second_poses_observations;

  double threshold = 0.03; //3cm 'pill-shaped' envelope around line
  for (size_t i=0; i<local_version_point_clouds_.size(); i++) {
    vector<int> first_obs;
    vector<int> second_obs;
    for (size_t j=0; j<local_version_point_clouds_[i].size(); j++) {
      if (!corners) {
        // first selection
        //const double dst_to_first_selection = DistanceToLineSegment(
        const double dst_to_first_selection = distToLineSeg(
                                                        selected_points[0],
                                                        selected_points[1],
                                            local_version_point_clouds_[i][j]);
        if (dst_to_first_selection < threshold) {
          first_obs.push_back(j);
        }
        // second selection
        //const double dst_to_second_selection = DistanceToLineSegment(
        const double dst_to_second_selection = distToLineSeg(
                                                        selected_points[2],
                                                        selected_points[3],
                                            local_version_point_clouds_[i][j]);
        if (dst_to_second_selection < threshold) {
          second_obs.push_back(j);
        }
      }
      // corner case
      else {

        // first selection
        const double dst1 = DistanceToLineSegment(selected_points[0],
                                                  selected_points[1],

        local_version_point_clouds_[i][j]);
        const double dst2 = DistanceToLineSegment(selected_points[2],
                                                  selected_points[3],

        local_version_point_clouds_[i][j]);
        if (dst1 < threshold || dst2 < threshold) {
          first_obs.push_back(j);
        }
        // second selection
        const double dst3 = DistanceToLineSegment(selected_points[4],
                                                  selected_points[5],
                                             local_version_point_clouds_[i][j]);
        const double dst4 = DistanceToLineSegment(selected_points[6],
                                                  selected_points[7],
                                             local_version_point_clouds_[i][j]);
        if (dst3 < threshold || dst4 < threshold) {
          second_obs.push_back(j);
        }
      }
    } // end obs
//cout << "OBS SIZES: " << first_obs.size() << ", " << second_obs.size() <<
//endl;
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

int VectorMapping::OrderAndFilterUserInput(const vector<Pose2Df>& poses,
                                           bool corners,
                                           vector<Vector2f>*
                                           selected_points_ptr) {
  vector<Vector2f> selected_points = *selected_points_ptr;
  int backprop_start = 0;
  if (corners) {
    CHECK_EQ(selected_points.size(), 8);
  } else {
    CHECK_EQ(selected_points.size(), 4);
  }

  pair<vector<pair<int, vector<int>>>, vector<pair<int, vector<int>>>>
      selected_poses;
  selected_poses = EstablishObservationSets(corners, selected_points_ptr);

  vector<int> first_selection_poses;
  vector<int> second_selection_poses;
  for (size_t i = 0; i < selected_poses.first.size(); ++i) {
    first_selection_poses.push_back(selected_poses.first[i].first);
  }
  for (size_t i = 0; i < selected_poses.second.size(); ++i) {
    second_selection_poses.push_back(selected_poses.second[i].first);
  }

  size_t second_start = selected_points.size()/2;
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
      /*
      float mean_first = 0.0;
      float mean_second = 0.0;
      for (size_t i = 0; i < first_affected_poses.size(); ++i) {
        mean_first += first_affected_poses[i];
      }
      mean_first = mean_first / float(first_affected_poses.size());
      for (size_t i = 0; i < second_affected_poses.size(); ++i) {
        mean_second += second_affected_poses[i];
      }
      mean_second = mean_second / float(second_affected_poses.size());
      */
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
                           selected_poses_new.second, poses);
    backprop_start = second_selection_max_pose;
  }
  else if (first_selection_max_pose < second_selection_min_pose) { //bad user
    vector<Vector2f> reordered_points;
    for (size_t i = second_start; i < selected_points.size(); ++i) {
      reordered_points.push_back(selected_points[i]);
    }
    for (size_t i = 0; i < second_start; ++i) {
      reordered_points.push_back(selected_points[i]);
    }
    CHECK_EQ(reordered_points.size(), selected_points.size());
    for (size_t i = 0; i < reordered_points.size(); ++i) {
      selected_points_ptr[0][i] = reordered_points[i];
    }

    SetCorrectionRelations(selected_poses_new.second,
                           selected_poses_new.first, poses);
    backprop_start = first_selection_max_pose;
  }
  else {
    cout << "ERROR: selection overlap." << endl;
    backprop_start = -1;
  }

  return backprop_start;
}

int VectorMapping::AddPointToPointHumanConstraint(
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
  for (size_t i=0; i<local_version_point_clouds_.size(); i++) {
    for (size_t j=0; j<local_version_point_clouds_[i].size(); j++) {
      for (size_t k=0; k<dst_to_POIs.size(); k++) {
        double dst = (selected_points[k] -
                      local_version_point_clouds_[i][j]).norm();
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
      local_version_point_clouds_[dst_to_POIs[0].second.first]
      [dst_to_POIs[0].second.second];
  const Vector2f& B =
      local_version_point_clouds_[dst_to_POIs[1].second.first]
      [dst_to_POIs[1].second.second];
  const Vector2f partial_correction = B - A;
  const Eigen::Vector3f C(partial_correction(0), partial_correction(1), 0.0);
  CorrectionPair new_correction = make_pair(dst_to_POIs[0].second.first, C);
  corrections->push_back(new_correction);
  return dst_to_POIs[1].second.first;
}

void VectorMapping::AddLineToLineHumanConstraint(
    vector<Vector2f> selected_points,
    const vector<Pose2Df>& poses,
    vector<CorrectionPair>* corrections) {
  CHECK_EQ(selected_points.size(), 4);
  //determine transformation between lines
  Vector2f cmA = (selected_points[1] + selected_points[0])/2.0;
  Vector2f cmB = (selected_points[3] + selected_points[2])/2.0;

  Vector2f A(selected_points[1] - selected_points[0]);
  A.normalize();
  Vector2f B(selected_points[3] - selected_points[2]);
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
    Vector2f p0(poses[pose_index].translation);
    Vector2f p1 = cmB + R*(p0 - cmA);
    Vector2f T = p1-p0;
    Eigen::Vector3f c(T(0), T(1), theta);
    CorrectionPair new_correction = make_pair(pose_index, c);
    corrections->push_back(new_correction);
  }
}

void VectorMapping::AddCornerToCornerHumanConstraint(
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

void VectorMapping::AddColinearHumanConstraint(
    vector<Vector2f> selected_points,
    const vector<Pose2Df>& poses,
    vector<CorrectionPair>* corrections) {
  CHECK_EQ(selected_points.size(), 4);
  //determine transformation between lines
  const Vector2f cmA = 0.5 * (selected_points[1] + selected_points[0]);
  const Vector2f cmB = 0.5 * (selected_points[3] + selected_points[2]);
  const Vector2f A = (selected_points[1] - selected_points[0]).normalized();
  const Vector2f B = (selected_points[3] - selected_points[2]).normalized();
  const float theta =
      (ScalarCross(A, B) >= 0.0) ? (acos(A.dot(B))) : (-acos(A.dot(B)));
  const Rotation2Df R(theta);

  // project A onto B
  const float alpha = (cmA - cmB).dot(B);
  // new_cmA is colinear with B's endpoints
  const Vector2f new_cmA = cmB + alpha * B;



  for (size_t i = 0; i < corrected_poses_.size(); ++i) {
    int pose_index = corrected_poses_[i];
    Vector2f p0(poses[pose_index].translation);
    Vector2f p1 = new_cmA + R*(p0 - cmA);
    Vector2f T = p1-p0;
    Eigen::Vector3f c(T(0), T(1), theta);
    CorrectionPair new_correction = make_pair(pose_index, c);
    corrections->push_back(new_correction);
  }
}

void VectorMapping::AddPerpendicularHumanConstraint(
                                         vector<Vector2f> selected_points,
                                         const vector<Pose2Df>& poses,
                                         vector<CorrectionPair>* corrections) {
  CHECK_EQ(selected_points.size(), 4);
  //determine transformation between lines
  Vector2f cmA = (selected_points[1] + selected_points[0])/2.0;
  Vector2f A(selected_points[1] - selected_points[0]);
  A.normalize();
  Vector2f B(selected_points[3] - selected_points[2]);
  B.normalize();

  Eigen::Vector3f A3(A(0), A(1), 0);
  Eigen::Vector3f B3(B(0), B(1), 0);

  double theta = 0.0;
  if (A3.cross(B3)(2) < 0.0) { //A cross B
    theta = -acos(A.dot(B));
  }
  else {
    theta = acos(A.dot(B));
  }

  if (theta == M_PI/2.0 || theta == - M_PI/2.0) {
    theta = 0.;
  }
  else if (theta > 0.0) {
    theta = -(-theta + M_PI/2.0);
  }
  else {
    theta = -(-theta - M_PI/2.0);
  }

  cout << "PERPENDICULAR TEST THETA: " << theta << endl;

  // Rotation matrix R(theta)
  const Rotation2Df R(theta);


  for (size_t i = 0; i < corrected_poses_.size(); ++i) {
    int pose_index = corrected_poses_[i];
    Vector2f p0(poses[pose_index].translation);
    Vector2f p1 = cmA + R*(p0 - cmA);
    Vector2f T = p1-p0;
    Eigen::Vector3f c(T(0), T(1), theta);
    CorrectionPair new_correction = make_pair(pose_index, c);
    corrections->push_back(new_correction);
  }
}

void VectorMapping::AddParallelHumanConstraint(
                                    vector<Vector2f> selected_points,
                                    const vector<Pose2Df>& poses,
                                    vector<CorrectionPair>* corrections) {
  CHECK_EQ(selected_points.size(), 4);
  //determine transformation between lines
  const Vector2f cmA = 0.5 * (selected_points[1] + selected_points[0]);
  const Vector2f A = (selected_points[1] - selected_points[0]).normalized();
  const Vector2f B = (selected_points[3] - selected_points[2]).normalized();
  const float theta =
      (ScalarCross(A, B) >= 0.0) ? (acos(A.dot(B))) : (-acos(A.dot(B)));
  const Rotation2Df R(theta);


  for (size_t i = 0; i < corrected_poses_.size(); ++i) {
    int pose_index = corrected_poses_[i];
    Vector2f p0(poses[pose_index].translation);
    Vector2f p1 = cmA + R*(p0 - cmA);
    Vector2f T = p1 - p0;
    Eigen::Vector3f c(T(0), T(1), theta);
    CorrectionPair new_correction = make_pair(pose_index, c);
    corrections->push_back(new_correction);
  }
}

// void VectorMapping::AddHumanCorrections(vector<Pose2Df>* poses,
//                                         const vector<Vector2f> selected_points,
//                                         const int mode,
//                                         vector<HumanConstraint>&
//                                         human_constraints) {
//   HumanConstraint new_human_constraint;
//   new_human_constraint.mode = mode;
//   new_human_constraint.obs_a = corrected_observations_;
//   new_human_constraint.obs_b = correcting_observations_;
//   new_human_constraint.pose_obs_ids_a = corrected_pose_obs_;
//   new_human_constraint.pose_obs_ids_b = correcting_pose_obs_;
//   new_human_constraint.x_a = corrected_poses_;
//   new_human_constraint.x_b = correcting_poses_;
//   new_human_constraint.len_a = double((selected_points[1] -
//                                        selected_points[0]).norm());
//   new_human_constraint.len_b = double((selected_points[3] -
//                                        selected_points[2]).norm());
//   //new_human_constraint.cm_a = (selected_points[0] + selected_points[1])/2.0;
//   new_human_constraint.cm_a = (selected_points[0] + selected_points[1])/2.0;
//   new_human_constraint.cm_b = (selected_points[2] + selected_points[3])/2.0;
//   Rotation2Df rot(M_PI / 2.0);
//   Vector2f a_dir = (selected_points[1] - selected_points[0]);
//   Vector2f b_dir = (selected_points[3] - selected_points[2]);
//   a_dir.normalize();
//   b_dir.normalize();
//   //new_human_constraint.n_a = rot * a_dir;
//   new_human_constraint.n_a = rot * b_dir;
//   new_human_constraint.n_b = rot * b_dir;
//   human_constraints.push_back(new_human_constraint);
//   if (hc_params_array_.size() > human_constraints.size() * 8) {
//     int sz = human_constraints.size();
//     //cout << "hc params size: " << hc_params_array_.size() << endl;
//     hc_params_array_[8 * sz] = double(new_human_constraint.cm_a(0));
//     hc_params_array_[8 * sz + 1] = double(new_human_constraint.cm_a(1));
//     hc_params_array_[8 * sz + 2] = double(new_human_constraint.n_a(0));
//     hc_params_array_[8 * sz + 3] = double(new_human_constraint.n_a(1));
//     hc_params_array_[8 * sz + 4] = double(new_human_constraint.cm_b(0));
//     hc_params_array_[8 * sz + 5] = double(new_human_constraint.cm_b(1));
//     hc_params_array_[8 * sz + 6] = double(new_human_constraint.n_b(0));
//     hc_params_array_[8 * sz + 7] = double(new_human_constraint.n_b(1));
//   }
//   else {
//     //cout << "hc params size: " << hc_params_array_.size() << endl;
//     hc_params_array_.push_back(double(new_human_constraint.cm_a(0)));
//     hc_params_array_.push_back(double(new_human_constraint.cm_a(1)));
//     hc_params_array_.push_back(double(new_human_constraint.n_a(0)));
//     hc_params_array_.push_back(double(new_human_constraint.n_a(1)));
//     hc_params_array_.push_back(double(new_human_constraint.cm_b(0)));
//     hc_params_array_.push_back(double(new_human_constraint.cm_b(1)));
//     hc_params_array_.push_back(double(new_human_constraint.n_b(0)));
//     hc_params_array_.push_back(double(new_human_constraint.n_b(1)));
//   }
// }

int VectorMapping::CalculateExplicitCorrections(
    vector<Vector2f> selected_points,
    const vector<Pose2Df>& poses,
    int mode,
    vector<CorrectionPair>* corrections) {

  int backprop_start = 0;
  if (mode != 1) {
    const bool corners = (mode == 3) ? true : false;
    backprop_start = OrderAndFilterUserInput(poses, corners, &selected_points);
  }
  if (backprop_start < 0) {
    return -1;
  }

  switch (mode) {
    case 1 : {
      CHECK_EQ(selected_points.size(), 2);
      backprop_start = AddPointToPointHumanConstraint(selected_points,
                                                      corrections);
    } break;
    case 2 : {
      CHECK_EQ(selected_points.size(), 4);
      AddLineToLineHumanConstraint(selected_points, poses, corrections);
    } break;
    case 3 : {
      CHECK_EQ(selected_points.size(), 8);
      AddCornerToCornerHumanConstraint(selected_points, poses, corrections);
    } break;
    case 4 : {
      CHECK_EQ(selected_points.size(), 4);
      AddColinearHumanConstraint(selected_points, poses, corrections);
    } break;
    case 5 : {
      CHECK_EQ(selected_points.size(), 4);
      AddPerpendicularHumanConstraint(selected_points, poses, corrections);
    } break;
    case 7 : {
      CHECK_EQ(selected_points.size(), 4);
      AddParallelHumanConstraint(selected_points, poses, corrections);
    } break;
    default : {
      fprintf(stderr,
              "Error: Unknown correction HiTL correction mode %d\n",
              mode);
      return -1;
    }
  }
  return backprop_start;
}

void VectorMapping::CopyTempLaserScans(const vector<Pose2Df>* poses) {
  local_version_point_clouds_.resize(point_clouds_e_.size());
  for (size_t i = 0; i < point_clouds_e_.size(); ++i) {
    const PointCloudf& point_cloud = point_clouds_e_[i];
    local_version_point_clouds_[i].resize(point_cloud.size());
  }

  // put laser scans in world frame... as they are displayed in the gui
  for (size_t i = 0; i < point_clouds_e_.size(); i++) {
    const Rotation2Df rotation((*poses)[i].angle);
    const Vector2f pose_location = (*poses)[i].translation;
    for (size_t j = 0; j < point_clouds_e_[i].size(); ++j) {
      local_version_point_clouds_[i][j] = (rotation * point_clouds_e_[i][j] +
          pose_location);
    }
  }
}

void VectorMapping::FindContiguousGroups(size_t min_poses, size_t max_poses,
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

void VectorMapping::BackPropagatePoseCorrections(vector<Eigen::Matrix3d>*
                                                 covariances_ptr,
                                                 size_t min_poses,
                                                 size_t max_poses,
                                                 Eigen::Vector3f C,
                                                 vector<Pose2Df>*
                                                 poses_ptr) {

  cout << "correction:\n" << C << endl;

  vector<Pose2Df>& poses = *poses_ptr;
  vector<Eigen::Matrix3d>& covars = *covariances_ptr;
  Vector2f destination = poses[max_poses-1].translation + Vector2f(C(0), C(1));
  //cout << "original\n" << C << endl;

  double destination_rot_variance = 0.0001; //radians
  double destination_trans_variance = 0.001; //meters
  vector<double> rot_sigmas;
  vector<double> trans_sigmas;
  for (size_t i = 0; i < covars.size(); ++i) {
    rot_sigmas.push_back(covars[i](2,2));
    trans_sigmas.push_back((covars[i](0,0) + covars[i](1,1))/2.0);
  //rot_sigmas.push_back(0.000001 / covars[i](2,2));
  //trans_sigmas.push_back(0.000001 /( (covars[i](0,0) + covars[i](1,1))/2.0));
  }

  vector<double> rot_weights;
  vector<double> trans_weights;
  double sum_of_rot_var = 0.0;
  double sum_of_trans_var = 0.0;
  //tabulate variances
  for (size_t i = min_poses + 1; i < max_poses; ++i) {
    sum_of_rot_var += rot_sigmas[i];
    sum_of_trans_var += trans_sigmas[i];
  }
  //Fuse destination variance with current final pose variance
  sum_of_rot_var += destination_rot_variance;
  sum_of_trans_var += destination_trans_variance;

  //calculate weights
  for (size_t i = min_poses + 1; i < max_poses; ++i) {
    rot_weights.push_back(rot_sigmas[i]/sum_of_rot_var);
    trans_weights.push_back(trans_sigmas[i]/sum_of_trans_var);
  }

  //initialize alphas
  CHECK_EQ(rot_weights.size(), trans_weights.size());

  // calculate beta for variance updates
  double rot_beta = 1/(1 + (rot_sigmas[max_poses - 1] /
                               destination_rot_variance));
  double trans_beta = 1/(1 + (trans_sigmas[max_poses - 1] /
                                 destination_trans_variance));

  //update fused rotation uncertainty
  rot_sigmas[max_poses - 1] = rot_sigmas[max_poses - 1] *
                              destination_rot_variance /
                              (rot_sigmas[max_poses - 1] +
                              destination_rot_variance);
  //update fused translation uncertainty
  trans_sigmas[max_poses - 1] = trans_sigmas[max_poses - 1] *
                              destination_trans_variance /
                              (trans_sigmas[max_poses - 1] +
                              destination_trans_variance);

  cout << "trans_beta: " << trans_beta << endl;
  cout << "rot_beta: " << rot_beta << endl;

  // update covariance estimates
  for (size_t i = min_poses + 1; i < max_poses - 1; ++i) {
    covars[i](0,0) *= trans_beta;
    covars[i](0,1) *= trans_beta;
    covars[i](1,0) *= trans_beta;
    covars[i](1,1) *= trans_beta;

    covars[i](0,2) *= rot_beta;
    covars[i](0,2) *= rot_beta;
    covars[i](2,0) *= rot_beta;
    covars[i](2,1) *= rot_beta;

    covars[i](2,2) *= rot_beta;
  }

  cout << "min pose: " << min_poses << endl;
  cout << "max pose: " << max_poses << endl;

  //APPLY ROTATION
  float theta = C(2);
  float delta_theta;
  //figure out new pose locations
  for (size_t i = min_poses + 1; i < max_poses; ++i) {
    delta_theta = rot_weights[i - min_poses - 1] * theta;
    const Affine2f post_i_correction = Translation2Df(poses[i].translation) *
                                       Rotation2Df(delta_theta) *
                                       Translation2Df(-poses[i].translation);
    poses[i].angle += delta_theta;
    for (auto k = i + 1; k < max_poses; ++k) {
      poses[k].angle += delta_theta;
      poses[k].translation = (post_i_correction * poses[k].translation);
    }
  }

  //APPLY TRANSLATION
  Vector2f trans = destination - poses[max_poses - 1].translation;

  //cout << "after theta\n" << trans << endl;

  Vector2f delta_trans;
  //figure out new pose locations
  for (size_t i = min_poses + 1; i < max_poses; ++i) {
    delta_trans = trans_weights[i - min_poses - 1] * trans;
    for (auto k = i + 1; k < max_poses; ++k) {
      poses[k].translation += delta_trans;
    }
  }
}

int VectorMapping::ApplyExplicitCorrections(size_t i,
                                            vector<vector<CorrectionPair>>*
                                            contiguous_corrections,
                                            vector<Pose2Df>* poses) {

  for (size_t j = 0; j < contiguous_corrections[0][i].size(); ++j) {
    //cout << "editing pose " << contiguous_corrections[0][i][j].first << endl;
    (*poses)[contiguous_corrections[0][i][j].first].translation.x() +=
        contiguous_corrections[0][i][j].second(0);
    (*poses)[contiguous_corrections[0][i][j].first].translation.y() +=
        contiguous_corrections[0][i][j].second(1);
    (*poses)[contiguous_corrections[0][i][j].first].angle +=
        contiguous_corrections[0][i][j].second(2);
  }
  //int last_pose =
  //  contiguous_corrections[0][i][contiguous_corrections[0][i].size() -
  //1].first;
  int last_pose = contiguous_corrections[0][0].back().first;
  //if (i == contiguous_corrections->size() - 1) { //last group
    cout << "last group! group number " << i << endl;
    cout << "last pose number " << last_pose << endl;
    Vector3f last_correction = contiguous_corrections[0][i].back().second;
    for (size_t k = last_pose + 1; k < poses->size(); ++k) {
      //calculate new position
      (*poses)[k].angle += last_correction(2);
      Vector2f ab = (*poses)[k].translation - (*poses)[last_pose].translation;
      Rotation2Df rot = Rotation2Df(last_correction(2));
      Vector2f new_ab = rot*ab;

      (*poses)[k].translation = (*poses)[last_pose].translation + new_ab +
                               Vector2f(last_correction(0), last_correction(1));
    }
  //}
  return last_pose;
}

// ceres::TerminationType VectorMapping::SolveHumanConstraints(
//                                           vector<Matrix3d>* D3_covariances,
//                                           vector<Pose2Df>* poses,
//                                           vector<HumanConstraint>*
//                                           human_constraints) {
// 
//   float odom_std_dev_weight = 1.0;
//   cout << "Begin solving problem" << endl;
//   ceres::Solver::Options solver_options;
//   solver_options.minimizer_progress_to_stdout = true;
//   //solver_options.min_relative_decrease = 0.00001;
//   //solver_options.max_num_iterations = 200; // works well on easy test
//   solver_options.max_num_iterations = 300; // p amazeballs
//   //solver_options.update_state_every_iteration = true;
//   //SetSolverOptions(localization_options_, &solver_options);
// 
// 
//   vector<pair<Vector2f, Vector2f>> updated_P_sets;
//   updated_P_sets = RefitConstraints(human_constraints);
// 
//   for (size_t i = 0; i < human_constraints->size(); ++i) {
//     human_constraints[0][i].cm_a = updated_P_sets[2*i].first;
//     human_constraints[0][i].n_a = updated_P_sets[2*i].second;
//     human_constraints[0][i].cm_b = updated_P_sets[2*i + 1].first;
//     human_constraints[0][i].n_b = updated_P_sets[2*i + 1].second;
//   }
// 
//   for (size_t i = 0; i < human_constraints->size(); ++i) {
//     cout << "CMA:\n" << human_constraints[0][i].cm_a << endl;
//     cout << "NA:\n" << human_constraints[0][i].n_a << endl;
//     cout << "CMB:\n" << human_constraints[0][i].cm_b << endl;
//     cout << "NB:\n" << human_constraints[0][i].n_b << endl;
//   }
// 
//   ceres::Problem problem;
//   cout << "Adding loop constraints" << endl;
//   AddLoopConstraint(*D3_covariances, *poses, odom_std_dev_weight, &problem);
//   problem.SetParameterBlockConstant(&pose_array_[0]);
// 
//   cout << "Adding human constraints" << endl;
//   AddHumanConstraints(*human_constraints, *poses, &problem);
// 
//   //CHECK_EQ(ceres_cost_.size(), poses->size());
//   //for (size_t i = 0; i < poses->size(); ++i) {
//   //  cout << i << " ::: " << ceres_cost_[i] << endl;
//   //}
// 
//   ceres::Solver::Summary summary;
//   cout << "iter\t  cost\t      cost_change\t  |gradient|\t  |step|  "
//        << "tr_ratio\t tr_radius\t   ls_iter    \titer_time" << endl;
//   ceres::Solve(solver_options, &problem, &summary);
//   //cout << summary.BriefReport() << endl;
//   cout << summary.FullReport() << endl;
//   cout << "Solve finished running... evaluating result" << endl;
// 
//   cout << "HC PARAMS ARRAY POST OPT" << endl;
//   for (size_t i = 0; i < human_constraints->size(); ++i) {
//   //for (size_t i = 0; i < 1; ++i) {
//     cout << "constraint number " << i << endl;
//     for (int j = 0; j < 8; ++j) {
//     //for (int j = 0; j < 4; ++j) {
//       //cout << test_array_[8*i + j] << endl;
//       cout << hc_params_array_[8*i + j] << endl;
//     }
//     human_constraints[0][i].cm_a(0) = float(hc_params_array_[8*i]);
//     human_constraints[0][i].cm_a(1) = float(hc_params_array_[8*i+1]);
//     human_constraints[0][i].n_a(0) = float(hc_params_array_[8*i+2]);
//     human_constraints[0][i].n_a(1) = float(hc_params_array_[8*i+3]);
//     human_constraints[0][i].cm_b(0) = float(hc_params_array_[8*i+4]);
//     human_constraints[0][i].cm_b(1) = float(hc_params_array_[8*i+5]);
//     human_constraints[0][i].n_b(0) = float(hc_params_array_[8*i+6]);
//     human_constraints[0][i].n_b(1) = float(hc_params_array_[8*i+7]);
//     human_constraints[0][i].n_a.normalize();
//     human_constraints[0][i].n_b.normalize();
//     //cout << "normed normals:\n" << human_constraints[0][i].n_a << "\n"
//     //     << human_constraints[0][i].n_b << endl;
//   }
// 
// 
// 
// 
// 
// 
// 
// 
// 
//   vector<pair<Vector2f, Vector2f> > lines;
//   vector<Vector2f> points;
//   for (size_t i = 0; i < human_constraints->size(); ++i) {
//     Vector2f line_dir = Rotation2Df(-M_PI/2.0) * human_constraints[0][i].n_a;
//     Vector2f ep1 = human_constraints[0][i].cm_a +
//                   (human_constraints[0][i].len_a/2.0) * line_dir;
//     Vector2f ep2 = human_constraints[0][i].cm_a -
//                   (human_constraints[0][i].len_a/2.0) * line_dir;
//     pair<Vector2f, Vector2f> new_line = make_pair(ep1, ep2);
//     lines.push_back(new_line);
// 
//     line_dir = Rotation2Df(-M_PI/2.0) * human_constraints[0][i].n_b;
//     ep1 = human_constraints[0][i].cm_b +
//          (human_constraints[0][i].len_b/2.0) * line_dir;
//     ep2 = human_constraints[0][i].cm_b -
//          (human_constraints[0][i].len_b/2.0) * line_dir;
//     new_line = make_pair(ep1, ep2);
//     lines.push_back(new_line);
//   }
// 
//   for (size_t k = 0; k < human_constraints->size(); k++) {
//     HumanConstraint H = human_constraints[0][k];
//     for (size_t i=0; i<H.pose_obs_ids_a.size(); i++) {
//       vector<pair<int, vector<int>>> po = H.pose_obs_ids_a;
//       for (size_t j=0; j<po[i].second.size(); ++j) {
//           temp_inliers_.push_back(
//             local_version_point_clouds_[po[i].first][po[i].second[j]]);
//       }
//     }
//     for (size_t i=0; i<H.pose_obs_ids_b.size(); i++) {
//       vector<pair<int, vector<int>>> po = H.pose_obs_ids_b;
//       for (size_t j=0; j<po[i].second.size(); ++j) {
//           temp_inliers_.push_back(
//             local_version_point_clouds_[po[i].first][po[i].second[j]]);
//       }
//     }
//   }
//   points = temp_inliers_;
//   cout << "points size POST: " << points.size() << endl;
//   temp_inliers_.clear();
//   //AnimateCeres(lines, points);
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
//   int num_residuals = problem.NumResiduals();
//   cout << "num residuals: " << num_residuals << endl;
// 
//   //double residuals[num_residuals];
//   //double residuals = NULL;
//   vector<double> res;
//   vector<double>* residuals = &res;
//   for (int i = 0; i < num_residuals; ++i) {
//     residuals->push_back(0);
//   }
//   ceres::CRSMatrix ceres_jacobian;
// 
// 
//   ceres::Problem::EvaluateOptions evalOptions =
//                             ceres::Problem::EvaluateOptions();
// 
//   problem.Evaluate(evalOptions, NULL, residuals, NULL, &ceres_jacobian);
// 
// 
//   //ceres_jacobian = problem.Evaluate();
// 
//   Eigen::MatrixXd dense_jacobian = ConvertJ(ceres_jacobian);
// 
//   //cout << dense_jacobian << endl;
//   cout << "rows: " << dense_jacobian.rows() << endl;
//   cout << "cols: " << dense_jacobian.cols() << endl;
// 
// 
//   cout << dense_jacobian.block<9,9>(0,0) << endl << endl;
//   //cout << dense_jacobian.block<8,8>(0,0) << endl << endl;
// 
//   //TODO: write observation consistency matrix
// 
// 
//   //cout << "here1" << endl;
//   Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Zero(poses->size(),
//                                                              poses->size());
//   //cout << "here2" << endl;
//   information_matrix(0,0) = 1;
//   for (size_t i = 1; i < poses->size(); ++i) {
//     //cout << "here: " << i << endl;
//     information_matrix(i, i) = 1;
//     information_matrix(i - 1, i) = 1;
//     //information_matrix(i, i - 1) = 1;
//   }
//   //cout << "here3" << endl;
// 
//   /*
//    * INFORMATION MATRIX
//   for (size_t i = 0; i < human_constraints->size(); ++i) {
//     int row = min(human_constraints[0][i].constrained_pose_id,
//                   human_constraints[0][i].linked_pose_id);
//     int col = max(human_constraints[0][i].constrained_pose_id,
//                   human_constraints[0][i].linked_pose_id);
//     information_matrix(row, col) = 1;
//   }
//   */
// 
//   //cout << "here4" << endl;
//   cout << "info cols: " << information_matrix.cols() << endl;
// 
// // takes too long when testing big map
// /*
//   std::ofstream info("information.txt");
//   if (info.is_open()) {
//     info << information_matrix << '\n';
//   }
// cout << "here info done" << endl;
//   std::ofstream con("consistency.txt");
//   if (con.is_open()) {
//     con << dense_jacobian << '\n';
//   }
// cout << "here con done" << endl;
//   std::ofstream cost("cost.txt");
//   if (cost.is_open()) {
//     for (size_t i = 0; i < poses->size(); ++i) {
//       cost << ceres_cost_[i] << '\n';
//     }
//   }
// 
// cout << "here cost done" << endl;
//   std::ofstream file("jacobian.txt");
//   if (file.is_open()) {
//     file << dense_jacobian << '\n';
//   }
// */
//   //const int cols = 781; //dense_jacobian.cols();
//   //const int rows = 648; //dense_jacobian.rows();
//   //cout << dense_jacobian.block<cols - rows, 10>(cols - 1,0) << endl << endl;
// 
//   cout << "rows: " << dense_jacobian.rows() << endl;
//   cout << "cols: " << dense_jacobian.cols() << endl;
// 
// 
// 
//   return summary.termination_type;
// }
// 
// ceres::TerminationType VectorMapping::PostHumanOptimization(
//                                           vector<Matrix3d>* D3_covariances,
//                                           vector<Pose2Df>* poses,
//                                           vector<HumanConstraint>*
//                                           human_constraints,
//                                           int min_pose,
//                                           int max_pose) {
//   FunctionTimer timer(__FUNCTION__);
//   float odom_std_dev_weight = 1.0;
// 
//   timer.Lap(__LINE__);
//   cout << "Performing post-HiTL optimization\n";
//   ceres::Solver::Options solver_options;
//   solver_options.min_relative_decrease = 0.00001;
//   //solver_options.minimizer_progress_to_stdout = true;
// 
//   //SetSolverOptions(localization_options_, &solver_options);
// 
//   // this time add STF constraints
//   ceres::Problem problem;
//   timer.Lap(__LINE__);
// //  cout << "Adding loop constraints" << endl;
// //  AddLoopConstraint(*D3_covariances, *poses, odom_std_dev_weight, &problem);
// //  problem.SetParameterBlockConstant(&pose_array_[0]);
// 
//   cout << "Finding correspondences" << endl;
//   timer.Lap(__LINE__);
//   //FindSTFCorrespondences(0, poses->size()-1);
//   cout << "min_pose" << min_pose << endl;
//   cout << "max_pose" << max_pose << endl;
//   FindSTFCorrespondences(min_pose, max_pose);
//   cout << "Add STF constraints" << endl;
//   timer.Lap(__LINE__);
//   AddSTFConstraints(&problem);
// 
//   //cout << "Adding pose constraints" << endl;
//   //AddPoseConstraints(0, poses->size()-1, *poses, &problem);
//   //problem.SetParameterBlockConstant(&pose_array_[0]);
// 
//   //AddHumanConstraints(*human_constraints, *poses, &problem);
// 
//   timer.Lap(__LINE__);
//   cout << "Solving post-HitL problem" << endl;
//   ceres::Solver::Summary summary;
//   cout << "iter\t  cost\t      cost_change\t  |gradient|\t  |step|  "
//        << "tr_ratio\t tr_radius\t   ls_iter    \titer_time" << endl;
//   ceres::Solve(solver_options, &problem, &summary);
//   cout << summary.BriefReport() << endl;
//   //cout << summary.FullReport() << endl;
// 
//   return summary.termination_type;
// }

// vector<VectorMapping::HumanConstraint> VectorMapping::BatchLocalizeHotStart(
//                              const VectorMappingOptions& localization_options,
//                              const string& map_name,
//                              const vector<PointCloudf>& point_clouds,
//                              const vector<NormalCloudf>& normal_clouds,
//                              const bool debug,
//                              vector<Pose2Df>* poses,
//                              vector<Vector2f> selected_points,
//                              const int mode,
//                              vector<Matrix3d> covars,
//                              vector<HumanConstraint> human_constraints) {
//   static const bool kDebug = false;
// 
//   //strictly for CorrespondenceCallback... not really needed.
//   vector<double> gradients;
//   vector<Matrix2f> covariances;
// 
//   // Human constraints stores pose constraints
// 
//   CHECK_EQ(poses->size(), point_clouds.size());
// 
//   localization_options_ = localization_options;
// 
//   point_clouds_e_ = point_clouds;
//   ConvertPointClouds(point_clouds_e_);
// 
//   if (normal_clouds_e_.size() == 0) { normal_clouds_e_ = normal_clouds; }
//   if (kdtrees_.size() == 0) { BuildKDTrees(point_clouds_e_, normal_clouds); }
// 
//   // make a copy of laser scans in world frame
//   CopyTempLaserScans(poses);
// 
//   cout << "Update covariance estimates" << endl;
//   vector<Matrix3d> D3_covariances;
// 
//   bool calc_con_first_time = false;
//   if (pose_array_.size() == 0) {
//     calc_con_first_time = true;
//   }
// 
//   pose_array_.resize(poses->size() * 3);
//   for (size_t i = 0; i < poses->size(); ++i) {
//     const int j = 3 * i;
//     pose_array_[j + 0] = (*poses)[i].translation.x();
//     pose_array_[j + 1] = (*poses)[i].translation.y();
//     pose_array_[j + 2] = (*poses)[i].angle;
//   }
// 
//   if (calc_con_first_time) {
//     cout << "calc init incon" << endl;
//     //CalculateConsistency();
//   }
// 
//   //TODO: frame check
//   UpdateCovariances(poses->size(), &covars, &D3_covariances);
// 
//   size_t min_poses = 0;
//   size_t max_poses = poses->size()-1;
//   //size_t temp_max_poses = 0;
// 
//   //store corrections resulting from user adjustments to relative observations
//   vector<CorrectionPair> corrections;
//   cout << "Calculate explicit corrections" << endl;
//   int backprop_end = 0;
//   int backprop_start = 0;
//   backprop_start = CalculateExplicitCorrections(selected_points, *poses,
//                                                 mode, &corrections);
//   //TODO handle overlapping input better
//   if (backprop_start < 0) { //overlapping user input is ambiguous
//     cout << "returning no new human constraints" << endl;
//     return human_constraints;
//   }
//   //for (size_t i = 0; i < corrections.size(); ++i) {
//   //  cout << "correction pose: " << corrections[i].first << endl;
//   //}
// 
//   //backprop_end = corrections[0].first;
// 
//   cout << "Finding contiguous groups" << endl;
//   vector<vector<CorrectionPair>> contiguous_corrections;
//   FindContiguousGroups(min_poses, max_poses, &corrections,
//                        &contiguous_corrections);
// 
//   //impose changes for each contiguous group.
//   for (size_t i = 0; i < contiguous_corrections.size(); i++) {
//     if (i == 0) { //TODO: handle multiple contiguous groups properly
//       for (size_t x = 0; x < contiguous_corrections.size(); x++) {
//         //cout << "group number: " << x << endl;
//         for (size_t y = 0; y < contiguous_corrections[x].size(); y++) {
//           //cout << "pose number: "<<contiguous_corrections[x][y].first<< endl;
//         }
//       }
// 
//       backprop_end = contiguous_corrections[i][0].first;
//       Vector3f C = contiguous_corrections[i][0].second;
//       cout << "backprop_start: " << backprop_start << endl;
//       cout << "backprop_end: " << backprop_end << endl;
//       //cant backprop without intermediate poses
//       if (backprop_start + 1 < backprop_end) {
//         // apply implicit pose adjustments via back propagation of corrections
//         cout << "Applying back propagation corrections" << endl;
//         BackPropagatePoseCorrections(&D3_covariances, backprop_start,
//                                      backprop_end, C, poses);
//         D3_covariances_ = D3_covariances;
//       }
//       //apply explicit pose corrections within one contiguous group
//       cout << "Applying explicit constraints" << endl;
//       int mg_temp;
//       mg_temp = ApplyExplicitCorrections(i, &contiguous_corrections, poses);
//     }
//   }
//   cout << "PRE hc size: " << human_constraints.size() << endl;
// 
//   //const Vector2f const_line = selected_points[3] - selected_points[2];
//   //cout << "constraint line direction: " << const_line << endl;
//   AddHumanCorrections(poses, selected_points, mode, human_constraints);
// 
//   cout << "POST hc size: " << human_constraints.size() << endl;
// 
//   // make a copy of laser scans in world frame
//   CopyTempLaserScans(poses);
// 
//   //problem setup complete here, begin solving below.
//   //copy poses to array for ceres solver
//   pose_array_.resize(poses->size() * 3);
//   for (size_t i = 0; i < poses->size(); ++i) {
//     const int j = 3 * i;
//     pose_array_[j + 0] = (*poses)[i].translation.x();
//     pose_array_[j + 1] = (*poses)[i].translation.y();
//     pose_array_[j + 2] = (*poses)[i].angle;
//   }
// 
//   //visualize updated poses
//   TryCorrCallback(gradients, covariances, poses, min_poses, max_poses);
// 
//   cout << "sleepiong to view results" << endl;
// 
//   Sleep(1);
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
//   cout << "BEGIN SOLVERS" << endl;
// 
// 
//   ceres::TerminationType term_type;
//   term_type = ceres::CONVERGENCE;
// 
//   term_type  = SolveHumanConstraints(&D3_covariances, poses,
//                                      &human_constraints);
// 
// 
//   if (term_type == ceres::FAILURE || term_type == ceres::USER_FAILURE) {
//     cout << "\nCeres failure\n" << endl;
//     exit(1);
//   }
// 
//   else if (term_type == ceres::CONVERGENCE ||
//            term_type == ceres::USER_SUCCESS ||
//            term_type == ceres::NO_CONVERGENCE) {
// 
// 
//     cout << "\nCeres DID IT! Save this incremental progress now!\n" << endl;
// 
//     // Copy over the optimized poses.
//     for (size_t i = 0; i < poses->size(); ++i) {
//       const int j = 3 * i;
//       (*poses)[i].translation.x() = pose_array_[j + 0];
//       (*poses)[i].translation.y() = pose_array_[j + 1];
//       (*poses)[i].angle = angle_mod(pose_array_[j + 2]);
//     }
// 
//     //visualize updated poses
//     TryCorrCallback(gradients, covariances, poses, min_poses, max_poses);
// 
//     //CalculateConsistency();
// 
// 
// 
// 
// 
// 
// 
// /*
// 
//     //POST HUMAN OPTIMIZATION
// 
//     vector<Pose2Df> init_poses;
//     for (size_t i = 0; i < poses->size(); ++i) {
//       init_poses.push_back((*poses)[i]);
//     }
// 
//     //TODO try SGD / CGD / EpisodicGD
//     int ep_len = 10;
//     for (int i = backprop_start; i < backprop_end + ep_len; i+= ep_len) {
// 
//       term_type = PostHumanOptimization(&D3_covariances,
//                                         poses, &human_constraints,
//                                         i,
//                                         i + ep_len);
// 
//       if (term_type == ceres::CONVERGENCE ||
//           term_type == ceres::USER_SUCCESS ||
//           term_type == ceres::NO_CONVERGENCE) {
// 
//         // Copy over the optimized poses.
//         for (size_t i = 0; i < poses->size(); ++i) {
//           const int j = 3 * i;
//           (*poses)[i].translation.x() = pose_array_[j + 0];
//           (*poses)[i].translation.y() = pose_array_[j + 1];
//           (*poses)[i].angle = angle_mod(pose_array_[j + 2]);
//         }
//         cout << "Done performing post-HiTL optimization\n";
//       }
// 
//     }
// 
//     for (int i = backprop_start; i < backprop_end; ++i) {
//       const int j = 3 * i;
//       cout << "pose number: " << i << ": "
//            << fabs( init_poses[i].translation.x() - pose_array_[j + 0] ) << ", "
//            << fabs( init_poses[i].translation.y() - pose_array_[j + 1] ) << ", "
//            << fabs( init_poses[i].angle - angle_mod(pose_array_[j + 2]) )
//            << endl;
//     }
// 
// 
//     // vizualize post HitL optimization results
//     TryCorrCallback(gradients, covariances, poses, min_poses, max_poses);
// 
// 
// */
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
//     return human_constraints;
//   }
//   cout << "weird stuff with ceres... neither failure nor convergence" << endl;
//   return human_constraints;
// }





















bool VectorMapping::BatchLocalize(
    const VectorMappingOptions& localization_options,
    const string& map_name,
    const vector<PointCloudf>& point_clouds,
    const vector<NormalCloudf>& normal_clouds,
    const bool debug, const bool return_initial_poses,
    vector<Pose2Df>* poses) {
  CHECK_EQ(poses->size(), point_clouds.size());
  vector_map_.loadMap(map_name.c_str(), true);
  localization_options_ = localization_options;

  localization_options_.kMaxHistory = 10;

  point_clouds_e_ = point_clouds;
  // Make a copy of the point clouds in vector2f format for ray casting
  // with VectorMap.
  ConvertPointClouds(point_clouds_e_);
  normal_clouds_e_ = normal_clouds;

  // Build KD Trees for every pose.
  BuildKDTrees(point_clouds_e_, normal_clouds);

  // Make a mutable copy of the poses for opimization.
  pose_array_.resize(poses->size() * 3);
  for (size_t i = 0; i < poses->size(); ++i) {
    const int j = 3 * i;
    pose_array_[j + 0] = (*poses)[i].translation.x();
    pose_array_[j + 1] = (*poses)[i].translation.y();
    pose_array_[j + 2] = (*poses)[i].angle;
  }
  if (return_initial_poses) {
    printf("WARNING: Returning initial pose estimates!\n");
  }
  ceres::Solver::Options solver_options;
  SetSolverOptions(localization_options_, &solver_options);
  static const int kMaxIterations = 40000;
  const size_t kPoseIncrement = localization_options_.kPoseIncrement;
  int succesful_iterations = 0;
  int num_iterations = 0;

  vector<bool> pose_calculated(poses->size(), false);
  vector<Pose2Df> initial_poses(poses->begin(), poses->end());

  size_t max_poses = min(kPoseIncrement, point_clouds_g_.size() - 1);
  size_t min_poses = max(0, static_cast<int>(max_poses) -
      localization_options_.kMaxHistory);

  D3_covariances_.clear();
  Matrix3d one_cov = Matrix3d::Zero();
  D3_covariances_.push_back(one_cov);

  for (int k = 0; !terminate_ && k < kMaxIterations; ++k) {
    if (debug) {
      printf("Batch localization iteration %5d, poses %5lu:%5lu\n",
             k, min_poses, max_poses);
      printf("Finding correspondences\n");
    } else {
      printf("\rBatch localization iteration %d, poses %lu:%lu of %lu",
             k, min_poses, max_poses, point_clouds_g_.size());
      fflush(stdout);
    }

    FindSTFCorrespondences(min_poses, max_poses);
    // FindVisualOdometryCorrespondences(min_poses, max_poses);
    ceres::Problem problem;
    AddSTFConstraints(&problem);
    AddPoseConstraints(min_poses, max_poses, *poses, &problem);
    ceres::Solver::Summary summary;
    // The first pose should be constant since it's a "given" for the current
    // non-Markov episode.
    if (false && min_poses > 0) {
      problem.SetParameterBlockConstant(&(pose_array_[3*min_poses]));
    }

    if (min_poses == 0) {
      problem.SetParameterBlockConstant(&(pose_array_[0]));
    }

    ceres::Solve(solver_options, &problem, &summary);
    ++num_iterations;
    vector<double> gradients;
    vector<Matrix2f> covariances;
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
    if (summary.termination_type == ceres::FAILURE ||
        summary.termination_type == ceres::USER_FAILURE) {
      cout << "\nCeres failure\n" << summary.FullReport();
    }
    // update covariance estimates
    ceres::Covariance::Options covariance_options;
    //covariance_options.algorithm_type = ceres::SPARSE_QR;
    covariance_options.algorithm_type = ceres::DENSE_SVD;
    ceres::Covariance covariance(covariance_options);
    vector<pair<const double*, const double*>> covariance_blocks;
    Matrix2f rot;
    Matrix3d transform = Matrix3d::Zero();
    for (auto i = min_poses + 1; i <= max_poses; ++i) {
      covariance_blocks.push_back(make_pair(&pose_array_[3*i],
                                            &pose_array_[3*i]));
    }
    covariance.Compute(covariance_blocks, &problem);
    double covariance_ii[3 * 3];
    for (auto i = min_poses + 1; i <= max_poses; ++i) {
      covariance.GetCovarianceBlock(&pose_array_[3*i],
                                    &pose_array_[3*i],
                                    covariance_ii);
      for (int j=0; j<9; j+=3) {
        one_cov(j/3,0) = covariance_ii[j];
        one_cov(j/3,1) = covariance_ii[j+1];
        one_cov(j/3,2) = covariance_ii[j+2];
      }


      rot = Rotation2Df(-(*poses)[i].angle);
      transform.topLeftCorner(2,2) = rot.cast<double>();
      transform(2,2) = 1;

      //cout << "transform:\n" << transform << endl;

      one_cov = transform*one_cov*transform.transpose();

      //cout << "transformed cov:\n" << one_cov << endl;

      if (D3_covariances_.size() <= i) {
        D3_covariances_.push_back(one_cov);
      }
      else {
        D3_covariances_[i] = one_cov;
      }
    }

    if (summary.num_successful_steps < 1 &&
        summary.termination_type == ceres::CONVERGENCE) {
      // The initial state of the solver was already within convergence
      // tolerance.
      ++succesful_iterations;
      if (succesful_iterations > localization_options_.kNumRepeatIterations) {
        succesful_iterations = 0;
        num_iterations = 0;
        ResetGlobalPoses(max_poses,
                         min(max_poses + kPoseIncrement + kPoseIncrement / 2,
                            poses->size() - 1),
                         *poses);
        // if (max_poses + kPoseIncrement > point_clouds_.size() - 1) break;
        if (return_initial_poses) {
          for (size_t i = min_poses; i <= max_poses; ++i) {
            const int j = 3 * i;
            if (!pose_calculated[i]) {
              initial_poses[i].translation.x() = pose_array_[j + 0];
              initial_poses[i].translation.y() = pose_array_[j + 1];
              initial_poses[i].angle = angle_mod(pose_array_[j + 2]);
              pose_calculated[i] = true;
            }
          }
        }
        if (max_poses == point_clouds_g_.size() - 1) break;
        max_poses = min(
            static_cast<unsigned int>(max_poses + kPoseIncrement),
            static_cast<unsigned int>(point_clouds_g_.size() - 1));
        min_poses = max(0, static_cast<int>(max_poses) -
            localization_options_.kMaxHistory);
      }
    }
    if (num_iterations > localization_options_.kMaxRepeatIterations) {
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
      succesful_iterations = 0;
      num_iterations = 0;
      if (return_initial_poses) {
        for (size_t i = min_poses; i <= max_poses; ++i) {
          const int j = 3 * i;
          if (!pose_calculated[i]) {
            initial_poses[i].translation.x() = pose_array_[j + 0];
            initial_poses[i].translation.y() = pose_array_[j + 1];
            initial_poses[i].angle = angle_mod(pose_array_[j + 2]);
            pose_calculated[i] = true;
          }
        }
      }
      ResetGlobalPoses(max_poses,
                       min(max_poses + kPoseIncrement + kPoseIncrement / 2,
                       poses->size() - 1),
                       *poses);
      // if (max_poses + kPoseIncrement > point_clouds_.size() - 1) break;
      if (max_poses == point_clouds_g_.size() - 1) break;
      max_poses = min(
        static_cast<unsigned int>(max_poses + kPoseIncrement),
                      static_cast<unsigned int>(point_clouds_g_.size() - 1));
      min_poses = max(0, static_cast<int>(max_poses) -
          localization_options_.kMaxHistory);
    }

    // cout << summary.FullReport();
    if (debug) {
      printf("\n%s\n", summary.BriefReport().c_str());
    }
  } // end outer for loop

  if (debug) {
    printf("Optimized %d poses.\n", static_cast<int>(point_clouds_g_.size()));
  } else {
    // To clear the cursor of the line used to display progress.
    printf("\n");
  }
  // Copy over the optimized poses.
  if (return_initial_poses) {
    *poses = initial_poses;
  }
  else {
    for (size_t i = 0; i < poses->size(); ++i) {
      const int j = 3 * i;
      (*poses)[i].translation.x() = pose_array_[j + 0];
      (*poses)[i].translation.y() = pose_array_[j + 1];
      (*poses)[i].angle = angle_mod(pose_array_[j + 2]);
    }
  }
  pose_array_.clear();
  return true;
}

void VectorMapping::SensorUpdate(
    const PointCloudf& point_cloud, const NormalCloudf& normal_cloud) {
  static const bool debug = false;
  const double t_now = GetTimeSec();
  const bool has_moved =
      (pending_rotation_ > 0.0 && pending_translation_ > 0.0);
  const bool force_update = has_moved &&
      (t_now > t_last_update_ + localization_options_.max_update_period);
  if (pending_rotation_ > localization_options_.minimum_node_rotation ||
      pending_translation_ > localization_options_.minimum_node_translation ||
      force_update) {
    // Add to Pending nodes.
    AddPose(point_cloud, normal_cloud, pending_relative_pose_);
    t_last_update_ = t_now;
  } else if (debug) {
    printf("Ignoring sensor data, trans:%f rot:%f\n",
           pending_translation_, DEG(pending_rotation_));
  }
}

void VectorMapping::OdometryUpdate(
    const float dx, const float dy, const float d_theta) {
  const Vector2f delta(dx, dy);
  pending_relative_pose_.angle =
      angle_mod(pending_relative_pose_.angle + d_theta);
  const Rotation2Df rotation(pending_relative_pose_.angle);
  pending_relative_pose_.translation += (rotation * delta);
  pending_translation_ += delta.norm();
  pending_rotation_ += fabs(d_theta);
}

void VectorMapping::ClearPoses() {
  pose_array_.clear();
  poses_.clear();
  point_clouds_e_.clear();
  point_clouds_g_.clear();
  normal_clouds_e_.clear();
  kdtrees_.clear();
  latest_mle_pose_.Set(Pose2Df(0.0, Vector2f(0.0, 0.0)));
  latest_pending_pose_.Set(Pose2Df(0.0, Vector2f(0.0, 0.0)));
  pending_relative_pose_.Clear();
  pending_rotation_ = 0.0;
  pending_translation_ = 0.0;
  pending_normal_clouds_e_.clear();
  pending_point_clouds_e_.clear();
  pending_relative_poses_.clear();
  latest_pending_pose_.Set(Pose2Df(0.0, Vector2f(0.0, 0.0)));
  latest_mle_pose_.Set(Pose2Df(0.0, Vector2f(0.0, 0.0)));
}

void VectorMapping::AddPendingPoseNodes()
{
  const size_t num_old_point_clouds = point_clouds_e_.size();

  // Copy over pending point and normal clouds.
  point_clouds_e_.insert(point_clouds_e_.end(),
                         pending_point_clouds_e_.begin(),
                         pending_point_clouds_e_.end());
  normal_clouds_e_.insert(normal_clouds_e_.end(),
                          pending_normal_clouds_e_.begin(),
                          pending_normal_clouds_e_.end());

  // Build KD trees for new point clouds.
  CHECK_GT(point_clouds_e_.size(), 0);
  CHECK_EQ(point_clouds_e_.size(), normal_clouds_e_.size());
  kdtrees_.resize(point_clouds_e_.size(), NULL);
  const size_t num_new_point_clouds = pending_point_clouds_e_.size();
  OMP_PARALLEL_FOR
  for (size_t i = 0; i < num_new_point_clouds; ++i) {
    const PointCloudf& point_cloud = pending_point_clouds_e_[i];
    const NormalCloudf& normal_cloud = pending_normal_clouds_e_[i];
    vector<KDNodeValue<float, 2>> values(point_cloud.size());
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      values[j].index = j;
      values[j].point = point_cloud[j];
      values[j].normal = normal_cloud[j];
    }
    kdtrees_[num_old_point_clouds + i] = new KDTree<float, 2>(values);
  }

  // Transform and copy over poses.
  Pose2Df latest_pose = latest_mle_pose_.GetLock();
  for (size_t i = 0; i < pending_relative_poses_.size(); ++i) {
    latest_pose.ApplyPose(pending_relative_poses_[i]);
    poses_.push_back(latest_pose);
  }
  latest_mle_pose_.SetUnlock(latest_pose);

  // Add converted point clouds and copy of pose array.
  pose_array_.resize(point_clouds_e_.size() * 3);
  point_clouds_g_.resize(point_clouds_e_.size());
  OMP_PARALLEL_FOR
  for (size_t i = 0; i < num_new_point_clouds; ++i) {
    ConvertPointCloud(pending_point_clouds_e_[i],
                      &point_clouds_g_[num_old_point_clouds + i]);
    const size_t pose_array_offset = 3 * (num_old_point_clouds + i);
    const Pose2Df& pose = poses_[num_old_point_clouds + i];
    pose_array_[pose_array_offset + 0] = pose.translation.x();
    pose_array_[pose_array_offset + 1] = pose.translation.y();
    pose_array_[pose_array_offset + 2] = pose.angle;
  }

  CHECK_EQ(point_clouds_e_.size(), point_clouds_g_.size());
  pending_point_clouds_e_.clear();
  pending_normal_clouds_e_.clear();
  pending_relative_poses_.clear();
}

void VectorMapping::AddPose(
    const PointCloudf& point_cloud,
    const NormalCloudf& normal_cloud,
    const Pose2Df& relative_pose) {
  // Add point_cloud, normal_cloud, relative_pose to pending buffer.
  // Reset distance traversed since last node.
  // If (number of pending nodes > threshold) and (update is not in progress) :
  //   Take every pending buffer entry and add to current list of poses.
  //   Spawn a new update.
  CHECK_EQ(point_cloud.size(), normal_cloud.size());
  CHECK_GT(point_cloud.size(), 0);
  pending_point_clouds_e_.push_back(point_cloud);
  pending_normal_clouds_e_.push_back(normal_cloud);
  pending_relative_poses_.push_back(relative_pose);
  CHECK_GT(pending_point_clouds_e_.size(), 0);

  Pose2Df latest_pending_pose = latest_pending_pose_.GetLock();
  latest_pending_pose.ApplyPose(relative_pose);
  latest_pending_pose_.SetUnlock(latest_pending_pose);

  pending_relative_pose_.Clear();
  pending_translation_ = 0.0;
  pending_rotation_ = 0.0;

  if (static_cast<int>(pending_relative_poses_.size()) >=
      localization_options_.kPoseIncrement) {
    ScopedTryLock lock(update_mutex_);
    if (lock.Locked()) {
      // Copy over pending nodes to current nodes.
      AddPendingPoseNodes();
      // Reset cumulative pending pose.
      latest_pending_pose_.Set(Pose2Df(0.0, 0.0, 0.0));
      // Start a new background thread for the update.
      CHECK(sem_post(&update_semaphore_) == 0);
    }
  }
}

Pose2Df VectorMapping::GetLatestPose() const {
  // WARNING: This code fragment could potentially suffer from concurrency
  // issues arising from the fact that the latest_pose_ and latest_pending_pose_
  // reads and the compund pose computation are not performed in an atomic
  // manner.
  Pose2Df latest_pose = latest_mle_pose_.Get();
  Pose2Df latest_pending_pose = latest_pending_pose_.Get();
  latest_pose.ApplyPose(latest_pending_pose);
  latest_pose.ApplyPose(pending_relative_pose_);
  return latest_pose;
}

bool VectorMapping::GetNodeData(
    vector<Pose2Df>* poses,
    vector<PointCloudf>* point_clouds,
    vector<Pose2Df>* pending_poses,
    Pose2Df* latest_pose) const {
  ScopedTryLock lock(update_mutex_);
  if (lock.Locked()) {
    *poses = poses_;
    *point_clouds = point_clouds_e_;
    *pending_poses = pending_relative_poses_;
    *latest_pose = latest_mle_pose_.Get();
    const Pose2Df latest_pending_pose = latest_pending_pose_.Get();
    latest_pose->ApplyPose(latest_pending_pose);
    latest_pose->ApplyPose(pending_relative_pose_);
    return true;
  }
  return false;
}

void VectorMapping::SetOptions(const VectorMappingOptions& options) {
  localization_options_ = options;
}

Pose2Df VectorMapping::GetLastMLEPose() const {
  return (latest_mle_pose_.Get());
}

void VectorMapping::Initialize(const Pose2Df& pose,
                                       const string& map_name) {
  ScopedLock lock(update_mutex_);
  ClearPoses();
  latest_mle_pose_.Set(pose);
  if (map_name != vector_map_.mapName) {
    vector_map_.loadMap(map_name.c_str(), true);
  }

  // EnmlMaps::LoadPersistentObjects()
}

string VectorMapping::GetCurrentMapName() const {
  return (vector_map_.mapName);
}

bool VectorMapping::RunningSolver() const {
  ScopedTryLock lock(update_mutex_);
  return (!lock.Locked());
}

vector<Pose2Df> VectorMapping::GetLoggedPoses() const {
  return (logged_poses_.Get());
}

std::vector<int> VectorMapping::GetLoggedEpisodeLengths() const {
  std::vector<int> episode_lengths;
  ScopedLock lock(update_mutex_);
  episode_lengths = logged_episode_lengths_;
  return (episode_lengths);
}

void VectorMapping::RecomputeAbsolutePoses() {
  pose_array_.resize(relative_pose_array_.size());
  // The pose array size must be a multiple of 3, since each pose has 3-DOFs.
  CHECK_EQ((pose_array_.size() % 3), 0);
  // The first 3-DOF pose value is absolute.
  pose_array_[0] = relative_pose_array_[0];
  pose_array_[1] = relative_pose_array_[1];
  pose_array_[2] = relative_pose_array_[2];
  for (size_t i = 3; i < pose_array_.size(); ++i) {
    pose_array_[i] = pose_array_[i - 3] + relative_pose_array_[i];
  }
}

void VectorMapping::RecomputeRelativePoses() {
  relative_pose_array_.resize(pose_array_.size());
  // The pose array size must be a multiple of 3, since each pose has 3-DOFs.
  CHECK_EQ((pose_array_.size() % 3), 0);
  // The first 3-DOF pose value is absolute.
  relative_pose_array_[0] = pose_array_[0];
  relative_pose_array_[1] = pose_array_[1];
  relative_pose_array_[2] = pose_array_[2];
  for (size_t i = 3; i < pose_array_.size(); ++i) {
    relative_pose_array_[i] = pose_array_[i] - pose_array_[i - 3];
  }
}

}  // namespace vector_localization
