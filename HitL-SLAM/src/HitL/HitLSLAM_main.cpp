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
// Main entry point for non-Markov localization.

#include <stdint.h>
#include <stdio.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pthread.h>
#include <queue>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <termios.h>
#include <time.h>
#include <utility>
#include <vector>

#include "eigen_helper.h"
#include "helpers.h"
#include "../vmapping/vector_mapping.h"
#include "../../extern_libraries/CImg/CImg.h"
#include "glog/logging.h"
#include "ceres/ceres.h"
#include "cobot_msgs/CobotEventsMsg.h"
#include "cobot_msgs/CobotLocalizationMsg.h"
#include "cobot_msgs/CobotOdometryMsg.h"
#include "cobot_msgs/GuiKeyboardEvent.h"
#include "cobot_msgs/GuiMouseMoveEvent.h"
#include "cobot_msgs/GuiMouseClickEvent.h"
#include "cobot_msgs/LidarDisplayMsg.h"
#include "cobot_msgs/LocalizationGuiCaptureSrv.h"
#include "cobot_msgs/LocalizationMsg.h"
#include "cobot_msgs/CobotLocalizationSrv.h"
//#include "cross_entropy_minimizer/cross_entropy_minimizer.h"
#include "../gui/gui_publisher_helper.h"
#include "nav_msgs/Odometry.h"
//#include "object_map.h"
#include "../perception_tools/perception_2d.h"
#include "popt_pp.h"
#include "proghelp.h"
#include "pthread_utils.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "../shared/math/geometry.h"
#include "../shared/util/configreader.h"
#include "sensor_msgs/LaserScan.h"
#include "timer.h"
#include "util.h"
#include "../map/vector_map.h"
//#include "vector_localization/relocalization.h"
//#include "vector_localization/residual_functors.h"
#include "residual_functors.h"

#include "EMinput.h"
#include "ApplyExplicitCorrection.h"
#include "Backprop.h"
#include "JointOptimization.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using cobot_gui::ClearDrawingMessage;
using cobot_gui::DrawCircle;
using cobot_gui::DrawLine;
using cobot_gui::DrawPoint;
using cobot_gui::DrawText;
using cobot_msgs::CobotEventsMsg;
using cobot_msgs::CobotLocalizationSrv;
using Eigen::Affine2d;
using Eigen::Affine2f;
using Eigen::Matrix2d;
using Eigen::Matrix2f;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Perp2;
using Eigen::Rotation2Dd;
using Eigen::Rotation2Df;
using Eigen::ScalarCross;
using Eigen::Translation2d;
using Eigen::Translation2f;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector3d;
using Eigen::DistanceToLineSegment;
//using EnmlMaps::PersistentObject;
using perception_2d::GenerateNormals;
using perception_2d::NormalCloudf;
using perception_2d::PointCloudf;
using perception_2d::Pose2Df;
using ros::ServiceServer;
using ros::Subscriber;
using std::cout;
using std::endl;
using std::pair;
using std::queue;
using std::size_t;
using std::sort;
using std::string;
using std::vector;
using vector_localization::CorrectionType;
using vector_localization::CorrectionTypeNames;
using vector_localization::VectorMapping;

using vector_localization::EMInput;
using vector_localization::AppExpCorrect;
using vector_localization::Backprop;
using vector_localization::JointOpt;

typedef KDNodeValue<float, 2> KDNodeValue2f;

namespace {


class DestructorTrace {
 public:
  DestructorTrace(const std::string& file_name, const int line_number) :
      kFileName(file_name), kLineNumber(line_number) {}
  ~DestructorTrace() {
    printf("Global Destructors called @ %s:%d\n",
           kFileName.c_str(), kLineNumber);
  }
 private:
  const std::string kFileName;
  const int kLineNumber;
};

}  // namespace

// recording data
string bag_name;
struct SingleInput {
  CorrectionType type_of_constraint;
  int undone;
  vector<Vector2f> input_points;
};

struct timespec ms1;
struct timespec ms2;

double total_runtime = 0.0;
int num_completed_cycles = 0;
int num_total_constraints = 0;
vector<SingleInput> input_history;
vector<SingleInput> logged_input;
int current_replay_index = 0;

vector<float> prev_ceres_cost;

// gradients from ceres solver
std::vector<double> ceres_gradients_;

// sparse jacobian representation from ceres
ceres::CRSMatrix ceres_jacobian_;
bool jacobian_init_ = false;

// number of residuals due to human constraints
int num_hc_residuals_;

// For visualization
float max_theta_var = 0.0;

vector<double> message_timestamps_;

// pose estimates
vector<Pose2Df> poses;

// original relative odometry
//vector<perception_2d::Pose2Df> orig_odom_;

// in case human wants to undo last constraint
vector<Pose2Df> prev_poses;
int added_human_constraints = 0;

// point clouds in robot frame
vector<PointCloudf> init_point_clouds;
vector<NormalCloudf> normal_clouds;

//covariances
vector<Matrix3d> covariances_;
vector<Matrix3d> prev_covariances_;

// human constraints
vector<vector<VectorMapping::HumanConstraint>> human_constraints_;

// points selected by user during HitL
vector<Vector2f> selected_points_;
CorrectionType correction_type_ = CorrectionType::kUnknownCorrection;
CorrectionType pending_correction_type_ = CorrectionType::kUnknownCorrection;
bool loop_corrections_on_ = false;

// Name of the topic on Cobot's software stack that laser data is published on.
static const string kCobotLaserTopic("/Cobot/Laser");
// Name of the topic on Cobot's software stack that Kinect scan data is
// published on.
static const string kKinectScanTopic("/Cobot/Kinect/Scan");
// Name of the topic in a standardized data bag files that Kinect scan data is
// published on.
static const string kStandardKinectScanTopic("kinect_scan");
// Name of the topic on Cobot's software stack that odometry is published on.
static const string kCobotOdometryTopic("/Cobot/Odometry");
// Name of the topic in a standardized data bag files that laser data is
// published on.
static const string kStandardLaserTopic("laser");
// Name of the topic in a standardized data bag files that odometry is
// published on.
static const string kStandardOdometryTopic("odom");
// Name of the topic in a standardized data bag files that location reset
// commands are published on.
static const string kStandardSetLocationTopic("set_location");
// Name of the map to localize the robot on.
const string kMapName("EmptyMap");
// Robot's starting location.
Vector2f kStartingLocation = Vector2f(0.0, 0.0);
// Robot's starting angle.
float kStartingAngle = 0.0;
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
// Minimum distance of observed points from the robot.
float kMinPointCloudRange = 0.2;
// Maximum distance of observed points from the robot.
float kMaxPointCloudRange = 6.0;
// Maximum distance between adjacent points to use for computation of normals.
float kMaxNormalPointDistance = 0.03;
// Indicates whether the bag file being read is a standardized data bag file.
bool kStandardizedData = false;
// Angular margin from the scan area boundary to ignore laser readings.
float kAngularMargin = 0.0;

char* bag_file = NULL;

char* stfs_file = NULL;

char* log_file = NULL;

bool lost_poses = true;

int correction_number = 0;

// Mutex to ensure only a single relocalization call is made at a time.
pthread_mutex_t relocalization_mutex_ = PTHREAD_MUTEX_INITIALIZER;

// The path of the cobot_linux ROS stack.
static const string kCobotStackPath(ros::package::getPath("cobot_linux"));

// The directory where all the maps are stored.
static const string kMapsDirectory(kCobotStackPath + "/../maps");

// Index of test set. This will determine which file the results of the test are
// saved to.
int test_set_index_ = -1;

// Indicates that a statistical test is being run, and to save the localization
// results to the file with the specified index.
int statistical_test_index_ = -1;

// The fraction of additive odometry noise for statistical tests.
double odometry_additive_noise_ = 0.05;

//static const uint32_t kTrajectoryColor = 0x6FFF0000;
static const uint32_t kTrajectoryColor = 0x7F000000;
static const uint32_t kPoseCovarianceColor = 0xFF808080;
static const uint32_t kOdometryColor = 0x70FF0000;
// static const uint32_t kTrajectoryColor = 0xFFC0C0C0;
static const uint32_t kLtfCorrespondenceColor = 0x7FFF7700;
static const uint32_t kLtfPointColor = 0xFFFF7700;
static const uint32_t kStfPointColor = 0xFFFF5500;
static const uint32_t kStfCorrespondenceColor = 0x7F994CD9;
static const uint32_t kDfPointColor  = 0x7F37B30C;
static const uint32_t kObjectColor = 0xFF56C4C3;

bool run_ = true;
int debug_level_ = -1;

// Indicates that /Cobot/Kinect/Scan should be used instead of
// /Cobot/Laser, to localize using the Kinect sensor instead of the laser
// rangefinder.
bool use_kinect_ = false;

// Indicates that the user will provide an estimate of an object instance via
// the localization_gui.
bool get_object_instance_ = false;

// Display message for drawing debug vizualizations on the localization_gui.
cobot_msgs::LidarDisplayMsg display_message_;

// Service client to ask localization_gui to save images.
ros::ServiceClient gui_capture_client;

// ROS publisher to publish display_message_ to the ROS topic
// Cobot/VectorLocalization/Gui
ros::Publisher display_publisher_;

// ROS publisher to the CoBot events topic.
ros::Publisher events_publisher_;

// ROS publisher to publish the latest robot localization estimate to
// Cobot/Localization
ros::Publisher localization_publisher_;

// ROS subscriber to the localization gui mouse move events
ros::Subscriber mouse_move_subscriber_;

// ROS subscriber to the localization gui mouse click events
ros::Subscriber mouse_click_subscriber_;

// ROS subscriber to /Cobot/VectorLocalization/GuiKeyboardEvents
ros::Subscriber keyboard_event_subscriber_;

// Parameters and settings for Non-Markov Localization.
VectorMapping::VectorMappingOptions localization_options_;

// Parameters for learning HitL SLAM


// Main class instance for Non-Markov Localization.
VectorMapping localization_(kMapsDirectory);


EMInput em_input_;

AppExpCorrect app_exp_corr_;

Backprop backprop_;

JointOpt joint_opt_;



// Relocalization interface.
//vector_localization::Relocalization relocalization_(kMapsDirectory);

// The last observed laser scan, used for auto localization.
sensor_msgs::LaserScan last_laser_scan_;

// Parameters used for relocalization.
//VectorLocalization2D::LidarParams relocalization_lidar_params_;

// File name of the final image of all observations from all poses with their
// classifications.
char* save_image_file = NULL;

// Directory where images of every episode at every timestep will be saved.
char* episode_images_path = NULL;

// Determines whether STFS will be saved for later object mapping or not.
bool save_stfs_ = false;

// WatchFiles to track changes to config files.
WatchFiles watch_files_;

// Config reader for localization options.
ConfigReader config_((kCobotStackPath + "/").c_str());

// Corrections to angle-dependent errors of the laser scanner.
vector<float> laser_corrections_;

// Resolution of the laser corrections lookup table.
float laser_corrections_resolution_ = 0.0;

// Boolean flag to indicate use of laser scanner corrections.
bool use_laser_corrections_ = false;

// Flag to write LTF observations to disk for error correction computation.
bool save_ltfs_ = false;

// Accept noise-free odometry, and apply noise by mimicing encoders of a
// four-wheel omnidirectional robot.
void ApplyNoiseModel(
    const float dx,
    const float dy,
    const float da,
    const float e,
    float* dx_n,
    float* dy_n,
    float* da_n) {
  // Wheel radius.
  const float R = 0.1;
  Eigen::Matrix<float, 4, 3> M_vel_to_enc;
  Eigen::Matrix<float, 3, 4> M_enc_to_vel;
  const float C = cos(RAD(45.0));
  M_vel_to_enc <<
      C, C, R,
      -C, C, R,
      -C, -C, R,
      C, -C, R;
  const float kSqRt2 = sqrt(2.0);
  M_enc_to_vel <<
      kSqRt2, -kSqRt2, -kSqRt2, kSqRt2,
      kSqRt2, kSqRt2, -kSqRt2, -kSqRt2,
      1.0 / R, 1.0 / R, 1.0 / R, 1.0 / R;
  M_enc_to_vel = M_enc_to_vel / 4.0;
  const Eigen::Vector3f delta(dx, dy, da);
  // Compute noise-free encoder reading.
  const Eigen::Vector4f enc = M_vel_to_enc * delta;
  Eigen::Vector4f enc_noisy = Eigen::Vector4f::Zero();
  // Add noise.
  for (int i = 0; i < 4; ++i) {
    enc_noisy(i) = enc(i) + randn(e * enc(i));
  }
  const Eigen::Vector3f delta_noisy = M_enc_to_vel * enc_noisy;
  *dx_n = delta_noisy(0);
  *dy_n = delta_noisy(1);
  *da_n = delta_noisy(2);
}

void PublishLocation(
    const string& map_name, const float x, const float y, const float angle) {
  cobot_msgs::CobotLocalizationMsg localization_msg_;
  localization_msg_.timeStamp = GetTimeSec();
  localization_msg_.map = map_name;
  localization_msg_.x = x;
  localization_msg_.y = y;
  localization_msg_.angle = angle;
  localization_publisher_.publish(localization_msg_);
}

void ClearDisplay() {
  ClearDrawingMessage(&display_message_);
  display_publisher_.publish(display_message_);
}

void PublishDisplay() {
  display_publisher_.publish(display_message_);
  ClearDrawingMessage(&display_message_);
}

int kbhit() {
  if(!run_) return 0;
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &fds);
}

void nonblock(const bool blocking) {
  struct termios ttystate;

  //get the terminal state
  tcgetattr(STDIN_FILENO, &ttystate);

  if (blocking) {
    //turn off canonical mode
    ttystate.c_lflag &= ~ICANON;
    //minimum of number input read.
    ttystate.c_cc[VMIN] = 1;
  } else {
    //turn on canonical mode
    ttystate.c_lflag |= ICANON;
  }
  //set the terminal attributes.
  tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}




void LoadCorrespondences() {
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


void LoadGradients(vector<double>* gradients) {
  ScopedFile fid("Gradients.txt", "r");
  float grad;
  while (fscanf(fid(), "%f,\n,", &(grad)) == 1) {
    //cout << "grad: " << grad << endl;
    gradients->push_back(double(grad));
  }
}

void LoadHCFitLines(vector<Vector2f>* fit_lines) {
   ScopedFile fid("HCFitLines.txt", "r");
   Vector2f ep1(0,0);
   Vector2f ep2(0,0);
   Vector2f ep3(0,0);
   Vector2f ep4(0,0);
   while (fscanf(fid(), "%f,%f,%f,%f,%f,%f,%f,%f\n,",
                        &(ep1(0)), &(ep1(1)), &(ep2(0)), &(ep2(1)),
                        &(ep3(0)), &(ep3(1)), &(ep4(0)), &(ep4(1))) == 8) {
     fit_lines->push_back(ep1);
     fit_lines->push_back(ep2);
     fit_lines->push_back(ep3);
     fit_lines->push_back(ep4);
   }
}

bool LoadConfiguration(VectorMapping::VectorMappingOptions* options) {
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

void DrawImageLine(const vector2f& p0, const vector2f& p1,
              const vector2f& origin, const float& scale,
              const uint8_t* color,
              cimg_library::CImg<uint8_t>* image) {
  static const int kThickness = 2;
  vector2f p0_transformed = (p0 - origin) / scale;
  vector2f p1_transformed = (p1 - origin) / scale;
  const int height = image->height();
  for (int dx = -kThickness; dx < kThickness; ++dx) {
    for (int dy = -kThickness; dy < kThickness; ++dy) {
      image->draw_line(p0_transformed.x + dx,
                       height - 1 - p0_transformed.y + dy,
                       p1_transformed.x + dx,
                       height - 1 - p1_transformed.y + dy,
                       color);
    }
  }
}

void SaveMap(const char* bag_name,
             const VectorMap& map) {
  static const float kImageResolution = 0.02;
  static const uint8_t kMapColor[] = { 255 };

  const int width = (map.maxX - map.minX) / kImageResolution;
  const int height = (map.maxY - map.minY) / kImageResolution;
  const vector2f g_origin(map.minX, map.minY);
  const Vector2f origin(map.minX, map.minY);

  // Save Observations
  cimg_library::CImg<uint8_t> image(width, height, 1, 1, 0);
  for (size_t i = 0; i < map.Lines().size(); ++i) {
    const line2f& line = map.Line(i);
    DrawImageLine(line.P0(), line.P1(), g_origin,
              kImageResolution, kMapColor, &image);
  }
  image.save("ghc7.png");
}

void SaveResults(const string& bag_name,
                 const vector<Pose2Df>& poses,
                 const vector<PointCloudf>& point_clouds,
                 const vector<NormalCloudf>& normal_clouds,
                 const string& map_name) {
  // Scale of the image in meters per pixel.
  static const float kImageResolution = 0.02;
  static const uint8_t kMapColor[] = { 255 };
  static const uint8_t kPointColor[] = { 255 };

  const VectorMap map(kMapName.c_str(), kMapsDirectory.c_str(), true);
  const int width = (map.maxX - map.minX) / kImageResolution;
  const int height = (map.maxY - map.minY) / kImageResolution;
  const vector2f g_origin(map.minX, map.minY);
  const Vector2f origin(map.minX, map.minY);

  // Save test set reults.
  if (test_set_index_ >=0) {
    const string file_name =
        StringPrintf("non_markov_test_%d.txt", test_set_index_);
    ScopedFile fid(file_name, "a");
    CHECK_NOTNULL(fid());
    for (unsigned int i = 0; i < poses.size(); ++i) {
      fprintf(fid, "%f,%f,%f, ",
              poses[i].translation.x(),
              poses[i].translation.y(),
              poses[i].angle);
    }
    fprintf(fid, "\n");
  }

  // Save Observations
  {
    cimg_library::CImg<uint8_t> image(width, height, 1, 1, 0);
    if (false) {
      for (size_t i = 0; i < map.Lines().size(); ++i) {
        const line2f& line = map.Line(i);
        DrawImageLine(line.P0(), line.P1(), g_origin,
                      kImageResolution, kMapColor, &image);
      }
    }
    for (unsigned int i = 0; i < point_clouds.size() - 1; ++i) {
      const PointCloudf& point_cloud = point_clouds[i];
      const Rotation2Df rotation(poses[i].angle);
      for (unsigned int j = 0; j < point_cloud.size(); ++j) {
        const Vector2f p = (Vector2f(rotation * point_cloud[j] +
            poses[i].translation) - origin) / kImageResolution;
        image.draw_point(p.x(), height - 1 - p.y(), kPointColor);
      }
    }
    image.save((string(bag_name) + string(".")
        + map.mapName + string(".points.png")).c_str());
  }

  // Save Pose locations
  {
    cimg_library::CImg<uint8_t> image(width, height, 1, 1, 0);
    if (false) {
      for (size_t i = 0; i < map.Lines().size(); ++i) {
        const line2f& line = map.Line(i);
        DrawImageLine(line.P0(), line.P1(), g_origin,
                      kImageResolution, kMapColor, &image);
      }
    }
    for (unsigned int i = 0; i < poses.size(); ++i) {
      const Vector2f p = (poses[i].translation - origin) / kImageResolution;
      image.draw_point(p.x(), height - 1 - p.y(), kPointColor);
    }
    image.save((string(bag_name) + string(".") +
        map.mapName + string(".poses.png")).c_str());
  }

  // Save raw data.
  {
    const string data_file = string(bag_name) + string(".") +
        map.mapName + string(".data");
    FILE* fid = fopen(data_file.c_str(), "wb");
    const int num_poses = poses.size();
    // Write all the pose locations.
    fwrite(&num_poses, sizeof(num_poses), 1, fid);
    for (int i = 0; i < num_poses; ++i) {
      fwrite(&(poses[i].translation.x()), sizeof(poses[i].translation.x()),
             1, fid);
      fwrite(&(poses[i].translation.y()), sizeof(poses[i].translation.y()),
             1, fid);
      fwrite(&(poses[i].angle), sizeof(poses[i].angle), 1, fid);
    }
    // Write the observation points.
    fwrite(&num_poses, sizeof(num_poses), 1, fid);
    for (int i = 0; i < num_poses; ++i) {
      const int num_points = point_clouds[i].size();
      fwrite(&num_points, sizeof(num_points), 1, fid);
      const PointCloudf& point_cloud = point_clouds[i];
      for (int j = 0; j < num_points; ++j) {
        fwrite(&(point_cloud[j].x()), sizeof(point_cloud[j].x()), 1, fid);
        fwrite(&(point_cloud[j].y()), sizeof(point_cloud[j].y()), 1, fid);
      }
    }
    fclose(fid);
  }
}

Eigen::MatrixXd ConvertJ(ceres::CRSMatrix& jacobian) {

  int num_rows = jacobian.num_rows;

  cout << "ceres jacobian rows: " << num_rows << endl;
  // Convert the sparse matrix to a dense matrix
  Eigen::MatrixXd denseJacobian = Eigen::MatrixXd::Zero(num_rows, jacobian.num_cols);
  if (num_rows > 0) {
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
  }
  //denseJacobian = Eigen::MatrixXd(jacobianMatrix);
  // Calculate j.transpose j
  //Eigen::MatrixXd jTj = denseJacobian.transpose() * denseJacobian;

  return denseJacobian;
}


void DrawGradients(const size_t start_pose, const size_t end_pose,
                   const vector<double>& gradients, uint32_t color) {

  cout << "drawing gradients" << endl;
  if (gradients.size() < poses.size()) {
    cout << "more poses than gradients" << endl;
    return;
  }
  cout << "start_pose: " << start_pose << endl;
  cout << "end_pose: " << end_pose << endl;
  for (size_t i = start_pose; i < end_pose; ++i) {
    const Vector2f location_gradient(gradients[3*i], gradients[3*i + 1]);
    const Vector2f pose_location = poses[i].translation;
    //const Rotation2Df pose_rotation(poses[j + 2]);
    // gradient DESCENT, so negatize the gradient to get direction of travel
    const Vector2f p2 = pose_location - location_gradient;
    //cout << (pose_location - p2).norm() << endl;
    DrawLine(pose_location, p2, color, &display_message_);
  }
}

void DrawPoseCovariance3D(const Vector2f& pose,
                          const Matrix2f covariance,
                          const float theta_var) {
  static const float kDTheta = RAD(15.0);
  Eigen::SelfAdjointEigenSolver<Matrix2f> solver;
  solver.compute(covariance);
  //cout << "cov mat" << covariance << endl;
  const Matrix2f eigenvectors = solver.eigenvectors();
  const Vector2f eigenvalues = solver.eigenvalues();
  float theta_color = (theta_var / max_theta_var)*255;
  uint32_t color = 0xFF808000 + int(theta_color);
  //cout << "theta_color" << theta_color << endl;
  //cout << "EV1 " << eigenvalues(0) << " EV2 " << eigenvalues(1) << endl;
  for (float a = 0; a < 2.0 * M_PI; a += kDTheta) {
    const Vector2f v1(cos(a) * sqrt(eigenvalues(0)),
                      sin(a) * sqrt(eigenvalues(1)));
    const Vector2f v2(cos(a + kDTheta) * sqrt(eigenvalues(0)),
                      sin(a + kDTheta) * sqrt(eigenvalues(1)));
    const Vector2f v1_global = eigenvectors.transpose() * v1 + pose;
    const Vector2f v2_global = eigenvectors.transpose() * v2 + pose;

    DrawLine(v1_global, v2_global, color, &display_message_);
    //DrawLine(v1_global, v2_global, kPoseCovarianceColor, &display_message_);
  }
}

















void DisplayPoses(const vector<Pose2Df>& poses,
                  const vector<PointCloudf>& point_clouds,
                  const vector<NormalCloudf>& normal_clouds,
                  const char* save_image_file) {
//   if (debug_level_ < 0) return;
  
  ClearDrawingMessage(&display_message_);
  
  static const int kSkipLaserScans = 1;
  CHECK_EQ(poses.size(), point_clouds.size());
  CHECK_GT(poses.size(), 0);

     
  //float max_cost = 1.0;  
  /*
  float max_cost = 0.0;
  vector<float> cc = localization_.GetCeresCost();
  if (cc.size() >= poses.size()) {
    for (size_t i = 0; i < cc.size();  ++i) {
        max_cost = max(cc[i], max_cost);
    }
  }
  else {
    max_cost = 1.0;
  }
  
  uint32_t cost_color;
  if (cc.size() >= poses.size()) {
    cost_color = int((cc[i]/max_cost)*255.0);
  }
  else {
    cost_color = 0x80;
  }

  //cout << "cost color: " << cost_color << endl;
  cost_color = 0xFF800080 + cost_color*16*16;
*/
 
  
  if (covariances_.size() >= poses.size()) {
    for (size_t i = 0; i < poses.size();  ++i) {
      max_theta_var = max(float(covariances_[i](2,2)), max_theta_var);
    }
  }
  else {
     max_theta_var = 1.0;
  }

  
  // Add robot localization trajectory.
  for (size_t i = 0; i < poses.size() - 1; ++i) {
    // color / draw covariances
    if (covariances_.size() > i) {
      Eigen::Matrix2f single_cov;
      single_cov(0,0) = float(covariances_[i](0,0));
      single_cov(0,1) = float(covariances_[i](0,1));
      single_cov(1,0) = float(covariances_[i](1,0));
      single_cov(1,1) = float(covariances_[i](1,1));
      float theta_var = covariances_[i](2,2);
      DrawPoseCovariance3D(poses[i].translation, single_cov, theta_var);
    }
    // draw trajectory
    DrawLine(poses[i].translation, poses[i + 1].translation, kTrajectoryColor, &display_message_);

    //draw arrows for poses here
    double L = 0.5;
    double l = 0.1;
    Vector2f arr(L*cos(poses[i].angle), L*sin(poses[i].angle));

    arr += poses[i].translation;
    Vector2f w1(l*cos(poses[i].angle + 5.0*3.14159/6.0),
                l*sin(poses[i].angle + 5.0*M_PI/6.0));
    Vector2f w2(l*cos(poses[i].angle - 5.0*3.14159/6.0),
                l*sin(poses[i].angle - 5.0*3.14159/6.0));
    Vector2f W1 = w1 + arr;
    Vector2f W2 = w2 + arr;
    // Assign color based on if the pose is part of ANY human constraint
    bool hc = false;
    for (size_t j = 0; j < human_constraints_.size(); ++j) {
      for (size_t k = 0; k < human_constraints_[j].size(); ++k) {
        if (human_constraints_[j][k].anchor_pose_id == int(i) ||
            human_constraints_[j][k].constrained_pose_id == int(i)) {
          hc = true;
        }
      }
    }
    if (hc) {
      DrawLine(poses[i].translation, arr, 0xFF8080FF, &display_message_);
      DrawLine(W1, arr, 0xFF8080FF, &display_message_);
      DrawLine(W2, arr, 0xFF8080FF, &display_message_);
    }
    else {
      DrawLine(poses[i].translation, arr, 0xFF000000, &display_message_);
      DrawLine(W1, arr, 0xFF000000, &display_message_);
      DrawLine(W2, arr, 0xFF000000, &display_message_);
    }
  }
  Eigen::MatrixXd jacobian_;
//   Eigen::MatrixXd jacobian_ = ConvertJ(ceres_jacobian_);
  // Draw all the gradients, and color them by residual origin
//   if (jacobian_init_ && jacobian_.rows() > (poses.size()-1)*3 -1 && false) { // break up all types (odom, hc, stfs)
//     cout << "rows: " << jacobian_.rows() << endl;
//     cout << "cols: " << jacobian_.cols() << endl;
if (false) {
    
    CHECK_GT(jacobian_.rows(), (poses.size()-1)*3 -1);
    //cout << "jacobian rows inside draw: " << jacobian.rows() << endl;
    uint32_t odom_color = 0xFF0000FF;
    uint32_t hc_color = 0xFF00FF00;
    uint32_t stfs_color = 0xFFFF00FF;
    int num_cols = jacobian_.cols();
    int num_odom_rows = (poses.size()-1)*3;
    int num_hc_rows = num_hc_residuals_;
    int num_stf_rows = jacobian_.rows() - (num_odom_rows + num_hc_rows);
    CHECK_EQ(jacobian_.rows(), num_odom_rows + num_hc_rows + num_stf_rows);
    
    vector<double> col_sums;
    // Odometry
    for (int i = 0; i < num_cols; ++i) {
      double col_sum = 0.0;
      for (int j = 0; j < num_odom_rows; ++j) { //just the odometry
        col_sum += jacobian_(j,i);
      }
      col_sums.push_back(col_sum);
    }
    // draw the gradients
    for (size_t i = 0; i < poses.size(); ++i) {
      Vector2f jacobian_dir(col_sums[3 * i], col_sums[3 * i + 1]);
      const Vector2f pose_location = poses[i].translation;
      const Vector2f p2 = pose_location - jacobian_dir;
      DrawLine(pose_location, p2, odom_color, &display_message_);
    }
    col_sums.clear();
    std::cout << "odom viz" << std::endl;
    // human constraints
    for (int i = 0; i < num_cols; ++i) {
      double col_sum = 0.0;
      for (int j = num_odom_rows; j < (num_hc_rows + num_odom_rows); ++j) { //just the human constraints
        col_sum += jacobian_(j,i);
      }
      col_sums.push_back(col_sum);
    }
    // draw the gradients
    for (size_t i = 0; i < poses.size(); ++i) {
      Vector2f jacobian_dir(col_sums[3 * i], col_sums[3 * i + 1]);
      const Vector2f pose_location = poses[i].translation;
      const Vector2f p2 = pose_location - jacobian_dir;
      DrawLine(pose_location, p2, hc_color, &display_message_);
    }
    col_sums.clear();
    std::cout << "odom viz" << std::endl;
    // short term features
    for (int i = 0; i < num_cols; ++i) {
      double col_sum = 0.0;
      for (int j = (num_hc_rows + num_odom_rows); j < jacobian_.rows(); ++j) { //just the stfs
        col_sum += jacobian_(j,i);
      }
      col_sums.push_back(col_sum);
    }
    // draw the gradients
    for (size_t i = 0; i < poses.size(); ++i) {
      Vector2f jacobian_dir(col_sums[3 * i], col_sums[3 * i + 1]);
      const Vector2f pose_location = poses[i].translation;
      const Vector2f p2 = pose_location - jacobian_dir;
      DrawLine(pose_location, p2, stfs_color, &display_message_);
    }
    std::cout << "passed viz" << std::endl;
  }
  else if (ceres_gradients_.size() > 0 && false) { // default condition. draw sum of all gradients
    uint32_t color = 0xFFFF0000;
    DrawGradients(0, poses.size()-1, ceres_gradients_, color);
  }

  
  
//   human_constraints_.clear();
//   ScopedFile numfid("numHC.txt", "r");
//   ScopedFile hcfid("HumanConstraints.txt", "r");
//   if (numfid != NULL && hcfid != NULL) {
//     LoadHumanConstraints();
//   }
// 
//   if (human_constraints_.size() > 2) {
// 
//   // Convert laser scans to WORLD FRAME and add them to display message.
//   //for (int i = 0; i < 2; ++i) {
//   for (size_t i = 0; i < human_constraints_[0].pose_obs_ids_a.size(); ++i) {
//     const int pose_idx = human_constraints_[0].pose_obs_ids_a[i].first;
//     const PointCloudf& point_cloud = point_clouds[pose_idx];
//     const Rotation2Df rotation(poses[pose_idx].angle);
//     const Vector2f pose_location = poses[pose_idx].translation;
//     uint32_t color;
//     if (i == 0) {
//       color = 0xFFFF0000;
//     }
//     else {
//       color = 0xFFFF7700;
//     }
//     //uint32_t color = kLtfPointColor;
//     for (size_t j=0; j<human_constraints_[0].pose_obs_ids_a[i].second.size(); ++j ) {
//       const int obs_idx = human_constraints_[0].pose_obs_ids_a[i].second[j];
//       const Vector2f p(rotation * point_cloud[obs_idx] + pose_location);
//  
//       DrawPoint(p, color, &display_message_);
//     }
//   }
//   //for (int i = 0; i < 2; ++i) {
//   for (size_t i = 0; i < human_constraints_[0].pose_obs_ids_b.size(); ++i) {
//     const int pose_idx = human_constraints_[0].pose_obs_ids_b[i].first;
//     const PointCloudf& point_cloud = point_clouds[pose_idx];
//     const Rotation2Df rotation(poses[pose_idx].angle);
//     const Vector2f pose_location = poses[pose_idx].translation;
//     uint32_t color;
//     if (i == 0) {
//       color = 0xFF0000FF;
//     }
//     else {
//       color = 0xFF00FFFF;
//     }
//     //uint32_t color = kLtfPointColor;
//     for (size_t j=0; j<human_constraints_[0].pose_obs_ids_b[i].second.size(); ++j ) {
//       const int obs_idx = human_constraints_[0].pose_obs_ids_b[i].second[j];
//       const Vector2f p(rotation * point_cloud[obs_idx] + pose_location);
//       DrawPoint(p, color, &display_message_);
//     }
//   }
//   }
//   else {

  // Convert laser scans to WORLD FRAME and add them to display message.
  for (size_t i = 0; i < point_clouds.size(); i += kSkipLaserScans) {
    const PointCloudf& point_cloud = point_clouds[i];
    const Rotation2Df rotation(poses[i].angle);
    // const Vector2f& pose_location = poses[i].translation;
    const Vector2f pose_location = poses[i].translation;
//     std::cout << "have poses in display" << std::endl;
    for (unsigned int j = 0; j < point_cloud.size(); ++j) {
//       if (j%200 == 0) { std::cout << "have points in display" << std::endl; }
      const Vector2f p(rotation * point_cloud[j] + pose_location);
      uint32_t color = kLtfPointColor;
      DrawPoint(p, color, &display_message_);
      // DrawLine(p, pose_location, 0x3Fc0c0c0, &display_message_);
    }
//   }
  }


  if (save_image_file != NULL) {
    const string filename = string(save_image_file) + ".bag";
    rosbag::Bag bag(filename, rosbag::bagmode::Write);
    bag.write("/Cobot/VectorLocalization/Gui",
              ros::Time::now(),
              display_message_);
    bag.close();
  }
  PublishDisplay();
}










void DisplayDebug() {
  static const bool debug = false;
  static Pose2Df last_pose_(0, 0, 0);
  static double t_last = GetTimeSec();
  vector<Pose2Df> poses;
  vector<PointCloudf> point_clouds;
  vector<Pose2Df> pending_poses;

  if (debug_level_ < 1 && GetTimeSec() < t_last + 0.5) {
    return;
  }
  t_last = GetTimeSec();
  const int skip_scans = (debug_level_ < 1) ? 8 : 2;

  if (debug || debug_level_ > 1) {
    printf("%4lu nodes, %3lu pending nodes\n",
           poses.size(), pending_poses.size());
  }
  // Add robot localization trajectory.
  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    DrawLine(poses[i].translation, poses[i + 1].translation, kTrajectoryColor,
             &display_message_);
  }

  // Add laser scans.
  for (size_t i = 0; i + 1 < point_clouds.size(); i += skip_scans) {
    const PointCloudf& point_cloud = point_clouds[i];
    const Rotation2Df rotation(poses[i].angle);
    const Vector2f& translation = poses[i].translation;
    for (unsigned int j = 0; j < point_cloud.size(); ++j) {
      const Vector2f p(rotation * point_cloud[j] + translation);
      const uint32_t color = kStfPointColor;
      DrawPoint(p, color, &display_message_);
    }
  }

  display_publisher_.publish(display_message_);
}

void GetCovarianceFromRelativePose(const Vector2f& relative_location,
                                   const float& relative_angle,
                                   Matrix3f* covariance) {
  covariance->setZero();
  if (relative_location.norm() > FLT_MIN) {
    const Vector2f radial_direction(relative_location.normalized());
    const Vector2f tangential_direction(Rotation2Df(M_PI_2) * radial_direction);
    Matrix2f eigenvectors;
    eigenvectors.leftCols<1>() = radial_direction;
    eigenvectors.rightCols<1>() = tangential_direction;
    Matrix2f eigenvalues;
    eigenvalues.setZero();
    eigenvalues(0, 0) = kRadialTranslationUncertainty;
    eigenvalues(1, 1) = kTangentialTranslationUncertainty;
    covariance->block<2, 2>(0, 0) =
        eigenvectors * eigenvalues * (eigenvectors.transpose());
  }
  (*covariance)(2, 2) = kAngleUncertainty * fabs(relative_angle);
}

bool AddPose(const sensor_msgs::LaserScanPtr& laser_message,
             const vector<double>& keyframes,
             Vector2f* relative_location_ptr,
             float* relative_angle_ptr,
             Vector2f* global_location_ptr,
             float* global_angle_ptr,
             vector<PointCloudf>* point_clouds,
             vector<NormalCloudf>* normal_clouds,
             vector<Pose2Df>* poses,
              vector<double>* timestamps,
             Vector2f* odometry_location,
             float* odometry_angle) {
  static const bool debug = false;
  // Aliasing dereferenced pointers for readability.
  float& relative_angle = *relative_angle_ptr;
  Vector2f& relative_location = *relative_location_ptr;
  float& global_angle = *global_angle_ptr;
  Vector2f& global_location = *global_location_ptr;
  const bool keyframe = binary_search(keyframes.begin(), keyframes.end(),
                                      laser_message->header.stamp.toSec());
  if (!keyframe && relative_location.norm() <
      localization_options_.minimum_node_translation &&
      fabs(relative_angle) < localization_options_.minimum_node_rotation) {
    // Ignore this pose since the robot has not moved since the last update.
    return false;
  }
  if (debug) {
    printf("Relative pose: %6.3f, %6.3f %4.1f\u00b0\n", relative_location.x(),
        relative_location.y(), DEG(relative_angle));
  }
  // Update global pose
  global_location = Rotation2Df(global_angle) * relative_location +
      global_location;
  global_angle = relative_angle + global_angle;

  // Update odometry pose
  *odometry_location = Rotation2Df(*odometry_angle) * relative_location +
      *odometry_location;
  *odometry_angle = relative_angle + *odometry_angle;

  // Compute global pose and covariance.
  Pose2Df pose;
  pose.translation = global_location;
  pose.angle = global_angle;

  // Add laser scan to list of entries.
  PointCloudf point_cloud;
  unsigned int clip_l = 60;
  unsigned int clip_u = 60;
  //for (unsigned int i = 0; i < laser_message->ranges.size(); ++i) {
  for (unsigned int i = clip_l; i < laser_message->ranges.size() - clip_u; ++i)
{
    if (lost_poses) {
      kMaxPointCloudRange = 1.5;
    }
    if (laser_message->ranges[i] < kMinPointCloudRange ||
        laser_message->ranges[i] > kMaxPointCloudRange ||
        !std::isfinite(laser_message->ranges[i])) continue;
    const float angle = laser_message->angle_min +
        laser_message->angle_increment * static_cast<float>(i);
    if (angle < laser_message->angle_min + kAngularMargin ||
        angle > laser_message->angle_max - kAngularMargin) {
      continue;
    }
    float range = laser_message->ranges[i];
    if (use_laser_corrections_) {
      const int angle_index =
          floor((angle + M_PI) / laser_corrections_resolution_);
      CHECK_GE(angle_index, 0);
      CHECK_LT(angle_index, static_cast<int>(laser_corrections_.size()));
      range = range * laser_corrections_[angle_index];
    }
    point_cloud.push_back(localization_options_.sensor_offset +
        Rotation2Df(angle) * Vector2f(range, 0.0));
  }

  NormalCloudf normal_cloud;
  GenerateNormals(kMaxNormalPointDistance, &point_cloud, &normal_cloud);
  if (point_cloud.size() > 1 || 1) {
    point_clouds->push_back(point_cloud);
    normal_clouds->push_back(normal_cloud);
    poses->push_back(pose);
    timestamps->push_back(laser_message->header.stamp.toSec());
    // Reset odometry accumulation.
    relative_angle = 0.0;
    relative_location = Vector2f(0.0, 0.0);
  }
  return true;
}

bool LoadLaserMessage(const rosbag::MessageInstance& message,
                      const vector<double>& keyframes,
                      Vector2f* relative_location,
                      float* relative_angle,
                      Vector2f* global_location,
                      float* global_angle,
                      vector<PointCloudf>* point_clouds,
                      vector<NormalCloudf>* normal_clouds,
                      vector<Pose2Df>* poses,
                      vector<double>* timestamps,
                      Vector2f* odometry_location,
                      float* odometry_angle) {
  sensor_msgs::LaserScanPtr laser_message =
      message.instantiate<sensor_msgs::LaserScan>();
  const bool is_cobot_laser = message.getTopic() == kCobotLaserTopic;
  const bool is_cobot_kinect = message.getTopic() == kKinectScanTopic;
  const bool is_standard_laser =
      kStandardizedData && message.getTopic() == kStandardLaserTopic;
  if (laser_message != NULL && (((!use_kinect_ && is_cobot_laser) ||
      (use_kinect_ && is_cobot_kinect)) || is_standard_laser)) {
    if (debug_level_ > 1) {
      printf("Laser Msg,    t:%.2f\n", message.getTime().toSec());
      fflush(stdout);
    }
    AddPose(laser_message, keyframes, relative_location, relative_angle,
            global_location, global_angle, point_clouds, normal_clouds,
            poses, timestamps, odometry_location, odometry_angle);
    return true;
  }
  return false;
}

// void LoadOdom() {
//   orig_odom_.clear();
// 
//   Pose2Df relative_odom;
//   ScopedFile fid("Odom.txt", "r");
//   CHECK_NOTNULL(fid());
//   static const int kNumFields = 3;
//   CHECK_EQ(orig_odom_.size(),0);
// 
//   while (fscanf(fid(), "%f,%f,%f\n", &(relative_odom.translation.x()),
//                                      &(relative_odom.translation.y()),
//                                      &(relative_odom.angle)) == kNumFields) {
//     relative_odom.angle = atan2(sin(relative_odom.angle), cos(relative_odom.angle));
//     //std::cout << relative_odom.angle << std::endl;
//     orig_odom_.push_back(relative_odom);
//   }
// }

bool LoadOdometryMessage(const rosbag::MessageInstance& message,
                         const Vector2f& odometry_location,
                         const float& odometry_angle,
                         Vector2f* relative_location,
                         float* relative_angle) {
  if (kStandardizedData) {
    nav_msgs::OdometryPtr odometry_message =
        message.instantiate<nav_msgs::Odometry>();
    const string topic_name = message.getTopic();
    if (odometry_message != NULL &&
        message.getTopic() == kStandardOdometryTopic) {
      if (debug_level_ > 1) {
        printf("Std Odometry Msg, t:%.2f\n", message.getTime().toSec());
        fflush(stdout);
      }
      const Vector2f odometry_message_location(
          odometry_message->pose.pose.position.x,
          odometry_message->pose.pose.position.y);
      *relative_location =  kOdometryTranslationScale * (
          Rotation2Df(-odometry_angle) *
          (odometry_message_location - odometry_location));
      const float odometry_message_angle =
          2.0 * atan2(odometry_message->pose.pose.orientation.z,
                      odometry_message->pose.pose.orientation.w);
      *relative_angle = kOdometryRotationScale *
          angle_diff(odometry_message_angle , odometry_angle);
      if (test_set_index_ >= 0 || statistical_test_index_ >= 0) {
        relative_location->x() +=
            randn(odometry_additive_noise_ * relative_location->x());
        relative_location->y() +=
            randn(odometry_additive_noise_ * relative_location->y());
        (*relative_angle) +=
            randn(odometry_additive_noise_ * (*relative_angle));
      }
      return true;
    }
  } else {
    cobot_msgs::CobotOdometryMsgPtr odometry_message =
        message.instantiate<cobot_msgs::CobotOdometryMsg>();
    const string topic_name = message.getTopic();
    if (odometry_message != NULL && message.getTopic() == kCobotOdometryTopic) {
      if (debug_level_ > 1) {
        printf("Cobot Odometry Msg, t:%.2f\n", message.getTime().toSec());
        fflush(stdout);
      }
      if (test_set_index_ >= 0 || statistical_test_index_ >= 0) {
        odometry_message->dx +=
            randn(odometry_additive_noise_ * odometry_message->dx);
        odometry_message->dy +=
            randn(odometry_additive_noise_ * odometry_message->dy);
        odometry_message->dr +=
            randn(odometry_additive_noise_ * odometry_message->dr);
      }
      // Accumulate odometry to update robot pose estimate.
      const Vector2f delta(odometry_message->dx, odometry_message->dy);
      // printf("Delta: %f, %f\n", odometry_message->dx, odometry_message->dy);
      *relative_location = *relative_location +
          kOdometryTranslationScale * (Rotation2Df(*relative_angle) *
          delta);
      *relative_angle = *relative_angle +
          kOdometryRotationScale *  odometry_message->dr;
          //- angle_mod(fabs(odometry_message->dr)) / RAD(5.0) * RAD(1.0);
      return true;
    }
  }
  return false;
}

bool LoadSetLocationMessage(const rosbag::MessageInstance& message,
                            Vector2f* global_location,
                            float* global_angle) {
  cobot_msgs::LocalizationMsgPtr set_location_message =
      message.instantiate<cobot_msgs::LocalizationMsg>();
  const string topic_name = message.getTopic();
  if (set_location_message != NULL &&
      message.getTopic() == kStandardSetLocationTopic) {
    if (debug_level_ > 1) {
      printf("Set Location, t:%.2f\n", message.getTime().toSec());
      fflush(stdout);
    }
    *global_angle = set_location_message->angle;
    global_location->x() = set_location_message->location.x;
    global_location->y() = set_location_message->location.y;
    return true;
  }
  return false;
}

void LoadLaserCorrections() {
  static const char kCorrectionsFile[] =
      "results/sensor_corrections.txt";
  if (!FileExists(kCorrectionsFile)) return;
  ScopedFile fid(kCorrectionsFile, "r");
  if (fid() == NULL) {
    printf("No laser corrections found.\n");
    return;
  }
  int num_bins = 0;
  if (fscanf(fid(), "%d,0\n", &num_bins) != 1) {
    printf("Error parsing laser corrections file!\n");
    return;
  }
  laser_corrections_.resize(num_bins, 1.0);
  laser_corrections_resolution_ = 2.0 * M_PI / static_cast<float>(num_bins);
  bool error = false;
  for (int i = 0; !error && i < num_bins; ++i) {
    float angle = 0.0;
    error = (fscanf(fid(), "%f,%f\n", &angle, &(laser_corrections_[i])) != 2);
    laser_corrections_[i] += 1.0;
  }
  if (error) {
    printf("Error parsing laser corrections file!\n");
    return;
  }
  printf("Loaded %d laser corrections\n", num_bins);
  use_laser_corrections_ = true;
}

void LoadRosBag(const string& bagName, int max_laser_poses, double time_skip,
                const vector<double>& keyframes,
                vector<PointCloudf>* point_clouds,
                vector<NormalCloudf>* normal_clouds,
                vector<Pose2Df>* poses,
                vector<double>* timestamps,
                double* t_duration) {
  CHECK_NOTNULL(point_clouds);
  CHECK_NOTNULL(normal_clouds);
  CHECK_NOTNULL(poses);

  if (lost_poses) {
    cout << "WARNING: Lost Poses" << endl;
  }

  // Last angle as determined by integration of robot odometry from an arbitrary
  // starting location.
  float odometry_angle(0.0);
  // Last location as determined by integration of robot odometry from an
  // arbitrary starting location.
  Vector2f odometry_location(0.0, 0.0);
  // Last angle as determined by integration of robot odometry since location of
  // last pose update.
  float relative_angle(0.0);
  // Last location as determined by integration of robot odometry since location
  // of last pose update.
  Vector2f relative_location(0.0, 0.0);
  // Estimated robot global angle as determined by integration of robot odometry
  // from set starting location.
  float global_angle(kStartingAngle);
  // Estimated robot global location as determined by integration of robot
  // odometry from set starting location.
  Vector2f global_location(kStartingLocation);
  rosbag::Bag bag;
  printf("Opening bag file %s...", bagName.c_str()); fflush(stdout);
  bag.open(bagName,rosbag::bagmode::Read);
  printf(" Done.\n"); fflush(stdout);

  std::vector<std::string> topics;
  if (kStandardizedData) {
    printf("Warning: Interpreting bag file as standardized data.\n");
    topics.push_back(kStandardLaserTopic);
    topics.push_back(kStandardOdometryTopic);
    topics.push_back(kStandardSetLocationTopic);
  } else {
    topics.push_back(kCobotOdometryTopic);
    if (use_kinect_) {
      topics.push_back(kKinectScanTopic);
    } else {
      topics.push_back(kCobotLaserTopic);
    }
  }

  bool localization_initialized = false;
  printf("Reading bag file..."); fflush(stdout);
  if (debug_level_ > 1) printf("\n");
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  double bag_time_start = -1.0;
  double bag_time = 0.0;
  for (rosbag::View::iterator it = view.begin();
       run_ && it != view.end() &&
       (max_laser_poses < 0 ||
       static_cast<int>(poses->size()) < max_laser_poses); ++it) {

    const rosbag::MessageInstance &message = *it;
    bag_time = message.getTime().toSec();
    if (bag_time_start < 0.0) {
      // Initialize bag starting time.
      bag_time_start = bag_time;
    }

    // Ignore messages before elapsed time_skip.
    if (!kStandardizedData && bag_time < bag_time_start + time_skip) continue;

    if (kStandardizedData && !localization_initialized) {
      if (LoadSetLocationMessage(message, &global_location,
                                 &global_angle)) {
        localization_initialized = true;
      }
      // If this message was a set_location message, we don't need to process
      // anything else. If it wasn't, that means localization is still not
      // initialized; so no point processing this message any further.
      continue;
    }

    // Check to see if this is a laser scan message.
    if (LoadLaserMessage(message, keyframes, &relative_location,
        &relative_angle, &global_location, &global_angle, point_clouds,
        normal_clouds, poses, timestamps, &odometry_location,
        &odometry_angle)) {
      continue;
    }

    // Check to see if this is an odometry message.
    if (LoadOdometryMessage(message, odometry_location, odometry_angle,
      &relative_location, &relative_angle)) {
      continue;
    }
  }
  *t_duration = bag_time - bag_time_start;
  if (debug_level_ > 1) printf("\n");
  printf(" Done.\n"); fflush(stdout);
  printf("%d poses loaded.\n", static_cast<int>(point_clouds->size()));
}

void DrawStfs(
    const vector<VectorMapping::PointToPointGlobCorrespondence>&
        point_point_correspondences,
    const vector<double>& poses,
    const vector<vector<vector2f>>& point_clouds,
    const vector<NormalCloudf>& normal_clouds) {
  static const bool kDrawPoints = false;
  for (size_t i = 0; i < point_point_correspondences.size(); ++i) {
    const int pose_index0 = point_point_correspondences[i].pose_index0;
    const int pose_index1 = point_point_correspondences[i].pose_index1;
    DCHECK_EQ(point_point_correspondences[i].points0.size(),
              point_point_correspondences[i].points1.size());
    const Affine2f pose_tf0 =
        Translation2f(poses[3 * pose_index0 + 0], poses[3 * pose_index0 + 1]) *
        Rotation2Df(poses[3 * pose_index0 + 2]);
    const Affine2f pose_tf1 =
        Translation2f(poses[3 * pose_index1 + 0], poses[3 * pose_index1 + 1]) *
        Rotation2Df(poses[3 * pose_index1 + 2]);
    for (size_t j = 0; j < point_point_correspondences[i].points0.size(); ++j) {
      const Vector2f p0 = pose_tf0 * point_point_correspondences[i].points0[j];
      const Vector2f p1 = pose_tf1 * point_point_correspondences[i].points1[j];
      if (kDrawPoints) {
        DrawPoint(p0, cobot_gui::kColorBlue, &display_message_);
        DrawPoint(p1, cobot_gui::kColorBlue, &display_message_);
      }
      DrawLine(p0, p1, kStfCorrespondenceColor, &display_message_);
    }
  }
}

void DrawObservations(
    const size_t start_pose, const size_t end_pose,
    const vector<double>& poses,
    const vector<vector< vector2f> >& point_clouds,
    const std::vector< NormalCloudf >& normal_clouds) {
  static const bool kDisplayTangents = false;
  for (size_t i = start_pose; i <= end_pose; ++i) {
    const vector<vector2f> &point_cloud = point_clouds[i];
    const vector<Vector2f> &normal_cloud = normal_clouds[i];
    const vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
    const float pose_angle = poses[3 * i + 2];
    const Rotation2Df pose_rotation(pose_angle);
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      const vector2f& point = point_cloud[j].rotate(pose_angle) + pose_location;
      if (kDisplayTangents) {
        const Vector2f normal_e = pose_rotation * normal_cloud[j];
        const vector2f normal(normal_e.x(), normal_e.y());
        const vector2f tangent = 0.05 * normal.perp();
        DrawLine(point + tangent,
                 point - tangent,
                 kStfPointColor,
                 &display_message_);
      }
      DrawPoint(point, kStfPointColor, &display_message_);
    }
  }
}

void DrawPoseCovariance(const Vector2f& pose, const Matrix2f& covariance) {
  static const float kDTheta = RAD(15.0);
  Eigen::SelfAdjointEigenSolver<Matrix2f> solver;
  solver.compute(covariance);
  const Matrix2f eigenvectors = solver.eigenvectors();
  const Vector2f eigenvalues = solver.eigenvalues();
  for (float a = 0; a < 2.0 * M_PI; a += kDTheta) {
    const Vector2f v1(cos(a) * sqrt(eigenvalues(0)),
                      sin(a) * sqrt(eigenvalues(1)));
    const Vector2f v2(cos(a + kDTheta) * sqrt(eigenvalues(0)),
                      sin(a + kDTheta) * sqrt(eigenvalues(1)));
    const Vector2f v1_global = eigenvectors.transpose() * v1 + pose;
    const Vector2f v2_global = eigenvectors.transpose() * v2 + pose;
    DrawLine(v1_global, v2_global, kPoseCovarianceColor, &display_message_);
  }
}

void DrawPoses(const size_t start_pose, const size_t end_pose,
    const vector<Pose2Df>& odometry_poses, const vector<double>& poses,
    const vector<Matrix2f>& covariances) {
  static const bool kDrawCovariances = false;
  Vector2f pose_location_last(0.0, 0.0);
  float pose_angle_last = 0.0;
  const bool valid_covariances = (covariances.size() > end_pose);
  //double L = 0.5;
  //double l = 0.1;

  for (size_t i = start_pose; i <= end_pose; ++i) {
    const Vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
/*
    //draw arrows for poses here
    Vector2f arr(L*cos(poses[3*i+2]), L*sin(poses[3*i+2]));
    arr += pose_location;
    Vector2f w1(l*cos(poses[3*i+2] + 5.0*3.14159/6.0), l*sin(poses[3*i+2] +
5.0*3.14159/6.0));
    Vector2f w2(l*cos(poses[3*i+2] - 5.0*3.14159/6.0), l*sin(poses[3*i+2] -
5.0*3.14159/6.0));
    Vector2f W1 = w1 + arr;
    Vector2f W2 = w2 + arr;

    DrawLine(pose_location, arr, kOdometryColor, &display_message_);
    DrawLine(W1, arr, kOdometryColor, &display_message_);
    DrawLine(W2, arr, kOdometryColor, &display_message_);
    */

   if (i > start_pose) {
      DrawLine(pose_location, pose_location_last, kTrajectoryColor,
               &display_message_);
      const Vector2f odometry =
          Rotation2Df(pose_angle_last) *
          Rotation2Df(-odometry_poses[i - 1].angle) *
          (odometry_poses[i].translation - odometry_poses[i - 1].translation);
      DrawLine(pose_location, Vector2f(pose_location_last + odometry),
               kOdometryColor, &display_message_);
      if (kDrawCovariances && valid_covariances) {
        DrawPoseCovariance(pose_location, covariances[i]);
      }
    }
    pose_location_last = pose_location;
    pose_angle_last = poses[3 * i + 2];
  }
}

void CorrespondenceCallback(
    const vector<double>& poses,
    const vector<vector< vector2f> >& point_clouds,
    const vector< NormalCloudf >& normal_clouds,
    const vector<VectorMapping::PointToPointGlobCorrespondence>&
        point_point_correspondences,
    const vector<double>& gradients,
    const vector<Matrix2f>& covariances,
    const vector<Pose2Df>& odometry_poses,
    const size_t start_pose,
    const size_t end_pose) {
  static const bool kDisplayStfCorrespondences = true;
  uint32_t color = 0xFFFF0000;
  CHECK_EQ(poses.size(), point_clouds.size() * 3);
  ClearDrawingMessage(&display_message_);
  DrawPoses(start_pose, end_pose, odometry_poses, poses, covariances);
  DrawGradients(start_pose, end_pose, gradients, color);
  DrawObservations(start_pose, end_pose, poses, point_clouds, normal_clouds);
  if (kDisplayStfCorrespondences) {
    DrawStfs(point_point_correspondences, poses, point_clouds, normal_clouds);
  }
  display_publisher_.publish(display_message_);
}

pair<Vector2f, cimg_library::CImg<float>> ConstructSingleSDF(
                          perception_2d::PointCloudf point_cloud,
                                            const Vector2f source,
                                            const float pose_angle) {
  float image_resolution = 0.05; //meters
  float image_border = 0.0; //meters
  const float pixel_half_width = sqrt(2.0) * image_resolution;
  const float laser_angular_resolution = M_PI * (270.0 / 1024.0) / 180.0;

  float min_x(FLT_MAX), min_y(FLT_MAX);
  float max_x(-FLT_MAX), max_y(-FLT_MAX);
  for (size_t k = 0; k < point_cloud.size() ; ++k) {
    const Vector2f point = Rotation2Df(pose_angle) * point_cloud[k] + source;
    min_x = min(min_x, point[0]);
    max_x = max(max_x, point[0]);
    min_y = min(min_y, point[1]);
    max_y = max(max_y, point[1]);
  }

  const float width = max_x - min_x + 2.0 * image_border;
  const float height = max_y - min_y + 2.0 * image_border;
  const unsigned int image_width = ceil(width / image_resolution);
  const unsigned int image_height = ceil(height /image_resolution);

  cimg_library::CImg<float> sdf_weights_image(image_width, image_height);
  const Vector2f image_origin(min_x - image_border, min_y - image_border);

  float eps = 0.04; //meters

  OMP_PARALLEL_FOR
  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      sdf_weights_image(x, y) = 0.0;
    }
  }

  OMP_PARALLEL_FOR
  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      for (size_t k = 0; k < point_cloud.size(); ++k) {
        const Vector2f& point = Rotation2Df(pose_angle)*point_cloud[k] + source;
        Vector2f line_dir = (point - source);
        line_dir.normalize();
        const Vector2f line_perp = Perp2(line_dir);
        const Vector2f pixel_loc= image_origin+image_resolution*Vector2f(x,y);

        const bool within_angle_tolerance =
            (fabs(line_perp.dot(point-pixel_loc)) / (point - source).norm()
            < 0.5 * laser_angular_resolution);

        const bool along_viewing_ray =
            (fabs(line_perp.dot(pixel_loc - point)) < pixel_half_width);
        if (!along_viewing_ray && !within_angle_tolerance) continue;

        // Locations farther than the observed point along the viewing ray
        // from the source have
        // a negative SDF value since they are "occupied".
        const float sdf_value = line_dir.dot(point - pixel_loc);

        //only revise values in front of or near range reading
        if (sdf_value <= eps) {
          sdf_weights_image(x,y) += 1.0;
        }
      }
    }
  }

  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      if (sdf_weights_image(x, y) > 0.0) {
        sdf_weights_image(x,y) = 255.0;
      }
    }
  }

  pair<Vector2f, cimg_library::CImg<float>> sdf = std::make_pair(image_origin,
                                                            sdf_weights_image);
  return sdf;
}

cimg_library::CImg<float> NormalizeSDF(
                             pair<Vector2f, cimg_library::CImg<float>> SDF_a,
                             pair<Vector2f, cimg_library::CImg<float>> SDF_b) {

  cimg_library::CImg<float> normalized_sdf;

  //create new master SdfObject with proper size
  float O0x = SDF_a.first(0);
  float O0y = SDF_a.first(1);
  float O1x = SDF_b.first(0);
  float O1y = SDF_b.first(1);

  int h0 = SDF_a.second.height();
  int h1 = SDF_b.second.height();
  int w0 = SDF_a.second.width();
  int w1 = SDF_b.second.width();

  float res = 0.05; //meters
  float new_height_wc =
                     max(max(O0y+h0*res,O0y+h1*res),max(O1y+h0*res,O1y+h1*res));
  float new_width_wc =
                     max(max(O0x+w0*res,O0x+w1*res),max(O1x+w0*res,O1x+w1*res));
  Vector2f new_origin(min(O0x, O1x), min(O0y, O1y));

  const unsigned int image_height = ceil(fabs(new_height_wc-new_origin(1))/res);
  const unsigned int image_width = ceil(fabs(new_width_wc - new_origin(0))/res);

  int ix_master = round((O0x - new_origin(0))/res);
  int iy_master = round((O0y - new_origin(1))/res);

  int ix_add = round((O1x - new_origin(0))/res);
  int iy_add = round((O1y - new_origin(1))/res);

  cimg_library::CImg<float> sdf_weights_image(image_width, image_height);

  OMP_PARALLEL_FOR
  for (size_t x=0; x<image_width; x++) {
    for (size_t y=0; y<image_height; y++) {
      sdf_weights_image(x,y) = 0.0;
    }
  }

  OMP_PARALLEL_FOR
  for (int x=0; x<w0; x++) {
    for (int y=0; y<h0; y++) {
      sdf_weights_image(x+ix_master,y+iy_master) += SDF_a.second(x,y);
    }
  }

  OMP_PARALLEL_FOR
  for (int x=0; x<w1; x++) {
    for (int y=0; y<h1; y++) {
      sdf_weights_image(x+ix_add,y+iy_add) += SDF_b.second(x,y);
    }
  }
  sdf_weights_image = sdf_weights_image/2.0;
  return sdf_weights_image;
}

void AnimateCeres(vector<pair<Vector2f, Vector2f> > lines,
                  vector<Vector2f> points) {

  cout << "animating poses!!!!" << endl;
  ClearDrawingMessage(&display_message_);
  for (size_t i = 0; i < lines.size(); ++i) { //lines
    DrawLine(lines[i].first, lines[i].second, 0xFFFF00FF, &display_message_);
  }
  for (size_t i = 0; i < points.size(); ++i) { //points
    DrawPoint(points[i], kStfPointColor, &display_message_);
  }
  display_publisher_.publish(display_message_);
  //exit(1);
  Sleep(2);
}

void EvaluateConsistency2(vector<Pose2Df> poses,
                       std::vector<perception_2d::PointCloudf> point_clouds) {

  int EnMLHistoryLength = 10;

  float laser_range = 10.0;
  CHECK_EQ(poses.size()%3, 0);
  int num_poses = poses.size() / 3;

  cimg_library::CImg<float> inconsistency(num_poses, num_poses);
  OMP_PARALLEL_FOR
  for (int x = 0; x < num_poses; ++x) {
    for (int y = 0; y < num_poses; ++y) {
      inconsistency(x, y) = 0.0;
    }
  }

  vector<pair<Vector2f, cimg_library::CImg<float>>> all_sdfs;
  OMP_PARALLEL_FOR
  for (int i = 0; i < num_poses; i++) {
    const vector<Vector2f> point_cloud = point_clouds[i];
    const Vector2f source = poses[i].translation;
    const float pose_angle = poses[i].angle;
    pair<Vector2f, cimg_library::CImg<float>> sdf;
    sdf = ConstructSingleSDF(point_cloud, source, pose_angle);
    all_sdfs.push_back(sdf);
    if (i%20 == 0) {
      cout << "outer" << i << endl;
    }
  }
  cout << "PASSED" << endl;
  for (int i = 0; i < num_poses; i++) {
    //for (int j = i+1; j < num_poses; j++) {
    for (int j = i + EnMLHistoryLength - 1; j < num_poses; j++) {
      const Vector2f source_a = poses[i].translation;
      const Vector2f source_b = poses[j].translation;
      //cout << " " << endl;
      //if ((source_a - source_b).norm() < (2*laser_range - 4.0)) {
      if ((source_a - source_b).norm() < laser_range) {
        float pairwise_inconsistency = 0.0;
        cimg_library::CImg<float> merged;
        merged = NormalizeSDF(all_sdfs[i], all_sdfs[j]);
        for (int x = 0; x < merged.width(); ++x) {
          for (int y = 0; y < merged.height(); ++y) {
            if (merged(x,y) > 0.0 && merged(x,y) < 255.0)  {
              pairwise_inconsistency += 1.0;
            }
          }
        }
        inconsistency(i,j) = pairwise_inconsistency;
        inconsistency(j,i) = pairwise_inconsistency;
      }
    }
  }
  //TODO tag iteration
  //TODO: pick a better norm for OT hist
  //cout << "before finding max" << endl;
  float max_incon = 0.0;
  for (int x = 0; x < num_poses; ++x) {
    for (int y = 0; y < num_poses; ++y) {
      max_incon = max(max_incon, inconsistency(x, y));
    }
  }

  cout << "normalized" << endl;
  for (int x = 0; x < num_poses; ++x) {
    for (int y = 0; y < num_poses; ++y) {
      inconsistency(x, y) = (inconsistency(x,y) / max_incon) * 255.0;
      //cout << inconsistency(x, y) << endl;
    }
  }
  int bins[10];
  for (int i = 0; i < 10; ++i) {
    bins[i] = 0;
  }
  for (int x = 0; x < num_poses; ++x) {
    for (int y = 0; y < num_poses; ++y) {
      bins[int(inconsistency(x,y) / 25.5)] ++;
    }
  }

  for (int i = 0; i < 10; ++i) {
    cout << bins[i] << endl;
  }

  const string consistency_image_file =
      StringPrintf("consistency%d.png", correction_number);
  inconsistency.save_png(consistency_image_file.c_str());
  correction_number++;
}

void SaveLoggedPoses(const string& filename,
                     const vector<Pose2Df>& logged_poses,
                     const vector<double>& timestamps) {
  ScopedFile fid(filename, "w");
  for (size_t i = 0; i < logged_poses.size(); ++i) {
    fprintf(fid(), "%f %f %f %f\n",
            timestamps[i],
            logged_poses[i].translation.x(),
            logged_poses[i].translation.y(),
            logged_poses[i].angle);
  }
}

void LoadKeyframes(const string& file, vector<double>* keyframes) {
  ScopedFile fid(file, "r");
  if (fid() == NULL) return;
  double t = 0.0;
  keyframes->clear();
  while (fscanf(fid(), "%lf\n", &t) == 1) {
    keyframes->push_back(t);
  }
  sort(keyframes->begin(), keyframes->end());
  printf("Loaded %lu keyframes\n", keyframes->size());
}

//transform point clouds and save them in WORLD FRAME
void SaveStfsandCovars(
    const string& map_name,
    const vector<Pose2Df>& poses,
    const vector<PointCloudf>& point_clouds,
    const vector<NormalCloudf>& normal_clouds,
    const string& bag_file,
    double timestamp) {
  static const bool kDisplaySteps = false;
  const string stfs_and_covars_file = bag_file + ".stfs.covars";
  ScopedFile fid(stfs_and_covars_file, "w");
  fprintf(fid(), "%s\n", map_name.c_str());
  fprintf(fid(), "%lf\n", timestamp);
  VectorMap map(map_name, kMapsDirectory, true);
  if (kDisplaySteps) {
    ClearDrawingMessage(&display_message_);
    nonblock(true);
  }
  // edit to save partial trajectories
  for (size_t i = 0; i < point_clouds.size(); ++i) {
    const Rotation2Df pose_rotation(poses[i].angle);
    const Affine2f pose_transform =
        Translation2f(poses[i].translation) * pose_rotation;
    //cout << i << "\n" << covariances_[i] << endl;
    //printf("%f\n", covariances_[i](0,0));
    for (unsigned int j = 0; j < point_clouds[i].size(); ++j) {
      const Vector2f p = pose_transform * point_clouds[i][j];
      if (kDisplaySteps) {
        DrawLine(p, poses[i].translation, 0x1FC0C0C0, &display_message_);
      }
      const vector2f p_g(p.x(), p.y());

      const Vector2f n = pose_rotation * normal_clouds[i][j];
      fprintf(fid(), "%.4f,%.4f,%.4f,%.4f,%.4f, %.4f,%.4f,"
              "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",
          poses[i].translation.x(), poses[i].translation.y(),
          poses[i].angle, p.x(), p.y(), n.x(), n.y(),
          covariances_[i](0,0), covariances_[i](0,1), covariances_[i](0,2),
          covariances_[i](1,0), covariances_[i](1,1), covariances_[i](1,2),
          covariances_[i](2,0), covariances_[i](2,1), covariances_[i](2,2));
      if (kDisplaySteps) {
        DrawPoint(p, kStfPointColor, &display_message_);
      }
    }
    if (kDisplaySteps) {
      PublishLocation(kMapName, poses[i].translation.x(),
                      poses[i].translation.y(), poses[i].angle);
      PublishDisplay();
      while (kbhit() == 0) {
        Sleep(0.02);
      }
      fgetc(stdin);
      printf("\r");
      fflush(stdout);
      const string file_name = StringPrintf(
          "poses/%04d.png", static_cast<int>(i));
      cobot_msgs::LocalizationGuiCaptureSrv::Request req;
      cobot_msgs::LocalizationGuiCaptureSrv::Response res;
      req.filename = string(file_name);
      printf("Saving image to %s\n", req.filename.c_str());
      gui_capture_client.call(req, res);
    }
  }
  if (kDisplaySteps) {
    PublishDisplay();
  }
}

void SaveStfs(
    const string& map_name,
    const vector<Pose2Df>& poses,
    const vector<PointCloudf>& point_clouds,
    const vector<NormalCloudf>& normal_clouds,
    const string& bag_file,
    double timestamp) {
  static const bool kDisplaySteps = false;
  const string stfs_file = bag_file + ".stfs";
  ScopedFile fid(stfs_file, "w");
  fprintf(fid(), "%s\n", map_name.c_str());
  fprintf(fid(), "%lf\n", timestamp);
  VectorMap map(map_name, kMapsDirectory, true);
  if (kDisplaySteps) {
    ClearDrawingMessage(&display_message_);
    nonblock(true);
  }
  for (size_t i = 0; i < point_clouds.size(); ++i) {
    const Rotation2Df pose_rotation(poses[i].angle);
    const Affine2f pose_transform =
        Translation2f(poses[i].translation) * pose_rotation;
    for (unsigned int j = 0; j < point_clouds[i].size(); ++j) {
      const Vector2f p = pose_transform * point_clouds[i][j];
      if (kDisplaySteps) {
        DrawLine(p, poses[i].translation, 0x1FC0C0C0, &display_message_);
      }
      const vector2f p_g(p.x(), p.y());
      const Vector2f n = pose_rotation * normal_clouds[i][j];
      fprintf(fid(), "%.4f,%.4f,%.4f, %.4f,%.4f, %.4f,%.4f\n",
          poses[i].translation.x(), poses[i].translation.y(),
          poses[i].angle, p.x(), p.y(), n.x(), n.y());
      if (kDisplaySteps) {
        DrawPoint(p, kStfPointColor, &display_message_);
      }
    }
    if (kDisplaySteps) {
      PublishLocation(kMapName, poses[i].translation.x(),
                      poses[i].translation.y(), poses[i].angle);
      PublishDisplay();
      while (kbhit() == 0) {
        Sleep(0.02);
      }
      fgetc(stdin);
      printf("\r");
      fflush(stdout);
      const string file_name = StringPrintf(
          "poses/%04d.png", static_cast<int>(i));
      cobot_msgs::LocalizationGuiCaptureSrv::Request req;
      cobot_msgs::LocalizationGuiCaptureSrv::Response res;
      req.filename = string(file_name);
      printf("Saving image to %s\n", req.filename.c_str());
      gui_capture_client.call(req, res);
    }
  }
  if (kDisplaySteps) {
    PublishDisplay();
  }
}

//Load point clouds into ROBOT FRAME
void LoadStfsandCovars(
    const string& stfs_file, vector<PointCloudf>* point_cloud_ptr,
    vector<NormalCloudf>* normal_cloud_ptr, vector<Pose2Df>* poses_ptr,
    double* timestamp) {
  vector<PointCloudf>& point_cloud = *point_cloud_ptr;
  vector<NormalCloudf>& normal_cloud = *normal_cloud_ptr;
  vector<Pose2Df>& poses = *poses_ptr;
  PointCloudf single_point_cloud;
  NormalCloudf single_normal_cloud;
  //ScopedFile fid(stfs_file + ".stfs.covars", "r");
  ScopedFile fid(stfs_file, "r");
  CHECK_NOTNULL(fid());
  static const int kNumFields = 16;
  Vector2f point(0, 0);
  Vector2f normal(0, 0);
  Pose2Df pose(0, 0, 0);
  Matrix3d one_cov = Matrix3d::Zero();
  char map_name_str[64];
  if (fscanf(fid(), "%s\n", map_name_str) != 1) {
    fprintf(stderr, "ERROR: Unable to read map name.\n");
    exit(1);
  }
  if (fscanf(fid(), "%lf\n", timestamp) != 1) {
    fprintf(stderr, "ERROR: Unable to read map timeStamp.\n");
    exit(1);
  }
  CHECK_EQ(point_cloud.size(),0);
  CHECK_EQ(normal_cloud.size(),0);
  CHECK_EQ(poses.size(),0);
  CHECK_EQ(covariances_.size(),0);

  while (fscanf(fid(), "%f,%f,%f, %f,%f, %f,%f,"
                       "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\n",
    &(pose.translation.x()), &(pose.translation.y()), &(pose.angle),
    &(point.x()), &(point.y()), &(normal.x()), &(normal.y()),
    &(one_cov(0,0)), &(one_cov(0,1)), &(one_cov(0,2)),
    &(one_cov(1,0)), &(one_cov(1,1)), &(one_cov(1,2)),
    &(one_cov(2,0)), &(one_cov(2,1)), &(one_cov(2,2))) == kNumFields) {

    if (poses.size() == 0) {
      poses.push_back(pose);
      //cout << covariances_.size() << "\n" << one_cov << endl;
      covariances_.push_back(one_cov);
      single_point_cloud.push_back(point);
      single_normal_cloud.push_back(normal);
    }
    else if (pose.translation != poses[poses.size()-1].translation ||
             pose.angle != poses[poses.size()-1].angle) {
      const Rotation2Df R(-poses[poses.size()-1].angle);
      const Vector2f pose_location = -poses[poses.size()-1].translation;
      for (size_t i = 0; i < single_point_cloud.size(); i++) {
          single_point_cloud[i] = R*(single_point_cloud[i] + pose_location);
          single_normal_cloud[i] = R*(single_normal_cloud[i] + pose_location);
      }
      point_cloud.push_back(single_point_cloud);
      normal_cloud.push_back(single_normal_cloud);
      single_point_cloud.clear();
      single_normal_cloud.clear();
      poses.push_back(pose);
      //cout << covariances_.size() << "\n" << one_cov << endl;
      covariances_.push_back(one_cov);
      single_point_cloud.push_back(point);
      single_normal_cloud.push_back(normal);
    }
    else {
      single_point_cloud.push_back(point);
      single_normal_cloud.push_back(normal);
    }
  }
  if (single_point_cloud.size() != 0) {
    const Rotation2Df R(-poses[poses.size()-1].angle);
    const Vector2f pose_location = -poses[poses.size()-1].translation;
    for (size_t i = 0; i < single_point_cloud.size(); i++) {
      single_point_cloud[i] = R*(single_point_cloud[i] + pose_location);
      single_normal_cloud[i] = R*(single_normal_cloud[i] + pose_location);
    }
    point_cloud.push_back(single_point_cloud);
    normal_cloud.push_back(single_normal_cloud);
  }
}

void LoadStfs(
    const string& stfs_file, vector<PointCloudf>* point_cloud_ptr,
    vector<NormalCloudf>* normal_cloud_ptr, vector<Pose2Df>* poses_ptr,
    double* timestamp) {
  vector<PointCloudf>& point_cloud = *point_cloud_ptr;
  vector<NormalCloudf>& normal_cloud = *normal_cloud_ptr;
  vector<Pose2Df>& poses = *poses_ptr;
  PointCloudf single_point_cloud;
  NormalCloudf single_normal_cloud;
  ScopedFile fid(stfs_file, "r");
  CHECK_NOTNULL(fid());
  static const int kNumFields = 7;
  Vector2f point(0, 0);
  Vector2f normal(0, 0);
  Pose2Df pose(0, 0, 0);
  char map_name_str[64];
  if (fscanf(fid(), "%s\n", map_name_str) != 1) {
    fprintf(stderr, "ERROR: Unable to read map name.\n");
    exit(1);
  }
  if (fscanf(fid(), "%lf\n", timestamp) != 1) {
    fprintf(stderr, "ERROR: Unable to read map timeStamp.\n");
    exit(1);
  }
  CHECK_EQ(point_cloud.size(),0);
  CHECK_EQ(normal_cloud.size(),0);
  CHECK_EQ(poses.size(),0);

  while (fscanf(fid(), "%f,%f,%f, %f,%f, %f,%f\n",
    &(pose.translation.x()), &(pose.translation.y()), &(pose.angle),
    &(point.x()), &(point.y()), &(normal.x()), &(normal.y())) == kNumFields) {

    if (poses.size() == 0) {
      poses.push_back(pose);
      single_point_cloud.push_back(point);
      single_normal_cloud.push_back(normal);
    }
    else if (pose.translation != poses[poses.size()-1].translation ||
             pose.angle != poses[poses.size()-1].angle) {
      const Rotation2Df R(-poses[poses.size()-1].angle);
      const Vector2f pose_location = -poses[poses.size()-1].translation;
      for (size_t i = 0; i < single_point_cloud.size(); i++) {
          single_point_cloud[i] = R*(single_point_cloud[i] + pose_location);
          single_normal_cloud[i] = R*(single_normal_cloud[i] + pose_location);
      }
      point_cloud.push_back(single_point_cloud);
      normal_cloud.push_back(single_normal_cloud);
      single_point_cloud.clear();
      single_normal_cloud.clear();
      poses.push_back(pose);
      single_point_cloud.push_back(point);
      single_normal_cloud.push_back(normal);
    }
    else {
      single_point_cloud.push_back(point);
      single_normal_cloud.push_back(normal);
    }
  }
  if (single_point_cloud.size() != 0) {
    const Rotation2Df R(-poses[poses.size()-1].angle);
    const Vector2f pose_location = -poses[poses.size()-1].translation;
    for (size_t i = 0; i < single_point_cloud.size(); i++) {
      single_point_cloud[i] = R*(single_point_cloud[i] + pose_location);
      single_normal_cloud[i] = R*(single_normal_cloud[i] + pose_location);
    }
    point_cloud.push_back(single_point_cloud);
    normal_cloud.push_back(single_normal_cloud);
  }
}











vector<SingleInput> LoadLogFile(const string& log_file) {
  vector<SingleInput> logged_input;
  string full_line;
  string line;

  std::fstream stream(log_file);

  getline(stream, full_line);
  double total_time = stod(full_line);
  cout << "total_time: " << total_time << endl;
  getline(stream, full_line);
  int num_entries = stoi(full_line);
  getline(stream, full_line);
  int num_constraints = stoi(full_line);
  cout << "num constriants: " << num_constraints << endl;

  for (int i = 0; i < num_entries; ++i) {
    if (getline(stream, full_line)) {
      SingleInput one_input;
      std::stringstream iss;
      iss << full_line;
      getline(iss, line, ',');
      int constraint_type = stoi(line);
      getline(iss, line, ',');
      int undone = stoi(line);
      //cout << constraint_type << endl;
      //cout << undone << endl;
//       one_input.type_of_constraint = constraint_type;
      one_input.undone = undone;

      vector<Vector2f> logged_selected_points;
      Vector2f new_point;
      int num_selected_points = 0;
      if (constraint_type == 1) { //colocation
       num_selected_points = 2;
       one_input.type_of_constraint = CorrectionType::kPointCorrection;
      }
      else if (constraint_type == 3) { // Corners
        num_selected_points = 8;
        one_input.type_of_constraint = CorrectionType::kCornerCorrection;
      }
      else if (constraint_type == 2 || constraint_type == 4 ||
               constraint_type == 5 || constraint_type == 7) { // other
        num_selected_points = 4;
        if (constraint_type == 2) {
          one_input.type_of_constraint = CorrectionType::kLineSegmentCorrection;
        }
        else if (constraint_type == 4) {
          one_input.type_of_constraint = CorrectionType::kColinearCorrection;
        }
        else if (constraint_type == 5) {
          one_input.type_of_constraint = CorrectionType::kPerpendicularCorrection;
        }
        else if (constraint_type == 7) {
          one_input.type_of_constraint = CorrectionType::kParallelCorrection;
        }
      }
      else { // garbage
        one_input.type_of_constraint = CorrectionType::kUnknownCorrection;
        cout << "ERROR: unidentified constraint type read in log file." << endl;
      }
      for (int j = 0; j < num_selected_points; ++j) {
        getline(stream, full_line);
        std::stringstream instream;
        instream << full_line;
        getline(instream, line, ',');
        float px = stof(line);
        getline(instream, line, ',');
        float py = stof(line);
        new_point(0) = px;
        new_point(1) = py;
        logged_selected_points.push_back(new_point);
      }
      one_input.input_points = logged_selected_points;
      logged_input.push_back(one_input);

      cout << constraint_type << ", " << one_input.undone << endl;
      for (size_t j = 0; j < one_input.input_points.size(); ++j) {
        Vector2f p = one_input.input_points[j];
        cout << p(0) << ", " << p(1) << endl;
      }
    }
    else {
      break;
    }
  }

  return logged_input;
}







//TODO: modify replay for new architecture

void ReplayLog() {
  bool log_entry_found = false;
  if (current_replay_index < int(logged_input.size())) {
    while (!log_entry_found && current_replay_index<int(logged_input.size())) {
      if (!logged_input[current_replay_index].undone) {


        //TODO: move to seperate function
        std::vector<std::vector<Eigen::Vector2f>> world_frame_point_clouds;
        for (size_t i = 0; i < init_point_clouds.size(); ++i) {
          const Rotation2Df pose_rotation(poses[i].angle);
          const Affine2f pose_transform = Translation2f(poses[i].translation) * pose_rotation;
          std::vector<Eigen::Vector2f> single_point_cloud;
          for (unsigned int j = 0; j < init_point_clouds[i].size(); ++j) {
            const Vector2f p = pose_transform * init_point_clouds[i][j];
            single_point_cloud.push_back(p);
          }
          world_frame_point_clouds.push_back(single_point_cloud);
        }
      
      
      
      //NOTE: maybe use this for evaluation.... maybe
      //if (localization_options_.EvaluateConsistency2 != NULL) {
      //  localization_options_.EvaluateConsistency2(poses, init_point_clouds);
      //}

      
        log_entry_found = true;
        SingleInput current_sim = logged_input[current_replay_index];
      
        //bool debug = false;
        int init_hc_size = human_constraints_.size();
        prev_poses = poses;
        prev_covariances_ = covariances_;
        selected_points_ = current_sim.input_points;
        correction_type_ = current_sim.type_of_constraint;


//TODO: change these to call constructor with all relavent args instead of assigning one by one 
      
        em_input_.local_version_point_clouds_ = world_frame_point_clouds;
        em_input_.selected_points_ = selected_points_;
        em_input_.correction_type_ = correction_type_;
      
        em_input_.Run();
      
        std::vector<int> corrected_poses = em_input_.corrected_poses_;
        std::vector<int> anchor_poses = em_input_.anchor_poses_;
        std::pair<int, int> backprop_bounds = em_input_.backprop_bounds_;
        std::vector<Eigen::Vector2f> new_selected_points = em_input_.selected_points_;

	std::cout << "made it here" << std::endl;
	
//       ClearDrawingMessage(&display_message_);
//       DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
        if (backprop_bounds.first >= 0 && backprop_bounds.second >= 1) {
          app_exp_corr_.correction_type_ = correction_type_;
          app_exp_corr_.selected_points_ = new_selected_points;
          app_exp_corr_.corrected_poses_ = corrected_poses;
          app_exp_corr_.anchor_poses_ = anchor_poses;
          app_exp_corr_.poses_ = poses;
     
          app_exp_corr_.Run();

          poses = app_exp_corr_.poses_;
          Eigen::Vector3f correction = app_exp_corr_.correction_;
          std::vector<VectorMapping::HumanConstraint> new_human_constraints;
          new_human_constraints = app_exp_corr_.new_human_constraints_;
          human_constraints_.push_back(new_human_constraints);
      
  //       ClearDrawingMessage(&display_message_);
  //       DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
        //std::cout << "BP bounds: " << backprop_bounds.first << ", " << backprop_bounds.second << std::endl;
      
          backprop_.poses_ = poses;
          backprop_.correction_ = correction;
          backprop_.backprop_bounds_ = backprop_bounds;
          backprop_.d3_covariances_ = covariances_;
      
          backprop_.Run();
      
          covariances_ = backprop_.d3_covariances_;
          poses = backprop_.poses_;

  //       ClearDrawingMessage(&display_message_);
  //       DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
          ceres_gradients_.clear();
          jacobian_init_ = false;
        
          for (size_t i = 0; i < poses.size(); ++i) { 
            float angle = poses[i].angle;
            //std::cout << angle << " ::: " << atan2(sin(angle), cos(angle)) << std::endl;
            poses[i].angle = atan2(sin(angle), cos(angle));
          }
        
          //joint_opt_.orig_odom_ = orig_odom_;
          joint_opt_.poses_ = poses;
          joint_opt_.robot_frame_point_clouds_ = init_point_clouds;
          joint_opt_.robot_frame_normal_clouds_ = normal_clouds;
          joint_opt_.d3_covariances_ = covariances_;
          joint_opt_.human_constraints_ = human_constraints_;
                  
          joint_opt_.Run();

          poses = joint_opt_.poses_;
          ceres_gradients_ = joint_opt_.gradients_;
          ceres_jacobian_ = joint_opt_.ceres_jacobian_;
          jacobian_init_ = true;
          num_hc_residuals_ = joint_opt_.num_hc_residuals_;
          std::cout << "num hc residuals: " << num_hc_residuals_ << std::endl;
      
          ClearDrawingMessage(&display_message_);
          DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
          int final_hc_size = human_constraints_.size();
          added_human_constraints = final_hc_size - init_hc_size;
          std::cout << "added constraints: " << added_human_constraints << std::endl;



//         vector<float> cc = localization_.GetCeresCost();
//         CHECK_EQ(cc.size(), poses.size());
//         float total_cost = 0.0;
//         for (size_t i = 0; i < cc.size(); ++i) {
//           cout << "pose: " << i << "cost: " << cc[i] << endl;
//           total_cost += cc[i];
//         }
//         cout << "total cost: " << total_cost << endl;


//           prev_covariances_ = covariances_;
//           covariances_ = localization_.GetPoseCovariances();
          cout << "size of covariances_: " << covariances_.size() << endl;

          cout << "completed cycle" << endl;
        }
      
        current_replay_index++;
      }
      else {
        cout << "No more inputs to replay!" << endl;
      }
    }
  }
}








//const string& bag_file
void BatchLocalize(const string& keyframes_file,
                   int max_laser_poses, bool use_point_constraints,
                   double time_skip, bool return_initial_poses) {
  //vector<PointCloudf> point_clouds;
  //vector<NormalCloudf> normal_clouds;
  //vector<Pose2Df> poses;

  //cout << "STFS FILE: " << stfs_file << endl;

  clock_gettime(CLOCK_MONOTONIC, &ms1);

  if (debug_level_ > 0) {
    localization_options_.AnimateCeres = AnimateCeres;
    localization_options_.EvaluateConsistency2 = EvaluateConsistency2;
    localization_options_.CorrespondenceCallback = CorrespondenceCallback;
    // Reset localization_gui to correct map.
    cobot_msgs::CobotLocalizationMsg localization_msg_;
    localization_msg_.timeStamp = GetTimeSec();
    localization_msg_.map = kMapName;
    localization_msg_.x = 0;
    localization_msg_.y = 0;
    localization_msg_.angle = 0;
    localization_publisher_.publish(localization_msg_);
  }


  if (bag_file != NULL) {
    cout << "use vector_mapping instead" << endl;
    /*
    vector<double> timestamps;
    vector<double> keyframes;

    double bag_duration;
    //printf("Processing %s\n", bag_file.c_str());
    printf("Processing %s\n", bag_file);

em_input_
    LoadKeyframes(keyframes_file, &keyframes);
    LoadRosBag(bag_file,
               max_laser_poses,
               time_skip,
               keyframes,
               &init_point_clouds,
               &normal_clouds,
               &poses,
               &timestamps,
               &bag_duration);

    ClearDrawingMessage(&display_message_);
    DisplayPoses(poses, init_point_clouds, normal_clouds, NULL);

    const double t_start = GetTimeSec();
    localization_.BatchLocalize(localization_options_, kMapName,
                                init_point_clouds,
                                normal_clouds, (debug_level_ > 1),
                                return_initial_poses, &poses);

    covariances_ = localization_.GetPoseCovariances();
    cout << "size of covariances_: " << covariances_.size() << endl;

    const double process_time = GetTimeSec() - t_start;
    printf("Done in %.3fs, bag time %.3fs (%.3fx).\n",
           process_time, bag_duration, bag_duration / process_time);

    message_timestamps_ = timestamps;
    if (message_timestamps_.size() == 0) {
      cout << "timestamps was empty" << endl;
      message_timestamps_.push_back(0.0);
    }
    cout << "saving poses, stfs, covarainces" << endl;
    SaveLoggedPoses(string(bag_file) + ".poses", poses, message_timestamps_);
    CHECK_GT(message_timestamps_.size(), 0);
    SaveStfsandCovars(kMapName,
            poses,
            init_point_clouds,
            normal_clouds,
            string(bag_file),
            message_timestamps_.front());
    SaveStfs(kMapName,
            poses,
            init_point_clouds,
            normal_clouds,
            string(bag_file),
            message_timestamps_.front());
    */
  }
  else if (stfs_file != NULL) {
    double timestamp[1];
    timestamp[0] = 0.0;
    LoadStfsandCovars(stfs_file,
                      &init_point_clouds,
                      &normal_clouds,
                      &poses,
                      timestamp);
  }
  else if (log_file != NULL) {
    //load log file
    logged_input = LoadLogFile(log_file);
    string lf_path = string(log_file);
    string delimiter = "_logged_";

    size_t pos = 0;
    if ((pos = lf_path.find(delimiter)) != string::npos) {
      stfs_file = strdup((lf_path.substr(0, pos)+".bag.stfs.covars").c_str());
    }

    double timestamp[1];
    timestamp[0] = 0.0;
    LoadStfsandCovars(stfs_file,
                      &init_point_clouds,
                      &normal_clouds,
                      &poses,
                      timestamp);
  }

  prev_poses = poses;
  ClearDrawingMessage(&display_message_);
  cout << "should hit upon loading" << endl;
  DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);


  /*
  SaveLoggedPoses(string(bag_file) + ".poses", poses, timestamps);
  if (kStandardizedData ||
      test_set_index_ >= 0 ||
      statistical_test_index_ >= 0) {
    SaveResults(bag_file, poses, point_clouds, normal_clouds, kMapName);
  }
  if (save_image_file != NULL) {
    cobot_msgs::LocalizationGuiCaptureSrv::Request req;
    cobot_msgs::LocalizationGuiCaptureSrv::Response res;
    req.filename = string(save_image_file);
    printf("Saving image to %s\n", req.filename.c_str());
    gui_capture_client.call(req, res);
  }
  if (save_stfs_) {
    SaveStfs(kMapName,
             poses,
             point_clouds,
             normal_clouds,
             string(bag_file),
             timestamps.front());
  }*/

}

void LogActivity() {
  clock_gettime(CLOCK_MONOTONIC, &ms2);
  total_runtime =  (ms2.tv_sec - ms1.tv_sec) +
                  ((ms2.tv_nsec - ms1.tv_nsec) / 1000000000.00);
  cout << "total runtime: " << total_runtime << endl;
  if (log_file == NULL) {
    num_total_constraints = human_constraints_.size();

    std::stringstream ss;
    ss.str(bag_name);
    string item;
    vector<string> elems;
    while (getline(ss, item, '.')) {
      elems.push_back(item);
    }
    bag_name = elems[0];

    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    string year = std::to_string((now->tm_year + 1900));
    string mon = std::to_string((now->tm_mon + 1));
    string day = std::to_string(now->tm_mday);
    string hour = std::to_string(now->tm_hour);
    string min = std::to_string(now->tm_min);
    string sec = std::to_string(now->tm_sec);

    string log_date = year +"-"+ mon +"-"+ day +"-"+ hour +"-"+ min +"-"+ sec;
    string log_name = bag_name + "_logged_" + log_date + ".log";

    ScopedFile fid(log_name, "w");
    fprintf(fid(), "%.4f \n", total_runtime);
    fprintf(fid(), "%d \n", num_completed_cycles);
    fprintf(fid(), "%d \n", num_total_constraints);
    for (size_t i = 0; i < input_history.size(); ++i) {
      fprintf(fid(), "%d, %d\n", input_history[i].type_of_constraint,
                                input_history[i].undone);
      cout << "writing undone status: " << input_history[i].undone << endl;
      for (size_t j = 0; j < input_history[i].input_points.size(); ++j) {
        fprintf(fid(), "%.4f, %.4f\n", input_history[i].input_points[j](0),
                                     input_history[i].input_points[j](1));
      }
    }
  }
}

//Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  if (log_file == NULL) {
    LogActivity();
  }
  printf("\nTerminating.\n");
  exit(0);
}

bool IsValidCorrectionType(const CorrectionType& type) {
  if (type == CorrectionType::kPointCorrection ||
      type == CorrectionType::kLineSegmentCorrection ||
      type == CorrectionType::kCornerCorrection ||
      type == CorrectionType::kColinearCorrection ||
      type == CorrectionType::kPerpendicularCorrection ||
      type == CorrectionType::kParallelCorrection) {
    return true;
  }
  return false;
}

void AddCorrectionPoints(const cobot_msgs::GuiMouseClickEvent& msg) {
  const CorrectionType correction_input_type =
      static_cast<CorrectionType>(msg.modifiers);
  const Vector2f p1(msg.mouse_down.x, msg.mouse_down.y);
  const Vector2f p2(msg.mouse_up.x, msg.mouse_up.y);
  if (correction_input_type != pending_correction_type_ &&
      IsValidCorrectionType(correction_input_type)) {
    const auto name_index = static_cast<size_t>(correction_input_type);
    cout << "Correction mode: "
         << CorrectionTypeNames[name_index]
         << endl;
    // Start a new correction type.
    selected_points_.clear();
    // All corrections need the first point.
    selected_points_.push_back(p1);
    if (correction_input_type != CorrectionType::kPointCorrection) {
      // Except for the point correction, all other corrections require
      // the second point.
      selected_points_.push_back(p2);
    }
    pending_correction_type_ = correction_input_type;
    correction_type_ = CorrectionType::kUnknownCorrection;
  } else {
    switch (correction_input_type) {
      case CorrectionType::kPointCorrection : {
        cout << "Points" << endl;
        if (correction_type_ == CorrectionType::kUnknownCorrection) {
          selected_points_.push_back(p1);
          correction_type_ = CorrectionType::kPointCorrection;
          pending_correction_type_ = CorrectionType::kUnknownCorrection;
        }
      } break;
      case CorrectionType::kLineSegmentCorrection : {
        cout << "Lines" << endl;
        if (correction_type_ == CorrectionType::kUnknownCorrection) {
          selected_points_.push_back(p1);
          selected_points_.push_back(p2);
          correction_type_ = CorrectionType::kLineSegmentCorrection;
          pending_correction_type_ = CorrectionType::kUnknownCorrection;
        }
      } break;
      case CorrectionType::kCornerCorrection : {
        cout << "Corners" << endl;
        if (pending_correction_type_ == CorrectionType::kCornerCorrection &&
            selected_points_.size() < 6) {
          selected_points_.push_back(p1);
          selected_points_.push_back(p2);
        } else if (correction_type_ == CorrectionType::kUnknownCorrection) {
          selected_points_.push_back(p1);
          selected_points_.push_back(p2);
          correction_type_ = CorrectionType::kCornerCorrection;
          pending_correction_type_ = CorrectionType::kUnknownCorrection;
        }
      } break;
      case CorrectionType::kColinearCorrection : {
        cout << "Colinear" << endl;
        if (correction_type_ == CorrectionType::kUnknownCorrection) {
          selected_points_.push_back(p1);
          selected_points_.push_back(p2);
          correction_type_ = CorrectionType::kColinearCorrection;
          pending_correction_type_ = CorrectionType::kUnknownCorrection;
        }
      } break;
      case CorrectionType::kPerpendicularCorrection : {
        cout << "Perpendicular" << endl;
        if (correction_type_ == CorrectionType::kUnknownCorrection) {
          selected_points_.push_back(p1);
          selected_points_.push_back(p2);
          correction_type_ = CorrectionType::kPerpendicularCorrection;
          pending_correction_type_ = CorrectionType::kUnknownCorrection;
        }
      } break;
      case CorrectionType::kParallelCorrection : {
        cout << "Parallel" << endl;
        if (correction_type_ == CorrectionType::kUnknownCorrection) {
          selected_points_.push_back(p1);
          selected_points_.push_back(p2);
          correction_type_ = CorrectionType::kParallelCorrection;
          pending_correction_type_ = CorrectionType::kUnknownCorrection;
        }
      } break;
      default : {
        // Do nothing.
      } break;
    }
  }
}

size_t VerifyUserInput(vector<PointCloudf>* temp_point_clouds) {
  size_t points_verified = 0;
  float local_select_thresh = 0.05;
  bool seen;
  for (size_t i=0; i<selected_points_.size(); i++) {
    seen = false;
    for (size_t j=0; j<temp_point_clouds->size(); j++) {
      for (size_t k=0; k<temp_point_clouds[0][j].size(); k++) {
        if ((temp_point_clouds[0][j][k] - selected_points_[i]).norm() <
              local_select_thresh) {
          points_verified ++;
          seen = true;
          break;
        }
      }
      if (seen) {
        break;
      }
    }
  }

  if (selected_points_[0] == selected_points_[1] ||
      selected_points_[2] == selected_points_[3]) {
    points_verified = 0;
  }
  return points_verified;
}



bool MouseDragged(const cobot_msgs::GuiMouseClickEvent& msg) {
  const int sq_distance_moved =
      sqrt(msg.mouse_down.x - msg.mouse_up.x) +
      sqrt(msg.mouse_down.y - msg.mouse_up.y);
  return (sq_distance_moved > 1.0);
}

void ResetCorrectionInputs() {
  selected_points_.clear();
  pending_correction_type_ = CorrectionType::kUnknownCorrection;
  correction_type_ = CorrectionType::kUnknownCorrection;
}

void KeyboardRequestCallback(const cobot_msgs::GuiKeyboardEvent& msg) {
  //cout << "key code: " << msg.keycode << endl;
  if (msg.keycode == 0x55) { //key code 85, 'u' for undo
    cout << "undo" << endl;
    cout << (log_file == NULL) << endl;
    if (log_file == NULL) {
      poses = prev_poses;
      covariances_ = prev_covariances_;
      input_history.back().undone = 1;
//       cout << (added_human_constraints > 0) << endl;
      if (added_human_constraints > 0) {
        cout << "removing constraints" << endl;
        for (int i = 0; i < added_human_constraints; ++i) {
          human_constraints_.pop_back();
        }
        added_human_constraints = 0;
      }
      ClearDrawingMessage(&display_message_);
      DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
    }
    else {
      cout << "undo not allowed in log replay mode." << endl;
    }
  }
  else if (msg.keycode == 0x56) { //key code 86, 'v' for save
    cout << "Are you sure you want to save? (y/n)" << endl;
    string s;
    std::cin >> s;
    if (s == "y") {
      cout << "time to save!" << endl;
      message_timestamps_.push_back(0.0);
      CHECK_GT(message_timestamps_.size(), 0);
    //SaveLoggedPoses(string(bag_file) + ".poses", poses, message_timestamps_);
      SaveLoggedPoses("test.poses", poses, message_timestamps_);
      if (save_stfs_) {
        SaveStfsandCovars(kMapName,
                poses,
                init_point_clouds,
                normal_clouds,
                "test.poses",
                message_timestamps_.front());
        SaveStfs(kMapName,
                poses,
                init_point_clouds,
                normal_clouds,
                "test.poses",
                message_timestamps_.front());
      }
      cout << "time to ctrl-C!" << endl;
    }
  }
  else if (msg.keycode == 0x4C) { //key code 76, 'l' for log
    cout << "step ahead log" << endl;
//     cout << "not currently available" << endl;
    ReplayLog();
  }
}






void MouseMoveCallback(const cobot_msgs::GuiMouseMoveEvent& msg) {
  //ClearDrawingMessage(&display_message_);
  //DrawCircle(msg.location,
  //           cobot_gui::kColorRed,
   //          &display_message_);
  //PublishDisplay();
}


















void LoopCloseCallback(const cobot_msgs::GuiMouseClickEvent& msg) {
  //static const bool kDebug = false;
  if (msg.modifiers == 0x06) {
    loop_corrections_on_ = !loop_corrections_on_;
    cout << "Loop corrections: " << loop_corrections_on_ << endl;
  }

  if (loop_corrections_on_) {
    AddCorrectionPoints(msg);
  }

  else if (correction_type_ != CorrectionType::kUnknownCorrection) {
    // Not in corrections mode, and there is a valid correction type.
    vector<PointCloudf> temp_point_clouds = init_point_clouds;

    CHECK_EQ(temp_point_clouds.size(), poses.size());
    CHECK_EQ(temp_point_clouds.size(), init_point_clouds.size());

    for (size_t i=0; i<init_point_clouds.size(); i++) {
      const Rotation2Df rotation(poses[i].angle);
      const Translation2f translation(poses[i].translation);
      const Affine2f pose_transform = translation * rotation;
      for (size_t j=0; j < init_point_clouds[i].size(); j++) {
        temp_point_clouds[i][j] = pose_transform * init_point_clouds[i][j];
      }
    }

    //NOTE: temp_point_clouds = world frame
    //NOTE: init_point_clouds = robot frame
    //NOTE: normal_clouds = robot frame
    
    size_t points_verified = VerifyUserInput(&temp_point_clouds);
    cout << "points verified: " << points_verified << endl;

    if (points_verified == selected_points_.size()) {

      //TODO: move to seperate function
      std::vector<std::vector<Eigen::Vector2f>> world_frame_point_clouds;
      for (size_t i = 0; i < init_point_clouds.size(); ++i) {
        const Rotation2Df pose_rotation(poses[i].angle);
        const Affine2f pose_transform = Translation2f(poses[i].translation) * pose_rotation;
        std::vector<Eigen::Vector2f> single_point_cloud;
        for (unsigned int j = 0; j < init_point_clouds[i].size(); ++j) {
          const Vector2f p = pose_transform * init_point_clouds[i][j];
          single_point_cloud.push_back(p);
        }
        world_frame_point_clouds.push_back(single_point_cloud);
      }
      
      
      
      //NOTE: maybe use this for evaluation.... maybe
      //if (localization_options_.EvaluateConsistency2 != NULL) {
      //  localization_options_.EvaluateConsistency2(poses, init_point_clouds);
      //}

      
      
      
      prev_poses = poses;
      prev_covariances_ = covariances_;
      //bool debug = false;
      int init_hc_size = human_constraints_.size();
      SingleInput current_input;
      current_input.type_of_constraint = correction_type_;
      current_input.input_points = selected_points_;
      current_input.undone = 0;
      input_history.push_back(current_input);
      printf("correction type: %d\n", input_history.back().type_of_constraint);

      
      
      
//TODO: change these to call constructor with all relavent args instead of assigning one by one 
      
      em_input_.local_version_point_clouds_ = world_frame_point_clouds;
      em_input_.selected_points_ = selected_points_;
      em_input_.correction_type_ = correction_type_;
      
      em_input_.Run();
      
      std::vector<int> corrected_poses = em_input_.corrected_poses_;
      std::vector<int> anchor_poses = em_input_.anchor_poses_;
      std::pair<int, int> backprop_bounds = em_input_.backprop_bounds_;
      std::vector<Eigen::Vector2f> new_selected_points = em_input_.selected_points_;

//       ClearDrawingMessage(&display_message_);
//       DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
      if (backprop_bounds.first >= 0 && backprop_bounds.second >= 1) {
        app_exp_corr_.correction_type_ = correction_type_;
        app_exp_corr_.selected_points_ = new_selected_points;
        app_exp_corr_.corrected_poses_ = corrected_poses;
        app_exp_corr_.anchor_poses_ = anchor_poses;
        app_exp_corr_.poses_ = poses;
     
        app_exp_corr_.Run();

        poses = app_exp_corr_.poses_;
        Eigen::Vector3f correction = app_exp_corr_.correction_;
        std::vector<VectorMapping::HumanConstraint> new_human_constraints;
        new_human_constraints = app_exp_corr_.new_human_constraints_;
        human_constraints_.push_back(new_human_constraints);
      
  //       ClearDrawingMessage(&display_message_);
  //       DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
        //std::cout << "BP bounds: " << backprop_bounds.first << ", " << backprop_bounds.second << std::endl;
      
        backprop_.poses_ = poses;
        backprop_.correction_ = correction;
        backprop_.backprop_bounds_ = backprop_bounds;
        backprop_.d3_covariances_ = covariances_;
      
        backprop_.Run();
      
        covariances_ = backprop_.d3_covariances_;
        poses = backprop_.poses_;

  //       ClearDrawingMessage(&display_message_);
  //       DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
        ceres_gradients_.clear();
        jacobian_init_ = false;
        
        for (size_t i = 0; i < poses.size(); ++i) { 
          float angle = poses[i].angle;
          //std::cout << angle << " ::: " << atan2(sin(angle), cos(angle)) << std::endl;
	  poses[i].angle = atan2(sin(angle), cos(angle));
        }
        
        //joint_opt_.orig_odom_ = orig_odom_;
        joint_opt_.poses_ = poses;
        joint_opt_.robot_frame_point_clouds_ = init_point_clouds;
        joint_opt_.robot_frame_normal_clouds_ = normal_clouds;
        joint_opt_.d3_covariances_ = covariances_;
        joint_opt_.human_constraints_ = human_constraints_;
                  
        joint_opt_.Run();

        poses = joint_opt_.poses_;
        ceres_gradients_ = joint_opt_.gradients_;
        ceres_jacobian_ = joint_opt_.ceres_jacobian_;
        jacobian_init_ = true;
        num_hc_residuals_ = joint_opt_.num_hc_residuals_;
	std::cout << "num hc residuals: " << num_hc_residuals_ << std::endl;
      
        ClearDrawingMessage(&display_message_);
        DisplayPoses(poses, init_point_clouds, normal_clouds, save_image_file);
      
        int final_hc_size = human_constraints_.size();
        added_human_constraints = final_hc_size - init_hc_size;
        std::cout << "added constraints: " << added_human_constraints << std::endl;
      }
    }
    
    cout << "completed cycle" << endl;
    num_completed_cycles++;

    ResetCorrectionInputs();
  }
  else {
    ResetCorrectionInputs();
  }

  // display_message_.text.clear();
  // display_message_.text_col.clear();
  // display_message_.text_height.clear();
  // display_message_.text_in_window_coords.clear();
  // display_message_.text_x.clear();
  // display_message_.text_y.clear();
  // DrawText(Vector2f(0, 0),
  //          status_string,
  //          cobot_gui::kColorBlack,
  //          11,
  //          true,
  //          &display_message_);
  // display_publisher_.publish(display_message_);
}

















// Signal handler bad signals. Print a backtrace and quit.
void HandleTerminalError(int i) {
  printf("\nSignal intercepted, exiting.\n");
  PrintBackTrace();
  exit(0);
}

int main(int argc, char** argv) {
  //InitHandleStop(&run_, 0);
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);
  signal(SIGABRT, HandleTerminalError);
  signal(SIGSEGV, HandleTerminalError);

  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();

  char* keyframes_file = NULL;
  int max_laser_poses = -1;
  bool disable_stfs = false;
  double time_skip = 0;
  bool unique_node_name = false;
  bool return_initial_poses = false;

  static struct poptOption options[] = {
    { "debug" , 'd', POPT_ARG_INT, &debug_level_, 1, "Debug level", "NUM" },
    { "bag-file", 'b', POPT_ARG_STRING, &bag_file, 1, "ROS bagfile to use",
        "STRING"},
    { "max-poses" , 'n', POPT_ARG_INT, &max_laser_poses, 1,
        "Maximum number of laser poses to optimize", "NUM" },
    { "time-skip", 's', POPT_ARG_DOUBLE, &time_skip, 1,
      "Time to skip from the bag file", "NUM" },
    { "standardized", 'S', POPT_ARG_NONE, &kStandardizedData, 0,
        "Load standardized data bag file.", "NONE" },
    { "test-set", 't', POPT_ARG_INT, &test_set_index_, 1,
        "Test set index", "NUM" },
    { "statistical-test", 'T', POPT_ARG_INT, &statistical_test_index_, 1,
        "Statistical test index", "NUM" },
    { "noise", 'N', POPT_ARG_DOUBLE, &odometry_additive_noise_, 1,
        "Statistical test additive random noise", "NUM" },
    { "keyframes", 'k', POPT_ARG_STRING, &keyframes_file, 1,
        "Keyframes file", "STRING" },
    { "image", 'I', POPT_ARG_STRING, &save_image_file, 1,
        "Image file", "STRING" },
    { "episode-images", 'e', POPT_ARG_STRING, &episode_images_path, 1,
        "Epidode images path", "STRING" },
    { "unique_node_name", 'u', POPT_ARG_NONE, &unique_node_name, 0,
        "Use unique ROS node name", "NONE" },
    { "initial_poses", 'i', POPT_ARG_NONE, &return_initial_poses, 0,
        "Return intial, instead of final, pose estimates", "NONE" },
    { "object_clustering", 'o', POPT_ARG_NONE, &save_stfs_, 0,
        "Do object clustering", "NONE"},
    { "disable-stfs", 'p', POPT_ARG_NONE, &disable_stfs, 0,
        "Disable STFs", "NONE"},
    { "save-ltfs", 'l', POPT_ARG_NONE, &save_ltfs_, 0,
        "Save LTFs", "NONE"},
    { "use-kinect", 'K', POPT_ARG_NONE, &use_kinect_, 0,
        "Use Kinect", "NONE"},
    { "load-stfs", 'L', POPT_ARG_STRING, &stfs_file, 1,
        "Load existing stfs file", "STRING"},
    { "load-recording", 'R', POPT_ARG_STRING, &log_file, 1,
        "Load existing log file", "STRING"},
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0){
  }
  config_.init(watch_files_);
  config_.addFile("../robot.cfg");
  config_.addFile("config/vector_mapping.cfg");
  config_.addFile("config/localization_parameters.cfg");
  CHECK(LoadConfiguration(&localization_options_));

  // if (bag_file == NULL) unique_node_name = true;
  const bool running_tests =
      (test_set_index_ >= 0 || statistical_test_index_ >= 0);
  if (running_tests) {
    const unsigned int seed = time(NULL);
    srand(seed);
    printf("Seeding with %u test=%.5f\n", seed, randn(1.0f));
  }
  const string node_name =
      (running_tests || unique_node_name) ?
      StringPrintf("VectorMapping_%lu",
                   static_cast<uint64_t>(GetTimeSec() * 1000000.0)) :
      string("VectorMapping");
  ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  ros::NodeHandle ros_node;
  display_publisher_ =
      ros_node.advertise<cobot_msgs::LidarDisplayMsg>(
      "Cobot/VectorLocalization/Gui",1,true);
  localization_publisher_ =
      ros_node.advertise<cobot_msgs::CobotLocalizationMsg>(
      "Cobot/Localization", 1, true);
  gui_capture_client =
      ros_node.serviceClient<cobot_msgs::LocalizationGuiCaptureSrv>(
      "VectorLocalization/Capture");

  mouse_click_subscriber_ =
      ros_node.subscribe("Cobot/VectorLocalization/GuiMouseClickEvents",
                         1,
                         LoopCloseCallback);
  mouse_move_subscriber_ =
      ros_node.subscribe("Cobot/VectorLocalization/GuiMouseMoveEvents",
                         1,
                         MouseMoveCallback);
  keyboard_event_subscriber_ = ros_node.subscribe(
                               "Cobot/VectorLocalization/GuiKeyboardEvents",
                               1, KeyboardRequestCallback);

  LoadLaserCorrections();
  if (stfs_file == NULL && bag_file == NULL && log_file == NULL) {
    fprintf(stderr,
        "ERROR: Must Specify bagfile (-b), STFs file (-L), or log file (-R)\n");
    return 1;
  }
  const string keyframes = (keyframes_file == NULL) ? "" : keyframes_file;

  if (stfs_file != NULL) {
    //TODO: move this to the right place
    //LoadOdom();
    
    cout << "got stfs" << endl;
    //bag_file = stfs_file;
    BatchLocalize(keyframes, max_laser_poses,
                  !disable_stfs, time_skip, return_initial_poses);
    bag_name = stfs_file;
  }
  else if (bag_file != NULL) {
    cout << "got bag" << endl;
    BatchLocalize(keyframes, max_laser_poses,
                  !disable_stfs, time_skip, return_initial_poses);
    bag_name = bag_file;
  }
  else if (log_file != NULL) {
    cout << "got log file" << endl;
    BatchLocalize(keyframes, max_laser_poses,
                  !disable_stfs, time_skip, return_initial_poses);
  }

  ros::spin();

  return 0;
}
