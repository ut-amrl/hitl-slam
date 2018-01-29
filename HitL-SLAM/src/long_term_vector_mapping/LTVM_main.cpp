#include <algorithm>
#include <cmath>
#include <dirent.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <Eigen/Sparse>
#include <eigen3/Eigen/Geometry>
#include <glog/logging.h>
#include <list>
#include <map>
#include <ros/package.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <termios.h>
#include <utility>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdlib.h>

#include "CImg/CImg.h"
#include "cobot_msgs/CobotLocalizationMsg.h"
#include "cobot_msgs/LidarDisplayMsg.h"
#include "cobot_msgs/LocalizationGuiCaptureSrv.h"
#include "configreader.h"
#include "connected_components/connected_components.h"
#include "eigen_helper.h"
#include "gui_publisher_helper.h"
#include "helpers.h"
#include "kdtree.h"
#include "maximal_cliques/graph.hpp"
#include "maximal_cliques/maximalcliquegenerator.h"
#include "perception_tools/perception_2d.h"
#include "object_map.h"
#include "openmp_utils.h"
#include "popt_pp.h"
#include "timer.h"
#include "glog/logging.h"
#include "ceres/ceres.h"

#include "CImg/CImg.h"
#include "png.h"

#define cimg_use_png


using cobot_gui::DrawLine;
using cobot_gui::DrawPoint;
using cobot_gui::DrawText;
using cobot_gui::ClearDrawingMessage;
using connected_components::compute_scc;
using connected_components::get_largest_components;
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
using EnmlMaps::GridObject;
using EnmlMaps::LoadPersistentObjects;
using EnmlMaps::SavePersistentObjects;
using EnmlMaps::PersistentObject;
using ros::ServiceServer;
using ros::Subscriber;
using std::fwrite;
using std::fprintf;
using std::list;
using std::make_pair;
using std::map;
using std::pair;
using std::size_t;
using std::sort;
using std::string;
using std::vector;
using perception_2d::NormalCloudf;
using perception_2d::Pose2Df;
using perception_2d::PointCloudf;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::Covariance;
using namespace cimg_library;

const double PI = M_PI;
// Display message for drawing debug vizualizations on the localization_gui.
cobot_msgs::LidarDisplayMsg display_message_;

// Service client to ask localization_gui to save images.
ros::ServiceClient gui_capture_client;

// ROS publisher to publish display_message_ to the ROS topic
// Cobot/VectorLocalization/Gui
ros::Publisher display_publisher_;

// ROS publisher to publish the latest robot localization estimate to
// Cobot/Localization
ros::Publisher localization_publisher_;

// The path of the cobot_linux ROS stack.
static const string kCobotStackPath(ros::package::getPath("cobot_linux"));

// Visualization colour of points belonging to STFs.
// const uint32_t kStfPointColor = 0x1F994CD9;
const uint32_t kStfPointColor = 0xFFFF0000;

// The timestamp of the current log, in epoch time.
double log_timestamp_ = 0.0;

// List of persistent objects.
vector<PersistentObject> persistent_objects_;

static const bool kUseKlDivergence = false;
static const bool kUseOverlap = false;
static const bool kUseNewMetric = true;

EnmlMaps::ObjectMapOptions map_options_;

bool LoadConfiguration() {
  WatchFiles watch_files;
  ConfigReader config((kCobotStackPath + "/").c_str());

  config.init(watch_files);
  config.addFile("../robot.cfg");
  config.addFile("config/non_markov_localization.cfg");
  if (!config.readFiles()) return false;
  bool error = false;
  ConfigReader::SubTree c(config,"ProbabilisticObjectMaps");
  error = error || !c.getReal("object_distance_threshold",
                              map_options_.object_distance_threshold);
  error = error || !c.getUInt("min_object_points",
                              map_options_.min_object_points);
  error = error || !c.getReal("laser_angular_resolution",
                              map_options_.laser_angular_resolution);
  error = error || !c.getReal("image_resolution",
                              map_options_.image_resolution);
  error = error || !c.getReal("image_border",
                              map_options_.image_border);
  error = error || !c.getReal("min_sdf_value",
                              map_options_.min_sdf_value);
  error = error || !c.getReal("max_sdf_value",
                              map_options_.max_sdf_value);
  error = error || !c.getReal("min_sdf_weight",
                              map_options_.min_sdf_weight);
  error = error || !c.getReal("max_sdf_weight",
                              map_options_.max_sdf_weight);
  error = error || !c.getReal("min_sdf_inference_weight",
                              map_options_.min_sdf_inference_weight);
  error = error || !c.getBool("generate_sdf",
                              map_options_.generate_sdf);
  error = error || !c.getReal("matching_angular_resolution",
                              map_options_.matching_angular_resolution);
  error = error || !c.getReal("matching_loc_resolution",
                              map_options_.matching_loc_resolution);
  error = error || !c.getReal("laser_std_dev",
                              map_options_.laser_std_dev);
  return (!error);
}

// Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
}

void LoadStfs(const string& stfs_file, PointCloudf* point_cloud_ptr,
    NormalCloudf* normal_cloud_ptr, vector<Pose2Df>* poses_ptr, double* timestamp) {
  PointCloudf& point_cloud = *point_cloud_ptr;
  NormalCloudf& normal_cloud = *normal_cloud_ptr;
  vector<Pose2Df>& poses = *poses_ptr;
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

  int ss_count = 0;
  while (fscanf(fid(), "%f,%f,%f, %f,%f, %f,%f\n",
    &(pose.translation.x()), &(pose.translation.y()), &(pose.angle),
    &(point.x()), &(point.y()), &(normal.x()), &(normal.y())) == kNumFields) {
    double x2 = (pose.translation.x() - point.x())*(pose.translation.x() - point.x());
    double y2 = (pose.translation.y() - point.y())*(pose.translation.y() - point.y());
    double d = sqrt(x2 + y2);
    if (d > 0.8) { //if not sensing self
      point_cloud.push_back(point);
      normal_cloud.push_back(normal);
      poses.push_back(pose);
    }
    else {
      ss_count ++;
    }
  }
}

/* ################################ RUN / TEST ################################# */

void writeSDFObject(CImg<float> weights, CImg<float> values, Vector2f origin) {
  std::ofstream file;
  file.open("latest_weights.txt");
  file << weights.width() << " " << weights.height() << std::endl;
  file << origin(0) << " " << origin(1) << std::endl;
  int index = 0;
  for (int x=0; x < weights.width(); x++) {
    for (int y=0; y < weights.height(); y++) {
      file << weights(x,y) << std::endl;
      index++;
    }
  }
  file.close();
  file.clear();
  file.open("latest_values.txt");
  file << values.width() << " " << values.height() << std::endl;
  file << origin(0) << " " << origin(1) << std::endl;
  for (int x=0; x < values.width(); x++) {
    for (int y=0; y < values.height(); y++) {
      file << values(x,y) << std::endl;
    }
  }
  file.close();
}


SdfObject loadSDFObject() {
  CImg<float> weights;
  CImg<float> values;
  Vector2f origin;

  std::ifstream infile;
  infile.open("latest_weights.txt", std::ios_base::in);

  if (!infile)  {
    std::cout << "Can't open SDF weights file!\n";
  }
  int line_num = 1;
  int w, h;
  float O_x, O_y, I;
  int index = 0;
  for(std::string line; std::getline(infile, line); ) {
    std::istringstream in(line);
    if (line_num == 1) {
      in >> w >> h;
      CImg<float> temp(w,h);
      weights = temp;
    }
    else if (line_num == 2) {
      in >> O_x >> O_y;
      origin(0) = O_x;
      origin(1) = O_y;
    }
    else {
      in >> I;
      weights(index/h,index%h) = I;
      index++;
    }
    line_num++;
  }
  infile.close();
  infile.clear();

  line_num = 1;
  index = 0;

  infile.open("latest_values.txt", std::ios_base::in);

  if (!infile)  {
    std::cout << "Can't open SDF values file!\n";
  }

  for(std::string line; std::getline(infile, line); ) {
    std::istringstream in(line);
    if (line_num == 1) {
      in >> w >> h;
      CImg<float> temp(w,h);
      values = temp;
    }
    else if (line_num == 2) {
      in >> O_x >> O_y;
      origin(0) = O_x;
      origin(1) = O_y;
    }
    else {
      in >> I;
      values(index/h,index%h) = I;
      index++;
    }
    line_num++;
  }

  SdfObject sdf;
  sdf.origin = origin;
  sdf.weights = weights;
  sdf.values = values;
  return sdf;
}


void writePoints(vector<Pose2Df> poses, vector<Vector2f> point_cloud, NormalCloudf normal_cloud) {
  std::ofstream file;
  file.open("latest_points.txt", std::ios::app);
  for (size_t i=0; i < point_cloud.size(); i++) {
    file << point_cloud[i](0) << " " << point_cloud[i](1) << " " <<
            poses[i].translation(0) << " " << poses[i].translation(1) << " " <<
            normal_cloud[i](0) << " " << normal_cloud[i](1) << std::endl;
  }
  file.close();
}

totalPointCloud loadPoints() {
  vector<Vector2f> pc;
  vector<Pose2Df> poses;
  NormalCloudf normals;
  Vector2f point;
  Pose2Df pose;
  Vector2f normal;

  std::ifstream infile;
  infile.open("latest_points.txt", std::ios_base::in);

  if (!infile)  {
    std::cout << "Can't open file!\n";
  }

  float x, y, px, py, nx, ny;
  for(std::string line; std::getline(infile, line); ) {
    std::istringstream in(line);
    in >> x >> y >> px >> py >> nx >> ny;
    point(0) = x;
    point(1) = y;
    pose.translation(0) = px;
    pose.translation(1) = py;
    normal(0) = nx;
    normal(1) = ny;
    pc.push_back(point);
    poses.push_back(pose);
    normals.push_back(normal);
  }
  totalPointCloud filtered_rpc;
  filtered_rpc.point_cloud = pc;
  filtered_rpc.poses = poses;
  filtered_rpc.normal_cloud = normals;

  return filtered_rpc;
}


worldMap BuildObjectsMap(const string& stfs_file, SdfObject master_sdf, vector<mappingVector> master_vectorMap,
  			 	int numruns, string map_name, int mode) {
  Matrix2f R;
  R << 1.0, 0.0, 0.0, 1.0;
  Vector2f T(0.0, 0.0);

  Matrix2f R11;
  R11 << 1.0, 0.0, 0.0, 1.0;
  Vector2f T11(0.0, 0.0);

  //AMRL
  if (map_name == "AMRL") {
    Matrix2f R12;
    R12 << 0.9942, -0.1076, 0.1076, 0.9942;
    Vector2f T12(0.1960, -0.1719);

    Matrix2f R13;
    R13 << 0.9999, -0.0105, 0.0105, 0.9999;
    Vector2f T13(-0.0657, -0.0314);

    Matrix2f R14;
    R14 << 1.0000, -0.0059, 0.0059, 1.0000;
    Vector2f T14(-0.0505, -0.1826);

    Matrix2f R15;
    R15 << 1.0000, 0.0071, -0.0071, 1.0000;
    Vector2f T15(-0.0964, -0.1856);

    Matrix2f R16;
    R16 << 0.9941, -0.1082, 0.1082, 0.9941;
    Vector2f T16(0.0431, -0.1091);

    Matrix2f R17;
    R17 << 0.9965, -0.0841, 0.0841, 0.9965;
    Vector2f T17(0.2881, 0.0102);

    Matrix2f R18;
    R18 << 0.9998, -0.0196, 0.0196, 0.9998;
    Vector2f T18(0.1225, -0.0530);

    if (numruns == 1) { R = R11; T = T11; }
    if (numruns == 2) { R = R12; T = T12; }
    if (numruns == 3) { R = R13; T = T13; }
    if (numruns == 4) { R = R14; T = T14; }
    if (numruns == 5) { R = R15; T = T15; }
    if (numruns == 6) { R = R16; T = T16; }
    if (numruns == 7) { R = R17; T = T17; }
    if (numruns == 8) { R = R18; T = T18; }
  }

  //MIT
  if (map_name == "MIT") {
    Matrix2f R12;
    R12 << 0.9101, -0.4143, 0.4143, 0.9101;
    Vector2f T12(-0.2030, -0.6528);

    Matrix2f R13;
    R13 << 0.9899, -0.1417, 0.1417, 0.9899;
    Vector2f T13(0.0988, 0.0550);

    Matrix2f R14;
    R14 << 0.9496, -0.3134, 0.3134, 0.9496;
    Vector2f T14(-0.1134, -0.7775);

    Matrix2f R15;
    R15 << 0.9377, -0.3475, 0.3475, 0.9377;
    Vector2f T15(-1.1683, 0.2163);

    Matrix2f R16;
    R16 << 0.9246, -0.3809, 0.3809, 0.9246;
    Vector2f T16(0.0283, -1.0189);

    Matrix2f R17;
    R17 << 0.9791, -0.2035, 0.2035, 0.9791;
    Vector2f T17(-1.3895, -0.8715);

    Matrix2f R18;
    R18 << 0.7680, -0.6404, 0.6404, 0.7680;
    Vector2f T18(-0.5580, -0.1149);

    Matrix2f R19;
    R19 << 0.9747, -0.2236, 0.2236, 0.9747;
    Vector2f T19(-0.2463, -0.7788);

    Matrix2f R110;
    R110 << 0.9250, -0.3800, 0.3800, 0.9250;
    Vector2f T110(-0.2266, -1.6836);

    Matrix2f R111;
    R111 << 0.9891, -0.1470, 0.1470, 0.9891;
    Vector2f T111(-0.9431, -0.3359);

    Matrix2f R112;
    R112 << 0.9235, -0.3837, 0.3837, 0.9235;
    Vector2f T112(0.0128, -1.2214);

    Matrix2f R113;
    R113 << 0.9838, -0.1793, 0.1793, 0.9838;
    Vector2f T113(-0.4399, -1.0695);

    Matrix2f R114;
    R114 << 0.9105, -0.4134, 0.4134, 0.9105;
    Vector2f T114(0.2907, -0.9064);

    Matrix2f R115;
    R115 << 0.9579, -0.2870, 0.2870, 0.9579;
    Vector2f T115(-0.1549, -0.1690);

    Matrix2f R116;
    R116 << 0.9127, -0.4087, 0.4087, 0.9127;
    Vector2f T116(-0.9010, -1.3718);

    Matrix2f R117;
    R117 << 0.9903, 0.1387, -0.1387, 0.9903;
    Vector2f T117(-1.1777, -0.1948);

    Matrix2f R118;
    R118 << 0.8162, -0.5778, 0.5778, 0.8162;
    Vector2f T118(0.3676, -2.0030);

    Matrix2f R119;
    R119 << 0.9969, 0.0789, -0.0789, 0.9969;
    Vector2f T119(-1.6137, 0.9628);

    Matrix2f R120;
    R120 << 0.9447, -0.3278, 0.3278, 0.9447;
    Vector2f T120(-0.2060, -1.4809);

    if (numruns == 1) { R = R11; T = T11; }
    if (numruns == 2) { R = R12; T = T12; }
    if (numruns == 3) { R = R13; T = T13; }
    if (numruns == 4) { R = R14; T = T14; }
    if (numruns == 5) { R = R15; T = T15; }
    if (numruns == 6) { R = R16; T = T16; }
    if (numruns == 7) { R = R17; T = T17; }
    if (numruns == 8) { R = R18; T = T18; }
    if (numruns == 9) { R = R19; T = T19; }
    if (numruns == 10) { R = R110; T = T110; }
    if (numruns == 11) { R = R111; T = T111; }
    if (numruns == 12) { R = R112; T = T112; }
    if (numruns == 13) { R = R113; T = T113; }
    if (numruns == 14) { R = R114; T = T114; }
    if (numruns == 15) { R = R115; T = T115; }
    if (numruns == 16) { R = R116; T = T116; }
    if (numruns == 17) { R = R117; T = T117; }
    if (numruns == 18) { R = R118; T = T118; }
    if (numruns == 19) { R = R119; T = T119; }
    if (numruns == 20) { R = R120; T = T120; }
  }

  //wall
  if (map_name == "WALL") {
    Matrix2f R12;
    R12 << 0.9994, 0.0357, -0.0357, 0.9994;
    Vector2f T12(-0.1203, 0.0264);

    Matrix2f R13;
    R13 << 1.0000, 0.0043, -0.0043, 1.0000;
    Vector2f T13(0.0035, -0.0160);

    Matrix2f R14;
    R14 << 0.9975, 0.0707, -0.0707, 0.9975;
    Vector2f T14(0.0184, -0.0303);

    Matrix2f R15;
    R15 << 0.9904, 0.1382, -0.1382, 0.9904;
    Vector2f T15(0.1184, -0.0086);

    if (numruns == 1) { R = R11; T = T11; }
    if (numruns == 2) { R = R12; T = T12; }
    if (numruns == 3) { R = R13; T = T13; }
    if (numruns == 4) { R = R14; T = T14; }
    if (numruns == 5) { R = R15; T = T15; }
  }

  //column wall
  if (map_name == "HALL") {
    Matrix2f R12;
    R12 << 0.9961, 0.0886, -0.0886, 0.9961;
    Vector2f T12(0.0991, -0.0600);

    Matrix2f R13;
    R13 << 0.9998, 0.0189, -0.0189, 0.9998;
    Vector2f T13(0.1330, -0.0015);

    Matrix2f R14;
    R14 << 0.9904, 0.1380, -0.1380, 0.9904;
    Vector2f T14(0.3377, -0.0719);

    Matrix2f R15;
    R15 << 0.9999, 0.0107, -0.0107, 0.9999;
    Vector2f T15(0.2860, -0.1341);

    if (numruns == 1) { R = R11; T = T11; }
    if (numruns == 2) { R = R12; T = T12; }
    if (numruns == 3) { R = R13; T = T13; }
    if (numruns == 4) { R = R14; T = T14; }
    if (numruns == 5) { R = R15; T = T15; }
  }

  PointCloudf point_cloud;
  NormalCloudf normal_cloud;
  vector<Pose2Df> poses;
  LoadStfs(stfs_file, &point_cloud, &normal_cloud, &poses, &log_timestamp_);
  CHECK_GT(point_cloud.size(), 0);
  CHECK_EQ(point_cloud.size(), normal_cloud.size());
  CHECK_EQ(point_cloud.size(), poses.size());

  for (size_t i=0; i<point_cloud.size(); i++) {
    point_cloud[i] = R*point_cloud[i] + T;
    poses[i].translation = R*poses[i].translation + T;
    normal_cloud[i] = R*normal_cloud[i] + T;
  }


  if (mode == 0) {
    //save points
    writePoints(poses, point_cloud, normal_cloud);
    //add visualization code here
  }

  if (mode == 1) {
    std::cout << "Build SDF" << std::endl;
    SdfObject one_pass = BuildSdfObjects(poses, point_cloud);
    std::cout << "Update SDF" << std::endl;
    master_sdf = updateSdf(master_sdf, one_pass, numruns);
  }

  if (mode == 2) {
    std::cout << "Filter  SDF" << std::endl;
    totalPointCloud filtered_rpc = filter(poses, point_cloud, normal_cloud, master_sdf);
    //save filtered points
    writePoints(filtered_rpc.poses, filtered_rpc.point_cloud, filtered_rpc.normal_cloud);
    //add visualization code here
  }

  if (mode == 3) {
    std::cout << "Build SDF" << std::endl;
    SdfObject one_pass = BuildSdfObjects(poses, point_cloud);
    std::cout << "Update SDF" << std::endl;
    master_sdf = updateSdf(master_sdf, one_pass, numruns);
    std::cout << "Filter  SDF" << std::endl;
    totalPointCloud filtered_rpc = filter(poses, point_cloud, normal_cloud, master_sdf);
    std::cout << "RANSAC" << std::endl;
    vector<mappingVector> new_vectors = localRANSAC(filtered_rpc.poses, filtered_rpc.point_cloud, filtered_rpc.normal_cloud);
    for (size_t i=0; i<new_vectors.size(); i++) {
      master_vectorMap.push_back(new_vectors[i]);
    }
  }

  if (mode == 4) {
    std::cout << "Build SDF" << std::endl;
    SdfObject one_pass = BuildSdfObjects(poses, point_cloud);
    std::cout << "Update SDF" << std::endl;
    master_sdf = updateSdf(master_sdf, one_pass, numruns);
    std::cout << "Update Vector Map" << std::endl;
    master_vectorMap = updateVectorMap(master_sdf, master_vectorMap);
    std::cout << "Filter  SDF" << std::endl;
    totalPointCloud filtered_rpc = filter(poses, point_cloud, normal_cloud, master_sdf);
    std::cout << "RANSAC" << std::endl;
    vector<mappingVector> new_vectors = localRANSAC(filtered_rpc.poses, filtered_rpc.point_cloud, filtered_rpc.normal_cloud);
    std::cout << "Merge New Lines" << std::endl;
    master_vectorMap = mergeNewVectors(new_vectors, master_vectorMap, false);
    std::cout << "Update Vector Map" << std::endl;
    master_vectorMap = updateVectorMap(master_sdf, master_vectorMap);
    std::cout << "Self Merge" << std::endl;
    master_vectorMap = selfMerge(master_vectorMap, master_vectorMap.size());
  }

/*
  EXAMPLE VIZUALIZATION CODE USING LOCALIZATION GUI

  POINTS
  vector<float> points_x;
  vector<float> points_y;
  for (size_t i=0; i<filtered_pc.size(); i++) {
    points_x.push_back(filtered_pc[i](0));
    points_y.push_back(filtered_pc[i](1));
  }
  display_message_.points_x = points_x;
  display_message_.points_y = points_y;
  display_publisher_.publish(display_message_);

  LINES
  vector<float> lines_p1x;
  vector<float> lines_p1y;
  vector<float> lines_p2x;
  vector<float> lines_p2y;
  for (size_t i=0; i<master_vectorMap.size(); i++) {
    lines_p1x.push_back(master_vectorMap[i].p1(0));
    lines_p1y.push_back(master_vectorMap[i].p1(1));
    lines_p2x.push_back(master_vectorMap[i].p2(0));
    lines_p2y.push_back(master_vectorMap[i].p2(1));
  }
  display_message_.lines_p1x = lines_p1x;
  display_message_.lines_p1y = lines_p1y;
  display_message_.lines_p2x = lines_p2x;
  display_message_.lines_p2y = lines_p2y;
  display_publisher_.publish(display_message_);
*/

  worldMap WM;
  WM.sdf = master_sdf;
  WM.vectorMap = master_vectorMap;
  return WM;

}

int main(int argc, char** argv) {
//  printf("Probabilistic Object Map Builder\n");
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();

  if (argc < 6) {
    std::cout << "Too few arguments.\nUsage: ./executable <mode> -m <map name> -f <one or more stfs files>" << std::endl;
    return 1;
  }
  vector<char*> stfs_files;
  for (int i=5; i<argc; i++) {
    stfs_files.push_back(argv[i]);
  }
  /*
    Modes
    0 - Raw data display
    1 - SDF construction only
    2 - Filter raw data given SDF
    3 - Full process w/o merge or delete updates
    4 - Full process
  */
  int num_modes = 4;
  std::istringstream ss(argv[1]);
  int mode;
  if (!(ss >> mode) || mode > num_modes) {
    std::cerr << "Invalid mode " << argv[1] << '\n';
    return 1;
  }

  char* stfs_file = NULL;
  /* Possible Maps: AMRL, MIT, WALL, HALL */
  char* map_name = NULL;
  static struct poptOption options[] = {
    { "stfs-file", 'f', POPT_ARG_STRING, &stfs_file, 0, "STFs bagfile to use", "STRING"},
    { "map_name", 'm', POPT_ARG_STRING, &map_name, 0, "Map Name", "STRING"},
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0){}

  string mn = string(map_name);
  std::cout << mn << std::endl;
  if (!(mn == "AMRL" || mn == "MIT" || mn == "WALL" || mn == "HALL")) {
    std::cout << "Invalid map name. Options: AMRL, MIT, WALL, HALL." << std::endl;
    return 1;
  }

  const bool config_loading_success = LoadConfiguration();
  CHECK(config_loading_success);

  string node_name = "SDFLTVM";
  ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);

  ros::NodeHandle ros_node;
  localization_publisher_ = ros_node.advertise<cobot_msgs::CobotLocalizationMsg>(
      "Cobot/Localization", 1, true);
  display_publisher_ = ros_node.advertise<cobot_msgs::LidarDisplayMsg>(
      "Cobot/VectorLocalization/Gui",1,true);
  gui_capture_client = ros_node.serviceClient<cobot_msgs::LocalizationGuiCaptureSrv>(
      "VectorLocalization/Capture");

  if (stfs_file == NULL) {
    printf("ERROR: STFs file not specified.\n");
    exit(1);
  }

  /* Some day, loading should be done in CImg... */

  SdfObject master_sdf;
  vector<mappingVector> master_vectorMap;
  worldMap MAP;
  MAP.sdf = master_sdf;
  MAP.vectorMap = master_vectorMap;

  if (mode == 2) {
    MAP.sdf = loadSDFObject();
  }
  for (size_t i=0; i<stfs_files.size(); i++) {
    MAP = BuildObjectsMap(stfs_files[i], MAP.sdf, MAP.vectorMap, i+1, mn, mode);
  }

  size_t image_width = MAP.sdf.weights.width();
  size_t image_height = MAP.sdf.weights.height();

  cimg_library::CImg<float> sdf_weights_image = MAP.sdf.weights;
  cimg_library::CImg<float> sdf_value_image = MAP.sdf.values;
  cimg_library::CImg<uint8_t> sdf_display_image(image_width, image_height, 1, 3, 0);

  float max_weight = 0.0;
  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      max_weight = max(max_weight, sdf_weights_image(x, y));
    }
  }

  double T = 0.85;
  double D = 0.05;

  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      if (sdf_weights_image(x, y) > T*max_weight) {
        if (sdf_value_image(x, y) >= D) {
          sdf_display_image(x, y, 0, 0) = 0;
          sdf_display_image(x, y, 0, 1) = 0;
          sdf_display_image(x, y, 0, 2) = static_cast<uint8_t>(
          (sdf_value_image(x, y) / map_options_.max_sdf_value) * 255.0);
        }
        else if(sdf_value_image(x, y) <= -D) {
          sdf_display_image(x, y, 0, 0) = static_cast<uint8_t>(
          (sdf_value_image(x, y) / map_options_.min_sdf_value) * 255.0);
          sdf_display_image(x, y, 0, 1) = 0;
          sdf_display_image(x, y, 0, 2) = 0;
        }
        else {
          sdf_display_image(x, y, 0, 0) = 0;
          sdf_display_image(x, y, 0, 1) = 255;
          sdf_display_image(x, y, 0, 2) = 0;
        }
      }
      else {
        sdf_display_image(x, y, 0, 0) = 0;
        sdf_display_image(x, y, 0, 1) = 0;
        sdf_display_image(x, y, 0, 2) = 0xFF;
      }
    }
  }

  bool vizSDF = false;
  if (vizSDF) {
    const string weights_image_file = StringPrintf("master_weights.png");
    MAP.sdf.weights.save_png(weights_image_file.c_str());
    const string values_image_file = StringPrintf("master_values.png");
    MAP.sdf.values.save_png(values_image_file.c_str());
    const string sdf_image_file = StringPrintf("master_sdf_display.png");
    sdf_display_image.save_png(sdf_image_file.c_str());
  }

  if (mode == 1) {
    writeSDFObject(MAP.sdf.weights, MAP.sdf.values, MAP.sdf.origin);
  }
  return 0;
}
