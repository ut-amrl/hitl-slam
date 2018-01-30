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
using namespace cimg_library;

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


// Visualization colour of points belonging to STFs.
// const uint32_t kStfPointColor = 0x1F994CD9;
const uint32_t kStfPointColor = 0xFFFF0000;


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



void transformMap(Affine2f transform) {
  for (size_t i=0; i<point_cloud.size(); i++) {
    point_cloud[i] = R*point_cloud[i] + T;
    poses[i].translation = R*poses[i].translation + T;
    normal_cloud[i] = R*normal_cloud[i] + T;
  }
}

int main(int argc, char** argv) {
//  printf("Probabilistic Object Map Builder\n");
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);
//  google::InitGoogleLogging(argv[0]);
//  google::LogToStderr();

  if (argc < 4) {
    std::cout << "Too few arguments.\nUsage: ./LTVM <num maps> "  
              << "<one or more map files> <one or more transform files>\n"
              << "Num maps and num transforms must be equal." << std::endl;
    return 1;
  }
  std::istringstream ss(argv[1]);
  int num_maps;
  if (!(ss >> num_maps)) {
    cout << "Failed to read number of maps." << endl;
    return 1;
  }
  if (2 * num_maps != (argc - 2)) {
    cout << "Number of maps, " << argv[1] 
         << ", does not match number of map files and transform files." << endl;
    return 1;
  }

  vector<char*> map_files;
  for (int i = 2; i < 2 + num_maps; i++) {
    map_files.push_back(argv[i]);
  }
  vector<char*> transform_files;
  for (int i = 2 + num_maps; i < argc; i++) {
    transform_files.push_back(argv[i]);
  }

  string node_name = "SDFLTVM";
  ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);

  ros::NodeHandle ros_node;
  localization_publisher_ = ros_node.advertise<cobot_msgs::CobotLocalizationMsg>(
      "Cobot/Localization", 1, true);
  display_publisher_ = ros_node.advertise<cobot_msgs::LidarDisplayMsg>(
      "Cobot/VectorLocalization/Gui",1,true);
  gui_capture_client = ros_node.serviceClient<cobot_msgs::LocalizationGuiCaptureSrv>(
      "VectorLocalization/Capture");

  
  LongTermVectorMap ltvm;
  ltvm.init();
//TODO: normal clouds?
  vector<Pose2Df> poses;
  vector<PointCloudf> point_clouds;
  vector<NormalCloudf> normal_clouds;
  Affine2f transform;
  for (int i = 0; i < num_maps; ++i) {
    poses.clear();
    point_clouds.clear();
    loadMap(map_files[i], &poses, &point_clouds);
    transform = loadTransform(transform_files[i]);
  //TODO: update instance as required
    
  }

  //TODO: save things!

  return 0;
}
