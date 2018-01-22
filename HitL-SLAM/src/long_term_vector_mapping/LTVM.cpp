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
//using EnmlMaps::GridObject;
//using EnmlMaps::LoadPersistentObjects;
//using EnmlMaps::SavePersistentObjects;
//using EnmlMaps::PersistentObject;
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
//vector<PersistentObject> persistent_objects_;

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

/* ################################ ABLATION TESTING ################################# */

void BuildOccupancyGrid(const vector<Pose2Df> poses, PointCloudf point_cloud) {
  const float pixel_half_width = sqrt(2.0) * map_options_.image_resolution;

  map_options_.min_sdf_weight = 0.01;

  float min_x(FLT_MAX), min_y(FLT_MAX);
  float max_x(-FLT_MAX), max_y(-FLT_MAX);
  for (size_t i=0; i<point_cloud.size() ;i++) {
    min_x = min(min_x, point_cloud[i](0));
    max_x = max(max_x, point_cloud[i](0));
    min_y = min(min_y, point_cloud[i](1));
    max_y = max(max_y, point_cloud[i](1));
  }
  const float width = max_x - min_x + 2.0 * map_options_.image_border;
  const float height = max_y - min_y + 2.0 * map_options_.image_border;

  const unsigned int image_width = ceil(width / map_options_.image_resolution);
  const unsigned int image_height = ceil(height / map_options_.image_resolution);

  // Initialize an empty grid.
  cimg_library::CImg<float> occ_grid_image(image_width, image_height);
  const Vector2f image_origin(min_x - map_options_.image_border, min_y - map_options_.image_border);

  OMP_PARALLEL_FOR
  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      occ_grid_image(x, y) = 0.0;
    }
  }

  OMP_PARALLEL_FOR
  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      for (size_t i=0; i<point_cloud.size(); i++) {
        const Vector2f point = point_cloud[i];
        const Vector2f pixel_loc = image_origin + map_options_.image_resolution * Vector2f(x, y);

        if ((point-pixel_loc).norm() <= pixel_half_width) {
          occ_grid_image(x,y) = 255.0;
        }
      }
    }
  }

  const string occ_grid_image_file = StringPrintf("occupancy.png");
  occ_grid_image.save_png(occ_grid_image_file.c_str());
}

inline Vector2f MarchingSquaresEdge(const float v0, const float v1,
                         const Vector2f& p0, const Vector2f& p1) {
  return ((v0 * p0 - v1 * p1) / (v0 - v1));
}

void MarchingSquares(const cimg_library::CImg<float>& sdf_image,
                     const cimg_library::CImg<float>& weights_image,
                     const Vector2f& image_origin,
                     const float image_resolution,
                     const float kMinWeight,
                     vector<pair<Vector2f, Vector2f> >* contour_ptr) {
  vector<pair<Vector2f, Vector2f> >& contour = *contour_ptr;
  // Convention of vertex locations 0-3, where origin is bottom left:
  //
  // 3 * * * 2
  // *       *
  // *       *
  // *       *
  // 0 * * * 1
  //
  // Convention of bitmask of zero-crossing locations using bits 0-3:
  //
  // * * 4 * *
  // *       *
  // 8       2
  // *       *
  // * * 1 * *
  CHECK_EQ(sdf_image.width(), weights_image.width());
  CHECK_EQ(sdf_image.height(), weights_image.height());
  for (int x = 0; x < sdf_image.width() - 1; ++x) {
    for (int y = 0; y < sdf_image.height() - 1; ++y) {
      if (weights_image(x, y) < kMinWeight ||
          weights_image(x + 1, y) < kMinWeight ||
          weights_image(x + 1, y + 1) < kMinWeight ||
          weights_image(x, y + 1) < kMinWeight) {
        continue;
      }
      const float& v0 = sdf_image(x, y);
      const float& v1 = sdf_image(x + 1, y);
      const float& v2 = sdf_image(x + 1, y + 1);
      const float& v3 = sdf_image(x, y + 1);
      const Vector2f p0 =
          image_origin + image_resolution * Vector2f(x, y);
      const Vector2f p1 =
          image_origin + image_resolution * Vector2f(x + 1, y);
      const Vector2f p2 =
          image_origin + image_resolution * Vector2f(x + 1, y+ 1);
      const Vector2f p3 =
          image_origin + image_resolution * Vector2f(x, y + 1);

      const Vector2f e1 = MarchingSquaresEdge(v0, v1, p0, p1);
      const Vector2f e2 = MarchingSquaresEdge(v1, v2, p1, p2);
      const Vector2f e4 = MarchingSquaresEdge(v2, v3, p2, p3);
      const Vector2f e8 = MarchingSquaresEdge(v3, v0, p3, p0);
      const uint8_t case_number =
          ((v0 < 0.0) ? 1 : 0) |
          ((v1 < 0.0) ? 2 : 0) |
          ((v2 < 0.0) ? 4 : 0) |
          ((v3 < 0.0) ? 8 : 0);
      switch (case_number) {
        case 0: case 15: {
          // No edges.
        } break;

        case 1: case 14: {
          // Edge from 1 - 8.
          contour.push_back(make_pair(e1, e8));
        } break;

        case 2: case 13: {
          // Edge from 1 - 2.
          contour.push_back(make_pair(e1, e2));
        } break;

        case 3: case 12: {
          // Edge from 2 - 8.
          contour.push_back(make_pair(e2, e8));
        } break;

        case 4: case 11: {
          // Edge from 2 - 4.
          contour.push_back(make_pair(e2, e4));
        } break;

        case 5: {
          // Edge from 1 - 8.
          contour.push_back(make_pair(e1, e8));
          // Edge from 2 - 4
          contour.push_back(make_pair(e2, e4));
        } break;

        case 10: {
          // Edge from 1 - 2.
          contour.push_back(make_pair(e1, e2));
          // Edge from 4 - 8.
          contour.push_back(make_pair(e4, e8));
        } break;

        case 6: case 9: {
          // Edge from 1 - 4.
          contour.push_back(make_pair(e1, e4));
        } break;

        case 7: case 8: {
          // Edge from 4 - 8.
          contour.push_back(make_pair(e4, e8));
        } break;

        default: {
          // Cannot happen unless there's a bug.
          static const bool kMarchingSquaresBug = false;
          CHECK(kMarchingSquaresBug);
        }
      }
    }
  }
}

/* ################################ STRUCTS ################################# */

struct segDistResidual {
  segDistResidual(double px, double py, double cmx, double cmy, double N) : px_(px), py_(py), cmx_(cmx), cmy_(cmy), N_(N) {}
  template <typename T> bool operator()(const T* const p1, const T* const p2, T* residual) const {
    T t = ((T(px_)-p1[0])*(p2[0]-p1[0])+(T(py_)-p1[1])*(p2[1]-p1[1])) / (pow(p2[0]-p1[0],2)+pow(p2[1]-p1[1],2));
    T partial_res, centroid1, centroid2;
    centroid1 = sqrt(pow(T(cmx_)-p1[0],2)+pow(T(cmy_)-p1[1],2)); //dist p1 to cm
    centroid2 = sqrt(pow(T(cmx_)-p2[0],2)+pow(T(cmy_)-p2[1],2)); //dist p2 to cm
    if (t < 0.0) {
      partial_res = sqrt(pow(T(px_)-p1[0],2)+pow(T(py_)-p1[1],2)); // Beyond the 'p1' end of the segment
    }
    else if (t > 1.0) {
      partial_res = sqrt(pow(T(px_)-p2[0],2)+pow(T(py_)-p2[1],2)); // Beyond the 'p2' end of the segment
    }
    else {
      T projx = p1[0] + t*(p2[0]-p1[0]);  // Projection falls on the segment
      T projy = p1[1] + t*(p2[1]-p1[1]);  // Projection falls on the segment
      partial_res = sqrt(pow((T(px_) - projx),2) + pow((T(py_) - projy),2));
    }
    residual[0] = partial_res + T(10.0)*centroid1/(N_) + T(10.0)*centroid2/(N_);
    return true;
  }
  private:
  const double px_;
  const double py_;
  const double cmx_;
  const double cmy_;
  const double N_;
};

struct totalPointCloud {
  PointCloudf point_cloud;
  NormalCloudf normal_cloud;
  vector<Pose2Df> poses;
};

struct SdfObject {
  bool empty;
  cimg_library::CImg<float> weights;
  cimg_library::CImg<float> values;
  Vector2f origin;
};

struct mappingVector{
  int mass;
  Vector2d p1;
  Vector2d p2;
  Vector2d p_bar;
  Matrix2d scatter;
  Matrix2d p1_cov;
  Matrix2d p2_cov;
};

struct worldMap {
  SdfObject sdf;
  vector<mappingVector> vectorMap;
};

/* ################################ RANSAC HELPERS ################################# */

Eigen::Vector2d closestPoint(Vector2d pstar, Vector2d ps, Vector2d p) {
  Eigen::Vector2d p0;
  double coeff = (p-ps).dot(pstar)/pstar.dot(pstar);
  p0 = ps + coeff*pstar;
  return p0;
}

double distToLine(Vector2d v, Vector2d w, Vector2d p) {
  double t = (p-v).dot(w-v)/(w-v).dot(w-v);
  Vector2d proj = v + t*(w-v);
  double dst = sqrt((p-proj).dot(p-proj));
  return dst;
}

double dirDistToLine(Vector2d p1, Vector2d p2, Vector2d p, Vector2d pdir) {
  Matrix2d rot;
  rot << 0, -1, 1, 0;
  Vector2d nhat = rot*(p2-p1);
  nhat.normalize();
  pdir.normalize();
  double numer = distToLine(p1, p2, p);
  double denom = nhat.dot(pdir);
  if (denom < 0.01) {
    denom = 0.01;
  }
  double dst = numer/denom;
  return dst;
}

double distToLineSeg(Vector2d p1, Vector2d p2, Vector2d p) {
  double t = (p-p1).dot(p2-p1)/(p2-p1).dot(p2-p1);
  if (t < 0.0) {
    return sqrt((p-p1).dot(p-p1));  // Beyond the 'v' end of the segment
  }
  else if (t > 1.0) {
    return sqrt((p-p2).dot(p-p2));  // Beyond the 'w' end of the segment
  }
  Vector2d proj = p1 + t*(p2-p1);  // Projection falls on the segment
  return sqrt((p-proj).dot(p-proj));
}

vector<Vector2d> segFit(double* p1, double* p2, double* cm, double* data, int size) {
  vector<Vector2d> fit;
  Vector2d ep1;
  Vector2d ep2;

  Problem problem;
  for (int i = 0; i<size; ++i) {
    problem.AddResidualBlock(
      new AutoDiffCostFunction<segDistResidual, 1, 2, 2>(
          new segDistResidual(data[2*i], data[2*i + 1], cm[0], cm[1], double(size))),
          NULL, p1, p2);
  }

  Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  ep1(0) = p1[0];
  ep1(1) = p1[1];
  ep2(0) = p2[0];
  ep2(1) = p2[1];

  fit.push_back(ep1);
  fit.push_back(ep2);
  return fit;
}

/* ################################ COMPUTE UNCERTAINTY ################################# */

mappingVector computeVectorUncertainty(Vector2d p1, Vector2d p2, vector<Vector2d> inlier_points, vector<Pose2Df> poses) {
  mappingVector new_vector;
  //number of times to sample points
  int k = 1000;
  //radians
  double sigma_theta = 0.002; //potentially lower to .0001 as suggested in literature. Laser samples every 0.25 deg = 0.004 rad
  //meters
  double sigma_d = 0.03; //upper bound according to data sheet... may be substantially lower
  Vector2d x_hat = Vector2d(1.0, 0.0);

  double dst, phi;
  vector<Matrix2f> Q;
  for (size_t i=0; i<inlier_points.size(); i++) {
    Vector2d source = Vector2d(poses[i].translation(0), poses[i].translation(1));
    Vector2d point = inlier_points[i];
    Vector2d ray = point- source;
    dst = ray.norm();
    phi = acos((ray/ray.norm()).dot(x_hat));
    Matrix2f cov_ang;
    cov_ang(0,0) = 2*sin(phi)*sin(phi);
    cov_ang(0,1) = -sin(2*phi);
    cov_ang(1,0) = -sin(2*phi);
    cov_ang(1,1) = 2*cos(phi)*cos(phi);
    cov_ang = cov_ang*((dst*dst*sigma_theta*sigma_theta)/2);
    Matrix2f cov_dst;
    cov_dst(0,0) = 2*cos(phi)*cos(phi);
    cov_dst(0,1) = sin(2*phi);
    cov_dst(1,0) = sin(2*phi);
    cov_dst(1,1) = 2*sin(phi)*sin(phi);
    cov_dst = cov_dst*((sigma_d*sigma_d)/2);
    Q.push_back(cov_ang + cov_dst);
  }

  //find major and minor axis via eigenvalue decomposition of Q
  vector<double> sigma1s;
  vector<double> sigma2s;
  vector<Vector2d> dir1s;
  vector<Vector2d> dir2s;
  Vector2d dir1;
  Vector2d dir2;
  Eigen::EigenSolver<Matrix2f> es;
  for (size_t i=0; i<Q.size(); i++) {
    Matrix2f q = Q[i];
    es.compute(q, true);
    double lambda1 = double(es.eigenvalues()[0].real());
    double lambda2 = double(es.eigenvalues()[1].real());
    double sigma1 = sqrt(5.991*lambda1);
    double sigma2 = sqrt(5.991*lambda2);
    sigma1s.push_back(sigma1);
    sigma2s.push_back(sigma2);
    dir1(0) = es.eigenvectors().col(0)(0).real();
    dir1(1) = es.eigenvectors().col(0)(1).real();
    dir2(0) = es.eigenvectors().col(1)(0).real();
    dir2(1) = es.eigenvectors().col(1)(1).real();
    dir1.normalize();
    dir2.normalize();
    dir1s.push_back(dir1);
    dir2s.push_back(dir2);
  }

  vector<double> p1xs;
  vector<double> p1ys;
  vector<double> p2xs;
  vector<double> p2ys;
  size_t NI = inlier_points.size();
  for (int i=0; i<k; i++) {
    vector<Vector2d> bootstrap;
    Vector2d pstar;
    for (size_t j=0; j<NI; j++) {
      //resample based on gaussian noise model and covariance matrices for each point
      double alpha, beta;
      alpha = randn(sigma1s[j], 0.0);
      beta = randn(sigma2s[j], 0.0);
      pstar = inlier_points[j] + alpha*dir1s[j] + beta*dir2s[j] ;
      bootstrap.push_back(pstar);
    }

    double data[2*NI];
    Vector2d cm(0.0,0.0);
    Vector2d ip;
    for (size_t j=0; j<NI; j++) {
      ip = bootstrap[j];
      data[2*j] = ip(0);
      data[2*j+1] = ip(1);
      cm += ip;
    }
    cm = cm/double(NI);

    double CM[2];
    CM[0] = cm(0);
    CM[1] = cm(1);
    double P1[2];
    double P2[2];
    P1[0] = p1(0);
    P1[1] = p1(1);
    P2[0] = p2(0);
    P2[1] = p2(1);

    vector<Vector2d> end_points = segFit(P1, P2, CM, data, NI);

    p1xs.push_back(end_points[0](0));
    p1ys.push_back(end_points[0](1));
    p2xs.push_back(end_points[1](0));
    p2ys.push_back(end_points[1](1));
  }

  double p1xbar = p1(0);
  double p1ybar = p1(1);
  double p2xbar = p2(0);
  double p2ybar = p2(1);
  size_t N = p1xs.size();

  double cov_x1x1 = 0.0;
  double cov_x1y1 = 0.0;
  double cov_y1y1 = 0.0;
  double cov_x2x2 = 0.0;
  double cov_x2y2 = 0.0;
  double cov_y2y2 = 0.0;
  for (size_t i=0; i<p1xs.size(); i++) {
    cov_x1x1 += (p1xbar-p1xs[i])*(p1xbar-p1xs[i]);
    cov_x1y1 += (p1xbar-p1xs[i])*(p1ybar-p1ys[i]);
    cov_y1y1 += (p1ybar-p1ys[i])*(p1ybar-p1ys[i]);
    cov_x2x2 += (p2xbar-p2xs[i])*(p2xbar-p2xs[i]);
    cov_x2y2 += (p2xbar-p2xs[i])*(p2ybar-p2ys[i]);
    cov_y2y2 += (p2ybar-p2ys[i])*(p2ybar-p2ys[i]);
  }
  cov_x1x1 = cov_x1x1/double(N);
  cov_x1y1 = cov_x1y1/double(N);
  cov_y1y1 = cov_y1y1/double(N);
  cov_x2x2 = cov_x2x2/double(N);
  cov_x2y2 = cov_x2y2/double(N);
  cov_y2y2 = cov_y2y2/double(N);

  Matrix2d p1_cov;
  p1_cov(0,0) = cov_x1x1;
  p1_cov(1,0) = cov_x1y1;
  p1_cov(0,1) = cov_x1y1;
  p1_cov(1,1) = cov_y1y1;
  Matrix2d p2_cov;
  p2_cov(0,0) = cov_x2x2;
  p2_cov(1,0) = cov_x2y2;
  p2_cov(0,1) = cov_x2y2;
  p2_cov(1,1) = cov_y2y2;


  //compute centroid, scatter matrix components
  Matrix2d Si = Matrix2d::Zero();
  Vector2d p_bar(0.0, 0.0);
  Vector2d p_bar2(0.0, 0.0);
  for (size_t i=0; i<NI; i++) {
    p_bar += inlier_points[i];
    Si += inlier_points[i]*inlier_points[i].transpose();
  }
  p_bar = p_bar/double(NI);
  Si = Si/double(NI);

  new_vector.mass = NI;
  new_vector.p1 = p1;
  new_vector.p2 = p2;
  new_vector.p1_cov = p1_cov;
  new_vector.p2_cov = p2_cov;
  new_vector.p_bar = p_bar;
  new_vector.scatter = Si;

  return new_vector;
}


/* ################################ LOCAL RANSAC ################################# */

vector<mappingVector> localRANSAC(vector<Pose2Df> poses, vector<Vector2f> point_cloud, NormalCloudf normal_cloud) {
  vector<mappingVector> new_vectors;

  vector<Vector2d> used;
  vector<pair<Vector2d, Vector2d> > segments;

  vector<Vector2d> working_set;
  for (size_t i=0; i<point_cloud.size(); i++) {
    working_set.push_back(Vector2d(point_cloud[i](0), point_cloud[i](1)));
  }

  double minx, miny, maxx, maxy;
  for (size_t i=0; i<working_set.size(); i++) {
    minx = min(working_set[i](0), minx);
    miny = min(working_set[i](1), miny);
    maxx = max(working_set[i](0), maxx);
    maxy = max(working_set[i](1), maxy);
  }
  vector<Vector2d> model;
  //standard deviation in sensed data in meters
  double sigma = .04;
  //inlier threshold necessary to fit
  double w = .8;
  //threshold for determining if a data point is local
  double radius = .4;
  //threshold for determining if a data point is local along a line
  double l = 0.5;
  //threshold for determining if a data point reasonably fits the proposed model
  double t = 3*sigma;
  //threshold for determining if a data point has a normal that agrees with the model
  double tn = 0.5;
  //minimum number of observations warranting a line fit
  int d = 150;
  //iterations
  int k = 2000;

  Eigen::Matrix2d rot;
  rot << 0, -1, 1, 0;

  int iter = 0;
  while (iter < k && working_set.size() >= size_t(d)) {
    iter++;
    int N = working_set.size();
    vector<Vector2d> neighborhood;
    Vector2d v1;
    pair<Vector2d, Vector2d> new_model;
    while (neighborhood.empty()) {
      int v1_index = rand() % N;
      v1 = working_set[v1_index];
      for (int i=0; i<N; i++) {
        if ((working_set[i] - v1).norm() <= radius && working_set[i] != v1) {
          neighborhood.push_back(working_set[i]);
        }
      }
    }

    bool model_ok = false;
    while (!model_ok) {
      int v2_index = rand() % neighborhood.size();
      if (neighborhood[v2_index] != v1) {
        new_model = std::make_pair(v1,neighborhood[v2_index]);
        model_ok = true;
      }
    }

    bool converge = false;
    int outliers = 0;
    int inliers = 0;
    vector<Vector2d> inlier_points;
    vector<int> inlier_indices;
    vector<Pose2Df> inlier_poses;
    Vector2d p1;
    Vector2d p2;
    Vector2d norm_model = (new_model.first-new_model.second)/(new_model.first-new_model.second).norm();
    for (int i=0; i<N; i++) {
      Vector2d point = working_set[i];
      Vector2d source = Vector2d(poses[i].translation(0), poses[i].translation(1));
      Vector2d dir = (point - source).normalized();

      double dirdst = dirDistToLine(new_model.first, new_model.second, working_set[i], dir);
      double dst = distToLineSeg(new_model.first, new_model.second, working_set[i]);
      Vector2f normalf = normal_cloud[i]/(normal_cloud[i].norm());
      Vector2d normal(double(normalf(0)),double(normalf(1)));
      double ndst = fabs(norm_model.dot(normal));
      if (dst <= t && tn >= ndst) { //close to line segment
        inliers ++;
        inlier_points.push_back(working_set[i]);
        inlier_poses.push_back(poses[i]);
        inlier_indices.push_back(i);
      }
      else if (dst > t && dirdst > t && dirdst < l) { // in the neighborhood l, but not close enough to infinite line
        outliers ++;
      }
    }
    double in_frac = double(inliers)/double(inliers + outliers);

    if (inliers >= d && in_frac >= w) {
      while (!converge) {
        int NI = inlier_points.size();
        Vector2d cm(0.0,0.0);
        double data[2*NI];
        Vector2d ip;
        for (int i=0; i<NI; i++) {
          ip = inlier_points[i];
          data[2*i] = ip(0);
          data[2*i+1] = ip(1);
          cm += ip;
        }
        cm = cm/double(NI);
        double CM[2];
        CM[0] = cm(0);
        CM[1] = cm(1);
        double P1[2];
        double P2[2];
        P1[0] = new_model.first(0) + 0.000001;
        P1[1] = new_model.first(1) + 0.000001;
        P2[0] = new_model.second(0) + 0.000001;
        P2[1] = new_model.second(1) + 0.000001;

        vector<Vector2d> end_points = segFit(P1, P2, CM, data, NI);
        p1 = end_points[0];
        p2 = end_points[1];

        // no significant change in p1 or p2
        if ((new_model.first - p1).norm() < 0.1 && (new_model.second - p2).norm() < 0.1) {
          converge = true;
        }
        else {
          outliers = 0;
          inliers = 0;
          new_model.first = p1;
          new_model.second = p2;
          inlier_points.clear();
          inlier_indices.clear();

          for (int i=0; i<N; i++) {
            Vector2d point = working_set[i];
            Vector2d source = Vector2d(poses[i].translation(0), poses[i].translation(1));
            Vector2d dir = (point - source).normalized();

            double dirdst = dirDistToLine(new_model.first, new_model.second, working_set[i], dir);
            double dst = distToLineSeg(new_model.first, new_model.second, working_set[i]);
            Vector2f normalf = normal_cloud[i]/(normal_cloud[i].norm());
            Vector2d normal(double(normalf(0)),double(normalf(1)));
            double ndst = fabs(norm_model.dot(normal));
            if (dst <= t && tn >= ndst) {
              inliers ++;
              inlier_points.push_back(working_set[i]);
              inlier_poses.push_back(poses[i]);
              inlier_indices.push_back(i);
            }
            else if (dst > t && dirdst > t && dirdst < l) {
              outliers ++;
            }
          }
          in_frac = double(inliers)/double(inliers + outliers);
        }
      } //end while

      pair<Vector2d, Vector2d> seg = make_pair(p1,p2);
      segments.push_back(seg);

      int NI = inlier_points.size();
      for (int i=NI-1; i>=0; i--) {
        working_set.erase(working_set.begin() + inlier_indices[i]);
        poses.erase(poses.begin() + inlier_indices[i]);
        normal_cloud.erase(normal_cloud.begin() + inlier_indices[i]);
      }

      //compute uncertainty
      mappingVector new_vector = computeVectorUncertainty(p1, p2, inlier_points, inlier_poses);
      new_vectors.push_back(new_vector);

    } //end if (valid model)

  } //end ransac main while

  return new_vectors;
}

/* ################################ FILTERING / INTERPOLATING ################################# */

double cubicInterpolate (double I[4], double u[4], double X) {
  double t = X - u[0];
  double fneg = -0.5*pow(t,3) + pow(t,2) - 0.5*t;
  t = X - u[1];
  double f0 = 1.5*pow(t,3) - 2.5*pow(t,2) + 1;
  t = X - u[2];
  double f1 = -1.5*pow(t,3) + 2*pow(t,2) + 0.5*t;
  t = X - u[3];
  double f2 = 0.5*pow(t,3) - 0.5*pow(t,2);
  double f = I[0]*fneg + I[1]*f0 + I[2]*f1 + I[3]*f2;
  return f;
}

double bicubicInterpolate (double I[4][4], double xl[4], double yl[4], double X, double Y) {
  double arr[4];
  arr[0] = cubicInterpolate(I[0], yl, Y);
  arr[1] = cubicInterpolate(I[1], yl, Y);
  arr[2] = cubicInterpolate(I[2], yl, Y);
  arr[3] = cubicInterpolate(I[3], yl, Y);
  return cubicInterpolate(arr, xl, X);
}

totalPointCloud filter(const vector<Pose2Df> poses, PointCloudf point_cloud, NormalCloudf normal_cloud, SdfObject master_sdf) {
  cimg_library::CImg<float> weights = master_sdf.weights;
  cimg_library::CImg<float> values = master_sdf.values;
  Vector2f image_origin = master_sdf.origin;
  float max_weight = 0.0;
  for (int x=0; x<weights.width(); x++) {
    for (int y=0; y<weights.height(); y++) {
      max_weight = max(weights(x,y), max_weight);
    }
  }
  double T = 0.95*max_weight;
  double D = 0.05;

  pair<vector<Pose2Df>, vector<Vector2f> > filtered_pair;
  vector<Vector2f> filtered_pc;
  vector<Vector2f> filtered_normals;
  vector<Pose2Df> filtered_poses;

  for (size_t i = 0; i < point_cloud.size(); i++) {
    const Vector2f point = point_cloud[i];
    double x = point(0);
    double y = point(1);
    double cx = ceil( (point(0) - image_origin(0))/map_options_.image_resolution );
    double fx = floor( (point(0) - image_origin(0))/map_options_.image_resolution );
    double cy = ceil( (point(1) - image_origin(1))/map_options_.image_resolution );
    double fy = floor( (point(1) - image_origin(1))/map_options_.image_resolution );
    if (cx == fx) {cx++;}
    if (cy == fy) {cy++;}
    double step = map_options_.image_resolution;
    Vector2f patch_origin = image_origin + step * Vector2f(fx-1, fy-1);

    double xl[4] = {patch_origin(0), patch_origin(0)+step, patch_origin(0)+2*step, patch_origin(0)+3*step};
    double yl[4] = {patch_origin(1), patch_origin(1)+step, patch_origin(1)+2*step, patch_origin(1)+3*step};

    double w[4][4] = {{weights(fx-1, cy+1), weights(fx-1, cy), weights(fx-1, fy), weights(fx-1, fy-1)},
                      {weights(fx, cy+1), weights(fx, cy), weights(fx, fy), weights(fx, fy-1)},
                      {weights(cx, cy+1), weights(cx, cy), weights(cx, fy), weights(cx, fy-1)},
                      {weights(cx+1, cy+1), weights(cx+1, cy), weights(cx+1, fy), weights(cx+1, fy-1)}};

    double v[4][4] = {{values(fx-1, cy+1), values(fx-1, cy), values(fx-1, fy), values(fx-1, fy-1)},
                      {values(fx, cy+1), values(fx, cy), values(fx, fy), values(fx, fy-1)},
                      {values(cx, cy+1), values(cx, cy), values(cx, fy), values(cx, fy-1)},
                      {values(cx+1, cy+1), values(cx+1, cy), values(cx+1, fy), values(cx+1, fy-1)}};

    if (bicubicInterpolate(w,xl,yl,x,y) > T && fabs(bicubicInterpolate(v,xl,yl,x,y)) < D) {
      filtered_pc.push_back(point);
      filtered_poses.push_back(poses[i]);
      filtered_normals.push_back(normal_cloud[i]);
    }
  }

  totalPointCloud pc;
  pc.point_cloud = filtered_pc;
  pc.poses = filtered_poses;
  pc.normal_cloud = filtered_normals;
  return pc;
}

/* ################################ BUILD SDF ################################# */

SdfObject BuildSdfObjects(const vector<Pose2Df> poses, PointCloudf point_cloud) {
  const float pixel_half_width = sqrt(2.0) * map_options_.image_resolution;

  map_options_.min_sdf_weight = 0.01;

  float min_x(FLT_MAX), min_y(FLT_MAX);
  float max_x(-FLT_MAX), max_y(-FLT_MAX);
  for (size_t i=0; i<point_cloud.size() ;i++) {
    min_x = min(min_x, point_cloud[i](0));
    max_x = max(max_x, point_cloud[i](0));
    min_y = min(min_y, point_cloud[i](1));
    max_y = max(max_y, point_cloud[i](1));
  }
  const float width = max_x - min_x + 2.0 * map_options_.image_border;
  const float height = max_y - min_y + 2.0 * map_options_.image_border;

  const unsigned int image_width = ceil(width / map_options_.image_resolution);
  const unsigned int image_height = ceil(height / map_options_.image_resolution);

  // Initialize an empty SDF image and weights image.
  cimg_library::CImg<float> sdf_value_image(image_width, image_height);
  cimg_library::CImg<float> sdf_weights_image(image_width, image_height);
  cimg_library::CImg<uint8_t> sdf_display_image(image_width, image_height, 1, 3, 0);
  const Vector2f image_origin(min_x - map_options_.image_border, min_y - map_options_.image_border);

  //meters
  float eps = 0.02;
  float sigma = 0.02;

  OMP_PARALLEL_FOR
  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      sdf_value_image(x, y) = map_options_.min_sdf_value;
      sdf_weights_image(x, y) = 0.0;
      sdf_display_image(x, y, 0, 0) = 0;
      sdf_display_image(x, y, 0, 1) = 0;
      sdf_display_image(x, y, 0, 2) = 0;
    }
  }

  OMP_PARALLEL_FOR
  for (size_t x = 0; x < image_width; ++x) {
    if (x%20 == 0) {
      std::cout << x << "/" << image_width << std::endl;
    }
    for (size_t y = 0; y < image_height; ++y) {
      for (size_t i=0; i<point_cloud.size(); i++) {
        const Vector2f point = point_cloud[i];
        const Vector2f source = poses[i].translation;
        const Vector2f line_dir = (point - source).normalized();
        const Vector2f line_perp = Perp2(line_dir);

        const Vector2f pixel_loc = image_origin + map_options_.image_resolution * Vector2f(x, y);
        const bool within_angle_tolerance = (fabs(line_perp.dot(point - pixel_loc)) /
            (point - source).norm() < 0.5 * map_options_.laser_angular_resolution);
        const bool along_viewing_ray = (fabs(line_perp.dot(pixel_loc - point)) < pixel_half_width);
        if (!along_viewing_ray && !within_angle_tolerance) continue;

        // Locations farther than the observed point along the viewing ray from the source have
        // a negative SDF value since they are "occupied".
        const float sdf_value = line_dir.dot(point - pixel_loc);

        // If pixel is along visible ray and between the point and its source: Update SDF
        const float tsdf_value = min(sdf_value, map_options_.max_sdf_value);
        if (sdf_value >= map_options_.min_sdf_value) { //only revise values in front of or near range reading
          //exponential decaying weight
          float sdf_weight = 0.0;
          if (fabs(tsdf_value) < eps) {
            sdf_weight = 1.0;
          }
          else if (sdf_value > map_options_.max_sdf_value) {
            sdf_weight = map_options_.min_sdf_weight;
          }
          else {
            sdf_weight = exp(-sigma*(tsdf_value-eps)*(tsdf_value-eps)); //tsdf == sdf in this case
          }
           // Update values
          sdf_value_image(x, y) =
              (sdf_value_image(x, y) * sdf_weights_image(x, y) + sdf_weight * tsdf_value) /
              (sdf_weights_image(x, y) + sdf_weight);
          // Update weights
          sdf_weights_image(x, y) += sdf_weight;
        }
        else {
          sdf_weights_image(x,y) += 1.0;
        }
      }
    }
  }

  float max_weight = 0.0;
  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      max_weight = max(max_weight, sdf_weights_image(x, y));
    }
  }

  double T = .2;
  double D = 0.05;

  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      if (sdf_weights_image(x, y) > T*max_weight) {
        sdf_weights_image(x,y) = 255.0;
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
        sdf_weights_image(x,y) = 0.0;
        sdf_display_image(x, y, 0, 0) = 0;
        sdf_display_image(x, y, 0, 1) = 0;
        sdf_display_image(x, y, 0, 2) = 0xFF;
      }
    }
  }

  SdfObject one_pass;
  one_pass.origin = image_origin;
  one_pass.weights = sdf_weights_image;
  one_pass.values = sdf_value_image;

  float max_val = 0.0;
  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      max_val = max(max_val, sdf_value_image(x, y));
    }
  }

  const string weights_image_file = StringPrintf("sdf_weights.png");
  //sdf_weights_image.save_png(weights_image_file.c_str());

  const string values_image_file = StringPrintf("sdf_values.png");
  //((255.0/max_val)*sdf_value_image).save_png(values_image_file.c_str());

  const string sdf_image_file = StringPrintf("sdf_display.png");
  //sdf_display_image.save_png(sdf_image_file.c_str());

  return one_pass;
}

/* ################################ UPDATE SDF ################################# */

SdfObject updateSdf(SdfObject master_sdf, SdfObject one_pass, int numruns) {
  if (master_sdf.empty) {
    master_sdf = one_pass;
    master_sdf.empty = false;
    return master_sdf;
  }
  else {
    //create new master SdfObject with proper size
    double O0x = master_sdf.origin(0);
    double O0y = master_sdf.origin(1);
    double O1x = one_pass.origin(0);
    double O1y = one_pass.origin(1);

    int h0 = master_sdf.weights.height();
    int h1 = one_pass.weights.height();
    int w0 = master_sdf.weights.width();
    int w1 = one_pass.weights.width();

    double res = map_options_.image_resolution;
    double new_height_wc = max(max(O0y+h0*res,O0y+h1*res),max(O1y+h0*res,O1y+h1*res));
    double new_width_wc = max(max(O0x+w0*res,O0x+w1*res),max(O1x+w0*res,O1x+w1*res));
    Vector2f new_origin(min(O0x, O1x), min(O0y, O1y));

    const unsigned int image_height = ceil(fabs(new_height_wc - new_origin(1))/res);
    const unsigned int image_width = ceil(fabs(new_width_wc - new_origin(0))/res);

    int ix_master = round((O0x - new_origin(0))/res);
    int iy_master = round((O0y - new_origin(1))/res);

    int ix_add = round((O1x - new_origin(0))/res);
    int iy_add = round((O1y - new_origin(1))/res);

    cimg_library::CImg<float> sdf_value_image(image_width, image_height);
    cimg_library::CImg<float> sdf_weights_image(image_width, image_height);

    for (size_t x=0; x<image_width; x++) {
      for (size_t y=0; y<image_height; y++) {
        sdf_value_image(x,y) = 0.0;
        sdf_weights_image(x,y) = 0.0;
      }
    }

    for (int x=0; x<w0; x++) {
      for (int y=0; y<h0; y++) {
        sdf_value_image(x+ix_master,y+iy_master) += master_sdf.values(x,y)*(numruns-1);
        sdf_weights_image(x+ix_master,y+iy_master) += master_sdf.weights(x,y)*(numruns-1);
      }
    }

    for (int x=0; x<w1; x++) {
      for (int y=0; y<h1; y++) {
        sdf_value_image(x+ix_add,y+iy_add) += one_pass.values(x,y);
	sdf_weights_image(x+ix_add,y+iy_add) += one_pass.weights(x,y);
      }
    }
    master_sdf.weights = sdf_weights_image/numruns;
    master_sdf.values = sdf_value_image/numruns;
    master_sdf.origin = new_origin;
    return master_sdf;
  }
}

/* ################################ UPDATE / MERGE MAPS ################################# */

vector<mappingVector> mergeNewVectors(vector<mappingVector> new_vectors, vector<mappingVector> master_vectorMap, bool self_merge) {
  if (master_vectorMap.size() == 0) {
    return new_vectors;
  }
  vector<mappingVector> new_master_vectorMap;
  vector<pair<int, vector<mappingVector> > > merge_map;
  vector<pair<int, vector<double> > > extrema;
  vector<int> used_indices;
  for (size_t i=0; i<new_vectors.size(); i++) {
    Vector2d new_p1 = new_vectors[i].p1;
    Vector2d new_p2 = new_vectors[i].p2;
    Vector2d new_pstar = (new_p2-new_p1);
    Matrix2d new_p1_cov = new_vectors[i].p1_cov;
    Matrix2d new_p2_cov = new_vectors[i].p2_cov;
    bool merged = false;
    int vec = 0;
    for (size_t j=0; j<master_vectorMap.size(); j++) {
      if (self_merge) { j=i+vec; vec++;}
      if ((self_merge && i != j) || !self_merge) {
        Vector2d p1 = master_vectorMap[j].p1;
        Vector2d p2 = master_vectorMap[j].p2;
        Vector2d pstar = (p2-p1);
        double t1 = (new_p1-p1).dot(pstar)/pstar.dot(pstar);
        double t2 = (new_p2-p1).dot(pstar)/pstar.dot(pstar);

        //else, doorway case: 	p1--------p2      p1`----------p2`
        if (!((t1 < 0 && t2 < 0) || (t1 > 1 && t2 > 1)) ) {
          Matrix2d C1;
          Matrix2d C2;
          Vector2d p1_prime = p1+t1*pstar;
          Vector2d p2_prime = p1+t2*pstar;
          //interpolate covariance
          if (t1 < 0) {
            C1 = master_vectorMap[j].p1_cov;
          }
          else if (t1 > 1) {
            C1 = master_vectorMap[j].p2_cov;
          }
          else {
            C1 = (1-t1)*master_vectorMap[j].p1_cov + t1*master_vectorMap[j].p2_cov;
          }
          if (t2 < 0) {
            C2 = master_vectorMap[j].p1_cov;
          }
          else if (t2 > 1) {
            C2 = master_vectorMap[j].p2_cov;
          }
          else {
            C2 = (1-t2)*master_vectorMap[j].p1_cov + t2*master_vectorMap[j].p2_cov;
          }

          double chi_squared1 = (p1_prime-new_p1).transpose()*(C1 + new_p1_cov).inverse()*(p1_prime-new_p1);
          double chi_squared2 = (p2_prime-new_p2).transpose()*(C2 + new_p2_cov).inverse()*(p2_prime-new_p2);
          pstar.normalize();
          new_pstar.normalize();
          double dir_check = fabs(pstar.dot(new_pstar));
          if (chi_squared1 < 50 && chi_squared2 < 50 && dir_check > 0.5) {
            if (self_merge) { used_indices.push_back(i); }
            merged = true;
            bool merge_set_exists = false;
            for (size_t k=0; k<merge_map.size(); k++) {
              if (merge_map[k].first == int(j)) {
                merge_map[k].second.push_back(new_vectors[i]);
                extrema[k].second.push_back(t1);
                extrema[k].second.push_back(t2);
                merge_set_exists = true;
              }
            }
            if (!merge_set_exists) {
              vector<mappingVector> merge_set;
              merge_set.push_back(new_vectors[i]);
              pair<int, vector<mappingVector> > individual_merge_map = make_pair(j, merge_set);
              merge_map.push_back(individual_merge_map);

              vector<double> new_extrema;
              new_extrema.push_back(t1);
              new_extrema.push_back(t2);
              pair<int, vector<double> > individual_extrema = make_pair(j, new_extrema);
              extrema.push_back(individual_extrema);
            }
            j = master_vectorMap.size(); //advance to next new vector so we don't double-merge
          }
        }
      }
    }
    //add new lines to master vector map
    if (!merged && !self_merge) {
      new_master_vectorMap.push_back(new_vectors[i]);
    }
  }

  //transfer old lines which were not merged to new master vector map
  for (size_t i=0; i<master_vectorMap.size(); i++) {
    bool merged = false;
    for (size_t j=0; j<merge_map.size(); j++) {
      if (merge_map[j].first == int(i)) {
        merged = true;
      }
    }
    bool used = false;
    for (size_t j=0; j<used_indices.size(); j++) {
      if (used_indices[j] == int(i)) {
        used = true;
      }
    }
    if (!merged && !used) {
      new_master_vectorMap.push_back(master_vectorMap[i]);
    }
  }

  for (size_t i=0; i<merge_map.size(); i++) {
    int total_mass = master_vectorMap[merge_map[i].first].mass;
    for (size_t j=0; j<merge_map[i].second.size(); j++) {
      total_mass += merge_map[i].second[j].mass;
    }
    mappingVector merged_vector;
    mappingVector parent_vector = master_vectorMap[merge_map[i].first];
    Vector2d pstar = (parent_vector.p2-parent_vector.p1);
    int M1 = parent_vector.mass;
    merged_vector.mass = total_mass;
    Vector2d merged_p_bar = (double(M1)/double(total_mass))*parent_vector.p_bar;
    Matrix2d merged_scatter = (double(M1)/double(total_mass))*parent_vector.scatter;
    Matrix2d merged_p1_cov = (double(M1)/double(total_mass))*parent_vector.p1_cov;
    Matrix2d merged_p2_cov = (double(M1)/double(total_mass))*parent_vector.p2_cov;
    for (size_t j=0; j<merge_map[i].second.size(); j++) {
      mappingVector new_vector = merge_map[i].second[j];
      int M2 = new_vector.mass;
      merged_p_bar += (double(M2)/double(total_mass))*new_vector.p_bar;
      merged_scatter += (double(M2)/double(total_mass))*new_vector.scatter;
      merged_p1_cov += (double(M2)/double(total_mass))*new_vector.p1_cov;
      merged_p2_cov += (double(M2)/double(total_mass))*new_vector.p2_cov;
    }
    merged_vector.p_bar = merged_p_bar;
    merged_vector.scatter = merged_scatter;
    merged_vector.p1_cov = merged_p1_cov;
    merged_vector.p2_cov = merged_p2_cov;

    Eigen::EigenSolver<Matrix2d> es;
    Matrix2d S = merged_scatter - merged_p_bar*merged_p_bar.transpose();
    es.compute(S, true);
    double lambda1 = double(es.eigenvalues()[0].real());
    double lambda2 = double(es.eigenvalues()[1].real());
    Vector2d merged_dir;
    if (lambda1 > lambda2) {
      merged_dir(0) = es.eigenvectors().col(0)(0).real();
      merged_dir(1) = es.eigenvectors().col(0)(1).real();
    }
    else {
      merged_dir(0) = es.eigenvectors().col(1)(0).real();
      merged_dir(1) = es.eigenvectors().col(1)(1).real();
    }
    merged_dir.normalize();
    if (merged_dir.dot(pstar) < 0){
      merged_dir = -merged_dir;
    }

    double tmin = 0.0;
    double tmax = 1.0;
    for (size_t j=0; j<extrema[i].second.size(); j++) {
      tmax = max(tmax,extrema[i].second[j]);
      tmin = min(tmin,extrema[i].second[j]);
    }
    double len = (tmax - tmin)*pstar.norm();
    double t_p_bar = (merged_p_bar-parent_vector.p1).dot(pstar)/pstar.dot(pstar);
    Vector2d merged_p1 = merged_p_bar + (((tmin - t_p_bar)*len)/(tmax-tmin))*merged_dir;
    Vector2d merged_p2 = merged_p_bar + (((tmax - t_p_bar)*len)/(tmax-tmin))*merged_dir;
    merged_vector.p1 = merged_p1;
    merged_vector.p2 = merged_p2;

    new_master_vectorMap.push_back(merged_vector);
  }
  return new_master_vectorMap;
}

vector<mappingVector> selfMerge(vector<mappingVector> master_vectorMap, size_t size) {
  if (size == 0) {
    return master_vectorMap;
  }
  master_vectorMap = mergeNewVectors(master_vectorMap, master_vectorMap, true);
  if (size == master_vectorMap.size()) {
    return master_vectorMap;
  }
  else {
    return selfMerge(master_vectorMap, master_vectorMap.size());
  }
}

vector<mappingVector> updateVectorMap(SdfObject master_sdf, vector<mappingVector> master_vectorMap) {
  if (master_vectorMap.size() == 0) {
    return master_vectorMap;
  }
  double sigma = 0.04;
  int K = 1000;

  vector<mappingVector> new_master_vectorMap;
  cimg_library::CImg<float> weights = master_sdf.weights;
  const Vector2f origin = master_sdf.origin;
  double step = map_options_.image_resolution;
  float thresh = 0.95*255.0;
  for (size_t i=0; i<master_vectorMap.size(); i++) {
    vector<mappingVector> new_components;
    double t = 0.0;
    double t1(0), t2(0);
    bool high_weight = false;
    const Vector2d p1 = master_vectorMap[i].p1;
    const Vector2d p2 = master_vectorMap[i].p2;
    double T = (p2-p1).norm();
    const Vector2d line_dir = (p2-p1).normalized();
    Matrix2d rot;
    rot << 0, -1, 1, 0;
    Vector2d normal = rot*line_dir;
    mappingVector new_component;
    while (t <= T) {
      Vector2d curr_point = p1 + t*line_dir;
      double x = curr_point(0);
      double y = curr_point(1);
      double ix = round( (x - origin(0))/map_options_.image_resolution );
      double iy = round( (y - origin(1))/map_options_.image_resolution );
      if (weights(ix,iy) < thresh && high_weight) {
        t2 = t;
        if (int((t2-t1)*master_vectorMap[i].mass) > 10) {
          //turn off
          high_weight = false;
          t2 = t;
          new_component.p2 = curr_point;
          new_component.mass = int((t2-t1)*master_vectorMap[i].mass);

          vector<double> p1xs;
          vector<double> p1ys;
          vector<double> p2xs;
          vector<double> p2ys;
          int NI = new_component.mass;
          if (NI > 100000) {
            NI = 100000;
          }
          vector<Vector2d> bootstrap;
          for (int j=0; j<K; j++) {
            bootstrap.clear();
            float alpha;
            float noise;
            Vector2d point;
            for (int k=0; k<NI; k++) {
              alpha = frand(0.0, t2-t1);
              noise = randn(sigma, 0.0);
              point = p1 + (t1 + alpha)*line_dir + noise*normal;
              bootstrap.push_back(point);
            }
            double* data = NULL;
            data = new double[2*NI];
            Vector2d cm(0.0,0.0);
            Vector2d ip;
            for (int k=0; k<NI; k++) {
              ip = bootstrap[k];
              data[2*k] = ip(0);
              data[2*k+1] = ip(1);
              cm += ip;
            }
            cm = cm/double(NI);

            double CM[2];
            CM[0] = cm(0);
            CM[1] = cm(1);
            double P1[2];
            double P2[2];
            P1[0] = new_component.p1(0);
            P1[1] = new_component.p1(1);
            P2[0] = new_component.p2(0);
            P2[1] = new_component.p2(1);

            vector<Vector2d> end_points = segFit(P1, P2, CM, data, NI);

            delete [] data;

            p1xs.push_back(end_points[0](0));
            p1ys.push_back(end_points[0](1));
            p2xs.push_back(end_points[1](0));
            p2ys.push_back(end_points[1](1));
          }

          double p1xbar = p1(0);
          double p1ybar = p1(1);
          double p2xbar = p2(0);
          double p2ybar = p2(1);
          size_t N = p1xs.size();

          double cov_x1x1 = 0.0;
          double cov_x1y1 = 0.0;
          double cov_y1y1 = 0.0;
          double cov_x2x2 = 0.0;
          double cov_x2y2 = 0.0;
          double cov_y2y2 = 0.0;
          for (size_t j=0; j<p1xs.size(); j++) {
            cov_x1x1 += (p1xbar-p1xs[j])*(p1xbar-p1xs[j]);
            cov_x1y1 += (p1xbar-p1xs[j])*(p1ybar-p1ys[j]);
            cov_y1y1 += (p1ybar-p1ys[j])*(p1ybar-p1ys[j]);
            cov_x2x2 += (p2xbar-p2xs[j])*(p2xbar-p2xs[j]);
            cov_x2y2 += (p2xbar-p2xs[j])*(p2ybar-p2ys[j]);
            cov_y2y2 += (p2ybar-p2ys[j])*(p2ybar-p2ys[j]);
          }
          cov_x1x1 = cov_x1x1/double(N);
          cov_x1y1 = cov_x1y1/double(N);
          cov_y1y1 = cov_y1y1/double(N);
          cov_x2x2 = cov_x2x2/double(N);
          cov_x2y2 = cov_x2y2/double(N);
          cov_y2y2 = cov_y2y2/double(N);

          Matrix2d p1_cov;
          p1_cov(0,0) = cov_x1x1;
          p1_cov(1,0) = cov_x1y1;
          p1_cov(0,1) = cov_x1y1;
          p1_cov(1,1) = cov_y1y1;
          Matrix2d p2_cov;
          p2_cov(0,0) = cov_x2x2;
          p2_cov(1,0) = cov_x2y2;
          p2_cov(0,1) = cov_x2y2;
          p2_cov(1,1) = cov_y2y2;


          //compute centroid, scatter matrix components
          Matrix2d Si = Matrix2d::Zero();
          Vector2d p_bar(0.0, 0.0);
          Vector2d p_bar2(0.0, 0.0);
          for (int j=0; j<NI; j++) {
            p_bar += bootstrap[j];
            Si += bootstrap[j]*bootstrap[j].transpose();
          }
          p_bar = p_bar/double(NI);
          Si = Si/double(NI);

          new_component.p1_cov = p1_cov;
          new_component.p2_cov = p2_cov;
          new_component.p_bar = p_bar;
          new_component.scatter = Si;

          new_components.push_back(new_component);
        }
      }
      else if (weights(ix,iy) >= thresh && !high_weight) {
        //turn on
        high_weight = true;
        t1 = t;
        new_component.p1 = curr_point;
      }
      t += step;
    }
    //terminated with line
    if (high_weight) {
      //turn off
      t2 = T;
      if (fabs((t2-t1)-T) < 2*map_options_.image_resolution) {
        new_master_vectorMap.push_back(master_vectorMap[i]);
      }
      else if (int((t2-t1)*master_vectorMap[i].mass) > 10) {
        new_component.p2 = p2;
        new_component.mass = int((t2-t1)*master_vectorMap[i].mass);

        vector<double> p1xs;
        vector<double> p1ys;
        vector<double> p2xs;
        vector<double> p2ys;
        int NI = new_component.mass;
        if (NI > 100000) {
          NI = 100000;
        }
        vector<Vector2d> bootstrap;
        for (int j=0; j<K; j++) {
          bootstrap.clear();
          float alpha;
          float noise;
          Vector2d point;
          for (int k=0; k<NI; k++) {
            alpha = frand(0.0, t2-t1);
            noise = randn(sigma, 0.0);
            point = p1 + (t1 + alpha)*line_dir + noise*normal;
            bootstrap.push_back(point);
          }

          double* data = NULL;
          data = new double[2*NI];
          Vector2d cm(0.0,0.0);
          Vector2d ip;
          for (int k=0; k<NI; k++) {
            ip = bootstrap[k];
            data[2*k] = ip(0);
            data[2*k+1] = ip(1);
            cm += ip;
          }
          cm = cm/double(NI);

          double CM[2];
          CM[0] = cm(0);
          CM[1] = cm(1);
          double P1[2];
          double P2[2];
          P1[0] = new_component.p1(0);
          P1[1] = new_component.p1(1);
          P2[0] = new_component.p2(0);
          P2[1] = new_component.p2(1);

          vector<Vector2d> end_points = segFit(P1, P2, CM, data, NI);

          delete [] data;

          p1xs.push_back(end_points[0](0));
          p1ys.push_back(end_points[0](1));
          p2xs.push_back(end_points[1](0));
          p2ys.push_back(end_points[1](1));
        }

        double p1xbar = p1(0);
        double p1ybar = p1(1);
        double p2xbar = p2(0);
        double p2ybar = p2(1);
        size_t N = p1xs.size();

        double cov_x1x1 = 0.0;
        double cov_x1y1 = 0.0;
        double cov_y1y1 = 0.0;
        double cov_x2x2 = 0.0;
        double cov_x2y2 = 0.0;
        double cov_y2y2 = 0.0;

        for (size_t j=0; j<p1xs.size(); j++) {
          cov_x1x1 += (p1xbar-p1xs[j])*(p1xbar-p1xs[j]);
          cov_x1y1 += (p1xbar-p1xs[j])*(p1ybar-p1ys[j]);
          cov_y1y1 += (p1ybar-p1ys[j])*(p1ybar-p1ys[j]);
          cov_x2x2 += (p2xbar-p2xs[j])*(p2xbar-p2xs[j]);
          cov_x2y2 += (p2xbar-p2xs[j])*(p2ybar-p2ys[j]);
          cov_y2y2 += (p2ybar-p2ys[j])*(p2ybar-p2ys[j]);
        }
        cov_x1x1 = cov_x1x1/double(N);
        cov_x1y1 = cov_x1y1/double(N);
        cov_y1y1 = cov_y1y1/double(N);
        cov_x2x2 = cov_x2x2/double(N);
        cov_x2y2 = cov_x2y2/double(N);
        cov_y2y2 = cov_y2y2/double(N);

        Matrix2d p1_cov;
        p1_cov(0,0) = cov_x1x1;
        p1_cov(1,0) = cov_x1y1;
        p1_cov(0,1) = cov_x1y1;
        p1_cov(1,1) = cov_y1y1;
        Matrix2d p2_cov;
        p2_cov(0,0) = cov_x2x2;
        p2_cov(1,0) = cov_x2y2;
        p2_cov(0,1) = cov_x2y2;
        p2_cov(1,1) = cov_y2y2;


        //compute centroid, scatter matrix components
        Matrix2d Si = Matrix2d::Zero();
        Vector2d p_bar(0.0, 0.0);
        Vector2d p_bar2(0.0, 0.0);
        for (int j=0; j<NI; j++) {
          p_bar += bootstrap[j];
          Si += bootstrap[j]*bootstrap[j].transpose();
        }
        p_bar = p_bar/double(NI);
        Si = Si/double(NI);

        new_component.p1_cov = p1_cov;
        new_component.p2_cov = p2_cov;
        new_component.p_bar = p_bar;
        new_component.scatter = Si;

        new_components.push_back(new_component);
      }
    }
    for (size_t j=0; j<new_components.size(); j++) {
      new_master_vectorMap.push_back(new_components[j]);
    }
  }
  vector<mappingVector> nmvm;
  for (size_t i=0; i<new_master_vectorMap.size(); i++) {
    double seg_len = (new_master_vectorMap[i].p2 - new_master_vectorMap[i].p1).norm();
    if (seg_len > 0.05) {
      nmvm.push_back(new_master_vectorMap[i]);
    }
  }
  return nmvm;
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
  printf("Probabilistic Object Map Builder\n");
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
