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
#include "eigen_helper.h"
#include "helpers.h"
#include "kdtree.h"
#include "perception_tools/perception_2d.h"
#include "openmp_utils.h"
#include "popt_pp.h"
#include "timer.h"
#include "glog/logging.h"
#include "ceres/ceres.h"

#include "CImg/CImg.h"
#include "png.h"

#define cimg_use_png


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
using std::fwrite;
using std::fprintf;
using std::list;
using std::make_pair;
using std::map;
using std::pair;
using std::size_t;
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


void LongTermVectorMap::init() {

}

/* ################################ STRUCTS ################################# */

struct totalPointCloud {
  PointCloudf point_cloud;
  NormalCloudf normal_cloud;
  vector<Pose2Df> poses;
};


struct worldMap {
  SdfObject sdf;
  vector<mappingVector> vectorMap;
};





/* ################################ COMPUTE UNCERTAINTY ################################# */

void LongTermVectorMap::resampleSegmentUncertainty(float sigma,
                                                   MappingVector* new_vector) {
  vector<float> p1xs;
  vector<float> p1ys;
  vector<float> p2xs;
  vector<float> p2ys;
  // limit how much we resample for time efficiency
  
  int mass = min((*new_vector).mass, 100000);
  //if (mass > 100000) {
  //  mass = 100000;
  //}
  vector<Vector2d> bootstrap;
  Vector2f p1 = (*new_vector).p1;
  Vector2f p2 = (*new_vector).p2;
  float len = (p2 - p1).norm();
  Vector2f line_dir = (p2 - p1).normalize();
  Vector2f normal = Perp2(line_dir);
  for (int i = 0; i < num_samples_; ++i) {
    bootstrap.clear();
    float alpha;
    float noise;
    Vector2d point;
    for (int j = 0; j < mass; j++) {
      alpha = frand(0.0, len);
      noise = randn(sigma, 0.0);
      point = p1 + alpha * line_dir + noise * normal;
      bootstrap.push_back(point);
    }
    //double* data = NULL;
    double* data = new double[2 * mass];
    Vector2d cm(0.0,0.0);
    Vector2d ip;
    for (int j = 0; j < mass; ++j) {
      ip = bootstrap[j];
      data[2 * j] = ip(0);
      data[2 * j + 1] = ip(1);
      cm += ip;
    }
    cm = cm/double(mass);

    double CM[2];
    CM[0] = cm(0);
    CM[1] = cm(1);
    double P1[2];
    double P2[2];
    P1[0] = p1(0);
    P1[1] = p1(1);
    P2[0] = p2(0);
    P2[1] = p2(1);

    vector<Vector2d> end_points = segFit(P1, P2, CM, data, mass);

    delete [] data;

    p1xs.push_back(float(end_points[0](0)));
    p1ys.push_back(float(end_points[0](1)));
    p2xs.push_back(float(end_points[1](0)));
    p2ys.push_back(float(end_points[1](1)));
  }

  float p1xbar = p1(0);
  float p1ybar = p1(1);
  float p2xbar = p2(0);
  float p2ybar = p2(1);
  
  ASSERT_EQ(p1xs.size(), num_samples_);

  float cov_x1x1 = 0.0;
  float cov_x1y1 = 0.0;
  float cov_y1y1 = 0.0;
  float cov_x2x2 = 0.0;
  float cov_x2y2 = 0.0;
  float cov_y2y2 = 0.0;
  for (size_t i = 0; i < num_samples_; ++i) {
    cov_x1x1 += (p1xbar - p1xs[i]) * (p1xbar - p1xs[i]);
    cov_x1y1 += (p1xbar - p1xs[i]) * (p1ybar - p1ys[i]);
    cov_y1y1 += (p1ybar - p1ys[i]) * (p1ybar - p1ys[i]);
    cov_x2x2 += (p2xbar - p2xs[i]) * (p2xbar - p2xs[i]);
    cov_x2y2 += (p2xbar - p2xs[i]) * (p2ybar - p2ys[i]);
    cov_y2y2 += (p2ybar - p2ys[i]) * (p2ybar - p2ys[i]);
  }
//  cov_x1x1 = cov_x1x1/float(num_samples_);
//  cov_x1y1 = cov_x1y1/float(num_samples_);
//  cov_y1y1 = cov_y1y1/float(num_samples_);
//  cov_x2x2 = cov_x2x2/float(num_samples_);
//  cov_x2y2 = cov_x2y2/float(num_samples_);
//  cov_y2y2 = cov_y2y2/float(num_samples_);

  Matrix2f p1_cov;
  p1_cov(0,0) = cov_x1x1;
  p1_cov(1,0) = cov_x1y1;
  p1_cov(0,1) = cov_x1y1;
  p1_cov(1,1) = cov_y1y1;
  p1_cov = p1_cov / float(num_samples_);
  
  Matrix2f p2_cov;
  p2_cov(0,0) = cov_x2x2;
  p2_cov(1,0) = cov_x2y2;
  p2_cov(0,1) = cov_x2y2;
  p2_cov(1,1) = cov_y2y2;
  p2_cov = p2_cov / float(num_samples_);

  new_vector->p1_cov = p1_cov;
  new_vector->p2_cov = p2_cov;

  // Use last sample to compute centroid, scatter matrix components.
  Matrix2f Si = Matrix2f::Zero();
  Vector2f p_bar(0.0, 0.0);
  for (int i = 0; i < mass; ++i) {
    p_bar += bootstrap[i];
    Si += bootstrap[i] * bootstrap[i].transpose();
  }
  p_bar = p_bar / float(mas);
  Si = Si / float(mass);

  new_vector->p_bar = p_bar;
  new_vector->scatter = Si;
}





void LongTermVectorMap::computeVectorUncertainty(Vector2d p1, Vector2d p2, 
                                                 vector<Vector2d> inlier_points, 
                                                 vector<Pose2Df> poses) {
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







  //TODO: turn into function call to resampleSegmentUncertainty

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















/* ################################ UPDATE / MERGE MAPS ################################# */

vector<mappingVector> mergeNewVectors(vector<mappingVector> new_vectors, vector<mappingVector> master_vectorMap, bool self_merge) {
  if (master_vectorMap.size() == 0) {
    return new_vectors;
  }
  vector<mappingVector> new_master_vector_map;
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
      new_master_vector_map.push_back(new_vectors[i]);
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
      new_master_vector_map.push_back(master_vectorMap[i]);
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

    new_master_vector_map.push_back(merged_vector);
  }
  return new_master_vector_map;
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









// Prune old vectors which are discovered to be STFS. 
// Resample covariances for any vectors which are updated
void LongTermVectorMap::pruneVectorMap() {
  if (vector_map_.size() == 0) { // No features were extracted
    return;
  }

  double sigma = 0.04;

  vector<MappingVector> new_master_vector_map;
  cimg_library::CImg<float> weights = sdf_map_.getWeights();
  const Vector2f origin = sdf_map_.getOrigin();
  double step = sdf_map_.getImageResolution();
  float thresh = 0.95*255.0;
  for (size_t i = 0; i < r_vector_map.size(); i++) {
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
      if (weights(ix,iy) >= thresh && !high_weight) {
        //turn on
        high_weight = true;
        t1 = t;
        new_component.p1 = curr_point;
      }
      else if (weights(ix,iy) < thresh && high_weight) {
        t2 = t;
        if (int(((t2-t1) / T) * master_vectorMap[i].mass) > 10) {
          //turn off
          high_weight = false;
          t2 = t;
          new_component.p2 = curr_point;
          new_component.mass = int(((t2-t1) / T) * master_vectorMap[i].mass);

          resampleSegmentUncertainty(sigma, &new_component);

          new_components.push_back(new_component);
        }
      }
      t += step;
    }
    //terminated with line
    if (high_weight) {
      //turn off
      t2 = T;
      // no change to original vector
      if (fabs((t2-t1)-T) < 2*map_options_.image_resolution) {
        new_master_vector_map.push_back(master_vectorMap[i]);
      }
      // latter part of line is kept
      else if (int(((t2-t1) / T) * master_vectorMap[i].mass) > 10) {
        new_component.p2 = p2;
        new_component.mass = int(((t2-t1) / T) * master_vectorMap[i].mass);

        resampleSegmentUncertainty(sigma, &new_component);
        
        new_components.push_back(new_component);
      }
    }
    for (size_t j=0; j<new_components.size(); j++) {
      new_master_vector_map.push_back(new_components[j]);
    }
  }
  vector<mappingVector> nmvm;
  for (size_t i=0; i<new_master_vector_map.size(); i++) {
    double seg_len = (new_master_vector_map[i].p2 - new_master_vector_map[i].p1).norm();
    if (seg_len > 0.05) {
      nmvm.push_back(new_master_vector_map[i]);
    }
  }
  return nmvm;
}


























/* ################################ RUN ################################# */

void LongTermVectorMapping::Curate(vector<Pose2Df> poses,
                                   vector<PointCloudf> point_clouds,
                                   vector<NormalCloudf> normal_clouds,
                                   Affine2f map_transform) {
  
  CHECK_GT(point_cloud.size(), 0);
  CHECK_EQ(point_cloud.size(), normal_cloud.size());
  CHECK_EQ(point_cloud.size(), poses.size());

  // Transform everying into the given global frame
  for (size_t i=0; i<point_cloud.size(); i++) {
    point_cloud[i] = R*point_cloud[i] + T;
    poses[i].translation = R*poses[i].translation + T;
    normal_cloud[i] = R*normal_cloud[i] + T;
  }

  if (!sdf_init_) {
     sdf_map_.init(poses, point_clouds);
     sdf_init_ = true;

     //TODO: filter
     //      extract
  }
  else {
     SignedDistanceFunction new_sdf;
     new_sdf.init(poses, point_clouds);
     sdf_map_.update(new_sdf);

     // TODO: prune
     //       filter
     //       extract
     //       merge
     //       prune
     //       selfmerge
  }




  if (mode == 4) {

    

    // prune old vectors which are discovered to be STFS. 
    // resample covariances for any vectors which are updated
    std::cout << "Update Vector Map" << std::endl;
    master_vectorMap = pruneVectorMap(master_sdf, master_vectorMap);
    
    // self-contained in LTVM class
    std::cout << "Filter  SDF" << std::endl;
    totalPointCloud filtered_rpc = filter(poses, point_cloud, normal_cloud, master_sdf);
    
    // RANSAC class
    // TODO: pull out call to estimate uncertainty
    std::cout << "RANSAC" << std::endl;
    vector<mappingVector> new_vectors = localRANSAC(filtered_rpc.poses, filtered_rpc.point_cloud, filtered_rpc.normal_cloud);
    

    // TODO: what do?
    std::cout << "Merge New Lines" << std::endl;
    master_vectorMap = mergeNewVectors(new_vectors, master_vectorMap, false);
    
    // prune old vectors which are discovered to be STFS. 
    // resample covariances for any vectors which are updated
    std::cout << "Update Vector Map" << std::endl;
    master_vectorMap = pruneVectorMap(master_sdf, master_vectorMap);
    
    // Recursivley call mergeNewVectors in order to merge any vectors
    // which remain very similar. This is necessary since mapping from
    // new vectors to existing ones has no onto / on-to-one guarantee.
    std::cout << "Self Merge" << std::endl;
    master_vectorMap = selfMerge(master_vectorMap, master_vectorMap.size());
  }


  worldMap WM;
  WM.sdf = master_sdf;
  WM.vectorMap = master_vectorMap;
  return WM;

}

