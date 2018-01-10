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
// A general K-Dimension Tree (KDTree) implementation.

#include "kdtree.h"

#include <stdio.h>
#include <float.h>
#include <algorithm>
#include <vector>
#include "glog/logging.h"

using std::vector;
using Eigen::Matrix;

namespace {

template<typename T> inline T min(const T& a, const T& b) {
  return (a < b) ? a : b;
}

template<typename T, unsigned int K>
int GetSplittingPlane(const vector<KDNodeValue<T, K> >& values) {
  Matrix<T, K, 1> mean;
  Matrix<T, K, 1> std_dev;
  mean.setZero();
  std_dev.setZero();

  // Compute mean along all dimensions.
  for (unsigned int i = 0; i < values.size(); ++i) {
    mean = mean + values[i].point;
  }
  mean = mean / static_cast<T>(values.size());

  // Compute standard deviation along all dimensions.
  for (unsigned int i = 0; i < values.size(); ++i) {
    for (unsigned int j = 0; j < K; ++j) {
      std_dev(j) = std_dev(j) +
          (values[i].point(j)-mean(j)) * (values[i].point(j)-mean(j));
    }
  }

  // Chose the splitting plane along the dimension that has the greatest spread,
  // as indicated by the standard deviation along that dimension.
  int splitting_plane = 0;
  T max_std_dev(0.0);
  for (unsigned int j = 0; j < K; ++j) {
    if (std_dev(j) > max_std_dev) {
      splitting_plane = j;
      max_std_dev = std_dev(j);
    }
  }
  return splitting_plane;
}

// Comparator functor used for sorting points based on their values along a
// particular dimension.
template<typename T, unsigned int K>
struct VectorComparator {
  const unsigned int comparator_dimension;
  explicit VectorComparator(int dimension) : comparator_dimension(dimension) {}
  bool operator()(const KDNodeValue<T, K>& v1,
                  const KDNodeValue<T, K>& v2) {
    return v1.point(comparator_dimension) < v2.point(comparator_dimension);
  }
};

}  // namespace

template<typename T, unsigned int K>
KDTree<T, K>::KDTree(const vector<KDNodeValue<T, K> >& values)
    : splitting_dimension_(0),
    left_tree_(NULL),
    right_tree_(NULL),
    parent_tree_(NULL) {
  BuildKDTree(values);
}

template<typename T, unsigned int K>
KDTree<T, K>::~KDTree() {
  if (left_tree_ != NULL) {
    delete left_tree_;
    left_tree_ = NULL;
  }
  if (right_tree_ != NULL) {
    delete right_tree_;
    right_tree_ = NULL;
  }
}

template<typename T, unsigned int K>
KDTree< T, K >* KDTree<T, K>::BuildKDTree(
    vector<KDNodeValue<T, K> > values) {

  // Determine splitting plane.
  splitting_dimension_ = GetSplittingPlane<T, K>(values);
  VectorComparator<T, K> comparator(splitting_dimension_);

  // Sort the points in order of their distance from the splitting plane.
  std::sort(values.begin(), values.end(), comparator);

  // Take the median point and make that the root.
  unsigned int ind = values.size() / 2;
  value_ = values[ind];

  // Insert the KD tree of the left hand part of the list to the left node.
  left_tree_ = NULL;
  if (ind > 0) {
    vector<KDNodeValue<T, K> > points_left(values.begin(),
                                           values.begin() + ind);
    left_tree_ = new KDTree<T, K>(points_left);
    left_tree_->parent_tree_ = this;
  }

  // Insert the KD tree of the right hand part of the list to the right node.
  right_tree_ = NULL;
  if (ind < values.size() - 1) {
    vector<KDNodeValue<T, K> > points_right(values.begin() + ind + 1,
                                            values.end());
    right_tree_ = new KDTree<T, K>(points_right);
    right_tree_->parent_tree_ = this;
  }
  return this;
}

template<typename T, unsigned int K>
T KDTree<T, K>::FindNearestPointNormal(
    const Eigen::Matrix< T, K, 1  >& point,
    const T& threshold,
    KDNodeValue< T, K >* neighbor_node) {
  T current_best_dist = FLT_MAX;
  if ((value_.point - point).squaredNorm() < threshold * threshold) {
    *neighbor_node = value_;
    current_best_dist = fabs(value_.normal.dot(point - value_.point));
    if (current_best_dist < FLT_MIN) {
      return T(0.0);
    }
  }

  // The signed distance of the point from the splitting plane.
  const T point_distance(point(splitting_dimension_) -
      value_.point(splitting_dimension_));

  // Follow the query point down the tree and return the best neighbor down
  // that branch.
  KDTree<T, K>* other_tree(NULL);
  if (point_distance <= 0.0 && left_tree_ != NULL) {
    KDNodeValue<T, K> left_tree_neighbor_node;
    T left_tree_dist = left_tree_->FindNearestPointNormal(
        point, threshold, &left_tree_neighbor_node);
    if (left_tree_dist < current_best_dist) {
      current_best_dist = left_tree_dist;
      *neighbor_node = left_tree_neighbor_node;
    }
    other_tree = right_tree_;
  }
  if (point_distance >= 0.0 && right_tree_ != NULL) {
    KDNodeValue<T, K> right_tree_neighbor_node;
    T right_tree_dist = right_tree_->FindNearestPointNormal(
      point, threshold, &right_tree_neighbor_node);
    if (right_tree_dist < current_best_dist) {
      current_best_dist = right_tree_dist;
      *neighbor_node = right_tree_neighbor_node;
    }
    other_tree = left_tree_;
  }

  // Check if the point is closer to the splitting plane than the current
  // best neighbor. If so, the other side needs to be checked as well.
  if (other_tree != NULL && point_distance != 0.0 &&
      fabs(point_distance) < min(current_best_dist, threshold)) {
    KDNodeValue<T, K> other_tree_neighbor_node;
    T other_tree_dist = other_tree->FindNearestPointNormal(
        point, threshold, &other_tree_neighbor_node);
    if (other_tree_dist < current_best_dist) {
      current_best_dist = other_tree_dist;
      *neighbor_node = other_tree_neighbor_node;
    }
  }

  return current_best_dist;
}

template<typename T, unsigned int K>
void KDTree<T, K>::FindNeighborPoints(
    const Eigen::Matrix<T, K, 1>& point,
    const T& threshold,
    std::vector<KDNodeValue<T, K> >* neighbor_points) {
  T current_dist = (value_.point - point).norm();
  if (current_dist < threshold) neighbor_points->push_back(value_);

  // The signed distance of the point from the splitting plane.
  const T point_distance(point(splitting_dimension_) -
      value_.point(splitting_dimension_));

  if (point_distance < threshold && left_tree_ != NULL) {
    left_tree_->FindNeighborPoints(point, threshold, neighbor_points);
  }

  if (point_distance > -threshold && right_tree_ != NULL) {
    right_tree_->FindNeighborPoints(point, threshold, neighbor_points);
  }
}

template<typename T, unsigned int K>
T KDTree<T, K>::FindNearestPoint(
    const Eigen::Matrix< T, K, 1  >& point,
    const T& threshold,
    KDNodeValue<T, K>* neighbor_node) {
  T current_best_dist = (value_.point - point).norm();
  *neighbor_node = value_;
  if (current_best_dist < FLT_MIN) {
    return T(0.0);
  }

  // The signed distance of the point from the splitting plane.
  const T point_distance(point(splitting_dimension_) -
      value_.point(splitting_dimension_));

  // Follow the query point down the tree and return the best neighbor down
  // that branch.
  KDTree<T, K>* other_tree(NULL);
  if (point_distance <= 0.0 && left_tree_ != NULL) {
    KDNodeValue<T, K> left_tree_neighbor_node;
    const T left_tree_dist = left_tree_->FindNearestPoint(
        point, min(current_best_dist, threshold), &left_tree_neighbor_node);
    if (left_tree_dist < current_best_dist) {
      current_best_dist = left_tree_dist;
      *neighbor_node = left_tree_neighbor_node;
    }
    other_tree = right_tree_;
  }
  if (point_distance >= 0.0 && right_tree_ != NULL) {
    KDNodeValue<T, K> right_tree_neighbor_node;
    const T right_tree_dist = right_tree_->FindNearestPoint(
        point, min(current_best_dist, threshold), &right_tree_neighbor_node);
    if (right_tree_dist < current_best_dist) {
      current_best_dist = right_tree_dist;
      *neighbor_node = right_tree_neighbor_node;
    }
    other_tree = left_tree_;
  }

  // Check if the point is closer to the splitting plane than the current
  // best neighbor. If so, the other side needs to be checked as well.
  if (other_tree != NULL && point_distance != 0.0 &&
      fabs(point_distance) < min(current_best_dist, threshold)) {
    KDNodeValue<T, K> other_tree_neighbor_node;
    const T other_tree_dist = other_tree->FindNearestPoint(
        point, min(current_best_dist, threshold), &other_tree_neighbor_node);
    if (other_tree_dist < current_best_dist) {
      current_best_dist = other_tree_dist;
      *neighbor_node = other_tree_neighbor_node;
    }
  }

  return current_best_dist;
}

// Explicit instantiation of template classes minimizes redundant compilation,
// separates declarations from definitions, and prevents use of the class in
// ways other than those that have been tested.

// Explicit instantiation of KDTree class for 3D double values.
template struct KDNodeValue<double, 3>;
template class KDTree<double, 3>;

// Explicit instantiation of KDTree class for 3D float values.
template struct KDNodeValue<float, 3>;
template class KDTree<float, 3>;

// Explicit instantiation of KDTree class for 2D double values.
template struct KDNodeValue<double, 2>;
template class KDTree<double, 2>;

// Explicit instantiation of KDTree class for 2D float values.
template struct KDNodeValue<float, 2>;
template class KDTree<float, 2>;
