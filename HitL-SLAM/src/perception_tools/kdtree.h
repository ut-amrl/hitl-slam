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

#include <eigen3/Eigen/Core>
#include <vector>

#ifndef KDTREE_H
#define KDTREE_H

template<typename T, unsigned int K>
struct KDNodeValue {
  Eigen::Matrix<T, K, 1> point;
  Eigen::Matrix<T, K, 1> normal;
  int index;
  KDNodeValue() : index(0) {}
  KDNodeValue(const Eigen::Matrix<T, K, 1>& _point,
              const Eigen::Matrix<T, K, 1>& _normal,
              int _index) : point(_point), normal(_normal), index(_index) {}
};


template<typename T, unsigned int K>
class KDTree {
 public:
  // Default constructor: Creates an empty KDTree with uninitialized root node.
  KDTree() : splitting_dimension_(0),
      left_tree_(NULL),
      right_tree_(NULL),
      parent_tree_(NULL)  {}

  // Destructor: Deletes children trees.
  ~KDTree();

  // Disallow the copy constructor.
  KDTree(const KDTree<T, K>& other);

  // Disallow the assignment operator.
  KDTree<T, K>& operator=(const KDTree<T, K>& other);

  // Construct the KDTree using the @values provided.
  explicit KDTree(const std::vector<KDNodeValue<T, K> >& values);

  // Rebuild the KDTree using the @values provided.
  KDTree<T, K>* BuildKDTree(std::vector<KDNodeValue<T, K> > values);

  // Finds the nearest point in the KDTree to the provided &point. Returns
  // the distance to the nearest neighbor found if one is found within the
  // specified threshold. Euclidean L2 norm is used as the distance metric for
  // nearest neighbor search. This is useful for (for example) point to
  // point ICP.
  T FindNearestPoint(const Eigen::Matrix<T, K, 1>& point,
                     const T& threshold,
                     KDNodeValue<T, K>* neighbor_node);

  // Finds the set of points in the KDTree closer than the distance &threshold
  // to the provided &point. Euclidean L2 norm is used as the distance metric
  // for nearest neighbor search.
  void FindNeighborPoints(const Eigen::Matrix<T, K, 1>& point,
                          const T& threshold,
                          std::vector<KDNodeValue<T, K> >* neighbor_points);

  // Finds the nearest point and normal in the KDTree to the provided &point.
  // Returns the distance to the nearest neighbor if one is found within the
  // specified threshold. Distance from the nearest neighbor along the
  // associated normal is used as the distance metric. This is useful for
  // (for example) point to plane ICP.
  T FindNearestPointNormal(const Eigen::Matrix<T, K, 1>& point,
                           const T& threshold,
                           KDNodeValue<T, K>* neighbor_node);

 private:
  // The dimension along which the split is, as this node.
  int splitting_dimension_;

  KDNodeValue<T, K> value_;

  KDTree* left_tree_;
  KDTree* right_tree_;
  KDTree* parent_tree_;
};

#endif  // KDTREE_H
