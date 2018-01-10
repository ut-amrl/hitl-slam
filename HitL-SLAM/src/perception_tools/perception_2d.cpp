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
// Copyright 2015 joydeepb@cs.umass.edu
// Computer Science Department, University of Massachusetts Amherst
//
// Data types and helper functions for perception in 2D.

//#include "perception_tools/perception_2d.h"
#include "perception_2d.h"

#include <math.h>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using Eigen::Rotation2Df;
using Eigen::Vector2f;

namespace perception_2d {

void GenerateNormals(const float max_point_neighbor_distance,
                     PointCloudf* point_cloud_ptr,
                     NormalCloudf* normal_cloud_ptr) {
  static const Rotation2Df kRotateBy90(M_PI_2);
  PointCloudf& point_cloud = *point_cloud_ptr;
  NormalCloudf& normal_cloud = *normal_cloud_ptr;
  normal_cloud.resize(point_cloud.size());
  for (unsigned int i = 0; i <  point_cloud.size(); ++i) {
    float count = 0.0;
    Vector2f normal(0.0, 0.0);
    if (i > 0 &&
        (point_cloud[i] - point_cloud[i-1]).norm() <
        max_point_neighbor_distance) {
      normal += kRotateBy90 * (point_cloud[i] - point_cloud[i-1]).normalized();
      count += 1.0;
    }
    if (i < point_cloud.size() - 1 &&
        (point_cloud[i+1] - point_cloud[i]).norm() <
        max_point_neighbor_distance) {
      normal += kRotateBy90 * (point_cloud[i+1] - point_cloud[i]).normalized();
      count += 1.0;
    }
    if (count > 0.0) {
      normal = (normal / count).normalized();
      normal_cloud[i] = normal;
    } else {
      point_cloud.erase(point_cloud.begin() + i);
      normal_cloud.erase(normal_cloud.begin() + i);
      --i;
    }
  }
}

}  // namespace perception_2d

