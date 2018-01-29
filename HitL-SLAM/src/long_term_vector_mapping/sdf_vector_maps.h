#ifndef MAPPING_VECTOR_H
#define MAPPING_VECTOR_H

#include <eigen3/Eigen/Core>

struct MappingVector{
  int mass;
  Eigen::Vector2f p1;
  Eigen::Vector2f p2;
  Eigen::Vector2f p_bar;
  Eigen::Matrix2f scatter;
  Eigen::Matrix2f p1_cov;
  Eigen::Matrix2f p2_cov;
};

#endif
