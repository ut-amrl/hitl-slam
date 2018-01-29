#ifndef LONG_TERM_VECTOR_MAP_H
#define LONG_TERM_VECTOR_MAP_H

#include <eigen3/Eigen/Core>
#include "SDF.h"
#include "sdf_vector_maps.h"

class LongTermVectorMap {
  public:
    void init();
    void Curate(std::vector<perception_2d::Pose2Df> poses,
                std::vector<perception_2d::PointCloudf> point_clouds,
                std::vector<perception_2d::NormalCloudf> normal_clouds,
                const Eigen::Affine2f map_transform);
 
 

  private:
    void pruneVectorMap();

    
    
    std::vector<MappingVector> vector_map_;
    SignedDistanceFunction sdf_map_;
   

 
    bool sdf_init_ = false;

    size_t num_samples_ = 1000;
    
    
    
};

#endif
