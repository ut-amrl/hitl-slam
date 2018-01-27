class LongTermVectorMap {
  public:
    void init();
    void Curate(std::vector<perception_2d::Pose2Df> poses,
                std::vector<perception_2d::PointCloudf> point_clouds,
                Eigen::Affine2f map_transform);
  private:
    SDFFilter();
    
    
};
