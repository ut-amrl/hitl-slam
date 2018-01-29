class SignedDistanceFunction {

  public:
    void setParameters();
    void save(const string weights_image_file,
              const string values_image_file,
              const string sdf_image_file);
    void init();
    void update();

    cimg_library::CImg<float> getWeights();
    cimg_library::CImg<float> getValues();
    Eigen::Vector2f getOrigin();
 
    //TODO: all other setters

  private:

    // weights for each pixel
    cimg_library::CImg<float> weights_;
    
    // values for each pixel
    cimg_library::CImg<float> values_;

    // stores the number of updates each pixel has seen
    cimg_library::CImg<float> age_;

    // Location in R^2 of the top left corner of the weights and values images
    Eigen::Vector2f origin_;

    // Image used to display the surfaces learned by the sdf coloer coded
    // by whether they are on the inside or outside relative to observer
    cimg_library::CImg<uint8_t> display_;
    
    // 
    
    // meters per pixel in the SDF
    float image_resolution_ = 0.02;

    // Minimum and maximum weights which can be assigned to a pixel in the sdf.
    // The min is greater than 0 to avoid division by zero. The greater the
    // difference between min and max, the greater the difference between 
    // pixels exactly on the surface and those near the surface.
    float min_sdf_weight_ = 0.01;
    float max_sdf_weight_ = 1.0;
    
    // Minimum and maximum values which can be assigned to a pixel in the sdf.
    // More extreme values increase the effect of updates which are more
    // distant from object surfaces.
    float min_sdf_value_ = -0.2;
    float max_sdf_value_ = 0.2;

    // Buffer, in meters, added to the border of the image beyond
    // most extreme observation points
    float image_border_ = 0.3;

    // Parameters for the weight function. 
    // See paper "Curating Long-Term Vector Maps" (IROS 2016) for details.
    float eps_ = 0.02;
    float sigma_ = 0.02;

    // Dynamic features threshold
    float T_dynamic_ = 0.2;
   
    // Angular resolution of laser. 
    // A laser with a 180 degree FOV which casts 360 rays has a resolution of 0.5
    float laser_angular_resolution_ = M_PI * ((270.0 / 1024.0) / 180.0);
};
