void SignedDistanceFunction::setParameters() {
  image_resolution_ = 
  min_sdf_weight_ = 
  image_border_ = 
}

cimg_library::CImg<float> SignedDistanceFunctio::getWeights() {
  return weights_;
}

cimg_library::CImg<float> SignedDistanceFunctio::getValues() {
  return values_;
}

Eigen::Vector2f SignedDistanceFunctio::getOrigin() {
  return origin_;
}

//TODO incorporate age into init and update

void SignedDistanceFunction::init(const vector<Pose2Df> poses, 
                                  vector<PointCloudf> point_clouds) {

  const float pixel_half_width = sqrt(2.0) * image_resolution_;

  min_sdf_weight_ = 0.01;

  float min_x(FLT_MAX), min_y(FLT_MAX);
  float max_x(-FLT_MAX), max_y(-FLT_MAX);
  for (size_t i = 0; i < point_clouds.size(); ++i) {
    for (size_t j = 0; j < point_cloud[i].size(); ++j) {
      min_x = min(min_x, point_clouds[i][j](0));
      max_x = max(max_x, point_clouds[i][j](0));
      min_y = min(min_y, point_clouds[i][j](1));
      max_y = max(max_y, point_clouds[i][j](1));
    }
  }
  const float width = max_x - min_x + 2.0 * image_border_;
  const float height = max_y - min_y + 2.0 * image_border_;

  const unsigned int image_width = ceil(width / image_resolution_);
  const unsigned int image_height = ceil(height / image_resolution_);

  // Initialize an empty SDF image and weights image.
  weights_(image_width, image_height);
  values_(image_width, image_height);
  display_(image_width, image_height, 1, 3, 0);
  origin_ = Vector2f(min_x - image_border_, min_y - image_border_);

  // Initialize
  OMP_PARALLEL_FOR
  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      sdf_value_image(x, y) = min_sdf_value_;
      sdf_weights_image(x, y) = 0.0;
      sdf_display_image(x, y, 0, 0) = 0;
      sdf_display_image(x, y, 0, 1) = 0;
      sdf_display_image(x, y, 0, 2) = 0;
    }
  }

  // Compute weights and values
  OMP_PARALLEL_FOR
  for (size_t x = 0; x < image_width; ++x) {
    //if (x%20 == 0) {
    //  std::cout << x << "/" << image_width << std::endl;
    //}
    for (size_t y = 0; y < image_height; ++y) {
      for (size_t i = 0; i < point_clouds.size(); ++i) {
        for (size_t j = 0; j < point_clouds[i].size(); ++j) {
        const Vector2f point = point_clouds[i][j];
        const Vector2f source = poses[i].translation;
        const Vector2f line_dir = (point - source).normalized();
        const Vector2f line_perp = Perp2(line_dir);

        const Vector2f pixel_loc = image_origin + image_resolution_ * Vector2f(x, y);

        // If pixels are small relative to laser angular resolution
        // this will make sure pixels are still updated.
        const bool within_angle_tolerance = 
            ((fabs(line_perp.dot(point - pixel_loc)) / (point - source).norm()) < 
            0.5 * laser_angular_resolution);

        // If pixels are large relative to laser angular resolution
        // this will make sure observations are still used.
        const bool along_viewing_ray = 
            (fabs(line_perp.dot(pixel_loc - point)) < pixel_half_width);

        if (!along_viewing_ray && !within_angle_tolerance) continue;

        // Locations farther than the observed point along the viewing ray from the source have
        // a negative SDF value since they are "occupied".
        const float sdf_value = line_dir.dot(point - pixel_loc);

        // If pixel is along visible ray and between the point and its source: Update SDF
        const float truncated_sdf_value = min(sdf_value, max_sdf_value_);
        if (sdf_value >= min_sdf_value_) { //only revise values in front of or near range reading
          
          //exponential decaying weight
          float sdf_weight;
          if (fabs(truncated_sdf_value) <= eps_) {
            sdf_weight = max_sdf_weight_;
          }
          else if (sdf_value > max_sdf_value) {
            sdf_weight = min_sdf_weight_;
          }
          else {
            //tsdf == sdf in this case
            sdf_weight = 
                exp(-sigma_ * (truncated_sdf_value - eps_) * 
                              (truncated_sdf_value - eps_)); 
          }
          // Update values
          values_(x, y) =
              (values_(x, y) * weights_(x, y) + sdf_weight * truncated_sdf_value) /
              (weights_(x, y) + sdf_weight);
          // Update weights
          weights_(x, y) += sdf_weight;
        }
        //TODO: need to figure out a way of merging sdfs better across different visibilities
        //else {
        //  weights_(x,y) += 1.0;
        //}
      }
    }
  }

  //TODO: normalize across obs frequency?
  // Filter out dynamic observations 
  float max_weight = 0.0;
  for (size_t x = 0; x < width; ++x) {
    for (size_t y = 0; y < height; ++y) {
      max_weight = max(max_weight, weights_(x, y));
    }
  }
  for (size_t x = 0; x < image_width; ++x) {
    for (size_t y = 0; y < image_height; ++y) {
      if (weights_(x, y) > (T_dynamic_ * max_weight)) {
        weights_(x,y) = 255.0;
      }
      else {
        weights_(x,y) = 0.0;
      }
    }
  }
}

void SignedDistanceFunction::save(const string weights_image_file,
                                  const string values_image_file,
                                  const string sdf_image_file) {

  size_t width = weights_.width();
  size_t height = weights_.height();

  cimg_library::CImg<float> save_values(width, height);
 
  float max_val = FLT_MIN;
  for (size_t x = 0; x < width; ++x) {
    for (size_t y = 0; y < height; ++y) {
      max_val = max(max_val, values_(x, y));
    }
  }

  save_values = (255.0 / max_val) * values_;

  for (size_t x = 0; x < width; ++x) {
    for (size_t y = 0; y < height; ++y) {
      if (weights_(x, y)) {
        if (values_(x, y) >= 0.0) {
          display_(x, y, 0, 0) = 0;
          display_(x, y, 0, 1) = static_cast<uint8_t>(
          (values_(x, y) / max_sdf_value_) * 255.0);
          display_(x, y, 0, 2) = 0;
        }
        else {
          display_(x, y, 0, 0) = static_cast<uint8_t>(
          (values_(x, y) / min_sdf_value_) * 255.0);
          display_(x, y, 0, 1) = 0;
          display_(x, y, 0, 2) = 0;
        }
      }
      else {
        display_(x, y, 0, 0) = 0;
        display_(x, y, 0, 1) = 0;
        display_(x, y, 0, 2) = 255.0;
      }
    }
  }

  //const string weights_image_file = StringPrintf("sdf_weights.png");
  weights_.save_png(weights_image_file.c_str());

  //const string values_image_file = StringPrintf("sdf_values.png");
  save_values.save_png(values_image_file.c_str());

  //const string sdf_image_file = StringPrintf("sdf_display.png");
  display_.save_png(sdf_image_file.c_str());
}

void update(SignedDistanceFunction subsumed) {
  
  float O0x = origin_(0);
  float O0y = origin_(1);
  float O1x = subsumed.getOrigin()(0);
  float O1y = subsumed.getOrigin()(1);

  cimg_library::CImg<float> subsumed_weights = subsumed.getWeights();
  cimg_library::CImg<float> subsumed_values = subsumed.getValues();

  int h0 = weights_.height();
  int h1 = subsumed_weights.height();
  int w0 = weights_.width();
  int w1 = subsumed_weights.width();

  


  //TODO: make sure the origin is being properly set. be careful since image coords and robot coords may be different handedness / direction



  float new_height_world_coords = 
      max(max(O0y+h0*image_resolution_,O0y+h1*image_resolution_),
          max(O1y+h0*image_resolution_,O1y+h1*image_resolution_));
  float new_width_world_coords = 
      max(max(O0x+w0*image_resolution_,O0x+w1*image_resolution_),
          max(O1x+w0*image_resolution_,O1x+w1*image_resolution_));
  Vector2f new_origin(min(O0x, O1x), min(O0y, O1y));



  const unsigned int image_height = ceil(fabs(new_height_wc - new_origin(1))/image_resolution_);
  const unsigned int image_width = ceil(fabs(new_width_wc - new_origin(0))/image_resolution_);

  int ix_master = round((O0x - new_origin(0))/image_resolution_);
  int iy_master = round((O0y - new_origin(1))/image_resolution_);

  int ix_add = round((O1x - new_origin(0))/image_resolution_);
  int iy_add = round((O1y - new_origin(1))/image_resolution_);

  cimg_library::CImg<float> new_value_(image_width, image_height);
  cimg_library::CImg<float> new_weights_(image_width, image_height);

  for (size_t x=0; x<image_width; x++) {
    for (size_t y=0; y<image_height; y++) {
      new_values_(x,y) = 0.0;
      new_weights_(x,y) = 0.0;
    }
  }

  for (int x=0; x<w0; x++) {
    for (int y=0; y<h0; y++) {
      new_value_image(x + ix_master, y + iy_master) += values_(x,y) * (numruns-1);
      new_weights_image(x + ix_master, y + iy_master) += weights_(x,y) * (numruns-1);
    }
  }

  for (int x=0; x<w1; x++) {
    for (int y=0; y<h1; y++) {
      new_values_(x + ix_add, y + iy_add) += subsumed_values(x,y);
      new_weights_(x + ix_add, y + iy_add) += subsumed_weights(x,y);
    }
  }
  origin_ = new_origin;
  weights_ = new_weights;
  values_ = new_values;
  //TODO update age
}

