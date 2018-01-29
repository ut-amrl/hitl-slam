class LocalSequentialRANSAC {
  public:
    void setParameters();
    std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> Run(
        std::vector<perception_2d::Pose2Df> poses, 
        std::vector<Eigen::Vector2f> point_cloud, 
        perception_2d::NormalCloudf normal_cloud);
    std::vector<Eigen::Vector2d> segFit(double* p1, double* p2, 
                                        double* cm, double* data, 
                                        int size); 

    struct segDistResidual {
      segDistResidual(double px, double py, 
                      double cmx, double cmy, 
                      double N) 
          : px_(px), py_(py), 
            cmx_(cmx), cmy_(cmy), 
            N_(N) {}
      
      template <typename T> bool operator()(const T* const p1, 
                                            const T* const p2, 
                                            T* residual) const {
        T t = ((T(px_)-p1[0])*(p2[0]-p1[0])+(T(py_)-p1[1])*(p2[1]-p1[1])) / 
               (pow(p2[0]-p1[0],2)+pow(p2[1]-p1[1],2));
        T partial_res, centroid1, centroid2;
        centroid1 = sqrt(pow(T(cmx_)-p1[0],2)+pow(T(cmy_)-p1[1],2)); //dist p1 to cm
        centroid2 = sqrt(pow(T(cmx_)-p2[0],2)+pow(T(cmy_)-p2[1],2)); //dist p2 to cm

        if (t < 0.0) { // Beyond the 'p1' end of the segment
          partial_res = sqrt(pow(T(px_)-p1[0],2)+pow(T(py_)-p1[1],2));
        }
        else if (t > 1.0) { // Beyond the 'p2' end of the segment
           partial_res = sqrt(pow(T(px_)-p2[0],2)+pow(T(py_)-p2[1],2)); 
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

  private:
    

    Eigen::Vector2d closestPoint(Eigen::Vector2d pstar, 
                                 Eigen::Vector2d ps, 
                                 Eigen::Vector2d p);

    double dirDistToLine(Eigen::Vector2d p1, Eigen::Vector2d p2, 
                         Eigen::Vector2d p, Eigen::Vector2d pdir);
    
    double distToLine(Eigen::Vector2d v, Eigen::Vector2d w, Eigen::Vector2d p);







    


    
  

};

//TODO: add any other implementations of RANSAC
