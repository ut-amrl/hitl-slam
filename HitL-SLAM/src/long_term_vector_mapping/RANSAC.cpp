void LocalSequentialRANSAC::setParameters() {


}

Eigen::Vector2d LocalSequentialRANSAC::closestPoint(Vector2d pstar, Vector2d ps, Vector2d p) {
  Eigen::Vector2d p0;
  double coeff = (p-ps).dot(pstar)/pstar.dot(pstar);
  p0 = ps + coeff*pstar;
  return p0;
}

double LocalSequentialRANSAC::distToLine(Vector2d v, Vector2d w, Vector2d p) {
  double t = (p-v).dot(w-v)/(w-v).dot(w-v);
  Vector2d proj = v + t*(w-v);
  double dst = sqrt((p-proj).dot(p-proj));
  return dst;
}

double LocalSequentialRANSAC::dirDistToLine(Vector2d p1, Vector2d p2, Vector2d p, Vector2d pdir) {
  Matrix2d rot;
  rot << 0, -1, 1, 0;
  Vector2d nhat = rot*(p2-p1);
  nhat.normalize();
  pdir.normalize();
  double numer = distToLine(p1, p2, p);
  double denom = nhat.dot(pdir);
  if (denom < 0.01) {
    denom = 0.01;
  }
  double dst = numer/denom;
  return dst;
}

vector<Vector2d> LocalSequentialRANSAC::segFit(double* p1, double* p2, double* cm, double* data, int size) {
  vector<Vector2d> fit;
  Vector2d ep1;
  Vector2d ep2;

  Problem problem;
  for (int i = 0; i<size; ++i) {
    problem.AddResidualBlock(
      new AutoDiffCostFunction<segDistResidual, 1, 2, 2>(
          new segDistResidual(data[2*i], data[2*i + 1], cm[0], cm[1], double(size))),
          NULL, p1, p2);
  }

  Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  ep1(0) = p1[0];
  ep1(1) = p1[1];
  ep2(0) = p2[0];
  ep2(1) = p2[1];

  fit.push_back(ep1);
  fit.push_back(ep2);
  return fit;
}


vector<mappingVector> LocalSequentialRANSAC::Run(vector<Pose2Df> poses, vector<Vector2f> point_cloud, NormalCloudf normal_cloud) {
  vector<mappingVector> new_vectors;

  vector<Vector2d> used;
  vector<pair<Vector2d, Vector2d> > segments;

  vector<Vector2d> working_set;
  for (size_t i=0; i<point_cloud.size(); i++) {
    working_set.push_back(Vector2d(point_cloud[i](0), point_cloud[i](1)));
  }

  double minx, miny, maxx, maxy;
  for (size_t i=0; i<working_set.size(); i++) {
    minx = min(working_set[i](0), minx);
    miny = min(working_set[i](1), miny);
    maxx = max(working_set[i](0), maxx);
    maxy = max(working_set[i](1), maxy);
  }
  vector<Vector2d> model;
  //standard deviation in sensed data in meters
  double sigma = .04;
  //inlier threshold necessary to fit
  double w = .8;
  //threshold for determining if a data point is local
  double radius = .4;
  //threshold for determining if a data point is local along a line
  double l = 0.5;
  //threshold for determining if a data point reasonably fits the proposed model
  double t = 3*sigma;
  //threshold for determining if a data point has a normal that agrees with the model
  double tn = 0.5;
  //minimum number of observations warranting a line fit
  int d = 150;
  //iterations
  int k = 2000;

  Eigen::Matrix2d rot;
  rot << 0, -1, 1, 0;

  int iter = 0;
  while (iter < k && working_set.size() >= size_t(d)) {
    iter++;
    int N = working_set.size();
    vector<Vector2d> neighborhood;
    Vector2d v1;
    pair<Vector2d, Vector2d> new_model;
    while (neighborhood.empty()) {
      int v1_index = rand() % N;
      v1 = working_set[v1_index];
      for (int i=0; i<N; i++) {
        if ((working_set[i] - v1).norm() <= radius && working_set[i] != v1) {
          neighborhood.push_back(working_set[i]);
        }
      }
    }

    bool model_ok = false;
    while (!model_ok) {
      int v2_index = rand() % neighborhood.size();
      if (neighborhood[v2_index] != v1) {
        new_model = std::make_pair(v1,neighborhood[v2_index]);
        model_ok = true;
      }
    }

    bool converge = false;
    int outliers = 0;
    int inliers = 0;
    vector<Vector2d> inlier_points;
    vector<int> inlier_indices;
    vector<Pose2Df> inlier_poses;
    Vector2d p1;
    Vector2d p2;
    Vector2d norm_model = (new_model.first-new_model.second)/(new_model.first-new_model.second).norm();
    for (int i=0; i<N; i++) {
      Vector2d point = working_set[i];
      Vector2d source = Vector2d(poses[i].translation(0), poses[i].translation(1));
      Vector2d dir = (point - source).normalized();

      double dirdst = dirDistToLine(new_model.first, new_model.second, working_set[i], dir);
      double dst = distToLineSeg(new_model.first, new_model.second, working_set[i]);
      Vector2f normalf = normal_cloud[i]/(normal_cloud[i].norm());
      Vector2d normal(double(normalf(0)),double(normalf(1)));
      double ndst = fabs(norm_model.dot(normal));
      if (dst <= t && tn >= ndst) { //close to line segment
        inliers ++;
        inlier_points.push_back(working_set[i]);
        inlier_poses.push_back(poses[i]);
        inlier_indices.push_back(i);
      }
      else if (dst > t && dirdst > t && dirdst < l) { // in the neighborhood l, but not close enough to infinite line
        outliers ++;
      }
    }
    double in_frac = double(inliers)/double(inliers + outliers);

    if (inliers >= d && in_frac >= w) {
      while (!converge) {
        int NI = inlier_points.size();
        Vector2d cm(0.0,0.0);
        double data[2*NI];
        Vector2d ip;
        for (int i=0; i<NI; i++) {
          ip = inlier_points[i];
          data[2*i] = ip(0);
          data[2*i+1] = ip(1);
          cm += ip;
        }
        cm = cm/double(NI);
        double CM[2];
        CM[0] = cm(0);
        CM[1] = cm(1);
        double P1[2];
        double P2[2];
        P1[0] = new_model.first(0) + 0.000001;
        P1[1] = new_model.first(1) + 0.000001;
        P2[0] = new_model.second(0) + 0.000001;
        P2[1] = new_model.second(1) + 0.000001;

        vector<Vector2d> end_points = segFit(P1, P2, CM, data, NI);
        p1 = end_points[0];
        p2 = end_points[1];

        // no significant change in p1 or p2
        if ((new_model.first - p1).norm() < 0.1 && (new_model.second - p2).norm() < 0.1) {
          converge = true;
        }
        else {
          outliers = 0;
          inliers = 0;
          new_model.first = p1;
          new_model.second = p2;
          inlier_points.clear();
          inlier_indices.clear();

          for (int i=0; i<N; i++) {
            Vector2d point = working_set[i];
            Vector2d source = Vector2d(poses[i].translation(0), poses[i].translation(1));
            Vector2d dir = (point - source).normalized();

            double dirdst = dirDistToLine(new_model.first, new_model.second, working_set[i], dir);
            double dst = distToLineSeg(new_model.first, new_model.second, working_set[i]);
            Vector2f normalf = normal_cloud[i]/(normal_cloud[i].norm());
            Vector2d normal(double(normalf(0)),double(normalf(1)));
            double ndst = fabs(norm_model.dot(normal));
            if (dst <= t && tn >= ndst) {
              inliers ++;
              inlier_points.push_back(working_set[i]);
              inlier_poses.push_back(poses[i]);
              inlier_indices.push_back(i);
            }
            else if (dst > t && dirdst > t && dirdst < l) {
              outliers ++;
            }
          }
          in_frac = double(inliers)/double(inliers + outliers);
        }
      } //end while

      pair<Vector2d, Vector2d> seg = make_pair(p1,p2);
      segments.push_back(seg);

      int NI = inlier_points.size();
      for (int i=NI-1; i>=0; i--) {
        working_set.erase(working_set.begin() + inlier_indices[i]);
        poses.erase(poses.begin() + inlier_indices[i]);
        normal_cloud.erase(normal_cloud.begin() + inlier_indices[i]);
      }

      //compute uncertainty
      mappingVector new_vector = computeVectorUncertainty(p1, p2, inlier_points, inlier_poses);
      new_vectors.push_back(new_vector);

    } //end if (valid model)

  } //end ransac main while

  return new_vectors;
}
