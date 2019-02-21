#ifndef HUMAN_CONSTRAINTS_H
#define HUMAN_CONSTRAINTS_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>

enum class CorrectionType : uint32_t {
  kUnknownCorrection = 0,
  kPointCorrection = 1,          // ALT        (not supported yet)
  kLineSegmentCorrection = 2,    // CTRL
  kCornerCorrection = 3,         // ALT + CTRL (not supported yet)
  kColinearCorrection = 4,       // SHIFT
  kPerpendicularCorrection = 5,  // SHIFT + ALT
  kParallelCorrection = 6,       // CTRL + SHIFT
};

const std::string CorrectionTypeNames[] = {
  "Unknown",
  "Point",
  "LineSegment",
  "Corner",
  "Colinear",
  "Perpendicular",
  "Reserved",
  "Parallel"
};

struct HumanConstraint {
    // Type of constraint
    CorrectionType constraint_type;
    // pose indices
    int constrained_pose_id;
    int anchor_pose_id;
    // initial poses
    float delta_parallel;
    float delta_perpendicular;
    float delta_angle;
    // needed for colinear constraints
    float relative_penalty_dir;
};

struct SingleInput {
  CorrectionType type_of_constraint;
  int undone;
  std::vector<Eigen::Vector2f> input_points;
};

#endif
