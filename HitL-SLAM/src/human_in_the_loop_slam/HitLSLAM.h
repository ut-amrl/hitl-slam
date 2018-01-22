
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "../perception_tools/perception_2d.h"

struct SingleInput {
  CorrectionType type_of_constraint;
  int undone;
  std::vector<Eigen::Vector2f> input_points;
};

enum class CorrectionType : uint32_t {
  kUnknownCorrection = 0,
  kPointCorrection = 1,
  kLineSegmentCorrection = 2,
  kCornerCorrection = 3,
  kColinearCorrection = 4,
  kPerpendicularCorrection = 5,
  kParallelCorrection = 7,
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

class HitLSLAM {
  public:
    void DisplayPoses(const DisplaySet display_flags, guivizmsg* display_msg);
    void replayFromLog(const string log_file_name);
    void tryLoopClosure(const std::pair(CorrectionType, std::vector<Eigen::Vector2f>) proposed_correction);
 
    
  private:
    bool IsValidCorrectionType(const CorrectionType& type);
    void AddCorrectionPoints(const std::pair(CorrectionType, std::vector<Eigen::Vector2f>)& proposed_correction);
    size_t VerifyUserInput(std::vector<perception_2d::PointCloudf>* point_clouds);
    void ResetCorrectionInputs();
    
    
    double total_runtime = 0.0;
    int num_completed_cycles = 0;
    int num_total_constraints = 0;
    std::vector<SingleInput> input_history;
    std::vector<SingleInput> logged_input;
    int current_replay_index = 0;  
  
};