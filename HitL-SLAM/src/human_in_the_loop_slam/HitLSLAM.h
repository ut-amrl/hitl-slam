
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
    void tryLoopClosure(const std::pair<CorrectionType, std::vector<Eigen::Vector2f>> proposed_correction);
 
    
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
  
    
    
    
    
    
    
    
    
    
    vector<float> prev_ceres_cost;

// gradients from ceres solver
std::vector<double> ceres_gradients_;

// sparse jacobian representation from ceres
ceres::CRSMatrix ceres_jacobian_;
bool jacobian_init_ = false;

// number of residuals due to human constraints
int num_hc_residuals_;

// For visualization
float max_theta_var = 0.0;

vector<double> message_timestamps_;

// pose estimates
vector<Pose2Df> poses;

// original relative odometry
//vector<perception_2d::Pose2Df> orig_odom_;

// in case human wants to undo last constraint
vector<Pose2Df> prev_poses;
int added_human_constraints = 0;

// point clouds in robot frame
vector<PointCloudf> init_point_clouds;
vector<NormalCloudf> normal_clouds;

//covariances
vector<Matrix3d> covariances_;
vector<Matrix3d> prev_covariances_;

// human constraints
vector<vector<VectorMapping::HumanConstraint>> human_constraints_;

// points selected by user during HitL
vector<Vector2f> selected_points_;
CorrectionType correction_type_ = CorrectionType::kUnknownCorrection;
CorrectionType pending_correction_type_ = CorrectionType::kUnknownCorrection;
bool loop_corrections_on_ = false;

// Name of the topic on Cobot's software stack that laser data is published on.
static const string kCobotLaserTopic("/Cobot/Laser");
// Name of the topic on Cobot's software stack that Kinect scan data is
// published on.
static const string kKinectScanTopic("/Cobot/Kinect/Scan");
// Name of the topic in a standardized data bag files that Kinect scan data is
// published on.
static const string kStandardKinectScanTopic("kinect_scan");
// Name of the topic on Cobot's software stack that odometry is published on.
static const string kCobotOdometryTopic("/Cobot/Odometry");
// Name of the topic in a standardized data bag files that laser data is
// published on.
static const string kStandardLaserTopic("laser");
// Name of the topic in a standardized data bag files that odometry is
// published on.
static const string kStandardOdometryTopic("odom");
// Name of the topic in a standardized data bag files that location reset
// commands are published on.
static const string kStandardSetLocationTopic("set_location");
// Name of the map to localize the robot on.
const string kMapName("EmptyMap");
// Robot's starting location.
Vector2f kStartingLocation = Vector2f(0.0, 0.0);
// Robot's starting angle.
float kStartingAngle = 0.0;
// Uncertainty of translation in the direction of travel.
float kRadialTranslationUncertainty = 0.05;
// Uncertainty of translation perpendicular to the direction of travel.
float kTangentialTranslationUncertainty = 0.05;
// Uncertainty of rotation in radians after moving 1 radian.
float kAngleUncertainty = 0.05;
// Scaling constant to correct for error in angular odometry.
float kOdometryRotationScale = 1.0;
// Scaling constant to correct for error in translation odometry.
float kOdometryTranslationScale = 1.0;
// Minimum distance of observed points from the robot.
float kMinPointCloudRange = 0.2;
// Maximum distance of observed points from the robot.
float kMaxPointCloudRange = 6.0;
// Maximum distance between adjacent points to use for computation of normals.
float kMaxNormalPointDistance = 0.03;
// Indicates whether the bag file being read is a standardized data bag file.
bool kStandardizedData = false;
// Angular margin from the scan area boundary to ignore laser readings.
float kAngularMargin = 0.0;

char* bag_file = NULL;

char* stfs_file = NULL;

char* log_file = NULL;

bool lost_poses = true;

int correction_number = 0;

// Mutex to ensure only a single relocalization call is made at a time.
pthread_mutex_t relocalization_mutex_ = PTHREAD_MUTEX_INITIALIZER;

// The path of the cobot_linux ROS stack.
static const string kCobotStackPath(ros::package::getPath("cobot_linux"));

// The directory where all the maps are stored.
static const string kMapsDirectory(kCobotStackPath + "/../maps");

// Index of test set. This will determine which file the results of the test are
// saved to.
int test_set_index_ = -1;

// Indicates that a statistical test is being run, and to save the localization
// results to the file with the specified index.
int statistical_test_index_ = -1;

// The fraction of additive odometry noise for statistical tests.
double odometry_additive_noise_ = 0.05;

//static const uint32_t kTrajectoryColor = 0x6FFF0000;
static const uint32_t kTrajectoryColor = 0x7F000000;
static const uint32_t kPoseCovarianceColor = 0xFF808080;
static const uint32_t kOdometryColor = 0x70FF0000;
// static const uint32_t kTrajectoryColor = 0xFFC0C0C0;
static const uint32_t kLtfCorrespondenceColor = 0x7FFF7700;
static const uint32_t kLtfPointColor = 0xFFFF7700;
static const uint32_t kStfPointColor = 0xFFFF5500;
static const uint32_t kStfCorrespondenceColor = 0x7F994CD9;
static const uint32_t kDfPointColor  = 0x7F37B30C;
static const uint32_t kObjectColor = 0xFF56C4C3;

bool run_ = true;
int debug_level_ = -1;

// Indicates that /Cobot/Kinect/Scan should be used instead of
// /Cobot/Laser, to localize using the Kinect sensor instead of the laser
// rangefinder.
bool use_kinect_ = false;

// Indicates that the user will provide an estimate of an object instance via
// the localization_gui.
bool get_object_instance_ = false;

// Display message for drawing debug vizualizations on the localization_gui.
vector_slam_msgs::LidarDisplayMsg display_message_;

// Service client to ask localization_gui to save images.
ros::ServiceClient gui_capture_client;

// ROS publisher to publish display_message_ to the ROS topic
// Cobot/VectorLocalization/Gui
ros::Publisher display_publisher_;

// ROS publisher to the CoBot events topic.
ros::Publisher events_publisher_;

// ROS publisher to publish the latest robot localization estimate to
// Cobot/Localization
ros::Publisher localization_publisher_;

// ROS subscriber to the localization gui mouse move events
ros::Subscriber mouse_move_subscriber_;

// ROS subscriber to the localization gui mouse click events
ros::Subscriber mouse_click_subscriber_;

// ROS subscriber to /Cobot/VectorLocalization/GuiKeyboardEvents
ros::Subscriber keyboard_event_subscriber_;

// Parameters and settings for Non-Markov Localization.
VectorMapping::VectorMappingOptions localization_options_;

// Parameters for learning HitL SLAM


// Main class instance for Non-Markov Localization.
VectorMapping localization_(kMapsDirectory);


EMInput em_input_;

AppExpCorrect app_exp_corr_;

Backprop backprop_;

JointOpt joint_opt_;



// Relocalization interface.
//vector_localization::Relocalization relocalization_(kMapsDirectory);

// The last observed laser scan, used for auto localization.
sensor_msgs::LaserScan last_laser_scan_;

// Parameters used for relocalization.
//VectorLocalization2D::LidarParams relocalization_lidar_params_;

// File name of the final image of all observations from all poses with their
// classifications.
char* save_image_file = NULL;

// Directory where images of every episode at every timestep will be saved.
char* episode_images_path = NULL;

// Determines whether STFS will be saved for later object mapping or not.
bool save_stfs_ = false;

// WatchFiles to track changes to config files.
WatchFiles watch_files_;

// Config reader for localization options.
ConfigReader config_((kCobotStackPath + "/").c_str());

// Corrections to angle-dependent errors of the laser scanner.
vector<float> laser_corrections_;

// Resolution of the laser corrections lookup table.
float laser_corrections_resolution_ = 0.0;

// Boolean flag to indicate use of laser scanner corrections.
bool use_laser_corrections_ = false;

// Flag to write LTF observations to disk for error correction computation.
bool save_ltfs_ = false;
    
};