
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "../perception_tools/perception_2d.h"

#include "human_constraints.h"

#include "EMinput.h"
#include "ApplyExplicitCorrection.h"
#include "Backprop.h"
#include "JointOptimization.h"


class HitLSLAM {
  public:
    //void DisplayPoses(const DisplaySet display_flags, guivizmsg* display_msg);
    void Run();
    void init(const std::vector<perception_2d::Pose2Df> odom,
              const std::vector<perception_2d::PointCloudf> rob_frame_pcs,
              const std::vector<perception_2d::NormalCloudf> norm_clouds,
              std::vector<Eigen::Matrix3f> covars,
              std::vector<perception_2d::Pose2Df> poses);
    void replayFromLog(const std::vector<SingleInput> input_log);
    void addCorrectionPoints(const uint32_t type, 
                             const Eigen::Vector2f mouse_down, 
                             const Eigen::Vector2f mouse_up);
 
    std::vector<perception_2d::Pose2Df> getPoses();
    std::vector<Eigen::Matrix3f> getCovariances();
    std::vector<perception_2d::PointCloudf> getWorldFrameScans();

    //TODO: getter for ceres information such as gradients, 
    //      constraint membership, point to point correspondences, etc.
    
  private:
    bool isValidCorrectionType(const CorrectionType& type);
    size_t verifyUserInput();
    void resetCorrectionInputs();
    void transformPointCloudsToWorldFrame();    

    // class which handles interpretation of user input
    EMInput em_input_;

    // class which applies affine transforms directly specified by user
    AppExpCorrect app_exp_corr_;

    // class implementing COP-SLAM
    Backprop backprop_;

    // class which handles optimization of factor graph
    JointOpt joint_opt_;

    // original relative odometry
    std::vector<perception_2d::Pose2Df> odometry_;
    
    // point clouds in robot frame, normal clouds in robot frame
    std::vector<perception_2d::PointCloudf> ROBOT_FRAME_point_clouds_;
    std::vector<perception_2d::NormalCloudf> normal_clouds_;

    // point clouds in world frame
    std::vector<perception_2d::PointCloudf> WORLD_FRAME_point_clouds_;
    
    // pose estimates
    std::vector<perception_2d::Pose2Df> poses_;
    
    //covariances
    std::vector<Eigen::Matrix3f> covariances_;
    
    // in case human wants to undo last constraint
    std::vector<perception_2d::Pose2Df> prev_poses_;
    std::vector<Eigen::Matrix3f> prev_covariances_;
    
    // points selected by user during HitL
    std::vector<Eigen::Vector2f> selected_points_;
    
    // correction type of current user input
    CorrectionType correction_type_;  
    
    // pending correction type, used to evaluate whether current user input
    // can be used to complete a correction started by previous input
    CorrectionType pending_correction_type_ = CorrectionType::kUnknownCorrection;

    // history of user input, used for logging
    std::vector<SingleInput> input_history_;
 
    // human constraints
    std::vector<std::vector<HumanConstraint>> human_constraints_;










    int added_human_constraints = 0;


    double total_runtime = 0.0;

    int num_completed_cycles = 0;

    int num_total_constraints = 0;


    std::vector<SingleInput> logged_input;

    int current_replay_index = 0;  
    
    int correction_number = 0;
    
    std::vector<float> prev_ceres_cost;

    // gradients from ceres solver
    std::vector<double> ceres_gradients_;

    // sparse jacobian representation from ceres
    ceres::CRSMatrix ceres_jacobian_;
    bool jacobian_init_ = false;

    // number of residuals due to human constraints
    int num_hc_residuals_;

    // For visualization
    float max_theta_var = 0.0;





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




    static const uint32_t kTrajectoryColor = 0x7F000000;
    static const uint32_t kPoseCovarianceColor = 0xFF808080;
    static const uint32_t kOdometryColor = 0x70FF0000;
    static const uint32_t kCorrespondenceColor = 0x7FFF7700;
};
