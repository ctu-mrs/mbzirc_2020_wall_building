/* include //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mutex>

#include <boost/shared_ptr.hpp>

#include <actionlib/server/simple_action_server.h>
#include <eigen3/Eigen/Eigen>

#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/median_filter.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/quadratic_thrust_model.h>

#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/OdometryDiag.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/GripperDiagnostics.h>
#include <mrs_msgs/Float64Srv.h>
#include <mrs_msgs/GetFloat64.h>

#include <mbzirc_msgs/MbzircBrick.h>
#include <mbzirc_msgs/BanArea.h>
#include <mbzirc_msgs/GraspingAction.h>
#include <mbzirc_msgs/GraspingActionGoal.h>
#include <mbzirc_msgs/BrickGraspingDiagnostics.h>
#include <mbzirc_msgs/StartGrasping.h>
#include <mbzirc_msgs/MagnetControl.h>
#include <mbzirc_msgs/ObjectWithType.h>
#include <mbzirc_msgs/Drop.h>
#include <mbzirc_msgs/DetectionType.h>

#include <nav_msgs/Odometry.h>

#include <dynamic_reconfigure/ReconfigureRequest.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

//}

/* using //{ */

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

using namespace Eigen;

namespace brick_grasping
{

/* defines //{ */


#define POS_X 0
#define POS_Y 1
#define POS_Z 2
#define ANGLE 3

#define ALIGNMENT_CRITERION_CONTROL_ERROR 0
#define ALIGNMENT_CRITERION_BRICK_DETECTION 1

//}

/* STRUCTURES and TYPEDEFS //{ */

// action server
typedef actionlib::SimpleActionServer<mbzirc_msgs::GraspingAction> GraspingServer;

// type of the object we are grasping
typedef enum
{

  // from brick detection and brick estimation
  BRICK_RED   = 1,
  BRICK_GREEN = 2,
  BRICK_BLUE  = 3,
  WALL        = 5,
  WALL_RED    = 6,
  WALL_GREEN  = 7,
  WALL_BLUE   = 8,

  BRICK_ANY = 666,

} Object_t;

typedef enum
{

  MODE_2D = 1,
  MODE_3D = 2,

} Alignment_t;

// state machine
[[maybe_unused]] enum {

  IDLE_STATE,
  ALIGN_STATE,
  DESCEND_STATE,
  ALIGN2_GRASP_STATE,
  GRASP_STATE,
  REPEAT_STATE,
  ASCEND_STATE,
  ASCEND_AFTER_PLACE_STATE,
  ABORT_STATE,
  ALIGN_PLACE_STATE,
  GROUND_PLACING_STATE,
  WALL_PLACING_STATE,
  WAITING_AFTER_PLACING,
  PREEMPTED_STATE,

} States_t;

typedef enum
{

  SEE_EVERYTHING        = 0,
  SEE_RED               = 1,
  SEE_GREEN             = 2,
  SEE_BLUE              = 3,
  SEE_ORANGE            = 4,
  SEE_WALL              = 5,
  SEE_WALL_HAVING_RED   = 6,
  SEE_WALL_HAVING_GREEN = 7,
  SEE_WALL_HAVING_BLUE  = 8,

} VisionMode_t;

const char *vision_mode_namses[9] = {
    "SEE EVERYTHING",
    "SEE RED",
    "SEE GREEN",
    "SEE BLUE",
    "SEE ORANGE",
    "SEE WALL",
    "SEE WALL WHILE CARRYING RED",
    "SEE WALL WHILE CARRYING GREEN",
    "SEE WALL WHILE CARRYING BLUE",
};

const int vision_mode_values[9] = {0, 1, 2, 3, 4, 8, 9, 10, 11};

const char *state_names[14] = {
    "IDLING",
    "ALIGNING",
    "DESCENDING",
    "ALIGNING 2 FOR GRASPING",
    "GRASPING",
    "REPEATING",
    "ASCENDING",
    "ASCENDING AFTER PLACING",
    "ABORTING",
    "ALIGNING FOR PLACING",
    "GROUND PLACING",
    "WALL PLACING",
    "WAITING AFTER PLACING",
    "PREEMPTING",
};

// trajectory types
[[maybe_unused]] enum {

  ALIGN_TRAJECTORY,
  DESCEND_TRAJECTORY,
  ASCEND_TRAJECTORY,
  GRASPING_TRAJECTORY,
  REPEAT_TRAJECTORY,
  ABORT_TRAJECTORY,
  ALIGN_TO_PLACE_TRAJECTORY,
  GROUND_PLACING_TRAJECTORY,
  PLACING_TRAJECTORY,
  ASCEND_AFTER_PLACE_TRAJECTORY,

} Trajectory_t;

// action server result ids
[[maybe_unused]] enum {

  RESULT_SUCCESS             = 0,
  RESULT_STOPPED_BY_COMMAND  = 1,
  RESULT_OBJECT_NOT_RELIABLE = 2,
  RESULT_OBJECT_NOT_VISIBLE  = 3,
  RESULT_TIMEOUT             = 4,
  RESULT_FAILED              = 5,

} Result_t;

//}

// --------------------------------------------------------------
// |                          the class                         |
// --------------------------------------------------------------

/* class BrickGrasping //{ */

class BrickGrasping : public nodelet::Nodelet {

public:
  virtual void onInit();
  bool         is_initialized_ = false;

private:
  ros::NodeHandle nh_;

  std::string _uav_name_;

  // transfomer
  mrs_lib::Transformer transformer_;

  double _map_long_inactive_time_;
  double _map_short_inactive_time_;

  ros::Publisher publisher_trajectory_;
  ros::Publisher publisher_relative_position_;
  ros::Publisher publisher_diagnostics_;
  ros::Publisher publisher_debug_trajectory_;
  ros::Publisher publisher_current_target_;
  ros::Publisher publisher_current_target_uav_odom_;
  ros::Publisher publisher_current_target_uav_heading_;
  ros::Publisher publisher_current_target_debug_;
  ros::Publisher publisher_grasping_result_;

  ros::Subscriber subscriber_closest_red_;
  ros::Subscriber subscriber_closest_green_;
  ros::Subscriber subscriber_closest_blue_;
  ros::Subscriber subscriber_closest_wall_;
  ros::Subscriber subscriber_closest_any_;

  ros::Subscriber subscriber_odometry_main_;
  ros::Subscriber subscriber_odometry_diagnostics_;
  ros::Subscriber subscriber_cmd_odom_;
  ros::Subscriber subscriber_simulation_gripper_;
  ros::Subscriber subscriber_mrs_gripper_;

  ros::ServiceServer service_goal_;
  ros::ServiceServer service_server_start_;
  ros::ServiceServer service_servcer_stop_;
  ros::ServiceServer service_server_ground_place_;
  ros::ServiceServer service_server_wall_place_;

  ros::ServiceClient simulation_magnet_service_;
  ros::ServiceClient service_client_gripper_on_;
  ros::ServiceClient service_client_gripper_off_;
  ros::ServiceClient odom_rangefinder_service_;
  ros::ServiceClient service_client_ban_area_;
  ros::ServiceClient constraints_client_;
  ros::ServiceClient gains_client_;
  ros::ServiceClient service_client_reset_map_;
  ros::ServiceClient service_client_switch_controller_;
  ros::ServiceClient service_client_switch_lateral_odometry_;
  ros::ServiceClient service_client_switch_height_odometry_;
  ros::ServiceClient service_client_hover_;
  ros::ServiceClient service_client_reset_tracker_;
  ros::ServiceClient service_client_set_vision_mode_;
  ros::ServiceClient service_client_set_min_height_;
  ros::ServiceClient service_client_get_min_height_;
  ros::ServiceClient service_client_set_map_inactive_time_;
  ros::ServiceClient service_client_set_avoidance_;

  // params loaded from config file
  double _trajectory_dt_;

  ros::Time timeouter_;
  bool      timeout(const double timeout);

  void freshFocusedBrick(const mbzirc_msgs::MbzircBrick brick_in);

  std::unique_ptr<MedianFilter> median_filter_gripper_;
  std::unique_ptr<MedianFilter> median_filter_simulation_gripper_;
  int                           _gripper_filter_buffer_size_;
  double                        _gripper_filter_max_difference_;

  std::string _brick_traverse_constraints_;
  std::string _nobrick_traverse_constraints_;

  // aligning_grasping params
  double      _aligning_speed_;
  double      _aligning_height_;
  double      _aligning_radius_;
  double      _aligning_timeout_;
  std::string _aligning_controller_;
  std::string _aligning_odometry_lateral_;
  std::string _aligning_odometry_height_;
  std::string _aligning_constraints_;
  std::string _aligning_gains_;

  // descending params
  double      _descending_speed_;
  double      _descending_timeout_;
  double      _descending_height_;
  std::string _descending_controller_;
  std::string _descending_odometry_lateral_;
  std::string _descending_odometry_height_;
  std::string _descending_constraints_;
  std::string _descending_gains_;

  // aligning2_grasping params
  double _aligning2_grasping_timeout_;

  double _aligning2_grasping_criterion_initial_x_;
  double _aligning2_grasping_criterion_initial_y_;
  double _aligning2_grasping_criterion_increase_rate_x_;
  double _aligning2_grasping_criterion_increase_rate_y_;

  double      _aligning2_in_alignment_duration_;
  int         _aligning2_grasping_alignment_criterion_;
  std::string _aligning2_grasping_controller_;
  std::string _aligning2_grasping_odometry_lateral_;
  std::string _aligning2_grasping_odometry_height_;
  std::string _aligning2_grasping_constraints_;
  std::string _aligning2_grasping_gains_;

  ros::Time aligning2_in_radius_time_;
  bool      aligning2_in_radius_ = false;
  double    aligning2_current_x_crit_;
  double    aligning2_current_y_crit_;

  // grasping params
  double      _grasping_timeout_;
  double      _grasping_speed_;
  double      _grasping_height_;
  int         _grasping_repeat_threshold_;
  std::string _grasping_controller_;
  std::string _grasping_odometry_lateral_;
  std::string _grasping_odometry_height_;
  std::string _grasping_constraints_;
  std::string _grasping_gains_;

  bool      _grasping_thrust_limiter_enabled_ = false;
  double    _grasping_thrust_limiter_ratio_;
  double    _grasping_thrust_timeout_;
  bool      grasping_thrust_under_threshold_ = false;
  ros::Time grasping_thrust_first_time_;

  double      _aligning_placing_speed_;
  double      _aligning_placing_radius_;
  double      _aligning_placing_height_;
  double      _aligning_placing_timeout_;
  double      _aligning_placing_safety_area_min_height_;
  std::string _aligning_placing_controller_;
  std::string _aligning_placing_odometry_lateral_;
  std::string _aligning_placing_odometry_height_;
  std::string _aligning_placing_constraints_;
  std::string _aligning_placing_gains_;

  double      _ground_placing_speed_;
  double      _ground_placing_timeout_;
  double      _ground_placing_height_;
  std::string _ground_placing_controller_;
  std::string _ground_placing_odometry_lateral_;
  std::string _ground_placing_odometry_height_;
  std::string _ground_placing_constraints_;
  std::string _ground_placing_gains_;

  double    _ground_placing_thrust_limiter_ratio_;
  double    _ground_placing_thrust_timeout_;
  bool      ground_placing_thrust_under_threshold_ = false;
  ros::Time ground_placing_thrust_first_time_;

  double      _placing_speed_;
  double      _placing_timeout_;
  double      _placing_height_;
  std::string _placing_controller_;
  std::string _placing_odometry_lateral_;
  std::string _placing_odometry_height_;
  std::string _placing_constraints_;
  std::string _placing_gains_;

  double    _placing_thrust_limiter_ratio_;
  double    _placing_thrust_timeout_;
  bool      placing_thrust_under_threshold_ = false;
  ros::Time placing_thrust_first_time_;

  // after_placing params
  double      _after_placing_delay_;
  std::string _after_placing_controller_;
  std::string _after_placing_odometry_lateral_;
  std::string _after_placing_odometry_height_;
  std::string _after_placing_constraints_;
  std::string _after_placing_gains_;

  // repeating params
  double      _repeating_speed_;
  double      _repeating_height_;
  std::string _repeating_controller_;
  std::string _repeating_odometry_lateral_;
  std::string _repeating_odometry_height_;
  std::string _repeating_constraints_;
  std::string _repeating_gains_;

  // ascending params
  double      _ascending_speed_;
  std::string _ascending_controller_;
  std::string _ascending_odometry_lateral_;
  std::string _ascending_odometry_height_;
  std::string _ascending_constraints_;
  std::string _ascending_gains_;
  double      ascending_height_;
  std::mutex  mutex_ascending_height_;

  // ascending params
  double      _ascending_after_place_speed_;
  std::string _ascending_after_place_controller_;
  std::string _ascending_after_place_odometry_lateral_;
  std::string _ascending_after_place_odometry_height_;
  std::string _ascending_after_place_constraints_;
  std::string _ascending_after_place_gains_;
  double      ascending_after_place_height_;
  std::mutex  mutex_ascending_after_place_height_;
  double      ascending_initial_height_;

  // aborting params
  double      aborting_height_;
  std::string _aborting_controller_;
  std::string _aborting_odometry_lateral_;
  std::string _aborting_odometry_height_;
  std::string _aborting_constraints_;
  std::string _aborting_gains_;

  // fcu offset
  double _fcu_offset_x_;
  double _fcu_offset_y_;

  int    _loosing_alignment_threshold_;
  int    _number_of_states_;
  double _uav_mass_;
  double _object_visibility_timeout_;

  // gain schedulling
  std::string _gains_wind_suffix_;
  std::string _gains_nowind_suffix_;
  std::string _grasping_gains_name_;
  double      _world_disturbace_thr_;
  bool        isHighWind(void);
  bool        high_wind_situation_here_ = false;

  double safety_area_min_height_;

  geometry_msgs::PoseStamped odometry_main_stable_;
  geometry_msgs::PoseStamped odometry_main_brick_;
  std::mutex                 mutex_odometry_main_;

  mrs_msgs::OdometryDiag odometry_diag_;
  std::mutex             mutex_odometry_diag_;

  geometry_msgs::PoseStamped cmd_odom_stable_;
  geometry_msgs::PoseStamped cmd_odom_brick_;
  std::mutex                 mutex_cmd_odom_;

  mbzirc_msgs::MbzircBrick red_brick_;
  std::mutex               mutex_red_brick_;

  mbzirc_msgs::MbzircBrick blue_brick_;
  std::mutex               mutex_blue_brick_;

  mbzirc_msgs::MbzircBrick green_brick_;
  std::mutex               mutex_green_brick_;

  mbzirc_msgs::MbzircBrick wall_;
  std::mutex               mutex_wall_;

  mbzirc_msgs::MbzircBrick any_brick_;
  std::mutex               mutex_any_brick_;

  mbzirc_msgs::MbzircBrick focused_brick_;
  std::mutex               mutex_focused_brick_;
  std::mutex               mutex_object_gripped_;

  ros::Time red_brick_last_time_;
  ros::Time green_brick_last_time_;
  ros::Time blue_brick_last_time_;
  ros::Time wall_last_time_;
  ros::Time any_brick_last_time_;

  bool got_odometry_main_ = false;
  bool got_odometry_diag_ = false;
  bool got_cmd_odom_      = false;
  bool got_brick_         = false;
  bool got_wall_          = false;

  mrs_msgs::ReferenceStamped current_target_;
  std::mutex                 mutex_current_target_;

  bool _simulation_ = false;

  bool      objectGripped(void);
  ros::Time last_object_gripped_time_;
  bool      object_gripped_;
  bool      doingServoing(void);

  bool            callbackStart(mbzirc_msgs::StartGrasping::Request &req, mbzirc_msgs::StartGrasping::Response &res);
  bool            callbackStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool            callbackGroundPlace(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool            callbackWallPlace(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void            callbackGripperSimulation(const mrs_msgs::GripperDiagnosticsConstPtr &msg);
  void            callbackGripper(const mrs_msgs::GripperDiagnosticsConstPtr &msg);
  Eigen::VectorXd objectMotionModel(const Eigen::VectorXd in, const double dt);
  void            publishDebugTrajectory(mrs_msgs::TrajectoryReference &trajectory);

  void magnetToggle(bool in);

  void setGains(std::string desired_gains);
  void setConstraints(std::string desired_constraints);
  void setController(std::string desired_controller);
  void setVisionMode(VisionMode_t desired_mode);
  void setAvoidance(const bool in);
  void setMapInactiveTime(const double time);
  void setOdometry(std::string lateral_odometry, std::string height_odometry);
  void hover(void);
  void resetTracker(void);
  void resetMap(void);

  void   setMinHeight(double min_height);
  double getMinHeight();
  double original_min_height_ = 0;

  double alignedWithTarget(const double position_thr, const double heading_thr, Alignment_t mode);
  double lastAlignmentCheck(void);

  // | ---------------------- action server --------------------- |

  GraspingServer *grasping_server_;
  std::mutex      mutex_grasping_server_;

  mbzirc_msgs::GraspingResult   grasping_result_;
  mbzirc_msgs::GraspingFeedback grasping_feedback_;
  void                          callbackActionServerGoal();
  void                          callbackActionServerPreempt();
  bool                          _not_grasping_ = false;
  ros::Time                     grasping_time_;
  ros::Time                     waiting_start_time_;
  ros::Time                     placing_time_;
  ros::Time                     wait_1_fail_time_;
  mbzirc_msgs::GraspingGoal     current_goal_;
  std::mutex                    mutex_current_goal_;

  // | --------------------- / action server -------------------- |

  // state machine
  int current_state_, previous_state_;
  int lost_alignment_counter, repeat_grasping_counter;

  ros::Timer state_machine_timer_;
  void       stateMachineTimer(const ros::TimerEvent &event);

  int grasping_object_type_;

  ros::Timer diagnostics_timer_;
  void       diagnosticsTimer(const ros::TimerEvent &event);

  ros::Timer current_target_timer_;
  void       currentTargetTimer(const ros::TimerEvent &event);

  ros::Timer gripper_timer_;
  void       gripperTimer(const ros::TimerEvent &event);
  bool       gripper_ = false;

  void publishDiagnostics(void);

public:
  double _main_rate_;
  double _diagnostics_rate_;
  double _gripper_timer_rate_;

  void callbackOdometryMain(const nav_msgs::OdometryConstPtr &msg);
  void callbackOdomDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg);
  void callbackPositionCmd(const nav_msgs::OdometryConstPtr &msg);
  void callbackClosestRed(const mbzirc_msgs::MbzircBrickConstPtr &msg);
  void callbackClosestGreen(const mbzirc_msgs::MbzircBrickConstPtr &msg);
  void callbackClosestBlue(const mbzirc_msgs::MbzircBrickConstPtr &msg);
  void callbackClosestWall(const mbzirc_msgs::MbzircBrickConstPtr &msg);
  void callbackClosestAny(const mbzirc_msgs::MbzircBrickConstPtr &msg);

  bool                          brickVisible(int object_type);
  double                        distToObject();
  void                          changeState(int newState);
  mrs_msgs::TrajectoryReference createTrajectory(int trajectoryType);

private:
  double                                         _g_;
  mrs_lib::quadratic_thrust_model::MotorParams_t _motor_params_;

private:
  void                      callbackAttitudeCommand(const mrs_msgs::AttitudeCommandConstPtr &msg);
  ros::Subscriber           subscriber_attitude_command_;
  bool                      got_attitude_command_ = false;
  mrs_msgs::AttitudeCommand attitude_command_;
  std::mutex                mutex_attitude_command_;

  double landing_uav_mass_;

private:
  Object_t carrying_brick_type_;
};

//}

/* onInit() //{ */

void BrickGrasping::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  mrs_lib::ParamLoader param_loader(nh_, "BrickGrasping");

  ros::Time::waitForValid();

  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam("brick_traverse_constraints", _brick_traverse_constraints_);
  param_loader.loadParam("nobrick_traverse_constraints", _nobrick_traverse_constraints_);

  param_loader.loadParam("rate", _main_rate_);
  param_loader.loadParam("diagnostics_rate", _diagnostics_rate_);
  param_loader.loadParam("gripper_timer_rate", _gripper_timer_rate_);

  param_loader.loadParam("map_long_inactive_time", _map_long_inactive_time_);
  param_loader.loadParam("map_short_inactive_time", _map_short_inactive_time_);

  param_loader.loadParam("trajectory_dt", _trajectory_dt_);

  param_loader.loadParam("number_of_states", _number_of_states_);

  param_loader.loadParam("safety_area_min_height", safety_area_min_height_);

  param_loader.loadParam("fcu_offset/x", _fcu_offset_x_);
  param_loader.loadParam("fcu_offset/y", _fcu_offset_y_);

  param_loader.loadParam("gain_schedulling/wind_suffix", _gains_wind_suffix_);
  param_loader.loadParam("gain_schedulling/nowind_suffix", _gains_nowind_suffix_);
  param_loader.loadParam("gain_schedulling/grasping_gains_name", _grasping_gains_name_);
  param_loader.loadParam("gain_schedulling/world_disturbace_thr", _world_disturbace_thr_);

  // aligning_grasping params
  param_loader.loadParam("stages/aligning_grasping/speed", _aligning_speed_);
  param_loader.loadParam("stages/aligning_grasping/height", _aligning_height_);
  param_loader.loadParam("stages/aligning_grasping/radius", _aligning_radius_);
  param_loader.loadParam("stages/aligning_grasping/timeout", _aligning_timeout_);
  param_loader.loadParam("stages/aligning_grasping/controller", _aligning_controller_);
  param_loader.loadParam("stages/aligning_grasping/odometry/lateral", _aligning_odometry_lateral_);
  param_loader.loadParam("stages/aligning_grasping/odometry/height", _aligning_odometry_height_);
  param_loader.loadParam("stages/aligning_grasping/constraints", _aligning_constraints_);
  param_loader.loadParam("stages/aligning_grasping/gains", _aligning_gains_);

  // descending params
  param_loader.loadParam("stages/descending/speed", _descending_speed_);
  param_loader.loadParam("stages/descending/timeout", _descending_timeout_);
  param_loader.loadParam("stages/descending/height", _descending_height_);
  param_loader.loadParam("stages/descending/controller", _descending_controller_);
  param_loader.loadParam("stages/descending/odometry/lateral", _descending_odometry_lateral_);
  param_loader.loadParam("stages/descending/odometry/height", _descending_odometry_height_);
  param_loader.loadParam("stages/descending/constraints", _descending_constraints_);
  param_loader.loadParam("stages/descending/gains", _descending_gains_);

  // aligning2_grasping params
  param_loader.loadParam("stages/aligning2_grasping/timeout", _aligning2_grasping_timeout_);

  param_loader.loadParam("stages/aligning2_grasping/criterion/initial_x", _aligning2_grasping_criterion_initial_x_);
  param_loader.loadParam("stages/aligning2_grasping/criterion/initial_y", _aligning2_grasping_criterion_initial_y_);
  param_loader.loadParam("stages/aligning2_grasping/criterion/x_increase_rate", _aligning2_grasping_criterion_increase_rate_x_);
  param_loader.loadParam("stages/aligning2_grasping/criterion/y_increase_rate", _aligning2_grasping_criterion_increase_rate_y_);

  param_loader.loadParam("stages/aligning2_grasping/alignment_criterion", _aligning2_grasping_alignment_criterion_);
  param_loader.loadParam("stages/aligning2_grasping/in_alignment_duration", _aligning2_in_alignment_duration_);
  param_loader.loadParam("stages/aligning2_grasping/controller", _aligning2_grasping_controller_);
  param_loader.loadParam("stages/aligning2_grasping/odometry/lateral", _aligning2_grasping_odometry_lateral_);
  param_loader.loadParam("stages/aligning2_grasping/odometry/height", _aligning2_grasping_odometry_height_);
  param_loader.loadParam("stages/aligning2_grasping/constraints", _aligning2_grasping_constraints_);
  param_loader.loadParam("stages/aligning2_grasping/gains", _aligning2_grasping_gains_);

  if (!(_aligning2_grasping_alignment_criterion_ == ALIGNMENT_CRITERION_CONTROL_ERROR ||
        _aligning2_grasping_alignment_criterion_ == ALIGNMENT_CRITERION_BRICK_DETECTION)) {

    ROS_ERROR("[BrickGrasping]: the chosen alignment criterion not valid!");
    ros::shutdown();

  } else {

    std::string criterion_name;

    switch (_aligning2_grasping_alignment_criterion_) {
      case ALIGNMENT_CRITERION_CONTROL_ERROR: {
        criterion_name = "control error";
        break;
      }
      case ALIGNMENT_CRITERION_BRICK_DETECTION: {
        criterion_name = "brick detection";
        break;
      }
    }

    ROS_INFO("[BrickGrasping]: alignment criterion: %s", criterion_name.c_str());
  }

  // grasping params
  param_loader.loadParam("stages/grasping/timeout", _grasping_timeout_);
  param_loader.loadParam("stages/grasping/speed", _grasping_speed_);
  param_loader.loadParam("stages/grasping/height", _grasping_height_);
  param_loader.loadParam("stages/grasping/repeat_threshold", _grasping_repeat_threshold_);
  param_loader.loadParam("stages/grasping/controller", _grasping_controller_);
  param_loader.loadParam("stages/grasping/odometry/lateral", _grasping_odometry_lateral_);
  param_loader.loadParam("stages/grasping/odometry/height", _grasping_odometry_height_);
  param_loader.loadParam("stages/grasping/constraints", _grasping_constraints_);
  param_loader.loadParam("stages/grasping/gains", _grasping_gains_);

  param_loader.loadParam("stages/grasping/thrust_limiter/enabled", _grasping_thrust_limiter_enabled_);
  param_loader.loadParam("stages/grasping/thrust_limiter/thrust_ratio", _grasping_thrust_limiter_ratio_);
  param_loader.loadParam("stages/grasping/thrust_limiter/thrust_timeout", _grasping_thrust_timeout_);

  // aligning_placing params
  param_loader.loadParam("stages/aligning_placing/speed", _aligning_placing_speed_);
  param_loader.loadParam("stages/aligning_placing/radius", _aligning_placing_radius_);
  param_loader.loadParam("stages/aligning_placing/height", _aligning_placing_height_);
  param_loader.loadParam("stages/aligning_placing/timeout", _aligning_placing_timeout_);
  param_loader.loadParam("stages/aligning_placing/safety_area_min_height", _aligning_placing_safety_area_min_height_);
  param_loader.loadParam("stages/aligning_placing/controller", _aligning_placing_controller_);
  param_loader.loadParam("stages/aligning_placing/odometry/lateral", _aligning_placing_odometry_lateral_);
  param_loader.loadParam("stages/aligning_placing/odometry/height", _aligning_placing_odometry_height_);
  param_loader.loadParam("stages/aligning_placing/constraints", _aligning_placing_constraints_);
  param_loader.loadParam("stages/aligning_placing/gains", _aligning_placing_gains_);

  param_loader.loadParam("stages/placing/thrust_limiter/thrust_ratio", _placing_thrust_limiter_ratio_);
  param_loader.loadParam("stages/placing/thrust_limiter/thrust_timeout", _placing_thrust_timeout_);

  // ground placing params
  param_loader.loadParam("stages/ground_placing/speed", _ground_placing_speed_);
  param_loader.loadParam("stages/ground_placing/timeout", _ground_placing_timeout_);
  param_loader.loadParam("stages/ground_placing/height", _ground_placing_height_);
  param_loader.loadParam("stages/ground_placing/controller", _ground_placing_controller_);
  param_loader.loadParam("stages/ground_placing/odometry/lateral", _ground_placing_odometry_lateral_);
  param_loader.loadParam("stages/ground_placing/odometry/height", _ground_placing_odometry_height_);
  param_loader.loadParam("stages/ground_placing/constraints", _ground_placing_constraints_);
  param_loader.loadParam("stages/ground_placing/gains", _ground_placing_gains_);

  param_loader.loadParam("stages/ground_placing/thrust_limiter/thrust_ratio", _ground_placing_thrust_limiter_ratio_);
  param_loader.loadParam("stages/ground_placing/thrust_limiter/thrust_timeout", _ground_placing_thrust_timeout_);

  // placing params
  param_loader.loadParam("stages/placing/speed", _placing_speed_);
  param_loader.loadParam("stages/placing/timeout", _placing_timeout_);
  param_loader.loadParam("stages/placing/height", _placing_height_);
  param_loader.loadParam("stages/placing/controller", _placing_controller_);
  param_loader.loadParam("stages/placing/odometry/lateral", _placing_odometry_lateral_);
  param_loader.loadParam("stages/placing/odometry/height", _placing_odometry_height_);
  param_loader.loadParam("stages/placing/constraints", _placing_constraints_);
  param_loader.loadParam("stages/placing/gains", _placing_gains_);

  // after_placing params
  param_loader.loadParam("stages/after_placing/delay", _after_placing_delay_);
  param_loader.loadParam("stages/after_placing/controller", _after_placing_controller_);
  param_loader.loadParam("stages/after_placing/odometry/lateral", _after_placing_odometry_lateral_);
  param_loader.loadParam("stages/after_placing/odometry/height", _after_placing_odometry_height_);
  param_loader.loadParam("stages/after_placing/constraints", _after_placing_constraints_);
  param_loader.loadParam("stages/after_placing/gains", _after_placing_gains_);

  // repeating params
  param_loader.loadParam("stages/repeating/speed", _repeating_speed_);
  param_loader.loadParam("stages/repeating/height", _repeating_height_);
  param_loader.loadParam("stages/repeating/controller", _repeating_controller_);
  param_loader.loadParam("stages/repeating/odometry/lateral", _repeating_odometry_lateral_);
  param_loader.loadParam("stages/repeating/odometry/height", _repeating_odometry_height_);
  param_loader.loadParam("stages/repeating/constraints", _repeating_constraints_);
  param_loader.loadParam("stages/repeating/gains", _repeating_gains_);

  // ascending params
  param_loader.loadParam("stages/ascending/speed", _ascending_speed_);
  param_loader.loadParam("stages/ascending/height", ascending_height_);
  param_loader.loadParam("stages/ascending/controller", _ascending_controller_);
  param_loader.loadParam("stages/ascending/odometry/lateral", _ascending_odometry_lateral_);
  param_loader.loadParam("stages/ascending/odometry/height", _ascending_odometry_height_);
  param_loader.loadParam("stages/ascending/constraints", _ascending_constraints_);
  param_loader.loadParam("stages/ascending/gains", _ascending_gains_);

  // ascending_after_place params
  param_loader.loadParam("stages/ascending_after_placing/speed", _ascending_after_place_speed_);
  param_loader.loadParam("stages/ascending_after_placing/height", ascending_after_place_height_);
  param_loader.loadParam("stages/ascending_after_placing/controller", _ascending_after_place_controller_);
  param_loader.loadParam("stages/ascending_after_placing/odometry/lateral", _ascending_after_place_odometry_lateral_);
  param_loader.loadParam("stages/ascending_after_placing/odometry/height", _ascending_after_place_odometry_height_);
  param_loader.loadParam("stages/ascending_after_placing/constraints", _ascending_after_place_constraints_);
  param_loader.loadParam("stages/ascending_after_placing/gains", _ascending_after_place_gains_);

  // aborting params
  param_loader.loadParam("stages/aborting/height", aborting_height_);
  param_loader.loadParam("stages/aborting/controller", _aborting_controller_);
  param_loader.loadParam("stages/aborting/odometry/lateral", _aborting_odometry_lateral_);
  param_loader.loadParam("stages/aborting/odometry/height", _aborting_odometry_height_);
  param_loader.loadParam("stages/aborting/constraints", _aborting_constraints_);
  param_loader.loadParam("stages/aborting/gains", _aborting_gains_);

  param_loader.loadParam("loosing_alignment_threshold", _loosing_alignment_threshold_);
  param_loader.loadParam("object_visibility_timeout", _object_visibility_timeout_);
  param_loader.loadParam("not_grasping", _not_grasping_);

  param_loader.loadParam("uav_mass", _uav_mass_);
  param_loader.loadParam("g", _g_);
  param_loader.loadParam("motor_params/a", _motor_params_.A);
  param_loader.loadParam("motor_params/b", _motor_params_.B);
  param_loader.loadParam("motor_params/n_motors", _motor_params_.n_motors);
  param_loader.loadParam("simulation", _simulation_);

  param_loader.loadParam("gripper_filter/buffer_size", _gripper_filter_buffer_size_);
  param_loader.loadParam("gripper_filter/max_difference", _gripper_filter_max_difference_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[BrickGrasping]: [ControlManager]: Could not load all parameters!");
    ros::shutdown();
  }

  median_filter_gripper_            = std::make_unique<MedianFilter>(_gripper_filter_buffer_size_, 2.0, -1.0, _gripper_filter_max_difference_);
  median_filter_simulation_gripper_ = std::make_unique<MedianFilter>(3, 2.0, -1.0, _gripper_filter_max_difference_);

  transformer_ = mrs_lib::Transformer("BrickGrasping", _uav_name_);

  lost_alignment_counter  = 0;
  repeat_grasping_counter = 0;

  publisher_trajectory_                 = nh_.advertise<mrs_msgs::TrajectoryReference>("desired_trajectory_out", 1);
  publisher_diagnostics_                = nh_.advertise<mbzirc_msgs::BrickGraspingDiagnostics>("diagnostics_out", 1);
  publisher_current_target_             = nh_.advertise<mbzirc_msgs::ObjectWithType>("current_target_out", 1);
  publisher_current_target_uav_odom_    = nh_.advertise<geometry_msgs::PoseStamped>("current_target_uav_odom_out", 1);
  publisher_current_target_uav_heading_ = nh_.advertise<mrs_msgs::Float64Stamped>("current_target_uav_heading_out", 1);
  publisher_current_target_debug_       = nh_.advertise<geometry_msgs::PoseStamped>("current_target_debug_out", 1);
  publisher_grasping_result_            = nh_.advertise<std_msgs::Int32>("grasping_result_out", 1);
  publisher_relative_position_          = nh_.advertise<mrs_msgs::ReferenceStamped>("desired_relative_position_out", 1);
  publisher_debug_trajectory_           = nh_.advertise<geometry_msgs::PoseArray>("debug_trajectory_out", 1);

  if (_simulation_) {

    ROS_INFO("[BrickGrasping]: Setting up simulation gripper.");
    simulation_magnet_service_ = nh_.serviceClient<mbzirc_msgs::MagnetControl>("simulation_magnet_out");

  } else {

    ROS_INFO("[BrickGrasping]: Setting up MRS gripper.");

    service_client_gripper_on_  = nh_.serviceClient<std_srvs::Trigger>("gripper_on_out");
    service_client_gripper_off_ = nh_.serviceClient<std_srvs::Trigger>("gripper_off_out");
  }

  odom_rangefinder_service_               = nh_.serviceClient<std_srvs::SetBool>("toggle_rangefinder_out");
  constraints_client_                     = nh_.serviceClient<mrs_msgs::String>("set_constraints_out");
  service_client_switch_controller_       = nh_.serviceClient<mrs_msgs::String>("switch_controller_out");
  service_client_switch_lateral_odometry_ = nh_.serviceClient<mrs_msgs::String>("switch_lateral_odometry_out");
  service_client_switch_height_odometry_  = nh_.serviceClient<mrs_msgs::String>("switch_height_odometry_out");
  service_client_hover_                   = nh_.serviceClient<std_srvs::Trigger>("hover_out");
  service_client_reset_tracker_           = nh_.serviceClient<std_srvs::Trigger>("reset_tracker_out");
  service_client_set_vision_mode_         = nh_.serviceClient<mbzirc_msgs::DetectionType>("vision_mode_out");
  service_client_set_min_height_          = nh_.serviceClient<mrs_msgs::Float64Srv>("set_min_height_out");
  service_client_get_min_height_          = nh_.serviceClient<mrs_msgs::GetFloat64>("get_min_height_out");
  service_client_set_map_inactive_time_   = nh_.serviceClient<mrs_msgs::Float64Srv>("set_map_inactive_time_out");
  service_client_set_avoidance_           = nh_.serviceClient<std_srvs::SetBool>("set_avoidance_out");
  gains_client_                           = nh_.serviceClient<mrs_msgs::String>("set_gains_out");
  service_client_ban_area_                = nh_.serviceClient<mbzirc_msgs::BanArea>("ban_area_out");
  service_client_reset_map_               = nh_.serviceClient<std_srvs::Trigger>("reset_map_out");

  service_server_start_        = nh_.advertiseService("start_in", &BrickGrasping::callbackStart, this);
  service_servcer_stop_        = nh_.advertiseService("stop_in", &BrickGrasping::callbackStop, this);
  service_server_ground_place_ = nh_.advertiseService("ground_place_in", &BrickGrasping::callbackGroundPlace, this);
  service_server_wall_place_   = nh_.advertiseService("wall_place_in", &BrickGrasping::callbackWallPlace, this);

  subscriber_closest_red_   = nh_.subscribe("closest_red_in", 1, &BrickGrasping::callbackClosestRed, this, ros::TransportHints().tcpNoDelay());
  subscriber_closest_green_ = nh_.subscribe("closest_green_in", 1, &BrickGrasping::callbackClosestGreen, this, ros::TransportHints().tcpNoDelay());
  subscriber_closest_blue_  = nh_.subscribe("closest_blue_in", 1, &BrickGrasping::callbackClosestBlue, this, ros::TransportHints().tcpNoDelay());
  subscriber_closest_wall_  = nh_.subscribe("closest_wall_in", 1, &BrickGrasping::callbackClosestWall, this, ros::TransportHints().tcpNoDelay());
  subscriber_closest_any_   = nh_.subscribe("closest_any_in", 1, &BrickGrasping::callbackClosestAny, this, ros::TransportHints().tcpNoDelay());

  subscriber_attitude_command_     = nh_.subscribe("attitude_command_in", 1, &BrickGrasping::callbackAttitudeCommand, this, ros::TransportHints().tcpNoDelay());
  subscriber_odometry_main_        = nh_.subscribe("odometry_main_in", 1, &BrickGrasping::callbackOdometryMain, this, ros::TransportHints().tcpNoDelay());
  subscriber_odometry_diagnostics_ = nh_.subscribe("odom_diagnostics_in", 1, &BrickGrasping::callbackOdomDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_cmd_odom_             = nh_.subscribe("cmd_odom_in", 1, &BrickGrasping::callbackPositionCmd, this, ros::TransportHints().tcpNoDelay());

  // subscribe for simulation gripper
  if (_simulation_) {

    subscriber_simulation_gripper_ =
        nh_.subscribe("simulated_gripper_in", 1, &BrickGrasping::callbackGripperSimulation, this, ros::TransportHints().tcpNoDelay());
    ROS_INFO("[BrickGrasping]: Subscribing simulated gripper.");

  } else {

    subscriber_mrs_gripper_ = nh_.subscribe("mrs_gripper_in", 1, &BrickGrasping::callbackGripper, this, ros::TransportHints().tcpNoDelay());
    ROS_INFO("[BrickGrasping]: Subscribing MRS gripper.");
  }

  // state machine
  current_state_  = IDLE_STATE;
  previous_state_ = IDLE_STATE;

  // grasping action server
  grasping_server_ = new GraspingServer(nh_, ros::this_node::getName(), false);
  grasping_server_->registerGoalCallback(boost::bind(&BrickGrasping::callbackActionServerGoal, this));
  grasping_server_->registerPreemptCallback(boost::bind(&BrickGrasping::callbackActionServerPreempt, this));
  grasping_server_->start();

  last_object_gripped_time_ = ros::Time(0);

  // start timers
  state_machine_timer_  = nh_.createTimer(ros::Rate(_main_rate_), &BrickGrasping::stateMachineTimer, this);
  diagnostics_timer_    = nh_.createTimer(ros::Rate(_diagnostics_rate_), &BrickGrasping::diagnosticsTimer, this);
  current_target_timer_ = nh_.createTimer(ros::Rate(_main_rate_), &BrickGrasping::currentTargetTimer, this);

  if (!_simulation_) {
    gripper_timer_ = nh_.createTimer(ros::Rate(_gripper_timer_rate_), &BrickGrasping::gripperTimer, this);
  }

  grasping_object_type_ = BRICK_ANY;

  is_initialized_ = true;

  ROS_INFO("[BrickGrasping]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackOdometryMain() //{ */

void BrickGrasping::callbackOdometryMain(const nav_msgs::OdometryConstPtr &msg) {

  geometry_msgs::PoseStamped odometry_main;

  odometry_main.header = msg->header;
  odometry_main.pose   = msg->pose.pose;

  // | ---------------- transform to stable origin --------------- |

  {
    auto ret = transformer_.transformSingle("stable_origin", odometry_main);

    if (ret) {

      mrs_lib::set_mutexed(mutex_odometry_main_, ret.value(), odometry_main_stable_);

      got_odometry_main_ = true;
    }
  }

  // | ---------------- transform to brick origin --------------- |

  {
    auto ret = transformer_.transformSingle("brick_origin", odometry_main);

    if (ret) {

      mrs_lib::set_mutexed(mutex_odometry_main_, ret.value(), odometry_main_brick_);
    }
  }
}

//}

/* callbackOdomDiagnostics() //{ */

void BrickGrasping::callbackOdomDiagnostics(const mrs_msgs::OdometryDiagConstPtr &msg) {

  {
    std::scoped_lock lock(mutex_odometry_diag_);

    odometry_diag_ = *msg;

    got_odometry_diag_ = true;
  }
}

//}

/* callbackActionServerPreempt() //{ */

void BrickGrasping::callbackActionServerPreempt() {

  ROS_INFO("[BrickGrasping]: Preemption toggled.");

  // abort the mission
  changeState(PREEMPTED_STATE);
}

//}

/* callbackActionServerGoal() //{ */

void BrickGrasping::callbackActionServerGoal() {

  ROS_INFO("[BrickGrasping]: callbackActionServerGoal begin.");

  auto cmd_odom_stable = mrs_lib::get_mutexed(mutex_cmd_odom_, cmd_odom_stable_);
  auto red_brick       = mrs_lib::get_mutexed(mutex_red_brick_, red_brick_);
  auto green_brick     = mrs_lib::get_mutexed(mutex_green_brick_, green_brick_);
  auto blue_brick      = mrs_lib::get_mutexed(mutex_blue_brick_, blue_brick_);
  auto wall            = mrs_lib::get_mutexed(mutex_wall_, wall_);
  auto any_brick       = mrs_lib::get_mutexed(mutex_any_brick_, any_brick_);

  boost::shared_ptr<const mbzirc_msgs::GraspingGoal> temp_result;
  {
    std::scoped_lock lock(mutex_grasping_server_);

    temp_result = (grasping_server_->acceptNewGoal());
  }

  auto current_goal = mrs_lib::set_mutexed(mutex_current_goal_, *temp_result, current_goal_);

  ROS_INFO_STREAM("[BrickGrasping]: current_goal is " << current_goal.goal);

  if (current_goal.goal == WALL || current_goal.goal == WALL_RED || current_goal.goal == WALL_GREEN || current_goal.goal == WALL_BLUE) {

    if (!objectGripped()) {

      ROS_WARN("[BrickGrasping]: don't have a brick");

      grasping_result_.success   = false;
      grasping_result_.result_id = RESULT_FAILED;
      grasping_result_.message   = "Don't have a brick.";

      changeState(ABORT_STATE);
    }

    if (!brickVisible(WALL)) {

      ROS_WARN("[BrickGrasping]: don't see a wall");

      grasping_result_.success   = false;
      grasping_result_.result_id = RESULT_FAILED;
      grasping_result_.message   = "Don't see a wall.";

      changeState(ABORT_STATE);
    }
  }

  switch (current_goal.goal) {

    case 0: {

      if (brickVisible(BRICK_ANY)) {

        std::scoped_lock lock(mutex_focused_brick_);

        freshFocusedBrick(any_brick);

        ascending_height_ = current_goal.return_altitude;

        grasping_object_type_ = BRICK_ANY;
        setVisionMode(SEE_EVERYTHING);

        ROS_INFO("[BrickGrasping]: Action server triggered grasping of a ANY object.");

      } else {
        ROS_INFO("[BrickGrasping]: Action server do not see ANY object.");
      }

      break;
    }

    case BRICK_RED: {

      std::scoped_lock lock(mutex_focused_brick_);

      freshFocusedBrick(red_brick);

      ascending_height_ = current_goal.return_altitude;

      grasping_object_type_ = BRICK_RED;
      setVisionMode(SEE_RED);

      ROS_INFO("[BrickGrasping]: Action server triggered grasping of a RED object.");

      if (!brickVisible(BRICK_RED)) {
        ROS_WARN("[BrickGrasping]: can not see RED object.");
      }

      break;
    }

    case BRICK_GREEN: {

      std::scoped_lock lock(mutex_focused_brick_);

      freshFocusedBrick(green_brick);

      ascending_height_ = current_goal.return_altitude;

      grasping_object_type_ = BRICK_GREEN;
      setVisionMode(SEE_GREEN);

      ROS_INFO("[BrickGrasping]: Action server triggered grasping of a GREEN object.");

      if (!brickVisible(BRICK_GREEN)) {
        ROS_WARN("[BrickGrasping]: can not see GREEN object.");
      }

      break;
    }

    case BRICK_BLUE: {

      std::scoped_lock lock(mutex_focused_brick_);

      freshFocusedBrick(blue_brick);

      ascending_height_ = current_goal.return_altitude;

      grasping_object_type_ = BRICK_BLUE;
      setVisionMode(SEE_BLUE);

      ROS_INFO("[BrickGrasping]: Action server triggered grasping of a BLUE object.");

      if (!brickVisible(BRICK_BLUE)) {
        ROS_WARN("[BrickGrasping]: can not see BLUE object.");
      }

      break;
    }

    case WALL: {

      std::scoped_lock lock(mutex_focused_brick_);

      freshFocusedBrick(wall);

      grasping_object_type_         = WALL;
      ascending_after_place_height_ = current_goal.return_altitude;

      VisionMode_t desired_vision_mode;

      switch (carrying_brick_type_) {
        case BRICK_RED: {
          desired_vision_mode = SEE_WALL_HAVING_RED;
          break;
        }
        case BRICK_GREEN: {
          desired_vision_mode = SEE_WALL_HAVING_GREEN;
          break;
        }
        case BRICK_BLUE: {
          desired_vision_mode = SEE_WALL_HAVING_BLUE;
          break;
        }
        default: {
          desired_vision_mode = SEE_WALL;
          break;
        }
      }

      setVisionMode(desired_vision_mode);

      ROS_INFO("[BrickGrasping]: Action server triggered placing on the wall with unknown brick.");

      break;
    }

    case WALL_RED: {

      std::scoped_lock lock(mutex_focused_brick_);

      focused_brick_        = wall;
      grasping_object_type_ = WALL;

      ascending_after_place_height_ = current_goal.return_altitude;

      setVisionMode(SEE_WALL_HAVING_RED);

      ROS_INFO("[BrickGrasping]: Action server triggered placing on the wall having red brick.");

      break;
    }

    case WALL_GREEN: {

      std::scoped_lock lock(mutex_focused_brick_);

      focused_brick_        = wall;
      grasping_object_type_ = WALL;

      ascending_after_place_height_ = current_goal.return_altitude;

      setVisionMode(SEE_WALL_HAVING_GREEN);

      ROS_INFO("[BrickGrasping]: Action server triggered placing on the wall having green brick.");

      break;
    }

    case WALL_BLUE: {

      std::scoped_lock lock(mutex_focused_brick_);

      focused_brick_        = wall;
      grasping_object_type_ = WALL;

      ascending_after_place_height_ = current_goal.return_altitude;

      setVisionMode(SEE_WALL_HAVING_BLUE);

      ROS_INFO("[BrickGrasping]: Action server triggered placing on the wall having blue brick.");

      break;
    }

    case BRICK_ANY: {

      std::scoped_lock lock(mutex_focused_brick_);

      freshFocusedBrick(any_brick);
      grasping_object_type_ = BRICK_ANY;

      ascending_height_ = current_goal.return_altitude;

      setVisionMode(SEE_EVERYTHING);

      ROS_INFO("[BrickGrasping]: Action server triggered grasping of a ANY object.");

      break;
    }
  }

  aborting_height_ = current_goal.return_altitude;

  if (current_goal.goal == WALL || current_goal.goal == WALL_RED || current_goal.goal == WALL_GREEN || current_goal.goal == WALL_BLUE) {
    changeState(ALIGN_PLACE_STATE);
  } else {
    changeState(ALIGN_STATE);
  }
}

//}

/* callbackPositionCmd() //{ */

void BrickGrasping::callbackPositionCmd(const nav_msgs::OdometryConstPtr &msg) {

  geometry_msgs::PoseStamped cmd_odom;

  cmd_odom.header = msg->header;
  cmd_odom.pose   = msg->pose.pose;

  // | ---------------- transform to stable origin --------------- |

  {
    auto ret = transformer_.transformSingle("stable_origin", cmd_odom);

    if (ret) {

      mrs_lib::set_mutexed(mutex_cmd_odom_, ret.value(), cmd_odom_stable_);

      got_cmd_odom_ = true;
    }
  }

  // | ---------------- transform to brick origin --------------- |

  {
    auto ret = transformer_.transformSingle("brick_origin", cmd_odom);

    if (ret) {

      mrs_lib::set_mutexed(mutex_cmd_odom_, ret.value(), cmd_odom_brick_);
    }
  }
}

//}

/* callbackClosestRed() //{ */

void BrickGrasping::callbackClosestRed(const mbzirc_msgs::MbzircBrickConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[BrickGrasping]: getting RED");

  // update the stored member
  mrs_lib::set_mutexed(mutex_red_brick_, std::make_tuple(*msg, ros::Time::now()), std::forward_as_tuple(red_brick_, red_brick_last_time_));

  got_brick_ = true;

  // update the currently focused brick
  if (msg->valid) {

    std::scoped_lock lock(mutex_focused_brick_);

    if (msg->type == grasping_object_type_) {

      focused_brick_ = *msg;
    }
  }
}

//}

/* callbackClosestGreen() //{ */

void BrickGrasping::callbackClosestGreen(const mbzirc_msgs::MbzircBrickConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[BrickGrasping]: getting GREEN");

  // update the stored member
  mrs_lib::set_mutexed(mutex_green_brick_, std::make_tuple(*msg, ros::Time::now()), std::forward_as_tuple(green_brick_, green_brick_last_time_));

  got_brick_ = true;

  // update the currently focused brick
  if (msg->valid) {

    std::scoped_lock lock(mutex_focused_brick_);

    if (msg->type == grasping_object_type_) {

      focused_brick_ = *msg;
    }
  }
}

//}

/* callbackClosestBlue() //{ */

void BrickGrasping::callbackClosestBlue(const mbzirc_msgs::MbzircBrickConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[BrickGrasping]: getting BLUE");

  // update the stoblue member
  mrs_lib::set_mutexed(mutex_blue_brick_, std::make_tuple(*msg, ros::Time::now()), std::forward_as_tuple(blue_brick_, blue_brick_last_time_));

  got_brick_ = true;

  // update the currently focused brick
  if (msg->valid) {

    std::scoped_lock lock(mutex_focused_brick_);

    if (msg->type == grasping_object_type_) {

      focused_brick_ = *msg;
    }
  }
}

//}

/* callbackClosestWall() //{ */

void BrickGrasping::callbackClosestWall(const mbzirc_msgs::MbzircBrickConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[BrickGrasping]: getting WALL");

  // update the stoblue member
  mrs_lib::set_mutexed(mutex_wall_, std::make_tuple(*msg, ros::Time::now()), std::forward_as_tuple(wall_, wall_last_time_));

  got_wall_ = true;

  // update the currently focused brick
  if (msg->valid) {

    std::scoped_lock lock(mutex_focused_brick_);

    if (msg->type == grasping_object_type_) {

      focused_brick_ = *msg;
    }
  }
}

//}

/* callbackClosestAny() //{ */

void BrickGrasping::callbackClosestAny(const mbzirc_msgs::MbzircBrickConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[BrickGrasping]: getting ANY");

  // update the stoany member
  mrs_lib::set_mutexed(mutex_any_brick_, std::make_tuple(*msg, ros::Time::now()), std::forward_as_tuple(any_brick_, any_brick_last_time_));

  got_brick_ = true;

  // update the currently focused brick
  if (msg->valid) {

    std::scoped_lock lock(mutex_focused_brick_);

    if (BRICK_ANY == grasping_object_type_) {

      focused_brick_ = *msg;
    }
  }
}

//}

/* callbackGripperSimulation() //{ */

void BrickGrasping::callbackGripperSimulation(const mrs_msgs::GripperDiagnosticsConstPtr &msg) {

  std::scoped_lock lock(mutex_object_gripped_);

  ROS_INFO_ONCE("[BrickGrasping]: getting gripper feedback");

  median_filter_simulation_gripper_->isValid(double(msg->gripping_object));

  if (median_filter_simulation_gripper_->isFilled()) {

    if (median_filter_simulation_gripper_->getMedian() > 0) {

      last_object_gripped_time_ = ros::Time::now();
      object_gripped_           = true;

    } else {

      last_object_gripped_time_ = ros::Time(0);
      object_gripped_           = false;
    }
  }
}

//}

/* callbackGripper() //{ */

void BrickGrasping::callbackGripper(const mrs_msgs::GripperDiagnosticsConstPtr &msg) {

  std::scoped_lock lock(mutex_object_gripped_);

  ROS_INFO_ONCE("[BrickGrasping]: getting gripper feedback");

  median_filter_gripper_->isValid(double(msg->gripping_object));

  if (median_filter_gripper_->isFilled()) {

    if (median_filter_gripper_->getMedian() > 0) {

      last_object_gripped_time_ = ros::Time::now();
      object_gripped_           = true;

    } else {

      last_object_gripped_time_ = ros::Time(0);
      object_gripped_           = false;
    }
  }
}

//}

/* //{ callbackAttitudeCommand() */

void BrickGrasping::callbackAttitudeCommand(const mrs_msgs::AttitudeCommandConstPtr &msg) {

  if (!is_initialized_)
    return;

  {
    std::scoped_lock lock(mutex_attitude_command_);

    attitude_command_ = *msg;

    got_attitude_command_ = true;
  }
}

//}

// | ------------------------ services ------------------------ |

/* callbackStop() //{ */

bool BrickGrasping::callbackStop([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  res.success = true;
  res.message = "Switching to ABORT state";

  grasping_result_.success   = false;
  grasping_result_.result_id = RESULT_STOPPED_BY_COMMAND;
  grasping_result_.message   = "Stopped by command.";

  changeState(ABORT_STATE);

  return true;
}

//}

/* callbackWallPlace() //{ */

bool BrickGrasping::callbackWallPlace([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  auto wall = mrs_lib::get_mutexed(mutex_wall_, wall_);

  if (!objectGripped()) {

    res.success = false;
    res.message = "Don't have a brick";
    return true;
  }

  if (!brickVisible(WALL)) {

    res.success = false;
    res.message = "Don't see a wall";
    return true;
  }

  {
    std::scoped_lock lock(mutex_focused_brick_);

    freshFocusedBrick(wall);

    grasping_object_type_ = WALL;

    VisionMode_t desired_vision_mode;

    switch (carrying_brick_type_) {
      case BRICK_RED: {
        desired_vision_mode = SEE_WALL_HAVING_RED;
        break;
      }
      case BRICK_GREEN: {
        desired_vision_mode = SEE_WALL_HAVING_GREEN;
        break;
      }
      case BRICK_BLUE: {
        desired_vision_mode = SEE_WALL_HAVING_BLUE;
        break;
      }
      default: {
        desired_vision_mode = SEE_WALL;
        break;
      }
    }

    setVisionMode(desired_vision_mode);
  }

  changeState(ALIGN_PLACE_STATE);

  res.success = true;
  res.message = "Placing the brick on the wall.";

  return true;
}

//}

/* callbackGroundPlace() //{ */

bool BrickGrasping::callbackGroundPlace([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!objectGripped()) {

    res.success = false;
    res.message = "Cannot place, don't have anything.";

  } else {

    changeState(GROUND_PLACING_STATE);

    res.success = true;
    res.message = "Placing the brick on the ground.";
  }

  return true;
}

//}

/* callbackStart() //{ */

bool BrickGrasping::callbackStart(mbzirc_msgs::StartGrasping::Request &req, mbzirc_msgs::StartGrasping::Response &res) {

  auto cmd_odom_stable = mrs_lib::get_mutexed(mutex_cmd_odom_, cmd_odom_stable_);
  auto red_brick       = mrs_lib::get_mutexed(mutex_red_brick_, red_brick_);
  auto green_brick     = mrs_lib::get_mutexed(mutex_green_brick_, green_brick_);
  auto blue_brick      = mrs_lib::get_mutexed(mutex_blue_brick_, blue_brick_);
  auto wall            = mrs_lib::get_mutexed(mutex_wall_, wall_);
  auto any_brick       = mrs_lib::get_mutexed(mutex_any_brick_, any_brick_);

  if (current_state_ == IDLE_STATE || current_state_ == ABORT_STATE) {

    if (req.goal == BRICK_RED || req.goal == BRICK_GREEN || req.goal == BRICK_BLUE || req.goal == BRICK_ANY) {

      if (objectGripped()) {

        res.success = false;
        res.message = "Already have a brick.";
        return true;
      }
    }

    // pick the closest
    if (req.goal == 0) {

      if (brickVisible(BRICK_ANY)) {
        grasping_object_type_ = BRICK_ANY;
      } else {

        res.success = false;
        res.message = "No object visible.";
        return true;
      }

    } else if (brickVisible(req.goal)) {

      switch (req.goal) {

        case BRICK_RED: {

          std::scoped_lock lock(mutex_focused_brick_);

          freshFocusedBrick(red_brick);

          grasping_object_type_ = req.goal;
          setVisionMode(SEE_RED);

          break;
        }

        case BRICK_GREEN: {

          std::scoped_lock lock(mutex_focused_brick_);

          freshFocusedBrick(green_brick);

          grasping_object_type_ = req.goal;
          setVisionMode(SEE_GREEN);

          break;
        }

        case BRICK_BLUE: {

          std::scoped_lock lock(mutex_focused_brick_);

          freshFocusedBrick(blue_brick);

          grasping_object_type_ = req.goal;
          setVisionMode(SEE_BLUE);

          break;
        }

        case WALL: {

          std::scoped_lock lock(mutex_focused_brick_);

          freshFocusedBrick(wall);

          grasping_object_type_ = req.goal;

          // TODO: add WALL+BRICK mode
          setVisionMode(SEE_WALL);

          break;
        }

        case BRICK_ANY: {

          std::scoped_lock lock(mutex_focused_brick_);

          freshFocusedBrick(any_brick);

          grasping_object_type_ = req.goal;
          setVisionMode(SEE_EVERYTHING);

          break;
        }
      }

    } else {

      res.success = false;
      res.message = "No object visible.";
      return true;
    }

    if (req.goal == WALL) {
      changeState(ALIGN_PLACE_STATE);
    } else {
      changeState(ALIGN_STATE);
    }

    res.success = true;
    res.message = "Grasping started.";

  } else {

    res.success = false;
    res.message = "Cannot start, already in action.";
  }

  return true;
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* publishDiagnostics() //{ */

void BrickGrasping::publishDiagnostics(void) {

  auto focused_brick        = mrs_lib::get_mutexed(mutex_focused_brick_, focused_brick_);
  auto odometry_main_stable = mrs_lib::get_mutexed(mutex_odometry_main_, odometry_main_stable_);

  mbzirc_msgs::BrickGraspingDiagnostics diagnostics;

  diagnostics.stamp  = ros::Time::now();
  diagnostics.state  = state_names[current_state_];
  diagnostics.active = ((current_state_ == 0) ? false : true);

  {
    auto res = transformer_.transformSingle("utm_origin", odometry_main_stable);

    geometry_msgs::PoseStamped odom_utm;

    if (res) {

      diagnostics.position = res.value().pose;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[BrickGrasping]: could not transform diagnostics to UTM");
      return;
    }
  }

  {

    geometry_msgs::PoseStamped focused_brick_stable;
    focused_brick_stable.header.frame_id = "stable_origin";
    focused_brick_stable.header.stamp    = ros::Time::now();

    focused_brick_stable.pose.position.x    = focused_brick.states[POS_X];
    focused_brick_stable.pose.position.y    = focused_brick.states[POS_Y];
    focused_brick_stable.pose.position.z    = focused_brick.states[POS_Z];
    focused_brick_stable.pose.orientation.w = 1.0;

    auto res = transformer_.transformSingle("utm_origin", focused_brick_stable);

    if (res) {

      diagnostics.target = res.value().pose;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[BrickGrasping]: could not transform diagnostics to UTM");
      return;
    }
  }

  try {
    publisher_diagnostics_.publish(diagnostics);
  }
  catch (...) {
    ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_diagnostics_.getTopic().c_str());
  }

  if (grasping_server_->isActive())
    grasping_server_->publishFeedback(grasping_feedback_);
}

//}

/* magnetToggle() //{ */

void BrickGrasping::magnetToggle(bool in) {

  if (_simulation_) {

    mbzirc_msgs::MagnetControl msg;
    msg.request.turnOnOff = in;
    simulation_magnet_service_.call(msg);
    ROS_INFO("[BrickGrasping]: Setting simulated magnetic gripper to %d", in);

  } else {

    gripper_ = in;

    std_srvs::Trigger msg;

    bool res;
    if (in) {
      res = service_client_gripper_on_.call(msg);
    } else {
      res = service_client_gripper_off_.call(msg);
    }

    if (!res) {
      ROS_ERROR_THROTTLE(1.0, "[BrickGrasping]: service call for gripper failed");
    }

    ROS_INFO("[BrickGrasping]: Setting mrs magnetic gripper to %d", in);
  }
}

//}

/* brickVisible() //{ */

bool BrickGrasping::brickVisible(int type) {

  switch (Object_t(type)) {

    case BRICK_RED: {

      return ((ros::Time::now() - red_brick_last_time_).toSec() < _object_visibility_timeout_);

      break;
    }

    case BRICK_GREEN: {

      return ((ros::Time::now() - green_brick_last_time_).toSec() < _object_visibility_timeout_);

      break;
    }

    case BRICK_BLUE: {

      return ((ros::Time::now() - blue_brick_last_time_).toSec() < _object_visibility_timeout_);

      break;
    }

    case WALL: {

      return ((ros::Time::now() - wall_last_time_).toSec() < _object_visibility_timeout_);

      break;
    }

    case BRICK_ANY: {

      return ((ros::Time::now() - any_brick_last_time_).toSec() < _object_visibility_timeout_);

      break;
    }

    case WALL_RED:
      break;
    case WALL_BLUE:
      break;
    case WALL_GREEN:
      break;
  }

  return false;
}

//}

//}

/* publishDebugTrajectory() //{ */

void BrickGrasping::publishDebugTrajectory(mrs_msgs::TrajectoryReference &trajectory) {

  // prepare the trajectorie
  // pose array for debugging
  geometry_msgs::PoseArray array;
  tf::Quaternion           orientation;
  array.header = trajectory.header;

  // fill the trajectory
  for (size_t i = 0; i < trajectory.points.size(); i++) {

    geometry_msgs::Pose pose;
    pose.position.x = trajectory.points[i].position.x;
    pose.position.y = trajectory.points[i].position.y;
    pose.position.z = 0;
    orientation.setEuler(0, 0, trajectory.points[i].heading);
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    array.poses.push_back(pose);
  }

  // publish the trajectory
  try {
    publisher_debug_trajectory_.publish(array);
  }
  catch (...) {
    ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_debug_trajectory_.getTopic().c_str());
  }
}

//}

/* changeState() //{ */

void BrickGrasping::changeState(int newState) {

  // copy member variables
  auto attitude_command = mrs_lib::get_mutexed(mutex_attitude_command_, attitude_command_);
  auto cmd_odom_stable  = mrs_lib::get_mutexed(mutex_cmd_odom_, cmd_odom_stable_);
  auto focused_brick    = mrs_lib::get_mutexed(mutex_focused_brick_, focused_brick_);

  // just for ROS_INFO
  ROS_INFO("[BrickGrasping]: Switching states: %s -> %s", state_names[current_state_], state_names[newState]);

  timeouter_ = ros::Time::now();

  previous_state_ = current_state_;
  current_state_  = newState;

  grasping_feedback_.stage   = newState;
  grasping_feedback_.message = state_names[newState];

  if (grasping_server_->isActive()) {
    grasping_server_->publishFeedback(grasping_feedback_);
  }

  publishDiagnostics();

  // if changing to idle, stop the drone
  switch (newState) {

      /* IDLE_STATE //{ */

    case IDLE_STATE:

      lost_alignment_counter  = 0;
      repeat_grasping_counter = 0;

      setAvoidance(true);

      if (previous_state_ == ASCEND_STATE) {

        ROS_INFO("[BrickGrasping]: setting constraints for traversing with a brick");
        setConstraints(_brick_traverse_constraints_);

      } else if (previous_state_ == ASCEND_AFTER_PLACE_STATE) {

        ROS_INFO("[BrickGrasping]: setting constraints for traversing without a brick");
        setConstraints(_nobrick_traverse_constraints_);
      }

      setMinHeight(original_min_height_);

      break;

      //}

      /* ALIGN_STATE //{ */

    case ALIGN_STATE:

      // decide which gains should be used
      high_wind_situation_here_ = isHighWind();

      setController(_aligning_controller_);
      setOdometry(_aligning_odometry_lateral_, _aligning_odometry_height_);
      setConstraints(_aligning_constraints_);
      setGains(_aligning_gains_);
      setAvoidance(false);
      setMapInactiveTime(_map_short_inactive_time_);

      original_min_height_ = getMinHeight();
      setMinHeight(safety_area_min_height_);

      break;

      //}

      /* DESCEND_STATE //{ */

    case DESCEND_STATE:

      setController(_descending_controller_);
      setOdometry(_descending_odometry_lateral_, _descending_odometry_height_);
      setConstraints(_descending_constraints_);
      setGains(_descending_gains_);
      resetMap();

      break;

      //}

      /* ALIGN2_GRASP_STATE //{ */

    case ALIGN2_GRASP_STATE:

      setController(_aligning2_grasping_controller_);
      setOdometry(_aligning2_grasping_odometry_lateral_, _aligning2_grasping_odometry_height_);
      setConstraints(_aligning2_grasping_constraints_);
      setGains(_aligning2_grasping_gains_);

      aligning2_in_radius_time_ = ros::Time(0);
      aligning2_in_radius_      = false;
      aligning2_current_x_crit_ = _aligning2_grasping_criterion_initial_x_;
      aligning2_current_y_crit_ = _aligning2_grasping_criterion_initial_y_;

      break;

      //}

      /* GRASP_STATE //{ */

    case GRASP_STATE:

      // log when we started the grasping
      grasping_time_ = ros::Time::now();

      setController(_grasping_controller_);
      setOdometry(_grasping_odometry_lateral_, _grasping_odometry_height_);
      setConstraints(_grasping_constraints_);
      setGains(_grasping_gains_);

      landing_uav_mass_ = attitude_command.total_mass;

      magnetToggle(true);

      break;

      //}

      /* REPEAT_STATE //{ */

    case REPEAT_STATE:

      magnetToggle(false);

      setController(_repeating_controller_);
      setOdometry(_repeating_odometry_lateral_, _repeating_odometry_height_);
      setConstraints(_repeating_constraints_);
      setGains(_repeating_gains_);

      if (repeat_grasping_counter++ >= _grasping_repeat_threshold_) {

        ROS_INFO("[BrickGrasping]: Exceeded the number of grasping attemptes, going up");

        grasping_result_.success   = false;
        grasping_result_.result_id = RESULT_FAILED;
        grasping_result_.message   = "Too many attempts.";

        changeState(ABORT_STATE);
      }

      if ((repeat_grasping_counter % 3) == 0) {

        mbzirc_msgs::BanArea ban_area;

        ban_area.request.x = focused_brick.states[POS_X];
        ban_area.request.y = focused_brick.states[POS_Y];

        bool res = service_client_ban_area_.call(ban_area);

        ROS_INFO("[BrickGrasping]: banning this brick's area");

        if (!res) {

          ROS_ERROR("[BrickGrasping]: could not call the service for banning this area");

        } else {

          if (!ban_area.response.success) {
            ROS_ERROR("[BrickGrasping]: the service for bannig this area returned false: %s", ban_area.response.message.c_str());
          }
        }
      }

      break;

      //}

      /* ABORT_STATE //{ */

    case ABORT_STATE:

      magnetToggle(false);

      setController(_aborting_controller_);
      setOdometry(_aborting_odometry_lateral_, _aborting_odometry_height_);
      setConstraints(_aborting_constraints_);
      setGains(_aborting_gains_);
      setVisionMode(SEE_EVERYTHING);
      setMapInactiveTime(_map_short_inactive_time_);

      break;

      //}

      /* ASCEND_STATE //{ */

    case ASCEND_STATE:

      setController(_ascending_controller_);
      setOdometry(_ascending_odometry_lateral_, _ascending_odometry_height_);
      setConstraints(_ascending_constraints_);
      setGains(_ascending_gains_);
      setMapInactiveTime(_map_long_inactive_time_);

      ascending_initial_height_ = cmd_odom_stable.pose.position.z;

      break;

      //}

      /* ASCEND_AFTER_PLACE_STATE //{ */

    case ASCEND_AFTER_PLACE_STATE:

      setController(_ascending_after_place_controller_);
      setOdometry(_ascending_after_place_odometry_lateral_, _ascending_after_place_odometry_height_);
      setConstraints(_ascending_after_place_constraints_);
      setGains(_ascending_after_place_gains_);
      setVisionMode(SEE_EVERYTHING);
      setMapInactiveTime(_map_short_inactive_time_);

      ascending_initial_height_ = cmd_odom_stable.pose.position.z;

      break;

      //}

      /* ALIGN_PLACE_STATE //{ */

    case ALIGN_PLACE_STATE:

      setController(_aligning_placing_controller_);
      setOdometry(_aligning_placing_odometry_lateral_, _aligning_placing_odometry_height_);
      setConstraints(_aligning_placing_constraints_);
      setGains(_aligning_placing_gains_);
      setAvoidance(false);

      original_min_height_ = getMinHeight();
      setMinHeight(_aligning_placing_safety_area_min_height_);

      break;

      //}

      /* WALL_PLACING_STATE //{ */

    case WALL_PLACING_STATE:

      setController(_placing_controller_);
      setOdometry(_placing_odometry_lateral_, _placing_odometry_height_);
      setConstraints(_placing_constraints_);
      setGains(_placing_gains_);

      landing_uav_mass_ = attitude_command.total_mass;

      break;

      //}

      /* GROUND_PLACING_STATE //{ */

    case GROUND_PLACING_STATE:

      setController(_ground_placing_controller_);
      setOdometry(_ground_placing_odometry_lateral_, _ground_placing_odometry_height_);
      setConstraints(_ground_placing_constraints_);
      setGains(_ground_placing_gains_);

      original_min_height_ = getMinHeight();
      setMinHeight(safety_area_min_height_);

      landing_uav_mass_ = attitude_command.total_mass;

      break;

      //}

      /* WAITING_AFTER_PLACING //{ */

    case WAITING_AFTER_PLACING:

      placing_time_ = ros::Time::now();

      resetTracker();
      setController(_after_placing_controller_);
      setOdometry(_after_placing_odometry_lateral_, _after_placing_odometry_height_);
      setConstraints(_after_placing_constraints_);
      setGains(_after_placing_gains_);

      break;

      //}

      /* PREEMPTED_STATE //{ */

    case PREEMPTED_STATE:

      setController(_ascending_controller_);
      setOdometry(_ascending_odometry_lateral_, _ascending_odometry_height_);
      setConstraints(_ascending_constraints_);
      setGains(_ascending_gains_);
      setVisionMode(SEE_EVERYTHING);
      setMapInactiveTime(_map_short_inactive_time_);

      magnetToggle(false);

      break;

      //}
  }
}

//}

/* objectMotionModel() //{ */

Eigen::VectorXd BrickGrasping::objectMotionModel(const Eigen::VectorXd in, const double dt) {

  // set output to the input
  VectorXd out = VectorXd::Zero(_number_of_states_);
  out          = in;

  // update position x
  out(0) = in(0) + in(1) * dt;

  // update velocity x
  out(1) = in(4) * cos(in(5));

  // update position y
  out(2) = in(2) + in(3) * dt;

  // update velocity y
  out(3) = in(4) * sin(in(5));

  // update the speed
  out(4) = in(4) + in(6) * dt;

  // update the angle
  out(5) = in(5) + in(7) * dt;

  // update the acceleration
  out(6) = in(6);

  // update the anglular speed
  out(7) = in(8) * in(4);

  // update the curvature
  out(8) = in(8) + in(9) * dt;

  return out;
}

//}

/* createTrajectory() //{ */

mrs_msgs::TrajectoryReference BrickGrasping::createTrajectory(int trajectoryType) {

  auto [cmd_odom_stable, cmd_odom_brick] = mrs_lib::get_mutexed(mutex_cmd_odom_, cmd_odom_stable_, cmd_odom_brick_);
  auto focused_brick                     = mrs_lib::get_mutexed(mutex_focused_brick_, focused_brick_);

  double brick_stable_x       = focused_brick.states[POS_X];
  double brick_stable_y       = focused_brick.states[POS_Y];
  double brick_stable_z       = focused_brick.states[POS_Z];
  double brick_stable_heading = focused_brick.yaw;

  double cmd_odom_brick_heading = 0;

  try {
    cmd_odom_brick_heading = mrs_lib::AttitudeConverter(cmd_odom_brick.pose.orientation).getHeading();
  }
  catch (...) {
  }

  double cmd_odom_stable_heading = 0;

  try {
    cmd_odom_stable_heading = mrs_lib::AttitudeConverter(cmd_odom_stable.pose.orientation).getHeading();
  }
  catch (...) {
  }

  // fcu offset
  geometry_msgs::Vector3Stamped fcu_offset;
  fcu_offset.header.frame_id = "fcu_untilted";
  fcu_offset.vector.x        = _fcu_offset_x_;
  fcu_offset.vector.y        = _fcu_offset_y_;
  fcu_offset.vector.z        = _fcu_offset_y_;

  // prepare the trajectorie
  // pose array for debugging
  mrs_msgs::TrajectoryReference trajectory;
  tf::Quaternion                orientation;
  trajectory.fly_now     = true;
  trajectory.use_heading = true;

  /* ALIGN_TRAJECTORY //{ */

  if (trajectoryType == ALIGN_TRAJECTORY) {

    double target_heading, target_distance, desired_heading;

    trajectory.header.stamp    = cmd_odom_stable.header.stamp;
    trajectory.header.frame_id = "stable_origin";
    target_heading             = atan2(brick_stable_y - cmd_odom_stable.pose.position.y, brick_stable_x - cmd_odom_stable.pose.position.x);
    target_distance = mrs_lib::geometry::dist(vec2_t(cmd_odom_stable.pose.position.x, cmd_odom_stable.pose.position.y), vec2_t(brick_stable_x, brick_stable_y));
    desired_heading = fabs(radians::dist(brick_stable_heading, cmd_odom_stable_heading)) < (M_PI / 2.0) ? brick_stable_heading : brick_stable_heading + M_PI;

    /* double desired_height = _aligning_height_ > cmd_odom_stable.pose.position.z ? cmd_odom_stable.pose.position.z : _aligning_height_; */
    double desired_height = brick_stable_z + _aligning_height_;

    double step_size = _aligning_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      point.position.x = cmd_odom_stable.pose.position.x;
      point.position.y = cmd_odom_stable.pose.position.y;
      point.position.z = desired_height;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    for (int i = 0; i < n_steps; i++) {

      mrs_msgs::Reference point;

      point.position.x = trajectory.points.back().position.x + cos(target_heading) * step_size;
      point.position.y = trajectory.points.back().position.y + sin(target_heading) * step_size;
      point.position.z = desired_height;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // the last point = the brick
    {
      mrs_msgs::Reference point;

      point.position.x = brick_stable_x;
      point.position.y = brick_stable_y;
      point.position.z = desired_height;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);

      {
        std::scoped_lock lock(mutex_current_target_);

        current_target_.header.frame_id      = trajectory.header.frame_id;
        current_target_.reference.position.x = point.position.x;
        current_target_.reference.position.y = point.position.y;
        current_target_.reference.position.z = point.position.z;
        current_target_.reference.heading    = point.heading;
      }
    }

    // return the trajectory
    return trajectory;

    //}

    /* DESCEND_TRAJECTORY //{ */

  } else if (trajectoryType == DESCEND_TRAJECTORY) {

    trajectory.header.stamp = cmd_odom_stable.header.stamp;

    double target_distance, direction, desired_height, desired_vector, desired_heading;

    if (_aligning_odometry_lateral_ == "brick") {
      trajectory.header.stamp    = cmd_odom_brick.header.stamp;
      trajectory.header.frame_id = "brick_origin";
      desired_height             = _descending_height_;
      desired_vector             = desired_height - cmd_odom_brick.pose.position.z;
      target_distance            = fabs(desired_vector);
      direction                  = (desired_vector <= 0) ? -1 : 1;
      desired_heading            = fabs(radians::diff(0.0, cmd_odom_brick_heading)) < (M_PI / 2.0) ? 0 : M_PI;
    } else {
      trajectory.header.stamp    = cmd_odom_stable.header.stamp;
      trajectory.header.frame_id = "stable_origin";
      /* desired_height             = brick_stable_z + _descending_height_; */
      desired_height  = _descending_height_;
      desired_vector  = desired_height - cmd_odom_stable.pose.position.z;
      target_distance = fabs(desired_vector);
      direction       = (desired_vector <= 0) ? -1 : 1;
      desired_heading = fabs(radians::diff(brick_stable_heading, cmd_odom_stable_heading)) < (M_PI / 2.0) ? brick_stable_heading : brick_stable_heading + M_PI;
    }

    double step_size = _descending_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      if (_aligning_odometry_lateral_ == "brick") {
        point.position.x = 0;
        point.position.y = 0;
        point.position.z = cmd_odom_brick.pose.position.z;
      } else {
        point.position.x = brick_stable_x;
        point.position.y = brick_stable_y;
        point.position.z = cmd_odom_stable.pose.position.z;
      }

      point.heading = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::Reference point;

      for (int i = 0; i < n_steps; i++) {

        if (_aligning_odometry_lateral_ == "brick") {
          point.position.x = 0;
          point.position.y = 0;
        } else {
          point.position.x = brick_stable_x;
          point.position.y = brick_stable_y;
        }

        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = desired_heading;

        trajectory.points.push_back(point);
      }
    }

    // the last point
    {
      mrs_msgs::Reference point;

      if (_aligning_odometry_lateral_ == "brick") {
        point.position.x = 0;
        point.position.y = 0;
      } else {
        point.position.x = brick_stable_x;
        point.position.y = brick_stable_y;
      }

      point.position.z = _descending_height_;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);

      {
        std::scoped_lock lock(mutex_current_target_);

        current_target_.header.frame_id      = trajectory.header.frame_id;
        current_target_.reference.position.x = point.position.x;
        current_target_.reference.position.y = point.position.y;
        current_target_.reference.position.z = point.position.z;
        current_target_.reference.heading    = point.heading;
      }
    }

    return trajectory;

    //}

    /* ASCEND_TRAJECTORY //{ */

  } else if (trajectoryType == ASCEND_TRAJECTORY) {

    double desired_height, desired_vector, target_distance, direction;

    trajectory.header.stamp    = cmd_odom_stable.header.stamp;
    trajectory.header.frame_id = "stable_origin";
    desired_height             = ascending_height_;
    desired_vector             = desired_height - cmd_odom_stable.pose.position.z;
    target_distance            = fabs(desired_vector);
    direction                  = (desired_vector <= 0) ? -1 : 1;

    double step_size = _ascending_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      point.position.x = cmd_odom_stable.pose.position.x;
      point.position.y = cmd_odom_stable.pose.position.y;
      point.position.z = cmd_odom_stable.pose.position.z;
      point.heading    = cmd_odom_stable_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::Reference point;

      for (int i = 0; i < n_steps; i++) {

        point.position.x = cmd_odom_stable.pose.position.x;
        point.position.y = cmd_odom_stable.pose.position.y;
        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = cmd_odom_stable_heading;

        trajectory.points.push_back(point);
      }
    }

    {
      std::scoped_lock lock(mutex_current_target_);

      current_target_.header.frame_id      = trajectory.header.frame_id;
      current_target_.reference.position.x = cmd_odom_stable.pose.position.x;
      current_target_.reference.position.y = cmd_odom_stable.pose.position.y;
      current_target_.reference.position.z = desired_height;
      current_target_.reference.heading    = cmd_odom_stable_heading;
    }

    return trajectory;

    //}

    /* ASCEND_AFTER_PLACE_TRAJECTORY //{ */

  } else if (trajectoryType == ASCEND_AFTER_PLACE_TRAJECTORY) {

    double desired_height, desired_vector, target_distance, direction;

    trajectory.header.stamp    = cmd_odom_stable.header.stamp;
    trajectory.header.frame_id = "stable_origin";
    desired_height             = ascending_after_place_height_;
    desired_vector             = desired_height - cmd_odom_stable.pose.position.z;
    target_distance            = fabs(desired_vector);
    direction                  = (desired_vector <= 0) ? -1 : 1;

    double step_size = _ascending_after_place_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      point.position.x = cmd_odom_stable.pose.position.x;
      point.position.y = cmd_odom_stable.pose.position.y;
      point.position.z = cmd_odom_stable.pose.position.z;
      point.heading    = cmd_odom_stable_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::Reference point;

      for (int i = 0; i < n_steps; i++) {

        point.position.x = cmd_odom_stable.pose.position.x;
        point.position.y = cmd_odom_stable.pose.position.y;
        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = cmd_odom_stable_heading;

        trajectory.points.push_back(point);
      }
    }

    {
      std::scoped_lock lock(mutex_current_target_);

      current_target_.header.frame_id      = trajectory.header.frame_id;
      current_target_.reference.position.x = cmd_odom_stable.pose.position.x;
      current_target_.reference.position.y = cmd_odom_stable.pose.position.y;
      current_target_.reference.position.z = desired_height;
      current_target_.reference.heading    = cmd_odom_stable_heading;
    }

    return trajectory;

    //}

    /* GRASPING_TRAJECTORY //{ */

  } else if (trajectoryType == GRASPING_TRAJECTORY) {

    double desired_heading;

    if (_aligning_odometry_lateral_ == "brick") {
      trajectory.header.stamp    = cmd_odom_brick.header.stamp;
      trajectory.header.frame_id = "brick_origin";
      desired_heading            = fabs(radians::diff(0.0, cmd_odom_brick_heading)) < (M_PI / 2.0) ? 0 : M_PI;
    } else {
      trajectory.header.stamp    = cmd_odom_stable.header.stamp;
      trajectory.header.frame_id = "stable_origin";
      desired_heading = fabs(radians::diff(brick_stable_heading, cmd_odom_stable_heading)) < (M_PI / 2.0) ? brick_stable_heading : brick_stable_heading + M_PI;
    }

    auto res = transformer_.transformSingle(trajectory.header.frame_id, fcu_offset);

    geometry_msgs::Vector3Stamped current_offset;

    if (res) {

      current_offset = res.value();

    } else {

      ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: could not transform fcu offset to %s", trajectory.header.frame_id.c_str());

      current_offset.vector.x = 0;
      current_offset.vector.y = 0;
    }

    double target_distance = fabs(_grasping_height_);
    double direction       = -1;
    double step_size       = _grasping_speed_ * _trajectory_dt_;
    int    n_steps         = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      if (_aligning_odometry_lateral_ == "brick") {
        point.position.x = current_offset.vector.x;
        point.position.y = current_offset.vector.y;
        point.position.z = cmd_odom_brick.pose.position.z;
      } else {
        point.position.x = brick_stable_x + current_offset.vector.x;
        point.position.y = brick_stable_y + current_offset.vector.y;
        point.position.z = cmd_odom_stable.pose.position.z;
      }

      point.heading = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::Reference point;

      for (int i = 0; i < n_steps; i++) {

        if (_aligning_odometry_lateral_ == "brick") {
          point.position.x = current_offset.vector.x;
          point.position.y = current_offset.vector.y;
        } else {
          point.position.x = brick_stable_x + current_offset.vector.x;
          point.position.y = brick_stable_y + current_offset.vector.y;
        }

        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = desired_heading;

        trajectory.points.push_back(point);
      }
    }

    // the last point
    {
      mrs_msgs::Reference point;

      if (_aligning_odometry_lateral_ == "brick") {
        point.position.x = current_offset.vector.x;
        point.position.y = current_offset.vector.y;
      } else {
        point.position.x = brick_stable_x + current_offset.vector.x;
        point.position.y = brick_stable_y + current_offset.vector.y;
      }

      point.position.z = cmd_odom_stable.pose.position.z + _grasping_height_;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    return trajectory;

    //}

    /* REPEAT_TRAJECTORY //{ */

  } else if (trajectoryType == REPEAT_TRAJECTORY) {

    double target_distance, desired_height, desired_vector, direction, desired_heading;

    trajectory.header.stamp    = cmd_odom_stable.header.stamp;
    trajectory.header.frame_id = "stable_origin";
    desired_height             = _repeating_height_;
    desired_vector             = desired_height - cmd_odom_stable.pose.position.z;
    target_distance            = fabs(desired_vector);
    direction                  = (desired_vector <= 0) ? -1 : 1;
    desired_heading = fabs(radians::diff(brick_stable_heading, cmd_odom_stable_heading)) < (M_PI / 2.0) ? brick_stable_heading : brick_stable_heading + M_PI;

    double step_size = _repeating_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      point.position.x = brick_stable_x;
      point.position.y = brick_stable_y;
      point.position.z = cmd_odom_stable.pose.position.z;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::Reference point;

      for (int i = 0; i < n_steps; i++) {

        point.position.x = brick_stable_x;
        point.position.y = brick_stable_y;
        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = desired_heading;

        trajectory.points.push_back(point);
      }
    }

    // the last point
    {
      mrs_msgs::Reference point;

      point.position.x = brick_stable_x;
      point.position.y = brick_stable_y;
      point.position.z = _repeating_height_;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);

      {
        std::scoped_lock lock(mutex_current_target_);

        current_target_.header.frame_id      = trajectory.header.frame_id;
        current_target_.reference.position.x = point.position.x;
        current_target_.reference.position.y = point.position.y;
        current_target_.reference.position.z = point.position.z;
        current_target_.reference.heading    = point.heading;
      }
    }

    return trajectory;

    //}

    /* ABORT_TRAJECTORY //{ */

  } else if (trajectoryType == ABORT_TRAJECTORY) {

    mrs_msgs::Reference point;

    trajectory.header.stamp    = ros::Time(0);
    trajectory.header.frame_id = "stable_origin";

    double desired_height = brick_stable_z + aborting_height_;

    point.position.x = cmd_odom_stable.pose.position.x;
    point.position.y = cmd_odom_stable.pose.position.y;
    point.position.z = desired_height;
    point.heading    = cmd_odom_stable_heading;

    trajectory.points.push_back(point);

    {
      std::scoped_lock lock(mutex_current_target_);

      current_target_.header.frame_id      = trajectory.header.frame_id;
      current_target_.reference.position.x = point.position.x;
      current_target_.reference.position.y = point.position.y;
      current_target_.reference.position.z = point.position.z;
      current_target_.reference.heading    = point.heading;
    }

    return trajectory;

    //}

    /* ALIGN_TO_PLACE_TRAJECTORY //{ */

  } else if (trajectoryType == ALIGN_TO_PLACE_TRAJECTORY) {

    double target_heading, target_distance, desired_heading, desired_height;

    trajectory.header.stamp    = cmd_odom_stable.header.stamp;
    trajectory.header.frame_id = "stable_origin";
    target_heading             = atan2(brick_stable_y - cmd_odom_stable.pose.position.y, brick_stable_x - cmd_odom_stable.pose.position.x);
    desired_height             = _aligning_placing_height_;
    target_distance = mrs_lib::geometry::dist(vec2_t(cmd_odom_stable.pose.position.x, cmd_odom_stable.pose.position.y), vec2_t(brick_stable_x, brick_stable_y));
    desired_heading = fabs(radians::dist(brick_stable_heading, cmd_odom_stable_heading)) < (M_PI / 2.0) ? brick_stable_heading : brick_stable_heading + M_PI;

    double step_size = _aligning_placing_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      point.position.x = cmd_odom_stable.pose.position.x;
      point.position.y = cmd_odom_stable.pose.position.y;
      point.position.z = desired_height;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    for (int i = 0; i < n_steps; i++) {

      mrs_msgs::Reference point;

      point.position.x = trajectory.points.back().position.x + cos(target_heading) * step_size;
      point.position.y = trajectory.points.back().position.y + sin(target_heading) * step_size;
      point.position.z = desired_height;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // the last point = the brick
    {
      mrs_msgs::Reference point;

      point.position.x = brick_stable_x;
      point.position.y = brick_stable_y;
      point.position.z = desired_height;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);

      {
        std::scoped_lock lock(mutex_current_target_);

        current_target_.header.frame_id      = trajectory.header.frame_id;
        current_target_.reference.position.x = point.position.x;
        current_target_.reference.position.y = point.position.y;
        current_target_.reference.position.z = point.position.z;
        current_target_.reference.heading    = point.heading;
      }
    }

    // return the trajectory
    return trajectory;

    //}

    /* PLACING_TRAJECTORY //{ */

  } else if (trajectoryType == PLACING_TRAJECTORY) {

    double desired_height, desired_vector, target_distance, direction, desired_heading;

    trajectory.header.stamp    = cmd_odom_stable.header.stamp;
    trajectory.header.frame_id = "stable_origin";
    desired_height             = brick_stable_z + _placing_height_;
    desired_vector             = desired_height - cmd_odom_stable.pose.position.z;
    target_distance            = fabs(desired_vector);
    direction                  = (desired_vector <= 0) ? -1 : 1;
    desired_heading = fabs(radians::dist(brick_stable_heading, cmd_odom_stable_heading)) < (M_PI / 2.0) ? brick_stable_heading : brick_stable_heading + M_PI;

    double step_size = _placing_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      point.position.x = brick_stable_x;
      point.position.y = brick_stable_y;
      point.position.z = cmd_odom_stable.pose.position.z;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::Reference point;

      for (int i = 0; i < n_steps; i++) {

        point.position.x = brick_stable_x;
        point.position.y = brick_stable_y;
        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = desired_heading;

        trajectory.points.push_back(point);
      }
    }

    // the last point
    {
      mrs_msgs::Reference point;

      point.position.x = brick_stable_x;
      point.position.y = brick_stable_y;
      point.position.z = desired_height;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);

      {
        std::scoped_lock lock(mutex_current_target_);

        current_target_.header.frame_id      = trajectory.header.frame_id;
        current_target_.reference.position.x = point.position.x;
        current_target_.reference.position.y = point.position.y;
        current_target_.reference.position.z = point.position.z;
        current_target_.reference.heading    = point.heading;
      }
    }

    return trajectory;

    //}

    /* GROUND_PLACING_TRAJECTORY //{ */

  } else if (trajectoryType == GROUND_PLACING_TRAJECTORY) {

    double target_distance, direction;

    trajectory.header.stamp    = cmd_odom_stable.header.stamp;
    trajectory.header.frame_id = "stable_origin";
    target_distance            = fabs(_ground_placing_height_);
    direction                  = -1;

    double step_size = _ground_placing_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      point.position.x = cmd_odom_stable.pose.position.x;
      point.position.y = cmd_odom_stable.pose.position.y;
      point.position.z = cmd_odom_stable.pose.position.z;
      point.heading    = cmd_odom_stable_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::Reference point;

      for (int i = 0; i < n_steps; i++) {

        point.position.x = cmd_odom_stable.pose.position.x;
        point.position.y = cmd_odom_stable.pose.position.y;
        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = cmd_odom_stable_heading;

        trajectory.points.push_back(point);
      }
    }

    // the last point
    {
      mrs_msgs::Reference point;

      point.position.x = cmd_odom_stable.pose.position.x;
      point.position.y = cmd_odom_stable.pose.position.y;
      point.position.z = cmd_odom_stable.pose.position.z + _ground_placing_height_;
      point.heading    = cmd_odom_stable_heading;

      trajectory.points.push_back(point);

      {
        std::scoped_lock lock(mutex_current_target_);

        current_target_.header.frame_id      = trajectory.header.frame_id;
        current_target_.reference.position.x = point.position.x;
        current_target_.reference.position.y = point.position.y;
        current_target_.reference.position.z = point.position.z;
        current_target_.reference.heading    = point.heading;
      }
    }

    return trajectory;

    //}
  }

  return trajectory;
}

//}

// | ------------------------- getters ------------------------ |

/* objectGripped() //{ */

bool BrickGrasping::objectGripped(void) {

  std::scoped_lock lock(mutex_object_gripped_);

  double time = (ros::Time::now() - last_object_gripped_time_).toSec();

  if (time < 1.0) {

    return object_gripped_;

  } else {
    return false;
  }
}

//}

/* doingServoing() //{ */

bool BrickGrasping::doingServoing(void) {

  auto odometry_diag = mrs_lib::get_mutexed(mutex_odometry_diag_, odometry_diag_);

  if (odometry_diag.estimator_type.name.compare("BRICK") == 0) {
    return true;
  } else {
    return false;
  }
}

//}

// | ------------------------- setters ------------------------ |

/* setGains() //{ */

void BrickGrasping::setGains(std::string desired_gains) {

  std::string sanitized_gains;

  if (desired_gains == _grasping_gains_name_) {
    if (high_wind_situation_here_) {
      ROS_WARN("[BrickGrasping]: high wind, sanitizing gains");
      sanitized_gains = _grasping_gains_name_ + _gains_wind_suffix_;
    } else {
      sanitized_gains = _grasping_gains_name_ + _gains_nowind_suffix_;
    }
  } else {
    sanitized_gains = desired_gains;
  }

  mrs_msgs::String srv;
  srv.request.value = sanitized_gains;

  ROS_INFO("[BrickGrasping]: setting gains to \"%s\"", sanitized_gains.c_str());

  bool res = gains_client_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for setGains() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[BrickGrasping]: service call for setGains() failed!");
  }
}

//}

/* setConstraints() //{ */

void BrickGrasping::setConstraints(std::string desired_constraints) {

  mrs_msgs::String srv;
  srv.request.value = desired_constraints;

  ROS_INFO("[BrickGrasping]: setting constraints to \"%s\"", desired_constraints.c_str());

  bool res = constraints_client_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for setConstraints() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[BrickGrasping]: service call for setConstraints() failed!");
  }
}

//}

/* setController() //{ */

void BrickGrasping::setController(std::string desired_controller) {

  mrs_msgs::String srv;
  srv.request.value = desired_controller;

  ROS_INFO("[BrickGrasping]: switching to controller: \"%s\"", desired_controller.c_str());

  bool res = service_client_switch_controller_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for setController() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[BrickGrasping]: service call for setController() failed!");
  }
}

//}

/* setVisionMode() //{ */

void BrickGrasping::setVisionMode(VisionMode_t desired_mode) {

  mbzirc_msgs::DetectionType srv;
  srv.request.type = vision_mode_values[desired_mode];

  ROS_INFO("[BrickGrasping]: switching vision mode to: \"%s\"", vision_mode_namses[desired_mode]);

  bool res = service_client_set_vision_mode_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for setVisionMode() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[BrickGrasping]: service call for setVisionMode() failed!");
  }
}

//}

/* setAvoidance() //{ */

void BrickGrasping::setAvoidance(const bool in) {

  std_srvs::SetBool srv;
  srv.request.data = in;

  ROS_INFO("[BrickGrasping]: setting collision avoidance to: \"%s\"", in ? "ON" : "OFF");

  bool res = service_client_set_avoidance_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for setAvoidance() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[BrickGrasping]: service call for setAvoidance() failed!");
  }
}

//}

/* setMapInactiveTime() //{ */

void BrickGrasping::setMapInactiveTime(const double time) {

  mrs_msgs::Float64Srv srv;
  srv.request.value = time;

  ROS_INFO("[BrickGrasping]: setting map inactive time to: \"%.1f\"", time);

  bool res = service_client_set_map_inactive_time_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for setMapInactiveTime() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[BrickGrasping]: service call for setMapInactiveTime() failed!");
  }
}

//}

/* setMinHeight() //{ */

void BrickGrasping::setMinHeight(double min_height) {

  mrs_msgs::Float64Srv srv;
  srv.request.value = min_height;

  ROS_INFO("[BrickGrasping]: setting minimum height to: \"%.2f\"", min_height);

  bool res = service_client_set_min_height_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for setMinHeight() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[BrickGrasping]: service call for setMinHeight() failed!");
  }
}

//}

/* getMinHeight() //{ */

double BrickGrasping::getMinHeight() {

  ROS_INFO("[BrickGrasping]: asking ControlManager for min height");

  mrs_msgs::GetFloat64 srv;

  bool res = service_client_get_min_height_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for getMinHeight() returned false");
    } else {

      ROS_INFO("[BrickGrasping]: obtained min height: %.2f", srv.response.value);
      return srv.response.value;
    }

  } else {

    ROS_ERROR("[BrickGrasping]: service call for getMinHeight() failed!");

    return 0;
  }

  return 0;
}

//}

/* setOdometry() //{ */

void BrickGrasping::setOdometry(std::string lateral_odometry, std::string height_odometry) {

  {
    mrs_msgs::String srv;
    srv.request.value = lateral_odometry;

    ROS_INFO("[BrickGrasping]: switching lateral odometry to \"%s\"", lateral_odometry.c_str());

    bool res = service_client_switch_lateral_odometry_.call(srv);

    if (res) {
      if (!srv.response.success) {
        ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for setLateralOdometry() returned false: %s", srv.response.message.c_str());
      }
    } else {
      ROS_ERROR("[BrickGrasping]: service call for setLateralOdometry() failed!");
    }
  }

  {
    mrs_msgs::String srv;
    srv.request.value = height_odometry;

    ROS_INFO("[BrickGrasping]: switching height odometry to \"%s\"", height_odometry.c_str());

    bool res = service_client_switch_height_odometry_.call(srv);

    if (res) {
      if (!srv.response.success) {
        ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for setHeightOdometry() returned false: %s", srv.response.message.c_str());
      }
    } else {
      ROS_ERROR("[BrickGrasping]: service call for setHeightOdometry() failed!");
    }
  }
}

//}

/* hover() //{ */

void BrickGrasping::hover(void) {

  std_srvs::Trigger srv;

  bool res = service_client_hover_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for hover() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[BrickGrasping]: service call for hover() failed!");
  }
}

//}

/* resetTracker() //{ */

void BrickGrasping::resetTracker(void) {

  ROS_INFO("[BrickGrasping]: resetting tracker");

  std_srvs::Trigger srv;

  bool res = service_client_reset_tracker_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for resetTracker() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[BrickGrasping]: service call for resetTracker() failed!");
  }
}

//}

/* resetMap() //{ */

void BrickGrasping::resetMap(void) {

  ROS_INFO("[BrickGrasping]: resetting map");

  std_srvs::Trigger srv;

  bool res = service_client_reset_map_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[BrickGrasping]: service call for resetMap() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[BrickGrasping]: service call for resetMap() failed!");
  }
}

//}

/* isHighWind() //{ */

bool BrickGrasping::isHighWind() {

  auto attitude_command = mrs_lib::get_mutexed(mutex_attitude_command_, attitude_command_);

  if (sqrt(pow(attitude_command.disturbance_wx_w, 2) + pow(attitude_command.disturbance_wx_w, 2)) > _world_disturbace_thr_) {

    return true;
  }

  return false;
}

//}

// | -------------------- support routines -------------------- |

/* alignedWithTarget() //{ */

double BrickGrasping::alignedWithTarget(const double position_thr, const double heading_thr, Alignment_t mode) {

  auto current_target       = mrs_lib::get_mutexed(mutex_current_target_, current_target_);
  auto odometry_main_stable = mrs_lib::get_mutexed(mutex_odometry_main_, odometry_main_stable_);
  auto focused_brick        = mrs_lib::get_mutexed(mutex_focused_brick_, focused_brick_);

  double cur_heading = 0;

  try {
    cur_heading = mrs_lib::AttitudeConverter(odometry_main_stable.pose.orientation).getHeading();
  }
  catch (...) {
  }

  // transform current target to stable origin

  auto ret = transformer_.transformSingle("stable_origin", current_target);

  mrs_msgs::ReferenceStamped current_target_stable;

  if (ret) {
    current_target_stable = ret.value();
  } else {
    ROS_ERROR("[BrickGrasping]: could not transform current target to stable_origin");
    return false;
  }

  double tar_x, tar_y, tar_z, tar_heading;
  double cur_x, cur_y, cur_z;

  if (current_state_ == ALIGN2_GRASP_STATE && _aligning2_grasping_alignment_criterion_ == ALIGNMENT_CRITERION_BRICK_DETECTION) {

    tar_x       = 0;
    tar_y       = 0;
    tar_heading = fabs(radians::diff(0, focused_brick.uav_odom.yaw)) < (M_PI / 2.0) ? 0 : M_PI;

    cur_x       = focused_brick.uav_odom.x;
    cur_y       = focused_brick.uav_odom.y;
    cur_heading = focused_brick.uav_odom.yaw;

    mode = MODE_2D;

  } else {

    tar_x       = current_target_stable.reference.position.x;
    tar_y       = current_target_stable.reference.position.y;
    tar_z       = current_target_stable.reference.position.z;
    tar_heading = current_target_stable.reference.heading;

    cur_x = odometry_main_stable.pose.position.x;
    cur_y = odometry_main_stable.pose.position.y;
    cur_z = odometry_main_stable.pose.position.z;
  }

  double position_error = 0;

  if (mode == MODE_3D) {
    position_error = sqrt(pow(cur_x - tar_x, 2) + pow(cur_y - tar_y, 2) + pow(cur_z - tar_z, 2));
  } else if (mode == MODE_2D) {
    position_error = sqrt(pow(cur_x - tar_x, 2) + pow(cur_y - tar_y, 2));
  }

  double heading_error = fabs(radians::diff(cur_heading, tar_heading));

  ROS_INFO_THROTTLE(1.0, "[BrickGrasping]: position error during alignment: %.3f m", position_error);

  if (position_error < position_thr && heading_error < heading_thr) {
    return true;
  } else {
    return false;
  }
}

//}

/* lastAlignmentCheck() //{ */

double BrickGrasping::lastAlignmentCheck(void) {

  auto current_target      = mrs_lib::get_mutexed(mutex_current_target_, current_target_);
  auto odometry_main_brick = mrs_lib::get_mutexed(mutex_odometry_main_, odometry_main_brick_);
  auto focused_brick       = mrs_lib::get_mutexed(mutex_focused_brick_, focused_brick_);

  double tar_x, tar_y, tar_heading;
  double cur_x, cur_y;

  double cur_heading = 0;

  try {
    cur_heading = mrs_lib::AttitudeConverter(odometry_main_brick.pose.orientation).getHeading();
  }
  catch (...) {
  }

  // transform current target to brick origin

  auto ret = transformer_.transformSingle("brick_origin", current_target);

  mrs_msgs::ReferenceStamped current_target_brick_frame;

  if (ret) {
    current_target_brick_frame = ret.value();
  } else {
    ROS_ERROR("[BrickGrasping]: could not transform current target to stable_origin");
    return false;
  }

  if (_aligning2_grasping_alignment_criterion_ == ALIGNMENT_CRITERION_BRICK_DETECTION) {

    tar_x       = 0;
    tar_y       = 0;
    tar_heading = fabs(radians::dist(0, focused_brick.uav_odom.yaw)) < (M_PI / 2.0) ? 0 : M_PI;

    cur_x       = focused_brick.uav_odom.x;
    cur_y       = focused_brick.uav_odom.y;
    cur_heading = focused_brick.uav_odom.yaw;

  } else {

    tar_x       = current_target_brick_frame.reference.position.x;
    tar_y       = current_target_brick_frame.reference.position.y;
    tar_heading = current_target_brick_frame.reference.heading;

    cur_x = odometry_main_brick.pose.position.x;
    cur_y = odometry_main_brick.pose.position.y;
  }

  double position_error_x = abs(cur_x - tar_x);
  double position_error_y = abs(cur_y - tar_y);
  double heading_error    = fabs(radians::diff(cur_heading, tar_heading));

  if (_aligning2_grasping_alignment_criterion_ == ALIGNMENT_CRITERION_BRICK_DETECTION) {
    ROS_INFO_THROTTLE(1.0, "[BrickGrasping]: alignment error (vision mode): x=%.3f m, y=%.3f m, heading=%.3f", position_error_x, position_error_y,
                      heading_error);
  } else {
    ROS_INFO_THROTTLE(1.0, "[BrickGrasping]: alignment error (control mode): x=%.3f m, y=%.3f m, heading=%.3f", position_error_x, position_error_y,
                      heading_error);
  }

  if (position_error_x < aligning2_current_x_crit_ && position_error_y < aligning2_current_y_crit_ && heading_error < 0.1) {
    return true;
  } else {
    return false;
  }
}

//}

/* distToObject() //{ */

double BrickGrasping::distToObject() {

  auto odometry_main_stable = mrs_lib::get_mutexed(mutex_odometry_main_, odometry_main_stable_);
  auto focused_brick        = mrs_lib::get_mutexed(mutex_focused_brick_, focused_brick_);

  double obj_dist = mrs_lib::geometry::dist(vec2_t(odometry_main_stable.pose.position.x, odometry_main_stable.pose.position.y),
                                            vec2_t(focused_brick.states[POS_X], focused_brick.states[POS_Y]));

  return obj_dist;
}

//}

/* timeout() //{ */

bool BrickGrasping::timeout(const double timeout) {

  return (ros::Time::now() - timeouter_).toSec() > timeout;
}

//}

/* freshFocusedBrick() //{ */

void BrickGrasping::freshFocusedBrick(const mbzirc_msgs::MbzircBrick brick_in) {

  focused_brick_ = brick_in;

  focused_brick_.uav_odom.z = -1;

  ROS_INFO("[BrickGrasping]: fresh brick being piped out, requesting odom reset");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* stateMachineTimer() //{ */

void BrickGrasping::stateMachineTimer([[maybe_unused]] const ros::TimerEvent &event) {

  ROS_INFO_ONCE("[BrickGrasping]: got data, working...");

  auto attitude_command = mrs_lib::get_mutexed(mutex_attitude_command_, attitude_command_);
  auto cmd_odom_stable  = mrs_lib::get_mutexed(mutex_odometry_main_, cmd_odom_stable_);

  double cmd_odom_stable_heading = 0;

  try {
    cmd_odom_stable_heading = mrs_lib::AttitudeConverter(cmd_odom_stable.pose.orientation).getHeading();
  }
  catch (...) {
  }

  auto focused_brick = mrs_lib::get_mutexed(mutex_focused_brick_, focused_brick_);

  switch (current_state_) {

      /* IDLE_STATE //{ */

    case IDLE_STATE: {

      // | ------------ check if we still have the object ----------- |
      if (gripper_) {

        if (!objectGripped()) {

          ROS_WARN("[BrickGrasping]: we lost something, turning the chripper off");

          magnetToggle(false);
        }
      }

      break;
    }

      //}

      /* ALIGN_STATE //{ */

    case ALIGN_STATE: {

      // | ------------------------- timeout ------------------------ |
      if (timeout(_aligning_timeout_)) {

        ROS_ERROR("[BrickGrasping]: timed out, ABORTING");

        grasping_result_.success   = false;
        grasping_result_.result_id = RESULT_TIMEOUT;
        grasping_result_.message   = "Aligning timed out.";

        changeState(ABORT_STATE);
      }

      // | ----------------- check brick visibility ----------------- |
      if (!brickVisible(grasping_object_type_)) {

        // we have lost the object from sight
        ROS_INFO("[BrickGrasping]: Object not visible");

        grasping_result_.success   = false;
        grasping_result_.result_id = RESULT_OBJECT_NOT_VISIBLE;
        grasping_result_.message   = "Object not visible.";

        changeState(ABORT_STATE);
      }

      // | ----------------- publish the trajectory ----------------- |
      mrs_msgs::TrajectoryReference trajectory = createTrajectory(ALIGN_TRAJECTORY);

      // publish the trajectory
      try {
        publisher_trajectory_.publish(trajectory);
      }
      catch (...) {
        ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_trajectory_.getTopic().c_str());
      }

      publishDebugTrajectory(trajectory);

      // | ------------------- check the alignment ------------------ |
      if (alignedWithTarget(_aligning_radius_, 0.1, MODE_3D)) {

        // we are aligned
        ROS_INFO_THROTTLE(1, "[BrickGrasping]: Aligned with the object, DESCENDING");

        changeState(DESCEND_STATE);
      }

      break;
    }

      //}

      /* DESCEND_STATE //{ */

    case DESCEND_STATE: {

      // | ------------------------- timeout ------------------------ |
      if (timeout(_descending_timeout_)) {

        ROS_ERROR("[BrickGrasping]: timed out, re-ALIGNING");

        changeState(REPEAT_STATE);
      }

      // | ----------------- check brick visibility ----------------- |
      if (!brickVisible(grasping_object_type_)) {

        // we have lost the object from sight
        ROS_WARN_THROTTLE(1, "[BrickGrasping]: Object not visible");

        grasping_result_.success   = false;
        grasping_result_.result_id = RESULT_OBJECT_NOT_VISIBLE;
        grasping_result_.message   = "Object lost";

        changeState(ABORT_STATE);
      }

      // | ------------------ create the trajectory ----------------- |
      mrs_msgs::TrajectoryReference trajectory = createTrajectory(DESCEND_TRAJECTORY);

      // publish the trajectory
      try {
        publisher_trajectory_.publish(trajectory);
      }
      catch (...) {
        ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_trajectory_.getTopic().c_str());
      }

      publishDebugTrajectory(trajectory);

      // | ------------- check if we reached the height ------------- |

      // check whether we are above the object
      if (alignedWithTarget(0.2, 0.1, MODE_3D)) {

        ROS_INFO("[BrickGrasping]: correct height reached, ALIGNING for grasping");

        ROS_INFO("[BrickGrasping]: DESCENDING took %.1f out of %.1f s", (ros::Time::now() - timeouter_).toSec(), _descending_timeout_);

        changeState(ALIGN2_GRASP_STATE);
      }

      break;
    }

      //}

      /* ALIGN2_GRASP_STATE //{ */

    case ALIGN2_GRASP_STATE: {

      // | ----------------- check the state timeout ---------------- |
      if (timeout(_aligning2_grasping_timeout_)) {

        grasping_result_.success   = false;
        grasping_result_.result_id = RESULT_TIMEOUT;
        grasping_result_.message   = "Grasping took too long.";

        ROS_WARN("[BrickGrasping]: Aligning for grasping took too long, ABORTING.");

        changeState(ABORT_STATE);
      }

      // | ----------------- check brick visibility ----------------- |
      if (!brickVisible(grasping_object_type_)) {

        // we have lost the object from sight
        ROS_WARN_THROTTLE(1, "[BrickGrasping]: Object not visible");

        grasping_result_.success   = false;
        grasping_result_.result_id = RESULT_OBJECT_NOT_VISIBLE;
        grasping_result_.message   = "Object lost";

        changeState(ABORT_STATE);
      }

      // | ------------- publish the desired trajectory ------------- |
      mrs_msgs::TrajectoryReference trajectory = createTrajectory(DESCEND_TRAJECTORY);

      // publish the trajectory
      try {
        publisher_trajectory_.publish(trajectory);
      }
      catch (...) {
        ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_trajectory_.getTopic().c_str());
      }

      publishDebugTrajectory(trajectory);

      aligning2_current_x_crit_ += (1.0 / _main_rate_) * _aligning2_grasping_criterion_increase_rate_x_;
      aligning2_current_y_crit_ += (1.0 / _main_rate_) * _aligning2_grasping_criterion_increase_rate_y_;

      ROS_INFO_THROTTLE(1.0, "[BrickGrasping]: alignment x crit: %.1f cm, y crit: %.1f cm", aligning2_current_x_crit_ * 100.0,
                        aligning2_current_y_crit_ * 100.0);

      // | ------------------ update the alignment ------------------ |
      if (lastAlignmentCheck()) {

        if (!aligning2_in_radius_) {

          aligning2_in_radius_      = true;
          aligning2_in_radius_time_ = ros::Time::now();
        }

      } else {

        if (aligning2_in_radius_) {

          aligning2_in_radius_      = false;
          aligning2_in_radius_time_ = ros::Time(0);
          ROS_WARN("[BrickGrasping]: alignment disturbed");
        }
      }

      // | ---------------- check the alignment time ---------------- |
      if (aligning2_in_radius_) {

        double alignemnt_held_for = (ros::Time::now() - aligning2_in_radius_time_).toSec();

        if (alignemnt_held_for >= _aligning2_in_alignment_duration_) {

          ROS_INFO("[BrickGrasping]: alignment finished, GRASPING");

          ROS_INFO("[BrickGrasping]: ALIGNMENT took %.1f out of %.1f s", (ros::Time::now() - timeouter_).toSec(), _aligning2_grasping_timeout_);

          changeState(GRASP_STATE);

        } else {

          ROS_INFO_THROTTLE(0.1, "[BrickGrasping]: alignment holds for %.2f/%.2f s", alignemnt_held_for, _aligning2_in_alignment_duration_);
        }
      }

      break;
    }

      //}

      /* GRASP_STATE //{ */

    case GRASP_STATE: {

      // | ----------------- check the state timeout ---------------- |
      if (timeout(_grasping_timeout_)) {

        ROS_WARN("[BrickGrasping]: Grasping took too long, REPEATING.");

        changeState(REPEAT_STATE);
      }

      // | ------------------ create the trajectory ----------------- |
      mrs_msgs::TrajectoryReference trajectory = createTrajectory(GRASPING_TRAJECTORY);

      try {
        publisher_trajectory_.publish(trajectory);
      }
      catch (...) {
        ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_trajectory_.getTopic().c_str());
      }

      publishDebugTrajectory(trajectory);

      // | --------------------- thrust limiter --------------------- |
      if (_grasping_thrust_limiter_enabled_) {

        double thrust_mass_estimate = mrs_lib::quadratic_thrust_model::thrustToForce(_motor_params_, attitude_command.thrust) / _g_;
        ROS_INFO_THROTTLE(1.0, "[BrickGrasping]: landing_uav_mass_: %f thrust_mass_estimate: %f", landing_uav_mass_, thrust_mass_estimate);

        if (((thrust_mass_estimate < _grasping_thrust_limiter_ratio_ * landing_uav_mass_) || attitude_command.thrust < 0.01)) {

          if (!grasping_thrust_under_threshold_) {

            grasping_thrust_first_time_      = ros::Time::now();
            grasping_thrust_under_threshold_ = true;
          }

          ROS_INFO_THROTTLE(0.1, "[BrickGrasping]: thrust is under cutoff factor for %.2f s", (ros::Time::now() - grasping_thrust_first_time_).toSec());

        } else {

          grasping_thrust_under_threshold_ = false;
        }

        if (grasping_thrust_under_threshold_ && ((ros::Time::now() - grasping_thrust_first_time_).toSec() > _grasping_thrust_timeout_)) {

          ROS_INFO("[BrickGrasping]: we touched the object, repeating");

          ROS_INFO("[BrickGrasping]: GRASPING took %.1f out of %.1f s", (ros::Time::now() - timeouter_).toSec(), _grasping_timeout_);

          resetTracker();

          changeState(REPEAT_STATE);

          return;
        }
      }

      // | --------------------- gripper checker -------------------- |
      if (objectGripped()) {

        ROS_INFO("[BrickGrasping]: Object gripped");

        ROS_INFO("[BrickGrasping]: GRASPING took %.1f out of %.1f s", (ros::Time::now() - timeouter_).toSec(), _grasping_timeout_);

        resetTracker();

        grasping_result_.success   = true;
        grasping_result_.result_id = RESULT_SUCCESS;
        grasping_result_.message   = "Object gripped.";

        changeState(ASCEND_STATE);
      }

      break;
    }

      //}

      /* REPEAT_STATE //{ */

    case REPEAT_STATE: {

      // | ------------------ create the trajectory ----------------- |
      mrs_msgs::TrajectoryReference trajectory = createTrajectory(REPEAT_TRAJECTORY);

      try {
        publisher_trajectory_.publish(trajectory);
      }
      catch (...) {
        ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_trajectory_.getTopic().c_str());
      }

      publishDebugTrajectory(trajectory);

      // | -------------------- check the height -------------------- |
      if (alignedWithTarget(0.3, 0.1, MODE_3D)) {

        changeState(ALIGN_STATE);
      }

      break;
    }

      //}

      /* ASCEND_STATE //{ */

    case ASCEND_STATE: {

      // | ------------------ create the trajectory ----------------- |
      mrs_msgs::TrajectoryReference trajectory = createTrajectory(ASCEND_TRAJECTORY);

      try {
        publisher_trajectory_.publish(trajectory);
      }
      catch (...) {
        ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_trajectory_.getTopic().c_str());
      }

      publishDebugTrajectory(trajectory);

      // | ------------ check if we still have the object ----------- |
      if (!objectGripped()) {

        ROS_WARN("[BrickGrasping]: We lost something, repeating.");

        changeState(REPEAT_STATE);
      }

      // | ------------------- ckeck the climbing ------------------- |
      if (alignedWithTarget(0.5, 0.1, MODE_3D)) {

        std::string brick_color;

        switch (focused_brick.type) {
          case BRICK_RED: {
            brick_color = "RED";
            break;
          }
          case BRICK_GREEN: {
            brick_color = "GREEN";
            break;
          }
          case BRICK_BLUE: {
            brick_color = "BLUE";
            break;
          }
        }

        ROS_INFO("[BrickGrasping]: we succeded with grasping of the %s brick.", brick_color.c_str());

        carrying_brick_type_ = Object_t(focused_brick.type);

        changeState(IDLE_STATE);

        if (grasping_server_->isActive()) {
          grasping_server_->setSucceeded(grasping_result_);
        }
      }

      break;
    }

      //}

      /* ASCEND_AFTER_PLACE_STATE //{ */

    case ASCEND_AFTER_PLACE_STATE: {

      // | -------------------- create trajectory ------------------- |
      mrs_msgs::TrajectoryReference trajectory = createTrajectory(ASCEND_AFTER_PLACE_TRAJECTORY);

      try {
        publisher_trajectory_.publish(trajectory);
      }
      catch (...) {
        ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_trajectory_.getTopic().c_str());
      }

      publishDebugTrajectory(trajectory);

      // | -------- check whether we are in the target place -------- |
      if (alignedWithTarget(0.5, 0.1, MODE_3D)) {

        changeState(IDLE_STATE);

        if (grasping_server_->isActive()) {
          grasping_server_->setSucceeded(grasping_result_);
        }
      }

      break;
    }

      //}

      /* ABORT_STATE //{ */

    case ABORT_STATE: {

      // | ------------------ create the trajectory ----------------- |
      mrs_msgs::TrajectoryReference trajectory = createTrajectory(ABORT_TRAJECTORY);

      try {
        publisher_trajectory_.publish(trajectory);
      }
      catch (...) {
        ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_trajectory_.getTopic().c_str());
      }

      // | --------------- check if we reached to top --------------- |
      if (alignedWithTarget(0.5, 0.1, MODE_3D)) {

        if (grasping_server_->isActive()) {
          grasping_server_->setAborted(grasping_result_);
        }

        changeState(IDLE_STATE);
      }

      break;
    }

      //}

      /* PREEMPTED_STATE //{ */

    case PREEMPTED_STATE: {

      // | ------------------ create the trajectory ----------------- |
      mrs_msgs::TrajectoryReference trajectory = createTrajectory(ABORT_TRAJECTORY);

      try {
        publisher_trajectory_.publish(trajectory);
      }
      catch (...) {
        ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_trajectory_.getTopic().c_str());
      }

      // check whether we are still climbing up
      if (alignedWithTarget(0.3, 0.1, MODE_3D)) {

        if (grasping_server_->isActive()) {
          grasping_server_->setPreempted();
        }

        changeState(IDLE_STATE);
      }

      break;
    }

      //}

      /* ALIGN_PLACE_STATE //{ */

    case ALIGN_PLACE_STATE: {

      // | ------------------------- timeout ------------------------ |
      if (timeout(_aligning_placing_timeout_)) {

        ROS_ERROR("[BrickGrasping]: timed out, ABORTING");

        grasping_result_.success   = false;
        grasping_result_.result_id = RESULT_TIMEOUT;
        grasping_result_.message   = "Aligning timed out.";

        changeState(ABORT_STATE);
      }

      // | ------------ check if we still have the object ----------- |
      if (!objectGripped()) {

        ROS_WARN("[BrickGrasping]: We lost something, mission success, I guess.");

        grasping_result_.success   = true;
        grasping_result_.result_id = RESULT_SUCCESS;
        grasping_result_.message   = "Object fell off.";

        changeState(ASCEND_AFTER_PLACE_STATE);
      }

      // | ----------------- check wall visibility ----------------- |
      if (!brickVisible(grasping_object_type_)) {

        // we have lost the object from sight
        ROS_WARN_THROTTLE(1, "[BrickGrasping]: Object not visible");

        grasping_result_.success   = false;
        grasping_result_.result_id = RESULT_OBJECT_NOT_VISIBLE;
        grasping_result_.message   = "Object lost";

        changeState(ABORT_STATE);
      }

      // | ------------------ create the trajectory ----------------- |
      mrs_msgs::TrajectoryReference trajectory = createTrajectory(ALIGN_TO_PLACE_TRAJECTORY);

      try {
        publisher_trajectory_.publish(trajectory);
      }
      catch (...) {
        ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_trajectory_.getTopic().c_str());
      }

      // check whether we are above the dropping zone
      if (alignedWithTarget(_aligning_placing_radius_, 0.05, MODE_2D)) {

        changeState(WALL_PLACING_STATE);
      }

      break;
    }

      //}

      /* WALL_PLACING_STATE //{ */

    case WALL_PLACING_STATE: {

      // | ------------------------- timeout ------------------------ |
      if (timeout(_placing_timeout_)) {

        ROS_ERROR("[BrickGrasping]: timed out, ABORTING");

        grasping_result_.success   = false;
        grasping_result_.result_id = RESULT_OBJECT_NOT_VISIBLE;
        grasping_result_.message   = "Placing timed out.";

        changeState(ABORT_STATE);
      }

      // | ----------------- check wall visibility ----------------- |
      if (!brickVisible(grasping_object_type_)) {

        // we have lost the object from sight
        ROS_WARN_THROTTLE(1, "[BrickGrasping]: Object not visible");

        grasping_result_.success   = false;
        grasping_result_.result_id = RESULT_OBJECT_NOT_VISIBLE;
        grasping_result_.message   = "Object lost";

        changeState(ABORT_STATE);
      }

      // | ------------ check if we still have the object ----------- |
      if (!objectGripped()) {

        ROS_WARN("[BrickGrasping]: We lost something, mission success, I guess.");

        grasping_result_.success   = true;
        grasping_result_.result_id = RESULT_SUCCESS;
        grasping_result_.message   = "Object fell off";

        changeState(ASCEND_AFTER_PLACE_STATE);
      }

      // | --------------------- touch detection -------------------- |
      double thrust_mass_estimate = mrs_lib::quadratic_thrust_model::thrustToForce(_motor_params_, attitude_command.thrust) / _g_;
      ROS_INFO_THROTTLE(1.0, "[BrickGrasping]: landing_uav_mass: %f thrust_mass_estimate: %f", landing_uav_mass_, thrust_mass_estimate);

      // condition for automatic motor turn off
      if (((thrust_mass_estimate < _placing_thrust_limiter_ratio_ * landing_uav_mass_) || attitude_command.thrust < 0.01)) {

        if (!placing_thrust_under_threshold_) {

          placing_thrust_first_time_      = ros::Time::now();
          placing_thrust_under_threshold_ = true;
        }

        ROS_INFO_THROTTLE(0.1, "[BrickGrasping]: thrust is under cutoff factor for %.2f s", (ros::Time::now() - placing_thrust_first_time_).toSec());

      } else {

        placing_thrust_under_threshold_ = false;
      }

      if (placing_thrust_under_threshold_ && ((ros::Time::now() - placing_thrust_first_time_).toSec() > _placing_thrust_timeout_)) {

        ROS_INFO("[BrickGrasping]: we touched the wall, ungripping");

        magnetToggle(false);

        resetTracker();

        grasping_result_.success   = true;
        grasping_result_.result_id = RESULT_SUCCESS;
        grasping_result_.message   = "Placing successful.";

        changeState(WAITING_AFTER_PLACING);
      }

      // | --------------------- alignment check -------------------- |
      if (alignedWithTarget(0.1, 0.1, MODE_3D)) {

        ROS_INFO("[BrickGrasping]: reached the target altitude, ungripping");

        magnetToggle(false);

        resetTracker();

        grasping_result_.success   = true;
        grasping_result_.result_id = RESULT_SUCCESS;
        grasping_result_.message   = "Placing successful.";

        changeState(WAITING_AFTER_PLACING);
      }

      // | ------------------ create the trajectory ----------------- |
      mrs_msgs::TrajectoryReference trajectory = createTrajectory(PLACING_TRAJECTORY);

      try {
        publisher_trajectory_.publish(trajectory);
      }
      catch (...) {
        ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_trajectory_.getTopic().c_str());
      }

      break;
    }

      //}

      /* GROUND_PLACING_STATE //{ */

    case GROUND_PLACING_STATE: {

      // | ------------------ create the trajectory ----------------- |
      mrs_msgs::TrajectoryReference trajectory = createTrajectory(GROUND_PLACING_TRAJECTORY);

      try {
        publisher_trajectory_.publish(trajectory);
      }
      catch (...) {
        ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_trajectory_.getTopic().c_str());
      }

      // | -------------------- check the gripper ------------------- |
      if (!objectGripped()) {

        ROS_INFO("[BrickGrasping]: we lost something, ASCENDING");

        magnetToggle(false);

        grasping_result_.success   = false;
        grasping_result_.result_id = RESULT_FAILED;
        grasping_result_.message   = "The object fell.";

        changeState(ASCEND_AFTER_PLACE_STATE);
      }

      // | ------------------ the thrust condition ------------------ |
      double thrust_mass_estimate = mrs_lib::quadratic_thrust_model::thrustToForce(_motor_params_, attitude_command.thrust) / _g_;
      ROS_INFO_THROTTLE(1.0, "[BrickGrasping]: landing_uav_mass: %f thrust_mass_estimate: %f", landing_uav_mass_, thrust_mass_estimate);

      // condition for automatic motor turn off
      if (((thrust_mass_estimate < _ground_placing_thrust_limiter_ratio_ * landing_uav_mass_) || attitude_command.thrust < 0.01)) {

        if (!ground_placing_thrust_under_threshold_) {

          ground_placing_thrust_first_time_      = ros::Time::now();
          ground_placing_thrust_under_threshold_ = true;
        }

        ROS_INFO_THROTTLE(0.1, "[BrickGrasping]: thrust is under cutoff factor for %.2f s", (ros::Time::now() - ground_placing_thrust_first_time_).toSec());

      } else {

        ground_placing_thrust_under_threshold_ = false;
      }

      if (ground_placing_thrust_under_threshold_ && ((ros::Time::now() - ground_placing_thrust_first_time_).toSec() > _ground_placing_thrust_timeout_)) {

        ROS_INFO("[BrickGrasping]: we touched the ground, dropping");

        magnetToggle(false);

        resetTracker();

        grasping_result_.success   = true;
        grasping_result_.result_id = RESULT_SUCCESS;
        grasping_result_.message   = "Placing successful.";

        changeState(WAITING_AFTER_PLACING);
      }

      break;
    }

      //}

      /* WAITING_AFTER_PLACING //{ */

    case WAITING_AFTER_PLACING: {

      // check whether we have not exceeded timeout
      if ((ros::Time::now() - placing_time_).toSec() > _after_placing_delay_) {

        changeState(ASCEND_AFTER_PLACE_STATE);
      }

      break;
    }

      //}
  }
}

//}

/* diagnosticsTimer() //{ */

void BrickGrasping::diagnosticsTimer([[maybe_unused]] const ros::TimerEvent &event) {

  publishDiagnostics();
}

//}

/* currentTargetTimer() //{ */

void BrickGrasping::currentTargetTimer([[maybe_unused]] const ros::TimerEvent &event) {

  auto focused_brick = mrs_lib::get_mutexed(mutex_focused_brick_, focused_brick_);

  mbzirc_msgs::ObjectWithType object;
  mrs_msgs::Float64Stamped    object_uav_heading;

  object.stamp = ros::Time::now();
  object.x     = focused_brick.states[POS_X];
  object.y     = focused_brick.states[POS_Y];
  object.z     = focused_brick.states[POS_Z];
  object.type  = grasping_object_type_;

  try {
    publisher_current_target_.publish(object);
  }
  catch (...) {
    ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_current_target_debug_.getTopic().c_str());
  }

  geometry_msgs::PoseStamped pose;

  pose.header.stamp    = ros::Time::now();
  pose.header.frame_id = focused_brick.header.frame_id;

  pose.pose.position.x = object.x;
  pose.pose.position.y = object.y;
  pose.pose.position.z = object.z;

  try {
    publisher_current_target_debug_.publish(pose);
  }
  catch (...) {
    ROS_ERROR("[BrickGrasping]: Exception caught during publishing topic %s.", publisher_current_target_debug_.getTopic().c_str());
  }

  geometry_msgs::PoseStamped object_uav_odom;

  object_uav_odom.header.frame_id = _uav_name_ + "/brick_origin";
  object_uav_odom.header.stamp    = focused_brick.uav_odom.header.stamp;
  object_uav_odom.pose.position.x = focused_brick.uav_odom.x;
  object_uav_odom.pose.position.y = focused_brick.uav_odom.y;
  object_uav_odom.pose.position.z = focused_brick.uav_odom.z;

  object_uav_odom.pose.orientation.x = 0;
  object_uav_odom.pose.orientation.y = 0;
  object_uav_odom.pose.orientation.z = sin((focused_brick.uav_odom.yaw) / 2.0);
  object_uav_odom.pose.orientation.w = cos((focused_brick.uav_odom.yaw) / 2.0);

  object_uav_heading.header = object_uav_odom.header;
  try {
    publisher_current_target_uav_odom_.publish(object_uav_odom);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", publisher_current_target_uav_odom_.getTopic().c_str());
  }

  // publish heading separately for debug (plotjuggler)
  object_uav_heading.value = focused_brick.uav_odom.yaw;
  try {
    publisher_current_target_uav_heading_.publish(object_uav_heading);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", publisher_current_target_uav_heading_.getTopic().c_str());
  }
}

//}

/* gripperTimer() //{ */

void BrickGrasping::gripperTimer([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  if (gripper_) {

    std_srvs::Trigger msg;

    bool res = service_client_gripper_on_.call(msg);

    if (!res) {

      ROS_ERROR_THROTTLE(1.0, "[BrickGrasping]: service call for griped failed");
    }
  }
}

//}

}  // namespace brick_grasping

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(brick_grasping::BrickGrasping, nodelet::Nodelet)
