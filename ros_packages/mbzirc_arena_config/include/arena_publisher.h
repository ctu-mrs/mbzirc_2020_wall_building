#pragma once
#ifndef MBZIRCARENACONFIG_ARENAPUBLISHER_H
#define MBZIRCARENACONFIG_ARENAPUBLISHER_H

/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>

/* for writing and reading from streams */
#include <fstream>
#include <iostream>

/* for challenge mode */
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>

/* tf2 transform handling */
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* custom msgs of MRS group */
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/SetInt.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/safety_zone/polygon.h>
#include <mrs_lib/mutex.h>

#include <mbzirc_msgs/MbzircArenaParameters.h>
#include <mbzirc_msgs/ArenaZoneSrv.h>

/* markers for rviz visualization */
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

/* for operations with matrices */
#include <Eigen/Dense>

//}

#define TAKEOFF_ZONE_NAME "takeoff_zone"

#define DROPOFF_ZONE_NAME "dropoff_zone"

#define UAV_BRICK_ZONE_NAME "uav_brick_zone"
#define UAV_WALL_ZONE_NAME "uav_wall_zone"
#define UGV_BRICK_ZONE_NAME "ugv_brick_zone"
#define UGV_WALL_ZONE_NAME "ugv_wall_zone"
#define UGV_WALL_ZONE_NAME "ugv_wall_zone"

#define GROUND_FLOOR_ZONE_NAME "ground_floor_zone"
#define GROUND_CEILING_ZONE_NAME "ground_ceiling_zone"
#define FIRST_FLOOR_ZONE_NAME "first_floor_zone"
#define FIRST_CEILING_ZONE_NAME "first_ceiling_zone"
#define SECOND_FLOOR_ZONE_NAME "second_floor_zone"
#define SECOND_CEILING_ZONE_NAME "second_ceiling_zone"

namespace mbzirc_arena_config
{

/* typedef //{ */

typedef enum
{

  BALL_ARENA,
  WALL_ARENA,
  FIRE_ARENA,
  DEFAULT_ARENA

} ArenaType_t;

typedef struct
{
  double r;
  double g;
  double b;

} MarkerColor_t;


//}

/* class ArenaPublisher //{ */
class ArenaPublisher : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  const MarkerColor_t _takeoff_zone_color_   = {0, 1, 0};
  const MarkerColor_t _dropoff_zone_color_   = {0, 0, 1};
  const MarkerColor_t _uav_brick_zone_color_ = {0, 0, 1};
  const MarkerColor_t _uav_wall_zone_color_  = {0, 1, 1};
  const MarkerColor_t _ugv_brick_zone_color_ = {0.5, 0.5, 0};
  const MarkerColor_t _ugv_wall_zone_color_  = {1, 1, 0};
  const MarkerColor_t _default_color_        = {0, 0, 0};

private:
  /* flags */
  bool is_initialized_ = false;

  /* ros parameters */
  std::string _uav_name_;

  // transform publisher

  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  geometry_msgs::TransformStamped tf_arena_;
  geometry_msgs::TransformStamped tf_takeoff_;
  geometry_msgs::TransformStamped tf_dropoff_;
  geometry_msgs::TransformStamped tf_gf_f_;
  geometry_msgs::TransformStamped tf_gf_c_;
  geometry_msgs::TransformStamped tf_1f_f_;
  geometry_msgs::TransformStamped tf_1f_c_;
  geometry_msgs::TransformStamped tf_2f_f_;
  geometry_msgs::TransformStamped tf_2f_c_;

  ros::Timer timer_static_tf_;
  int        _rate_timer_static_tf_;
  void       callbackTimerStaticTf(const ros::TimerEvent& event);

  // | ---------------------- msg callbacks --------------------- |


  // | --------------------- timer callbacks -------------------- |

  ros::Publisher pub_markers_;
  ros::Publisher pub_arena_parameters_;

  ArenaType_t arena_type_;
  std::mutex  mutex_arena_type_;

  mbzirc_msgs::MbzircArenaParameters arena_parameters_;
  std::mutex                         mutex_arena_parameters_;

  visualization_msgs::MarkerArray marker_array_;
  std::mutex                      mutex_marker_array_;

  // arena corners
  double                      arena_ang_diff_;
  Eigen::Matrix<double, 2, 1> arena_center_;
  Eigen::MatrixXd             arena_corners_;

  // safety area
  Eigen::MatrixXd                   safety_area_;
  std::shared_ptr<mrs_lib::Polygon> safety_polygon_;
  std::mutex                        mutex_safety_polygon_;

  // takeoff area
  Eigen::Matrix<double, 2, 1>       takeoff_center_;
  Eigen::MatrixXd                   takeoff_zone_;
  std::shared_ptr<mrs_lib::Polygon> takeoff_polygon_;
  std::mutex                        mutex_takeoff_polygon_;

  /* ball arena //{ */

  // dropoff area
  bool                              got_dropoff_area_ = false;
  Eigen::Matrix<double, 2, 1>       dropoff_center_;
  Eigen::MatrixXd                   dropoff_zone_;
  std::shared_ptr<mrs_lib::Polygon> dropoff_polygon_;
  std::mutex                        mutex_dropoff_polygon_;
  ros::Publisher                    pub_dropoff_pose_;

  //}

  /* wall arena //{ */

  // uav brick area
  bool                              got_uav_brick_area_ = false;
  Eigen::Matrix<double, 2, 1>       uav_brick_center_;
  Eigen::MatrixXd                   uav_brick_zone_;
  std::shared_ptr<mrs_lib::Polygon> uav_brick_polygon_;
  std::mutex                        mutex_uav_brick_polygon_;

  // uav wall area
  bool                              got_uav_wall_area_ = false;
  Eigen::Matrix<double, 2, 1>       uav_wall_center_;
  Eigen::MatrixXd                   uav_wall_zone_;
  std::shared_ptr<mrs_lib::Polygon> uav_wall_polygon_;
  std::mutex                        mutex_uav_wall_polygon_;

  // ugv brick area
  bool                              got_ugv_brick_area_ = false;
  Eigen::Matrix<double, 2, 1>       ugv_brick_center_;
  Eigen::MatrixXd                   ugv_brick_zone_;
  std::shared_ptr<mrs_lib::Polygon> ugv_brick_polygon_;
  std::mutex                        mutex_ugv_brick_polygon_;

  // ugv wall area
  bool                              got_ugv_wall_area_ = false;
  Eigen::Matrix<double, 2, 1>       ugv_wall_center_;
  Eigen::MatrixXd                   ugv_wall_zone_;
  std::shared_ptr<mrs_lib::Polygon> ugv_wall_polygon_;
  std::mutex                        mutex_ugv_wall_polygon_;

  //}

  /* fire arena //{ */

  // ground floor
  double                            ground_floor_floor_;
  double                            ground_floor_ceiling_;
  Eigen::Matrix<double, 2, 1>       ground_floor_center_;
  Eigen::MatrixXd                   ground_floor_outside_zone_;
  std::shared_ptr<mrs_lib::Polygon> ground_floor_outside_polygon_;
  std::mutex                        mutex_ground_floor_outside_polygon_;
  Eigen::MatrixXd                   ground_ceiling_outside_zone_;
  std::shared_ptr<mrs_lib::Polygon> ground_ceiling_outside_polygon_;
  std::mutex                        mutex_ground_ceiling_outside_polygon_;

  // first floor
  double                            first_floor_floor_;
  double                            first_floor_ceiling_;
  Eigen::Matrix<double, 2, 1>       first_floor_center_;
  Eigen::MatrixXd                   first_floor_outside_zone_;
  std::shared_ptr<mrs_lib::Polygon> first_floor_outside_polygon_;
  std::mutex                        mutex_first_floor_outside_polygon_;
  Eigen::MatrixXd                   first_ceiling_outside_zone_;
  std::shared_ptr<mrs_lib::Polygon> first_ceiling_outside_polygon_;
  std::mutex                        mutex_first_ceiling_outside_polygon_;

  // second floor
  double                            second_floor_floor_;
  double                            second_floor_ceiling_;
  Eigen::Matrix<double, 2, 1>       second_floor_center_;
  Eigen::MatrixXd                   second_floor_outside_zone_;
  std::shared_ptr<mrs_lib::Polygon> second_floor_outside_polygon_;
  std::mutex                        mutex_second_floor_outside_polygon_;
  Eigen::MatrixXd                   second_ceiling_outside_zone_;
  std::shared_ptr<mrs_lib::Polygon> second_ceiling_outside_polygon_;
  std::mutex                        mutex_second_ceiling_outside_polygon_;

  // fires
  ros::Publisher  pub_outdoor_fires_;
  Eigen::MatrixXd outdoor_fires_;

  // windows
  ros::Publisher  pub_windows_;
  Eigen::MatrixXd windows_;

  //}

  int connecting_line_id_ = 0;

  // | ---------------- service server callbacks ---------------- |

  // Start challenge
  bool               callbackStartChallenge(mrs_msgs::SetInt::Request& req, mrs_msgs::SetInt::Response& res);
  ros::ServiceServer srv_server_start_challenge_;
  int                challenge_mode_ = 0;
  std::mutex         mutex_challenge_mode_;
  int                _default_challenge_mode_ = 0;  // what mode should be default?
  bool               received_challenge_mode_ = false;
  ros::ServiceClient srv_client_start_challenge_;
  ros::Publisher     pub_challenge_mode_;

  ros::Timer timer_call_start_;
  double     _period_timer_call_start_;
  void       callbackTimerCallStart(const ros::TimerEvent& event);

  bool               callbackChangeFloor(mrs_msgs::SetInt::Request& req, mrs_msgs::SetInt::Response& res);
  ros::ServiceServer srv_server_change_floor_;
  int                target_floor_ = 0;
  std::mutex         mutex_target_floor_;
  ros::Publisher     pub_target_floor_;

  bool               callbackSetIndoor(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  ros::ServiceServer srv_server_set_indoor_;
  bool               target_indoor_ = false;
  std::mutex         mutex_target_indoor_;
  ros::Publisher     pub_target_indoor_;

  bool               callbackIsInSafetyArea(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
  ros::ServiceServer srv_server_is_in_safety_area_;

  bool               callbackIsInTakeoffArea(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
  ros::ServiceServer srv_server_is_in_takeoff_area_;

  bool               callbackIsInDropoffArea(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
  ros::ServiceServer srv_server_is_in_dropoff_area_;

  bool               callbackAddArenaZone(mbzirc_msgs::ArenaZoneSrv::Request& req, mbzirc_msgs::ArenaZoneSrv::Response& res);
  ros::ServiceServer srv_server_add_arena_zone_;

  // | --------------------- service clients -------------------- |

  // | -------------------- support functions ------------------- |

  void addMarker(const mbzirc_msgs::ArenaZone& arena_zone_in);

  void addConnectingMarker(const mbzirc_msgs::ArenaZone& arena_zone1_in, const mbzirc_msgs::ArenaZone& arena_zone2_in);

  std::vector<mrs_msgs::Reference> matrixToPoints(const Eigen::MatrixXd& matrix);

  void offsetPoints(std::vector<mrs_msgs::Reference>& points, const Eigen::MatrixXd& offset);

  double distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose);
};
//}

}  // namespace mbzirc_arena_config
#endif
