/* include //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <Eigen/Eigen>
#include <mutex>
#include <map>
#include <cmath>

#include <mrs_lib/lkf.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <mrs_msgs/Float64Srv.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <mbzirc_msgs/ObjectWithTypeArray.h>
#include <mbzirc_msgs/BrickGraspingDiagnostics.h>
#include <mbzirc_msgs/MbzircBrick.h>
#include <mbzirc_msgs/BrickWithId.h>
#include <mbzirc_msgs/BrickWithIdArray.h>
#include <mbzirc_msgs/BanArea.h>
#include <mbzirc_msgs/ObjectUavOdometry.h>
#include <mbzirc_msgs/EstimatedBrick.h>
#include <mbzirc_msgs/BrickMap.h>

#include <mrs_msgs/ValidateReferenceList.h>

#include <brick_estimation/brick_estimation_drsConfig.h>

#include <dynamic_reconfigure/server.h>

#include "batch_visualizer.h"

#include "tf/LinearMath/Transform.h"

//}

/* using //{ */

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

/* LKF helpers //{ */

// Define the LKF we will be using
const int _n_states_       = 4;
const int _n_inputs_       = 0;
const int _n_measurements_ = 4;

using lkf_t = mrs_lib::LKF<_n_states_, _n_inputs_, _n_measurements_>;

using A_t        = lkf_t::A_t;
using B_t        = lkf_t::B_t;
using H_t        = lkf_t::H_t;
using Q_t        = lkf_t::Q_t;
using x_t        = lkf_t::x_t;
using P_t        = lkf_t::P_t;
using R_t        = lkf_t::R_t;
using statecov_t = lkf_t::statecov_t;

//}

namespace brick_estimation
{

using namespace Eigen;

/* structures //{ */

[[maybe_unused]] typedef enum {

  // from brick detector
  OBJECT_RED            = 1,
  OBJECT_GREEN          = 2,
  OBJECT_BLUE           = 3,
  OBJECT_ORANGE         = 4,
  OBJECT_WALL           = 5,
  OBJECT_GROUND_PATTERN = 66,

  // mine
  OBJECT_ANY = 666,

} Object_t;

// structure for metadata about an object
struct ObjectHandler_t
{

  std_msgs::Header header;

  ros::Time last_updated;
  ros::Time firs_seen;
  bool      updated_this_iter;
  bool      active;
  double    hue;
  int       type;
  double    x, y, z;
  double    yaw;
  int       id;
  long      flipper_bias_none;
  long      flipper_bias_positive;
  long      flipper_bias_negative;
  double    len;
  int       n_corrections;

  // uav odometry in the frame of the object
  mbzirc_msgs::ObjectUavOdometry uav_odom;

  /* std::unique_ptr<mrs_lib::lkf_t> kalman; */
  statecov_t statecov;
};

// structure for holding banned areas
struct BannedArea_t
{

  ros::Time stamp;
  double    x, y;
};

//}

// --------------------------------------------------------------
// |                          the class                         |
// --------------------------------------------------------------

/* class BrickEstimation //{ */

class BrickEstimation : public nodelet::Nodelet {

public:
  virtual void onInit();
  bool         is_initialized_ = false;

  bool publishing_map_ = false;

  bool   _wall_height_filtering_enabled = false;
  double _wall_min_height_;
  double _wall_max_height_;
  double _wall_projection_distance_limit_;
  double _wall_max_len_;

  int  _rate_;
  int  _map_rate_;
  int  _rviz_rate_;
  void iterate(void);
  void publish(void);

private:
  ros::NodeHandle nh_;

  ros::Publisher pub_red_closest_;
  ros::Publisher pub_green_closest_;
  ros::Publisher pub_blue_closest_;
  ros::Publisher pub_wall_closest_;
  ros::Publisher pub_any_closest_;
  ros::Publisher pub_map_;

  ros::Publisher pub_red_closest_debugging_;
  int            last_closest_red_ = -1;

  ros::Publisher pub_green_closest_debugging_;
  int            last_closest_green_ = -1;

  ros::Publisher pub_blue_closest_debugging_;
  int            last_closest_blue_ = -1;

  ros::Publisher pub_any_closest_debugging_;
  int            last_closest_any_ = -1;

  ros::Publisher pub_wall_closest_debugging_;
  int            last_closest_wall_ = -1;

  ros::ServiceServer service_server_reset_map_;
  ros::ServiceServer service_server_ban_area_;
  ros::ServiceServer service_server_set_map_timeout_;
  ros::ServiceServer service_server_stop_map_;
  ros::ServiceServer service_server_set_inactive_time_;

  ros::ServiceClient service_client_validate_reference_;

  mrs_lib::Transformer transformer_;

  bool callbackRestart(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackBanArea(mbzirc_msgs::BanArea::Request &req, mbzirc_msgs::BanArea::Response &res);
  bool callbackSetMapTimeout(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res);
  bool callbackStopMap(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackSetInactiveTime(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res);

  dynamic_reconfigure::Server<brick_estimation::brick_estimation_drsConfig>               srv;
  dynamic_reconfigure::Server<brick_estimation::brick_estimation_drsConfig>::CallbackType f;
  void callbackDynamicReconfigure(brick_estimation::brick_estimation_drsConfig &config, uint32_t level);

  // Pixhawk odometry subscriber and callback
  ros::Subscriber            sub_odometry_;
  geometry_msgs::PoseStamped odometry_stable_;
  std::mutex                 mutex_odometry_;
  void                       callbackOdometry(const nav_msgs::OdometryConstPtr &msg);

  void               callbackBrickHeight(const sensor_msgs::RangeConstPtr &msg);
  ros::Subscriber    sub_brick_height_;
  std::mutex         mutex_brick_height_;
  sensor_msgs::Range brick_height_;
  bool               got_brick_height_ = false;

  // Object detector topic
  ros::Subscriber sub_bricks_;
  bool            first_brick_thrown_away_ = false;
  void            callbackBricks(const mbzirc_msgs::ObjectWithTypeArrayConstPtr &msg);

  double dt;
  bool   got_odometry_ = false;
  bool   got_bricks_   = false;

  bool _wall_angle_discrimination_ = false;

  // lkf matrices
  A_t A;
  R_t R;
  Q_t Q;
  H_t H;
  B_t B;

  std::unique_ptr<lkf_t> lkf_;

  double     object_inactive_time_;
  std::mutex mutex_object_inactive_time_;
  double     _object_distance_thr_;

  double     object_delete_time_;
  std::mutex mutex_object_delete_time_;

  std::string map_frame_;
  std::mutex  mutex_map_frame_;

  bool                                                         _exclude_drones_targets_ = false;
  std::string                                                  _uav_name_;
  std::vector<std::string>                                     _drones_names_;
  std::map<std::string, mbzirc_msgs::BrickGraspingDiagnostics> drones_diagnostics_;
  std::mutex                                                   mutex_drones_diagnostics_;
  std::vector<ros::Subscriber>                                 other_drones_subscribers_;
  double                                                       _exclude_drones_targets_timeout_;
  double                                                       _exclude_drones_targets_radius_;
  void                                                         otherDroneDiagnostics(const mbzirc_msgs::BrickGraspingDiagnosticsConstPtr &msg);

  int object_id_;

  // support for multiple objects
  std::list<ObjectHandler_t>           object_list_;
  std::mutex                           mutex_object_list_;
  std::list<ObjectHandler_t>::iterator findClosest(const double x, const double y, bool active, int type);
  std::list<ObjectHandler_t>::iterator findClosestFusion(mbzirc_msgs::ObjectWithType &obj_in, bool active, int type, double &new_len);

  // banning object dynamically
  double                  ban_area_radius_;
  double                  ban_area_timeout_;
  bool                    _allow_banning_areas_ = false;
  std::list<BannedArea_t> bannedAreaList;
  std::mutex              mutex_banned_areas_;

  ros::Timer estimator_timer_;
  void       estimatorTimer(const ros::TimerEvent &event);

  ros::Timer debug_timer_;
  void       debugTimer(const ros::TimerEvent &event);

  ros::Timer map_timer_;
  void       mapTimer(const ros::TimerEvent &event);

  // visualizer
  BatchVisualizer visualizer_red_bricks_;
  BatchVisualizer visualizer_green_bricks_;
  BatchVisualizer visualizer_blue_bricks_;
  BatchVisualizer visualizer_walls_;
  BatchVisualizer visualizer_ground_patterns_;
  BatchVisualizer visualizer_measurements_;

  // flipping stastistics
  std::mutex mutex_statistics_;
  int        double_detections_               = 0;
  int        smaller_brick_double_detections_ = 0;
  int        pi_2_flipping_high_              = 0;
  int        pi_flipping_high_                = 0;
  int        pi_2_flipping_low_               = 0;
  int        pi_flipping_low_                 = 0;
};

//}

/* onInit() //{ */

void BrickEstimation::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "BrickEstimation");

  param_loader.loadParam("wall_filtering/height_filtering/enabled", _wall_height_filtering_enabled);
  param_loader.loadParam("wall_filtering/height_filtering/min_height", _wall_min_height_);
  param_loader.loadParam("wall_filtering/height_filtering/max_height", _wall_max_height_);
  param_loader.loadParam("wall_filtering/projection_distance_limit", _wall_projection_distance_limit_);
  param_loader.loadParam("wall_filtering/max_length", _wall_max_len_);

  param_loader.loadParam("wall_angle_discrimination", _wall_angle_discrimination_);

  param_loader.loadParam("estimator_rate", _rate_);
  param_loader.loadParam("map_rate", _map_rate_);
  param_loader.loadParam("rviz_rate", _rviz_rate_);
  dt = 1.0 / double(_rate_);

  // subscriber to odometry and object detector topics
  sub_odometry_     = nh_.subscribe("odometry_in", 1, &BrickEstimation::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  sub_bricks_       = nh_.subscribe("bricks_in", 1, &BrickEstimation::callbackBricks, this, ros::TransportHints().tcpNoDelay());
  sub_brick_height_ = nh_.subscribe("brick_height_in", 1, &BrickEstimation::callbackBrickHeight, this, ros::TransportHints().tcpNoDelay());

  // publisher for the estimate for the lander
  pub_red_closest_   = nh_.advertise<mbzirc_msgs::MbzircBrick>("closest_red_out", 1);
  pub_green_closest_ = nh_.advertise<mbzirc_msgs::MbzircBrick>("closest_green_out", 1);
  pub_blue_closest_  = nh_.advertise<mbzirc_msgs::MbzircBrick>("closest_blue_out", 1);
  pub_wall_closest_  = nh_.advertise<mbzirc_msgs::MbzircBrick>("closest_wall_out", 1);
  pub_any_closest_   = nh_.advertise<mbzirc_msgs::MbzircBrick>("closest_any_out", 1);
  pub_map_           = nh_.advertise<mbzirc_msgs::BrickMap>("map_out", 1);

  // publishers for the state machine
  pub_red_closest_debugging_   = nh_.advertise<geometry_msgs::PointStamped>("closest_red_debugging_out", 1);
  pub_green_closest_debugging_ = nh_.advertise<geometry_msgs::PointStamped>("closest_green_debugging_out", 1);
  pub_blue_closest_debugging_  = nh_.advertise<geometry_msgs::PointStamped>("closest_blue_debugging_out", 1);
  pub_wall_closest_debugging_  = nh_.advertise<geometry_msgs::PointStamped>("closest_wall_debugging_out", 1);
  pub_any_closest_debugging_   = nh_.advertise<geometry_msgs::PointStamped>("closest_any_debugging_out", 1);

  // service for restarting the estimator
  service_server_reset_map_         = nh_.advertiseService("reset_map_in", &BrickEstimation::callbackRestart, this);
  service_server_ban_area_          = nh_.advertiseService("ban_area_in", &BrickEstimation::callbackBanArea, this);
  service_server_set_map_timeout_   = nh_.advertiseService("set_map_timeout_in", &BrickEstimation::callbackSetMapTimeout, this);
  service_server_stop_map_          = nh_.advertiseService("stop_map_out", &BrickEstimation::callbackStopMap, this);
  service_server_set_inactive_time_ = nh_.advertiseService("set_inactive_time_in", &BrickEstimation::callbackSetInactiveTime, this);

  service_client_validate_reference_ = nh_.serviceClient<mrs_msgs::ValidateReferenceList>("validate_reference_out");

  // state matrix
  param_loader.loadMatrixStatic("lkf/A", A);

  // input matrix
  param_loader.loadMatrixStatic("lkf/B", B);

  // measurement noise
  param_loader.loadMatrixStatic("lkf/R", R);

  // process covariance
  param_loader.loadMatrixStatic("lkf/Q", Q);

  // measurement mapping
  param_loader.loadMatrixStatic("lkf/H", H);

  lkf_ = std::make_unique<lkf_t>(A, B, H);

  param_loader.loadParam("object_distance_thr", _object_distance_thr_);
  param_loader.loadParam("object_inactive_time", object_inactive_time_);
  param_loader.loadParam("object_delete_time", object_delete_time_);

  param_loader.loadParam("UAV_NAME", _uav_name_);

  visualizer_measurements_    = BatchVisualizer(nh_, _uav_name_ + "/stable_origin", "debug_measurements_out");
  visualizer_red_bricks_      = BatchVisualizer(nh_, _uav_name_ + "/stable_origin", "debug_red_bricks_out");
  visualizer_green_bricks_    = BatchVisualizer(nh_, _uav_name_ + "/stable_origin", "debug_green_bricks_out");
  visualizer_blue_bricks_     = BatchVisualizer(nh_, _uav_name_ + "/stable_origin", "debug_blue_bricks_out");
  visualizer_walls_           = BatchVisualizer(nh_, _uav_name_ + "/stable_origin", "debug_walls_out");
  visualizer_ground_patterns_ = BatchVisualizer(nh_, _uav_name_ + "/stable_origin", "debug_ground_patterns_out");

  if (_uav_name_.empty()) {
    ROS_ERROR("[BrickEstimation]: UAV name has not been set!");
    ros::shutdown();
  }

  // excluding other drones targets
  param_loader.loadParam("exclude_drones_targets", _exclude_drones_targets_);
  param_loader.loadParam("drones_names", _drones_names_);
  param_loader.loadParam("exclude_drones_targets_timeout", _exclude_drones_targets_timeout_);
  param_loader.loadParam("exclude_drones_targets_radius", _exclude_drones_targets_radius_);

  // baning objects
  param_loader.loadParam("ban_area_radius", ban_area_radius_);
  param_loader.loadParam("ban_area_timeout", ban_area_timeout_);
  param_loader.loadParam("allow_banning_areas", _allow_banning_areas_);

  // exclude this drone from the list
  std::vector<std::string>::iterator it = _drones_names_.begin();
  while (it != _drones_names_.end()) {

    std::string temp_str = *it;

    if (!temp_str.compare(_uav_name_)) {

      _drones_names_.erase(it);
      continue;
    }

    it++;
  }

  std::string _diagnostics_topic_;
  param_loader.loadParam("diagnostics_topic", _diagnostics_topic_);

  if (_diagnostics_topic_.empty()) {
    ROS_ERROR("[BrickEstimation]: You need to define brick grasping's diagnostics topic.");
    ros::shutdown();
  }

  // create subscribers on other drones diagnostics
  for (unsigned int i = 0; i < _drones_names_.size(); i++) {

    std::string topic_name = std::string("/") + _drones_names_[i] + std::string("/") + _diagnostics_topic_;

    ROS_INFO("[BrickEstimation]: subscribing to %s", topic_name.c_str());

    other_drones_subscribers_.push_back(nh_.subscribe(topic_name, 1, &BrickEstimation::otherDroneDiagnostics, this, ros::TransportHints().tcpNoDelay()));
  }

  object_id_ = 0;

  // set up dynamically reconfigure server
  f = boost::bind(&BrickEstimation::callbackDynamicReconfigure, this, _1, _2);
  srv.setCallback(f);

  estimator_timer_ = nh_.createTimer(ros::Rate(_rate_), &BrickEstimation::estimatorTimer, this);
  map_timer_       = nh_.createTimer(ros::Rate(_map_rate_), &BrickEstimation::mapTimer, this);
  debug_timer_     = nh_.createTimer(ros::Rate(_rviz_rate_), &BrickEstimation::debugTimer, this);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[BrickEstimation]: Could not load all parameters!");
    ros::shutdown();
  }

  transformer_ = mrs_lib::Transformer("BrickEstimation", _uav_name_);

  is_initialized_ = true;

  ROS_INFO("[BrickEstimation]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackOdometry() //{ */

void BrickEstimation::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  geometry_msgs::PoseStamped odometry_main;

  odometry_main.header = msg->header;
  odometry_main.pose   = msg->pose.pose;

  // | ---------------- transform to stable origin --------------- |

  auto res = transformer_.transformSingle("stable_origin", odometry_main);

  if (res) {

    // set the member variable
    mrs_lib::set_mutexed(mutex_odometry_, res.value(), odometry_stable_);

    got_odometry_ = true;
  }
}

//}

/* callbackBrickHeight() //{ */

void BrickEstimation::callbackBrickHeight(const sensor_msgs::RangeConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[BrickEstimation]: getting brick height");

  std::scoped_lock lock(mutex_brick_height_);

  brick_height_ = *msg;

  got_brick_height_ = true;
}

//}

/* callbackBricks() //{ */

void BrickEstimation::callbackBricks(const mbzirc_msgs::ObjectWithTypeArrayConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  if (!got_odometry_) {
    return;
  }

  std::scoped_lock lock(mutex_object_list_);

  auto odometry_stable = mrs_lib::get_mutexed(mutex_odometry_, odometry_stable_);
  auto brick_height    = mrs_lib::get_mutexed(mutex_brick_height_, brick_height_);

  // throw the message awa if it is empty
  if (msg->objects.size() == 0)
    return;

  if (!first_brick_thrown_away_) {
    first_brick_thrown_away_ = true;
    return;
  }

  mbzirc_msgs::ObjectWithType          detected_obj;
  std::list<ObjectHandler_t>::iterator object_iter = object_list_.end();

  mrs_lib::set_mutexed(mutex_map_frame_, msg->header.frame_id, map_frame_);

  // clear update_this_time flags on all objects
  for (std::list<ObjectHandler_t>::iterator it = object_list_.begin(); it != object_list_.end(); it++) {

    it->updated_this_iter = false;
  }

  tf::Quaternion tf_quat;

  visualizer_measurements_.clear();

  /* validate against safety area //{ */

  mrs_msgs::ValidateReferenceList area_service;
  area_service.request.list.header.frame_id = msg->header.frame_id;
  area_service.request.list.header.stamp    = msg->header.stamp;

  for (int i = 0; i < int(msg->objects.size()); i++) {

    mrs_msgs::Reference reference;
    reference.position.x = msg->objects[i].x;
    reference.position.y = msg->objects[i].y;
    reference.position.z = 2.0;

    area_service.request.list.list.push_back(reference);
  }

  bool res = service_client_validate_reference_.call(area_service);

  bool safety_area_checked = false;

  if (res) {

    safety_area_checked = true;

  } else {

    ROS_ERROR_THROTTLE(1.0, "[BrickEstimation]: could not check objects agains the safety area");
    safety_area_checked = false;
  }

  //}

  // iterate over all objects
  for (int i = 0; i < int(msg->objects.size()); i++) {

    // load the object
    detected_obj = msg->objects[i];

    // do not see blue bricks
    if (detected_obj.type == OBJECT_BLUE) {
      continue;
    }

    if (std::isnan(detected_obj.x) || std::isinf(detected_obj.x) || std::isnan(detected_obj.y) || std::isinf(detected_obj.y)) {

      ROS_WARN_THROTTLE(1, "[BrickEstimation]: Object detector produced NANs!");
      continue;
    }

    /* publish the measurements markers //{ */

    Eigen::Vector3d pose;
    pose << detected_obj.x, detected_obj.y, detected_obj.z - 0.2;

    tf_quat.setEuler(0, 0, detected_obj.yaw);
    Eigen::Quaterniond orientation(tf_quat.w(), tf_quat.x(), tf_quat.y(), tf_quat.z());

    Eigen::Vector3d scale;
    Eigen::Vector4d color;

    switch (detected_obj.type) {

        /* RED //{ */

      case OBJECT_RED: {

        // show the brick
        scale << 0.3, 0.2, 0.2;
        color << 0.3, 0.0, 0.0, 1.0;

        break;
      }

        //}

        /* GREEN //{ */

      case OBJECT_GREEN: {

        scale << 0.6, 0.2, 0.2;
        color << 0.0, 0.3, 0.0, 1.0;

        break;
      }

        //}

        /* BLUE //{ */

      case OBJECT_BLUE: {

        scale << 1.2, 0.2, 0.2;
        color << 0.0, 0.0, 0.3, 1.0;

        break;
      }

        //}

        /* WALL //{ */

      case OBJECT_WALL: {

        scale << detected_obj.len, 0.2, 0.01;
        color << 0.3, 0.3, 0.3, 1.0;

        break;
      }

        //}

        /* WALL //{ */

      case OBJECT_GROUND_PATTERN: {

        scale << 1.0, 1.0, 0.01;
        color << 0.0, 0.0, 0.0, 1.0;

        break;
      }

        //}
    }

    visualizer_measurements_.addCuboid(pose, orientation, scale, color);

    //}

    /* exclude other UAV targets //{ */

    if (_exclude_drones_targets_ && (detected_obj.type >= OBJECT_RED && detected_obj.type <= OBJECT_BLUE)) {

      std::map<std::string, mbzirc_msgs::BrickGraspingDiagnostics>::iterator map_it;
      bool                                                                   exclude_object = false;

      // go throught map full of other drones object lander diagnostics
      for (map_it = drones_diagnostics_.begin(); map_it != drones_diagnostics_.end(); map_it++) {

        // if it is fresh enought
        if ((ros::Time::now() - map_it->second.stamp).toSec() < _exclude_drones_targets_timeout_) {

          // if the drones lander is active
          if (map_it->second.active) {

            // if the drones target is too close
            if (mrs_lib::geometry::dist(vec2_t(map_it->second.target.position.x, map_it->second.target.position.y), vec2_t(detected_obj.x, detected_obj.y)) <
                _exclude_drones_targets_radius_) {

              exclude_object = true;
              ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: excluding other UAV targets from the measurement vector, other UAV target is too close");
            }

            // if the drone itself is too close
            if (mrs_lib::geometry::dist(vec2_t(map_it->second.position.position.x, map_it->second.position.position.y),
                                        vec2_t(detected_obj.x, detected_obj.y)) < _exclude_drones_targets_radius_) {

              exclude_object = true;
              ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: excluding other UAV targets from the measurement vector, other UAV is too close");
            }
          }
        }
      }

      // remove this object from out map
      if (exclude_object) {
        continue;
      }
    }

    //}

    /* exclude safety area violating objects //{ */

    if (safety_area_checked) {
      if (!area_service.response.success[i]) {
        ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: rejection object outside of the safety area");
        continue;
      }
    }

    //}

    /* exclude measurements from banned areas //{ */

    if (_allow_banning_areas_) {

      std::list<BannedArea_t>::iterator area_it;
      bool                              exclude_object = false;

      // go throught all banned areas
      {

        std::scoped_lock lock(mutex_banned_areas_);

        for (area_it = bannedAreaList.begin(); area_it != bannedAreaList.end(); area_it++) {

          // if it is up to date
          if ((ros::Time::now() - area_it->stamp).toSec() < ban_area_timeout_) {

            // if the object lies within the area
            if (mrs_lib::geometry::dist(vec2_t(area_it->x, area_it->y), vec2_t(detected_obj.x, detected_obj.y)) < ban_area_radius_) {

              exclude_object = true;
            }

          } else {

            // delete the banned area, since it has expired
            ROS_INFO("[BrickEstimation]: Removing banned area around x=%.2f, y=%.2f.", area_it->x, area_it->y);
            area_it = bannedAreaList.erase(area_it);
            continue;
          }
        }
      }

      // remove this object from out map
      if (exclude_object) {
        continue;
      }
    }

    //}

    /* exclude walls under min and over max height //{ */

    if (_wall_height_filtering_enabled && got_brick_height_) {

      if (detected_obj.type == OBJECT_WALL) {

        double wall_ground_height = brick_height.range - (odometry_stable.pose.position.z - detected_obj.z);

        if (wall_ground_height < _wall_min_height_) {

          ROS_WARN("[BrickEstimation]: detected wall below the minimum height: z = %.2f !", wall_ground_height);
          continue;

        } else if (wall_ground_height > _wall_max_height_) {

          ROS_WARN("[BrickEstimation]: detected wall above the maximum height: z = %.2f !", wall_ground_height);
          continue;
        }
      }
    }

    //}

    double new_len = 0;

    // find match in the map
    object_iter = findClosestFusion(detected_obj, false, OBJECT_ANY, new_len);

    double object_dist = std::numeric_limits<double>::max();

    if (object_iter != object_list_.end()) {
      object_dist = mrs_lib::geometry::dist(vec2_t(detected_obj.x, detected_obj.y), vec2_t(object_iter->statecov.x[0], object_iter->statecov.x[1]));
    }

    // match not found, we should create fresh object
    if ((object_iter->type != OBJECT_WALL && object_dist > _object_distance_thr_) || (object_iter->type == OBJECT_WALL && object_iter == object_list_.end())) {

      // a closest brick of any type
      {
        std::list<ObjectHandler_t>::iterator object_iter_alternative = object_list_.end();

        object_iter_alternative = findClosestFusion(detected_obj, false, OBJECT_ANY, new_len);

        double object_dist_alternative = std::numeric_limits<double>::max();

        if (object_iter_alternative != object_list_.end()) {
          object_dist_alternative =
              mrs_lib::geometry::dist(vec2_t(detected_obj.x, detected_obj.y), vec2_t(object_iter_alternative->x, object_iter_alternative->y));
        }

        if (object_dist_alternative < _object_distance_thr_) {

          if (object_iter_alternative->type > detected_obj.type) {

            ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: detected a smaller brick in a larger brick");
            {
              std::scoped_lock lock(mutex_statistics_);

              smaller_brick_double_detections_++;
            }
            continue;

          } else if (object_iter_alternative->type == detected_obj.type) {

            ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: brown alert, double detection!");

            {
              std::scoped_lock lock(mutex_statistics_);

              double_detections_++;
            }
            continue;
          }
        }
      }

      ObjectHandler_t new_object;
      new_object.header.stamp      = detected_obj.stamp;
      new_object.hue               = 0;
      new_object.type              = detected_obj.type;
      new_object.active            = true;
      new_object.last_updated      = ros::Time::now();
      new_object.firs_seen         = ros::Time::now();
      new_object.updated_this_iter = false;
      new_object.id                = object_id_++;
      new_object.x                 = detected_obj.x;
      new_object.y                 = detected_obj.y;
      new_object.z                 = detected_obj.z;
      new_object.len               = detected_obj.len;
      new_object.n_corrections     = 0;

      new_object.flipper_bias_none     = 0;
      new_object.flipper_bias_positive = 0;
      new_object.flipper_bias_negative = 0;

      if (i < int(size(msg->uav_odometries))) {
        new_object.uav_odom = msg->uav_odometries[i];
      }

      new_object.yaw = detected_obj.yaw;

      // Generate initial state and covariance
      x_t        x0 = {detected_obj.x, detected_obj.y, detected_obj.z, detected_obj.yaw};
      P_t        P0 = Eigen::MatrixXd::Identity(_n_states_, _n_states_);
      statecov_t sc0({x0, P0});

      new_object.statecov = sc0;

      ROS_INFO("[BrickEstimation]: Instancing new LKF.");

      object_list_.push_back(new_object);
    } else {  // match found, we are gonna `fuse`

      // | ------------------ fix the brick's color ----------------- |

      if (detected_obj.type >= OBJECT_RED && detected_obj.type <= OBJECT_BLUE) {

        object_iter->type = detected_obj.type > object_iter->type ? detected_obj.type : object_iter->type;

        ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: sanitizing object type!");

      } else {
        object_iter->type = detected_obj.type;
      }

      double alpha     = 0.5;
      object_iter->len = (alpha)*object_iter->len + (1 - alpha) * new_len;

      // create the measurement vector
      VectorXd measurement = VectorXd::Zero(_n_measurements_);

      double unwrapped_measurement = sradians::unwrap(detected_obj.yaw, object_iter->statecov.x[3]);
      double yaw_difference        = sradians::diff(unwrapped_measurement, object_iter->statecov.x[3]);

      double disambiguated_measurement = unwrapped_measurement;

      std::string brick_color;

      switch (object_iter->type) {
        case OBJECT_RED: {
          brick_color = "red";
          break;
        }
        case OBJECT_GREEN: {
          brick_color = "green";
          break;
        }
        case OBJECT_BLUE: {
          brick_color = "blue";
          break;
        }
        case OBJECT_WALL: {
          brick_color = "wall";
          break;
        }
      }

      // check the positive offset
      if ((yaw_difference >= ((3.0 / 8.0) * M_PI)) && (yaw_difference <= ((3.0 / 4.0) * M_PI))) {

        disambiguated_measurement += -(M_PI / 2.0);
        object_iter->flipper_bias_negative++;

        ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: yaw of the \"%s\" differs too much (+pi/2), old estimate: %.2f, measurement: %.2f", brick_color.c_str(),
                          object_iter->yaw, unwrapped_measurement);

      } else if ((yaw_difference >= ((3.0 / 4.0) * M_PI)) && (yaw_difference <= ((5.0 / 4.0) * M_PI))) {

        disambiguated_measurement += -M_PI;
        object_iter->flipper_bias_none++;

        ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: yaw of the \"%s\" differs too much (+pi), old estimate: %.2f, measurement: %.2f", brick_color.c_str(),
                          object_iter->yaw, unwrapped_measurement);

        // check the negative offset
      } else if ((yaw_difference <= -((3.0 / 8.0) * M_PI)) && (yaw_difference >= -((3.0 / 4.0) * M_PI))) {

        disambiguated_measurement += M_PI / 2.0;
        object_iter->flipper_bias_positive++;

        ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: yaw of the \"%s\" differs too much (-pi/2), old estimate: %.2f, measurement: %.2f", brick_color.c_str(),
                          object_iter->yaw, unwrapped_measurement);

      } else if ((yaw_difference <= -((3.0 / 4.0) * M_PI)) && (yaw_difference >= -((5.0 / 4.0) * M_PI))) {

        disambiguated_measurement += M_PI;
        object_iter->flipper_bias_none++;

        ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: yaw of the \"%s\" differs too much (-pi), old estimate: %.2f, measurement: %.2f", brick_color.c_str(),
                          object_iter->yaw, unwrapped_measurement);
      } else {

        object_iter->flipper_bias_none++;
      }

      measurement << detected_obj.x, detected_obj.y, detected_obj.z, disambiguated_measurement;

      // update the altitude
      object_iter->z = detected_obj.z;

      try {
        object_iter->statecov = lkf_->correct(object_iter->statecov, measurement, R);
      }
      catch (...) {
        ROS_WARN("[BrickEstimation]: Deleting the object from object_list_ based on an exception (correction).");

        object_list_.erase(object_iter);
        continue;
      }

      object_iter->n_corrections++;

      // update the positions in the handler
      object_iter->x   = object_iter->statecov.x[0];
      object_iter->y   = object_iter->statecov.x[1];
      object_iter->z   = object_iter->statecov.x[2];
      object_iter->yaw = object_iter->statecov.x[3];

      if (i < int(size(msg->uav_odometries))) {

        /* double cos_y = cos(object_iter->odom_yaw); */
        /* double sin_y = sin(object_iter->odom_yaw); */

        /* double temp_x = cos_y * msg->uav_odometries[i].x - sin_y * msg->uav_odometries[i].y; */
        /* double temp_y = sin_y * msg->uav_odometries[i].x + cos_y * msg->uav_odometries[i].y; */

        object_iter->uav_odom.header = msg->uav_odometries[i].header;
        object_iter->uav_odom.z      = msg->uav_odometries[i].z;

        double yaw_odom_offset = 0;

        double unwrapped_measurement = sradians::unwrap(msg->uav_odometries[i].yaw, object_iter->uav_odom.yaw);
        double yaw_difference        = sradians::diff(unwrapped_measurement, object_iter->uav_odom.yaw);

        if (yaw_difference >= ((3.0 / 4.0) * M_PI)) {

          yaw_odom_offset = -M_PI;
          ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: odom yaw from the \"%s\" differs too much (+pi), old estimate: %.2f, measurement: %.2f",
                            brick_color.c_str(), object_iter->uav_odom.yaw, unwrapped_measurement);

          {
            std::scoped_lock lock(mutex_statistics_);

            if (odometry_stable.pose.position.z >= 1.5) {
              pi_flipping_high_++;
            } else {
              pi_flipping_low_++;
            }
          }

        } else if (yaw_difference <= (-(3.0 / 4.0) * M_PI)) {

          yaw_odom_offset = M_PI;
          ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: odom yaw from the \"%s\" differs too much (-pi), old estimate: %.2f, measurement: %.2f",
                            brick_color.c_str(), object_iter->uav_odom.yaw, unwrapped_measurement);

          {
            std::scoped_lock lock(mutex_statistics_);

            if (odometry_stable.pose.position.z >= 1.5) {
              pi_flipping_high_++;
            } else {
              pi_flipping_low_++;
            }
          }

        } else if (yaw_difference >= ((3.0 / 8.0) * M_PI) && yaw_difference < ((3.0 / 4.0) * M_PI)) {

          ROS_DEBUG("[BrickEstimation]: !!! odom yaw from the \"%s\" differs by pi/2, old estimate: %.2f, measurement: %.2f, brick height: %.2f",
                    brick_color.c_str(), object_iter->uav_odom.yaw, unwrapped_measurement, object_iter->uav_odom.z);

          {
            std::scoped_lock lock(mutex_statistics_);

            if (odometry_stable.pose.position.z >= 1.5) {
              pi_2_flipping_high_++;
            } else {
              pi_2_flipping_low_++;
            }
          }

        } else if (yaw_difference <= (-(3.0 / 8.0) * M_PI) && yaw_difference > (-(3.0 / 4.0) * M_PI)) {

          ROS_DEBUG("[BrickEstimation]: !!! odom yaw from the \"%s\" differs by -pi/2, old estimate: %.2f, measurement: %.2f, brick height: %.2f",
                    brick_color.c_str(), object_iter->uav_odom.yaw, unwrapped_measurement, object_iter->uav_odom.z);

          {
            std::scoped_lock lock(mutex_statistics_);

            if (odometry_stable.pose.position.z >= 1.5) {
              pi_2_flipping_high_++;
            } else {
              pi_2_flipping_low_++;
            }
          }
        }

        double cos_y = cos(yaw_odom_offset);
        double sin_y = sin(yaw_odom_offset);

        double temp_x = cos_y * msg->uav_odometries[i].x - sin_y * msg->uav_odometries[i].y;
        double temp_y = sin_y * msg->uav_odometries[i].x + cos_y * msg->uav_odometries[i].y;

        object_iter->uav_odom.x   = temp_x;
        object_iter->uav_odom.y   = temp_y;
        object_iter->uav_odom.yaw = unwrapped_measurement + yaw_odom_offset;
      }

      object_iter->updated_this_iter = true;

      // update the time
      object_iter->last_updated = ros::Time::now();
      object_iter->active       = true;
    }
  }  // namespace brick_estimation

  visualizer_measurements_.publish(600, false);
}  // namespace brick_estimation

//}

/* callbackDynamicReconfigure() //{ */
void BrickEstimation::callbackDynamicReconfigure([[maybe_unused]] brick_estimation::brick_estimation_drsConfig &config, uint32_t level) {

  if (level == 4294967295 || level == 0)  // nothing has changed
    return;
}
//}

/* callbackBanArea() //{ */

bool BrickEstimation::callbackBanArea(mbzirc_msgs::BanArea::Request &req, mbzirc_msgs::BanArea::Response &res) {

  if (!is_initialized_) {
    return false;
  }

  std::scoped_lock lock(mutex_banned_areas_);

  BannedArea_t newArea;

  newArea.stamp = ros::Time::now();

  newArea.x = req.x;
  newArea.y = req.y;

  bannedAreaList.push_back(newArea);

  ROS_INFO("[BrickEstimation]: Area temporarily banned %.2f meters around x=%.2f, y=%.2f", ban_area_radius_, newArea.x, newArea.y);

  res.success = true;
  res.message = "Area banned.";

  return true;
}

//}

/* callbackSetMapTimeout() //{ */

bool BrickEstimation::callbackSetMapTimeout(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res) {

  mrs_lib::set_mutexed(mutex_object_delete_time_, req.value, object_delete_time_);

  ROS_INFO("[BrickEstimation]: update map timeout to %.2f", object_delete_time_);

  res.success = true;
  res.message = "timeout set";

  return true;
}

//}

/* callbackStopMap() //{ */

bool BrickEstimation::callbackStopMap(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  ROS_INFO("[BrickEstimation]: %s map publisher", req.data ? "starting" : "stopping");

  publishing_map_ = req.data;

  res.success = true;
  res.message = "done";

  return true;
}

//}

/* callbackSetInactiveTime() //{ */

bool BrickEstimation::callbackSetInactiveTime(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res) {

  if (!is_initialized_) {
    return false;
  }

  std::scoped_lock lock(mutex_object_inactive_time_);

  object_inactive_time_ = req.value;

  ROS_INFO("[BrickEstimation]: inacative time se to %.1f s", object_inactive_time_);

  res.success = true;
  res.message = "done";

  return true;
}

//}

/* callbackRestart() //{ */

bool BrickEstimation::callbackRestart([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  std::scoped_lock lock(mutex_object_list_);

  ROS_INFO("[BrickEstimation]: reseting map");

  /* object_list_.clear(); */

  std::list<ObjectHandler_t> old_object_list = object_list_;

  object_list_.clear();

  // find the closest object in the list of filters
  for (std::list<ObjectHandler_t>::iterator it1 = old_object_list.begin(); it1 != old_object_list.end(); it1++) {

    if (it1->type == OBJECT_WALL || it1->type == OBJECT_GROUND_PATTERN) {

      object_list_.push_back(*it1);
    }
  }

  res.success = true;
  res.message = "map cleared.";

  return true;
}

//}

/* otherDroneDiagnostics() //{ */

void BrickEstimation::otherDroneDiagnostics(const mbzirc_msgs::BrickGraspingDiagnosticsConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lock(mutex_drones_diagnostics_);

  ROS_INFO_ONCE("[BrickEstimation]: getting other drone diagnostics");

  mbzirc_msgs::BrickGraspingDiagnostics temp_diagnostics = *msg;

  temp_diagnostics.stamp = ros::Time::now();

  {
    geometry_msgs::PoseStamped other_drone_odom_its_stable;
    other_drone_odom_its_stable.header.frame_id = "utm_origin";
    other_drone_odom_its_stable.header.stamp    = ros::Time::now();
    other_drone_odom_its_stable.pose            = msg->position;

    auto res = transformer_.transformSingle("stable_origin", other_drone_odom_its_stable);

    geometry_msgs::PoseStamped other_drone_odom_my_stable;

    if (res) {

      temp_diagnostics.position = res.value().pose;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[BrickEstimation]: could not transform other drone odom to my stable");
      return;
    }
  }

  {
    geometry_msgs::PoseStamped other_drone_target_its_stable;
    other_drone_target_its_stable.header.frame_id = "utm_origin";
    other_drone_target_its_stable.header.stamp    = ros::Time::now();
    other_drone_target_its_stable.pose            = msg->target;

    auto res = transformer_.transformSingle("stable_origin", other_drone_target_its_stable);

    if (res) {

      temp_diagnostics.target = res.value().pose;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[BrickEstimation]: could not transform other drone target to my stable");
      return;
    }
  }

  // update the diagnostics
  drones_diagnostics_[msg->uav_name] = temp_diagnostics;
}

//}

// --------------------------------------------------------------
// |                           models                           |
// --------------------------------------------------------------

/* findClosest() //{ */

std::list<ObjectHandler_t>::iterator BrickEstimation::findClosest(const double x, const double y, bool active, int type) {

  auto odometry_stable = mrs_lib::get_mutexed(mutex_odometry_, odometry_stable_);

  double uav_heading = 0;

  try {
    uav_heading = mrs_lib::AttitudeConverter(odometry_stable.pose.orientation).getHeading();
  }
  catch (...) {
  }

  std::list<ObjectHandler_t>::iterator obj_iter = object_list_.end();

  double mindist = std::numeric_limits<double>::max();

  // find the closest object in the list of filters
  for (std::list<ObjectHandler_t>::iterator it = object_list_.begin(); it != object_list_.end(); it++) {

    if (active)         // we want active object
      if (!it->active)  // and the one iterating over is not active
        continue;       // skip it

    // the type has to match, unless is OBJECT_ANY
    if (type != OBJECT_ANY) {
      if (type != it->type) {
        continue;
      }
    }

    // Find only the walls, which are aligned with the drone.
    // This helps to place the bricks on the right walls.
    if (_wall_angle_discrimination_) {
      if (it->type == OBJECT_WALL) {
        double angle_between = fabs(radians::diff(it->yaw, uav_heading));
        if (fabs(angle_between) > ((1.0 / 4.0) * M_PI) && fabs(angle_between) < ((3.0 / 4.0) * M_PI)) {
          continue;
        }
      }
    }

    double distToObject = mrs_lib::geometry::dist(vec2_t(x, y), vec2_t(it->x, it->y));

    if (distToObject < mindist) {
      mindist  = distToObject;
      obj_iter = it;
    }
  }

  return obj_iter;
}

//}

/* findClosestFusion() //{ */

std::list<ObjectHandler_t>::iterator BrickEstimation::findClosestFusion(mbzirc_msgs::ObjectWithType &obj_in, bool active, int type, double &new_len) {

  std::list<ObjectHandler_t>::iterator obj_iter = object_list_.end();

  double mindist = std::numeric_limits<double>::max();
  new_len        = 0;

  // find the closest object in the list of filters
  for (std::list<ObjectHandler_t>::iterator it = object_list_.begin(); it != object_list_.end(); it++) {

    if (active)         // we want active object
      if (!it->active)  // and the one iterating over is not active
        continue;       // skip it

    // the type has to match, unless is OBJECT_ANY
    if (type != OBJECT_ANY) {
      if (type != it->type) {
        continue;
      }
    }

    // dont cross bricks with wall
    if (obj_in.type <= OBJECT_BLUE && it->type == OBJECT_WALL) {
      continue;
    }

    // dont cross wall with bricks
    if (obj_in.type == OBJECT_WALL && it->type <= OBJECT_BLUE) {
      continue;
    }

    double distToObject = mrs_lib::geometry::dist(vec2_t(obj_in.x, obj_in.y), vec2_t(it->statecov.x[0], it->statecov.x[1]));

    if (obj_in.type == OBJECT_WALL) {

      Eigen::Vector2d center1 = Eigen::Vector2d(it->statecov.x[0], it->statecov.x[1]);
      Eigen::Vector2d center2 = Eigen::Vector2d(obj_in.x, obj_in.y);

      double yaw_1 = sradians::wrap(it->statecov.x[3]);
      double yaw_2 = sradians::wrap(obj_in.yaw);

      double len_1 = it->len;
      double len_2 = obj_in.len;

      double angle_between = sradians::diff(yaw_1, yaw_2);

      Eigen::Vector2d end_1_1 = center1 + Eigen::Vector2d(cos(yaw_1) * len_1 / 2.0, sin(yaw_1) * len_1 / 2.0);
      Eigen::Vector2d end_1_2 = center1 + Eigen::Vector2d(cos(yaw_1 + M_PI) * len_1 / 2.0, sin(yaw_1 + M_PI) * len_1 / 2.0);

      Eigen::Vector2d end_2_1 = center2 + Eigen::Vector2d(cos(yaw_2) * len_2 / 2.0, sin(yaw_2) * len_2 / 2.0);
      Eigen::Vector2d end_2_2 = center2 + Eigen::Vector2d(cos(yaw_2 + M_PI) * len_2 / 2.0, sin(yaw_2 + M_PI) * len_2 / 2.0);

      Eigen::Vector2d base_1 = end_1_1 - center1;
      base_1.normalize();
      Eigen::Vector2d base_2 = end_2_1 - center2;
      base_2.normalize();

      Eigen::Matrix2d projector_1 = base_1 * base_1.transpose();

      Eigen::Vector2d end_2_1_proj  = (projector_1 * (end_2_1 - center1)) + center1;
      Eigen::Vector2d end_2_2_proj  = (projector_1 * (end_2_2 - center1)) + center1;
      Eigen::Vector2d center_2_proj = (projector_1 * (center2 - center1)) + center1;

      // will it make the object longer?
      std::vector<double> lens;
      lens.insert(lens.begin(), (center1 - end_1_1).norm());
      lens.insert(lens.begin(), (center1 - end_1_2).norm());
      lens.insert(lens.begin(), (center1 - end_2_1_proj).norm());
      lens.insert(lens.begin(), (center1 - end_2_2_proj).norm());

      double my_max = 0;
      for (int i = 0; i < 4; i++) {
        if (lens.at(i) > my_max) {
          my_max = lens.at(i);
        }
      }

      double new_wall_len_upper      = len_1 / 2.0 + my_max;
      double projection_end_2_1_len  = (end_2_1 - end_2_1_proj).norm();
      double projection_end_2_2_len  = (end_2_2 - end_2_2_proj).norm();
      double projection_center_2_len = (center2 - center_2_proj).norm();
      double centers_dist            = (center1 - center2).norm();

      ROS_INFO("[BrickEstimation]: wall candidate for [%.2f, %.2f] at [%.2f, %.2f], new len %.2f, proj1 %.2f, proj2 %.2f", center1[0], center1[1], center2[0],
               center2[1], new_wall_len_upper, projection_end_2_1_len, projection_end_2_2_len);

      if (projection_end_2_1_len < _wall_projection_distance_limit_ && projection_end_2_2_len < _wall_projection_distance_limit_ && centers_dist <= 3.0) {

        if (fabs(angle_between) >= (M_PI / 4.0) && fabs(angle_between) <= ((3.0 / 4.0) * M_PI)) {

          ROS_WARN("[BrickEstimation]: found match, but the angle is wrong, [%.2f, %.2f: %.2f], [%.2f, %.2f: %.2f]", center1[0], center1[1], yaw_1, center2[0],
                   center2[1], yaw_2);
          continue;
        }

        mindist  = projection_center_2_len;
        obj_iter = it;
        new_len  = new_wall_len_upper <= _wall_max_len_ ? new_wall_len_upper : _wall_max_len_;

        ROS_WARN("[BrickEstimation]: found wall match, center distance: %.2f m, length upper bound: %.2f m", centers_dist, new_wall_len_upper);
      }

    } else {

      if (distToObject < mindist) {
        mindist  = distToObject;
        obj_iter = it;
      }
    }
  }

  return obj_iter;
}

//}

/* publish() //{ */

/**
 * @brief BrickEstimation::publish
 */
void BrickEstimation::publish() {

  std::scoped_lock lock(mutex_object_list_);

  auto odometry_stable = mrs_lib::get_mutexed(mutex_odometry_, odometry_stable_);
  auto map_frame       = mrs_lib::get_mutexed(mutex_map_frame_, map_frame_);

  mbzirc_msgs::MbzircBrick brick;

  std::list<ObjectHandler_t>::iterator brick_ptr;

  // | ----------------------- closest red ---------------------- |

  /* closest red //{ */

  brick.header.stamp    = ros::Time::now();
  brick.header.frame_id = map_frame;

  // find the closest and active red brick
  brick_ptr = findClosest(odometry_stable.pose.position.x, odometry_stable.pose.position.y, true, OBJECT_RED);

  // if there are none, return
  if (brick_ptr == object_list_.end()) {

    brick.valid = false;

  } else {

    // fill up the state vector
    for (int i = 0; i < _n_states_; i++) {

      brick.states[i] = brick_ptr->statecov.x[i];
    }

    brick.valid    = true;
    brick.altitude = brick_ptr->z;
    brick.yaw      = brick_ptr->yaw;
    brick.type     = brick_ptr->type;

    brick.uav_odom = brick_ptr->uav_odom;

    if (last_closest_red_ != brick_ptr->id) {
      ROS_INFO("[BrickEstimation]: new red brick odometry being published, requesting odom reset");
      brick.uav_odom.z  = -1;
      last_closest_red_ = brick_ptr->id;
    }

    // publish the pose
    try {
      pub_red_closest_.publish(brick);
    }
    catch (...) {
      ROS_ERROR("[BrickEstimation]: Exception caught during publishing topic %s.", pub_red_closest_.getTopic().c_str());
    }
  }

  //}

  // | ---------------------- closest green --------------------- |

  /* closest green //{ */

  // find the closest and active green brick
  brick_ptr = findClosest(odometry_stable.pose.position.x, odometry_stable.pose.position.y, true, OBJECT_GREEN);

  // if there are none, return
  if (brick_ptr == object_list_.end()) {

    brick.valid = false;

  } else {

    // fill up the state vector
    for (int i = 0; i < _n_states_; i++) {

      brick.states[i] = brick_ptr->statecov.x[i];
    }

    brick.valid    = true;
    brick.altitude = brick_ptr->z;
    brick.yaw      = brick_ptr->yaw;
    brick.type     = brick_ptr->type;

    brick.uav_odom = brick_ptr->uav_odom;

    if (last_closest_green_ != brick_ptr->id) {
      ROS_INFO("[BrickEstimation]: new green brick odometry being published, requesting odom reset");
      brick.uav_odom.z    = -1;
      last_closest_green_ = brick_ptr->id;
    }

    // publish the pose
    try {
      pub_green_closest_.publish(brick);
    }
    catch (...) {
      ROS_ERROR("[BrickEstimation]: Exception caught during publishing topic %s.", pub_green_closest_.getTopic().c_str());
    }
  }

  //}

  // | ---------------------- closest blue ---------------------- |

  /* closest blue //{ */

  // find the closest and active blue brick
  brick_ptr = findClosest(odometry_stable.pose.position.x, odometry_stable.pose.position.y, true, OBJECT_BLUE);

  // if there are none, return
  if (brick_ptr == object_list_.end()) {

    brick.valid = false;

  } else {

    // fill up the state vector
    for (int i = 0; i < _n_states_; i++) {

      brick.states[i] = brick_ptr->statecov.x[i];
    }

    brick.valid    = true;
    brick.altitude = brick_ptr->z;
    brick.yaw      = brick_ptr->yaw;
    brick.type     = brick_ptr->type;

    brick.uav_odom = brick_ptr->uav_odom;

    if (last_closest_blue_ != brick_ptr->id) {
      ROS_INFO("[BrickEstimation]: new blue brick odometry being published, requesting odom reset");
      brick.uav_odom.z   = -1;
      last_closest_blue_ = brick_ptr->id;
    }

    // publish the pose
    try {
      pub_blue_closest_.publish(brick);
    }
    catch (...) {
      ROS_ERROR("[BrickEstimation]: Exception caught during publishing topic %s.", pub_blue_closest_.getTopic().c_str());
    }
  }

  //}

  // | ---------------------- closest wall ---------------------- |

  /* closest wall //{ */

  // find the closest and active wall
  brick_ptr = findClosest(odometry_stable.pose.position.x, odometry_stable.pose.position.y, true, OBJECT_WALL);

  // if there are none, return
  if (brick_ptr == object_list_.end()) {

    brick.valid = false;

  } else {

    // fill up the state vector
    for (int i = 0; i < _n_states_; i++) {

      brick.states[i] = brick_ptr->statecov.x[i];
    }

    brick.valid    = true;
    brick.altitude = brick_ptr->z;
    brick.yaw      = brick_ptr->yaw;
    brick.type     = brick_ptr->type;

    brick.uav_odom = brick_ptr->uav_odom;

    if (last_closest_wall_ != brick_ptr->id) {
      ROS_INFO("[BrickEstimation]: new wall brick odometry being published, requesting odom reset");
      brick.uav_odom.z   = -1;
      last_closest_wall_ = brick_ptr->id;
    }

    // publish the pose
    try {
      pub_wall_closest_.publish(brick);
    }
    catch (...) {
      ROS_ERROR("[BrickEstimation]: Exception caught during publishing topic %s.", pub_wall_closest_.getTopic().c_str());
    }
  }

  //}

  // | ----------------------- closest any ---------------------- |

  /* closest any //{ */

  // find the closest and active any
  brick_ptr = findClosest(odometry_stable.pose.position.x, odometry_stable.pose.position.y, true, OBJECT_ANY);

  // if there are none, return
  if (brick_ptr == object_list_.end()) {

    brick.valid = false;

  } else {

    // fill up the state vector
    for (int i = 0; i < _n_states_; i++) {

      brick.states[i] = brick_ptr->statecov.x[i];
    }

    brick.valid    = true;
    brick.altitude = brick_ptr->z;
    brick.yaw      = brick_ptr->yaw;
    brick.type     = brick_ptr->type;

    brick.uav_odom = brick_ptr->uav_odom;

    if (last_closest_any_ != brick_ptr->id) {
      ROS_INFO("[BrickEstimation]: new any brick odometry being published, requesting odom reset");
      brick.uav_odom.z  = -1;
      last_closest_any_ = brick_ptr->id;
    }

    // publish the pose
    try {
      pub_any_closest_.publish(brick);
    }
    catch (...) {
      ROS_ERROR("[BrickEstimation]: Exception caught during publishing topic %s.", pub_any_closest_.getTopic().c_str());
    }
  }

  //}

}  // namespace brick_estimation

//}

/* iterate() //{ */

void BrickEstimation::iterate() {

  auto object_delete_time   = mrs_lib::get_mutexed(mutex_object_delete_time_, object_delete_time_);
  auto object_inactive_time = mrs_lib::get_mutexed(mutex_object_inactive_time_, object_inactive_time_);

  std::scoped_lock lock(mutex_object_list_);

  std::list<ObjectHandler_t>::iterator it = object_list_.begin();

  while (it != object_list_.end()) {

    ObjectHandler_t *obj_pointer = &(*it);

    // exlude object beeing grasped by other drones
    if (_exclude_drones_targets_ && (obj_pointer->type >= OBJECT_RED && obj_pointer->type <= OBJECT_BLUE)) {

      std::scoped_lock lock(mutex_drones_diagnostics_);

      std::map<std::string, mbzirc_msgs::BrickGraspingDiagnostics>::iterator map_it;

      bool exclude_object = false;

      // go throught map full of other drones object lander diagnostics
      for (map_it = drones_diagnostics_.begin(); map_it != drones_diagnostics_.end(); map_it++) {

        // if it is new enought
        if ((ros::Time::now() - map_it->second.stamp).toSec() < _exclude_drones_targets_timeout_) {

          // if the drones lander is active
          if (map_it->second.active) {

            // if the drones target is too close
            if (mrs_lib::geometry::dist(vec2_t(map_it->second.target.position.x, map_it->second.target.position.y),
                                        vec2_t(obj_pointer->statecov.x[0], obj_pointer->statecov.x[1])) < _exclude_drones_targets_radius_) {

              ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: excluding objects from the map, other drone's target is too close");
              exclude_object = true;
            }

            // if the drone itself is too close
            if (mrs_lib::geometry::dist(vec2_t(map_it->second.position.position.x, map_it->second.position.position.y),
                                        vec2_t(obj_pointer->statecov.x[0], obj_pointer->statecov.x[1])) < _exclude_drones_targets_radius_) {

              ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: excluding objects from the map, another drone is too close");
              exclude_object = true;
            }
          }
        }
      }

      // remove this object from out map
      if (exclude_object) {
        it = object_list_.erase(it);
        continue;
      }
    }

    // exlude object from temporarly banned areas
    if (_allow_banning_areas_) {

      std::list<BannedArea_t>::iterator area_it;
      bool                              exclude_object = false;

      // go throught all banned areas
      {
        std::scoped_lock lock(mutex_banned_areas_);

        for (area_it = bannedAreaList.begin(); area_it != bannedAreaList.end(); area_it++) {

          // if it is up to date
          if ((ros::Time::now() - area_it->stamp).toSec() < ban_area_timeout_) {

            // if the object lies within the area
            if (mrs_lib::geometry::dist(vec2_t(area_it->x, area_it->y), vec2_t(obj_pointer->x, obj_pointer->y)) < ban_area_radius_) {

              exclude_object = true;
            }

          } else {

            // delete the banned area, since it has expired
            ROS_INFO("[BrickEstimation]: Removing banned area around x=%2.2f, y=%2.2f.", area_it->x, area_it->y);
            area_it = bannedAreaList.erase(area_it);
            continue;
          }
        }
      }

      // remove this object from the map
      if (exclude_object) {
        it = object_list_.erase(it);
        continue;
      }
    }

    if (obj_pointer->active) {

      // flipper bias

      if (obj_pointer->flipper_bias_negative > obj_pointer->flipper_bias_none) {

        obj_pointer->yaw -= M_PI / 2.0;
        obj_pointer->statecov.x[3] -= M_PI / 2.0;

        obj_pointer->flipper_bias_none     = obj_pointer->flipper_bias_negative;
        obj_pointer->flipper_bias_negative = 0;
        obj_pointer->flipper_bias_positive = 0;

      } else if (obj_pointer->flipper_bias_positive > obj_pointer->flipper_bias_none) {

        obj_pointer->yaw += M_PI / 2.0;
        obj_pointer->statecov.x[3] += M_PI / 2.0;

        obj_pointer->flipper_bias_none     = obj_pointer->flipper_bias_positive;
        obj_pointer->flipper_bias_negative = 0;
        obj_pointer->flipper_bias_positive = 0;
      }

      // kalman prediction

      try {

        obj_pointer->statecov = lkf_->predict(obj_pointer->statecov, Eigen::VectorXd::Zero(_n_inputs_), Q, 1.0 / _rate_);

        obj_pointer->x   = obj_pointer->statecov.x[0];
        obj_pointer->y   = obj_pointer->statecov.x[1];
        obj_pointer->z   = obj_pointer->statecov.x[2];
        obj_pointer->yaw = obj_pointer->statecov.x[3];

        // make the object inactive after some time
        if ((ros::Time::now() - obj_pointer->last_updated).toSec() > object_inactive_time) {

          obj_pointer->active = false;
          ROS_INFO("[BrickEstimation]: Deactivating object");
        }
      }
      catch (...) {

        ROS_WARN("[BrickEstimation]: Deleting the object from object_list_ based on an exception (prediction).");

        it = object_list_.erase(it);
        continue;
      }

    } else {

      // make the object dissapear after some time
      if ((ros::Time::now() - obj_pointer->last_updated).toSec() > object_delete_time) {

        ROS_INFO("[BrickEstimation]: Deleting an inactive object from object_list_.");
        it = object_list_.erase(it);
        continue;
      }
    }

    ++it;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* estimatorTimer() //{ */

void BrickEstimation::estimatorTimer([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  iterate();
  publish();
}

//}

/* mapTimer() //{ */

void BrickEstimation::mapTimer([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lock(mutex_object_list_);

  if (object_list_.empty()) {
    return;
  }

  auto map_frame = mrs_lib::get_mutexed(mutex_map_frame_, map_frame_);

  // | --------------- remove other drone targets --------------- |

  {
    std::list<ObjectHandler_t> old_object_list = object_list_;

    object_list_.clear();

    for (std::list<ObjectHandler_t>::iterator it1 = old_object_list.begin(); it1 != old_object_list.end(); it1++) {

      bool exclude_object = false;

      // exlude object beeing grasped by other drones
      if (_exclude_drones_targets_ && (it1->type >= OBJECT_RED && it1->type <= OBJECT_BLUE)) {

        std::scoped_lock lock(mutex_drones_diagnostics_);

        std::map<std::string, mbzirc_msgs::BrickGraspingDiagnostics>::iterator map_it;

        // go throught map full of other drones object lander diagnostics
        for (map_it = drones_diagnostics_.begin(); map_it != drones_diagnostics_.end(); map_it++) {

          // if it is new enought
          if ((ros::Time::now() - map_it->second.stamp).toSec() < _exclude_drones_targets_timeout_) {

            // if the drones lander is active
            if (map_it->second.active) {

              // if the drones target is too close
              if (mrs_lib::geometry::dist(vec2_t(map_it->second.target.position.x, map_it->second.target.position.y),
                                          vec2_t(it1->statecov.x[0], it1->statecov.x[1])) < _exclude_drones_targets_radius_) {

                ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: excluding objects from the map, other drone's target is too close");
                exclude_object = true;
              }

              // if the drone itself is too close
              if (mrs_lib::geometry::dist(vec2_t(map_it->second.position.position.x, map_it->second.position.position.y),
                                          vec2_t(it1->statecov.x[0], it1->statecov.x[1])) < _exclude_drones_targets_radius_) {

                ROS_WARN_THROTTLE(1.0, "[BrickEstimation]: excluding objects from the map, another drone is too close");
                exclude_object = true;
              }
            }
          }
        }
      }

      // remove this object from out map
      if (exclude_object) {
        ROS_INFO("[BrickEstimation]: one object excluded");
      } else {
        object_list_.push_back(*it1);
      }
    }
  }

  // | ----------------- merge walls in the map ----------------- |

  {

    ROS_INFO("[BrickEstimation]: sanitized map: size before %d", int(object_list_.size()));

    std::list<ObjectHandler_t>           old_object_list = object_list_;
    std::list<ObjectHandler_t>::iterator object_iter     = old_object_list.end();

    object_list_.clear();

    // find the closest object in the list of filters
    for (std::list<ObjectHandler_t>::iterator it1 = old_object_list.begin(); it1 != old_object_list.end(); it1++) {

      if (it1->type != OBJECT_WALL && it1->active) {

        object_list_.push_back(*it1);
        continue;
      }

      mbzirc_msgs::ObjectWithType obj1;

      obj1.len  = it1->len;
      obj1.x    = it1->statecov.x[0];
      obj1.y    = it1->statecov.x[1];
      obj1.z    = it1->statecov.x[2];
      obj1.yaw  = it1->statecov.x[3];
      obj1.type = it1->type;

      double new_len;
      object_iter = findClosestFusion(obj1, false, it1->type, new_len);

      if (object_iter != object_list_.end()) {

        if (it1->type != OBJECT_WALL) {

          double object_dist =
              mrs_lib::geometry::dist(vec2_t(it1->statecov.x[0], it1->statecov.x[1]), vec2_t(object_iter->statecov.x[0], object_iter->statecov.x[1]));

          if (object_dist >= _object_distance_thr_) {

            object_list_.push_back(*it1);
            continue;
          }
        }

        double alpha = pow(double(object_iter->n_corrections + 1) / (double(it1->n_corrections + 1) + double(object_iter->n_corrections + 1)), 2);

        object_iter->statecov.x[0] = alpha * object_iter->statecov.x[0] + (1 - alpha) * it1->statecov.x[0];
        object_iter->statecov.x[1] = alpha * object_iter->statecov.x[1] + (1 - alpha) * it1->statecov.x[1];
        object_iter->statecov.x[2] = alpha * object_iter->statecov.x[2] + (1 - alpha) * it1->statecov.x[2];
        object_iter->statecov.x[3] = sradians::interp(object_iter->statecov.x[3], it1->statecov.x[3], 1 - alpha);

        object_iter->len = alpha * object_iter->len + (1 - alpha) * it1->len;

        object_iter->x   = it1->statecov.x[0];
        object_iter->y   = it1->statecov.x[1];
        object_iter->z   = it1->statecov.x[2];
        object_iter->yaw = it1->statecov.x[3];

        object_iter->n_corrections += it1->n_corrections;

        if (it1->type == OBJECT_WALL) {
          ROS_INFO("[BrickEstimation]: merged two walls in the map together");
        } else {
          ROS_INFO("[BrickEstimation]: merged two bricks in the map together");
        }

      } else {

        object_list_.push_back(*it1);
      }
    }

    ROS_INFO("[BrickEstimation]: sanitized map: size after %d", int(object_list_.size()));
  }

  // | --------------------------- map -------------------------- |

  /* map for state machine//{ */

  if (publishing_map_) {

    mbzirc_msgs::BrickMap map_out;

    map_out.header.stamp    = ros::Time::now();
    map_out.header.frame_id = transformer_.resolveFrameName("gps_origin");

    auto res = transformer_.getTransform(map_frame, "gps_origin", ros::Time::now());

    if (res) {

      mrs_lib::TransformStamped tf;

      tf = res.value();

      // iterate over all objects
      for (std::list<ObjectHandler_t>::iterator it = object_list_.begin(); it != object_list_.end(); it++) {

        ObjectHandler_t *obj_pointer = &(*it);

        mrs_msgs::ReferenceStamped pose;
        pose.reference.position.x = obj_pointer->x;
        pose.reference.position.y = obj_pointer->y;
        pose.reference.position.z = obj_pointer->z - 0.2;
        pose.reference.heading    = obj_pointer->yaw;

        auto res = transformer_.transform(tf, pose);

        if (!res) {
          continue;
        }

        mbzirc_msgs::EstimatedBrick brick;

        brick.type = obj_pointer->type;

        brick.x   = res.value().reference.position.x;
        brick.y   = res.value().reference.position.y;
        brick.z   = res.value().reference.position.z;
        brick.yaw = res.value().reference.heading;

        geometry_msgs::Vector3Stamped covariance;
        covariance.header.stamp    = ros::Time::now();
        covariance.header.frame_id = map_frame;
        covariance.vector.x        = obj_pointer->statecov.P(0, 0);
        covariance.vector.y        = obj_pointer->statecov.P(1, 1);
        covariance.vector.z        = obj_pointer->statecov.P(2, 2);

        auto res2 = transformer_.transform(tf, covariance);

        if (!res2) {
          continue;
        }

        brick.len = obj_pointer->len;

        brick.last_seen  = obj_pointer->last_updated;
        brick.first_seen = obj_pointer->firs_seen;

        brick.n_corrections = obj_pointer->n_corrections > 255 ? 255 : obj_pointer->n_corrections;

        map_out.bricks.push_back(brick);
      }
    }

    try {
      pub_map_.publish(map_out);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", pub_map_.getTopic().c_str());
    }
  }

  //}
}

//}

/* debugTimer() //{ */

void BrickEstimation::debugTimer([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lock(mutex_object_list_);

  auto odometry_stable = mrs_lib::get_mutexed(mutex_odometry_, odometry_stable_);
  auto map_frame       = mrs_lib::get_mutexed(mutex_map_frame_, map_frame_);

  mbzirc_msgs::MbzircBrick brick;

  std::list<ObjectHandler_t>::iterator brick_ptr;

  // | ----------------------- closest red ---------------------- |

  /* closest red //{ */

  brick.header.stamp    = ros::Time::now();
  brick.header.frame_id = map_frame;

  // find the closest and active red brick
  brick_ptr = findClosest(odometry_stable.pose.position.x, odometry_stable.pose.position.y, true, OBJECT_RED);

  // if there are none, return
  if (brick_ptr == object_list_.end()) {

    brick.valid = false;

  } else {

    // fill up the state vector
    for (int i = 0; i < _n_states_; i++) {

      brick.states[i] = brick_ptr->statecov.x[i];
    }

    geometry_msgs::PointStamped point_stamped;

    point_stamped.header.stamp    = ros::Time::now();
    point_stamped.header.frame_id = map_frame;
    point_stamped.point.x         = brick_ptr->statecov.x[0];
    point_stamped.point.y         = brick_ptr->statecov.x[1];
    point_stamped.point.z         = brick_ptr->statecov.x[2];

    try {
      pub_red_closest_debugging_.publish(point_stamped);
    }
    catch (...) {
      ROS_ERROR("[BrickEstimation]: Exception caught during publishing topic %s.", pub_red_closest_debugging_.getTopic().c_str());
    }
  }

  //}

  // | ---------------------- closest green --------------------- |

  /* closest green //{ */

  // find the closest and active green brick
  brick_ptr = findClosest(odometry_stable.pose.position.x, odometry_stable.pose.position.y, true, OBJECT_GREEN);

  // if there are none, return
  if (brick_ptr == object_list_.end()) {

    brick.valid = false;

  } else {

    // fill up the state vector
    for (int i = 0; i < _n_states_; i++) {

      brick.states[i] = brick_ptr->statecov.x[i];
    }

    geometry_msgs::PointStamped point_stamped;

    point_stamped.header.stamp    = ros::Time::now();
    point_stamped.header.frame_id = map_frame;
    point_stamped.point.x         = brick_ptr->statecov.x[0];
    point_stamped.point.y         = brick_ptr->statecov.x[1];
    point_stamped.point.z         = brick_ptr->statecov.x[2];

    try {
      pub_green_closest_debugging_.publish(point_stamped);
    }
    catch (...) {
      ROS_ERROR("[BrickEstimation]: Exception caught during publishing topic %s.", pub_green_closest_debugging_.getTopic().c_str());
    }
  }

  //}

  // | ---------------------- closest blue ---------------------- |

  /* closest blue //{ */

  // find the closest and active blue brick
  brick_ptr = findClosest(odometry_stable.pose.position.x, odometry_stable.pose.position.y, tf::getYaw(odometry_stable.pose.orientation), OBJECT_BLUE);

  // if there are none, return
  if (brick_ptr == object_list_.end()) {

    brick.valid = false;

  } else {

    // fill up the state vector
    for (int i = 0; i < _n_states_; i++) {

      brick.states[i] = brick_ptr->statecov.x[i];
    }

    geometry_msgs::PointStamped point_stamped;

    point_stamped.header.stamp    = ros::Time::now();
    point_stamped.header.frame_id = map_frame;
    point_stamped.point.x         = brick_ptr->statecov.x[0];
    point_stamped.point.y         = brick_ptr->statecov.x[1];
    point_stamped.point.z         = brick_ptr->statecov.x[2];

    try {
      pub_blue_closest_debugging_.publish(point_stamped);
    }
    catch (...) {
      ROS_ERROR("[BrickEstimation]: Exception caught during publishing topic %s.", pub_blue_closest_debugging_.getTopic().c_str());
    }
  }

  //}

  // | ---------------------- closest wall ---------------------- |

  /* closest wall //{ */

  // find the closest and active wall
  brick_ptr = findClosest(odometry_stable.pose.position.x, odometry_stable.pose.position.y, true, OBJECT_WALL);

  // if there are none, return
  if (brick_ptr == object_list_.end()) {

    brick.valid = false;

  } else {

    // fill up the state vector
    for (int i = 0; i < _n_states_; i++) {

      brick.states[i] = brick_ptr->statecov.x[i];
    }

    geometry_msgs::PointStamped point_stamped;

    point_stamped.header.stamp    = ros::Time::now();
    point_stamped.header.frame_id = map_frame;
    point_stamped.point.x         = brick_ptr->statecov.x[0];
    point_stamped.point.y         = brick_ptr->statecov.x[1];
    point_stamped.point.z         = brick_ptr->statecov.x[2];

    try {
      pub_wall_closest_debugging_.publish(point_stamped);
    }
    catch (...) {
      ROS_ERROR("[BrickEstimation]: Exception caught during publishing topic %s.", pub_wall_closest_debugging_.getTopic().c_str());
    }
  }

  //}

  // | ----------------------- closest any ---------------------- |

  /* closest any //{ */

  // find the closest and active any
  brick_ptr = findClosest(odometry_stable.pose.position.x, odometry_stable.pose.position.y, true, OBJECT_ANY);

  // if there are none, return
  if (brick_ptr == object_list_.end()) {

    brick.valid = false;

  } else {

    // fill up the state vector
    for (int i = 0; i < _n_states_; i++) {

      brick.states[i] = brick_ptr->statecov.x[i];
    }

    geometry_msgs::PointStamped point_stamped;

    point_stamped.header.stamp    = ros::Time::now();
    point_stamped.header.frame_id = map_frame;
    point_stamped.point.x         = brick_ptr->statecov.x[0];
    point_stamped.point.y         = brick_ptr->statecov.x[1];
    point_stamped.point.z         = brick_ptr->statecov.x[2];

    try {
      pub_any_closest_debugging_.publish(point_stamped);
    }
    catch (...) {
      ROS_ERROR("[BrickEstimation]: Exception caught during publishing topic %s.", pub_any_closest_debugging_.getTopic().c_str());
    }
  }

  //}

  // | ----------------------- map markers ---------------------- |

  /* map markers //{ */

  tf::Quaternion tf_quat;

  visualizer_red_bricks_.clear();
  visualizer_green_bricks_.clear();
  visualizer_blue_bricks_.clear();
  visualizer_walls_.clear();
  visualizer_ground_patterns_.clear();

  int max_wall_corrections = 1;
  // find the highest wall correction count
  for (std::list<ObjectHandler_t>::iterator it = object_list_.begin(); it != object_list_.end(); it++) {

    if (it->type == OBJECT_WALL) {

      if (it->n_corrections > max_wall_corrections) {
        max_wall_corrections = it->n_corrections;
      }
    }
  }

  // iterate over all objects
  for (std::list<ObjectHandler_t>::iterator it = object_list_.begin(); it != object_list_.end(); it++) {

    ObjectHandler_t *obj_pointer = &(*it);

    Eigen::Vector3d pose;
    pose << obj_pointer->x, obj_pointer->y, obj_pointer->z - 0.2;

    tf_quat.setEuler(0, 0, obj_pointer->yaw);
    Eigen::Quaterniond orientation(tf_quat.w(), tf_quat.x(), tf_quat.y(), tf_quat.z());

    switch (obj_pointer->type) {

        /* RED //{ */

      case OBJECT_RED: {

        // show the brick

        Eigen::Vector3d scale;
        scale << 0.3, 0.2, 0.2;

        Eigen::Vector4d color;

        if (obj_pointer->active) {
          color << 1.0, 0.0, 0.0, 1.0;
        } else {
          color << 0.5, 0.5, 0.5, 1.0;
        }

        visualizer_red_bricks_.addCuboid(pose, orientation, scale, color);

        // show the white plate

        scale << 0.2, 0.15, 0.01;
        color << 1.0, 1.0, 1.0, 1.0;
        pose[2] += 0.205;

        visualizer_red_bricks_.addCuboid(pose, orientation, scale, color);

        break;
      }

        //}

        /* GREEN //{ */

      case OBJECT_GREEN: {

        Eigen::Vector3d scale;
        scale << 0.6, 0.2, 0.2;

        Eigen::Vector4d color;

        if (obj_pointer->active) {
          color << 0.0, 1.0, 0.0, 1.0;
        } else {
          color << 0.5, 0.5, 0.5, 1.0;
        }

        visualizer_green_bricks_.addCuboid(pose, orientation, scale, color);

        // show the white plate

        scale << 0.3, 0.15, 0.01;
        color << 1.0, 1.0, 1.0, 1.0;
        pose[2] += 0.205;

        visualizer_green_bricks_.addCuboid(pose, orientation, scale, color);

        break;
      }

        //}

        /* BLUE //{ */

      case OBJECT_BLUE: {

        Eigen::Vector3d scale;
        scale << 1.2, 0.2, 0.2;

        Eigen::Vector4d color;

        if (obj_pointer->active) {
          color << 0.0, 0.0, 1.0, 1.0;
        } else {
          color << 0.5, 0.5, 0.5, 1.0;
        }

        visualizer_blue_bricks_.addCuboid(pose, orientation, scale, color);

        // show the white plate

        scale << 0.3, 0.15, 0.01;
        color << 1.0, 1.0, 1.0, 1.0;
        pose[2] += 0.205;

        visualizer_blue_bricks_.addCuboid(pose, orientation, scale, color);

        break;
      }

        //}

        /* WALL //{ */

      case OBJECT_WALL: {

        Eigen::Vector3d scale;
        scale << obj_pointer->len, 0.2, 0.01;

        Eigen::Vector4d color;

        /* double grey_level; */
        double grey_level = 0.5 - 0.5 * (double(obj_pointer->n_corrections) / double(max_wall_corrections));
        /* if (obj_pointer->n_corrections >= 10) { */
        /*   grey_level = 0.0; */
        /* } else { */
        /*   grey_level = 0.75; */
        /* } */

        color << grey_level, grey_level, grey_level, 1.0;

        visualizer_walls_.addCuboid(pose, orientation, scale, color);

        break;
      }

        //}

        /* OBJECT_GROUND_PATTERN //{ */

      case OBJECT_GROUND_PATTERN: {

        Eigen::Vector3d scale;
        scale << 1.0, 1.0, 0.01;

        Eigen::Vector4d color;

        color << 0, 1.0, 1.0, 1;

        visualizer_ground_patterns_.addCuboid(pose, orientation, scale, color);

        break;
      }

        //}
    }
  }

  visualizer_red_bricks_.publish(100, true);
  visualizer_green_bricks_.publish(200, true);
  visualizer_blue_bricks_.publish(300, true);
  visualizer_walls_.publish(400, true);
  visualizer_ground_patterns_.publish(500, true);

  //}

  // show the statistics
  {
    std::scoped_lock lock(mutex_statistics_);

    ROS_INFO_THROTTLE(5.0, "[BrickEstimation]:");
    ROS_INFO_THROTTLE(
        5.0, "[BrickEstimation]: STATISTICS: [double detections: %d, smaller brick doubles: %d], [high alt: pi: %d, pi/2: %d], [low alt: pi: %d, pi/2: %d]",
        double_detections_, smaller_brick_double_detections_, pi_flipping_high_, pi_2_flipping_high_, pi_flipping_low_, pi_2_flipping_low_);
    ROS_INFO_THROTTLE(5.0, "[BrickEstimation]:");
  }
}

//}

}  // namespace brick_estimation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(brick_estimation::BrickEstimation, nodelet::Nodelet)
