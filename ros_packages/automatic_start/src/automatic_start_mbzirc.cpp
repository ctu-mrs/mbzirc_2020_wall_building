#define VERSION "0.0.5.0"

/* includes //{ */

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mutex>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandBool.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <mrs_msgs/SetInt.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ValidateReference.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/CameraInfo.h>

//}

/* defines //{ */

#define PWM_LOW_THIRD 1250
#define PWM_HIGH_THIRD 1750

//}

namespace automatic_start_mbzirc
{

/* class AutomaticStartMbzirc //{ */

// state machine
typedef enum
{

  STATE_IDLE,
  STATE_TAKEOFF,
  STATE_IN_ACTION,
  STATE_LAND,
  STATE_FINISHED

} LandingStates_t;

const char* state_names[5] = {

    "IDLING", "TAKING OFF", "IN ACTION", "LANDING", "FINISHED"};

class AutomaticStartMbzirc : public nodelet::Nodelet {

public:
  virtual void onInit();
  std::string  _version_;

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  bool            _simulation_    = false;
  std::string     _challenge_     = "";

private:
  double _safety_timeout_;

private:
  bool gotSensors(void);

  bool            _check_bluefox_1_;
  ros::Subscriber subscriber_bluefox_1_;
  ros::Time       bluefox_1_last_time_;
  std::mutex      mutex_bluefox_1_;
  void            callbackBluefox1(const sensor_msgs::CameraInfoConstPtr& msg);
  bool            gotBluefox1(void);

  bool            _check_bluefox_2_;
  ros::Subscriber subscriber_bluefox_2_;
  ros::Time       bluefox_2_last_time_;
  std::mutex      mutex_bluefox_2_;
  void            callbackBluefox2(const sensor_msgs::CameraInfoConstPtr& msg);
  bool            gotBluefox2(void);

  bool            _check_realsense_;
  ros::Subscriber subscriber_realsense_;
  ros::Time       realsense_last_time_;
  std::mutex      mutex_realsense_;
  void            callbackRealsense(const sensor_msgs::CameraInfoConstPtr& msg);
  bool            gotRealsense(void);

private:
  ros::ServiceClient service_client_motors_;
  ros::ServiceClient service_client_arm_;
  ros::ServiceClient service_client_takeoff_;
  ros::ServiceClient service_client_land_home_;
  ros::ServiceClient service_client_land_there_;
  ros::ServiceClient service_client_land_;
  ros::ServiceClient service_client_eland_;
  ros::ServiceClient service_client_validate_reference_;

  ros::ServiceClient service_client_start_;
  ros::ServiceClient service_client_stop_;

  ros::ServiceServer service_server_shutdown_;

private:
  ros::Subscriber subscriber_mavros_state_;
  ros::Subscriber subscriber_rc_;
  ros::Subscriber subscriber_control_manager_diagnostics_;
  ros::Subscriber subscriber_dropoff_pose_;

private:
  double      _shutdown_timeout_;
  std::string _scripts_path_;
  bool        callbackShutdown(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

private:
  void                       callbackDropoffPose(const geometry_msgs::PoseStampedConstPtr& msg);
  geometry_msgs::PoseStamped dropoff_pose_;
  std::mutex                 mutex_dropoff_pose_;
  bool                       got_dropoff_pose_ = false;

private:
  ros::Timer main_timer_;
  void       mainTimer(const ros::TimerEvent& event);
  double     main_timer_rate_;

private:
  ros::Timer shutdown_timer_;
  ros::Time  shutdown_time_;
  void       shutdownTimer(const ros::TimerEvent& event);

private:
  void callbackRC(const mavros_msgs::RCInConstPtr& msg);
  bool got_rc_channels_ = false;
  int  _channel_number_;

  int        rc_mode_ = -1;
  std::mutex mutex_rc_mode_;

  int        start_mode_ = -1;
  std::mutex mutex_start_mode_;

private:
  void       callbackMavrosState(const mavros_msgs::StateConstPtr& msg);
  bool       got_mavros_state_ = false;
  std::mutex mutex_mavros_state_;

  ros::Time armed_time_;
  bool      armed_ = false;

  ros::Time offboard_time_;
  bool      offboard_ = false;

private:
  void                                callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  std::mutex                          mutex_control_manager_diagnostics_;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_;
  bool                                got_control_manager_diagnostics_ = false;

private:
  bool takeoff();

  bool landImpl();
  bool elandImpl();
  bool landHomeImpl();
  bool landThereImpl();
  bool land();
  void validateReference();

  bool setMotors(const bool value);
  bool disarm();
  bool start(const int value);
  bool stop();

private:
  ros::Time start_time_;

  double      _action_duration_;
  double      _pre_takeoff_sleep_;
  bool        _handle_landing_ = false;
  bool        _handle_takeoff_ = false;
  std::string _land_mode_;

private:
  int _start_n_attempts_;
  int call_attempt_counter_ = 0;

private:
  uint current_state = STATE_IDLE;
  void changeState(LandingStates_t new_state);
};

//}

/* onInit() //{ */

void AutomaticStartMbzirc::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  bluefox_1_last_time_ = ros::Time(0);
  bluefox_2_last_time_ = ros::Time(0);
  realsense_last_time_ = ros::Time(0);

  armed_      = false;
  armed_time_ = ros::Time(0);

  offboard_      = false;
  offboard_time_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "AutomaticStartMbzirc");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[AutomaticStartMbzirc]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("safety_timeout", _safety_timeout_);
  param_loader.loadParam("main_timer_rate", main_timer_rate_);
  param_loader.loadParam("simulation", _simulation_);
  param_loader.loadParam("CHALLENGE", _challenge_);
  param_loader.loadParam("channel_number", _channel_number_);
  param_loader.loadParam("call_n_attempts", _start_n_attempts_);

  param_loader.loadParam("challenges/" + _challenge_ + "/land_mode", _land_mode_);
  param_loader.loadParam("challenges/" + _challenge_ + "/handle_landing", _handle_landing_);
  param_loader.loadParam("challenges/" + _challenge_ + "/handle_takeoff", _handle_takeoff_);
  param_loader.loadParam("challenges/" + _challenge_ + "/action_duration", _action_duration_);
  param_loader.loadParam("challenges/" + _challenge_ + "/pre_takeoff_sleep", _pre_takeoff_sleep_);

  param_loader.loadParam("scripts_path", _scripts_path_);
  param_loader.loadParam("shutdown_timeout", _shutdown_timeout_);

  // recaltulate the acion duration to seconds
  _action_duration_ *= 60;

  // applies only in simulation
  param_loader.loadParam("rc_mode", rc_mode_);

  if (!(_land_mode_ == "land_home" || _land_mode_ == "land" || _land_mode_ == "eland" || _land_mode_ == "land_there")) {

    ROS_ERROR("[MavrosInterface]: land_mode (\"%s\") was specified wrongly, will eland by default!!!", _land_mode_.c_str());
  }

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MavrosInterface]: Could not load all parameters!");
    ros::shutdown();
  }

  // --------------------------------------------------------------
  // |                     sensors subscriber                     |
  // --------------------------------------------------------------

  param_loader.loadParam("check_bluefox1", _check_bluefox_1_);
  subscriber_bluefox_1_ = nh_.subscribe("bluefox1_in", 1, &AutomaticStartMbzirc::callbackBluefox1, this, ros::TransportHints().tcpNoDelay());

  param_loader.loadParam("check_bluefox2", _check_bluefox_2_);
  subscriber_bluefox_2_ = nh_.subscribe("bluefox2_in", 1, &AutomaticStartMbzirc::callbackBluefox2, this, ros::TransportHints().tcpNoDelay());

  param_loader.loadParam("check_realsense", _check_realsense_);
  subscriber_realsense_ = nh_.subscribe("realsense_in", 1, &AutomaticStartMbzirc::callbackRealsense, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_mavros_state_ = nh_.subscribe("mavros_state_in", 1, &AutomaticStartMbzirc::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics_ =
      nh_.subscribe("control_manager_diagnostics_in", 1, &AutomaticStartMbzirc::callbackControlManagerDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_rc_           = nh_.subscribe("rc_in", 1, &AutomaticStartMbzirc::callbackRC, this, ros::TransportHints().tcpNoDelay());
  subscriber_dropoff_pose_ = nh_.subscribe("dropoff_pose_in", 1, &AutomaticStartMbzirc::callbackDropoffPose, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                       service clients                      |
  // --------------------------------------------------------------

  service_client_takeoff_    = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_land_home_  = nh_.serviceClient<std_srvs::Trigger>("land_home_out");
  service_client_land_there_ = nh_.serviceClient<mrs_msgs::ReferenceStampedSrv>("land_there_out");
  service_client_land_       = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_eland_      = nh_.serviceClient<std_srvs::Trigger>("eland_out");
  service_client_motors_     = nh_.serviceClient<std_srvs::SetBool>("motors_out");
  service_client_arm_        = nh_.serviceClient<mavros_msgs::CommandBool>("arm_out");

  service_client_validate_reference_ = nh_.serviceClient<mrs_msgs::ValidateReference>("validate_reference_out");

  if (_challenge_ == "balloons") {

    service_client_start_ = nh_.serviceClient<mrs_msgs::SetInt>("start_out");

  } else if (_challenge_ == "ball") {

    service_client_start_ = nh_.serviceClient<mrs_msgs::SetInt>("start_out");

  } else if (_challenge_ == "fire") {

    service_client_start_ = nh_.serviceClient<mrs_msgs::SetInt>("start_out");

  } else if (_challenge_ == "fire_indoor") {

    service_client_start_ = nh_.serviceClient<mrs_msgs::SetInt>("start_out");

  } else if (_challenge_ == "blanket") {

    service_client_start_ = nh_.serviceClient<mrs_msgs::SetInt>("start_out");

  } else if (_challenge_ == "wall") {

    service_client_start_ = nh_.serviceClient<mrs_msgs::SetInt>("start_out");
  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: FATAL in onInit(): the challenge name is probably wrong");
    ros::shutdown();
  }

  service_client_stop_ = nh_.serviceClient<std_srvs::Trigger>("stop_out");

  // --------------------------------------------------------------
  // |                       service servers                      |
  // --------------------------------------------------------------

  service_server_shutdown_ = nh_.advertiseService("shutdown_in", &AutomaticStartMbzirc::callbackShutdown, this);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer_     = nh_.createTimer(ros::Rate(main_timer_rate_), &AutomaticStartMbzirc::mainTimer, this);
  shutdown_timer_ = nh_.createTimer(ros::Rate(1.0), &AutomaticStartMbzirc::shutdownTimer, this, false, false);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackMavrosState() //{ */

void AutomaticStartMbzirc::callbackMavrosState(const mavros_msgs::StateConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStartMbzirc]: getting mavros state");

  std::scoped_lock lock(mutex_mavros_state_);

  // check armed_ state
  if (armed_ == false) {

    // if armed_ state changed to true, please "start the clock"
    if (msg->armed > 0) {

      armed_      = true;
      armed_time_ = ros::Time::now();
    }

    // if we were armed_ previously
  } else if (armed_ == true) {

    // and we are not really now
    if (msg->armed == 0) {

      armed_ = false;
    }
  }

  // check offboard_ state
  if (offboard_ == false) {

    // if offboard_ state changed to true, please "start the clock"
    if (msg->mode == "OFFBOARD") {

      offboard_      = true;
      offboard_time_ = ros::Time::now();
    }

    // if we were in offboard_ previously
  } else if (offboard_ == true) {

    // and we are not really now
    if (msg->mode != "OFFBOARD") {

      offboard_ = false;
    }
  }

  got_mavros_state_ = true;
}

//}

/* callbackControlManagerDiagnostics() //{ */

void AutomaticStartMbzirc::callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStartMbzirc]: getting control manager diagnostics");

  {
    std::scoped_lock lock(mutex_control_manager_diagnostics_);

    control_manager_diagnostics_ = *msg;

    got_control_manager_diagnostics_ = true;
  }
}

//}

/* //{ callbackRC() */

void AutomaticStartMbzirc::callbackRC(const mavros_msgs::RCInConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStartMbzirc]: getting RC channels");

  std::scoped_lock lock(mutex_rc_mode_);

  if (uint(_channel_number_) >= msg->channels.size()) {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: RC joystick activation channel number (%d) is out of range [0-%d]", uint(_channel_number_),
                       uint(msg->channels.size()));

  } else {

    // detect the switch of a switch on the RC
    if (msg->channels[_channel_number_] < PWM_LOW_THIRD) {

      rc_mode_ = 0;

    } else if ((msg->channels[_channel_number_] >= PWM_LOW_THIRD) && (msg->channels[_channel_number_] <= PWM_HIGH_THIRD)) {

      rc_mode_ = 1;

    } else if (msg->channels[_channel_number_] > PWM_HIGH_THIRD) {

      rc_mode_ = 2;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: the RC channel value is outside of the set of possible values");
    }
  }

  got_rc_channels_ = true;
}

//}

/* //{ callbackDropoffPose() */

void AutomaticStartMbzirc::callbackDropoffPose(const geometry_msgs::PoseStampedConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStartMbzirc]: getting dropoffpose");

  std::scoped_lock lock(mutex_dropoff_pose_);

  dropoff_pose_ = *msg;

  got_dropoff_pose_ = true;
}

//}

/* callbackBluefox1() //{ */

void AutomaticStartMbzirc::callbackBluefox1([[maybe_unused]] const sensor_msgs::CameraInfoConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStartMbzirc]: getting bluefox 1");

  std::scoped_lock lock(mutex_bluefox_1_);

  bluefox_1_last_time_ = ros::Time::now();
}

//}

/* callbackBluefox2() //{ */

void AutomaticStartMbzirc::callbackBluefox2([[maybe_unused]] const sensor_msgs::CameraInfoConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStartMbzirc]: getting bluefox 2");

  std::scoped_lock lock(mutex_bluefox_2_);

  bluefox_2_last_time_ = ros::Time::now();
}

//}

/* callbackRealsense() //{ */

void AutomaticStartMbzirc::callbackRealsense([[maybe_unused]] const sensor_msgs::CameraInfoConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStartMbzirc]: getting realsense");

  std::scoped_lock lock(mutex_realsense_);

  realsense_last_time_ = ros::Time::now();
}

//}

/* callbackShutdown() //{ */

bool AutomaticStartMbzirc::callbackShutdown([[maybe_unused]] std_srvs::Trigger::Request& req, [[maybe_unused]] std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  if (_challenge_ == "fire_indoor") {

    shutdown_time_ = ros::Time::now();
    shutdown_timer_.start();

    res.success = true;
    res.message = "shutting down";

  } else {

    res.success = false;
    res.message = "only for the fire_indoor challenge";
  }

  return true;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* mainTimer() //{ */

void AutomaticStartMbzirc::mainTimer([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!got_control_manager_diagnostics_ || !got_mavros_state_ || !(got_rc_channels_ || _simulation_)) {
    ROS_WARN_THROTTLE(5.0, "[AutomaticStartMbzirc]: waiting for data: ControManager=%s, Mavros=%s, RC=%s", got_control_manager_diagnostics_ ? "true" : "FALSE",
                      got_mavros_state_ ? "true" : "FALSE", got_rc_channels_ ? "true" : "FALSE");
    return;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_mavros_state_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);
  auto rc_mode                                      = mrs_lib::get_mutexed(mutex_rc_mode_, rc_mode_);

  bool   motors           = control_manager_diagnostics.motors;
  double time_from_arming = (ros::Time::now() - armed_time).toSec();

  if (gotSensors()) {
    ROS_INFO_ONCE("[AutomaticStartMbzirc]: GOT ALL SENSORS, I AM HAPPY!");
  }

  switch (current_state) {

    case STATE_IDLE: {

      validateReference();

      if (armed && !motors) {

        if (!gotSensors()) {

          ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: cannot set motors ON, missing sensors!");

        } else {

          double res = setMotors(true);

          if (!res) {
            ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: could not set motors ON");
          }
        }

        if (time_from_arming > 1.5) {

          ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: could not set motors ON for 1.5 secs, disarming");
          disarm();
        }
      }

      // when armed and in offboard, takeoff
      if (armed && offboard && motors) {

        // sae the current rc mode, so it can be later used for start()
        mrs_lib::set_mutexed(mutex_start_mode_, rc_mode, start_mode_);

        double armed_time_diff    = (ros::Time::now() - armed_time).toSec();
        double offboard_time_diff = (ros::Time::now() - offboard_time).toSec();

        if ((armed_time_diff > _safety_timeout_) && (offboard_time_diff > _safety_timeout_)) {

          if (_handle_takeoff_) {
            changeState(STATE_TAKEOFF);
          } else {
            changeState(STATE_IN_ACTION);
          }

        } else {

          double min = (armed_time_diff < offboard_time_diff) ? armed_time_diff : offboard_time_diff;

          ROS_WARN_THROTTLE(1.0, "Starting in %1.0f", (_safety_timeout_ - min));
        }
      }

      break;
    }

    case STATE_TAKEOFF: {

      std::scoped_lock lock(mutex_control_manager_diagnostics_);

      // if takeoff finished
      if (control_manager_diagnostics.active_tracker == "MpcTracker" && control_manager_diagnostics.tracker_status.callbacks_enabled) {

        ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: takeoff finished");

        changeState(STATE_IN_ACTION);
      }

      break;
    }

    case STATE_IN_ACTION: {

      if (_handle_landing_) {

        double in_action_time = (ros::Time::now() - start_time_).toSec();

        ROS_INFO_THROTTLE(5.0, "[AutomaticStartMbzirc]: in action for %.0f second out of %.0f", in_action_time, _action_duration_);

        if (in_action_time > _action_duration_) {

          ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: the action duration time has passed, landing");

          changeState(STATE_LAND);
        }

      } else {

        changeState(STATE_FINISHED);
      }

      break;
    }

    case STATE_LAND: {

      if (!armed || !offboard || !motors) {

        ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: the UAV has probably landed");

        changeState(STATE_FINISHED);
      }

      break;
    }

    case STATE_FINISHED: {

      ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: we are done here");

      break;
    }
  }
}

//}

/* shutdownTimer() //{ */

void AutomaticStartMbzirc::shutdownTimer([[maybe_unused]] const ros::TimerEvent& event) {

  double time_diff = (ros::Time::now() - shutdown_time_).toSec();

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: shutting down in %d s", int(_shutdown_timeout_ - time_diff));

  if (time_diff > _shutdown_timeout_) {

    ROS_INFO("[AutomaticStartMbzirc]: calling for shutdown");
    [[maybe_unused]] int res = system((_scripts_path_ + std::string("/shutdown.sh")).c_str());
  }
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* changeState() //{ */

void AutomaticStartMbzirc::changeState(LandingStates_t new_state) {

  auto start_mode = mrs_lib::get_mutexed(mutex_start_mode_, start_mode_);

  ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: switching states %s -> %s", state_names[current_state], state_names[new_state]);

  switch (new_state) {

    case STATE_IDLE: {

      break;
    }

    case STATE_TAKEOFF: {

      if (_pre_takeoff_sleep_ > 1.0) {
        ROS_INFO("[AutomaticStartMbzirc]: sleeping for %.2f secs before takeoff", _pre_takeoff_sleep_);
        ros::Duration(_pre_takeoff_sleep_).sleep();
      }

      bool res = takeoff();

      if (!res) {
        return;
      }

      break;
    }

    case STATE_IN_ACTION: {

      bool res = start(start_mode);

      if (!res) {

        if (++call_attempt_counter_ < _start_n_attempts_) {

          ROS_WARN("[AutomaticStartMbzirc]: failed to call start, attempting again");
          return;

        } else {

          ROS_ERROR("[AutomaticStartMbzirc]: failed to call start for the %dth time, giving up", call_attempt_counter_);
        }
      }

      call_attempt_counter_ = 0;

      start_time_ = ros::Time::now();

      break;
    }

    case STATE_LAND: {

      {
        bool res = stop();

        if (++call_attempt_counter_ < _start_n_attempts_) {

          ROS_WARN("[AutomaticStartMbzirc]: failed to call stop, attempting again");

          if (!res) {
            return;
          }

        } else {

          ROS_ERROR("[AutomaticStartMbzirc]: failed to call stop for the %dth time, giving up", call_attempt_counter_);
        }
      }

      call_attempt_counter_ = 0;

      {
        bool res = land();

        if (!res) {
          return;
        }
      }

      break;
    }

    case STATE_FINISHED: {

      break;
    }

    break;
  }

  current_state = new_state;
}

//}

/* takeoff() //{ */

bool AutomaticStartMbzirc::takeoff() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: taking off");

  std_srvs::Trigger srv;

  bool res = service_client_takeoff_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: taking off failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for taking off failed");
  }

  return false;
}

//}

/* landHomeImpl() //{ */

bool AutomaticStartMbzirc::landHomeImpl() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: landing home");

  std_srvs::Trigger srv;

  bool res = service_client_land_home_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: landing home failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for landing home failed");
  }

  return false;
}

//}

/* landThereImpl() //{ */

bool AutomaticStartMbzirc::landThereImpl() {

  auto dropoff_pose = mrs_lib::get_mutexed(mutex_dropoff_pose_, dropoff_pose_);

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: landing there (in the dropoff zone)");

  mrs_msgs::ReferenceStampedSrv srv;

  srv.request.header.stamp    = ros::Time::now();
  srv.request.header.frame_id = dropoff_pose.header.frame_id;

  srv.request.reference.position   = dropoff_pose.pose.position;
  srv.request.reference.position.z = 3.0;
  srv.request.reference.heading    = tf::getYaw(dropoff_pose.pose.orientation);

  bool res = service_client_land_there_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: landing in the dropoff zone failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for landing in the dropoff zone failed");
  }

  return false;
}

//}

/* landImpl() //{ */

bool AutomaticStartMbzirc::landImpl() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: landing");

  std_srvs::Trigger srv;

  bool res = service_client_land_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: landing failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for landing failed");
  }

  return false;
}

//}

/* elandImpl() //{ */

bool AutomaticStartMbzirc::elandImpl() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: elanding");

  std_srvs::Trigger srv;

  bool res = service_client_eland_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: elanding failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for elanding failed");
  }

  return false;
}

//}

/* land() //{ */

bool AutomaticStartMbzirc::land() {

  bool res;

  if (_land_mode_ == "land") {

    res = landImpl();

  } else if (_land_mode_ == "land_home") {

    res = landHomeImpl();

  } else if (_land_mode_ == "land_there") {

    if (got_dropoff_pose_) {

      res = landThereImpl();

    } else {

      ROS_ERROR("[AutomaticStartMbzirc]: missing dropoff pose, landing home");
      res = landHomeImpl();
    }

  } else {

    res = elandImpl();
  }

  return res;
}

//}

/* validateReference() //{ */

void AutomaticStartMbzirc::validateReference() {

  mrs_msgs::ValidateReference srv_out;

  srv_out.request.reference.header.frame_id      = "fcu";
  srv_out.request.reference.reference.position.z = 3.0;

  bool res = service_client_validate_reference_.call(srv_out);

  if (res) {

    if (srv_out.response.success) {

      ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: current pos is inside of the safety area");

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: the current pos is outside of the safety area!");
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: current pos could not be validated");
  }
}

//}

/* motors() //{ */

bool AutomaticStartMbzirc::setMotors(const bool value) {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: setting motors %s", value ? "ON" : "OFF");

  std_srvs::SetBool srv;
  srv.request.data = value;

  bool res = service_client_motors_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: setting motors failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for setting motors failed");
  }

  return false;
}

//}

/* disarm() //{ */

bool AutomaticStartMbzirc::disarm() {

  if (!got_mavros_state_) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: cannot not disarm, missing mavros state!");

    return false;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_mavros_state_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);

  if (offboard) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: cannot not disarm, not in offboard mode!");

    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: disarming");

  mavros_msgs::CommandBool srv;
  srv.request.value = 0;

  bool res = service_client_arm_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: disarming failed");
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for disarming failed");
  }

  return false;
}

//}

/* start() //{ */

bool AutomaticStartMbzirc::start(const int value) {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: starting action, mode %d", value);

  if (_challenge_ == "balloons") {

    mrs_msgs::SetInt srv;
    srv.request.value = value;

    bool res = service_client_start_.call(srv);

    if (res) {

      if (srv.response.success) {

        return true;

      } else {

        ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: starting action failed failed: %s", srv.response.message.c_str());
      }

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for starting action failed");
    }

  } else if (_challenge_ == "ball") {

    mrs_msgs::SetInt srv;
    srv.request.value = value;

    bool res = service_client_start_.call(srv);

    if (res) {

      if (srv.response.success) {

        return true;

      } else {

        ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: starting action failed failed: %s", srv.response.message.c_str());
      }

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for starting action failed");
    }

  } else if (_challenge_ == "fire") {

    mrs_msgs::SetInt srv;
    srv.request.value = value;

    bool res = service_client_start_.call(srv);

    if (res) {

      if (srv.response.success) {

        return true;

      } else {

        ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: starting action failed failed: %s", srv.response.message.c_str());
      }

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for starting action failed");
    }

  } else if (_challenge_ == "fire_indoor") {

    mrs_msgs::SetInt srv;
    srv.request.value = value;

    bool res = service_client_start_.call(srv);

    if (res) {

      if (srv.response.success) {

        return true;

      } else {

        ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: starting action failed failed: %s", srv.response.message.c_str());
      }

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for starting action failed");
    }

  } else if (_challenge_ == "blanket") {

    mrs_msgs::SetInt srv;
    srv.request.value = value;

    bool res = service_client_start_.call(srv);

    if (res) {

      if (srv.response.success) {

        return true;

      } else {

        ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: starting action failed failed: %s", srv.response.message.c_str());
      }

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for starting action failed");
    }

  } else if (_challenge_ == "wall") {

    mrs_msgs::SetInt srv;
    srv.request.value = value;

    bool res = service_client_start_.call(srv);

    if (res) {

      if (srv.response.success) {

        return true;

      } else {

        ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: starting action failed failed: %s", srv.response.message.c_str());
      }

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for starting action failed");
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: FATAL in start(): the challenge name is wrong");
  }

  return false;
}

//}

/* stop() //{ */

bool AutomaticStartMbzirc::stop() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStartMbzirc]: stopping action");

  std_srvs::Trigger srv;

  bool res = service_client_stop_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: stopping action failed failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStartMbzirc]: service call for stopping action failed");
  }

  return false;
}

//}

/* gotBluefox1() //{ */

bool AutomaticStartMbzirc::gotBluefox1(void) {

  if (!_check_bluefox_1_) {
    return true;
  }

  if ((ros::Time::now() - bluefox_1_last_time_).toSec() < 1.0) {

    return true;

  } else {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: missing bluefox 1");
    return false;
  }
}

//}

/* gotBluefox2() //{ */

bool AutomaticStartMbzirc::gotBluefox2(void) {

  if (!_check_bluefox_2_) {
    return true;
  }

  if ((ros::Time::now() - bluefox_2_last_time_).toSec() < 1.0) {

    return true;
  } else {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: missing bluefox 2");
    return false;
  }
}

//}

/* gotRealsense() //{ */

bool AutomaticStartMbzirc::gotRealsense(void) {

  if (!_check_realsense_) {
    return true;
  }

  if ((ros::Time::now() - realsense_last_time_).toSec() < 1.0) {

    return true;
  } else {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStartMbzirc]: missing realsense");
    return false;
  }
}

//}

/* gotSensors() //{ */

bool AutomaticStartMbzirc::gotSensors(void) {

  if (!gotBluefox1()) {
    return false;
  }

  if (!gotBluefox2()) {
    return false;
  }

  if (!gotRealsense()) {
    return false;
  }

  return true;
}

//}

}  // namespace automatic_start_mbzirc

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(automatic_start_mbzirc::AutomaticStartMbzirc, nodelet::Nodelet)
