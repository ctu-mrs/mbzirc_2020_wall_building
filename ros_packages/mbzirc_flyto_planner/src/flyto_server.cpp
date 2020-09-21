/* author: Vojtech Spurny*/

/* includes //{ */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mbzirc_flyto_planner/FlyToAction.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <std_srvs/Trigger.h>
#include <plan_keeper/PlanDiagnostics.h>
#include <nav_msgs/Odometry.h>
#include <limits>
#include <math.h>
#include <vector>
#include <thread>
#include <mutex>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

//}

#define STRING_EQUAL 0

/* Class FlyToAction definition //{ */

class FlyToAction {
public:
  FlyToAction(const std::string name);
  ~FlyToAction(void) {
  }

private:
  /* config parameters */
  size_t index_in_desired_trajectory_;
  double detected_radius_;
  double safety_radius_;
  double velocity_;
  double desired_altitude_;
  double allowed_error_in_position_;
  double time_of_valid_msg_;
  bool   debug_;
  bool   debug_all_solutions_;
  int    length_of_horizon_;
  int    rate_;
  double goal_reach_distance_;

private:
  ros::NodeHandle                                                  nh_;
  actionlib::SimpleActionServer<mbzirc_flyto_planner::FlyToAction> as_;
  std::string                                                      action_name_;
  mbzirc_flyto_planner::FlyToFeedback                              feedback_;
  mbzirc_flyto_planner::FlyToResult                                result_;
  mrs_msgs::Reference                                              goal_;
  bool                                                             use_yaw_;
  bool                                                             plan_in_loop_;

  double time_stamp_;
  bool   was_previous_trajectory_without_dangerous_areas_;

  mrs_msgs::TrajectoryReference    trajectory_;
  std::vector<mrs_msgs::Reference> desired_trajectory_;
  std::vector<mrs_msgs::Reference> dangerous_points_;

  // server state
  bool first_trajectory_sended_;

  // thread
  std::thread action_thread_;

  // mutexes
  std::mutex server_state_mutex_;

  // service definition
  ros::ServiceClient service_client_trajectory_;
  ros::ServiceClient service_trigger_hover_;

  // subscriber definition
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>           sub_this_uav_odom_handler_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>           sub_uav1_odom_handler_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>           sub_uav2_odom_handler_;
  mrs_lib::SubscribeHandler<plan_keeper::PlanDiagnostics> sub_uav1_plan_diagnostics_handler_;
  mrs_lib::SubscribeHandler<plan_keeper::PlanDiagnostics> sub_uav2_plan_diagnostics_handler_;
  // mrs_lib::SubscribeHandlerPtr<mrs_msgs::MpcTrackerDiagnostics> sub_this_uav_tracker_diagnostics_handler_;

  // | ------------------- callbacks ------------------- |
  // callback for accepting goal of action server
  void goalCB();
  // callback for preemption of action server
  void preemptCB();

  // | ------------------- member functions ------------------- |

  // function which sample circle between two angles
  void getSampledCircle(const mrs_msgs::Reference &p_circle, const double radius, const double alpha, const double beta,
                        std::vector<mrs_msgs::Reference> &transition_points, const double sampling_step, const bool positive_direction);
  // function which sample trajectory between two points
  void getSampledLineSegment(const mrs_msgs::Reference &p_a, const mrs_msgs::Reference &p_b, std::vector<mrs_msgs::Reference> &trajectory,
                             double &remaining_dist);
  // function which sample trajectory between transtion points
  void getSampledTrajectory(const std::vector<mrs_msgs::Reference> &transition_points, std::vector<mrs_msgs::Reference> &trajectory, const bool loop);
  // functions which returns parameter "t". Nearest point on line segment to point can be found as [x_t,y_t] = p_a + t * (p_b - p_a)
  double getNearestPointLineSegment(const mrs_msgs::Reference &p_a, const mrs_msgs::Reference &p_b, const mrs_msgs::Reference &p_x);
  // function fill list of points which are intersection of tangent of circle and point - list_a[0], list_b[0] - ...
  void getTangentCirclePoint(const mrs_msgs::Reference &p_c, const double radius_c, const mrs_msgs::Reference &p_p, std::vector<mrs_msgs::Reference> &list_a,
                             std::vector<mrs_msgs::Reference> &list_b);
  // function fill list of points which are intersection of inner tangent of two circles - list_a[0], list_b[0] - ...
  void innerTangentTwoCircles(const mrs_msgs::Reference &p_a, const double radius_a, const mrs_msgs::Reference &p_b, const double radius_b,
                              std::vector<mrs_msgs::Reference> &list_a, std::vector<mrs_msgs::Reference> &list_b);
  // function fill list of points which are intersection of outer tangent of two circles - list_a[0], list_b[0] - ...
  void outerTangentTwoCircles(const mrs_msgs::Reference &p_a, const double radius_a, const mrs_msgs::Reference &p_b, const double radius_b,
                              std::vector<mrs_msgs::Reference> &list_a, std::vector<mrs_msgs::Reference> &list_b);
  // function which returns distance between two points
  double distanceTwoPoints(const mrs_msgs::Reference &p_a, const mrs_msgs::Reference &p_b);
  // function which returns distance between two points in 3d
  double distanceTwoPoints3d(const mrs_msgs::Reference &p_a, const mrs_msgs::Reference &p_b);
  // function which find nearest point "p_nearest" and distance "dist" to this point to couple line segment-Point
  void findNearestPoint(const mrs_msgs::Reference &p_a, const mrs_msgs::Reference &p_b, const mrs_msgs::Reference &p_x, mrs_msgs::Reference &p_nearest,
                        double &dist);
  // functions which returns parameter "t". Nearest point on line segment to circle can be found as [x_t,y_t] = p_a + t * (p_b - p_a)
  double getIntersectionCircleLineSegment(const mrs_msgs::Reference &p_a, const mrs_msgs::Reference &p_b, const mrs_msgs::Reference &p_circle,
                                          const double radius);
  // function which returns smallest difference between two angles
  // parameter positive_direction is telling in which direction the angle should be count
  double distanceBetweenTwoAngles(const double alpha, const double beta, const bool positive_direction);
  // function which returns nearest distance to dangerous zone
  double distanceFromAreaToTrajectory(const std::vector<mrs_msgs::Reference> &transition_points);
  // function which generates transitions points of trajectory
  void generateTransititonPoints(const mrs_msgs::Reference start, const mrs_msgs::Reference goal, std::vector<mrs_msgs::Reference> &transition_points);
  // function for finding new starting point. This method is called when start position is inside of some dangerous area
  mrs_msgs::Reference findNewStartingPosition(const mrs_msgs::Reference start, const mrs_msgs::Reference goal,
                                              const int number_of_intersection_with_dangerous_zone);

  // | ------------------- thread methods ------------------- |
  // thread for flying to desired position
  void planningThread();
};

//}

/* FlyToAction::FlyToAction(...) //{ */

FlyToAction::FlyToAction(const std::string name) : nh_("~"), as_(nh_, name, false), action_name_(name) {
  ros::Time::waitForValid();

  ROS_INFO("[FlyToAction]: Inititalization:");
  ROS_INFO("[FlyToAction]: ------------------------------------------------");
  ROS_INFO("[FlyToAction]: Loading general parameters:");

  mrs_lib::ParamLoader param_loader(nh_, "FlyToAction");

  std::string              diagnostics_topic;
  std::string              odom_topic;
  std::string              uav_name;
  std::vector<std::string> robot_name_list;

  param_loader.loadParam("rate", rate_);
  param_loader.loadParam("detected_radius", detected_radius_);  // radius to goal in which do not create plan
  param_loader.loadParam("goal_reach_distance", goal_reach_distance_);
  param_loader.loadParam("safety_radius", safety_radius_);  // radius aroud other UAV
  param_loader.loadParam("dt", time_stamp_);
  param_loader.loadParam("allowed_error_in_position", allowed_error_in_position_,
                         double(-0.05));  // how much planned trajectory can be inside of dangerous area
  param_loader.loadParam("debug", debug_, bool(false));
  param_loader.loadParam("debug_all_solutions", debug_all_solutions_, bool(false));
  param_loader.loadParam("length_of_horizon", length_of_horizon_, int(40));
  param_loader.loadParam("time_of_valid_msg", time_of_valid_msg_, double(5));
  param_loader.loadParam("uav_name", uav_name);
  param_loader.loadParam("main/robot_name_list", robot_name_list);
  param_loader.loadParam("diagnostics_topic", diagnostics_topic);
  param_loader.loadParam("odom_topic", odom_topic);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[FlyToAction]: Could not load all non-optional parameters!");
    ros::shutdown();
    return;
  }

  /* sanity checks //{ */

  if (robot_name_list.empty()) {
    ROS_ERROR("[FlyToAction]: robot_name_list (target robots) is empty!");
    ros::shutdown();
    return;
  }

  if (robot_name_list.size() != 3) {
    ROS_ERROR("[FlyToAction]: robot_name_list (target robots) is not equal to 3!");
    ros::shutdown();
    return;
  }

  std::vector<std::string>::iterator it = std::find(robot_name_list.begin(), robot_name_list.end(), uav_name);
  if (it != robot_name_list.end()) {
    robot_name_list.erase(it);
  } else {
    ROS_ERROR("[FlyToAction]: robot_name_list doesn't contain this ROBOT_NAME!");
    ros::shutdown();
    return;
  }

  ROS_WARN_COND(detected_radius_ >= safety_radius_, "[FlyToAction]: safety_radius should be smaller than detected_radius");

  //}

  ROS_INFO("[FlyToAction]: All parameters loaded.");
  ROS_INFO("[FlyToAction]: ------------------------------------------------");

  first_trajectory_sended_                         = false;
  was_previous_trajectory_without_dangerous_areas_ = false;

  // register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&FlyToAction::goalCB, this));
  // register the goal and feeback callbacks
  as_.registerPreemptCallback(boost::bind(&FlyToAction::preemptCB, this));

  // create service clients
  service_client_trajectory_ = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("service_set_trajectory");
  service_trigger_hover_     = nh_.serviceClient<std_srvs::Trigger>("service_trigger_hover");

  ROS_INFO("[FlyToAction]: Creating SubscribeHandlers using SubscribeMgr.");

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "FlyToAction";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sub_this_uav_odom_handler_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "this_uav_odom_in");
  // sub_this_uav_tracker_diagnostics_handler_ = mrs_lib::SubscribeHandler<mrs_msgs::MpcTrackerDiagnostics>("tracker_diagnostics_in", no_message_timeout, {},
  // {}, threadsafe);

  const std::string uav1_diagnostic_topic = std::string("/") + robot_name_list[0] + std::string("/") + diagnostics_topic;
  const std::string uav2_diagnostic_topic = std::string("/") + robot_name_list[1] + std::string("/") + diagnostics_topic;

  sub_uav1_plan_diagnostics_handler_ = mrs_lib::SubscribeHandler<plan_keeper::PlanDiagnostics>(shopts, uav1_diagnostic_topic);
  sub_uav2_plan_diagnostics_handler_ = mrs_lib::SubscribeHandler<plan_keeper::PlanDiagnostics>(shopts, uav2_diagnostic_topic);

  const std::string uav1_odom_topic = std::string("/") + robot_name_list[0] + std::string("/") + odom_topic;
  const std::string uav2_odom_topic = std::string("/") + robot_name_list[1] + std::string("/") + odom_topic;

  sub_uav1_odom_handler_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, uav1_odom_topic);
  sub_uav2_odom_handler_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, uav2_odom_topic);

  ROS_INFO("[FlyToAction]: ------------------------------------------------");

  // starting action server
  as_.start();
  ROS_INFO("[FlyToAction]: Starting action server!");

  // start the thread
  action_thread_ = std::thread(&FlyToAction::planningThread, this);
}

//}

/* FlyToAction::getSampledCircle(...) //{ */

// function which sample circle between two angles
void FlyToAction::getSampledCircle(const mrs_msgs::Reference &p_circle, const double radius, const double alpha, const double beta,
                                   std::vector<mrs_msgs::Reference> &transition_points, const double sampling_step, const bool positive_direction) {
  mrs_msgs::Reference point;
  point.position.x = p_circle.position.x + cos(alpha) * radius;
  point.position.y = p_circle.position.y + sin(alpha) * radius;

  transition_points.push_back(point);
  double inner_angle = distanceBetweenTwoAngles(alpha, beta, positive_direction);
  double length      = radius * fabs(inner_angle);
  int    n_step      = length / sampling_step;
  double step_angle;
  if (inner_angle < 0) {
    step_angle = -sampling_step / radius;
  } else {
    step_angle = +sampling_step / radius;
  }

  for (int i = 1; i < n_step + 1; i++) {
    point.position.x = p_circle.position.x + cos(alpha + i * step_angle) * radius;
    point.position.y = p_circle.position.y + sin(alpha + i * step_angle) * radius;
    transition_points.push_back(point);
  }

  double remaining_dist = fmod(length, sampling_step);
  if (remaining_dist > 10e-6) {
    point.position.x = p_circle.position.x + cos(beta) * radius;
    point.position.y = p_circle.position.y + sin(beta) * radius;
    transition_points.push_back(point);
  }
}

//}

/* FlyToAction::getSampledLineSegment(...) //{ */

// function which sample trajectrory between two points
void FlyToAction::getSampledLineSegment(const mrs_msgs::Reference &p_a, const mrs_msgs::Reference &p_b, std::vector<mrs_msgs::Reference> &trajectory,
                                        double &remaining_dist) {
  double ds      = time_stamp_ * velocity_;
  double v_x     = p_b.position.x - p_a.position.x;
  double v_y     = p_b.position.y - p_a.position.y;
  double dist    = sqrt(pow(v_x, 2) + pow(v_y, 2));
  double cos_a   = v_x / dist;
  double sin_a   = v_y / dist;
  int    n_parts = (dist + remaining_dist) / ds;

  mrs_msgs::Reference point = p_a;
  {
    std::scoped_lock lock(server_state_mutex_);
    point.position.z = desired_altitude_;
    point.heading    = goal_.heading;
  }

  if (n_parts < 1) {
    remaining_dist = fabs(fmod(dist + remaining_dist, ds));
    return;
  }

  point.position.x = (ds - remaining_dist) * cos_a + p_a.position.x;
  point.position.y = (ds - remaining_dist) * sin_a + p_a.position.y;

  trajectory.push_back(point);

  for (int j = 0; j < n_parts - 1; j++) {
    point.position.x = ds * cos_a + trajectory.back().position.x;
    point.position.y = ds * sin_a + trajectory.back().position.y;

    trajectory.push_back(point);
  }

  remaining_dist = fabs(fmod(dist + remaining_dist, ds));
}

//}

/* FlyToAction::getSampledTrajectory(...) //{ */

// function which sample trajectrory between transtion points
void FlyToAction::getSampledTrajectory(const std::vector<mrs_msgs::Reference> &transition_points, std::vector<mrs_msgs::Reference> &trajectory,
                                       const bool loop) {
  double remaining_dist = 0;

  mrs_msgs::Reference point = transition_points[0];
  {
    std::scoped_lock lock(server_state_mutex_);
    point.position.z = desired_altitude_;  // setting desired altitude for all points
    point.heading    = goal_.heading;
  }
  trajectory.push_back(point);

  for (size_t i = 0; i < transition_points.size() - 1; i++) {

    mrs_msgs::Reference p_a = transition_points[i];
    mrs_msgs::Reference p_b = transition_points[i + 1];
    getSampledLineSegment(p_a, p_b, trajectory, remaining_dist);
  }

  if (loop) {
    size_t index                     = index_in_desired_trajectory_;
    size_t index_plus_one            = index_in_desired_trajectory_;
    size_t prev_size                 = trajectory.size();
    size_t number_of_addition_points = 0;
    while (int(number_of_addition_points) < length_of_horizon_) {

      mrs_msgs::Reference p_a = desired_trajectory_[index];
      index_plus_one          = index + 1;
      if (index_plus_one >= desired_trajectory_.size()) {
        index_plus_one = 0;
      }
      mrs_msgs::Reference p_b = desired_trajectory_[index_plus_one];
      getSampledLineSegment(p_a, p_b, trajectory, remaining_dist);

      index                     = index_plus_one;
      number_of_addition_points = trajectory.size() - prev_size;
    }
  }

  if (remaining_dist > 10e-6) {  // 1mm
    point.position.x = transition_points.back().position.x;
    point.position.y = transition_points.back().position.y;
    trajectory.push_back(point);
  }
}

//}

/* FlyToAction::getNearestPointLineSegment(...) //{ */

// functions which returns parameter "t". Nearest point on line segment to point can be found as [x_t,y_t] = p_a + t * (p_b - p_a)
// When is t<0 nearest point is p_a
// When is t>0 nearest point is p_b
// When is 0<t<1 then nearest point is between p_a and p_b
double FlyToAction::getNearestPointLineSegment(const mrs_msgs::Reference &p_a, const mrs_msgs::Reference &p_b, const mrs_msgs::Reference &p_x) {
  double t = 0;
  // p_a - p_x
  double v_x = p_a.position.x - p_x.position.x;
  double v_y = p_a.position.y - p_x.position.y;
  // p_b - p_a
  double w_x = p_b.position.x - p_a.position.x;
  double w_y = p_b.position.y - p_a.position.y;
  // |p_b - p_a|^2
  double d = pow(w_x, 2) + pow(w_y, 2);
  // t =(p_a-p_x).(p_b-p_a)/|p_b-p_a|^2
  if (d > 10e-6) {
    t = -(v_x * w_x + v_y * w_y) / d;
  }

  return t;
}

//}

/* FlyToAction::getTangentCirclePoint(...) //{ */

// function fill list of points which are intersection of tangent of circle and point - list_a[0], list_b[0] - ...
void FlyToAction::getTangentCirclePoint(const mrs_msgs::Reference &p_c, const double radius_c, const mrs_msgs::Reference &p_p,
                                        std::vector<mrs_msgs::Reference> &list_a, std::vector<mrs_msgs::Reference> &list_b) {
  double dx = p_c.position.x - p_p.position.x;
  double dy = p_c.position.y - p_p.position.y;

  double dist  = sqrt(pow(dx, 2) + pow(dy, 2));
  double alpha = asin(radius_c / dist);
  double beta  = atan2(dy, dx);

  double theta  = beta - alpha;
  double theta2 = beta + alpha;

  mrs_msgs::Reference p1, p2;
  p1.position.x = p_c.position.x + radius_c * sin(theta);
  p1.position.y = p_c.position.y - radius_c * cos(theta);

  p2.position.x = p_c.position.x - radius_c * sin(theta2);
  p2.position.y = p_c.position.y + radius_c * cos(theta2);

  list_a.push_back(p1);
  list_a.push_back(p2);

  list_b.push_back(p_p);
  list_b.push_back(p_p);
}

//}

/* FlyToAction::innerTangentTwoCircles(...) //{ */

// function fill list of points which are intersection of inner tangent of two circles - list_a[0], list_b[0] - ...
void FlyToAction::innerTangentTwoCircles(const mrs_msgs::Reference &p_a, const double radius_a, const mrs_msgs::Reference &p_b, const double radius_b,
                                         std::vector<mrs_msgs::Reference> &list_a, std::vector<mrs_msgs::Reference> &list_b) {
  mrs_msgs::Reference a1, a2, b1, b2;
  // arctan((y2-y1)/(x2-x1))
  double gamma = atan2(p_b.position.y - p_a.position.y, p_b.position.x - p_a.position.x);
  // arcsin((r2-r1)/sqrt((x2-x1)^2+(y2-y1)^2))
  double beta  = acos((radius_b + radius_a) / sqrt(pow(p_b.position.x - p_a.position.x, 2) + pow(p_b.position.y - p_a.position.y, 2)));
  double alpha = gamma - beta;
  // first point on r_a
  a1.position.x = p_a.position.x + radius_a * cos(alpha);
  a1.position.y = p_a.position.y + radius_a * sin(alpha);
  // first point on r_b
  b1.position.x = p_b.position.x + radius_b * cos(alpha - M_PI);
  b1.position.y = p_b.position.y + radius_b * sin(alpha - M_PI);

  alpha = gamma + beta;
  // second point on r_a
  a2.position.x = p_a.position.x + radius_a * cos(alpha);
  a2.position.y = p_a.position.y + radius_a * sin(alpha);
  // second point on r_b
  b2.position.x = p_b.position.x + radius_b * cos(alpha - M_PI);
  b2.position.y = p_b.position.y + radius_b * sin(alpha - M_PI);

  list_a.push_back(a1);
  list_a.push_back(a2);

  list_b.push_back(b1);
  list_b.push_back(b2);
}

//}

/* FlyToAction::outerTangentTwoCircles(...) //{ */

// function fill list of points which are intersection of outer tangent of two circles - list_a[0], list_b[0] - ...
// https://en.wikipedia.org/wiki/Tangent_lines_to_circles
void FlyToAction::outerTangentTwoCircles(const mrs_msgs::Reference &p_a, const double radius_a, const mrs_msgs::Reference &p_b, const double radius_b,
                                         std::vector<mrs_msgs::Reference> &list_a, std::vector<mrs_msgs::Reference> &list_b) {

  mrs_msgs::Reference a1, a2, b1, b2;
  // arctan((y1-y2)/(x2-x1))
  double gamma = atan2(p_b.position.y - p_a.position.y, p_b.position.x - p_a.position.x);
  // arcsin((r2-r1)/sqrt((x2-x1)^2+(y2-y1)^2))
  double beta  = acos((radius_b - radius_a) / sqrt(pow(p_b.position.x - p_a.position.x, 2) + pow(p_b.position.y - p_a.position.y, 2)));
  double alpha = gamma - beta;
  // first point on r_a
  a1.position.x = p_a.position.x + radius_a * cos(alpha);
  a1.position.y = p_a.position.y + radius_a * sin(alpha);
  // first point on r_b
  b1.position.x = p_b.position.x + radius_b * cos(alpha);
  b1.position.y = p_b.position.y + radius_b * sin(alpha);

  alpha = gamma + beta;
  // second point on r_a
  a2.position.x = p_a.position.x + radius_a * cos(alpha);
  a2.position.y = p_a.position.y + radius_a * sin(alpha);
  // second point on r_b
  b2.position.x = p_b.position.x + radius_b * cos(alpha);
  b2.position.y = p_b.position.y + radius_b * sin(alpha);

  list_a.push_back(a1);
  list_a.push_back(a2);

  list_b.push_back(b1);
  list_b.push_back(b2);
}

//}

/* FlyToAction::distanceTwoPoints(...) //{ */

// function which returns distance between two points
double FlyToAction::distanceTwoPoints(const mrs_msgs::Reference &p_a, const mrs_msgs::Reference &p_b) {
  double dist_sq = pow(p_b.position.x - p_a.position.x, 2) + pow(p_b.position.y - p_a.position.y, 2);
  return sqrt(dist_sq);
}

// function which returns distance between two points in 3D
double FlyToAction::distanceTwoPoints3d(const mrs_msgs::Reference &p_a, const mrs_msgs::Reference &p_b) {
  double dist_sq = pow(p_b.position.x - p_a.position.x, 2) + pow(p_b.position.y - p_a.position.y, 2) + pow(p_b.position.z - p_a.position.z, 2);
  return sqrt(dist_sq);
}

//}

/* FlyToAction::findNearestPoint(...) //{ */

// function which find nearest point "p_nearest" and distance "dist" to this point to couple line segment-Point
void FlyToAction::findNearestPoint(const mrs_msgs::Reference &p_a, const mrs_msgs::Reference &p_b, const mrs_msgs::Reference &p_x,
                                   mrs_msgs::Reference &p_nearest, double &dist) {

  double t = getNearestPointLineSegment(p_a, p_b, p_x);
  if (t < 0) {
    dist      = distanceTwoPoints(p_x, p_a);
    p_nearest = p_a;
  } else if (t > 1) {
    dist      = distanceTwoPoints(p_x, p_b);
    p_nearest = p_b;
  } else {
    p_nearest.position.x = p_a.position.x + t * (p_b.position.x - p_a.position.x);
    p_nearest.position.y = p_a.position.y + t * (p_b.position.y - p_a.position.y);
    dist                 = distanceTwoPoints(p_x, p_nearest);
  }
}

//}

/* FlyToAction::getIntersectionCircleLineSegment(...) //{ */

// functions which returns parameter "t". Nearest point on line segment to circle can be found as [x_t,y_t] = p_a + t * (p_b - p_a)
double FlyToAction::getIntersectionCircleLineSegment(const mrs_msgs::Reference &p_a, const mrs_msgs::Reference &p_b, const mrs_msgs::Reference &p_circle,
                                                     const double radius) {
  double u  = p_b.position.x - p_a.position.x;
  double v  = p_b.position.y - p_a.position.y;
  double a  = pow(u, 2) + pow(v, 2);
  double b  = 2 * p_a.position.x * u - 2 * u * p_circle.position.x + 2 * p_a.position.y * v - 2 * v * p_circle.position.y;
  double c  = -(radius - pow(p_circle.position.x, 2) - pow(p_circle.position.y, 2) - pow(p_a.position.x, 2) - pow(p_a.position.y, 2) +
               2 * p_circle.position.x * p_a.position.x + 2 * p_circle.position.y * p_a.position.y);
  double d  = pow(b, 2) - 4 * a * c;
  double t1 = (-b + sqrt(d)) / (2 * a);
  double t2 = (-b - sqrt(d)) / (2 * a);

  if (0 < t1 < 1) {
    return t1;
  } else if (0 < t2 < 1) {
    return t2;
  } else {
    return 0;
  }
}

//}

/* FlyToAction::distanceBetweenTwoAngles(...) //{ */

// function which returns smallest difference between two angles
double FlyToAction::distanceBetweenTwoAngles(const double alpha, const double beta, const bool positive_direction = true) {
  double angle = atan2(sin(beta - alpha), cos(beta - alpha));
  if (positive_direction) {
    if (angle > 0) {
      return angle;
    } else {
      return 2 * M_PI + angle;
    }
  } else {
    if (angle > 0) {
      return -2 * M_PI + angle;
    } else {
      return angle;
    }
  }
  return angle;
}

//}

/* FlyToAction::distanceFromAreaToTrajectory(...) //{ */

// function which returns nearest distance to dangerous zone
double FlyToAction::distanceFromAreaToTrajectory(const std::vector<mrs_msgs::Reference> &transition_points) {
  double              dist = 0, min_dist = std::numeric_limits<double>::max();
  mrs_msgs::Reference p_nearest;
  for (size_t i = 0; i < transition_points.size() - 1; i++) {
    for (mrs_msgs::Reference p_x : dangerous_points_) {
      findNearestPoint(transition_points[i], transition_points[i + 1], p_x, p_nearest, dist);
      dist = dist - safety_radius_;
      if (dist < min_dist) {
        min_dist = dist;
      }
    }
  }
  return min_dist;
}

//}

/* FlyToAction::findNewStartingPosition(...) //{ */

// function for finding new starting point. This method is called when start position is inside of some dangerous area
mrs_msgs::Reference FlyToAction::findNewStartingPosition(const mrs_msgs::Reference start, [[maybe_unused]] const mrs_msgs::Reference goal,
                                                         const int number_of_intersection_with_dangerous_zone) {
  mrs_msgs::Reference new_start;

  if (number_of_intersection_with_dangerous_zone == 1) {

    mrs_msgs::Reference p_dangerous;
    for (mrs_msgs::Reference p_x : dangerous_points_) {
      if (distanceTwoPoints(p_x, start) < safety_radius_) {
        p_dangerous = p_x;
        break;
      }
    }

    double v_x  = start.position.x - p_dangerous.position.x;
    double v_y  = start.position.y - p_dangerous.position.y;
    double dist = sqrt(pow(v_x, 2) + pow(v_y, 2));

    new_start.position.x = p_dangerous.position.x + v_x / dist * (safety_radius_ + 10e-3);
    new_start.position.y = p_dangerous.position.y + v_y / dist * (safety_radius_ + 10e-3);
    return new_start;
  }

  if (number_of_intersection_with_dangerous_zone == 2) {

    mrs_msgs::Reference p_a, p_b;
    if (dangerous_points_[0].position.x < dangerous_points_[1].position.x) {
      p_a = dangerous_points_[0];
      p_b = dangerous_points_[1];
    } else {
      p_a = dangerous_points_[1];
      p_b = dangerous_points_[0];
    }
    double v_x  = p_a.position.x - p_b.position.x;
    double v_y  = p_a.position.y - p_b.position.y;
    double dist = sqrt(pow(v_x, 2) + pow(v_y, 2));
    double t    = sqrt(pow(safety_radius_, 2) - pow(dist / 2, 2));

    mrs_msgs::Reference inter_1, inter_2, p_middle;
    p_middle.position.x = p_b.position.x + v_x / 2;
    p_middle.position.y = p_b.position.y + v_y / 2;

    // first normal vector for example: v=[v_x,v_y] -> n =[v_y,-v_x]
    inter_1.position.x = p_middle.position.x + v_y / dist * (t + 10e-3);
    inter_1.position.y = p_middle.position.y - v_x / dist * (t + 10e-3);

    // second normal vector for example: v=[v_x,v_y] -> n =[-v_y,v_x]
    inter_2.position.x = p_middle.position.x - v_y / dist * (-t - 10e-3);
    inter_2.position.y = p_middle.position.y + v_x / dist * (-t - 10e-3);

    double dist1 = distanceTwoPoints(inter_1, start);
    double dist2 = distanceTwoPoints(inter_2, start);

    if (dist1 < dist2) {
      new_start = inter_1;
    } else {
      new_start = inter_2;
    }
  }

  return new_start;
}

//}

/* FlyToAction::generateTransititonPoints(...) //{ */

// function which generates transitions points of trajectory
void FlyToAction::generateTransititonPoints(const mrs_msgs::Reference start, const mrs_msgs::Reference goal,
                                            std::vector<mrs_msgs::Reference> &transition_points) {
  transition_points.clear();
  mrs_msgs::Reference point;

  std::vector<std::vector<mrs_msgs::Reference>> plan;
  mrs_msgs::Reference                           offset_plan;
  bool                                          offset_plan_used = false;

  // add start point
  transition_points.push_back(start);
  // add end point
  transition_points.push_back(goal);

  // if there are no dangerous areas
  if (dangerous_points_.size() == 0)
    return;

  double              dist, dist2;
  mrs_msgs::Reference p_start, p_goal, p_min, p_point;

  // checking if dangerous areas are in collision with planned trajectrory or if goal area is feasible
  p_start          = transition_points[0];
  p_goal           = transition_points[1];
  bool is_near     = false;
  bool is_feasible = true;

  for (mrs_msgs::Reference p_x : dangerous_points_) {
    findNearestPoint(p_start, p_goal, p_x, p_min, dist);
    if (dist < safety_radius_) {
      is_near = true;
    }
    dist = distanceTwoPoints(p_x, p_goal);
    if (dist < safety_radius_) {
      is_feasible = false;
    }
  }

  // desired area is no feasible
  if (!is_feasible) {
    ROS_WARN("[FlyToAction]: The goal is in safety area of another uav.");
    transition_points.clear();
    // double t = getIntersectionCircleLineSegment(p_start, p_goal, p_point, safety_radius_);
    return;
  }
  // no dangerous area around
  if (!is_near) {
    if (debug_)
      ROS_INFO("[FlyToAction]: No conflict.");
    return;
  }

  // checking if start point is in some dangerous zones
  int number_of_intersection_with_dangerous_zone = 0;
  for (mrs_msgs::Reference p_x : dangerous_points_) {
    if (distanceTwoPoints(p_x, start) < safety_radius_) {
      number_of_intersection_with_dangerous_zone++;
    }
  }

  mrs_msgs::Reference new_start;
  if (number_of_intersection_with_dangerous_zone > 0) {
    new_start            = findNewStartingPosition(start, goal, number_of_intersection_with_dangerous_zone);
    offset_plan          = start;
    transition_points[0] = new_start;
    offset_plan_used     = true;

    if (debug_) {
      ROS_WARN("[FlyToAction]: Starting position in dangerous area!");
      printf("number: %d\n", number_of_intersection_with_dangerous_zone);
    }

    number_of_intersection_with_dangerous_zone = 0;
    for (mrs_msgs::Reference p_x : dangerous_points_) {
      if (distanceTwoPoints(p_x, new_start) < safety_radius_) {
        number_of_intersection_with_dangerous_zone++;
      }
    }

    if (number_of_intersection_with_dangerous_zone > 0) {
      new_start            = findNewStartingPosition(transition_points[0], goal, number_of_intersection_with_dangerous_zone);
      transition_points[0] = new_start;
    }

    if (debug_) {
      printf("number: %d\n", number_of_intersection_with_dangerous_zone);
      printf("new_start: [%.02f %.02f]\n", transition_points[0].position.x, transition_points[0].position.y);
    }

    p_start = transition_points[0];
    p_goal  = transition_points[1];
    is_near = false;

    for (mrs_msgs::Reference p_x : dangerous_points_) {
      findNearestPoint(p_start, p_goal, p_x, p_min, dist);
      if (dist < safety_radius_) {
        is_near = true;
      }
    }

    // no dangerous area around
    if (!is_near) {
      if (debug_) {
        ROS_INFO("[FlyToAction]: No conflict.");
      }
      return;
    }
  }

  // ======== FINDING ANALYTIC SOLUTIONS ======
  //
  // for just one dangerous area
  if (dangerous_points_.size() == 1) {
    p_point = dangerous_points_[0];
    p_start = transition_points[0];
    p_goal  = transition_points[1];

    std::vector<mrs_msgs::Reference> list_a, list_b;
    // list_b[0:1] = p_start
    getTangentCirclePoint(p_point, safety_radius_, p_start, list_a, list_b);
    // list_b[0:1] = p_goal
    getTangentCirclePoint(p_point, safety_radius_, p_goal, list_a, list_b);

    double alpha_1 = atan2(list_a[0].position.y - p_point.position.y, list_a[0].position.x - p_point.position.x);
    double alpha_2 = atan2(list_a[1].position.y - p_point.position.y, list_a[1].position.x - p_point.position.x);

    double beta_1 = atan2(list_a[3].position.y - p_point.position.y, list_a[3].position.x - p_point.position.x);
    double beta_2 = atan2(list_a[2].position.y - p_point.position.y, list_a[2].position.x - p_point.position.x);

    dist =
        distanceTwoPoints(p_start, list_a[0]) + safety_radius_ * fabs(distanceBetweenTwoAngles(alpha_1, beta_1, true)) + distanceTwoPoints(p_goal, list_a[3]);
    dist2 =
        distanceTwoPoints(p_start, list_a[1]) + safety_radius_ * fabs(distanceBetweenTwoAngles(alpha_2, beta_2, false)) + distanceTwoPoints(p_goal, list_a[2]);

    std::vector<mrs_msgs::Reference> points;
    // first solution
    points.clear();
    p_start.heading = dist;
    points.push_back(p_start);
    getSampledCircle(p_point, safety_radius_, alpha_1, beta_1, points, 0.5, true);  // FIXME: constant sampling step
    points.push_back(p_goal);
    plan.push_back(points);
    // second solution
    points.clear();
    p_start.heading = dist2;
    points.push_back(p_start);
    getSampledCircle(p_point, safety_radius_, alpha_2, beta_2, points, 0.5, false);  // FIXME: constant sampling step
    points.push_back(p_goal);
    plan.push_back(points);
  }

  // For two dangerous areas
  if (dangerous_points_.size() == 2) {
    // ------------------------------------------------
    // for first dangerous area
    p_point = dangerous_points_[0];
    p_start = transition_points[0];
    p_goal  = transition_points[1];

    std::vector<mrs_msgs::Reference> list_a, list_b;
    // list_b[0:1] = p_start
    getTangentCirclePoint(p_point, safety_radius_, p_start, list_a, list_b);
    // list_b[0:1] = p_goal
    getTangentCirclePoint(p_point, safety_radius_, p_goal, list_a, list_b);

    double alpha_1 = atan2(list_a[0].position.y - p_point.position.y, list_a[0].position.x - p_point.position.x);
    double alpha_2 = atan2(list_a[1].position.y - p_point.position.y, list_a[1].position.x - p_point.position.x);

    double beta_1 = atan2(list_a[3].position.y - p_point.position.y, list_a[3].position.x - p_point.position.x);
    double beta_2 = atan2(list_a[2].position.y - p_point.position.y, list_a[2].position.x - p_point.position.x);

    dist =
        distanceTwoPoints(p_start, list_a[0]) + safety_radius_ * fabs(distanceBetweenTwoAngles(alpha_1, beta_1, true)) + distanceTwoPoints(p_goal, list_a[3]);
    dist2 =
        distanceTwoPoints(p_start, list_a[1]) + safety_radius_ * fabs(distanceBetweenTwoAngles(alpha_2, beta_2, false)) + distanceTwoPoints(p_goal, list_a[2]);

    std::vector<mrs_msgs::Reference> points;
    // first solution
    points.clear();
    p_start.heading = dist;
    points.push_back(p_start);
    getSampledCircle(p_point, safety_radius_, alpha_1, beta_1, points, 0.5, true);  // FIXME: constant sampling step
    points.push_back(p_goal);
    plan.push_back(points);
    // second solution
    points.clear();
    p_start.heading = dist2;
    points.push_back(p_start);
    getSampledCircle(p_point, safety_radius_, alpha_2, beta_2, points, 0.5, false);  // FIXME: constant sampling step
    points.push_back(p_goal);
    plan.push_back(points);

    // ----------------------------------------------
    // for second dangerous area
    p_point = dangerous_points_[1];
    p_start = transition_points[0];
    p_goal  = transition_points[1];

    list_a.clear();
    list_b.clear();
    // list_b[0:1] = p_start
    getTangentCirclePoint(p_point, safety_radius_, p_start, list_a, list_b);
    // list_b[0:1] = p_goal
    getTangentCirclePoint(p_point, safety_radius_, p_goal, list_a, list_b);

    alpha_1 = atan2(list_a[0].position.y - p_point.position.y, list_a[0].position.x - p_point.position.x);
    alpha_2 = atan2(list_a[1].position.y - p_point.position.y, list_a[1].position.x - p_point.position.x);

    beta_1 = atan2(list_a[3].position.y - p_point.position.y, list_a[3].position.x - p_point.position.x);
    beta_2 = atan2(list_a[2].position.y - p_point.position.y, list_a[2].position.x - p_point.position.x);

    dist =
        distanceTwoPoints(p_start, list_a[0]) + safety_radius_ * fabs(distanceBetweenTwoAngles(alpha_1, beta_1, true)) + distanceTwoPoints(p_goal, list_a[3]);
    dist2 =
        distanceTwoPoints(p_start, list_a[1]) + safety_radius_ * fabs(distanceBetweenTwoAngles(alpha_2, beta_2, false)) + distanceTwoPoints(p_goal, list_a[2]);

    // third solution
    points.clear();
    p_start.heading = dist;
    points.push_back(p_start);
    getSampledCircle(p_point, safety_radius_, alpha_1, beta_1, points, 0.5, true);  // FIXME: constant sampling step
    points.push_back(p_goal);
    plan.push_back(points);
    // fourth solution
    points.clear();
    p_start.heading = dist2;
    points.push_back(p_start);
    getSampledCircle(p_point, safety_radius_, alpha_2, beta_2, points, 0.5, false);  // FIXME: constant sampling step
    points.push_back(p_goal);
    plan.push_back(points);

    // ----------------------------------------------
    // for both dangerous areas
    mrs_msgs::Reference p_dangerous1, p_dangerous2;
    p_start = transition_points[0];
    p_goal  = transition_points[1];

    dist  = distanceTwoPoints(p_start, dangerous_points_[0]);
    dist2 = distanceTwoPoints(p_start, dangerous_points_[1]);

    if (dist < dist2) {
      p_dangerous1 = dangerous_points_[0];
      p_dangerous2 = dangerous_points_[1];
    } else {
      p_dangerous1 = dangerous_points_[1];
      p_dangerous2 = dangerous_points_[0];
    }

    list_a.clear();
    list_b.clear();
    // list_b[0:1] = p_start
    getTangentCirclePoint(p_dangerous1, safety_radius_, p_start, list_a, list_b);
    // list_b[0:1] = p_goal
    getTangentCirclePoint(p_dangerous2, safety_radius_, p_goal, list_a, list_b);

    alpha_1 = atan2(list_a[0].position.y - p_dangerous1.position.y, list_a[0].position.x - p_dangerous1.position.x);
    alpha_2 = atan2(list_a[1].position.y - p_dangerous1.position.y, list_a[1].position.x - p_dangerous1.position.x);

    beta_1 = atan2(list_a[3].position.y - p_dangerous2.position.y, list_a[3].position.x - p_dangerous2.position.x);
    beta_2 = atan2(list_a[2].position.y - p_dangerous2.position.y, list_a[2].position.x - p_dangerous2.position.x);

    // ---------------------------------------------
    // outer tangents
    outerTangentTwoCircles(p_dangerous1, safety_radius_, p_dangerous2, safety_radius_, list_a, list_b);
    double gamma_1, gamma_2;

    gamma_1 = atan2(list_a[4].position.y - p_dangerous1.position.y, list_a[4].position.x - p_dangerous1.position.x);
    gamma_2 = atan2(list_a[5].position.y - p_dangerous1.position.y, list_a[5].position.x - p_dangerous1.position.x);

    dist = distanceTwoPoints(p_start, list_a[0]) + safety_radius_ * fabs(distanceBetweenTwoAngles(alpha_1, gamma_1, true)) +
           distanceTwoPoints(list_a[4], list_b[4]) + safety_radius_ * fabs(distanceBetweenTwoAngles(gamma_1, beta_1, true)) +
           distanceTwoPoints(p_goal, list_a[3]);
    dist2 = distanceTwoPoints(p_start, list_a[1]) + safety_radius_ * fabs(distanceBetweenTwoAngles(alpha_2, gamma_2, false)) +
            distanceTwoPoints(list_a[5], list_b[5]) + safety_radius_ * fabs(distanceBetweenTwoAngles(gamma_2, beta_2, false)) +
            distanceTwoPoints(p_goal, list_a[2]);

    // fifth solution
    points.clear();
    p_start.heading = dist;
    points.push_back(p_start);
    getSampledCircle(p_dangerous1, safety_radius_, alpha_1, gamma_1, points, 0.5, true);  // FIXME: constant sampling step
    getSampledCircle(p_dangerous2, safety_radius_, gamma_1, beta_1, points, 0.5, true);   // FIXME: constant sampling step
    points.push_back(p_goal);
    plan.push_back(points);
    // sixth solution
    points.clear();
    p_start.heading = dist2;
    points.push_back(p_start);
    getSampledCircle(p_dangerous1, safety_radius_, alpha_2, gamma_2, points, 0.5, false);  // FIXME: constant sampling step
    getSampledCircle(p_dangerous2, safety_radius_, gamma_2, beta_2, points, 0.5, false);   // FIXME: constant sampling step
    points.push_back(p_goal);
    plan.push_back(points);

    // ----------------------------------------------
    // inner tangents
    if (distanceTwoPoints(p_dangerous1, p_dangerous2) > 2 * safety_radius_ + 10e-3) {
      innerTangentTwoCircles(p_dangerous1, safety_radius_, p_dangerous2, safety_radius_, list_a, list_b);

      gamma_1 = atan2(list_a[6].position.y - p_dangerous1.position.y, list_a[6].position.x - p_dangerous1.position.x);
      gamma_2 = atan2(list_a[7].position.y - p_dangerous1.position.y, list_a[7].position.x - p_dangerous1.position.x);

      dist = distanceTwoPoints(p_start, list_a[0]) + safety_radius_ * fabs(distanceBetweenTwoAngles(alpha_1, gamma_1, true)) +
             distanceTwoPoints(list_a[6], list_b[6]) + safety_radius_ * fabs(distanceBetweenTwoAngles(gamma_1 - M_PI, beta_2, false)) +
             distanceTwoPoints(p_goal, list_a[2]);
      dist2 = distanceTwoPoints(p_start, list_a[1]) + safety_radius_ * fabs(distanceBetweenTwoAngles(alpha_2, gamma_2, false)) +
              distanceTwoPoints(list_a[7], list_b[7]) + safety_radius_ * fabs(distanceBetweenTwoAngles(gamma_2 - M_PI, beta_1, true)) +
              distanceTwoPoints(p_goal, list_a[3]);

      // seventh solution
      points.clear();
      p_start.heading = dist;
      points.push_back(p_start);
      getSampledCircle(p_dangerous1, safety_radius_, alpha_1, gamma_1, points, 0.5, true);         // FIXME: constant sampling step
      getSampledCircle(p_dangerous2, safety_radius_, gamma_1 - M_PI, beta_2, points, 0.5, false);  // FIXME: constant sampling step
      points.push_back(p_goal);
      plan.push_back(points);

      // eighth solution
      points.clear();
      p_start.heading = dist2;
      points.push_back(p_start);
      getSampledCircle(p_dangerous1, safety_radius_, alpha_2, gamma_2, points, 0.5, false);       // FIXME: constant sampling step
      getSampledCircle(p_dangerous2, safety_radius_, gamma_2 - M_PI, beta_1, points, 0.5, true);  // FIXME: constant sampling step
      points.push_back(p_goal);
      plan.push_back(points);
    }
  }

  if (debug_all_solutions_) {
    ROS_INFO("[FlyToAction]: Particular solutions (Transition points):");

    for (size_t i = 0; i < plan.size(); i++) {
      ROS_INFO("[FlyToAction]: \t%zu solution", (i + 1));

      for (size_t j = 0; j < plan[i].size(); j++) {
        printf("%.02f %.02f %.02f %.02f\n", plan[i][j].position.x, plan[i][j].position.y, plan[i][j].position.z, plan[i][j].heading);
      }
    }
  }

  if (debug_) {
    ROS_INFO("[FlyToAction]: Distances [id, length, distance to dangerous area, is_safe]:");
  }

  double min_dist = std::numeric_limits<double>::max();
  int    index;

  for (size_t i = 0; i < plan.size(); i++) {

    if (plan[i][0].heading < min_dist &&
        distanceFromAreaToTrajectory(plan[i]) > allowed_error_in_position_) {  // FIXME: plan is feasible if is inside of dangerous maximally 5cm
      min_dist = plan[i][0].heading;
      index    = i;
    }

    if (debug_) {
      printf("%zu, \t%.03f, \t%0.3f, \t%s\n", (i + 1), plan[i][0].heading, distanceFromAreaToTrajectory(plan[i]),
             distanceFromAreaToTrajectory(plan[i]) > allowed_error_in_position_ ? "OK" : "--");
    }
  }

  if (debug_) {
    ROS_INFO("[FlyToAction]: Index of the selected solution: %d", (index + 1));
  }

  transition_points = plan[index];

  /*
     if (offset_plan_used){
     transition_points.insert(transition_points.begin(), offset_plan);
     transition_points[0].heading = transition_points[1].heading + distanceTwoPoints(transition_points[0], transition_points[1]);
     transition_points[1].heading = 0;
     }
     */
}

//}

/* FlyToAction::goalCB() //{ */

// goal callback of action server
void FlyToAction::goalCB() {
  std::scoped_lock lock(server_state_mutex_);

  const mbzirc_flyto_planner::FlyToGoal_<std::allocator<void>> *goal = as_.acceptNewGoal().get();
  index_in_desired_trajectory_                                       = 0;

  desired_trajectory_ = goal->points;
  if (desired_trajectory_.size() == 0) {
    ROS_ERROR("[FlyToAction]: EMPTY goal->points.");
    result_.success = false;
    result_.message = "EMPTY goal->points.";
    as_.setAborted(result_);
    return;
  }

  plan_in_loop_ = goal->plan_in_loop;
  if (plan_in_loop_ && desired_trajectory_.size() < 2) {
    plan_in_loop_ = false;
    ROS_WARN("[FlyToAction]: Not enough points for loop. => Loop disabled.");
  }

  goal_    = goal->points[index_in_desired_trajectory_];
  use_yaw_ = goal->use_yaw;

  velocity_         = goal->velocity;
  desired_altitude_ = goal_.position.z;
  ROS_WARN("[FlyToAction]: Accepting new goal: [%.02f, %.02f, %.02f, %.02f] with speed: %.02f, loop: %s", goal_.position.x, goal_.position.y, goal_.position.z,
           goal_.heading, goal->velocity, plan_in_loop_ ? "TRUE" : "FALSE");
  first_trajectory_sended_ = false;
}

//}

/* FlyToAction::preemptCB() //{ */

// preempt callback of action server
void FlyToAction::preemptCB() {
  std::scoped_lock lock(server_state_mutex_);
  ROS_WARN("[FlyToAction]: Action server preempted. Calling Hover.");

  result_.success = false;
  result_.message = "Action server preempted. Calling Hover.";

  // set the action state to preempted
  as_.setPreempted(result_);

  // stop tracking by service client
  std_srvs::Trigger srv_trigger;

  if (service_trigger_hover_.call(srv_trigger)) {
    // if calling was successful (mbzirc_tracker responded)
    if (!srv_trigger.response.success) {
      ROS_ERROR("[FlyToAction]: Hovering calling response: %s", srv_trigger.response.message.c_str());
    }
  } else {
    ROS_ERROR("[FlyToAction]:  Hovering service unreacheable");
  }
  first_trajectory_sended_ = false;
}

//}

/* FlyToAction::planningThread() //{ */

// thread for flying to desired position
void FlyToAction::planningThread() {
  ros::Rate r(rate_);
  ros::Rate fast_rate(10);

  mrs_msgs::Reference              goal, start;
  std::vector<mrs_msgs::Reference> transition_points;

  while (ros::ok()) {

    if (as_.isActive()) {

      // check if we got all necessary msg
      if (!sub_this_uav_odom_handler_.hasMsg()) {  //|| !sub_this_uav_tracker_diagnostics_handler_.hasMsg()) {
        fast_rate.sleep();
        continue;
      }

      /*
      // sub_this_uav_tracker_diagnostics_handler_.getMsg()->tracking_trajectory not good
      if (first_trajectory_sended_ && !plan_in_loop_) {
        if (!sub_this_uav_tracker_diagnostics_handler_.getMsg()->tracking_trajectory) {
          std::scoped_lock lock(server_state_mutex_);

          ROS_WARN("[FlyToAction]: Goal position reached");
          result_.message = "Flying done.";
          result_.success = true;
          // set the action state to succeeded
          as_.setSucceeded(result_);
          continue;
        }
      }
      */

      if (debug_) {
        printf("=======================================\n");
      }

      // get local variable from subscriber handler
      nav_msgs::OdometryConstPtr           odom_msg             = sub_this_uav_odom_handler_.getMsg();
      nav_msgs::OdometryConstPtr           uav1_odom_msg        = sub_uav1_odom_handler_.getMsg();
      nav_msgs::OdometryConstPtr           uav2_odom_msg        = sub_uav2_odom_handler_.getMsg();
      plan_keeper::PlanDiagnosticsConstPtr uav1_diagnostics_msg = sub_uav1_plan_diagnostics_handler_.getMsg();
      plan_keeper::PlanDiagnosticsConstPtr uav2_diagnostics_msg = sub_uav2_plan_diagnostics_handler_.getMsg();

      // proccess odom msg
      start.position.x = odom_msg->pose.pose.position.x;
      start.position.y = odom_msg->pose.pose.position.y;
      start.position.z = odom_msg->pose.pose.position.z;

      // get local variable from shared variable goal_
      {
        std::scoped_lock lock(server_state_mutex_);
        goal = goal_;
      }

      // use a new goal whether uav is close to the old one
      if (plan_in_loop_ && distanceTwoPoints(goal, start) <= detected_radius_) {

        index_in_desired_trajectory_++;
        if (index_in_desired_trajectory_ >= desired_trajectory_.size()) {
          index_in_desired_trajectory_ = 0;
        }

        {
          std::scoped_lock lock(server_state_mutex_);
          goal_                    = desired_trajectory_[index_in_desired_trajectory_];
          goal                     = goal_;
          first_trajectory_sended_ = false;
        }
      }

      double dd = distanceTwoPoints3d(start, goal);
      if (debug_) {
        ROS_INFO("[FlyToAction]: Start position:");
        printf("%.02f, %.02f, %.02f, %.02f\n", start.position.x, start.position.y, start.position.z, start.heading);
        ROS_INFO("[FlyToAction]: Goal:");
        printf("%.02f, %.02f, %.02f, %.02f\n", goal.position.x, goal.position.y, goal.position.z, goal.heading);
        ROS_INFO("[FlyToAction]: Distance to Goal:");
        printf("%.02f\n", dd);
      }

      if (dd <= goal_reach_distance_) {
        ROS_WARN("[FlyToAction]: Goal position reached");
        result_.message = "Flying done.";
        result_.success = true;
        // set the action state to succeeded
        as_.setSucceeded(result_);
        continue;
      }

      // clear all dangerous areas
      dangerous_points_.clear();

      // loading of dangerous areas
      // state_id = 3 -> GRASPING_STATE
      // state_id = 4 -> PLACING_STATE
      // state_id = 5 -> WAITING_STATE

      if (uav1_diagnostics_msg != nullptr && uav1_odom_msg != nullptr) {
        double time_diff = (ros::Time::now() - uav1_diagnostics_msg->stamp).toSec();
        if (uav1_diagnostics_msg->state.state_id >= 3 && uav1_diagnostics_msg->state.state_id <= 5 && time_diff < time_of_valid_msg_) {
          mrs_msgs::Reference p;
          p.position.x = uav1_odom_msg->pose.pose.position.x;
          p.position.y = uav1_odom_msg->pose.pose.position.y;
          dangerous_points_.push_back(p);
        }
      }

      if (uav2_diagnostics_msg != nullptr && uav2_odom_msg != nullptr) {
        double time_diff = (ros::Time::now() - uav2_diagnostics_msg->stamp).toSec();
        if (uav2_diagnostics_msg->state.state_id >= 3 && uav2_diagnostics_msg->state.state_id <= 5 && time_diff < time_of_valid_msg_) {
          mrs_msgs::Reference p;
          p.position.x = uav2_odom_msg->pose.pose.position.x;
          p.position.y = uav2_odom_msg->pose.pose.position.y;
          dangerous_points_.push_back(p);
        }
      }

      if (debug_) {
        ROS_INFO("[FlyToAction]: DANGEROUS ZONE [x,y,r]:");
        if (dangerous_points_.empty()) {
          printf("none\n");
        } else {
          for (size_t i = 0; i < dangerous_points_.size(); i++) {
            printf("%.02f, %.02f, %.02f\n", dangerous_points_[i].position.x, dangerous_points_[i].position.y, safety_radius_);
          }
        }
      }

      // find plan
      generateTransititonPoints(start, goal, transition_points);

      if (transition_points.size() == 0) {

        if (!plan_in_loop_) {
          std::scoped_lock lock(server_state_mutex_);

          if (as_.isActive()) {
            ROS_ERROR("[FlyToAction]: Goal action aborted! Goal is currently unfeasible. Calling Hover.");
            // set the action state to preempted
            result_.success = false;
            result_.message = "Goal is currently unfeasible! Hovering.";
            as_.setAborted(result_);

            // stop tracking by service client
            std_srvs::Trigger srv_trigger;

            if (service_trigger_hover_.call(srv_trigger)) {
              // if calling was successful (mbzirc_tracker responded)
              if (!srv_trigger.response.success) {
                ROS_ERROR("[FlyToAction]: Hovering calling response: %s", srv_trigger.response.message.c_str());
              }
            } else {
              ROS_ERROR("[FlyToAction]: Hovering service unreacheable");
            }
            first_trajectory_sended_ = false;
          } else {
            fast_rate.sleep();
            continue;
          }

        } else {

          index_in_desired_trajectory_++;
          if (index_in_desired_trajectory_ >= desired_trajectory_.size()) {
            index_in_desired_trajectory_ = 0;
          }

          {
            std::scoped_lock lock(server_state_mutex_);
            goal_                    = desired_trajectory_[index_in_desired_trajectory_];
            first_trajectory_sended_ = false;
          }
        }

        fast_rate.sleep();
        continue;
      }

      if (debug_) {
        ROS_INFO("[FlyToAction]: Transition points of solution:");
        for (mrs_msgs::Reference p : transition_points) {
          printf("%.02f %.02f %.02f %.02f\n", p.position.x, p.position.y, p.position.z, p.heading);
        }
      }

      bool send_trajectory =
          distanceTwoPoints(goal, start) > detected_radius_ && !(dangerous_points_.size() == 0 && was_previous_trajectory_without_dangerous_areas_);

      // if trajectory hasn't been send yet or helicopter is further that detected_radius send the trajectory
      if (!first_trajectory_sended_ || send_trajectory) {

        trajectory_.points.clear();
        getSampledTrajectory(transition_points, trajectory_.points, plan_in_loop_);
        if (trajectory_.points.size() > 1) {
          trajectory_.points.erase(trajectory_.points.begin());
        }

        trajectory_.header.stamp = odom_msg->header.stamp;
        trajectory_.use_heading  = use_yaw_;
        trajectory_.fly_now      = true;
        trajectory_.loop         = false;

        if (debug_) {
          ROS_INFO("[FlyToAction]: Solution (sampled transition points) without first point:");
          for (mrs_msgs::Reference p : trajectory_.points) {
            printf("%.02f %.02f %.02f %.02f\n", p.position.x, p.position.y, p.position.z, p.heading);
          }
        }

        // publish plan by service client
        mrs_msgs::TrajectoryReferenceSrv srv;
        srv.request.trajectory = trajectory_;

        {
          std::scoped_lock lock(server_state_mutex_);
          if (as_.isActive()) {
            if (service_client_trajectory_.call(srv)) {
              // if calling was successful (mbzirc_tracker responded)
              if (srv.response.success) {

                ROS_INFO("[FlyToAction]: SetTrajectory calling response: %s", srv.response.message.c_str());

                if (!first_trajectory_sended_) {
                  first_trajectory_sended_ = true;
                }

              } else {

                ROS_ERROR("[FlyToAction]: SetTrajectory calling response: %s", srv.response.message.c_str());
              }

            } else {

              ROS_ERROR("[FlyToAction]: SetTrajectory service unreacheable");
            }
          }
        }

        // fill feedback
        feedback_.distance_to_goal = dd;              // transition_points[0].heading;
        feedback_.remaining_time   = dd / velocity_;  // transition_points[0].heading / velocity_;
        feedback_.is_replanned     = true;

      } else {

        // fill feedback
        feedback_.distance_to_goal = dd;              // transition_points[0].heading;
        feedback_.remaining_time   = dd / velocity_;  // transition_points[0].heading / velocity_;
        feedback_.is_replanned     = false;
      }

      if (dangerous_points_.size() == 0) {
        was_previous_trajectory_without_dangerous_areas_ = true;
      } else {
        was_previous_trajectory_without_dangerous_areas_ = false;
      }

      as_.publishFeedback(feedback_);
      r.sleep();
      continue;
    }
    fast_rate.sleep();
  }
}

//}

/* MAIN FUNCTION //{ */

int main(int argc, char **argv) {
  ros::init(argc, argv, "flyto_action");

  FlyToAction fly_to(ros::this_node::getName());

  ros::spin();

  return EXIT_SUCCESS;
}

//}
