#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mbzirc_flyto_planner/FlyToAction.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_goTo");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<mbzirc_flyto_planner::FlyToAction> ac("flyto_action", true);

  ROS_INFO("[FlyToActionClient]: Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer();  // will wait for infinite time

  ROS_INFO("[FlyToActionClient]: Action server started, sending goal.");

  // send a goal to the action
  mbzirc_flyto_planner::FlyToGoal flygoal;
  mrs_msgs::Reference             goal;
  goal.position.x = 20;
  goal.position.y = 0;
  goal.position.z = 3;
  goal.heading    = 0;
  flygoal.points.push_back(goal);
  goal.position.x = -10;
  goal.position.y = 0;
  goal.position.z = 3;
  goal.heading    = 0;
  flygoal.points.push_back(goal);
  flygoal.velocity     = 2;
  flygoal.plan_in_loop = false;
  ac.sendGoal(flygoal);

  // wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("[FlyToActionClient]: Action finished: %s", state.toString().c_str());
  } else {
    ROS_INFO("[FlyToActionClient]: Action did not finish before the time out.");
  }

  return EXIT_SUCCESS;
}
