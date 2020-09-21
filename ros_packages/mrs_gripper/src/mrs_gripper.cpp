#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <mutex>

#include <string>
#include <mrs_msgs/BacaProtocol.h>
#include <mrs_msgs/GripperDiagnostics.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

/* class BacaProtocol //{ */

class MrsGripper {
public:
  MrsGripper();

  ros::ServiceServer grip_service;
  ros::ServiceServer ungrip_service;
  ros::ServiceServer grip_nocal_service;

  bool callbackGrip(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackUngrip(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackGripNoCal(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  void callbackBaca(const mrs_msgs::BacaProtocolConstPtr &msg);
  void callbackControlDiagnostics(const mrs_msgs::ControlManagerDiagnostics &msg);

  ros::NodeHandle nh_;

  ros::Publisher baca_protocol_publisher;
  ros::Publisher gripper_diagnostics_publisher;

  ros::Subscriber baca_protocol_subscriber;
  ros::Subscriber control_diag_subscriber;

  std::mutex mutex_msg;

  bool started_flying = false;
  bool aborted        = false;
};

//}

/* MrsGripper() //{ */

MrsGripper::MrsGripper() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  baca_protocol_publisher       = nh_.advertise<mrs_msgs::BacaProtocol>("baca_protocol_out", 1);
  gripper_diagnostics_publisher = nh_.advertise<mrs_msgs::GripperDiagnostics>("gripper_diagnostics_out", 1);

  baca_protocol_subscriber = nh_.subscribe("baca_protocol_in", 1, &MrsGripper::callbackBaca, this, ros::TransportHints().tcpNoDelay());
  control_diag_subscriber  = nh_.subscribe("control_manager_diag_in", 1, &MrsGripper::callbackControlDiagnostics, this, ros::TransportHints().tcpNoDelay());

  grip_service       = nh_.advertiseService("grip_in", &MrsGripper::callbackGrip, this);
  ungrip_service     = nh_.advertiseService("ungrip_in", &MrsGripper::callbackUngrip, this);
  grip_nocal_service = nh_.advertiseService("grip_nocal_in", &MrsGripper::callbackGripNoCal, this);
}

//}

// | ------------------------ callbacks ------------------------ |

/* callbackBaca() //{ */

void MrsGripper::callbackBaca(const mrs_msgs::BacaProtocolConstPtr &msg) {
  if (msg->payload.size() == 7 && msg->payload[0] == 0x43 && msg->checksum_correct == true) {
    mrs_msgs::GripperDiagnostics gripper_msg;
    gripper_msg.stamp           = ros::Time::now();
    gripper_msg.gripper_on      = msg->payload[1];
    gripper_msg.gripping_object = msg->payload[2];

    gripper_msg.has_hall = true;

    int16_t tmp_hall = msg->payload[3] << 8;
    tmp_hall |= msg->payload[4];
    gripper_msg.hall1_debug = tmp_hall;

    tmp_hall = msg->payload[5] << 8;
    tmp_hall |= msg->payload[6];
    gripper_msg.hall2_debug = tmp_hall;

    gripper_msg.has_proximity  = false;
    gripper_msg.has_ultrasonic = false;
    gripper_diagnostics_publisher.publish(gripper_msg);
  }

  // Msg Processing for Ashwin's gripper W/ only Prox Sensor
  else if (msg->payload.size() == 4 && msg->payload[0] == 0x43 && msg->checksum_correct == true) {
    mrs_msgs::GripperDiagnostics gripper_msg;
    gripper_msg.stamp = ros::Time::now();

    // Setting up gripper sensor configuration
    gripper_msg.has_hall       = false;
    gripper_msg.has_proximity  = true;
    gripper_msg.has_ultrasonic = false;

    // Processing Raw Msg
    int8_t proximity_val         = msg->payload[1];
    gripper_msg.proximity1_debug = proximity_val;

    // Feedback and Magnet State
    bool proximity_feedback = msg->payload[2];
    gripper_msg.gripper_on  = msg->payload[3];

    // Final Feedback
    gripper_msg.gripping_object = proximity_feedback;
    gripper_diagnostics_publisher.publish(gripper_msg);
  }


  // Msg Processing for Ashwin's gripper Hall Sensor + Prox Sensor
  else if (msg->payload.size() == 8 && msg->payload[0] == 0x43 && msg->checksum_correct == true) {
    mrs_msgs::GripperDiagnostics gripper_msg;
    gripper_msg.stamp      = ros::Time::now();
    gripper_msg.gripper_on = true;  // placeholder! - need to implement mosfet status function

    // Setting up gripper sensor configuration
    gripper_msg.has_hall       = true;
    gripper_msg.has_proximity  = true;
    gripper_msg.has_ultrasonic = false;

    // Diagnostics Info
    int8_t hall_val         = msg->payload[1];
    gripper_msg.hall1_debug = hall_val;

    int8_t proximity_val         = msg->payload[2];
    gripper_msg.proximity1_debug = proximity_val;

    // Feedback Status
    bool hall_feedback          = msg->payload[3];
    bool proximity_feedback     = msg->payload[4];
    gripper_msg.gripping_object = msg->payload[5];
    gripper_msg.gripper_on      = msg->payload[6];

    // Publish Message
    gripper_diagnostics_publisher.publish(gripper_msg);
  }
}

//}

/* callbackGrip() //{ */

bool MrsGripper::callbackGrip(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!aborted) {

    mrs_msgs::BacaProtocol msg_out;
    msg_out.stamp   = ros::Time::now();
    msg_out.payload = {0x40};
    baca_protocol_publisher.publish(msg_out);
    res.message = "Gripper started gripping";
    res.success = true;
    return true;

  } else {

    res.message = "Flight was aborted, gripper is disabled";
    res.success = false;
    return true;
  }
}

//}

/* callbackUngrip() //{ */

bool MrsGripper::callbackUngrip(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  mrs_msgs::BacaProtocol msg_out;
  msg_out.stamp   = ros::Time::now();
  msg_out.payload = {0x41};
  baca_protocol_publisher.publish(msg_out);
  res.message = "Gripper stopped gripping";
  res.success = true;
  return true;
}

//}

/* callbackGripNoCal() //{ */

bool MrsGripper::callbackGripNoCal(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!aborted) {

    mrs_msgs::BacaProtocol msg_out;
    msg_out.stamp   = ros::Time::now();
    msg_out.payload = {0x42};
    baca_protocol_publisher.publish(msg_out);
    res.message = "Gripper stopped gripping";
    res.success = true;
    return true;

  } else {

    res.message = "Flight was aborted, gripper is disabled";
    res.success = false;
    return true;
  }
}

//}

/* callbackControlDiagnostics() //{ */

void MrsGripper::callbackControlDiagnostics(const mrs_msgs::ControlManagerDiagnostics &msg) {
  bool msg_in = msg.flying_normally;
  if (msg_in) {
    started_flying = true;
  }
  if (started_flying && !msg_in) {
    aborted = true;
  }
}

//}

// | ------------------------ routines ------------------------ |

/* main() //{ */

int main(int argc, char **argv) {

  ros::init(argc, argv, "mrs_gripper");
  MrsGripper gripper;
  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}

//}
