// clang: PetrFormat

#ifndef __BRICK_H__
#define __BRICK_H__

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nodelet/nodelet.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Range.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>
#include <mbzirc_msgs/DetectionType.h>

#include <dynamic_reconfigure/server.h>
#include <brick_detection/brick_detectionConfig.h>

#include <nav_msgs/Odometry.h>
//#include <Eigen/Dense>

namespace brick_detection
{

enum Brick_type
{
  RED_TYPE    = 1,
  GREEN_TYPE  = 2,
  BLUE_TYPE   = 3,
  ORANGE_TYPE = 4,
  WALL_TYPE   = 5,
  WHITE_TYPE  = 6,
  UGV_TYPE    = 66
};

typedef struct
{
  float           c_x, c_y, c_z;
  int             minX, minY, maxX, maxY;
  float           ch, cs, cv;
  int             n_corners;
  int             corner_x[4];
  int             corner_y[4];
  int             id;
  float           yaw;
  float           size;
  enum Brick_type type;
  float           reliability;
  int             last_time;
} brick;

enum detection_mode
{
  SCAN_ALL       = 0,
  FIND_RED_BRICK = 1,
  FIND_GREEN_BRICK = 2,
  FIND_BLUE_BRICK = 3,
  FIND_ORANGE_BRICK = 4,
  
  FIND_WALL        = 24,
  
  /*FIND_RED_WALL    = 9,
  FIND_GREEN_WALL  = 10,
  FIND_BLUE_WALL   = 11,
  FIND_ORANGE_WALL = 12,*/
  
  FIND_TO_RED      = 32,
  FIND_TO_GREEN    = 64,
  FIND_TO_BLUE     = 96,
  FIND_TO_ORANGE   = 128
  
};

extern float wall_offset;

void brick_init(void);
int find_bricks(cv::Mat src, brick* brick_array, int brick_array_size, int input_bricks, bool use_gui, bool gui_red, bool gui_green, bool gui_blue, float& distance, float sin_a, float sin_b, float cos_a, float cos_b, int mode);
int find_depth_bricks(cv::Mat src, brick* brick_array, int brick_array_size, int input_bricks, bool use_gui, float& distance, float odom_dist, float a, float b, float c, int mode, float &alf, float &bet, float garmin);
void set_camera_param(cv::Mat intr, cv::Mat dist, bool simul_);
void bluefox_load_mask(const char *mask_name);
void realsense_camera_param(float cx, float cy, float fx, float fy, bool simul);
void realsense_load_mask(const char *mask_name);

float yaw_dif(float y1, float y2);
float yaw_avg(float y1, float y2);
float yaw_update(float rel, float map);

#define BRICK_MAP_SIZE 64

#define BRICK_MAPS 4  

#define _RELATIVE_MAP 0
#define _OBJECT_MAP 1
#define _BF_MAP 2
void map_init(int m, int time_limit);
void map_update(int m);
int find_map_brick(int m, float x, float y, enum Brick_type type);
int add_map_brick(int m, float x, float y, float z, float yaw, enum Brick_type t);
brick *get_map_brick(int m, int i);
int valid_map_brick(int m, int i);

void diff_vec(Eigen::Vector3d a, Eigen::Vector3d b, double &d_alfa, double &d_beta);

typedef brick_detection::brick_detectionConfig cfg_t;

class BrickDetection : public nodelet::Nodelet {
public:
  virtual void onInit();

public:
  // Callback for the Bluefox RGB image
  void img_callback(const sensor_msgs::ImageConstPtr& img_msg);
  void cam_info_callback(const sensor_msgs::CameraInfoConstPtr& msg);
  std::mutex img_mutex;

  // Callback for the RGBd image
  void depth_callback(const sensor_msgs::ImageConstPtr& img_msg);
  void depth_info_callback(const sensor_msgs::CameraInfoConstPtr& msg);
  std::mutex depth_mutex;

private:
  ros::NodeHandle m_nh;
  std::unique_ptr<dynamic_reconfigure::Server<cfg_t>> m_param_server;
  dynamic_reconfigure::Server<cfg_t>::CallbackType m_reconfigure_callback;
  void reconf_callback(const cfg_t& config, [[maybe_unused]] uint32_t level);

private:
  bool m_initialized;
  bool m_busy = false;
  bool m_have_camera_info;
  bool m_have_depth_info;
  ros::Timer m_main_timer;
  void main_loop(const ros::TimerEvent& event);

  // parameters
private:
  std::string m_nodelet_name;
  std::string m_uav_name;

private:
  ros::Publisher m_brick_pub;
  ros::Publisher m_altitude_pub;
  ros::Publisher m_dbg_pub;
  ros::Publisher pub_debug_brick_;

  image_transport::Subscriber m_img_sub;
  ros::Subscriber m_cam_info_sub;
  
  image_transport::Subscriber m_depth_sub;
  ros::Subscriber m_depth_info_sub;
  ros::Subscriber m_garmin_sub;
  
  ros::Time last_garmin_stamp_;
  float current_garmin_range_;
  void garmin_callback(const sensor_msgs::RangeConstPtr &msg);

  
  bool callbackType(mbzirc_msgs::DetectionType::Request &req, mbzirc_msgs::DetectionType::Response &res);
  ros::ServiceServer service_type_;
  int current_mode;
  bool callbackLayer(mbzirc_msgs::DetectionType::Request& req, mbzirc_msgs::DetectionType::Response& res);
  ros::ServiceServer service_layer_;
  int current_layer;
  
  tf2_ros::Buffer                             m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
  
  int imageNumber = 0;
  int noinit_bf_uav_tf;
  int noinit_rs_uav_tf;
  Eigen::Affine3d bf_to_uav;
  Eigen::Affine3d rs_to_uav;

private:
/*  ros::Subscriber    subscriber_odometry;
  nav_msgs::Odometry odometry;
  double             odometry_x;
  double             odometry_y;
  double             odometry_z;
  double             odometry_yaw;
  double             odometry_roll;
  double             odometry_pitch;
  std::mutex         mutex_odometry;
  bool               got_odometry = false;
  void               callbackOdometry(const nav_msgs::OdometryConstPtr& msg); */

private:
  bool m_new_img;
  bool m_new_depth;
  sensor_msgs::ImageConstPtr m_last_img_msg;
  sensor_msgs::ImageConstPtr m_last_depth_msg;
  float last_altitude;
  float last_odom_altitude;
  float last_alf, last_bet;
  float calib_alf, calib_bet;
  int valid_last_angle;

private:  // camera parameters
  float cam_fx, cam_fy;
  float cam_cx, cam_cy;
  float depth_fx, depth_fy;
  float depth_cx, depth_cy;

private:
  mrs_lib::Profiler* m_profiler;
  bool profiler_enabled = false;

  bool get_transform_to_world(const std::string& frame_name, ros::Time stamp, Eigen::Affine3d& tf_out);
  
};

}  // namespace brick_detection

#endif
