// clang: PetrFormat

/* includes //{ */

#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <mbzirc_msgs/DetectedObject.h>
#include <mbzirc_msgs/ObjectWithType.h>
#include <mbzirc_msgs/ObjectWithTypeArray.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include "opencv2/ml/ml.hpp"

#include <dynamic_reconfigure/server.h>
#include <brick_detection/brick_detectionConfig.h>

#include <sys/time.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#include "brick.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <mrs_lib/param_loader.h>

//}

using namespace std;
using namespace cv;

#define BRICK_ARRAY_SIZE 256

namespace brick_detection
{

static brick brick_array[BRICK_ARRAY_SIZE];
static brick brick_array2[BRICK_ARRAY_SIZE];
static brick brick_relative2[BRICK_ARRAY_SIZE];

static string uav_name;
static string bf_mask_name;
static string rs_mask_name;

static bool gui = false;
static bool debug = false;
static bool simul = false;

static bool gui_green = false;
static bool gui_red = false;
static bool gui_blue = false;
static bool gui_dbg = false;

// static bool debug_detector = false;
// static bool debug_line = false;

static int detectedObjects = 0;
static int detectedDepthObjects = 0;

static Eigen::Vector3d rs_shift;

// LATER FILL FROM CONFIG FILE
Mat distCoeffs = Mat_<double>(1, 5);
Mat intrinsic = Mat_<double>(3, 3);

cv_bridge::CvImage cv_ptr;

double camera_yaw_offset = 0;
double camera_phi_offset = 0;
double camera_psi_offset = 0;
double camera_delay = 0.005;
double rs_delay = 0.01;

float bf_x, bf_y;

static int first_image;

static std::string target_frame;
static std::string frame_name_;  // fcu_uavXX
static std::string drone_body;   // fcu_uavXX

static float brick_expected_altitude_ = 0.20;

static tf::StampedTransform lastTransform;

static mbzirc_msgs::ObjectWithTypeArray object_with_type_array;

static ros::Publisher pub_dbg_rel_rs;
static ros::Publisher pub_dbg_rel_bf;
static ros::Publisher pub_dbg_rel;

static mutex mutex_garmin_;

#define my_sqr(a) ((a) * (a))
#define my_abs(a) ((a < 0) ? (-(a)) : (a))

static cv_bridge::CvImagePtr image;

/* constructor //{ */


/* m_busy        = false; */
/* imageNumber   = 0; */
//}

/* get_transform_to_world() //{ */

bool BrickDetection::get_transform_to_world(const std::string& frame_name, ros::Time stamp, Eigen::Affine3d& tf_out) {
  try {
    const ros::Duration timeout(1.0 / 100.0);
    geometry_msgs::TransformStamped transform = m_tf_buffer.lookupTransform(target_frame, frame_name, stamp, timeout);
    tf_out = tf2::transformToEigen(transform.transform);
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", frame_name.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
  return true;
}

//}

/* reconfigureCallback() //{ */

void reconfigureCallback([[maybe_unused]] brick_detection::brick_detectionConfig& config, [[maybe_unused]] uint32_t level) {

  // ROS_INFO("Reconfigure Request: %lf %lf %lf %lf %lf %lf %lf", config.outputImageIndex);
  /*outputImageIndex = config.outputImageIndex;
  manualThreshold  = config.manualThreshold;
  maxSegmentSize   = config.maxBlobSize;
  minSegmentSize   = config.minBlobSize;
  // segmentation.minCircularity = config.minCircularity;
  minRoundness                    = config.minRoundness;
  circleDiameter                  = config.objectDiameter;
  histogramScale                  = config.histogramScale;
  visualDistanceToleranceRatio    = config.visualDistanceToleranceRatio;
  visualDistanceToleranceAbsolute = config.visualDistanceToleranceAbsolute; */
  // longObjectDetection = config.longObject;
  //  ROS_INFO("Reconfigure Request min circularity, min convexity: %lf %lf %lf", config.minCircularity, config.manualThreshold,circleDiameter);
  // outerDimMaster = config.masterDiameter/100.0;
  // distanceTolerance = config.distanceTolerance/100.0;
  // detector->reconfigure(config.initialCircularityTolerance, config.finalCircularityTolerance,
  // config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs);
}

//}

/* img_callback() //{ */

void BrickDetection::img_callback(const sensor_msgs::ImageConstPtr& img_msg) {

  //  mrs_lib::Routine profiler_routine = m_profiler->createRoutine("imageCallback");
  ROS_INFO_THROTTLE(10.0, "[%s]: New image - OK", m_nodelet_name.c_str());
  img_mutex.lock();
  m_last_img_msg = img_msg;
  m_new_img = true;
  img_mutex.unlock();
}

//}

/* img_callback() //{ */

void BrickDetection::depth_callback(const sensor_msgs::ImageConstPtr& img_msg) {

  //  mrs_lib::Routine profiler_routine = m_profiler->createRoutine("imageCallback");
  ROS_INFO_THROTTLE(10.0, "[%s]: New Depth image - OK", m_nodelet_name.c_str());
  depth_mutex.lock();
  m_last_depth_msg = img_msg;
  m_new_depth = true;
  depth_mutex.unlock();
}

//}

/* depth_info_callback() //{ */

void BrickDetection::depth_info_callback(const sensor_msgs::CameraInfoConstPtr& msg) {

  float r_fx, r_fy, r_cx, r_cy;
  float k1, k2, k3, k4, k5;

  if (!m_initialized)
    return;

  if (msg->K.size() < 6) {
    ROS_ERROR_THROTTLE(10.0, "Realsense has wrong calibration matricies.");
  } else {

    r_fx = msg->K.at(0);
    r_fy = msg->K.at(4);
    r_cx = msg->K.at(2);
    r_cy = msg->K.at(5);

    if (msg->D.size() >= 5) {
      k1 = msg->D.at(0);
      k2 = msg->D.at(1);
      k3 = msg->D.at(2);
      k4 = msg->D.at(3);
      k5 = msg->D.at(4);
    } else {
      k1 = k2 = k3 = k4 = k5 = 0.0;
    }
    if (k1 != 0.0 || k2 != 0.0 || k3 != 0.0 || k4 != 0.0 || k5 != 0.0) {
      ROS_WARN_THROTTLE(10.0, "Realsense with non zero distortion parameters k1:%f k2:%f k3:%f k4:%f k5:%f", k1, k2, k3, k4, k5);
    } else {
      if (m_have_depth_info) {
        if (r_fx != depth_fx || r_fy != depth_fy || r_cx != depth_cx || r_cy != depth_cy) {
          ROS_WARN_THROTTLE(10.0, "Realsense CHANGE intrictic matrix, new parameters : fx:%f fy:%f cx:%f cy:%f", r_fx, r_fy, r_cx, r_cy);
          realsense_camera_param(r_cx, r_cy, r_fx, r_fy, simul);
        }
      } else {
        ROS_INFO("Realsense Camera params: fx:%f fy:%f cx:%f cy:%f", r_fx, r_fy, r_cx, r_cy);
        realsense_camera_param(r_cx, r_cy, r_fx, r_fy, simul);
      }
    }
    m_have_depth_info = true;
    depth_fx = r_fx;
    depth_fy = r_fy;
    depth_cx = r_cx;
    depth_cy = r_cy;
  }
}

//}

/* cam_info_callback() //{ */

void BrickDetection::cam_info_callback(const sensor_msgs::CameraInfoConstPtr& msg) {

  float n_fx, n_fy, n_cx, n_cy;
  float k1, k2, k3, k4, k5;

  if (!m_initialized)
    return;

  if (msg->K.size() < 6) {
    ROS_ERROR_THROTTLE(1.0, "Bluefox has wrong calibration matricies.");
    k1 = -2.621542e+02;
    k2 = 0.0;
    k3 = 1.344121e-03;
    k4 = -2.588143e-07;
    k5 = 2.749293e-09;
  } else {
    n_fx = msg->K.at(0);
    n_fy = msg->K.at(4);
    n_cx = msg->K.at(2);
    n_cy = msg->K.at(5);
    if (msg->D.size() >= 5) {
      k1 = msg->D.at(0);
      k2 = msg->D.at(1);
      k3 = msg->D.at(2);
      k4 = msg->D.at(3);
      k5 = msg->D.at(4);
    } else {
      k1 = -2.621542e+02;
      k2 = 0.0;
      k3 = 1.344121e-03;
      k4 = -2.588143e-07;
      k5 = 2.749293e-09;
    }
    if (m_have_camera_info) {
      if (n_fx != cam_fx || n_fy != cam_fy || n_cx != cam_cx || n_cy != cam_cy) {
        ROS_WARN_THROTTLE(1.0, "Bluefox CHANGE intrictic matrix, new parameters : fx:%f fy:%f cx:%f cy:%f", n_fx, n_fy, n_cx, n_cy);
        intrinsic = (cv::Mat_<float>(3, 3) << n_fx, 0.0, n_cx, 0.0, n_fy, n_cy, 0.0, 0.0, 1.0);
        distCoeffs = (cv::Mat_<float>(1, 5) << k1, k2, k3, k4, k5);
        set_camera_param(intrinsic, distCoeffs, simul);
      }
    } else {
      ROS_INFO("Bluefox Camera params: fx:%f fy:%f cx:%f cy:%f", n_fx, n_fy, n_cx, n_cy);
      intrinsic = (cv::Mat_<float>(3, 3) << n_fx, 0.0, n_cx, 0.0, n_fy, n_cy, 0.0, 0.0, 1.0);
      distCoeffs = (cv::Mat_<float>(1, 5) << k1, k2, k3, k4, k5);
      set_camera_param(intrinsic, distCoeffs, simul);
    }

    m_have_camera_info = true;
    cam_fx = n_fx;
    cam_fy = n_fy;
    cam_cx = n_cx;
    cam_cy = n_cy;
  }
}

//}

/* main_loop() //{ */

void BrickDetection::main_loop([[maybe_unused]] const ros::TimerEvent& event) {

  bool have_depth = false;
  if (!m_initialized) {
    return;
  }

  if (m_busy) {
    ROS_INFO_STREAM("[BrickDetector]: Busy!");
    return;
  }

  m_busy = true;

  if (!m_new_img) {
    ROS_INFO_THROTTLE(60.0, "[BrickDetector]: waiting for image");
    m_busy = false;
    return;
  }

  if (!m_have_camera_info) {
    ROS_INFO_THROTTLE(1.0, "[BrickDetector]: waiting for camera_info");
    m_busy = false;
    return;
  }

  if (current_garmin_range_ < 0) {
    ROS_INFO_THROTTLE(1.0, "[BrickDetector]: waiting for GARMIN");
    m_busy = false;
    return;
  }

  m_new_img = false;
  double t = (double)getTickCount();
  double t2 = t, t3 = t, t4;
  try {
    // mrs_lib::Routine profiler_routine = m_profiler->createRoutine("brickDetecting", 100, 0.01, event);

    map_update(_RELATIVE_MAP);
    map_update(_OBJECT_MAP);
    float garmin_val;
    float best_rel_x = 10.0;
    float best_rel_y = 10.0;
    float best_rel_yaw = 0.0;
    float best_rel = 10000.0;

    {
      std::scoped_lock lock(mutex_garmin_);

      if (current_garmin_range_ >= 0) {
        double garmin_time = (m_last_img_msg->header.stamp - last_garmin_stamp_).toSec();
        if (garmin_time > 2.0) {
          ROS_ERROR("No garmin data for %lf sec", garmin_time);
          current_garmin_range_ = -1000.0;
        }
      }
      garmin_val = current_garmin_range_;
    }
    Eigen::Matrix3d angle_cor(3, 3);
    angle_cor << 1, 0, 0, 0, 1, 0, 0, 0, 1;


    img_mutex.lock();
    sensor_msgs::ImageConstPtr img_msg = m_last_img_msg;
    cv_bridge::CvImageConstPtr ros_img = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
    img_mutex.unlock();
    int global_mode = current_mode;
    if ((current_mode & FIND_WALL) != 0) {
      global_mode |= (current_layer << 4);
    }


    float altitude, altitude_change;
    ros::Time stamp;
    if (simul) {
      stamp = img_msg->header.stamp;
    } else {
      stamp = img_msg->header.stamp - ros::Duration(camera_delay);
    }

    cv_bridge::CvImageConstPtr ros_depth;
    ros::Time depth_stamp;
    sensor_msgs::ImageConstPtr depth_msg;
    if (m_new_depth) {
      depth_mutex.lock();
      depth_msg = m_last_depth_msg;
      ros_depth = cv_bridge::toCvShare(depth_msg);
      if (simul) {
        depth_stamp = depth_msg->header.stamp;
      } else {
        depth_stamp = depth_msg->header.stamp - ros::Duration(rs_delay);
      }
      depth_mutex.unlock();
      have_depth = true;
    }
    depth_stamp = stamp;

    if (noinit_bf_uav_tf) {
      noinit_bf_uav_tf = 0;
      try {
        const ros::Duration timeout(0.1);
        geometry_msgs::TransformStamped transform = m_tf_buffer.lookupTransform(drone_body, img_msg->header.frame_id, stamp, timeout);
        // geometry_msgs::TransformStamped transform = m_tf_buffer.lookupTransform(drone_body, "bluefox_brick2", stamp, timeout);
        bf_to_uav = tf2::transformToEigen(transform.transform);
        Eigen::Vector3d orig;
        orig << 0, 0, 0;
        orig = bf_to_uav * orig;
      }
      catch (tf2::TransformException& ex) {
        ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", drone_body.c_str(), img_msg->header.frame_id.c_str(), ex.what());
        noinit_bf_uav_tf = 1;
      }
    }
    Eigen::Affine3d s2w_tf = Eigen::Affine3d::Identity();
    Eigen::Affine3d r2w_tf, w2r_tf;
    bool tf_ok;
    Eigen::Vector3d pos, r_pos, vec, ahead, vecy, vec_z, obj, tmp, garmin_pos;
    pos << 0, 0, 0;
    r_pos << 0, 0, 0;
    tf_ok = get_transform_to_world(img_msg->header.frame_id, stamp, s2w_tf);
    // tf_ok = get_transform_to_world("bluefox_brick2", stamp, s2w_tf);

    if (!tf_ok) {
      ROS_ERROR_STREAM("TF failed ");
      m_busy = false;
      pos[2] = 1.5;
      s2w_tf = Eigen::Affine3d::Identity();
      return;
    }

    if (have_depth) {
      if (noinit_rs_uav_tf) {
        noinit_rs_uav_tf = 0;
        try {
          const ros::Duration timeout(0.1);
          geometry_msgs::TransformStamped transform = m_tf_buffer.lookupTransform(drone_body, depth_msg->header.frame_id, stamp, timeout);
          // geometry_msgs::TransformStamped transform = m_tf_buffer.lookupTransform(drone_body, "rs_d435_uav64_depth_optical_frame2", stamp, timeout);
          rs_to_uav = tf2::transformToEigen(transform.transform);
          rs_shift << 0, 0, 0;
          rs_shift = rs_to_uav * rs_shift;
        }
        catch (tf2::TransformException& ex) {
          ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", drone_body.c_str(), depth_msg->header.frame_id.c_str(), ex.what());
          noinit_rs_uav_tf = 1;
        }
      }
      tf_ok = get_transform_to_world(depth_msg->header.frame_id, stamp, r2w_tf);
      // tf_ok = get_transform_to_world("rs_d435_uav64_depth_optical_frame2", stamp, r2w_tf);
      w2r_tf = r2w_tf.inverse();

      if (!tf_ok) {
        ROS_ERROR_STREAM("Realsense TF failed ");
        m_busy = false;
        pos[2] = 1.5;
        r2w_tf = Eigen::Affine3d::Identity();
        have_depth = false;
      }
    } else {
      ROS_WARN_THROTTLE(10.0, "NO DEPTH DATA");
    }
    pos = s2w_tf * pos;
    altitude = pos[2];
    if (last_odom_altitude > -200.0) {
      altitude_change = altitude - last_odom_altitude;
    } else {
      altitude_change = 0.0;
    }
    last_odom_altitude = altitude;

    if (garmin_val >= 0.0) {
      garmin_pos << 0, 0, garmin_val;
      garmin_pos = s2w_tf.linear() * garmin_pos;
      garmin_val = -garmin_pos[2];
    }
    // SW gimbal - compute roll, pitch, yaw - alfa, beta, gama
    vec << 0, 0, 1;
    vec = s2w_tf.linear() * vec;
    vecy << 0, 1, 0;
    vecy = s2w_tf.linear() * vecy;
    ahead << 1, 0, 0;
    ahead = s2w_tf.linear() * ahead;

    // s2w_tf.linear().getRPY(g_rol, g_pit, g_yaw);

    float gama = atan2(ahead[1], ahead[0]);
    float cos_g = cos(gama);
    float sin_g = sin(gama);

    float gx = cos_g * vec[0] + sin_g * vec[1];
    float gy = -sin_g * vec[0] + cos_g * vec[1];
    [[maybe_unused]] float orig_x = gx;
    [[maybe_unused]] float orig_y = gy;
    [[maybe_unused]] float orig_z = vec[2];

    float alfa = atan2(-gx, -vec[2]);
    float sa = sin(alfa + bf_x);
    float ca = cos(alfa + bf_x);
    float beta = atan2(gy, -gx * sa - vec[2] * ca);
    float cb = cos(beta + bf_y);
    float sb = sin(beta + bf_y);

    Eigen::Matrix3d grot(3, 3);
    grot << ca, 0, -sa, sa * sb, cb, ca * sb, sa * cb, -sb, ca * cb;
    Eigen::Matrix3d irot = s2w_tf.linear().transpose();

    tmp << 0, 0, 1;
    tmp = grot * (irot * tmp);
    // cout <<"Kontrola 0,0,1 - "<<tmp(0)<<", "<<tmp(1)<<", "<<tmp(2)<<endl;

    detectedDepthObjects = 0;
    int obj_num = 0;
    if (have_depth) {
      Eigen::Vector3d ahead_r, vec_r, vecy_r, vec_z2;
      r_pos = r2w_tf * r_pos;

      vec_r << 0, 0, 1;
      vec_r = r2w_tf.linear() * vec_r;
      vecy_r << 0, 1, 0;
      vecy_r = r2w_tf.linear() * vecy_r;
      ahead_r << 1, 0, 0;
      ahead_r = r2w_tf.linear() * ahead_r;

      float r_gama = atan2(ahead_r[1], ahead_r[0]);
      float r_cos_g = cos(r_gama);
      float r_sin_g = sin(r_gama);

      float rgx = r_cos_g * vec_r[0] + r_sin_g * vec_r[1];
      float rgy = -r_sin_g * vec_r[0] + r_cos_g * vec_r[1];
      [[maybe_unused]] float rorig_x = rgx;
      [[maybe_unused]] float rorig_y = rgy;
      [[maybe_unused]] float rorig_z = vec_r[2];

      float r_alfa = atan2(-rgx, -vec_r[2]);
      float r_sa = -sin(r_alfa);
      float r_ca = cos(r_alfa);
      float r_beta = atan2(-rgy, -rgx * r_sa - vec_r[2] * r_ca);
      [[maybe_unused]] float r_cb = cos(r_beta);
      [[maybe_unused]] float r_sb = -sin(r_beta);

      // TODO control that rot==rot2
      Eigen::Matrix3d rot = r2w_tf.linear().inverse();
      [[maybe_unused]] Eigen::Matrix3d rot2 = r2w_tf.linear().transpose();

      float c_sa = sin(last_alf + calib_alf);
      float c_ca = cos(last_alf + calib_alf);
      float c_sb = sin(last_bet + calib_bet);
      float c_cb = cos(last_bet + calib_bet);
      angle_cor << c_ca, 0, -c_sa, -c_sa * c_sb, c_cb, -c_ca * c_sb, c_sa * c_cb, c_sb, c_ca * c_cb;
      rot = angle_cor * rot;

      vec_z << 0, 0, 1;
      vec_z = rot * vec_z;
      vec_z *= -1;
      // grot = rot.transpose();

      ROS_DEBUG("BF ahead %f, %f, %f RS ahead %f, %f, %f BF right %f, %f, %f RS right %f, %f, %f BF down %f, %f, %f RS down %f, %f, %f tf: %s", ahead[0], ahead[1], ahead[2],
                ahead_r[0], ahead_r[1], ahead_r[2], vecy[0], vecy[1], vecy[2], vecy_r[0], vecy_r[1], vecy_r[2], vec[0], vec[1], vec[2], vec_r[0], vec_r[1], vec_r[2],
                depth_msg->header.frame_id.c_str());
      // ROS_INFO("BF down %f, %f, %f RS down %f, %f, %f Gama %f RS gama %f",  gx, gy, vec[2], rgx, rgy, vec_r[2], gama, r_gama);

      // ROS_INFO("Plane eq: %f, %f, %f  last %f, odom %f", vec_z[0], vec_z[1], vec_z[2], last_altitude, altitude);

      if (last_altitude < -200.0) {
        last_altitude = altitude;
      } else {
        last_altitude += altitude_change;
      }
      float alf, bet;
      for (int i = 0; i < BRICK_MAP_SIZE; i++) {
        if (valid_map_brick(_OBJECT_MAP, i)) {
          brick_array2[obj_num] = *get_map_brick(_OBJECT_MAP, i);
          brick_array[obj_num] = brick_array2[obj_num];
          Eigen::Vector3d tmp_vec;
          tmp_vec << brick_array2[obj_num].c_x, brick_array2[obj_num].c_y, brick_array2[obj_num].c_z;
          tmp_vec = w2r_tf * tmp_vec;
          brick_array2[obj_num].c_x = tmp_vec[0];
          brick_array2[obj_num].c_y = tmp_vec[1];
          brick_array2[obj_num].c_z = tmp_vec[2];
          brick_array2[obj_num].yaw = gama - brick_array2[obj_num].yaw;

          float g_x = brick_array[obj_num].c_x - pos[0];
          float g_y = pos[1] - brick_array[obj_num].c_y;
          brick_array[obj_num].c_x = cos_g * g_x - sin_g * g_y;
          brick_array[obj_num].c_y = sin_g * g_x + cos_g * g_y;
          brick_array[obj_num].yaw = gama - brick_array[obj_num].yaw;
          obj_num++;
        }
      }
      t2 = (double)getTickCount();
      detectedDepthObjects = find_depth_bricks(ros_depth->image, brick_array2, BRICK_ARRAY_SIZE, obj_num, gui_dbg, last_altitude, altitude, vec_z[0], vec_z[1], vec_z[2],
                                               global_mode, alf, bet, garmin_val);

      ROS_DEBUG("Last alf:%f bet:%f new alf: %f bet:%f ", last_alf, last_bet, alf, bet);
      if (alf < 0.02 && alf > -0.02 && bet < 0.02 && bet > -0.02) {
        if (valid_last_angle) {
          last_alf += 0.2 * alf;
          last_bet += 0.2 * bet;
        } else {
          last_alf = 0.5 * alf;
          last_bet = 0.5 * bet;
          valid_last_angle = 1;
        }
        if (last_alf > 0.09 && last_alf < -0.09 && last_bet > 0.09 && bet < -0.09) {
          last_alf = 0.0;
          last_bet = 0.0;
          valid_last_angle = 0;
        }
      } else {
        last_alf = 0.0;
        last_bet = 0.0;
        valid_last_angle = 0;
      }
      last_alf = 0.0;
      last_bet = 0.0;

      sensor_msgs::Range alt;
      alt.header.frame_id = drone_body;
      alt.header.stamp = depth_stamp;
      if (noinit_rs_uav_tf == 0) {
        last_altitude -= rs_shift[2];
      } else {
        last_altitude += 0.04;
      }

      alt.range = last_altitude;
      alt.min_range = 0.10;
      alt.max_range = 15.0;
      alt.radiation_type = sensor_msgs::Range::INFRARED;
      alt.field_of_view = 1.0;
      try {
        m_altitude_pub.publish(alt);
      }
      catch (...) {
      }

    } else {
      // TODO add garmin data - last_altitude = altitude;
    }

    t3 = (double)getTickCount();
    detectedObjects = find_bricks(ros_img->image, brick_array, BRICK_ARRAY_SIZE, obj_num, gui_dbg, gui_red, gui_green, gui_blue, last_altitude, sa, sb, ca, cb, global_mode);

    ROS_DEBUG_STREAM("Detected obejects:" << detectedObjects << " position:" << pos[0] << "," << pos[1] << "," << pos[2] << " stamp:" << stamp << " gama " << gama << " alfa "
                                          << alfa << " beta " << beta);
    ROS_DEBUG_STREAM("Detected obejects:" << detectedDepthObjects << " r posit:" << r_pos[0] << "," << r_pos[1] << "," << r_pos[2] << " depth alt:" << last_altitude
                                          << " odom alt:" << altitude << " garmin:" << garmin_val);

    mbzirc_msgs::ObjectWithTypeArray object_with_type_array;
    tf::Quaternion orientation;
    geometry_msgs::PoseArray brick_dbg_array;
    brick_dbg_array.header.stamp = stamp;
    brick_dbg_array.header.frame_id = target_frame;
    object_with_type_array.header.stamp = stamp;
    object_with_type_array.header.frame_id = target_frame;

    for (int i = 0; i < detectedDepthObjects; i++) {

      // | --------------- objects in the world frame --------------- |

      // publish array of poses with type, needed byt landing object estimator
      mbzirc_msgs::ObjectWithType object_with_type;

      float ox = obj[0] = brick_array2[i].c_x;
      float oy = obj[1] = brick_array2[i].c_y;
      float oz = obj[2] = brick_array2[i].c_z;
      Eigen::Vector3d abs, rel;
      abs = r2w_tf * obj;
      /*abs = grot*obj;
      abs[0]+=pos[0];
      abs[1]+=pos[1];
      abs[2]+=last_altitude;*/

      brick_array2[i].c_x = abs[0];
      brick_array2[i].c_y = abs[1];
      brick_array2[i].c_z = abs[2];
      if (!noinit_rs_uav_tf) {
        rel = grot * angle_cor.transpose() * rs_to_uav * obj;
        brick_relative2[i].c_x = rel[0];
        brick_relative2[i].c_y = rel[1];
        brick_relative2[i].c_z = rel[2];
        brick_relative2[i].yaw = brick_array2[i].yaw;
      }
      brick_array2[i].yaw = gama - brick_array2[i].yaw;

      ROS_DEBUG_STREAM("Depth Object:" << i << " type:" << brick_array2[i].type << " pos:" << abs[0] << "," << abs[1] << ", " << abs[2] << " yaw " << brick_array2[i].yaw
                                       << " rel:" << brick_relative2[i].c_x << "," << brick_relative2[i].c_y << "," << brick_relative2[i].c_z << " orig:" << ox << "," << oy << ","
                                       << oz << " sh:" << rs_shift[0] << "," << rs_shift[1]);
      if (brick_array2[i].type == WALL_TYPE) {
        mbzirc_msgs::ObjectWithType object_with_type;
        object_with_type.x = brick_array2[i].c_x;
        object_with_type.y = brick_array2[i].c_y;
        object_with_type.z = brick_array2[i].c_z;
        object_with_type.type = brick_array2[i].type;
        object_with_type.yaw = brick_array2[i].yaw;
        object_with_type.len = brick_array2[i].size;
        object_with_type.stamp = stamp;
        object_with_type_array.objects.push_back(object_with_type);  // needed for estimation
        geometry_msgs::Pose pose;
        pose.position.x = object_with_type.x;
        pose.position.y = object_with_type.y;
        pose.position.z = object_with_type.z;
        orientation.setEuler(0, 0, object_with_type.yaw);
        pose.orientation.x = orientation.x();
        pose.orientation.y = orientation.y();
        pose.orientation.z = orientation.z();
        pose.orientation.w = orientation.w();
        brick_dbg_array.poses.push_back(pose);

        int map_i = find_map_brick(_OBJECT_MAP, object_with_type.x, object_with_type.y, WALL_TYPE);
        if ((map_i >= 0) && (my_sqr(object_with_type.x - get_map_brick(_OBJECT_MAP, map_i)->c_x) + my_sqr(object_with_type.y - get_map_brick(_OBJECT_MAP, map_i)->c_y) > 0.5)) {
          ROS_DEBUG_STREAM("Nearest OBJ:" << map_i << " dist:" << (object_with_type.x - get_map_brick(_OBJECT_MAP, map_i)->c_x) << ","
                                          << (object_with_type.y - get_map_brick(_OBJECT_MAP, map_i)->c_y));
          map_i = -1;
        }
        if (map_i < 0) {
          add_map_brick(_OBJECT_MAP, object_with_type.x, object_with_type.y, object_with_type.z, object_with_type.yaw, brick_array2[i].type);
          ROS_DEBUG("Added to OBJECT MAP %f, %f yaw %f", object_with_type.x, object_with_type.y, object_with_type.yaw);
        } else {
          get_map_brick(_OBJECT_MAP, map_i)->yaw = object_with_type.yaw;
          get_map_brick(_OBJECT_MAP, map_i)->c_x = object_with_type.x;
          get_map_brick(_OBJECT_MAP, map_i)->c_y = object_with_type.y;
          get_map_brick(_OBJECT_MAP, map_i)->c_z = object_with_type.z;
          ROS_DEBUG("Updated in OBJECT MAP %f, %f yaw %f", object_with_type.x, object_with_type.y, object_with_type.yaw);
        }
      }
    }


    // calculate global coords of the objects
    for (int i = 0; i < detectedObjects; i++) {

      // | --------------- objects in the world frame --------------- |

      // publish array of poses with type, needed byt landing object estimator
      mbzirc_msgs::ObjectWithType object_with_type;
      mbzirc_msgs::ObjectUavOdometry object_uav_odometry;

      obj[0] = brick_array[i].c_x;
      obj[1] = brick_array[i].c_y;
      obj[2] = brick_array[i].c_z;
      obj = s2w_tf * obj;
      [[maybe_unused]] float tt_x = obj[0];
      [[maybe_unused]] float tt_y = obj[1];
      object_with_type.x = pos[0] + cos_g * brick_array[i].c_x + sin_g * brick_array[i].c_y;
      object_with_type.y = pos[1] + sin_g * brick_array[i].c_x - cos_g * brick_array[i].c_y;
      object_with_type.z = brick_expected_altitude_;
      object_with_type.type = brick_array[i].type;
      object_with_type.yaw = gama - brick_array[i].yaw;

      // ROS_INFO("Cmp rotation %f, %f a gama  %f, %f", tt_x, tt_y, object_with_type.x, object_with_type.y);

      int find_depth = 0;
      int fill_rel = 0;
      float rel_x = 0.0, rel_y = 0.0, rel_yaw = 0.0;

      for (int ii = 0; ii < detectedDepthObjects; ii++) {
        if (my_sqr(object_with_type.x - brick_array2[ii].c_x) + my_sqr(object_with_type.y - brick_array2[ii].c_y) < 0.04 &&
            (brick_array[i].type == brick_array2[ii].type || brick_array[i].type == brick_array2[ii].type + 1 || brick_array[i].type + 1 == brick_array2[ii].type)) {

          int map_i = find_map_brick(_OBJECT_MAP, object_with_type.x, object_with_type.y, brick_array[i].type);
          float r_k = 0.7;
          float b_k;
          if (map_i >= 0) {
            float d_b = my_sqr(object_with_type.x - get_map_brick(_OBJECT_MAP, map_i)->c_x) + my_sqr(object_with_type.y - get_map_brick(_OBJECT_MAP, map_i)->c_y);
            float d_r = my_sqr(brick_array2[ii].c_x - get_map_brick(_OBJECT_MAP, map_i)->c_x) + my_sqr(brick_array2[ii].c_y - get_map_brick(_OBJECT_MAP, map_i)->c_y);
            if (d_r > 4 * d_b) {
              if (d_r > 8 * d_b) {
                r_k = 0.21;
              } else {
                r_k = 0.5;
              }
            }
            if ((d_r > 0.1) && (d_b > 0.1)) {
              ROS_DEBUG_STREAM("CANCEL Nearest OBJ:" << map_i << " dist:" << (object_with_type.x - get_map_brick(_OBJECT_MAP, map_i)->c_x) << ","
                                                     << (object_with_type.y - get_map_brick(_OBJECT_MAP, map_i)->c_y));
              map_i = -1;
            }
          }

          float yaw = brick_array2[ii].yaw;
          float y_br = yaw_dif(object_with_type.yaw, brick_array2[ii].yaw);
          if (my_abs(y_br) < (M_PI / 6.0)) {
            yaw = yaw_avg(object_with_type.yaw, brick_array2[ii].yaw);
          }
          if (map_i >= 0) {
            float y_m = get_map_brick(_OBJECT_MAP, map_i)->yaw;
            float yd_b = yaw_dif(object_with_type.yaw, y_m);
            float yd_r = yaw_dif(brick_array2[ii].yaw, y_m);
            if ((my_abs(yd_b) > (M_PI / 6.0)) && (my_abs(yd_r) < (M_PI / 6.0))) {
              yaw = brick_array2[ii].yaw;
            } else if ((my_abs(yd_b) < (M_PI / 6.0)) && (my_abs(yd_r) > (M_PI / 6.0))) {
              yaw = object_with_type.yaw;
              r_k = 0.2;
            }
          } else {
            b_k = 1.0 - r_k;
            add_map_brick(_OBJECT_MAP, object_with_type.x * b_k + brick_array2[ii].c_x * r_k, object_with_type.y * b_k + brick_array2[ii].c_y * r_k, object_with_type.z, yaw,
                          brick_array[i].type);
            ROS_DEBUG("Added to OBJECT MAP %f, %f yaw %f", object_with_type.x, object_with_type.y, object_with_type.yaw);
          }

          ROS_DEBUG_STREAM("FUSE DEPTH + Object:" << i << " type:" << object_with_type.type << " pos:" << object_with_type.x << "(" << brick_array2[ii].c_x << ") ,"
                                                  << object_with_type.y << "(" << brick_array2[ii].c_y << ") yaw " << object_with_type.yaw << " f_yaw:" << yaw << " b_k:" << b_k
                                                  << " r_k:" << r_k);
          b_k = 1.0 - r_k;

          object_with_type.x = object_with_type.x * b_k + brick_array2[ii].c_x * r_k;
          object_with_type.y = object_with_type.y * b_k + brick_array2[ii].c_y * r_k;
          object_with_type.z = brick_array2[ii].c_z;
          object_with_type.yaw = yaw;

          if (map_i >= 0) {
            get_map_brick(_OBJECT_MAP, map_i)->yaw = object_with_type.yaw;
            get_map_brick(_OBJECT_MAP, map_i)->c_x = object_with_type.x;
            get_map_brick(_OBJECT_MAP, map_i)->c_y = object_with_type.y;
            get_map_brick(_OBJECT_MAP, map_i)->c_z = object_with_type.z;
          }
          if (!noinit_bf_uav_tf && !noinit_rs_uav_tf) {
            obj[0] = brick_array[i].c_x;
            obj[1] = brick_array[i].c_y;
            obj[2] = brick_array[i].c_z;
            obj = bf_to_uav * obj;
            rel_x = obj[0];
            rel_y = obj[1];

            /*            geometry_msgs::Vector3 v;
                        v.x = rel_x;
                        v.y = rel_y;
                        v.z = brick_array[i].yaw;
                        try {
                          pub_dbg_rel_bf.publish(v);
                        }
                        catch (...) {
                        }

                        v.x = brick_relative2[ii].c_x;
                        v.y = brick_relative2[ii].c_y;
                        v.z = brick_relative2[ii].yaw;
                        try {
                          pub_dbg_rel_rs.publish(v);
                        }
                        catch (...) {
                        }
            */

            ROS_DEBUG_STREAM("FUSE relative obj:" << rel_x << "," << rel_y << " depth:" << brick_relative2[ii].c_x << "," << brick_relative2[ii].c_y
                                                  << " yaw bf:" << brick_array[i].yaw << " rs:" << brick_relative2[ii].yaw);

            rel_x = 0.7 * rel_x + 0.3 * brick_relative2[ii].c_x;
            rel_y = 0.7 * rel_y + 0.3 * brick_relative2[ii].c_y;

            int map_i = find_map_brick(_RELATIVE_MAP, rel_x, rel_y, brick_array[i].type);
            if ((map_i >= 0) && (my_sqr(rel_x - get_map_brick(_RELATIVE_MAP, map_i)->c_x) + my_sqr(rel_y - get_map_brick(_RELATIVE_MAP, map_i)->c_y) > 0.1)) {
              ROS_DEBUG_STREAM("CANCEL MAP Nearest OBJ:" << map_i << " dist:" << (rel_x - get_map_brick(_OBJECT_MAP, map_i)->c_x) << ","
                                                         << (rel_y - get_map_brick(_OBJECT_MAP, map_i)->c_y));
              map_i = -1;
            }
            if (map_i < 0) {
              float y_rs_bf_d = yaw_dif(brick_array[i].yaw, brick_relative2[ii].yaw);
              if (my_abs(y_rs_bf_d) < (M_PI / 8.0)) {
                rel_yaw = yaw_avg(brick_array[i].yaw, brick_relative2[ii].yaw);
              } else {
                rel_yaw = brick_array[i].yaw;
              }
              add_map_brick(_RELATIVE_MAP, rel_x, rel_y, last_altitude - brick_expected_altitude_, rel_yaw, brick_array[i].type);
              ROS_DEBUG("Added to MAP %f, %f yaw %f", rel_x, rel_y, rel_yaw);
            } else {
              float map_yaw = get_map_brick(_RELATIVE_MAP, map_i)->yaw;
              ROS_DEBUG("Get RELATIVEMAP %i : %f, %f yaw %f", map_i, get_map_brick(_RELATIVE_MAP, map_i)->c_x, get_map_brick(_RELATIVE_MAP, map_i)->c_y, map_yaw);

              if (last_altitude < 1.5) {
                float y_rs_bf_d = yaw_dif(brick_array[i].yaw, brick_relative2[ii].yaw);
                ROS_DEBUG("Diff yaw bf rs %f", y_rs_bf_d);
                if (my_abs(y_rs_bf_d) < (M_PI / 20.0)) {
                  rel_yaw = yaw_avg(brick_array[i].yaw, brick_relative2[ii].yaw);
                  float y_new_d = yaw_dif(rel_yaw + M_PI_2, map_yaw);
                  if (my_abs(y_new_d) < (M_PI / 4.0)) {
                    ROS_DEBUG("SOJKA pi/2 switch, avg %f, map %f, diff +pi/2 %f", rel_yaw, map_yaw, y_new_d);
                    rel_yaw += M_PI_2;
                  } else {
                    ROS_DEBUG("Only avg %f, map %f, diff +pi/2 %f", rel_yaw, map_yaw, y_new_d);
                  }
                } else {
                  float bf_yaw_d = yaw_dif(brick_array[i].yaw, map_yaw);
                  float rs_yaw_d = yaw_dif(brick_array2[ii].yaw, map_yaw);
                  ROS_DEBUG("Diff yaw vs. map bf %f rs %f", bf_yaw_d, rs_yaw_d);
                  if (my_abs(bf_yaw_d) < my_abs(rs_yaw_d)) {
                    rel_yaw = brick_array[i].yaw;
                    float y_new = yaw_dif(brick_array[i].yaw, brick_relative2[ii].yaw + M_PI_2);
                    if (my_abs(y_new) < (M_PI / 16.0)) {
                      rel_yaw = yaw_avg(brick_array[i].yaw, brick_relative2[ii].yaw + M_PI_2);
                      ROS_DEBUG("SOJKA pi/2 switch rs, avg %f, map %f, diff +pi/2 %f", rel_yaw, map_yaw, y_new);
                    }
                  } else {
                    rel_yaw = brick_array2[ii].yaw;
                    float y_new_d = yaw_dif(brick_array[i].yaw + M_PI_2, brick_relative2[ii].yaw);
                    if (my_abs(y_new_d) < (M_PI / 16.0)) {
                      rel_yaw = yaw_avg(brick_array[i].yaw + M_PI_2, brick_relative2[ii].yaw);
                      ROS_DEBUG("SOJKA pi/2 switch bf, avg %f, map %f, diff +pi/2 %f", rel_yaw, map_yaw, y_new_d);
                    }
                  }
                }
                rel_yaw = yaw_update(rel_yaw, map_yaw);
              } else {
                float y_rs_bf_d = yaw_dif(brick_array[i].yaw, brick_relative2[ii].yaw);
                if (my_abs(y_rs_bf_d) < (M_PI / 20.0)) {
                  rel_yaw = yaw_avg(brick_array[i].yaw, brick_relative2[ii].yaw);
                } else {
                  float bf_yaw_d = yaw_dif(brick_array[i].yaw, map_yaw);
                  float rs_yaw_d = yaw_dif(brick_array2[ii].yaw, map_yaw);
                  if (my_abs(bf_yaw_d) < my_abs(rs_yaw_d)) {
                    rel_yaw = brick_array[i].yaw;
                    float y_new = yaw_dif(brick_array[i].yaw, brick_relative2[ii].yaw + M_PI_2);
                    if (my_abs(y_new) < (M_PI / 16.0)) {
                      rel_yaw = yaw_avg(brick_array[i].yaw, brick_relative2[ii].yaw + M_PI_2);
                    }
                  } else {
                    rel_yaw = brick_array2[ii].yaw;
                    float y_new = yaw_dif(brick_array[i].yaw + M_PI_2, brick_relative2[ii].yaw);
                    if (my_abs(y_new) < (M_PI / 16.0)) {
                      rel_yaw = yaw_avg(brick_array[i].yaw + M_PI_2, brick_relative2[ii].yaw);
                    }
                  }
                  rel_yaw = yaw_update(rel_yaw, map_yaw);
                }
              }
              get_map_brick(_RELATIVE_MAP, map_i)->yaw = rel_yaw;
              get_map_brick(_RELATIVE_MAP, map_i)->c_x = rel_x;
              get_map_brick(_RELATIVE_MAP, map_i)->c_y = rel_y;
              get_map_brick(_RELATIVE_MAP, map_i)->c_z = last_altitude - brick_expected_altitude_;
              ROS_DEBUG("Updated in RELATIVEMAP %i to %f, %f yaw %f -> %f", map_i, rel_x, rel_y, map_yaw, rel_yaw);
            }

            if (my_sqr(rel_x) + my_sqr(rel_y) < best_rel) {
              best_rel = my_sqr(rel_x) + my_sqr(rel_y);
              best_rel_x = rel_x;
              best_rel_y = rel_y;
              best_rel_yaw = rel_yaw;
            }

            fill_rel = 1;
          } else {
            ROS_WARN_THROTTLE(5.0, "No bf or rs to fcu static transform");
          }

          find_depth = 1;
          break;
        }
      }
      if (find_depth == 0) {
        ROS_DEBUG_STREAM("NO DEPTH Object:" << i << " type:" << object_with_type.type << " pos:" << object_with_type.x << "," << object_with_type.y << " yaw "
                                            << object_with_type.yaw << " rel:" << brick_array[i].c_x << "," << brick_array[i].c_y);

        int map_i = find_map_brick(_OBJECT_MAP, object_with_type.x, object_with_type.y, brick_array[i].type);
        if ((map_i >= 0) && (my_sqr(object_with_type.x - get_map_brick(_OBJECT_MAP, map_i)->c_x) + my_sqr(object_with_type.y - get_map_brick(_OBJECT_MAP, map_i)->c_y) > 0.1)) {
          ROS_DEBUG_STREAM("Nearest OBJ:" << map_i << " dist:" << (object_with_type.x - get_map_brick(_OBJECT_MAP, map_i)->c_x) << ","
                                          << (object_with_type.y - get_map_brick(_OBJECT_MAP, map_i)->c_y));
          map_i = -1;
        }
        if (map_i < 0) {
          add_map_brick(_OBJECT_MAP, object_with_type.x, object_with_type.y, object_with_type.z, object_with_type.yaw, brick_array[i].type);
          ROS_DEBUG("Added to OBJECT MAP %f, %f yaw %f", object_with_type.x, object_with_type.y, object_with_type.yaw);
        } else {
          get_map_brick(_OBJECT_MAP, map_i)->yaw = object_with_type.yaw;
          get_map_brick(_OBJECT_MAP, map_i)->c_x = object_with_type.x;
          get_map_brick(_OBJECT_MAP, map_i)->c_y = object_with_type.y;
          get_map_brick(_OBJECT_MAP, map_i)->c_z = object_with_type.z;
          ROS_DEBUG("Updated in OBJECT MAP %f, %f yaw %f", object_with_type.x, object_with_type.y, object_with_type.yaw);
        }

        if (!noinit_bf_uav_tf) {
          obj[0] = brick_array[i].c_x;
          obj[1] = brick_array[i].c_y;
          obj[2] = brick_array[i].c_z;
          obj = bf_to_uav * obj;
          rel_x = obj[0];
          rel_y = obj[1];

          ROS_DEBUG_STREAM("NO DEPTH relative obj:" << rel_x << "," << rel_y);
          /*          geometry_msgs::Vector3 v;
                    v.x = rel_x;
                    v.y = rel_y;
                    v.z = brick_array[i].yaw;
                    try {
                      pub_dbg_rel_bf.publish(v);
                    }
                    catch (...) {
                    }
          */
          int map_i = find_map_brick(_RELATIVE_MAP, rel_x, rel_y, brick_array[i].type);
          if ((map_i >= 0) && (my_sqr(rel_x - get_map_brick(_RELATIVE_MAP, map_i)->c_x) + my_sqr(rel_y - get_map_brick(_RELATIVE_MAP, map_i)->c_y) > 0.1)) {
            ROS_DEBUG_STREAM("CANCEL MAP Nearest OBJ:" << map_i << " dist:" << (rel_x - get_map_brick(_OBJECT_MAP, map_i)->c_x) << ","
                                                       << (rel_y - get_map_brick(_OBJECT_MAP, map_i)->c_y));
            map_i = -1;
          }
          if (map_i < 0) {
            add_map_brick(_RELATIVE_MAP, rel_x, rel_y, last_altitude - brick_expected_altitude_, brick_array[i].yaw, brick_array[i].type);
            ROS_DEBUG("Added to RELATIVE MAP %f, %f yaw %f", rel_x, rel_y, rel_yaw);
          } else {
            float map_yaw = get_map_brick(_RELATIVE_MAP, map_i)->yaw;
            ROS_DEBUG("Get RELATIVEMAP %i : %f, %f yaw %f", map_i, get_map_brick(_RELATIVE_MAP, map_i)->c_x, get_map_brick(_RELATIVE_MAP, map_i)->c_y, map_yaw);

            if (last_altitude < 1.5) {
              rel_yaw = brick_array[i].yaw;
              float y_new_d = yaw_dif(rel_yaw + M_PI_2, map_yaw);
              if (my_abs(y_new_d) < (M_PI / 4.0)) {
                ROS_DEBUG("SOJKA pi/2 switch bf %f, map %f, diff +pi/2 %f", brick_array[i].yaw, map_yaw, y_new_d);
                rel_yaw += M_PI_2;
              }
            } else {
              rel_yaw = brick_array[i].yaw;
            }
            get_map_brick(_RELATIVE_MAP, map_i)->yaw = rel_yaw;
            get_map_brick(_RELATIVE_MAP, map_i)->c_x = rel_x;
            get_map_brick(_RELATIVE_MAP, map_i)->c_y = rel_y;
            get_map_brick(_RELATIVE_MAP, map_i)->c_z = last_altitude - brick_expected_altitude_;
            ROS_DEBUG("Updated in RELATIVE MAP %f, %f yaw %f ->%f", rel_x, rel_y, map_yaw, rel_yaw);
          }
          if (my_sqr(rel_x) + my_sqr(rel_y) < best_rel) {
            best_rel = my_sqr(rel_x) + my_sqr(rel_y);
            best_rel_x = rel_x;
            best_rel_y = rel_y;
            best_rel_yaw = rel_yaw;
          }
          fill_rel = 1;
        } else {
          ROS_WARN_THROTTLE(5.0, "No bf to fcu static transform");
        }
      }

      object_with_type.stamp = stamp;

      geometry_msgs::Pose pose;
      pose.position.x = object_with_type.x;
      pose.position.y = object_with_type.y;
      pose.position.z = object_with_type.z;

      orientation.setEuler(0, 0, object_with_type.yaw);
      pose.orientation.x = orientation.x();
      pose.orientation.y = orientation.y();
      pose.orientation.z = orientation.z();
      pose.orientation.w = orientation.w();

      brick_dbg_array.poses.push_back(pose);

      object_with_type_array.objects.push_back(object_with_type);  // needed for estimation

      // | ---------------- uav in the objects frame ---------------- |

      // fill in the uav odometry in the object frame of reference

      if (fill_rel) {

        float sin_y = sin(rel_yaw);
        float cos_y = cos(rel_yaw);

        rel_x *= -1;
        rel_y *= -1;

        // S timto to najednou zacalo fungovat fungovat
        // rel_x += odometry_z * sin(odometry_pitch);
        // rel_y += odometry_z * sin(-odometry_roll);

        object_uav_odometry.header.stamp = stamp;
        object_uav_odometry.header.frame_id = "brick_origin";
        object_uav_odometry.x = cos_y * rel_x - sin_y * rel_y;
        object_uav_odometry.y = sin_y * rel_x + cos_y * rel_y;
        object_uav_odometry.z = last_altitude - brick_expected_altitude_;
        object_uav_odometry.yaw = rel_yaw;



        ROS_DEBUG("Final rel position %f, %f yaw %f size %f", object_uav_odometry.x, object_uav_odometry.y, object_uav_odometry.yaw, sqrt(my_sqr(rel_x) + my_sqr(rel_y)));
        object_with_type_array.uav_odometries.push_back(object_uav_odometry);
      }
    }

    try {
      m_brick_pub.publish(object_with_type_array);
      if (detectedObjects > 0) {
        pub_debug_brick_.publish(brick_dbg_array);
      }
      geometry_msgs::Vector3 v;
      v.x = best_rel_x;
      v.y = best_rel_y;
      v.z = best_rel_yaw;
      pub_dbg_rel.publish(v);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", m_brick_pub.getTopic().c_str());
    }

    imageNumber++;
  }
  catch (...) {
    m_busy = false;
  }
  t4 = (double)getTickCount();
  ROS_INFO_STREAM("BRICK total time " << ((t4 - t) * 1000. / getTickFrequency()) << "ms init:" << ((t2 - t) * 1000. / getTickFrequency())
                                      << " rs:" << ((t3 - t2) * 1000. / getTickFrequency()) << " color:" << ((t4 - t3) * 1000. / getTickFrequency())
                                      << "ms detected obejects bf:" << detectedObjects << " rs:" << detectedDepthObjects << " alt:" << last_altitude);
  m_busy = false;
}

//}

/* garmin_callback //{ */

void BrickDetection::garmin_callback(const sensor_msgs::RangeConstPtr& msg) {
  if (std::isfinite(msg->range)) {
    std::scoped_lock lock(mutex_garmin_);
    last_garmin_stamp_ = msg->header.stamp;
    current_garmin_range_ = msg->range;
    ROS_INFO_THROTTLE(10.0, "[BrickDetector]: Received GARMIN val %f", current_garmin_range_);
  }
}

//}


/* callbackType() //{ */

static string brick_str[] = {"ALL", "RED", "GREEN", "BLUE", "ORANGE", "-5-", "-6-", "-7-"};

bool BrickDetection::callbackType(mbzirc_msgs::DetectionType::Request& req, mbzirc_msgs::DetectionType::Response& res) {
  current_mode = req.type;

  res.success = true;
  res.message = "New type " + to_string(current_mode) + " " + (((current_mode & FIND_WALL) != 0) ? "WALL " : "BRICK ") + brick_str[current_mode & 7];

  ROS_INFO("[BrickDetection]: vision mode set to \"%d\"", current_mode);

  return true;
}

//}

/* callbackLayer() //{ */

bool BrickDetection::callbackLayer(mbzirc_msgs::DetectionType::Request& req, mbzirc_msgs::DetectionType::Response& res) {
  current_layer = req.type;

  res.success = true;
  res.message = "Set layer " + to_string(current_layer);

  ROS_INFO("[BrickDetection]: Set layer to  \"%d\"", current_layer);

  return true;
}

//}


/* onInit() //{ */

void BrickDetection::onInit() {

  bool bag = false;
  first_image = 1;
  wall_offset = 0.0;
  brick_init();

  m_nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  mrs_lib::ParamLoader param_loader(m_nh, "BrickDetection");

  ROS_INFO("[BrickDetection]: initializing");

  m_nodelet_name = string("BrickDetection");
  ROS_INFO("onInit started");

  // ParamLoader pl(m_nh, m_nodelet_name);
  // LOAD STATIC PARAMETERS
  ROS_INFO("[%s]: Loading static parameters:", m_nodelet_name.c_str());

  uav_name = String();
  param_loader.loadParam("uav_name", uav_name, std::string());
  // param_loader.loadParam("enable_profiler", profiler_enabled);
  param_loader.loadParam("bf_mask", bf_mask_name, std::string());
  param_loader.loadParam("rs_mask", rs_mask_name, std::string());
  bluefox_load_mask(bf_mask_name.c_str());
  realsense_load_mask(rs_mask_name.c_str());

  param_loader.loadParam("debug/enabled", debug);
  param_loader.loadParam("debug/gui", gui);
  param_loader.loadParam("simul", simul);
  param_loader.loadParam("bag", bag);

  param_loader.loadParam("debug/red", gui_red);
  param_loader.loadParam("debug/green", gui_green);
  param_loader.loadParam("debug/blue", gui_blue);
  param_loader.loadParam("debug/dbg", gui_dbg);

  param_loader.loadParam("calib/bf_x", bf_x);
  param_loader.loadParam("calib/bf_y", bf_y);
  param_loader.loadParam("calib/rs_x", calib_alf);
  param_loader.loadParam("calib/rs_y", calib_bet);
  param_loader.loadParam("calib/wall_off", wall_offset);

  param_loader.loadParam("target_frame", target_frame);
  param_loader.loadParam("body_frame", drone_body);

  if (gui) {

    if (gui_red) {
      cv::namedWindow("red", cv::WINDOW_FREERATIO);
    }

    if (gui_green) {
      cv::namedWindow("green", cv::WINDOW_FREERATIO);
    }

    if (gui_blue) {
      cv::namedWindow("blue", cv::WINDOW_FREERATIO);
    }

    if (gui_dbg) {
      cv::namedWindow("bfdbg", cv::WINDOW_FREERATIO);
      cv::namedWindow("rsdbg", cv::WINDOW_FREERATIO);
    }
  }

  if (uav_name.empty()) {
    uav_name = "uav1";
    ROS_ERROR("UAV_NAME is empty, used default uav1");
    ros::shutdown();
  }

  current_mode = SCAN_ALL;
  current_layer = 0;
  current_garmin_range_ = -1000.0;

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[BrickDetection]: Could not load all parameters!");
    ros::shutdown();
  }

  m_have_camera_info = false;
  m_have_depth_info = false;
  m_new_img = false;
  m_new_depth = false;
  last_altitude = last_odom_altitude = -20000.0;
  valid_last_angle = 0;
  noinit_bf_uav_tf = noinit_rs_uav_tf = 1;

  map_init(_RELATIVE_MAP, 20);
  map_init(_OBJECT_MAP, 60);

  // initialize dynamic reconfiguration feedback
  dynamic_reconfigure::Server<brick_detection::brick_detectionConfig> server;
  dynamic_reconfigure::Server<brick_detection::brick_detectionConfig>::CallbackType dynSer;
  dynSer = boost::bind(&reconfigureCallback, _1, _2);
  server.setCallback(dynSer);

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  // Debugging PUBLISHERS
  if (debug) {
    m_dbg_pub = m_nh.advertise<visualization_msgs::MarkerArray>("object_dbg", 1);
  }

  pub_debug_brick_ = m_nh.advertise<geometry_msgs::PoseArray>("brick_raw", 1);
  m_altitude_pub = m_nh.advertise<sensor_msgs::Range>("altitude", 1);
  m_brick_pub = m_nh.advertise<mbzirc_msgs::ObjectWithTypeArray>("object_array", 1);

  pub_dbg_rel_bf = m_nh.advertise<geometry_msgs::Vector3>("rel_bf", 1);
  pub_dbg_rel_rs = m_nh.advertise<geometry_msgs::Vector3>("rel_rs", 1);
  pub_dbg_rel = m_nh.advertise<geometry_msgs::Vector3>("rel", 1);


  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  image_transport::ImageTransport it(m_nh);
  image_transport::ImageTransport it2(m_nh);
  image_transport::TransportHints th;
  if (bag) {
    th = image_transport::TransportHints("compressed");
  } else {
    th = image_transport::TransportHints("raw");
  }
  m_img_sub = it.subscribe("image", 1, &BrickDetection::img_callback, this, th);
  m_cam_info_sub = m_nh.subscribe("cam_info", 1, &BrickDetection::cam_info_callback, this, ros::TransportHints().tcpNoDelay());
  m_garmin_sub = m_nh.subscribe("garmin_in", 1, &BrickDetection::garmin_callback, this, ros::TransportHints().tcpNoDelay());

  m_depth_sub = it2.subscribe("depth_image", 1, &BrickDetection::depth_callback, this, image_transport::TransportHints("raw"));
  m_depth_info_sub = m_nh.subscribe("depth_camera_info", 1, &BrickDetection::depth_info_callback, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  service_type_ = m_nh.advertiseService("type_in", &BrickDetection::callbackType, this);
  service_layer_ = m_nh.advertiseService("layer_in", &BrickDetection::callbackLayer, this);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  // 50 original
  m_main_timer = m_nh.createTimer(ros::Rate(20), &BrickDetection::main_loop, this);  // detection should fit into 50ms

  // --------------------------------------------------------------
  // |                         tf listener                        |
  // --------------------------------------------------------------

  m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer, m_nodelet_name);

  // | ------------------------ profiler ------------------------ |

  m_profiler = new mrs_lib::Profiler(m_nh, "BrickDetection", profiler_enabled);

  m_initialized = true;
  ROS_INFO("[BrickDetection]: initialized");
}

//}

}  // namespace brick_detection

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(brick_detection::BrickDetection, nodelet::Nodelet)
