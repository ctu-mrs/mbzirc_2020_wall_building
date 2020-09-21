// clang: PetrFormat

/*
 *  brick.cpp
 *
 *  brick detection
 *
 *  Created on: 3. 1. 2019
 *  Author: petr
 */

#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <cstdio>
#include "brick.h"
#include <time.h>

/* #defien _DEBUG_LINE 1 */
#define _DEBUG_DETECTOR 1

using namespace std;
using namespace cv;

namespace brick_detection
{

#define MAX_HEIGHT 1024

#define my_sqr(a) ((a) * (a))
#define abs(a) (((a) < 0) ? (-(a)) : (a))
#define my_abs(a) (((a) < 0) ? (-(a)) : (a))

#define BRICK_SIZE 0.2
#define WHITE_SIZE 0.15
#define _WHITE_SAT_MAX 75

static Mat *dbg;
static int height, width;
static int image_max_x, image_max_y;

static Mat metric_coordinates;
static Mat image_coordinates;
static Mat intrinsic;
static Mat distCoeffs;

static Mat erosion_element, erosion_small, erosion_element_big;
static Mat pic, tmp_pic;

static Mat hsv_src, blue_thr, red_thr, red_thr1, red_thr2, orange_thr, green_thr, thr_pre_mask;
static Mat white_thr, white_erode_thr;
static Mat kernel;

static Mat wall_thr;


static float cam_cx, cam_cy, cam_fx, cam_fy;

static Mat img, tmp, tmp2, tmp3, tmp4, tmp_img;
static Mat orig;
static Mat buf;
static Mat element;

static bool simul_camera;

static float *src_ptr, *dst_ptr;

static Vec3b blue_bgr(250, 10, 10);
static Vec3b red_bgr(25, 30, 100);
static Vec3b green_bgr(10, 250, 10);
static Vec3b orange_bgr(10, 150, 250);

static Vec3b blue_hsv;
static Vec3b red_hsv;
static Vec3b green_hsv;
static Vec3b orange_hsv;

static Vec3b min_blue;
static Vec3b min_red;
static Vec3b min_red2;
static Vec3b min_green;
static Vec3b min_orange;
static Vec3b min_wall;

static Vec3b max_blue;
static Vec3b max_red;
static Vec3b max_red2;
static Vec3b max_green;
static Vec3b max_orange;
static Vec3b max_wall;

static int max_w_sat, min_w_val, max_w_val;
static int max_sh_sat, min_sh_val, max_sh_val;
static Vec3b min_white;
static Vec3b max_white;
static Vec3b min_shadow;
static Vec3b max_shadow;
static bool use_shadow;
static Mat white_calib, white_calib_erod, white_shadow;

static Vec3b max_tmp;
static Vec3b min_tmp;

static int local_brick_num;
static brick local_brick[64];
static int local_to_white[64];
static float local_dist_white[64];
static float brick_alt, altitude;

#define _WHITE_MAX_NUM 64
static brick white_arr[_WHITE_MAX_NUM];
static int white_to_brick[_WHITE_MAX_NUM];
static double white_to_size[_WHITE_MAX_NUM];
static int act_num_white;

static int act_num_bricks;
static double min_size, max_size;
static Mat labels, stats, centr;
static brick *brick_arr;
static int is_mask;
static Mat bf_mask, mask_c;
static vector<vector<Point>> mask_contours;
static vector<Vec4i> mask_hierarchy;
static int num_mask_correct;

static vector<vector<Point>> contours;
static vector<Vec4i> hierarchy;
static vector<Point> unite_cont, unite_convex, conv_hull_pix;
static vector<Point2f> conv_hull;
static vector<Point2f> conv_approx;
static vector<Point2f> real_contour;
static vector<Point2f> orig_real_contour;
static vector<Point2f> real_approx;
static Point2f real_c[4096];
static int real_c_num;
static int gBrick_array_size;

#define HUE_DIFF 20
#define SAT_DIFF 60
#define VAL_DIFF 80

/* update_hsv //{ */

static bool debug_detector;

[[maybe_unused]] static void upgrade_hsv(Vec3b hsv, Vec3b &hsv_min, Vec3b &hsv_max, Vec3b &hsv_min2, Vec3b &hsv_max2) {

  int hue = (int)hsv(0);
  int sat = (int)hsv(1);
  int val = (int)hsv(2);
  int m_h, m_s, m_v;
  int m_h_max = 0;
  if (sat > SAT_DIFF) {
    m_s = sat - SAT_DIFF;
  } else {
    m_s = 0;
  }
  m_s = 140;
  if (val > VAL_DIFF) {
    m_v = val - VAL_DIFF;
  } else {
    m_v = 0;
  }
  m_v = 20;
  if (hue > HUE_DIFF) {
    m_h = hue - HUE_DIFF;
    hsv_min = Vec3b(m_h, m_s, m_v);
    hsv_min2 = Vec3b(m_h, m_s, m_v);
  } else {
    m_h = 180 + hue - HUE_DIFF;
    hsv_min = Vec3b(0, m_s, m_v);
    hsv_min2 = Vec3b(m_h, m_s, m_v);
    m_h_max = 180;
  }

  if (sat < 255 - SAT_DIFF) {
    m_s = sat + SAT_DIFF;
  } else {
    m_s = 255;
  }
  m_s = 255;
  if (val < 255 - VAL_DIFF) {
    m_v = val + VAL_DIFF;
  } else {
    m_v = 255;
  }
  m_v = 255;
  if (hue < 255 - HUE_DIFF) {
    m_h = hue + HUE_DIFF;
    if (m_h_max == 180) {
      hsv_max = Vec3b(m_h, m_s, m_v);
      hsv_max2 = Vec3b(180, m_s, m_v);
    } else {
      hsv_max = Vec3b(m_h, m_s, m_v);
      hsv_max2 = Vec3b(m_h, m_s, m_v);
    }
  } else {
    m_h = hue + HUE_DIFF - 180;
    hsv_max = Vec3b(180, m_s, m_v);
    hsv_max2 = Vec3b(m_h, m_s, m_v);
    hsv_min2[0] = 0;
  }

  if (debug_detector) {
    ROS_DEBUG_STREAM("Min " << hsv_min << " max " << hsv_max << " mh " << m_h << "," << m_s << "," << m_v << " hsv " << hsv);
    ROS_DEBUG_STREAM("Min2 " << hsv_min2 << " max2 " << hsv_max2 << " hsv " << hue << "," << sat << "," << val << " hsv " << hsv);
  }
}

//}

static inline int dist_point(Point a, Point b) {
  return my_abs(a.x - b.x) + my_abs(a.y - b.y);
}

#define SIMUL_SCALE 0.4
//#define SIMUL_SCALE 1.0

static inline int dbg_real_x(float x) {
  return cam_cx + x * SIMUL_SCALE * cam_fx;
}

static inline int dbg_real_y(float y) {
  return cam_cy + y * SIMUL_SCALE * cam_fy;
}

static float m00, m01, m02, m10, m11, m12, m20, m21, m22;
static Mat rot_eu, vec, poi;

/* angle2rot() //{ */

static Mat angle2rot(float sa, float sb, float ca, float cb) {

  Mat rotationMatrix(3, 3, CV_32F);

  m00 = ca;
  m01 = 0;
  m02 = -sa;

  m10 = -sa * sb;
  m11 = cb;
  m12 = -ca * sb;  // m12 = -sb*cam_f;

  m20 = sa * cb;  // m20 = sa/cam_f;
  m21 = sb;       // m21 = ca*sb/cam_f;
  m22 = ca * cb;

  rotationMatrix.at<float>(0, 0) = m00;
  rotationMatrix.at<float>(0, 1) = m01;
  rotationMatrix.at<float>(0, 2) = m02;
  rotationMatrix.at<float>(1, 0) = m10;
  rotationMatrix.at<float>(1, 1) = m11;
  rotationMatrix.at<float>(1, 2) = m12;
  rotationMatrix.at<float>(2, 0) = m20;
  rotationMatrix.at<float>(2, 1) = m21;
  rotationMatrix.at<float>(2, 2) = m22;

  return rotationMatrix;
}

//}

static double pol[5] = {-2.630739e+02, 0.000000e+00, 1.583068e-03, -2.075182e-06, 6.208431e-09};
/*static double ocam_c = 1.000099;
static double ocam_d = 0.000316;
static double ocam_e = 0.000473;
*/

/* omnidir_undistortPoints() //{ */

static void omnidir_undistortPoints(float dist_x, float dist_y, float &undist_x, float &undist_y) {

  if (simul_camera) {
    // vec.at<float>(0,0) = (dist_x-cam_cx);
    // vec.at<float>(1,0) = (dist_y-cam_cy);
    // vec.at<float>(2,0) = cam_f;
    // rotationMatrix * vec
    float u_x = (dist_x - cam_cx);
    float u_y = (dist_y - cam_cy);

    // poi = rot_eu * vec;
    float x = u_x * m00 + u_y * m01 + m02 * cam_fx;
    float y = u_x * m10 + u_y * m11 + m12 * cam_fx;
    float z = u_x * m20 + u_y * m21 + m22 * cam_fx;

    undist_x = (x / z);  // *cam_fx
    undist_y = (y / z);  // *cam_fy
                         // undist_x = dist_x;
    // undist_y = dist_y;
    // fprintf(stderr, "X %f Y %f orig %f, %f\n", undist_x, undist_y, dist_x, dist_y);
  } else {

    // double invdet = 1 / (ocam_c - ocam_d * ocam_e);  // 1/det(A), where A = [c,d;e,1] as in the Matlab file

    double xp = (dist_x - cam_cx);
    double yp = (dist_y - cam_cy);

    double r = sqrt(xp * xp + yp * yp);  // distance [pixels] of  the point from the image center
    double zp = pol[0];
    double r_i = r;
    int i;

    for (i = 1; i < 5; i++) {
      zp += r_i * pol[i];
      r_i *= r;
    }

    // normalize to standard camera
    // double invnorm = pol[0];

    float u_x = xp;
    float u_y = yp;

    float x = u_x * m00 + u_y * m01 - m02 * zp;
    float y = u_x * m10 + u_y * m11 - m12 * zp;
    float z = -u_x * m20 - u_y * m21 + m22 * zp;

    undist_x = (-x / z);
    undist_y = (-y / z);
    // cout << "orig "<<dist_x<<", "<<dist_y<<" und2 "<<undist_x<<", "<<undist_y<<" und1 "<< u_x<<", "<<u_y<<" koef "<<((undist_x-cam_cx)/(u_x-cam_cx))<<endl;
  }
}

//}

/* update_white_color() //{ */
void update_white_color() {
  min_white = Vec3b(0, 0, min_w_val);
  max_white = Vec3b(200, max_w_sat, max_w_val);
  if (use_shadow) {
    min_shadow = Vec3d(0, 0, min_sh_val);
    max_shadow = Vec3d(200, max_sh_sat, max_sh_val);
  }
}
//}


/* brick_init() //{ */

#define MIN_SATURE 110
#define MIN_VALUE 20

void brick_init(void) {

  min_blue = Vec3b(80, 40, MIN_VALUE+40);
  max_blue = Vec3b(130, 255, 255);

  min_red = Vec3b(0, MIN_SATURE - 40, MIN_VALUE + 60);
  max_red = Vec3b(8, 255, 255);
  min_red2 = Vec3b(160, MIN_SATURE - 40, MIN_VALUE + 60);
  max_red2 = Vec3b(181, 255, 255);

  min_green = Vec3b(44, 60, MIN_VALUE+40);
  max_green = Vec3b(80, 255, 255);

  min_wall = Vec3b(16, 30, 160);
  max_wall = Vec3b(44, 255, 255);
  
  max_w_sat=60;
  min_w_val=180;
  max_w_val=255;
  max_sh_sat=0;
  min_sh_val=0;
  max_sh_val=0;
  use_shadow = false;
  update_white_color();
  // kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
  kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
  int erosion_size = 3;
  erosion_element = getStructuringElement(MORPH_RECT, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));
  erosion_size = 1;
  erosion_small = getStructuringElement(MORPH_RECT, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));
  erosion_size = 5;
  erosion_element_big = getStructuringElement(MORPH_RECT, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));
}

//}

/* bluefox_load_mask() //{ */

void bluefox_load_mask(const char *mask_name) {
  Mat dbg_mask;
  bf_mask = imread(mask_name, IMREAD_GRAYSCALE);
  if (bf_mask.empty()) {
    is_mask = 0;
  } else {
    is_mask = 1;
    for (int v = 0; v < bf_mask.rows; v++) {
      for (int u = 0; u < bf_mask.cols; u++) {
        if (bf_mask.at<unsigned char>(v, u) > 100) {
          bf_mask.at<unsigned char>(v, u) = 255;
        } else {
          bf_mask.at<unsigned char>(v, u) = 0;
        }
      }
    }
    Mat neg_mask, neg_tmp;
    bitwise_not(bf_mask, neg_tmp);
    cvtColor(neg_tmp, mask_c, CV_GRAY2BGR);
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
    dilate(neg_tmp, neg_mask, element);

    vector<vector<Point>> tmp;
    findContours(neg_mask, tmp, mask_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    mask_contours.clear();
    if (tmp.size() > 3) {
      num_mask_correct = 0;
    } else {
      num_mask_correct = tmp.size();
      mask_contours.resize(num_mask_correct);
      int max_cols = bf_mask.cols - 1;
      for (int i = 0; i < num_mask_correct; i++) {
        int border_start = -1, border_end = -1;
        for (int j = 0; border_end == -1 && j < (int)tmp[i].size(); j++) {
          if (border_start == -1 && (tmp[i][j].x == 0 || tmp[i][j].x == max_cols)) {
            border_start = j;
          }
          if (border_start >= 0 && (tmp[i][j].x != 0 && tmp[i][j].x != max_cols)) {
            border_end = j;
          }
        }
        if (border_end == -1) {
          border_end = 0;
        }
        if (border_start >= 0) {
          int j = border_end;
          while (j != border_start) {
            mask_contours[i].push_back(tmp[i][j]);
            j++;
            j %= tmp[i].size();
          }
        } else {
          ROS_ERROR_STREAM("[Brick Detection] MASK - ERROR border start:" << border_start << " end:" << border_end);
        }
      }
    }
#ifdef _DEBUG_MASK
    cvtColor(neg_mask, dbg_mask, CV_GRAY2BGR);
    drawContours(dbg_mask, tmp, -1, Scalar(255, 0, 0), 3);
    drawContours(dbg_mask, mask_contours, -1, Scalar(0, 255, 0), 2);
    imshow("dbgmask", dbg_mask);
#endif
  }
  ROS_INFO("BLUEFOX mask is %i num segments: %i", is_mask, num_mask_correct);
}

//}

/* correct_mask() //{ */

static vector<Point> fill_points;
static void correct_mask(Mat m) {
  int mask_s[64];
  int mask_e[64];
  int mask_near[64];

  for (int i = 0; i < num_mask_correct; i++) {
    int is_inside = 0;
    int seg_ptr = 0;
    for (int j = 0; j < (int)mask_contours[i].size(); j++) {
      if (m.at<unsigned char>(mask_contours[i][j]) > 0) {
        if (!is_inside) {
          is_inside = 1;
          mask_s[seg_ptr] = j;
        }
      } else {
        if (is_inside) {
          mask_e[seg_ptr++] = j - 1;
          is_inside = 0;
        }
        if (seg_ptr >= 62) {
          break;
        }
      }
    }
    for (int s = 0; s < seg_ptr - 1; s++) {
      if (dist_point(mask_contours[i][mask_e[s]], mask_contours[i][mask_s[s + 1]]) <= 3) {
        mask_e[s] = mask_e[s + 1];
        for (int t = s + 1; t < seg_ptr - 1; t++) {
          mask_s[t] = mask_s[t + 1];
          mask_e[t] = mask_e[t + 1];
        }
        seg_ptr--;
      }
    }
    for (int s = 0; s < seg_ptr; s++) {
      if (mask_e[s] - mask_s[s] < 7) {
        for (int t = s; t < seg_ptr - 1; t++) {
          mask_s[t] = mask_s[t + 1];
          mask_e[t] = mask_e[t + 1];
        }
        seg_ptr--;
        s--;
      }
    }
    if (seg_ptr == 1) {
      fill_points.clear();
      for (int j = mask_s[0]; j <= mask_e[0]; j++) {
        fill_points.push_back(mask_contours[i][j]);
      }
      fillConvexPoly(m, fill_points, Scalar(255));
    } else {
      for (int s = 0; s < seg_ptr; s++) {
        int min = 1000000;
        int min_i = -1;
        for (int t = 0; t < seg_ptr; t++) {
          if (t != s) {
            int dd = dist_point(mask_contours[i][mask_e[s]], mask_contours[i][mask_s[t]]) + dist_point(mask_contours[i][mask_s[s]], mask_contours[i][mask_e[t]]);
            if (dd < min) {
              min_i = t;
              min = dd;
            }
          }
        }
        mask_near[s] = min_i;
      }
      for (int s = 0; s < seg_ptr; s++) {
        if (mask_near[s] >= 0) {
          if (mask_near[mask_near[s]] == s) {
            fill_points.clear();
            for (int j = mask_s[s]; j <= mask_e[s]; j++) {
              fill_points.push_back(mask_contours[i][j]);
            }
            for (int j = mask_s[mask_near[s]]; j <= mask_e[mask_near[s]]; j++) {
              fill_points.push_back(mask_contours[i][j]);
            }
            fillConvexPoly(m, fill_points, Scalar(255));
            mask_near[mask_near[s]] = -1;
          } else {
            fill_points.clear();
            for (int j = mask_s[s]; j <= mask_e[s]; j++) {
              fill_points.push_back(mask_contours[i][j]);
            }
            fillConvexPoly(m, fill_points, 255);
          }
          mask_near[s] = -1;
        }
      }
    }
  }
}

//}

/* set_camera_param() //{ */

void set_camera_param(Mat intr, Mat dist, bool simul_) {

  simul_camera = simul_;
  metric_coordinates = cv::Mat(1, 1, CV_32FC2);
  image_coordinates = cv::Mat(1, 1, CV_32FC2);
  src_ptr = image_coordinates.ptr<float>(0);
  dst_ptr = metric_coordinates.ptr<float>(0);
  intrinsic = intr;
  distCoeffs = dist;

  cam_fx = intr.at<float>(0, 0);
  cam_fy = intr.at<float>(1, 1);
  if (simul_) {
    // cam_cx = 752 / 2;
    // cam_cy = 480 / 2;
    cam_cx = intr.at<float>(0, 2);
    cam_cy = intr.at<float>(1, 2);
    ROS_INFO_STREAM("SIMULATION CAMERA fx " << cam_fx << " fy " << cam_fy << " cx " << cam_cx << " cy " << cam_cy << " no distortion");
    min_red2 = Vec3b(145, MIN_SATURE - 40, MIN_VALUE + 80);
  } else {
    cam_cx = intr.at<float>(0, 2);
    cam_cy = intr.at<float>(1, 2);
    for (int i = 0; i < 5; i++) {
      pol[i] = dist.at<float>(i);
    }
    // cam_cx = 356.739009;
    // cam_cy = 242.049153;
    cam_fx = cam_fy = -pol[0];
    ROS_INFO_STREAM("CAMERA fx " << cam_fx << " fy " << cam_fy << " cx " << cam_cx << " cy " << cam_cy);
    ROS_INFO_STREAM("Distortion " << pol[0] << ", " << pol[1] << ", " << pol[2] << ", " << pol[3] << ", " << pol[4]);
    ;
  }
}

//}


[[maybe_unused]] static struct Pair
{ int a, b; } stack_pair[400];

static int c_buf_x[32768];
static int c_buf_y[32768];
static int c_buf_first;
static int c_buf_last;

static void c_stack_init() {
  c_buf_first = c_buf_last = 0;
}

static int c_stack_not_empty() {
  return c_buf_first != c_buf_last;
}

static int c_stack_pop(int &x, int &y) {
  if (c_buf_first != c_buf_last) {
    x = c_buf_x[c_buf_first];
    y = c_buf_y[c_buf_first++];
    c_buf_first &= 32767;
    return 1;
  } else {
    return 0;
  }
}

static int c_stack_push(int x, int y) {
  int old_last = c_buf_last;
  c_buf_x[c_buf_last] = x;
  c_buf_y[c_buf_last++] = y;
  c_buf_last &= 32767;
  if (c_buf_last == c_buf_first) {
    c_buf_last = old_last;
    return 0;
  }
  return 1;
}


static void enlarge_or(Mat m, Mat white) {
  // int cols = m.cols;
  int max_col_m_one = m.cols - 1;
  int max_row_m_one = m.rows - 2;
  unsigned char last;
  for (int v = 2; v < max_row_m_one; v++) {
    last = (m.at<unsigned char>(v, 0)>0?1:0);
    for (int u = 1; u < max_col_m_one; u++) {
      unsigned char val = m.at<unsigned char>(v, u);
      if (last > 1 && val == 0) {
        if (white.at<unsigned char>(v, u) > 0) {
          c_stack_init();
          c_stack_push(u, v);
          m.at<unsigned char>(v, u) = 255;
        }
      } else if (last == 0 && val > 0) {
        if (white.at<unsigned char>(v, u - 1) > 0 && m.at<unsigned char>(v, u+1)>0) {
          c_stack_init();
          c_stack_push(u - 1, v);
          m.at<unsigned char>(v, u - 1) = 255;
        }
      } else if (val == 0) {
        if (white.at<unsigned char>(v, u) > 0) {
          if (m.at<unsigned char>(v - 1, u) > 0 && m.at<unsigned char>(v - 2, u) > 0) {
            c_stack_init();
            c_stack_push(u, v);
            m.at<unsigned char>(v, u) = 255;
          } else if (m.at<unsigned char>(v + 1, u) > 0 && m.at<unsigned char>(v + 2, u) > 0) {
            c_stack_init();
            c_stack_push(u, v);
            m.at<unsigned char>(v, u) = 255;
          }
        }
      }
      last = val;
      while (c_stack_not_empty()) {
        int uu, vv;
        c_stack_pop(uu, vv);
        if (uu > 0 && m.at<unsigned char>(vv, uu - 1) == 0 && white.at<unsigned char>(vv, uu - 1) > 0) {
          c_stack_push(uu - 1, vv);
          m.at<unsigned char>(vv, uu - 1) = 255;
        }
        if (vv > 0 && m.at<unsigned char>(vv - 1, uu) == 0 && white.at<unsigned char>(vv - 1, uu) > 0) {
          c_stack_push(uu, vv - 1);
          m.at<unsigned char>(vv - 1, uu) = 255;
        }
        if (uu < max_col_m_one && m.at<unsigned char>(vv, uu + 1) == 0 && white.at<unsigned char>(vv, uu + 1) > 0) {
          c_stack_push(uu + 1, vv);
          m.at<unsigned char>(vv, uu + 1) = 255;
        }
        if (vv < max_row_m_one && m.at<unsigned char>(vv + 1, uu) == 0 && white.at<unsigned char>(vv + 1, uu) > 0) {
          c_stack_push(uu, vv + 1);
          m.at<unsigned char>(vv + 1, uu) = 255;
        }
      }
    }
  }
}

/*
[[maybe_unused]] static float dist(Point2f &a, Point2f &b) {
  return sqrt(my_sqr(a.x - b.x) + my_sqr(a.y - b.y));
}



static void skew_contour(vector<Point2f> &o, vector<Point2f> &n, float koef) {
  int num = 0;
  float limit_koef = (koef - 1.0) * 0.2;
  for (int i = 0; i < (int)n.size(); i++) {
    Point2f p;
    p.x = n[i].x * koef;
    p.y = n[i].y * koef;
    double limit = limit_koef * (my_abs(n[i].x) + my_abs(n[i].y));
    double d = pointPolygonTest(o, p, true);
    // cout <<"New point:"<<p.x<<","<<p.y<<" limit:"<<limit<<" dist:"<<d<<" koef:"<<koef<<endl;
    if (d > -limit) {
      n[i].x = p.x;
      n[i].y = p.y;
      num++;
    }
  }
  // cout << "SHIFTED "<<num<<"points"<<endl;
}
*/

static float min_ration[6] = {0.30, 0.15, 0.10, 0.04, 0.0, 0.25};
static float max_ration[6] = {0.98, 0.50, 0.33, 0.16, 0.0, 0.98};
// static float brick_len[6] = {0.3, 0.6, 1.2, 1.8, 4.0, 0.2};
static Scalar br_color[6] = {Scalar(100, 100, 255), Scalar(100, 255, 100), Scalar(255, 100, 100), Scalar(60, 200, 255), Scalar(250, 60, 255), Scalar(255, 255, 255)};
static RotatedRect box;

static float max_len_set1[6] = {0.4, 0.76, 1.42, 2.05, 0.0, 0.39};
static float min_len_set1[6] = {0.08, 0.15, 0.40, 1.55, 0.0, 0.12};
static float max_len_set2[6] = {0.5, 1.1, 2.0, 2.8, 0.0, 0.39};
static float min_len_set2[6] = {0.1, 0.2, 0.45, 1.55, 0.0, 0.12};

static float max_len_brick, min_len_brick;

static int test_brick_bottom(vector<Point2f> cont, enum Brick_type br_type, Point2f &c, float &yaw, float &s) {
  int ret = 0;
  int c_s = cont.size();
  int min_i = 0;
  float min_y = cont[0].y;
  for (int i = 1; i < (int)cont.size(); i++) {
    if (cont[i].y < min_y) {
      min_i = i;
      min_y = cont[i].y;
    }
  }
  int sec_i = (min_i + 1) % c_s;
  if (cont[(min_i + (c_s - 1)) % c_s].y < cont[sec_i].y) {
    sec_i = min_i;
    min_i = (min_i + (c_s - 1)) % c_s;
  }
  float v_x = cont[min_i].x - cont[sec_i].x;
  float v_y = cont[min_i].y - cont[sec_i].y;
  float size = sqrt(my_sqr(v_x) + my_sqr(v_y));
  int step = 0;
  while (step < 4 && size * altitude < min_len_set2[br_type - 1]) {
    step++;
    int tmp_min = (min_i + (c_s - 1)) % c_s;
    int tmp_sec = (sec_i + 1) % c_s;
    if (cont[tmp_min].y < cont[tmp_sec].y) {
      v_x = cont[tmp_min].x - cont[sec_i].x;
      v_y = cont[tmp_min].y - cont[sec_i].y;
      float n_size = sqrt(my_sqr(v_x) + my_sqr(v_y));
      if (n_size < size) {
        v_x = cont[min_i].x - cont[sec_i].x;
        v_y = cont[min_i].y - cont[sec_i].y;
        break;
      }
      min_i = tmp_min;
      size = n_size;
    } else {
      v_x = cont[min_i].x - cont[tmp_sec].x;
      v_y = cont[min_i].y - cont[tmp_sec].y;
      float n_size = sqrt(my_sqr(v_x) + my_sqr(v_y));
      if (n_size < size) {
        v_x = cont[min_i].x - cont[sec_i].x;
        v_y = cont[min_i].y - cont[sec_i].y;
        break;
      }
      sec_i = tmp_sec;
      size = n_size;
    }
  }

  if (debug_detector) {
    line(*dbg, Point(dbg_real_x(cont[min_i].x), dbg_real_y(cont[min_i].y)), Point(dbg_real_x(cont[sec_i].x), dbg_real_y(cont[sec_i].y)), Vec3b(200, 100, 255), 2, LINE_AA);
  }

  v_x /= size;
  v_y /= size;
  if (v_x > 0.0) {
    v_x *= -1;
    v_y *= -1;
  }

  float alt = altitude;
  if (cont[min_i].y < 0) {
    alt = brick_alt;
  }
  float real_size = size * alt;
  yaw = atan2(v_y, v_x);
  /*if (border) {
    if (cont[min_i].x<=1) {
      cont[min_i].x = cont[sec_i].x + v_x*len/alt;
      cont[min_i].y = cont[sec_i].y + v_y*len/alt;
    } else if (cont[min_i].x>=width-2) {
      cont[min_i].x = cont[sec_i].x - v_x*len/alt;
      cont[min_i].y = cont[sec_i].y - v_y*len/alt;
    } else if (cont[sec_i].x<=1) {
      cont[sec_i].x = cont[min_i].x + v_x*len/alt;
      cont[sec_i].y = cont[min_i].y + v_y*len/alt;
    } else if (cont[sec_i].x>=width-2) {
      cont[sec_i].x = cont[min_i].x - v_x*len/alt;
      cont[sec_i].y = cont[min_i].y - v_y*len/alt;
    }
    real_size = len;
  } */
  if (cont[min_i].y < 0) {
    real_size = size * brick_alt;
    if (real_size > min_len_set2[br_type - 1] && real_size < max_len_set2[br_type - 1]) {
      ret = 1;
      c.x = (cont[min_i].x + cont[sec_i].x) * brick_alt / 2.0 + v_y * BRICK_SIZE / 2.0;
      c.y = (cont[min_i].y + cont[sec_i].y) * brick_alt / 2.0 - v_x * BRICK_SIZE / 2.0;
      s = real_size;
    }
  } else {
    if (real_size > min_len_set2[br_type - 1] && real_size < max_len_set2[br_type - 1]) {
      ret = 1;
      c.x = (cont[min_i].x + cont[sec_i].x) * altitude / 2.0 + v_y * BRICK_SIZE / 2.0;
      c.y = (cont[min_i].y + cont[sec_i].y) * altitude / 2.0 - v_x * BRICK_SIZE / 2.0;
      s = real_size;
      if (local_brick_num > 0) {
        float min_dist = my_sqr(local_brick[0].c_x - c.x) + my_sqr(local_brick[0].c_y - c.y);
        int min_i = 0;
        for (int i = 1; i < local_brick_num; i++) {
          float d = my_sqr(local_brick[i].c_x - c.x) + my_sqr(local_brick[i].c_y - c.y);
          if (d < min_dist) {
            min_i = i;
            min_dist = d;
          }
        }
        if (min_dist < 0.6) {
          float y_d = yaw_dif(yaw, local_brick[min_i].yaw);
          ROS_DEBUG_STREAM("YAW YAW Yaw diff:" << y_d << " new pos:" << c.x << "," << c.y << " old:" << local_brick[min_i].c_x << "," << local_brick[min_i].c_y);
          if (y_d < 0.15 && y_d > -0.15) {
            yaw = yaw_avg(yaw, local_brick[min_i].yaw);
            c.x = (c.x + local_brick[min_i].c_x) / 2.0;
            c.y = (c.y + local_brick[min_i].c_y) / 2.0;
          } else {
            yaw = local_brick[min_i].yaw;
            c.x = (c.x * 0.3 + 0.7 * local_brick[min_i].c_x);
            c.y = (c.y * 0.3 + 0.7 * local_brick[min_i].c_y);
          }
        }
      }
    } else {
      ROS_DEBUG_STREAM("BORDER WRONG SIZE:" << real_size);
    }
  }

  return ret;
}

static int test_white_bottom(vector<Point2f> cont, enum Brick_type br_type, Point2f &c, float &yaw, float &s) {
  int ret = 0;
  int c_s = cont.size();
  int min_i = 0;
  float min_y = cont[0].y;
  for (int i = 1; i < (int)cont.size(); i++) {
    if (cont[i].y < min_y) {
      min_i = i;
      min_y = cont[i].y;
    }
  }
  int sec_i = (min_i + 1) % c_s;
  if (cont[(min_i + (c_s - 1)) % c_s].y < cont[sec_i].y) {
    sec_i = min_i;
    min_i = (min_i + (c_s - 1)) % c_s;
  }
  float v_x = cont[min_i].x - cont[sec_i].x;
  float v_y = cont[min_i].y - cont[sec_i].y;
  float size = sqrt(my_sqr(v_x) + my_sqr(v_y));
  int step = 0;
  while (step < 4 && size < min_len_set2[br_type - 1]) {
    step++;
    int tmp_min = (min_i + (c_s - 1)) % c_s;
    int tmp_sec = (sec_i + 1) % c_s;
    ROS_DEBUG_STREAM("Min i " << min_i << " sec i " << sec_i << " test min i " << tmp_min << " sec i " << tmp_sec);
    if (cont[tmp_min].y < cont[tmp_sec].y) {
      v_x = cont[tmp_min].x - cont[sec_i].x;
      v_y = cont[tmp_min].y - cont[sec_i].y;
      float n_size = sqrt(my_sqr(v_x) + my_sqr(v_y));
      if (n_size < size) {
        v_x = cont[min_i].x - cont[sec_i].x;
        v_y = cont[min_i].y - cont[sec_i].y;
        break;
      }
      min_i = tmp_min;
      size = n_size;
    } else {
      v_x = cont[min_i].x - cont[tmp_sec].x;
      v_y = cont[min_i].y - cont[tmp_sec].y;
      float n_size = sqrt(my_sqr(v_x) + my_sqr(v_y));
      if (n_size < size) {
        v_x = cont[min_i].x - cont[sec_i].x;
        v_y = cont[min_i].y - cont[sec_i].y;
        break;
      }
      sec_i = tmp_sec;
      size = n_size;
    }
  }

  ROS_DEBUG_STREAM("WHITE det finish:[" << cont[min_i].x << "," << cont[min_i].y << "],[" << cont[sec_i].x << "," << cont[sec_i].y << "] size:" << size);
  if (debug_detector) {
    line(*dbg, Point(dbg_real_x(cont[min_i].x), dbg_real_y(cont[min_i].y)), Point(dbg_real_x(cont[sec_i].x), dbg_real_y(cont[sec_i].y)), Vec3b(150, 150, 150), 2, LINE_AA);
  }

  v_x /= size;
  v_y /= size;
  if (v_x > 0.0) {
    v_x *= -1;
    v_y *= -1;
  }
  yaw = atan2(v_y, v_x);

  if (size > min_len_set2[br_type - 1] && size < max_len_set2[br_type - 1]) {
    ret = 1;
    ROS_DEBUG_STREAM("WHITE plate detected, shift:" << ((cont[min_i].x + cont[sec_i].x) / 2.0) << "," << ((cont[min_i].y + cont[sec_i].y) / 2.0)
                                                    << " to:" << ((cont[min_i].x + cont[sec_i].x) * brick_alt / 2.0 + v_y * WHITE_SIZE / 2.0) << ","
                                                    << ((cont[min_i].y + cont[sec_i].y) * brick_alt / 2.0 - v_x * WHITE_SIZE / 2.0));
    c.x = (cont[min_i].x + cont[sec_i].x) / 2.0 + v_y * WHITE_SIZE / 2.0;
    c.y = (cont[min_i].y + cont[sec_i].y) / 2.0 - v_x * WHITE_SIZE / 2.0;
    s = size;
  }

  return ret;
}


static inline bool correct_size(float b_len, float b_width, enum Brick_type br_type, bool border) {
  if (border) {
    return b_width > 0.03 && b_width < 0.32 && b_len < max_len_brick;
  } else {
    return b_width > 0.08 && b_width < 0.32 && b_len > min_len_brick && b_len < max_len_brick && (b_len * min_ration[(int)br_type - 1] < b_width) &&
           (b_len * max_ration[(int)br_type - 1] > b_width);
  }
}

static inline bool correct_white_size(float b_len, float b_width) {
  //ROS_DEBUG_STREAM("---- White plate width:"<<b_width<<" len:"<<b_len<<" correct:"<<(b_width > 0.08 && b_width < 0.25 && b_len > 0.12 && b_len < 0.40));
  return b_width > 0.08 && b_width < 0.25 && b_len > 0.12 && b_len < 0.40;
}

/* find_componenets() //{ */

static void add_new_brick(Point2f &center, float yaw_br, float size_br, enum Brick_type br_type, double pix_size) {
  float min_dist = 10000.0;
  int min_loc = -1;
  if (act_num_white > 0) {
    for (int i = 0; i< act_num_white; i++) {
      float d = my_sqr(white_arr[i].c_x - center.x) + my_sqr(white_arr[i].c_y - center.y);
      if (d < min_dist) {
        min_loc = i;
        min_dist = d;
      }
    }
  }
  if (min_dist < 0.36) {
    ROS_DEBUG_STREAM("OK -Brick type:" << br_type << " cen:" << center.x << "," << center.y << " distance to local_brick:" << min_dist << " ind:" << min_loc<< " size:"<<size_br<<" pix:"<<pix_size);
    if (white_to_brick[min_loc]>=0) {
      if (pix_size>white_to_size[min_loc]) {
        brick_arr[white_to_brick[min_loc]].type = br_type;
        brick_arr[white_to_brick[min_loc]].size = size_br;
        white_to_size[min_loc] = pix_size;
      }
    } else {  
      if (debug_detector) {
        circle(*dbg, Point(dbg_real_x(white_arr[min_loc].c_x / brick_alt), dbg_real_y(white_arr[min_loc].c_y / brick_alt)), 8, Scalar(100, 200, 2550), 2);
      }
      brick_arr[act_num_bricks].c_x = white_arr[min_loc].c_x;
      brick_arr[act_num_bricks].c_y = white_arr[min_loc].c_y;
      brick_arr[act_num_bricks].yaw = white_arr[min_loc].yaw;
      brick_arr[act_num_bricks].type = br_type;
      brick_arr[act_num_bricks].size = size_br;
      white_to_size[min_loc] = pix_size;
      white_to_brick[min_loc] = act_num_bricks;
      act_num_bricks++;
      if (act_num_bricks >= gBrick_array_size) {
        ROS_DEBUG("Lot of bricks.n");
        act_num_bricks--;
      } 
    }
 // } else {
 //   ROS_DEBUG_STREAM("ERROR: Brick without white type:" << br_type << " cen:" << center.x << "," << center.y << " distance to white:" << min_dist << " ind:" << min_loc<< " size:"<<size_br<<" pix:"<<pix_size);
/*      brick_arr[act_num_bricks].c_x = center.x;
      brick_arr[act_num_bricks].c_y = center.y;
      brick_arr[act_num_bricks].yaw = yaw_br;
    }
    brick_arr[act_num_bricks].type = br_type;
    brick_arr[act_num_bricks].size = size_br;
    act_num_bricks++;
    if (act_num_bricks >= gBrick_array_size) {
      ROS_DEBUG("Lot of bricks.n");
      act_num_bricks--;
    } */
  }
}


static void find_componenets(Mat thr, enum Brick_type br_type) {

  // findContours(thr, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
  int i = 0;
  int hierarchy_lvl = 0;
  float pos_limit;
  if (brick_alt < 0.4) {
    min_len_brick = min_len_set2[(int)br_type - 1];
    max_len_brick = max_len_set2[(int)br_type - 1];
    pos_limit = 200.0;
  } else {
    min_len_brick = min_len_set1[(int)br_type - 1];
    max_len_brick = max_len_set1[(int)br_type - 1];
    pos_limit = 10.0;
  }

  hierarchy.clear();
  contours.clear();
  findContours(thr, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  // findContours(thr, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

  for (; i < (int)hierarchy.size() && i >= 0;) {
    double pix_size = contourArea(contours[i]);
    bool border = false;
    bool border_y = false;
    if (pix_size > min_size && pix_size < max_size && (hierarchy_lvl % 2) == 0) {
      Point2f center;
      Point2f zero;
      zero.x = zero.y = 0.0;
      real_c_num = 0;
      Moments mu;
      bool valid = true;

      real_contour.clear();
      real_c_num = 0;
      float min_rx = 1000.0, min_ry = 1000.0;
      if (contours[i].size() < 4000) {
        for (auto &p : contours[i]) {
          if ((p.x == 0) || (p.x == image_max_x) || (p.y == 0)) {
            border = true;
          }
          if (p.y == image_max_y) {
            border_y = true;
          }
          omnidir_undistortPoints((float)p.x, (float)p.y, real_c[real_c_num].x, real_c[real_c_num].y);
          real_contour.push_back(real_c[real_c_num]);
          float a_x = abs(real_c[real_c_num].x);
          if (a_x > pos_limit) {
            valid = false;
            break;
          }
          if (a_x < min_rx) {
            min_rx = a_x;
          }
          float a_y = abs(real_c[real_c_num].y);
          if (a_y < min_ry) {
            min_ry = a_y;
          }
          if (a_y > pos_limit) {
            valid = false;
            break;
          }

          if (debug_detector) {
            circle(*dbg, Point(p.x, p.y), 2, br_color[(int)br_type - 1], 1);
          }
          real_c_num++;
        }
      } else {
        valid = false;
      }
/*      if (valid && ((min_rx < 7.0) || (min_ry < 7.0))) {
        float convex_ratio;
        convexHull(real_contour, conv_hull);
        double hull_size = contourArea(conv_hull);
        double real_size = contourArea(real_contour);
        //valid = (real_size > convex_ratio * hull_size);
      } else {
        valid = false;
      }*/
      if (valid && border_y && brick_alt < 1.0) {  // BORDER contour
        approxPolyDP(conv_hull, conv_approx, 0.05, true);
        if (debug_detector) {
          for (int ii = 0; ii < (int)conv_hull.size(); ii++) {
            line(*dbg, Point(dbg_real_x(conv_hull[ii].x), dbg_real_y(conv_hull[ii].y)),
                 Point(dbg_real_x(conv_hull[(ii + 1) % conv_hull.size()].x), dbg_real_y(conv_hull[(ii + 1) % conv_hull.size()].y)), br_color[(int)br_type], 3, LINE_AA);
          }
          for (int ii = 0; ii < (int)conv_approx.size(); ii++) {
            line(*dbg, Point(dbg_real_x(conv_approx[ii].x), dbg_real_y(conv_approx[ii].y)),
                 Point(dbg_real_x(conv_approx[(ii + 1) % conv_approx.size()].x), dbg_real_y(conv_approx[(ii + 1) % conv_approx.size()].y)), br_color[(int)br_type - 1], 2, LINE_AA);
          }
        }
        Point2f center;
        float yaw_br, size_br;
        if (test_brick_bottom(conv_approx, br_type, center, yaw_br, size_br)) {
          add_new_brick(center, yaw_br, size_br, br_type, pix_size);
          if (debug_detector) {
            ROS_DEBUG_STREAM("BORDER brick " << size_br << " type " << br_type);
          }
        }

      } else if (valid && !border) {  // INSIDE CONTOUR
        Point2f center(0, 0);
        float b_len, b_yaw;

        //approxPolyDP(conv_hull, real_contour, 0.05, true); not workin in simulation and in real

        if (debug_detector) {
          for (int i_tmp = 0; i_tmp <= real_c_num; i_tmp++) {
            circle(*dbg, Point(dbg_real_x(real_c[i_tmp].x), dbg_real_y(real_c[i_tmp].y)), 3, br_color[(int)br_type - 1], 1);
          }
          Point2f vtx[4];
          box.points(vtx);
          for (int ii = 0; ii < 4; ii++) {
            vtx[ii].x = dbg_real_x(vtx[ii].x);
            vtx[ii].y = dbg_real_y(vtx[ii].y);
          }
          for (int ii = 0; ii < 4; ii++) {
            line(*dbg, vtx[ii], vtx[(ii + 1) % 4], br_color[(int)br_type - 1], 1, LINE_AA);
          }
        }
        /* 
        if (pointPolygonTest(real_contour, center, false) < 0) {  // not centered object, now skew
          orig_real_contour = real_contour;
          skew_contour(orig_real_contour, real_contour, (altitude / brick_alt));
          if (debug_detector) {
            for (int ii = 0; ii < (int)real_contour.size(); ii++) {
              line(*dbg, Point(dbg_real_x(real_contour[ii].x), dbg_real_y(real_contour[ii].y)),
                   Point(dbg_real_x(real_contour[(ii + 1) % real_contour.size()].x), dbg_real_y(real_contour[(ii + 1) % real_contour.size()].y)), br_color[(int)br_type], 1,
                   LINE_AA);
            }
          }
        }
        */
        box = minAreaRect(real_contour);
        mu = moments(real_contour);
        if (box.size.width < box.size.height) {
          b_len = box.size.height * brick_alt;
          b_yaw = (box.angle * M_PI / 180.0) + M_PI / 2.0;
        } else {
          b_len = box.size.width * brick_alt;
          b_yaw = (box.angle * M_PI / 180.0);
        }
        box.center.x *= brick_alt;
        box.center.y *= brick_alt;

        //if (correct_size(b_len, b_width, br_type, border)) {
          add_new_brick(box.center, b_yaw, b_len, br_type, pix_size);
        /*  if (debug_detector) {
            ROS_DEBUG_STREAM("CORRECTED SIZE " << b_len << ", " << b_width << " type " << br_type);
          }
        } else {
          float bo_len = b_len;
          float bo_width = b_width;
          box = minAreaRect(real_contour);
          mu = moments(real_contour);
          if (box.size.width < box.size.height) {
            b_len = box.size.height * brick_alt;
            b_width = box.size.width * brick_alt;
            b_yaw = (box.angle * M_PI / 180.0) + M_PI / 2.0;
          } else {
            b_len = box.size.width * brick_alt;
            b_width = box.size.height * brick_alt;
            b_yaw = (box.angle * M_PI / 180.0);
          }
          box.center.x *= brick_alt;
          box.center.y *= brick_alt;

          //if (correct_size(b_len, b_width, br_type, border)) {
            add_new_brick(box.center, b_yaw, b_len, br_type, pix_size);
             if (debug_detector) {
              ROS_DEBUG_STREAM("CORRECTED SIZE " << b_len << ", " << b_width << " type " << br_type);
            }

          } else {
            ROS_DEBUG_STREAM("WRONG SIZE old " << bo_len << ", " << bo_width << " new " << b_len << ", " << b_width << " type " << br_type);
          }
          
        }*/
      }
    }

    if (hierarchy[i][2] != -1) {
      i = hierarchy[i][2];
      hierarchy_lvl++;
    } else if (hierarchy[i][0] != -1) {
      i = hierarchy[i][0];
    } else {
      int next_i = hierarchy[i][3];
      hierarchy_lvl--;
      while (next_i != -1 && hierarchy[next_i][0] == -1) {
        next_i = hierarchy[next_i][3];
        hierarchy_lvl--;
      }
      if (next_i == -1) {
        break;
      }
      i = hierarchy[next_i][0];
    }
  }
}

//}

/* find_ugv_pattern() //{ */

static void find_ugv_pattern(Mat thr) {
  int i = 0;

  hierarchy.clear();
  contours.clear();
  findContours(thr, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

  for (; i < (int)hierarchy.size() && i >= 0;) {
    double pix_size = contourArea(contours[i]);
    
    int n_holes=0;
    if (hierarchy[i][2] != -1) {
      int h_i = hierarchy[i][2];
      while (h_i!=-1) {
        h_i = hierarchy[h_i][0];
        n_holes++;
      }
    }
    if (n_holes > 30) {
      bool valid = true;
      real_contour.clear();
      
      if (contours[i].size() < 4000) {
        for (auto &p : contours[i]) {
          omnidir_undistortPoints((float)p.x, (float)p.y, real_c[real_c_num].x, real_c[real_c_num].y);
          real_contour.push_back(real_c[real_c_num]);
          float a_x = abs(real_c[real_c_num].x);
          if (a_x > 10.0) {
            valid = false;
            break;
          }
          float a_y = abs(real_c[real_c_num].y);
          if (a_y > 10.0) {
            valid = false;
            break;
          }

          if (debug_detector) {
            circle(*dbg, Point(p.x, p.y), 5, Vec3b(0,200,200), 2);
          }
        }
      }
      if (valid) {
        RotatedRect box = minAreaRect(real_contour);
        if (act_num_bricks>0 && brick_arr[act_num_bricks-1].type == UGV_TYPE) {
          brick_arr[act_num_bricks-1].c_x = (brick_arr[act_num_bricks-1].c_x+box.center.x)/2.0;
          brick_arr[act_num_bricks-1].c_y = (brick_arr[act_num_bricks-1].c_y+box.center.y)/2.0;
          brick_arr[act_num_bricks].type = UGV_TYPE;
          brick_arr[act_num_bricks].size = (pix_size + brick_arr[act_num_bricks].size)/2;
          ROS_DEBUG("UGV pattern %f, %f, n holes %i", brick_arr[act_num_bricks-1].c_x, brick_arr[act_num_bricks-1].c_y, n_holes);
        } else {
          brick_arr[act_num_bricks].c_x = box.center.x;
          brick_arr[act_num_bricks].c_y = box.center.y;
          brick_arr[act_num_bricks].yaw = 0;
          brick_arr[act_num_bricks].type = UGV_TYPE;
          brick_arr[act_num_bricks].size = pix_size;
          ROS_DEBUG("UGV pattern %f, %f, n holes %i", brick_arr[act_num_bricks].c_x, brick_arr[act_num_bricks].c_y, n_holes);
          if (act_num_bricks >= gBrick_array_size) {
            ROS_DEBUG("Lot of bricks.n");
            act_num_bricks--;
          }
          act_num_bricks++;
        }
      } 
    }
    i = hierarchy[i][0];
  }
}

//}

/* find_ugv_pattern() //{ */

struct LinePolar
{
    float rho;
    float angle;
};

struct hough_cmp_gt
{
    hough_cmp_gt(const int* _aux) : aux(_aux) {}
    inline bool operator()(int l1, int l2) const
    {
        return aux[l1] > aux[l2] || (aux[l1] == aux[l2] && l1 < l2);
    }
    const int* aux;
};

static void
createTrigTable( int numangle, double min_theta, double theta_step,
                 float irho, float *tabSin, float *tabCos )
{
    float ang = static_cast<float>(min_theta);
    for(int n = 0; n < numangle; ang += (float)theta_step, n++ )
    {
        tabSin[n] = (float)(sin((double)ang) * irho);
        tabCos[n] = (float)(cos((double)ang) * irho);
    }
}

static void
findLocalMaximums( int numrho, int numangle, int threshold,
                   const int *accum, std::vector<int>& sort_buf )
{
    for(int r = 0; r < numrho; r++ )
        for(int n = 0; n < numangle; n++ )
        {
            int base = (n+1) * (numrho+2) + r+1;
            if( accum[base] > threshold &&
                accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
                accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2] )
                sort_buf.push_back(base);
        }
}


void HoughLinesPointSet( InputArray _point, OutputArray _lines, int lines_max, int threshold,
                         double min_rho, double max_rho, double rho_step,
                         double min_theta, double max_theta, double theta_step )
{
    std::vector<Vec3d> lines;
    std::vector<Point2f> point;
    _point.copyTo(point);

    CV_Assert( _point.type() == CV_32FC2 || _point.type() == CV_32SC2 );
    if( lines_max <= 0 ) {
        CV_Error( Error::StsBadArg, "lines_max must be greater than 0" );
    }
    if( threshold < 0) {
        CV_Error( Error::StsBadArg, "threshold must be greater than 0" );
    }
    if( ((max_rho - min_rho) <= 0) || ((max_theta - min_theta) <= 0) ) {
        CV_Error( Error::StsBadArg, "max must be greater than min" );
    }
    if( ((rho_step <= 0)) || ((theta_step <= 0)) ) {
        CV_Error( Error::StsBadArg, "step must be greater than 0" );
    }

    int i;
    float irho = 1 / (float)rho_step;
    float irho_min = ((float)min_rho * irho);
    int numangle = cvRound((max_theta - min_theta) / theta_step);
    int numrho = cvRound((max_rho - min_rho + 1) / rho_step);

    Mat _accum = Mat::zeros( (numangle+2), (numrho+2), CV_32SC1 );
    std::vector<int> _sort_buf;
    float tabSin[numangle];
    float tabCos[numangle];
    int *accum = _accum.ptr<int>();
    //float *tabSin = _tabSin.data(), *tabCos = _tabCos.data();

    // create sin and cos table
    createTrigTable( numangle, min_theta, theta_step,
                     irho, tabSin, tabCos );

    // stage 1. fill accumulator
    for( i = 0; i < (int)point.size(); i++ )
        for(int n = 0; n < numangle; n++ )
        {
            int r = cvRound( point.at(i).x  * tabCos[n] + point.at(i).y * tabSin[n] - irho_min);
            accum[(n+1) * (numrho+2) + r+1]++;
        }

    // stage 2. find local maximums
    findLocalMaximums( numrho, numangle, threshold, accum, _sort_buf );

    // stage 3. sort the detected lines by accumulator value
    std::sort(_sort_buf.begin(), _sort_buf.end(), hough_cmp_gt(accum));

    // stage 4. store the first min(total,linesMax) lines to the output buffer
    lines_max = std::min(lines_max, (int)_sort_buf.size());
    double scale = 1./(numrho+2);
    for( i = 0; i < lines_max; i++ ) {
        LinePolar line;
        int idx = _sort_buf[i];
        int n = cvFloor(idx*scale) - 1;
        int r = idx - (n+1)*(numrho+2) - 1;
        line.rho = static_cast<float>(min_rho) + r * (float)rho_step;
        line.angle = static_cast<float>(min_theta) + n * (float)theta_step;
        lines.push_back(Vec3d((double)accum[idx], (double)line.rho, (double)line.angle));
    }

    Mat(lines).copyTo(_lines);
}

static vector<Vec3d>  h_lines;

[[maybe_unused]] static void find_wall(Mat thr) {
  int i = 0;

//  imshow("Wall_o", thr);
  hierarchy.clear();
  contours.clear();
  findContours(thr, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  //cout << "# of segments:"<<hierarchy.size()<<endl;
  for (; i < (int)hierarchy.size() && i >= 0;) {
    int n_holes=0;
    int h_i = hierarchy[i][2];
    while (h_i!=-1) {
      h_i = hierarchy[h_i][0];
      n_holes++;
    }
    if (n_holes > 20) {
      //bool valid = true;
      real_contour.clear();
      
      int h_j = hierarchy[i][2];
      while (h_j!=-1) {
         Moments mu = moments( contours[h_j], false );
         omnidir_undistortPoints(mu.m10/mu.m00, mu.m01/mu.m00, real_c[real_c_num].x, real_c[real_c_num].y);
         float a_x = abs(real_c[real_c_num].x);
         if (a_x > 10.0) {
           //valid = false;
           h_j = hierarchy[h_j][0];
           continue;
         }
         float a_y = abs(real_c[real_c_num].y);
         if (a_y > 10.0) {
           //valid = false;
           h_j = hierarchy[h_j][0];
           continue;
         }
         real_contour.push_back(real_c[real_c_num]);
         h_j = hierarchy[h_j][0];
      }
      double rhoMin = 0.0f, rhoMax = 4.0f, rhoStep = 0.2;
      double thetaMin = 0.0f, thetaMax = CV_PI / 2.0f, thetaStep = CV_PI / 36.0f;
      h_lines.clear();
      HoughLinesPointSet(real_contour, h_lines, 20, 5,
                       rhoMin, rhoMax, rhoStep,
                       thetaMin, thetaMax, thetaStep);
      
      /*RotatedRect box = minAreaRect(real_contour);
      if (act_num_bricks>0 && brick_arr[act_num_bricks-1].type == UGV_TYPE) {
        brick_arr[act_num_bricks-1].c_x = (brick_arr[act_num_bricks-1].c_x+box.center.x)/2.0;
        brick_arr[act_num_bricks-1].c_y = (brick_arr[act_num_bricks-1].c_y+box.center.y)/2.0;
        brick_arr[act_num_bricks].type = UGV_TYPE;
        brick_arr[act_num_bricks].size = (pix_size + brick_arr[act_num_bricks].size)/2;
        ROS_DEBUG("Wall pattern %f, %f, n holes %i", brick_arr[act_num_bricks-1].c_x, brick_arr[act_num_bricks-1].c_y, n_holes);
      } else {
        brick_arr[act_num_bricks].c_x = box.center.x;
        brick_arr[act_num_bricks].c_y = box.center.y;
        brick_arr[act_num_bricks].yaw = 0;
        brick_arr[act_num_bricks].type = UGV_TYPE;
        brick_arr[act_num_bricks].size = pix_size;
        ROS_DEBUG("Wall pattern %f, %f, n holes %i", brick_arr[act_num_bricks].c_x, brick_arr[act_num_bricks].c_y, n_holes);
        if (act_num_bricks >= gBrick_array_size) {
          ROS_DEBUG("Lot of bricks.n");
          act_num_bricks--;
        }
        act_num_bricks++;
      }
      */
      /*cout << "Find "<<h_lines.size()<<" lines.";
      if (h_lines.size()>0) {
        cout <<" Best line votes:"<<h_lines[0][0]<<endl;
      } else {
        cout <<endl;
      }
      */
    }
    i = hierarchy[i][0];
  }
}


//}

/* find_white_componenets //{ */

static void add_white_plate(Point2f &center, float yaw_br, float size_br, enum Brick_type br_type) {
  if (debug_detector) {
    circle(*dbg, Point(dbg_real_x(center.x / brick_alt), dbg_real_y(center.y / brick_alt)), 9, Scalar(200, 200, 200), 2);
    //ROS_DEBUG_STREAM("BORDER white center:"<<center.x<<","<<center.y<<" size:" << size_br << " type " << br_type);
  }

  white_arr[act_num_white].c_x = center.x;
  white_arr[act_num_white].c_y = center.y;
  white_arr[act_num_white].type = br_type;
  white_arr[act_num_white].yaw = yaw_br;
  white_arr[act_num_white].size = size_br;
  white_arr[act_num_white].id=1;
  white_to_brick[act_num_white]=-1;
  white_to_size[act_num_white]=0.0;
/*
  if (local_brick_num > 0) {
    float min_dist = my_sqr(local_brick[0].c_x - center.x) + my_sqr(local_brick[0].c_y - center.y);
    int min_loc = 0;
    for (int i = 1; i < local_brick_num; i++) {
      float d = my_sqr(local_brick[i].c_x - center.x) + my_sqr(local_brick[i].c_y - center.y);
      if (d < min_dist) {
        min_loc = i;
        min_dist = d;
      }
    }
    ROS_DEBUG_STREAM("White plate " << center.x << "," << center.y << " distance to local_brick:" << min_dist << " ind:" << min_loc);
    if (min_dist < 0.10) {
      white_to_local[act_num_white] = min_loc;
      if (local_to_white[min_loc] == -1 || min_dist < local_dist_white[min_loc]) {
        local_to_white[min_loc] = act_num_white;
        local_dist_white[min_loc] = min_dist;
      }
    }
  }
  */
  act_num_white++;
  if (act_num_white >= _WHITE_MAX_NUM) {
    ROS_DEBUG("Lot of white plates.n");
    act_num_white--;
  }
}

static void finish_calibration() {
  if (act_num_white>0) {
    max_w_sat=_WHITE_SAT_MAX;
    min_w_val=180;
    max_w_val=255;
    int min_white_val=255;
    int max_white_sat=0;
    int min_shadow_val=255;
    int max_shadow_sat=0;
    erode(white_calib, white_calib_erod, erosion_element);
    //imshow("CenEro", white_calib_erod);
    for (int v = 0; v < white_calib_erod.rows; v++) {
      for (int u = 0; u < white_calib_erod.cols; u++) {
        if (white_calib_erod.at<unsigned char>(v,u)>200) {
          Vec3b wh= hsv_src.at<Vec3b>(v,u);
          if (wh[2]<180) {
            if (wh[2]<min_shadow_val) {
              min_shadow_val = wh[2];
            }
            if (wh[1]>max_shadow_sat) {
              max_shadow_sat = wh[1];
            }
          } else {
            if (wh[2]<min_white_val) {
              min_white_val = wh[2];
            }
            if (wh[1]>max_white_sat) {
              max_white_sat = wh[1];
            }
          }
        }
      }
    }
    if (max_white_sat+5<_WHITE_SAT_MAX) {
      max_w_sat=max_white_sat+5;
    }
    if (min_white_val-8>180) {
      min_w_val=min_white_val-8;
    }
    max_w_val=255;
    if (min_shadow_val<180) {
      if (max_shadow_sat<64) {
        max_sh_sat=max_shadow_sat;
      } else {
        max_sh_sat = 64;
      }
      min_sh_val=min_shadow_val;
      max_sh_val=255;
      use_shadow = true;
    } else {
      use_shadow = false;
    }
    ROS_DEBUG("WHITE CALIBRATION Min val white %i, max sat white %i, min val shadow %i, max sat shadow %i used: Min val white %i, max sat white %i, min val shadow %i, max sat shadow %i", min_white_val, max_white_sat, min_shadow_val, max_shadow_sat, min_w_val, max_w_sat, min_sh_val, max_sh_sat );
    update_white_color();
  } else {
    max_w_sat=_WHITE_SAT_MAX;
    min_w_val=180;
    max_w_val=255;
    max_sh_sat=0;
    min_sh_val=0;
    max_sh_val=0;
    use_shadow = false;
    update_white_color();
    ROS_DEBUG("No white plate - WHITE CALIBRATION RESET");
  }
}

static void calibrate_white(vector<Point> c) {
  convexHull(c, conv_hull_pix);
  fillConvexPoly(white_calib, conv_hull_pix, Scalar(255));
}

static void find_white_componenets(Mat thr, bool detect_shadow) {
  enum Brick_type br_type = WHITE_TYPE;
  int i = 0;
  int hierarchy_lvl = 0;
  float pos_limit;
  float convex_ratio;
  pos_limit = 15.0;

  if (detect_shadow) {
    white_calib.create(thr.size(), CV_8U);
    white_calib.setTo(0);
  } else {
    max_w_sat=_WHITE_SAT_MAX;
    min_w_val=180;
    max_w_val=255;
    use_shadow = false;
    update_white_color();
  }

  if (brick_alt < 0.4) {
    min_len_brick = min_len_set2[(int)br_type - 1];
    max_len_brick = max_len_set2[(int)br_type - 1];
    convex_ratio = 0.5;
  } else {
    min_len_brick = min_len_set1[(int)br_type - 1];
    max_len_brick = max_len_set1[(int)br_type - 1];
    convex_ratio = 0.7;
  }

  hierarchy.clear();
  contours.clear();
  findContours(thr, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  // findContours(thr, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

  for (; i < (int)hierarchy.size() && i >= 0;) {
    double size = contourArea(contours[i]);
    bool border = false;
    bool border_y = false;

    if (size > min_size && size < max_size && (hierarchy_lvl % 2) == 0) {
      Point2f center;
      Point2f zero;
      zero.x = zero.y = 0.0;
      real_c_num = 0;
      Moments mu;
      bool valid = true;

      real_contour.clear();
      real_c_num = 0;
      float min_rx = 1000.0, min_ry = 1000.0;
      if (contours[i].size() < 5000) {
        for (auto &p : contours[i]) {
          if ((p.x == 0) || (p.x == image_max_x) || (p.y == 0)) {
            border = true;
          }
          if (p.y == image_max_y) {
            border_y = true;
          }
          omnidir_undistortPoints((float)p.x, (float)p.y, real_c[real_c_num].x, real_c[real_c_num].y);
          real_c[real_c_num].x *= brick_alt;
          real_c[real_c_num].y *= brick_alt;
          real_contour.push_back(real_c[real_c_num]);
          float a_x = abs(real_c[real_c_num].x);
          if (a_x > pos_limit) {
            valid = false;
            break;
          }
          if (a_x < min_rx) {
            min_rx = a_x;
          }
          float a_y = abs(real_c[real_c_num].y);
          if (a_y < min_ry) {
            min_ry = a_y;
          }
          if (a_y > pos_limit) {
            valid = false;
            break;
          }
          if (debug_detector) {
            circle(*dbg, Point(p.x, p.y), 2, br_color[(int)br_type - 1], 1);
          }
          real_c_num++;
        }
      } else {
        valid = false;
      }
      if (valid && ((min_rx < 5.0) || (min_ry < 5.0))) {
        convexHull(real_contour, conv_hull);
        double hull_size = contourArea(conv_hull);
        double real_size = contourArea(real_contour);
        //ROS_DEBUG_STREAM("Convex hull ratio " << real_size / hull_size);
        valid = (real_size > convex_ratio * hull_size);
      }
      if (valid && border_y && brick_alt < 1.8) {  // BORDER contour
        approxPolyDP(conv_hull, conv_approx, 0.05, true);
        if (debug_detector) {
          for (int ii = 0; ii < (int)conv_hull.size(); ii++) {
            line(*dbg, Point(dbg_real_x(conv_hull[ii].x / brick_alt), dbg_real_y(conv_hull[ii].y / brick_alt)),
                 Point(dbg_real_x(conv_hull[(ii + 1) % conv_hull.size()].x / brick_alt), dbg_real_y(conv_hull[(ii + 1) % conv_hull.size()].y / brick_alt)),
                 br_color[(int)br_type - 3], 3, LINE_AA);
          }
          for (int ii = 0; ii < (int)conv_approx.size(); ii++) {
            line(*dbg, Point(dbg_real_x(conv_approx[ii].x / brick_alt), dbg_real_y(conv_approx[ii].y / brick_alt)),
                 Point(dbg_real_x(conv_approx[(ii + 1) % conv_approx.size()].x / brick_alt), dbg_real_y(conv_approx[(ii + 1) % conv_approx.size()].y / brick_alt)),
                 br_color[(int)br_type - 1], 2, LINE_AA);
          }
        }
        Point2f center;
        float yaw_br, size_br;
        if (test_white_bottom(conv_approx, br_type, center, yaw_br, size_br)) {
          add_white_plate(center, yaw_br, size_br, br_type);
          if (detect_shadow) {
            calibrate_white(contours[i]);
          }
        }
      } else if (valid && !border) {  // INSIDE CONTOUR
        float b_len, b_width, b_yaw;
        box = minAreaRect(real_contour);
        mu = moments(real_contour);

        if (debug_detector) {
          for (int i_tmp = 0; i_tmp <= real_c_num; i_tmp++) {
            circle(*dbg, Point(dbg_real_x(real_c[i_tmp].x / brick_alt), dbg_real_y(real_c[i_tmp].y / brick_alt)), 3, br_color[(int)br_type - 1], 1);
          }
          Point2f vtx[4];
          box.points(vtx);
          for (int ii = 0; ii < 4; ii++) {
            vtx[ii].x = dbg_real_x(vtx[ii].x / brick_alt);
            vtx[ii].y = dbg_real_y(vtx[ii].y / brick_alt);
          }
          for (int ii = 0; ii < 4; ii++) {
            line(*dbg, vtx[ii], vtx[(ii + 1) % 4], br_color[(int)br_type - 1], 1, LINE_AA);
          }
        }

        if (box.size.width < box.size.height) {
          b_len = box.size.height;
          b_width = box.size.width;
          b_yaw = (box.angle * M_PI / 180.0) + M_PI / 2.0;
        } else {
          b_len = box.size.width;
          b_width = box.size.height;
          b_yaw = (box.angle * M_PI / 180.0);
        }

        if (correct_white_size(b_len, b_width)) {
          //ROS_DEBUG_STREAM("YES WHITE plate! " << b_len << ", " << b_width << " type " << br_type);
          add_white_plate(box.center, b_yaw, b_len, br_type);
          if (detect_shadow) {
            calibrate_white(contours[i]);
          }
        }
      }
    }

    if (hierarchy[i][2] != -1) {
      i = hierarchy[i][2];
      hierarchy_lvl++;
    } else if (hierarchy[i][0] != -1) {
      i = hierarchy[i][0];
    } else {
      int next_i = hierarchy[i][3];
      hierarchy_lvl--;
      while (next_i != -1 && hierarchy[next_i][0] == -1) {
        next_i = hierarchy[next_i][3];
        hierarchy_lvl--;
      }
      if (next_i == -1) {
        break;
      }
      i = hierarchy[next_i][0];
    }
  }
  if (detect_shadow) {
    finish_calibration();
  }
}

//}

static int img_index = 0;
string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

int min_sizes[4] = {100, 200, 400, 500};
int max_sizes[4] = {9000, 18000, 30000, 42000};

/* find_bricks() //{ */

int find_bricks(Mat src, brick *brick_array, int brick_array_size, int input_bricks, bool use_gui, bool gui_red, bool gui_green, bool gui_blue, float &distance, float sin_a,
                float sin_b, float cos_a, float cos_b, int mode) {

  [[maybe_unused]] double t = (double)getTickCount();
  rot_eu = angle2rot(sin_a, sin_b, cos_a, cos_b);
  if (distance < 0.25) {
    distance = 0.25;
  }
  double min_koef = 1.0;
  if (distance < 1.0) {
    min_koef = distance;
  }
  if (distance < 2.0) {
    if (input_bricks > 10) {
      input_bricks = 10;
    }
    local_brick_num = input_bricks;
    for (int i = 0; i < input_bricks; i++) {
      local_brick[i] = brick_array[i];
      local_to_white[i] = -1;
      local_dist_white[i] = 10000.0;
    }
  } else {
    local_brick_num = 0;
  }

  debug_detector = use_gui;
  brick_alt = distance - BRICK_SIZE;
  altitude = distance;
  brick_arr = brick_array;
  gBrick_array_size = brick_array_size;
  act_num_bricks = 0;
  act_num_white = 0;
  Size pic_size = src.size();
  height = pic_size.height;
  width = pic_size.width;
  image_max_x = width - 1;
  image_max_y = height - 1;
  vector<Vec3f> circles;

  if (use_gui) {
    if (is_mask) {
      addWeighted(src, 0.7, mask_c, 0.3, 0, img, CV_8UC3);
    } else {
      img = src.clone();
    }
    dbg = &img;
  }

  cvtColor(src, hsv_src, COLOR_BGR2HSV);
  
  if (mode>=0 && mode<8) {
    inRange(hsv_src, min_white, max_white, white_thr);

    if (use_shadow) {
      inRange(hsv_src, min_shadow, max_shadow, white_shadow);
      int num_shadow = countNonZero(white_shadow);
      //cout << "SHADOW:"<<num_shadow<<" img size half:"<<(height*width/2)<<endl;
      if (num_shadow < height*width/2) {
        enlarge_or(white_thr, white_shadow);
      }
    }
    if (distance<1.5) {
      dilate(white_thr, thr_pre_mask, erosion_element);
      erode(thr_pre_mask, white_erode_thr, erosion_element);
    } else {
      dilate(white_thr, thr_pre_mask, erosion_element);
      erode(thr_pre_mask, white_erode_thr, erosion_element);
    }
    
    if (simul_camera) {
      min_size = (int)(min_koef * (float)min_sizes[0] / (brick_alt * brick_alt));
      max_size = (int)((float)1.5 * max_sizes[0] / (brick_alt * brick_alt));
    } else {
      min_size = (int)(min_koef * (float)min_sizes[0] / (brick_alt * brick_alt));
      max_size = (int)((float)max_sizes[0] / (brick_alt * brick_alt));
    }
    find_white_componenets(white_erode_thr, (distance<2.0));
    if (act_num_white==0) {
      int num_white = countNonZero(white_erode_thr);
      if (num_white> height*width/2) {
        max_w_sat = 7;
        update_white_color();
        inRange(hsv_src, min_white, max_white, white_thr);
        if (distance<1.5) {
          dilate(white_thr, thr_pre_mask, erosion_element);
          erode(thr_pre_mask, white_erode_thr, erosion_element);
        } else {
          dilate(white_thr, thr_pre_mask, erosion_element);
          erode(thr_pre_mask, white_erode_thr, erosion_element);
        }
        find_white_componenets(white_erode_thr, (distance<2.0));
        //imshow("white2", white_erode_thr);
      }
    }
/*  } else {
    inRange(hsv_src, min_wall, max_wall, wall_thr);
    find_wall(wall_thr); */
  }

  if ((mode==0) || (mode==1)) {
    inRange(hsv_src, min_red, max_red, red_thr1);
    inRange(hsv_src, min_red2, max_red2, red_thr2);
    if (is_mask) {
      bitwise_or(red_thr1, red_thr2, thr_pre_mask);
//      enlarge_or(thr_pre_mask, white_thr);
      bitwise_and(thr_pre_mask, bf_mask, red_thr);
      correct_mask(red_thr);
    } else {
      bitwise_or(red_thr1, red_thr2, red_thr);
//      enlarge_or(red_thr, white_thr);
    }
    if (simul_camera) {
      min_size = (int)(min_koef * (float)min_sizes[0] / (brick_alt * brick_alt));
      max_size = (int)((float)1.5 * max_sizes[0] / (brick_alt * brick_alt));
    } else {
      min_size = (int)(min_koef * (float)min_sizes[0] / (brick_alt * brick_alt));
      max_size = (int)((float)max_sizes[0] / (brick_alt * brick_alt));
    }
    find_componenets(red_thr, RED_TYPE);
    if (mode==0) {
      find_ugv_pattern(red_thr);
//      imshow("red", red_thr);
      dilate(red_thr, thr_pre_mask, erosion_small);
      erode(thr_pre_mask, red_thr, erosion_small);
//      imshow("clesed_red", red_thr);
      find_ugv_pattern(red_thr);
    }
  }
  
  if ((mode==0) || (mode==2)) {
    if (is_mask) {
      inRange(hsv_src, min_green, max_green, thr_pre_mask);
      //imshow("we", white_erode_thr);
//      enlarge_or(thr_pre_mask, white_thr);
      bitwise_and(thr_pre_mask, bf_mask, green_thr);
      correct_mask(green_thr);
    } else {
      inRange(hsv_src, min_green, max_green, green_thr);
//      enlarge_or(green_thr, white_thr);
    }
    if (simul_camera) {
      min_size = (int)(0.7 * min_koef * (float)min_sizes[1] / (brick_alt * brick_alt));
      max_size = (int)((float)1.5 * max_sizes[1] / (brick_alt * brick_alt));
    } else {
      min_size = (int)(min_koef * (float)min_sizes[1] / (brick_alt * brick_alt));
      max_size = (int)((float)max_sizes[1] / (brick_alt * brick_alt));
    }
    find_componenets(green_thr, GREEN_TYPE);
  }
  
  if ((mode==0) || (mode==3)) {
    if (is_mask) {
      inRange(hsv_src, min_blue, max_blue, thr_pre_mask);
      if (mode==3) {
        erode(thr_pre_mask, blue_thr, erosion_small);
        dilate(blue_thr, thr_pre_mask, erosion_small);
      }
//      enlarge_or(thr_pre_mask, white_thr);
      bitwise_and(thr_pre_mask, bf_mask, blue_thr);
      correct_mask(blue_thr);
    } else {
      inRange(hsv_src, min_blue, max_blue, blue_thr);
//      enlarge_or(blue_thr, white_thr);
    }
    if (simul_camera) {
      min_size = (int)(0.66 * min_koef * (float)min_sizes[2] / (brick_alt * brick_alt));
      max_size = (int)((float)1.5 * max_sizes[2] / (brick_alt * brick_alt));
    } else {
      min_size = (int)(0.66 * min_koef * (float)min_sizes[2] / (brick_alt * brick_alt));
      max_size = (int)((float)max_sizes[2] / (brick_alt * brick_alt));
    }
    find_componenets(blue_thr, BLUE_TYPE);
  }
  
  /*   inRange(hsv_src, min_orange, max_orange, orange_thr);
  min_size = (int)(0.53*min_koef*(float)min_sizes[3] / (brick_alt * brick_alt));
  max_size = (int)((float)max_sizes[3] / (brick_alt * brick_alt));
  find_componenets(orange_thr, ORANGE_TYPE, use_gui);

 */

  /*   Mat bgr[3];
     split(src, bgr);
     Mat r_lap[6];
     Mat lap[6];
     Mat f_lap;
     for (int ii=0; ii<3; ii++) {
        Laplacian(bgr[ii], r_lap[ii], CV_16S, 3);
        convertScaleAbs( r_lap[ii], lap[ii]);
     }
     for (int ii=0; ii<3; ii++) {
        Canny(bgr[ii], lap[ii], 20, 60, 3);
     }
     add(lap[0],lap[1],f_lap);
     add(f_lap, lap[2], f_lap);
     Mat hsv[3];
     split(hsv_src, hsv);
     for (int ii=0; ii<3; ii++) {
        Laplacian(hsv[ii], r_lap[ii+3], CV_16S, 3);
        convertScaleAbs( r_lap[ii+3], lap[ii+3]);
     }
     imshow("Laplacian rgb", f_lap);
     //imshow("Laplacian hsv 0", lap[3]);
     //imshow("Laplacian hsv 1", lap[4]);
     //imshow("Laplacian hsv 2", lap[5]);
     //imshow("Laplacian hsv", red_thr2);
     */

  // GaussianBlur(src, img, Size(3, 3), 0, 0, BORDER_DEFAULT);
  // Convert the image to grayscale
  // cvtColor(src, src_gray, COLOR_BGR2GRAY);

  // if (use_gui) {
  //  double t1 = (double)getTickCount();
  //}

  // waitKey(0);

  /*
  Mat bgr[3];
  split(src, bgr);
  Mat grad_x[3];
  Mat grad_y[3];
  Mat tmp_x, tmp_y, out, out2, grad;
  int ksize = 3; // test -1
  int ddepth = CV_16S;
  for (int ii=0; ii<3; ii++) {
     Sobel(bgr[ii], tmp_x, ddepth, 1, 0, ksize);
     Sobel(bgr[ii], tmp_y, ddepth, 0, 1, ksize);
     convertScaleAbs(tmp_x, grad_x[ii]);
     convertScaleAbs(tmp_y, grad_y[ii]);
     if (ii==0) {
        max(grad_x[ii], grad_y[ii], out);
     } else {
        max(grad_x[ii], out, out);
        max(grad_y[ii], out, out);
     }
  }
  */
  /*
   Mat grad_x, grad_y;
   Mat abs_grad_x, abs_grad_y;
   int scale=1;
   int delta=0;
   Sobel(src_gray, grad_x, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT);
   Sobel(src_gray, grad_y, ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT);
   // converting back to CV_8U
   convertScaleAbs(grad_x, abs_grad_x);
   convertScaleAbs(grad_y, abs_grad_y);
   addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);


   MATLAB
       rx = imfilter(im(:,:,1), xfilter);
       gx = imfilter(im(:,:,2), xfilter);
       bx = imfilter(im(:,:,3), xfilter);

       ry = imfilter(im(:,:,1), yfilter);
       gy = imfilter(im(:,:,2), yfilter);
       by = imfilter(im(:,:,3), yfilter);

       Jx = rx.^2 + gx.^2 + bx.^2;
       Jy = ry.^2 + gy.^2 + by.^2;
       Jxy = rx.*ry + gx.*gy + bx.*by;

       %compute first (greatest) eigenvalue of 2x2 matrix J'*J.
       %note that the abs() is only needed because some values may be slightly
       %negative due to round-off error.
       D = sqrt(abs(Jx.^2 - 2*Jx.*Jy + Jy.^2 + 4*Jxy.^2));
       e1 = (Jx + Jy + D) / 2;
       %the 2nd eigenvalue would be:  e2 = (Jx + Jy - D) / 2;
       edge_magnitude = sqrt(e1);

   */
  // imshow("Sobel max", out);
  /*
  if (act_num_bricks < local_brick_num) {
    for (int i = 0; i < local_brick_num; i++) {
      if (white_arr[local_to_white[i]].id>0 && local_to_white[i] >= 0) {
        ROS_DEBUG("Adding brick only by white plane to %f, %f, type %i",white_arr[local_to_white[i]].c_x, white_arr[local_to_white[i]].c_y, local_brick[i].type);
        brick_arr[act_num_bricks] = white_arr[local_to_white[i]];
        brick_arr[act_num_bricks].type = local_brick[i].type;
        act_num_bricks++;
        if (act_num_bricks >= gBrick_array_size) {
          ROS_DEBUG("Lot of bricks.n");
          act_num_bricks--;
        }
      }
    }
  }
  */
  if (gui_red) {
    imshow("red", red_thr);
  }

  if (gui_green) {
    imshow("green", green_thr);
  }

  if (gui_blue) {
    imshow("blue", blue_thr);
  }
  if (use_gui) {
    imshow("bfdbg", img);

    int key = waitKey(20) & 0xff;
    if (key == 's') {
      char name[100];
      snprintf(name, 90, "/home/petr/brick%05i.bmp", img_index);
      imwrite(name, src);
      snprintf(name, 90, "/home/petr/brick%05i.info", img_index);
      FILE *f = fopen(name, "wt");
      if (f != NULL) {
        fprintf(f, "%f %f %f %f %f\n", distance, sin_a, sin_b, cos_a, cos_b);
        fclose(f);
      }
      ROS_DEBUG_STREAM("Saved image " << name);
      img_index++;
    }
  }
  //imwrite("/home/petr/big/brick_color.bmp", src);

  return act_num_bricks;
}

//}

#ifdef _DEBUG_DETECTOR_MAIN

#define BRICK_ARRAY_SIZE 256
static brick brick_array[BRICK_ARRAY_SIZE];

/* main() //{ */

int main(int argc, char *argv[]) {

  float altitude = 1.4;
  double k1, k2, k3, k4, k5;
  cam_cx = 373.884480;
  cam_cy = 250.852182;
  k1 = -2.621542e+02;
  k2 = 0.0;
  k3 = 1.344121e-03;
  k4 = -2.588143e-07;
  k5 = 2.749293e-09;
  intrinsic = (cv::Mat_<float>(3, 3) << 261.977816, 0.0, cam_cx, 0.0, 261.775019, cam_cy, 0.0, 0.0, 1.0);
  distCoeffs = (cv::Mat_<float>(1, 5) << k1, k2, k3, k4, k5);

  brick_init();
  set_camera_param(intrinsic, distCoeffs, false);

  if (argc < 2) {
    img = imread("brick00000.bmp");
  } else {
    img = imread(argv[1]);
  }
  imshow("read", img);
  waitKey(0);
  int detectedObjects = find_bricks(img, brick_array, BRICK_ARRAY_SIZE, true, altitude, 0, 0, 1, 1);
  ROS_DEBUG_STREAM("Detected " << detectedObjects);
  waitKey(0);
  return 0;
}

//}

#endif

}  // namespace brick_detection
