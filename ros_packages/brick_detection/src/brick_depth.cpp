// clang: PetrFormat

#include "brick.h"

using namespace cv;
using namespace std;

namespace brick_detection
{

//#define _DEBUG_MASK
//#define _DEBUG1

static float brick_len[5] = {0.3, 0.6, 1.2, 1.8, 4.0};
// static float brick_len_err[4]  = {0.05, 0.1, 1.2, 1.8};

static const Vec3b bcolors[] = {Vec3b(0, 0, 255),   Vec3b(0, 255, 0),   Vec3b(0, 255, 255), Vec3b(255, 0, 0),   Vec3b(255, 128, 0),
                                Vec3b(255, 255, 0), Vec3b(0, 128, 255), Vec3b(255, 0, 255), Vec3b(255, 0, 128), Vec3b(128, 0, 255)};

static const Vec3b brick_colors[] = {Vec3b(250, 255, 255), Vec3b(10, 10, 250), Vec3b(10, 250, 10), Vec3b(250, 10, 10), Vec3b(10, 150, 250), Vec3b(250, 50, 250)};

float wall_offset;

#define _LOCAL_BRICK_NUM 40
static int local_brick_num;
static brick local_brick[_LOCAL_BRICK_NUM];

static brick *brick_arr;
static int glob_brick_array_size;
static int glob_brick_num;
static int l_border;

static int is_mask, is_last_obj_mask, is_last_obj_cnt;
static Mat rs_mask, rs_mini_mask, rs_last_object_mask;
static Mat rs_object_mask, rs_depth_mask;
static Mat mini, mini_seg, mini_thr;
static Mat dbg_mini, dbg_mini2;

static float object_z;

static bool is_near;
static bool simulation;
static bool show_debug;

static int find_brick, to_brick, wall_lvl;

#define _SCALE 1000.0

#define _BRICK_HIGH 0.2
#define _BRICK_WIDTH 0.2
#define _BRICK_SHIFT 0.12
#define _BRICK_LIMIT 0.12

#define _BRICK_GAP 0.1

static float _WALL_HIGH;
static float _WALL_LIMIT;
static float _MIN_WALL_LEN;
#define _WALL_WIDTH 0.32


//#define MIN_DIST_DIFF 0.3
#define PLANE_DIFF 0.08

#define my_sqr(a) ((a) * (a))
#define my_abs(x) ((x >= 0) ? (x) : (-(x)))

unsigned short int inline sabs_diff(unsigned short int a, unsigned short int b) {
  if (a > b) {
    return a - b;
  }
  return b - a;
}

float inline fabs_diff(float a, float b) {
  if (a > b) {
    return a - b;
  }
  return b - a;
}

inline float dist(Point3f a, Point3f b) {
  return sqrt(my_sqr(a.x - b.x) + my_sqr(a.y - b.y) + my_sqr(a.z - b.z));
}

float yaw_dif(float y1, float y2) {
  float ret = y1 - y2;
  while (ret > M_PI_2) {
    ret -= M_PI;
  }
  while (ret < -M_PI_2) {
    ret += M_PI;
  }
  return ret;
}

float yaw_avg(float y1, float y2) {
  while (y1 < y2 - M_PI_2) {
    y1 += M_PI;
  }
  while (y1 > y2 + M_PI_2) {
    y1 -= M_PI;
  }
  float ret = (y1 + y2) / 2.0;
  while (ret > M_PI_2) {
    ret -= M_PI;
  }
  while (ret < -M_PI_2) {
    ret += M_PI;
  }
  return ret;
}

float yaw_update(float rel_yaw, float map_yaw) {
  while (rel_yaw < map_yaw - M_PI_2) {
    rel_yaw += M_PI;
  }
  while (rel_yaw > map_yaw + M_PI_2) {
    rel_yaw -= M_PI;
  }
  while (rel_yaw > M_PI) {
    rel_yaw -= 2.0 * M_PI;
  }
  while (rel_yaw < -M_PI) {
    rel_yaw += 2.0 * M_PI;
  }
  return rel_yaw;
}

/*
int buf_x[4096];
int buf_y[4096];
int buf_first;
int buf_last;

static void stack_init() {
  buf_first = buf_last = 0;
}

static int stack_empty() {
  return buf_first == buf_last;
}

static int stack_pop(int *x, int *y) {
  if (buf_first != buf_last) {
    *x = buf_x[buf_first];
    *y = buf_y[buf_first++];
    buf_first &= 4095;
    return 1;
  } else {
    return 0;
  }
}

static int stack_push(int x, int y) {
  int old_last = buf_last;
  buf_x[buf_last] = x;
  buf_y[buf_last++] = y;
  buf_last &= 4095;
  if (buf_last == buf_first) {
    buf_last = old_last;
    return 0;
  }
  return 1;
}
*/

static float r_cx = 638.503113;
static float r_cy = 360.834778;
static float r_fx = 648.361145;
static float r_fy = 648.361145;

#ifdef _DEBUG
static int img_num = 0;
#endif

static Mat erosion_element, erosion_element_wall;

void realsense_load_mask(const char *mask_name) {
  int erosion_size = 2;
  erosion_element = getStructuringElement(MORPH_RECT, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));
  erosion_element_wall = getStructuringElement(MORPH_RECT, Size(5, 5), Point(2, 2));
  rs_mask = imread(mask_name, IMREAD_GRAYSCALE);
  if (rs_mask.empty()) {
    is_mask = 0;
  } else {
    is_mask = 1;
    Size size = rs_mask.size();
    Size size_mini(size.width / 8, size.height / 8);
    rs_mini_mask.create(size_mini, CV_8U);
    rs_mini_mask.setTo(250);
    int v_m = 0;
    for (int v = 0; v < rs_mask.rows; v++) {
      int u_m = 0;
      for (int u = 0; u < rs_mask.cols; u++) {
        if (rs_mask.at<unsigned char>(v, u) > 50) {
          if (rs_mini_mask.at<unsigned char>(v_m, u_m) == 250) {
            rs_mini_mask.at<unsigned char>(v_m, u_m) = 255;
          } else if (rs_mini_mask.at<unsigned char>(v / 8, u / 8) == 0) {
            rs_mini_mask.at<unsigned char>(v_m, u_m) = 128;
          }
        } else {
          if (rs_mini_mask.at<unsigned char>(v_m, u_m) == 250) {
            rs_mini_mask.at<unsigned char>(v_m, u_m) = 0;
          } else if (rs_mini_mask.at<unsigned char>(v_m, u_m) == 255) {
            rs_mini_mask.at<unsigned char>(v_m, u_m) = 128;
          }
        }
        if ((u % 8) == 7) {
          u_m++;
        }
      }
      if ((v % 8) == 7) {
        v_m++;
      }
    }
  }
}

void realsense_camera_param(float cx, float cy, float fx, float fy, bool simul) {
  r_cx = cx;
  r_cy = cy;
  r_fx = fx;
  r_fy = fy;
  simulation = simul;
  if (simul) {
    _WALL_HIGH = 1.7;
    _WALL_LIMIT = 1.35;
  } else {
    _WALL_HIGH = 1.8;
    _WALL_LIMIT = 1.2;
  }
  is_last_obj_mask = 0;
  is_last_obj_cnt = 0;
}

typedef struct
{
  int start, end;
  float v_x, v_y, v_c;
} segment;

/*[[maybe_unused]] static int n_segments[1024];
[[maybe_unused]] static segment segments[1024][128];
[[maybe_unused]] static int u_coord[2048];
static Point2f line_p[2048];

[[maybe_unused]] static int find_lines([[maybe_unused]] Point2f l[], [[maybe_unused]] int coord[], [[maybe_unused]] int size, [[maybe_unused]] segment s[]) {
  return 0;
}
*/

int diff_vec(Eigen::Vector3f a, Eigen::Vector3f b, double &d_alfa, double &d_beta) {
  double sqr_az = a[0] * a[0] + a[2] * a[2];
  double disk = sqr_az - b[0] * b[0];
  // float ax, ay,
  float az = 0;
  int ret = 0;
  if (disk >= 0.0) {
    double cos_a = (a[0] * b[0] + a[2] * sqrt(disk)) / sqr_az;
    double sin_a;
    if (cos_a > 1.0) {
      cos_a = 1.0;
      sin_a = 0.0;
    } else {
      sin_a = sqrt(1.0 - cos_a * cos_a);
    }
    if ((b[0] - cos_a * a[0]) * a[2] > 0) {
      sin_a *= -1;
    }
    d_alfa = atan2(sin_a, cos_a);
    // ax = cos_a*a[0]-sin_a*a[2];
    az = sin_a * a[0] + cos_a * a[2];

    sqr_az = a[1] * a[1] + az * az;
    disk = sqr_az - b[1] * b[1];
    if (disk >= 0.0) {
      double cos_b = (a[1] * b[1] + az * sqrt(disk)) / sqr_az;
      double sin_b;
      if (cos_b > 1.0) {
        cos_b = 1.0;
        sin_b = 0.0;
      } else {
        sin_b = sqrt(1.0 - cos_b * cos_b);
      }
      if ((b[1] - cos_b * a[1]) * az > 0) {
        sin_b *= -1;
      }
      d_beta = atan2(sin_b, cos_b);
      // ay = cos_b*a[1]-sin_b*az;
      // az = sin_b*a[1]+cos_b*az;
      ret = 1;
    }
  } else {
    sqr_az = a[1] * a[1] + a[2] * a[2];
    disk = sqr_az - b[1] * b[1];
    if (disk >= 0.0) {
      double cos_b = (a[1] * b[1] + a[2] * sqrt(disk)) / sqr_az;
      double sin_b;
      if (cos_b > 1.0) {
        cos_b = 1.0;
        sin_b = 0.0;
      } else {
        sin_b = sqrt(1.0 - cos_b * cos_b);
      }
      if ((b[1] - cos_b * a[1]) * a[2] > 0) {
        sin_b *= -1;
      }
      d_beta = atan2(sin_b, cos_b);
      // ay = cos_b*a[1]-sin_b*a[2];
      az = sin_b * a[1] + cos_b * a[2];

      sqr_az = a[0] * a[0] + az * az;
      disk = sqr_az - b[0] * b[0];
      if (disk >= 0.0) {
        double cos_a = (a[0] * b[0] + az * sqrt(disk)) / sqr_az;
        double sin_a;
        if (cos_a > 1.0) {
          cos_a = 1.0;
          sin_a = 0.0;
        } else {
          sin_a = sqrt(1.0 - cos_a * cos_a);
        }
        if ((b[0] - cos_a * a[0]) * az > 0) {
          sin_a *= -1;
        }
        d_alfa = atan2(sin_a, cos_a);
        // ax = cos_a*a[0]-sin_a*az;
        // az = sin_a*a[0]+cos_a*az;
        ret = 2;
      }
    }
  }
  if (az != az) {
    ROS_ERROR("[Brick detection] Diff vec has nan");
  }
  // cout << "Test "<<ret<<" o:"<<ax<<","<<ay<<","<<az<<" first:"<<a[0]<<","<<a[1]<<","<<a[2]<<" sec:"<<b[0]<<","<<b[1]<<","<<b[2]<<endl;
  return ret;
}

#define _MAX_LAYER 5

static Mat pl_d, ind_d, big_d, big_x, big_y;
static Mat layers[_MAX_LAYER], layer_tmp, layer_seg, layer_wall;
static unsigned short int *depth_ptr;
static unsigned short int *mini_ptr;
static vector<vector<Point>> contours;
static vector<Vec4i> hierarchy;
static vector<Point> conv_convex;
static vector<Point> conv_approx;

static Eigen::Vector3f normal_x, normal_y, normal_z;

[[maybe_unused]] static int limit[1024];

struct Line_cnt
{
  Point2f s, e;
  float len;
  float a, b, c;
  bool sb, eb;
};


static void move_point_to_line(Point2f &p, struct Line_cnt l) {
  float koef = l.c - p.x * l.a - p.y * l.b;
  p.x += koef * l.a;
  p.y += koef * l.b;
}

static float inline back_rotation(float x, float y, float z, int i) {
  return normal_x(i) * x + normal_y(i) * y + normal_z(i) * z;
}

static void test_inside_box(RotatedRect &box) {
  float b_width = box.size.width;
  float b_height = box.size.height;
  float b_yaw = (box.angle * M_PI / 180.0) + M_PI / 2.0;

  if (b_width > b_height) {
    float t = b_width;
    b_width = b_height;
    b_height = t;
    b_yaw = (box.angle * M_PI / 180.0);
  }

  ROS_DEBUG_STREAM("Brick? width:" << b_width << " len:" << b_height);
  if (b_width > 0.5 * _BRICK_WIDTH && b_width < 2.0 * _BRICK_WIDTH) {
    brick_arr[glob_brick_num].c_x = back_rotation(box.center.x, box.center.y, object_z, 0);
    brick_arr[glob_brick_num].c_y = back_rotation(box.center.x, box.center.y, object_z, 1);
    brick_arr[glob_brick_num].c_z = back_rotation(box.center.x, box.center.y, object_z, 2);
    brick_arr[glob_brick_num].yaw = b_yaw;
    if (b_height < 0.48) {
      brick_arr[glob_brick_num].type = RED_TYPE;
    } else if (b_height < 0.95) {
      brick_arr[glob_brick_num].type = GREEN_TYPE;
    } else if (b_height < 1.6) {
      brick_arr[glob_brick_num].type = BLUE_TYPE;
    } else {
      brick_arr[glob_brick_num].type = ORANGE_TYPE;
    }
    brick_arr[glob_brick_num].size = brick_len[brick_arr[glob_brick_num].type-1];
    ROS_DEBUG_STREAM("DEPTH brick:" << brick_arr[glob_brick_num].c_x << ", " << brick_arr[glob_brick_num].c_y << ", " << brick_arr[glob_brick_num].c_z << " yaw:"
                                    << brick_arr[glob_brick_num].yaw << " width:" << b_width << " height:" << b_height << " type:" << (int)brick_arr[glob_brick_num].type);
    glob_brick_num++;
  }
}

static int test_wall_box(RotatedRect *box, float x_max) {
  float b_width = box->size.width;
  float b_height = box->size.height;
  float b_yaw = (box->angle * M_PI / 180.0) + M_PI / 2.0;
  int ret = 0;

  if (b_width > b_height) {
    float t = b_width;
    b_width = b_height;
    b_height = t;
    b_yaw = (box->angle * M_PI / 180.0);
  }

  if (b_width < 1.8 * _WALL_WIDTH && b_height>_MIN_WALL_LEN) {
    if (find_brick>0 && find_brick<=3) {
      float cos_y = cos(b_yaw);
      if (cos(b_yaw) > 0.0) {
        if (b_yaw > 0.0) {
          b_yaw -= M_PI;
        } else {
          b_yaw += M_PI;
        }
        cos_y = cos(b_yaw);
      }
      float l_x = (box->center.x+cos_y*(b_height / 2.0)-x_max)/cos_y;
      if (l_x<0.0) {
        l_x = 0.0;
      }
      float sh_move = ((b_height / 2.0 - brick_len[find_brick - 1] / 2.0) - _BRICK_GAP)-l_x;
      box->center.x += sh_move * cos(b_yaw);
      box->center.y += sh_move * sin(b_yaw);
      //cout << "TEST WALL BOX shift:"<<sh_move<<" l_x:"<<l_x<<endl;
    }
    brick_arr[glob_brick_num].c_x = back_rotation(box->center.x, box->center.y, object_z, 0);
    brick_arr[glob_brick_num].c_y = back_rotation(box->center.x, box->center.y, object_z, 1);
    brick_arr[glob_brick_num].c_z = back_rotation(box->center.x, box->center.y, object_z, 2);
    brick_arr[glob_brick_num].yaw = b_yaw;
    brick_arr[glob_brick_num].type = WALL_TYPE;
    brick_arr[glob_brick_num].size = b_height;
    ROS_DEBUG_STREAM("DEPTH wall:" << brick_arr[glob_brick_num].c_x << ", " << brick_arr[glob_brick_num].c_y << ", " << brick_arr[glob_brick_num].c_z << " yaw:"
                                   << brick_arr[glob_brick_num].yaw << " width:" << b_width << " height:" << b_height << " type:" << (int)brick_arr[glob_brick_num].type);
    glob_brick_num++;
    ret = 1;
  }
  return ret;
}


#define _MAX_NEW_CONV 10
static vector<Point2f> new_conv[_MAX_NEW_CONV];
static vector<Point2f> approx_conv[_MAX_NEW_CONV];
static vector<Point2f> tmp_cont;
static int left_border = 1;
static struct Line_cnt lines[100];
static float line_ang[100];
static int line_used[100];
static float local_brick_x, local_brick_y, local_brick_yaw;
static enum Brick_type local_brick_type;

static void test_U_shape(int max_i, int max_dist_i, int line_ptr, float avg_x, float avg_y) {
  double yaw = yaw_avg(line_ang[max_i], line_ang[max_dist_i]);

  Line_cnt mid_line;
  mid_line.a = -sin(yaw);
  mid_line.b = cos(yaw);
  float c1 = mid_line.a * lines[max_i].s.x + mid_line.b * lines[max_i].s.y + mid_line.a * lines[max_i].e.x + mid_line.b * lines[max_i].e.y;
  float c2 = mid_line.a * lines[max_dist_i].s.x + mid_line.b * lines[max_dist_i].s.y + mid_line.a * lines[max_dist_i].e.x + mid_line.b * lines[max_dist_i].e.y;
  mid_line.c = (c1 + c2) / 4.0;

  bool mid_init = false;
  Point2f mid_center1;
  float mid_avg_d = 0.0;

  for (int m_i = 0; m_i < line_ptr; m_i++) {
    if (m_i != max_dist_i && m_i != max_i) {
      if (mid_init) {
        Point2f mid_tmp;
        mid_tmp.x = (lines[m_i].s.x + lines[m_i].e.x) / 2.0;
        mid_tmp.y = (lines[m_i].s.y + lines[m_i].e.y) / 2.0;
        move_point_to_line(mid_tmp, mid_line);
        float tmp_avg_d = my_sqr(mid_tmp.x - avg_x) + my_sqr(mid_tmp.y - avg_y);
        if (tmp_avg_d > mid_avg_d) {
          mid_center1 = mid_tmp;
          mid_avg_d = tmp_avg_d;
        }
      } else {
        mid_center1.x = (lines[m_i].s.x + lines[m_i].e.x) / 2.0;
        mid_center1.y = (lines[m_i].s.y + lines[m_i].e.y) / 2.0;
        move_point_to_line(mid_center1, mid_line);
        mid_avg_d = my_sqr(mid_center1.x - avg_x) + my_sqr(mid_center1.y - avg_y);
        mid_init = true;
      }
    }
  }
  if (mid_init) {
    float dist = (brick_len[local_brick_type - 1] / 2.0);
    Point2f center1, center2;
    center1.x = mid_center1.x + dist * mid_line.b;
    center1.y = mid_center1.y - dist * mid_line.a;
    center2.x = mid_center1.x - dist * mid_line.b;
    center2.y = mid_center1.y + dist * mid_line.a;

    float a_dist1 = my_sqr(center1.x - avg_x) + my_sqr(center1.y - avg_y);
    float a_dist2 = my_sqr(center2.x - avg_x) + my_sqr(center2.y - avg_y);
    if (a_dist2 < a_dist1) {
      center1 = center2;
    }
    float l_dist1 = my_sqr(center1.x - local_brick_x) + my_sqr(center1.y - local_brick_y);
    if (l_dist1 < 0.09) {
      ROS_DEBUG_STREAM("1. U detection dist:" << l_dist1);
      brick_arr[glob_brick_num].c_x = back_rotation(center1.x, center1.y, object_z, 0);
      brick_arr[glob_brick_num].c_y = back_rotation(center1.x, center1.y, object_z, 1);
      brick_arr[glob_brick_num].c_z = back_rotation(center1.x, center1.y, object_z, 2);
      brick_arr[glob_brick_num].yaw = yaw;
      brick_arr[glob_brick_num].type = local_brick_type;
      brick_arr[glob_brick_num].size = brick_len[brick_arr[glob_brick_num].type-1];
      glob_brick_num++;
    } else {
      center2.x = local_brick_x;
      center2.y = local_brick_y;
      move_point_to_line(center2, mid_line);
      float l_dist2 = my_sqr(center2.x - center1.x) + my_sqr(center2.y - center1.y);
      if (l_dist2 < 0.16) {
        ROS_DEBUG_STREAM("2. U detection dist:" << l_dist2 << " old dist:" << l_dist1);
        brick_arr[glob_brick_num].c_x = back_rotation(center2.x, center2.y, object_z, 0);
        brick_arr[glob_brick_num].c_y = back_rotation(center2.x, center2.y, object_z, 1);
        brick_arr[glob_brick_num].c_z = back_rotation(center2.x, center2.y, object_z, 2);
        brick_arr[glob_brick_num].yaw = yaw;
        brick_arr[glob_brick_num].type = local_brick_type;
        brick_arr[glob_brick_num].size = brick_len[brick_arr[glob_brick_num].type-1];
        glob_brick_num++;
      } else {
        ROS_DEBUG_STREAM("NO U detection dist:" << l_dist2 << " old dist:" << l_dist1);
      }
    }
  }
}

static void test_L_I_shape(int max_i, int line_ptr, int cnt_ptr) {
  float exp_len = brick_len[local_brick_type - 1];
  Point2f center;
  bool center_fill = false;
  float l_dist, yaw;
  if (lines[max_i].len > exp_len - 0.06 && lines[max_i].len < exp_len + 0.06) {
    Point2f c1, c2;
    c1.x = (lines[max_i].s.x + lines[max_i].e.x) / 2.0;
    c1.y = (lines[max_i].s.y + lines[max_i].e.y) / 2.0;
    center.x = c1.x + lines[max_i].a * (_BRICK_WIDTH / 2.0);
    center.y = c1.y + lines[max_i].b * (_BRICK_WIDTH / 2.0);
    if (local_brick_type == RED_TYPE) {
      c2.x = c1.x + lines[max_i].a * (exp_len / 2.0);
      c2.y = c1.y + lines[max_i].b * (exp_len / 2.0);
    }
    if ((lines[max_i].c < 0) == ((lines[max_i].a * center.x + lines[max_i].b * center.y - lines[max_i].c) > 0)) {
      center.x = c1.x - lines[max_i].a * (_BRICK_WIDTH / 2.0);
      center.y = c1.y - lines[max_i].b * (_BRICK_WIDTH / 2.0);
      if (local_brick_type == RED_TYPE) {
        c2.x = c1.x - lines[max_i].a * (exp_len / 2.0);
        c2.y = c1.y - lines[max_i].b * (exp_len / 2.0);
      }
    }
    l_dist = my_sqr(center.x - local_brick_x) + my_sqr(center.y - local_brick_y);
    yaw = line_ang[max_i];
    if (local_brick_type == RED_TYPE) {
      float l_dist2 = my_sqr(c2.x - local_brick_x) + my_sqr(c2.y - local_brick_y);
      float y_d1 = yaw_dif(local_brick_yaw, yaw);
      float y_d2 = yaw_dif(local_brick_yaw, yaw + M_PI_2);
      y_d1 = my_abs(y_d1);
      y_d2 = my_abs(y_d2);
      //ROS_DEBUG_STREAM("LONG line:" << l_dist << "," << y_d1 << " SHORT line:" << l_dist2 << "," << y_d2);
      if (y_d2 < y_d1) {
        l_dist = l_dist2;
        center = c2;
        yaw += M_PI_2;
      }
    }
    center_fill = true;
    /*  } else if (lines[max_i].len>_BRICK_WIDTH-0.05 && lines[max_i].len<_BRICK_WIDTH+0.05) {
        Point2f c1, c2;
        c1.x = (lines[max_i].s.x+lines[max_i].e.x)/2.0;
        c1.y = (lines[max_i].s.y+lines[max_i].e.y)/2.0;
        center.x = c1.x-lines[max_i].b*(exp_len/2.0);
        center.y = c1.y+lines[max_i].a*(exp_len/2.0);
        if ((lines[max_i].c<0)==((lines[max_i].a*center.x+lines[max_i].b*center.y-lines[max_i].c)>0)) {
          center.x = c1.x+lines[max_i].b*(exp_len/2.0);
          center.y = c1.y-lines[max_i].a*(exp_len/2.0);
        }
        yaw = line_ang[max_i]+M_PI_2;
        l_dist = my_sqr(center.x-local_brick_x)+my_sqr(center.y-local_brick_y);
         ROS_DEBUG_STREAM("ONLY SHORT line:"<<l_dist);
        center_fill = true; */
  } else {
    Point2f corner, op;
    bool set_corner = false;
    if (line_ptr == 1 and cnt_ptr == 1) {
      int dp_size = new_conv[0].size();
      if (dp_size > 10) {
        float dp1 = my_sqr(new_conv[0][0].x - new_conv[0][4].x) + my_sqr(new_conv[0][0].y - new_conv[0][4].y);
        float dp2 = my_sqr(new_conv[0][dp_size - 1].x - new_conv[0][dp_size - 5].x) + my_sqr(new_conv[0][dp_size - 1].y - new_conv[0][dp_size - 5].y);
        ROS_DEBUG_STREAM("CORNER one line ends:" << dp1 << ", " << dp2);
        if (dp1 < 0.0009 && dp2 > 0.0009) {
          corner = lines[max_i].s;
          op = lines[max_i].e;
          set_corner = true;
        } else if (dp1 > 0.0009 && dp2 < 0.0009) {
          corner = lines[max_i].e;
          op = lines[max_i].s;
          set_corner = true;
        }
      }
    } else {
      if (lines[max_i].sb) {
        corner = lines[max_i].e;
        op = lines[max_i].s;
        set_corner = true;
        ROS_DEBUG_STREAM("MORE LINES, but start is border, corner:" << corner.x << "," << corner.y << " op:" << op.x << "," << op.y);
      } else if (lines[max_i].eb) {
        corner = lines[max_i].s;
        op = lines[max_i].e;
        set_corner = true;
        ROS_DEBUG_STREAM("MORE LINES, but end is border, corner:" << corner.x << "," << corner.y << " op:" << op.x << "," << op.y);
      }
    }
    if (set_corner) {
      Point2f a;
      a.x = op.x - corner.x;
      a.y = op.y - corner.y;
      float a_size = sqrt(my_sqr(a.x) + my_sqr(a.y));
      a.x /= a_size;
      a.y /= a_size;
      float c = a.y * corner.x - a.x * corner.y;
      Point2f vec;
      vec.x = a.y;
      vec.y = -a.x;
      Point2f c2;
      c2.x = corner.x + vec.x;
      c2.y = corner.y + vec.y;
      if ((c < 0) == ((a.y * c2.x - a.x * c2.y - c) > 0)) {
        vec.x = -vec.x;
        vec.y = -vec.y;
      }
      center.x = corner.x + a.x * (exp_len / 2.0) + vec.x * (_BRICK_WIDTH / 2.0);
      center.y = corner.y + a.y * (exp_len / 2.0) + vec.y * (_BRICK_WIDTH / 2.0);
      l_dist = my_sqr(center.x - local_brick_x) + my_sqr(center.y - local_brick_y);
      yaw = line_ang[max_i];
      c2.x = corner.x + a.x * (_BRICK_WIDTH / 2.0) + vec.x * (exp_len / 2.0);
      c2.y = corner.y + a.y * (_BRICK_WIDTH / 2.0) + vec.y * (exp_len / 2.0);
      float l_dist2 = my_sqr(c2.x - local_brick_x) + my_sqr(c2.y - local_brick_y);
      float y_d1 = yaw_dif(local_brick_yaw, yaw);
      float y_d2 = yaw_dif(local_brick_yaw, yaw + M_PI_2);
      y_d1 = my_abs(y_d1);
      y_d2 = my_abs(y_d2);
      ROS_DEBUG_STREAM("CORNER LONG line yaw" << y_d1 << " SHORT line yaw" << y_d2);
      if (y_d2 < y_d1) {
        l_dist = l_dist2;
        center = c2;
        yaw += M_PI_2;
      }
      center_fill = true;
    } else {
      Line_cnt mid_line, mid_line2;
      float d = lines[max_i].a * local_brick_x + lines[max_i].b * local_brick_y - lines[max_i].c;
      d = my_abs(d);
      float e1 = d - (exp_len / 2.0);
      e1 = my_abs(e1);
      float e2 = d - (_BRICK_WIDTH / 2.0);
      e2 = my_abs(e2);
      if (e1 < e2) {
        d = (exp_len / 2.0);
        yaw = line_ang[max_i] + M_PI_2;
      } else {
        d = (_BRICK_WIDTH / 2.0);
        yaw = line_ang[max_i];
      }
      mid_line.a = lines[max_i].a;
      mid_line.b = lines[max_i].b;
      if (lines[max_i].c > 0) {
        mid_line.c = lines[max_i].c + d;
      } else {
        mid_line.c = lines[max_i].c - d;
      }
      center.x = local_brick_x;
      center.y = local_brick_y;
      move_point_to_line(center, mid_line);
      l_dist = my_sqr(center.x - local_brick_x) + my_sqr(center.y - local_brick_y);
      ROS_DEBUG_STREAM("Move to line dist:" << d);
      center_fill = true;
    }
  }

  if (center_fill && l_dist < 0.09) {
    ROS_DEBUG_STREAM("L I detection dist real:" << sqrt(l_dist));
    brick_arr[glob_brick_num].c_x = back_rotation(center.x, center.y, object_z, 0);
    brick_arr[glob_brick_num].c_y = back_rotation(center.x, center.y, object_z, 1);
    brick_arr[glob_brick_num].c_z = back_rotation(center.x, center.y, object_z, 2);
    brick_arr[glob_brick_num].yaw = yaw;
    brick_arr[glob_brick_num].type = local_brick_type;
    brick_arr[glob_brick_num].size = brick_len[brick_arr[glob_brick_num].type-1];
    glob_brick_num++;
  }
}

static Mat dbg_n;

static void test_L_I_shape_wall(int max_i, float x_max, bool bottom) {
  Eigen::Vector3f pos;
  Point2f center;
  float multip = 1.0;
  //float l_x, l_y;
  if (bottom) {
    multip=-1.0;
  }

  if (my_abs(lines[max_i].b)>0.8) {
    Point2f c1, c2;
    if (x_max>-100.0) {
      c1.x = x_max;
      c1.y = (lines[max_i].c-lines[max_i].a*c1.x)/lines[max_i].b;
    } else {
      c1.x = (lines[max_i].s.x + lines[max_i].e.x) / 2.0;
      c1.y = (lines[max_i].s.y + lines[max_i].e.y) / 2.0;
    }
    if (lines[max_i].b>0.0) {
      center.x = c1.x - multip * lines[max_i].a * (_WALL_WIDTH / 2.0);
      center.y = c1.y - multip * lines[max_i].b * (_WALL_WIDTH / 2.0);
    } else {
      center.x = c1.x + multip * lines[max_i].a * (_WALL_WIDTH / 2.0);
      center.y = c1.y + multip * lines[max_i].b * (_WALL_WIDTH / 2.0);
    }

    /*
    float min_dist = 10000.0;
    int min_i = -1;
    for (int i = 0; i < local_brick_num; i++) {
      if (local_brick[i].type==WALL_TYPE) {
        float y_d = yaw_dif(local_brick[i].yaw, line_ang[max_i]);
        if (y_d > -0.2 && y_d < 0.2) {
          pos << local_brick[i].c_x, local_brick[i].c_y, local_brick[i].c_z;
          l_x = pos.dot(normal_x);
          l_y = pos.dot(normal_y);
          float l_a = -sin(local_brick[i].yaw);
          float l_b = cos(local_brick[i].yaw);
          float l_c = l_a * l_x + l_b * l_y;
          float d = center.x * l_a + center.y * l_b - l_c;
          d = my_abs(d);
          if (d < min_dist) {
            min_i = i;
            min_dist = d;
            local_brick_x = l_x;
            local_brick_y = l_y;
          }
        }
      }
    }
*/
  //  if (min_i >= 0 && min_dist < 0.3) {
      /*ROS_DEBUG_STREAM("L I detection dist real:" << sqrt(min_dist));
      struct Line_cnt l;
      pos << local_brick[min_i].c_x, local_brick[min_i].c_y, local_brick[min_i].c_z;
      l_x = pos.dot(normal_x);
      l_y = pos.dot(normal_y);
      l.a = -sin(local_brick[min_i].yaw);
      l.b = cos(local_brick[min_i].yaw);
      l.c = l.a * l_x + l.b * l_y;
      move_point_to_line(center, l);
*/
      
    if (find_brick>0 && find_brick<=3) {
      float sh_move = (brick_len[find_brick - 1] / 2.0) + _BRICK_GAP;

      if (lines[max_i].b>0.0) {
        if (show_debug) {
          line(dbg_n, Point((((center.x)*r_fx)/object_z+r_cx)/8, (((center.y)*r_fy)/object_z+r_cy)/8), 
           Point((((center.x+sh_move * lines[max_i].b)*r_fx)/object_z+r_cx)/8, (((center.y-sh_move * lines[max_i].a)*r_fy)/object_z+r_cy)/8), Scalar(0, 0, 255),3, LINE_AA);
        }
        center.x += sh_move * lines[max_i].b;
        center.y -= sh_move * lines[max_i].a;
      } else {
        if (show_debug) {
          line(dbg_n, Point((((center.x)*r_fx)/object_z+r_cx)/8, (((center.y)*r_fy)/object_z+r_cy)/8), 
           Point((((center.x-sh_move * lines[max_i].b)*r_fx)/object_z+r_cx)/8, (((center.y+sh_move * lines[max_i].a)*r_fy)/object_z+r_cy)/8), Scalar(0, 0, 255),3, LINE_AA);
        }
        center.x -= sh_move * lines[max_i].b;
        center.y += sh_move * lines[max_i].a;
      }  
      ROS_DEBUG_STREAM("L I shift:"<<sh_move<<" cos:"<<lines[max_i].b<<" sin:"<<lines[max_i].a<<" c_y:"<<center.y<<" c1.y:"<<c1.y);
   }

   if (show_debug) {
     circle(dbg_n, Point(((center.x*r_fx)/object_z+r_cx)/8, ((center.y*r_fy)/object_z+r_cy)/8), 5, Scalar(0, 255, 0), 1);
   }
      
    brick_arr[glob_brick_num].c_x = back_rotation(center.x, center.y, object_z, 0);
    brick_arr[glob_brick_num].c_y = back_rotation(center.x, center.y, object_z, 1);
    brick_arr[glob_brick_num].c_z = back_rotation(center.x, center.y, object_z, 2);
    brick_arr[glob_brick_num].yaw = line_ang[max_i];
    brick_arr[glob_brick_num].size = lines[max_i].len;
    brick_arr[glob_brick_num].type = WALL_TYPE;
    /*if (lines[max_i].len>5.0) {
      cout << "MAX I ERROR -------+++++++++++++++------------ wall len:"<<lines[max_i].len<<endl;
    }*/
    glob_brick_num++;
    //}
  }
}


static void analyze_part_object(int cnt_ptr) {
  int max_line_ind = -1;
  int line_ptr = 0;
  float max_line_len = 0.0;
  float avg_x = 0.0;
  float avg_y = 0.0;
  int num_pts = 0;
  for (int i = 0; i < cnt_ptr; i++) {
    approxPolyDP(new_conv[i], approx_conv[i], 0.05, false);
    ROS_DEBUG_STREAM("Approx:" << i << " num of lines:" << (approx_conv[i].size() - 1));
    for (int j = 0; j < ((int)approx_conv[i].size()) - 1; j++) {
      avg_x += approx_conv[i][j].x;
      avg_y += approx_conv[i][j].y;
      num_pts++;
      double d = atan2(approx_conv[i][j + 1].y - approx_conv[i][j].y, approx_conv[i][j + 1].x - approx_conv[i][j].x);
      line_ang[line_ptr] = d;
      int ind = (d + M_PI) * 10.0 / M_PI;
      if (ind < 0) {
        ind += 20;
      }
      if (ind >= 20) {
        ind -= 20;
      }
      lines[line_ptr].len = sqrt(my_sqr(approx_conv[i][j + 1].y - approx_conv[i][j].y) + my_sqr(approx_conv[i][j + 1].x - approx_conv[i][j].x));
      lines[line_ptr].a = (approx_conv[i][j + 1].y - approx_conv[i][j].y) / lines[line_ptr].len;
      lines[line_ptr].b = (approx_conv[i][j].x - approx_conv[i][j + 1].x) / lines[line_ptr].len;
      lines[line_ptr].c = lines[line_ptr].a * approx_conv[i][j].x + lines[line_ptr].b * approx_conv[i][j].y;
      lines[line_ptr].s = approx_conv[i][j];
      lines[line_ptr].e = approx_conv[i][j + 1];
      lines[line_ptr].sb = (j == 0);
      lines[line_ptr].eb = (j + 2 == (int)approx_conv[i].size());
      //ROS_DEBUG_STREAM("Line len" << lines[line_ptr].len << " angl:" << d);
      if (lines[line_ptr].len > max_line_len) {
        max_line_len = lines[line_ptr].len;
        max_line_ind = line_ptr;
      }
      if (line_ptr < 95) {
        line_ptr++;
      }
    }
  }
  int max_dist_i = -1;
  float max_dist = 0.0;
  if (max_line_ind >= 0) {
    for (int i = 0; i < line_ptr; i++) {
      if (i != max_line_ind) {
        float y_d = yaw_dif(line_ang[i], line_ang[max_line_ind]);
        if (my_abs(y_d) < 0.3) {
          float dist1 = lines[i].s.x * lines[max_line_ind].a + lines[i].s.y * lines[max_line_ind].b - lines[max_line_ind].c;
          float dist2 = lines[i].e.x * lines[max_line_ind].a + lines[i].e.y * lines[max_line_ind].b - lines[max_line_ind].c;
          dist1 = my_abs(dist1);
          dist2 = my_abs(dist2);
          if (dist1 > dist2) {
            dist1 = dist2;
          }
          if (dist1 > max_dist) {
            max_dist_i = i;
            max_dist = dist1;
          }
        }
      }
    }
    
  }

  float l_x, l_y;
  Eigen::Vector3f pos;

  avg_x /= (float)num_pts;
  avg_y /= (float)num_pts;
  float min_dist = 100000.0;
  int min_i = -1;
  for (int i = 0; i < local_brick_num; i++) {
    if (local_brick[i].type < WALL_TYPE) {
      pos << local_brick[i].c_x, local_brick[i].c_y, local_brick[i].c_z;
      l_x = pos.dot(normal_x);
      l_y = pos.dot(normal_y);
      float d = sqrt(my_sqr(avg_x - l_x) + my_sqr(avg_y - l_y));
      if (d < min_dist) {
        min_i = i;
        min_dist = d;
        local_brick_x = l_x;
        local_brick_y = l_y;
      }
    }
  }
  if (min_i >= 0) {
    local_brick_yaw = local_brick[min_i].yaw;
    local_brick_type = local_brick[min_i].type;

    if (max_dist_i >= 0) {
      test_U_shape(max_line_ind, max_dist_i, line_ptr, avg_x, avg_y);
    } else if (cnt_ptr == 1) {
      test_L_I_shape(max_line_ind, line_ptr, cnt_ptr);
    }
  }
}

static void analyze_part_wall(int cnt_ptr, int big_cnt, float pix_size, float x_max, bool bottom) {
  int max_line_ind = -1;
  int line_ptr = 0;
  float max_line_len = 0.0;
  int find_walls = 0;
// 202, 258 - length 1.2 
  
  for (int i = 0; i < cnt_ptr; i++) {
    approx_conv[i].clear();
    approxPolyDP(new_conv[i], approx_conv[i], 0.2, false);
    ROS_DEBUG_STREAM("Approx WALL: num of lines:" << (approx_conv[i].size() - 1));
    if (show_debug) {
      for (int ii=0; ii<(int)approx_conv[i].size() - 1; ii++) {
        line(dbg_n, Point(((approx_conv[i][ii].x*r_fx)/object_z+r_cx)/8, ((approx_conv[i][ii].y*r_fy)/object_z+r_cy)/8), 
             Point(((approx_conv[i][ii+1].x*r_fx)/object_z+r_cx)/8, ((approx_conv[i][ii+1].y*r_fy)/object_z+r_cy)/8), Scalar(255, 0, 0), 2, LINE_AA);
      }
    }
    int approx_size = approx_conv[i].size();
    for (int j = 0; j < approx_size - 1; j++) {
      double d = atan2(approx_conv[i][j + 1].y - approx_conv[i][j].y, approx_conv[i][j + 1].x - approx_conv[i][j].x);
      line_ang[line_ptr] = d;
      lines[line_ptr].len = sqrt(my_sqr(approx_conv[i][(j + 1)].y - approx_conv[i][j].y) + my_sqr(approx_conv[i][(j + 1)].x - approx_conv[i][j].x));
      lines[line_ptr].a = (approx_conv[i][(j + 1)].y - approx_conv[i][j].y) / lines[line_ptr].len;
      lines[line_ptr].b = (approx_conv[i][j].x - approx_conv[i][(j + 1)].x) / lines[line_ptr].len;
      lines[line_ptr].c = lines[line_ptr].a * approx_conv[i][j].x + lines[line_ptr].b * approx_conv[i][j].y;
      lines[line_ptr].s = approx_conv[i][j];
      lines[line_ptr].e = approx_conv[i][(j + 1)];

      ROS_DEBUG_STREAM("Line "<<line_ptr<<" len " << lines[line_ptr].len << " angl:" << d);
      if (lines[line_ptr].len > max_line_len) {
        max_line_len = lines[line_ptr].len;
        max_line_ind = line_ptr;
      }
      if (line_ptr < 95) {
        line_ptr++;
      }
    }
  }

  for (int j = 0; j < line_ptr; j++) {
    line_used[j] = ((lines[j].len > 0.35) ? 0 : 1);
  }

  for (int j = 0; j < line_ptr - 1; j++) {
    if (line_used[j] == 0) {
      int best_dist_i = -1;
      float best_dist = 10000.0;
      for (int i = j + 1; i < line_ptr; i++) {
        if (line_used[i] == 0) {
          float y_d = yaw_dif(line_ang[i], line_ang[j]);
          if (my_abs(y_d) < 0.20) {  // approx 15dg
            float dist1 = lines[i].s.x * lines[j].a + lines[i].s.y * lines[j].b - lines[j].c;
            float dist2 = lines[i].e.x * lines[j].a + lines[i].e.y * lines[j].b - lines[j].c;
            dist1 = my_abs(dist1);
            dist2 = my_abs(dist2);
            //cout << "Dist1:"<<dist1<<" dist2:"<<dist2<<endl;
            //if (dist1 > dist2) {
              dist1 = (dist1+dist2)/2.0;
            //}
            if (dist1 >= 0.5 * _WALL_WIDTH && dist1 <= 1.8 * _WALL_WIDTH) {
              float d = dist1 - _WALL_WIDTH;
              d = my_abs(d);
              if (d < best_dist) {
                best_dist_i = i;
                best_dist = d;
              }
            }
          }
        }
      }
      if (best_dist_i >= 0) {
        tmp_cont.clear();
        tmp_cont.push_back(lines[j].s);
        tmp_cont.push_back(lines[j].e);
        tmp_cont.push_back(lines[best_dist_i].s);
        tmp_cont.push_back(lines[best_dist_i].e);
        RotatedRect tmp_box = minAreaRect(tmp_cont);
        float b_width = tmp_box.size.width;
        float b_height = tmp_box.size.height;
        float b_yaw = (tmp_box.angle * M_PI / 180.0) + M_PI / 2.0;

        if (b_width > b_height) {
          float t = b_width;
          b_width = b_height;
          b_height = t;
          b_yaw = (tmp_box.angle * M_PI / 180.0);
        }
        float line_y = yaw_avg(line_ang[j], line_ang[best_dist_i]);
        float y_d = yaw_dif(b_yaw, line_y);
        y_d = my_abs(y_d);
        ROS_DEBUG_STREAM("WALL Partial box width:" << b_width << ", h:" << b_height << " sum lens:"<<0.9*(lines[j].len+lines[best_dist_i].len)<<" yaw diff:" << y_d << " j:" << j << " best:" << best_dist_i << " dist:" << best_dist);
        if (b_width < 1.8 * _WALL_WIDTH && y_d < 0.12 && /*b_width > 0.5 * _WALL_WIDTH &&*/ b_height>_MIN_WALL_LEN && b_height<0.9*(lines[j].len+lines[best_dist_i].len)) {
          line_used[j] = 1;
          line_used[best_dist_i] = 1;

          if (find_brick>0 && find_brick<=3) {
            float cos_y = cos(b_yaw);
            if (cos_y > 0.0) {
              if (b_yaw > 0.0) {
                b_yaw -= M_PI;
              } else {
                b_yaw += M_PI;
              }
              cos_y = cos(b_yaw);
            }
            float l_x = (tmp_box.center.x+cos_y*(b_height / 2.0)-x_max)/cos_y;
            if (l_x<0.0) {
              l_x = 0.0;
            }
            float sh_move = ((b_height / 2.0 - brick_len[find_brick - 1] / 2.0) - _BRICK_GAP) - l_x;

            if (show_debug) {
              line(dbg_n, Point((((tmp_box.center.x)*r_fx)/object_z+r_cx)/8, (((tmp_box.center.y)*r_fy)/object_z+r_cy)/8), 
                Point((((tmp_box.center.x+sh_move * cos_y)*r_fx)/object_z+r_cx)/8, (((tmp_box.center.y+sh_move * sin(b_yaw))*r_fy)/object_z+r_cy)/8), Scalar(0, 0, 255),3, LINE_AA);
            }
    
            tmp_box.center.x += sh_move * cos_y;
            tmp_box.center.y += sh_move * sin(b_yaw);
  
            float sh_up = 0.0;
            if (b_width<_WALL_WIDTH) {
              sh_up = (_WALL_WIDTH - b_width)/2.0;
              if (bottom) {
                tmp_box.center.x += sh_up * sin(b_yaw);
                tmp_box.center.y -= sh_up * cos_y;
              } else {
                tmp_box.center.x -= sh_up * sin(b_yaw);
                tmp_box.center.y += sh_up * cos_y;
              }
            }
              
            //cout << "TEST WALL BOX shift:"<<sh_move<<" l_x:"<<l_x<<" up:"<<sh_up<<" cos:"<<cos_y<<" b_yaw:"<<b_yaw<<endl;
          }

          if (show_debug) {
            circle(dbg_n, Point(((tmp_box.center.x*r_fx)/object_z+r_cx)/8, ((tmp_box.center.y*r_fy)/object_z+r_cy)/8), 5, Scalar(0, 255, 0), 1);
          }
    
          brick_arr[glob_brick_num].c_x = back_rotation(tmp_box.center.x, tmp_box.center.y, object_z, 0);
          brick_arr[glob_brick_num].c_y = back_rotation(tmp_box.center.x, tmp_box.center.y, object_z, 1);
          brick_arr[glob_brick_num].c_z = back_rotation(tmp_box.center.x, tmp_box.center.y, object_z, 2);
          brick_arr[glob_brick_num].yaw = b_yaw;
          brick_arr[glob_brick_num].size = b_height;
          brick_arr[glob_brick_num].type = WALL_TYPE;
          ROS_DEBUG_STREAM("DEPTH wall:" << brick_arr[glob_brick_num].c_x << ", " << brick_arr[glob_brick_num].c_y << ", " << brick_arr[glob_brick_num].c_z << " yaw:"
                                    << brick_arr[glob_brick_num].yaw << " width:" << b_width << " height:" << b_height << " type:" << (int)brick_arr[glob_brick_num].type);
          /*if (b_height>5.0) {
            cout << "ERROR -------+++++++++++++++------------ wall len:"<<b_height<<endl;
          }*/
          glob_brick_num++;
          find_walls++;
        }
      }
    }
  }

  if (cnt_ptr >= 1 && find_walls == 0 && big_cnt >= 1 && (find_brick>0 && find_brick<=3) && (lines[max_line_ind].len > 1.0)) {
    test_L_I_shape_wall(max_line_ind, x_max, bottom);
  }
}


static inline int get_border_type(Point p) {
  if (p.y >= mini.rows - 3) {
    return 3;
  } else if (p.x <= left_border) {
    return 0;
  } else if (p.x >= mini.cols - 1) {
    return 1;
  } else if (p.y == 0) {
    return 2;
  }
  return -1;
}

void init_big_mat(double altitude) {
  int v_mini = 0;
  Eigen::Vector3f pos;
  if ((find_brick > 0) && (find_brick<=3)) {
    altitude+=wall_offset;
  }
  for (int v = 0; v < mini.rows; v++) {
    for (int u = 0; u < mini.cols; u++) {
      float val = ((float)(mini_ptr[v_mini + u]) / _SCALE);  // the same scale as original depth image
      pos << (8 * u + 4 - r_cx) * val / r_fx, ((8 * v + 4) - r_cy) * val / r_fy, val;

      float d = pos.dot(normal_z);
      big_x.at<float>(v, u) = pos.dot(normal_x);
      big_y.at<float>(v, u) = pos.dot(normal_y);

      if (rs_mini_mask.at<unsigned char>(v, u) < 50) {
        big_d.at<float>(v, u) = 0.0;
      } else if (val < 0.10 || val > 20.0) {
        big_d.at<float>(v, u) = 20.0;
      } else {
        float val2 = altitude - d;
        big_d.at<float>(v, u) = val2;
      }
    }
    v_mini += mini.cols;
  }
}

void track_object(double altitude, float object_limit, float object_height) {
  layers[0].create(mini.size(), CV_8U);
  layers[0].setTo(0);
  is_near = true;
  object_z = altitude - object_height;

  left_border = 0;
  int v_mini = 0;
  for (int v = 0; v < mini.rows; v++) {
    for (int u = 0; u < mini.cols; u++) {
      float val2 = big_d.at<float>(v, u);
      if (val2 > object_limit && val2 < object_height + 2 * _BRICK_LIMIT) {
        layers[0].at<unsigned char>(v, u) = 250;
      } else if (val2 == 20.0) {
        layers[0].at<unsigned char>(v, u) = 128;
      }
    }
    v_mini += mini.cols;
  }

  for (left_border = 1; left_border < 15; left_border++) {
    int cor = 0;
    for (int v = mini.rows / 2; v < mini.rows; v++) {
      if (layers[0].at<unsigned char>(v, left_border) != 128) {
        cor++;
      }
    }
    if (cor > 6) {
      for (int v = 0; v < mini.rows; v++) {
        if (layers[0].at<unsigned char>(v, left_border) == 128 && layers[0].at<unsigned char>(v, left_border + 1) == 0) {
          layers[0].at<unsigned char>(v, left_border) = 0;
        }
      }
      break;
    } else {
      for (int v = 0; v < mini.rows; v++) {
        layers[0].at<unsigned char>(v, left_border) = 0;
      }
    }
  }
  //imshow("brick",layers[0]);
  ROS_DEBUG_STREAM("LEFT BORDER:" << left_border << "  -------------------------------------------- " << simulation);

  if (simulation) {
    erode(layers[0], layer_tmp, erosion_element);
    dilate(layer_tmp, layers[0], erosion_element);
    // imshow("layer-mm",layers[0]);
  }
  if (show_debug) {
    cvtColor(layers[0], dbg_n, cv::COLOR_GRAY2BGR);
  }
  contours.clear();
  hierarchy.clear();

  findContours(layers[0], contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
  if (show_debug) {
    drawContours(dbg_n, contours, -1, Scalar(255, 0, 0), 1);
  }
  ROS_DEBUG_STREAM("Find " << contours.size() << " contours");

  for (int i = 0; i < (int)contours.size(); i++) {
    float area = contourArea(contours[i]);
    if (area > 60) {
      bool border = false;
      int border_type = -1;
      bool data = false;
      int new_cnt_ptr = 0;
      new_conv[new_cnt_ptr].clear();
      for (auto &p : contours[i]) {
        int new_border_type = get_border_type(p);
        if (new_border_type >= 0) {
          border = true;
          if (data) {
            if ((new_border_type != border_type) || (new_conv[new_cnt_ptr].size() > 10)) {
              new_cnt_ptr++;
              if (new_cnt_ptr >= _MAX_NEW_CONV) {
                new_cnt_ptr--;
              }
            }
            new_conv[new_cnt_ptr].clear();
          }
          border_type = new_border_type;
          data = false;
        } else {
          if (layers[0].at<unsigned char>(p.y, p.x) > 200) {
            new_conv[new_cnt_ptr].push_back(Point2f(big_x.at<float>(p.y, p.x), big_y.at<float>(p.y, p.x)));
            data = true;
            if (show_debug) {
              circle(dbg_n, p, 2, Scalar(0, 255, 120), 1);
            }
          }
        }
      }
      if (border && get_border_type(contours[i][0]) < 0 && new_cnt_ptr > 0 && new_conv[new_cnt_ptr].size() > 0) {
        for (auto &p : new_conv[0]) {
          new_conv[new_cnt_ptr].push_back(p);
        }
        new_conv[0].swap(new_conv[new_cnt_ptr]);
      } else if (new_conv[new_cnt_ptr].size() > 0) {
        new_cnt_ptr++;
      }
      ROS_DEBUG_STREAM("Num of segments:" << new_cnt_ptr << " border:" << border);
      RotatedRect box;
      if (new_cnt_ptr == 1 && !border) {
        box = minAreaRect(new_conv[0]);
        ROS_DEBUG_STREAM("Box size:" << box.size.width << "," << box.size.height << " center:" << box.center.x << "," << box.center.y);
        test_inside_box(box);
      } else {
        if (find_brick>0 && find_brick<=3) {
          analyze_part_object(new_cnt_ptr);
        }
      }
    }
  }
  /*if (show_debug) {
    imshow("d0",dbg_n);
  }*/
}


void track_wall(double altitude, float object_limit, float object_height) {
  float precision = _BRICK_LIMIT;
  layers[0].create(mini.size(), CV_8U);
  layers[0].setTo(0u);
  layers[1].create(mini.size(), CV_8U);
  layers[1].setTo(255u);
  is_near = true;
  object_z = altitude - object_height;
  if ((find_brick > 0) && (find_brick<=3)) {
    object_z = altitude + wall_offset - object_height;
    if (object_z<0.8) {
      precision = 0.6*_BRICK_LIMIT;
    } else if (object_z<1.8) {
      precision = 1.25*_BRICK_LIMIT;
    } else if (object_z<2.5) {
      precision = 2.0*_BRICK_LIMIT;
    } else {
      precision = 2.5*_BRICK_LIMIT;
    }
  } else {
    precision = 4*_BRICK_HIGH;
  }

  ROS_DEBUG_STREAM("Track wall height:" << object_height<<" object z:"<<object_z<<" precision:"<<precision<<" wall offset:"<<wall_offset);
  left_border = 0;
  int v_mini = 0;
  int u_max = -1;
  float x_max = -10000.0;
  for (int v = 0; v < mini.rows; v++) {
    for (int u = 1; u < mini.cols; u++) {
      float val2 = big_d.at<float>(v, u);
      if (val2 > object_limit) {
        if (rs_object_mask.at<unsigned char>(v, u) < 50) {
          //layers[1].at<unsigned char>(v, u) = 255;
        } else if (val2==20.0) {
          layers[0].at<unsigned char>(v, u) = 200;
          //layers[1].at<unsigned char>(v, u) = 255;
        } else if (val2 > object_height + precision && val2 < object_height + precision + _BRICK_HIGH) {
          layers[1].at<unsigned char>(v, u) = 0;
          if (u<64 && u>u_max) {
            u_max = u;
          }
        } else {
          layers[0].at<unsigned char>(v, u) = 255;
          //layers[1].at<unsigned char>(v, u) = 255;
        }
      }
    }
    v_mini += mini.cols;
  }
  


  for (left_border = 1; left_border < 15; left_border++) {
    int cor = 0;
    for (int v = mini.rows / 2; v < mini.rows; v++) {
      if (layers[0].at<unsigned char>(v, left_border) != 128) {
        cor++;
      }
    }
    if (cor > 6) {
      for (int v = 0; v < mini.rows; v++) {
        if (layers[0].at<unsigned char>(v, left_border) == 128 && layers[0].at<unsigned char>(v, left_border + 1) == 0) {
          layers[0].at<unsigned char>(v, left_border) = 0;
        }
      }
      break;
    } else {
      for (int v = 0; v < mini.rows; v++) {
        layers[0].at<unsigned char>(v, left_border) = 0;
      }
    }
  }

  if (simulation) {
    if (find_brick>0 && find_brick<=3) {
      if (object_z>1.2) {
        erosion_element_wall = getStructuringElement(MORPH_RECT, Size(9, 9), Point(4, 4));
      } else {
        erosion_element_wall = getStructuringElement(MORPH_RECT, Size(11, 11), Point(5, 5));
      }
      _MIN_WALL_LEN = 0.55;
    } else {
      erosion_element_wall = getStructuringElement(MORPH_RECT, Size(5, 5), Point(2, 2));
      _MIN_WALL_LEN = 1.0;
    }
  } else {
    if (find_brick>0 && find_brick<=3) {
      /*if (object_z>1.2) {
        erosion_element_wall = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
      } else {
      }*/
      _MIN_WALL_LEN = 0.55;
    } else {
      //erosion_element_wall = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
      _MIN_WALL_LEN = 1.0;
    }
    erosion_element_wall = getStructuringElement(MORPH_RECT, Size(7, 7), Point(3, 3));
  }
  dilate(layers[0], layer_tmp, erosion_element_wall);
  erode(layer_tmp, layer_seg, erosion_element_wall);
  //erode(layers[1], layer_tmp, erosion_element_wall);
  //bitwise_and(layer_tmp, layer_seg, layer_wall);  
  
  /*erode(layer_seg, layer_tmp, erosion_element_wall);
  dilate(layer_tmp, layer_seg, erosion_element_wall);
  */
  
  contours.clear();
  hierarchy.clear();
  if (show_debug) {
    cvtColor(layer_seg, dbg_n, cv::COLOR_GRAY2BGR);
    //imshow("0m",layer_seg);
    //imshow("0",layers[0]);
    //imshow("1",layers[1]);
    //imshow("1m",layer_tmp);
    //imshow("w",layer_wall);
  }
  
  findContours(layer_seg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
  if (show_debug) {
    drawContours(dbg_n, contours, -1, Scalar(255,0,255), 1);
  }
  ROS_DEBUG_STREAM("Find " << contours.size() << " contours");

  int n_big_cnt = 0;
  for (int i = 0; i < (int)contours.size(); i++) {
    float area = contourArea(contours[i]);
    if (area > 20) {
      n_big_cnt++;
    }
  }
  for (int i = 0; i < (int)contours.size(); i++) {
    float area = contourArea(contours[i]);
    bool border_bottom=false;
    if (area > 20) {
      bool border = false;
      int border_type = -1;
      bool data = false;
      int new_cnt_ptr = 0;
      new_conv[new_cnt_ptr].clear();
      for (auto &p : contours[i]) {
        int new_border_type = get_border_type(p);
        if (new_border_type >= 0) {
          border = true;
          if (new_border_type==3 && (p.x>4) && (p.x<60)) {
            border_bottom = true;
          }
          if (data) {
            if ((new_border_type != border_type) || (new_conv[new_cnt_ptr].size() > 10)) {
              new_cnt_ptr++;
              if (new_cnt_ptr >= _MAX_NEW_CONV) {
                new_cnt_ptr--;
              }
            }
            new_conv[new_cnt_ptr].clear();
          }
          border_type = new_border_type;
          data = false;
        } else {
          if (layers[0].at<unsigned char>(p.y, p.x) > 200) {
            new_conv[new_cnt_ptr].push_back(Point2f(big_x.at<float>(p.y, p.x), big_y.at<float>(p.y, p.x)));
            //cout << " "<<p.x<<","<<p.y<<" ["<<big_x.at<float>(p.y,p.x)<<", "<<big_y.at<float>(p.y,p.x)<<"] "<<endl;
            if (p.x==u_max || p.x+1==u_max || p.x-1==u_max) {
              if (big_x.at<float>(p.y,p.x)>x_max) {
                x_max = big_x.at<float>(p.y,p.x);
              }
            }
            data = true;
            // circle(dbg_n, p, 2, Scalar(0,255,120), 1);
          }
        }
      }
      if (border && get_border_type(contours[i][0]) < 0 && new_cnt_ptr > 0 && new_conv[new_cnt_ptr].size() > 0) {
        for (auto &p : new_conv[0]) {
          new_conv[new_cnt_ptr].push_back(p);
        }
        new_conv[0].swap(new_conv[new_cnt_ptr]);
      } else if (new_conv[new_cnt_ptr].size() > 0) {
        new_cnt_ptr++;
      }
      ROS_DEBUG_STREAM("Num of segments:" << new_cnt_ptr << " border:" << border << " pix size:" << area);
      RotatedRect box;
      if (new_cnt_ptr == 1 && !border) {
        box = minAreaRect(new_conv[0]);
        ROS_DEBUG_STREAM("WALL box size:" << box.size.width << "," << box.size.height << " center:" << box.center.x << "," << box.center.y);
        test_wall_box(&box, x_max);
      } else {
        //cout <<" max u:"<<u_max<<" max x:"<<x_max<<" bottom:"<<border_bottom<<endl;
        analyze_part_wall(new_cnt_ptr, n_big_cnt, area, x_max, border_bottom);
      }
    }
  }
  /*
  if (show_debug) {
    imshow("d0",dbg_n);
  }*/
}

static void update_mask() {
  rs_mini_mask.copyTo(rs_object_mask);
  rs_mini_mask.copyTo(rs_depth_mask);
}

static void update_mask_griper([[maybe_unused]] int exp_br_type, float dist) {
  int v_mini;
  if (dist > 2.0) {
    if (!is_last_obj_mask) {
      rs_last_object_mask.create(mini.size(), CV_8U);
      rs_last_object_mask.setTo(255);
      is_last_obj_cnt=0;
    }
    is_last_obj_mask = 1;
    if (is_last_obj_cnt<120) {
      is_last_obj_cnt++;
      for (int u = 0; u < mini.cols; u++) {
        v_mini = 0;
        for (int v = 0; v < mini.rows / 2; v++) {
          int d = mini_ptr[u + v_mini];
          if (d > 30000 || d < 500) {
            rs_last_object_mask.at<unsigned char>(v, u) = 0;
          } else {
            break;
          }
          v_mini += mini.cols;
        }
      }
    }
  }
  if (is_last_obj_mask) {
    bitwise_and(rs_mini_mask, rs_last_object_mask, rs_object_mask);
    rs_object_mask.copyTo(rs_depth_mask);
  } else {
    is_last_obj_cnt = 0;
  }
  // imshow("grip", rs_object_mask);
}

static vector<Point> mask_vector;
static int step_x[4] = {1, 1, -1, -1};
static int step_y[4] = {-1, 1, 1, -1};

static void update_mask_object(brick &b) {
  mask_vector.clear();
  float x = r_cx + b.c_x * r_fx / b.c_z;
  float y = r_cy + b.c_y * r_fy / b.c_z;
  float sin_y = sin(b.yaw);
  float cos_y = cos(b.yaw);
  float b_h = 0.14;
  float b_w = brick_len[0] / 1.8;
  if (b.type >= 1 && b.type <= 5) {
    b_w = brick_len[b.type - 1] / 1.8;
  }
  for (int i = 0; i < 4; i++) {
    int xx = (x + (step_x[i] * cos_y * b_w - step_y[i] * sin_y * b_h) * r_fx / b.c_z) / 8.0;
    int yy = (y + (step_x[i] * sin_y * b_w + step_y[i] * cos_y * b_h) * r_fy / b.c_z) / 8.0;
    mask_vector.push_back(Point(xx, yy));
  }
  
  fillConvexPoly(rs_depth_mask, mask_vector, Scalar(0));
}

int find_depth_bricks(cv::Mat src, brick *brick_array, int brick_array_size, int input_bricks, bool use_gui, float &distance, float odom_dist, float pl_a, float pl_b, float pl_c,
                      int mode, float &alf, float &bet, float garmin) {
  double t = (double)getTickCount();
  Size size = src.size();
  Size size_mini(size.width / 8, size.height / 8);
  pl_d.create(size_mini, CV_32F);
  big_d.create(size_mini, CV_32F);
  big_x.create(size_mini, CV_32F);
  big_y.create(size_mini, CV_32F);
  ind_d.create(size_mini, CV_8U);
  is_near = false;
  show_debug = use_gui;
  if (input_bricks > _LOCAL_BRICK_NUM) {
    input_bricks = _LOCAL_BRICK_NUM;
  }
  _MIN_WALL_LEN = 0.55;
  local_brick_num = input_bricks;
  for (int i = 0; i < input_bricks; i++) {
    local_brick[i] = brick_array[i];
  }
  brick_arr = brick_array;
  glob_brick_array_size = brick_array_size;
  glob_brick_num = 0;

  mini.create(size_mini, CV_16U);
  mini_seg.create(size_mini, CV_8U);
  mini_thr.create(size_mini, CV_8U);
  mini_seg.setTo(0);
  mini_thr.setTo(0);

  // for short unsigned int opencv >at< doesn't work
  depth_ptr = (unsigned short int *)(src.data);
  mini_ptr = (unsigned short int *)(mini.data);
  [[maybe_unused]] int v_ind = 0, last_v_ind;

#ifdef _DEBUG_MASK
  Mat dbg;
  dbg.create(size, CV_8U);

  v_ind = 0;
  for (int v = 0; v < src.rows; v++) {
    for (int u = 0; u < src.cols; u++) {
      int val = ((int)(depth_ptr[v_ind + u])) / 20;  // scale to see details in standard depth image
      if (val > 255) {
        dbg.at<unsigned char>(v, u) = 255;
      } else {
        dbg.at<unsigned char>(v, u) = (unsigned char)val;
      }
    }
    v_ind += src.cols;
  }
  imshow("data", dbg);
#endif
  update_mask();
  if ((mode & FIND_WALL) != 0) {
    update_mask_griper(mode & 7, distance);
  } else {
    is_last_obj_mask = 0;
    is_last_obj_cnt = 0;
    for (int i = 0; i < input_bricks; i++) {
      update_mask_object(brick_array[i]);
    }
  }
  // imshow("obj_mask", rs_object_mask);

  int v_ind0 = 0, v_ind1 = src.cols, v_ind2 = 2 * src.cols, v_ind3 = 3 * src.cols, v_ind4 = 4 * src.cols, v_ind5 = 5 * src.cols, v_ind6 = 6 * src.cols, v_ind7 = 7 * src.cols;
  int v_step = 8 * src.cols;

  int v_mini = 0, u_mini = 0;
  for (int v = 0; v < src.rows - 7; v += 8) {
    unsigned short l_min = 65535, a;
    u_mini = 0;
    for (int u = 0; u < src.cols; u++) {
      if (is_mask && rs_depth_mask.at<unsigned char>(v / 8, u / 8) < 200) {
        a = depth_ptr[v_ind0 + u];
        if (rs_mask.at<unsigned char>(v, u) > 50 && a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind1 + u];
        if (rs_mask.at<unsigned char>(v + 1, u) > 50 && a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind2 + u];
        if (rs_mask.at<unsigned char>(v + 2, u) > 50 && a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind3 + u];
        if (rs_mask.at<unsigned char>(v + 3, u) > 50 && a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind4 + u];
        if (rs_mask.at<unsigned char>(v + 4, u) > 50 && a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind5 + u];
        if (rs_mask.at<unsigned char>(v + 5, u) > 50 && a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind6 + u];
        if (rs_mask.at<unsigned char>(v + 6, u) > 50 && a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind7 + u];
        if (rs_mask.at<unsigned char>(v + 7, u) > 50 && a > 0 && a < l_min) {
          l_min = a;
        }
      } else if (!is_mask || rs_depth_mask.at<unsigned char>(v / 8, u / 8) > 100) {
        a = depth_ptr[v_ind0 + u];
        if (a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind1 + u];
        if (a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind2 + u];
        if (a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind3 + u];
        if (a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind4 + u];
        if (a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind5 + u];
        if (a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind6 + u];
        if (a > 0 && a < l_min) {
          l_min = a;
        }
        a = depth_ptr[v_ind7 + u];
        if (a > 0 && a < l_min) {
          l_min = a;
        }
      }
      if ((u % 8) == 7) {
        mini_ptr[u_mini + v_mini] = l_min;
        l_min = 65535;
        u_mini++;
      }
    }
    v_ind0 += v_step;
    v_ind1 += v_step;
    v_ind2 += v_step;
    v_ind3 += v_step;
    v_ind4 += v_step;
    v_ind5 += v_step;
    v_ind6 += v_step;
    v_ind7 += v_step;
    v_mini += mini.cols;
  }


#ifdef _DEBUG1
  dbg_mini.create(size_mini, CV_8UC3);

  v_mini = 0;
  for (int v = 0; v < mini.rows; v++) {
    for (int u = 0; u < mini.cols; u++) {
      int d = mini_ptr[u + v_mini] / 10;
      if (d < 255) {
        dbg_mini.at<Vec3b>(v, u) = Vec3b(d, d, d);
      } else {
        dbg_mini.at<Vec3b>(v, u) = Vec3b(255, 255, 255);
        ;
      }
    }
    v_mini += mini.cols;
  }
  imshow("data", dbg_mini);
#endif

#ifdef _DEBUG
  Mat min_dbg3;
  min_dbg3.create(size_mini, CV_8UC3);

  for (int v = 0; v < mini.rows; v++) {
    for (int u = 0; u < mini.cols; u++) {
      min_dbg3.at<Vec3b>(v, u) = Vec3b(0, 0, 0);
    }
  }
#endif

//  int pt_count2 = 0;
//  int pt_count1 = 0;
  v_mini = 0;
  int v_init = 0;
#define _HIST_SIZE 140
#define _MAX_DIST_HIST 7.0
  int dist_hist[140];
  int max_dist_i = 0;
  int num_infi = 0;
  float koef = _MAX_DIST_HIST / (float)_HIST_SIZE;
  for (int i = 0; i < _HIST_SIZE; i++) {
    dist_hist[i] = 0;
  }
/*  float depth_min = -0.5;
//   float depth_max = 0.1;
  if (distance < 0.6) {
    v_init = mini.rows / 2;
    v_mini = v_init * mini.cols;
  } else if (distance > 2.5) {
    depth_min = -0.6;
    depth_max = 0.6;
  } else if (distance > 4.0) {
    depth_min = -0.8;
    depth_max = 0.8;
  }*/
  l_border = 100;
  for (int v = v_init; v < mini.rows; v++) {
    for (int u = 0; u < mini.cols; u++) {
      float val = ((float)(mini_ptr[v_mini + u]) / _SCALE);  // scale to meters
      if ((u < l_border) && (val < 20.0)) {
        l_border = u;
      }
      float d = (pl_a * (u * 8 + 4 - r_cx) / r_fx + pl_b * ((v * 8 + 4) - r_cy) / r_fy + pl_c) * val;
      // float d = (pl_a*(u*8 + 4 - r_cx)/r_fx + pl_b*(r_cy-(v*8 + 4)) / r_fy + pl_c) * val;
      pl_d.at<float>(v, u) = d;
      int i = (int)(d / koef);
      if (i >= 2 * _HIST_SIZE) {
        num_infi++;
      } else if (i >= _HIST_SIZE) {
        dist_hist[_HIST_SIZE - 1]++;
        max_dist_i = _HIST_SIZE - 1;
      } else if (i >= 0) {
        dist_hist[i]++;
        if (i > max_dist_i) {
          max_dist_i = i;
        }
      }
#ifdef _DEBUG
      if (fabs_diff(d, distance) < 0.3) {
        min_dbg3.at<Vec3b>(v, u) = bcolors[5];
      }
      if (fabs_diff(d, distance) < 0.25) {
        min_dbg3.at<Vec3b>(v, u) = bcolors[4];
      }
      if (fabs_diff(d, distance) < 0.2) {
        min_dbg3.at<Vec3b>(v, u) = bcolors[3];
      }
      if (fabs_diff(d, distance) < 0.15) {
        min_dbg3.at<Vec3b>(v, u) = bcolors[2];
      }
      if ((distance - pl_d.at<float>(v, u)) < 0.1 && (distance - pl_d.at<float>(v, u)) > -0.5) {
        min_dbg3.at<Vec3b>(v, u) = bcolors[1];
      }
      if (fabs_diff(d, distance) < 0.05) {
        min_dbg3.at<Vec3b>(v, u) = bcolors[0];
      }
#endif
     /* if (fabs_diff(d, distance) < 0.2) {
        pt_count2++;
      }
      if (rs_object_mask.at<unsigned char>(v, u) > 200 && (distance - pl_d.at<float>(v, u)) < depth_max && (distance - pl_d.at<float>(v, u)) > depth_min) {
        pt_count1++;
      } */
    }
    v_mini += mini.cols;
  }
  ROS_DEBUG_STREAM("L BORDER:" << l_border);
  l_border = 2;
#ifdef _DEBUG
  ROS_DEBUG_STREAM("Plane # 0.1:" << pt_count1 << " 0.2:" << pt_count2);
  imshow("plane", min_dbg3);
#endif
/*
  unsigned char seg_num = 0;
  for (int v = 0; v < mini.rows; v++) {
    for (int u = 0; u < mini.cols; u++) {
      if (mini_thr.at<unsigned char>(v, u) == 0) {
        stack_init();
        stack_push(u, v);
        mini_thr.at<unsigned char>(v, u) = 128;
        while (!stack_empty()) {
          int uu, vv;
          float tmp_d;
          stack_pop(&uu, &vv);
          mini_seg.at<unsigned char>(vv, uu) = seg_num;
          tmp_d = pl_d.at<float>(vv, uu);
          if (uu > 0 && mini_thr.at<unsigned char>(vv, uu - 1) == 0 && fabs_diff(pl_d.at<float>(vv, uu - 1), tmp_d) < PLANE_DIFF) {
            stack_push(uu - 1, vv);
            mini_thr.at<unsigned char>(vv, uu - 1) = 128;
          }
          if (uu + 1 < mini.cols && mini_thr.at<unsigned char>(vv, uu + 1) == 0 && fabs_diff(pl_d.at<float>(vv, uu + 1), tmp_d) < PLANE_DIFF) {
            stack_push(uu + 1, vv);
            mini_thr.at<unsigned char>(vv, uu + 1) = 128;
          }
          if (vv > 0 && mini_thr.at<unsigned char>(vv - 1, uu) == 0 && fabs_diff(pl_d.at<float>(vv - 1, uu), tmp_d) < PLANE_DIFF) {
            stack_push(uu, vv - 1);
            mini_thr.at<unsigned char>(vv - 1, uu) = 128;
          }
          if (vv + 1 < mini.rows && mini_thr.at<unsigned char>(vv + 1, uu) == 0 && fabs_diff(pl_d.at<float>(vv + 1, uu), tmp_d) < PLANE_DIFF) {
            stack_push(uu, vv + 1);
            mini_thr.at<unsigned char>(vv + 1, uu) = 128;
          }
        }
        seg_num++;
      }
    }
  }
*/
#ifdef _DEBUG
  ROS_DEBUG_STREAM("SEGMENTS " << seg_num);
  Mat min_dbg2;
  min_dbg2.create(size_mini, CV_8UC3);

  for (int v = 0; v < mini.rows; v++) {
    for (int u = 0; u < mini.cols; u++) {
      min_dbg2.at<Vec3b>(v, u) = bcolors[mini_seg.at<unsigned char>(v, u) % 10];
    }
  }
  imshow("segment", min_dbg2);
#endif
  float plane_d = distance;

  /*
  if (pt_count1 > 100000000) {
    Eigen::MatrixXf plane(3, pt_count1);
    int ii = 0;

    v_mini = v_init * mini.cols;
    for (int v = v_init; v < mini.rows; v++) {
      for (int u = 0; u < mini.cols; u++) {
        if (rs_object_mask.at<unsigned char>(v, u) > 200 && (distance - pl_d.at<float>(v, u)) < depth_max && (distance - pl_d.at<float>(v, u)) > depth_min) {
          float Z = (static_cast<float>(mini_ptr[v_mini + u])) / _SCALE;
          plane.col(ii++) << ((u * 8 + 4 - r_cx) * Z / r_fx), (((v * 8 + 4) - r_cy) * Z / r_fy), Z;
          // plane.col(ii++) << ((u*8 + 4 - r_cx) * Z / r_fx), ((r_cy- (v*8 + 4)) * Z / r_fy), Z;
        }
      }
      v_mini += mini.cols;
    }
    Eigen::Matrix<float, 3, 1> mean = plane.rowwise().mean();
    const Eigen::Matrix3Xf points_centered = plane.colwise() - mean;
    int setting = Eigen::ComputeFullU | Eigen::ComputeThinV;
    Eigen::JacobiSVD<Eigen::Matrix3Xf> svd = points_centered.jacobiSvd(setting);
    normal_z = svd.matrixU().col(2);
    Eigen::Vector3f plane_x = svd.matrixU().col(0);
    Eigen::Vector3f plane_y = svd.matrixU().col(1);
    normal_z.normalize();
    plane_x.normalize();
    plane_y.normalize();
    plane_d = normal_z(0) * mean(0) + normal_z(1) * mean(1) + normal_z(2) * mean(2);
    if (plane_d < 0) {
      normal_z(0) = -normal_z(0);
      normal_z(1) = -normal_z(1);
      normal_z(2) = -normal_z(2);
      plane_d = normal_z(0) * mean(0) + normal_z(1) * mean(1) + normal_z(2) * mean(2);
    }
    //#ifdef _DEBUG
    ROS_DEBUG_STREAM("Normal " << normal_z(0) << "," << normal_z(1) << "," << normal_z(2) << " sum hist:" << pt_count1 << "m aprox:" << plane_d << "last altitude:" << distance
                               << " odom alt:" << odom_dist);
    //#endif

    double alf1, bet1;
    Eigen::Vector3f orig;
    orig[0] = pl_a;
    orig[1] = pl_b;
    orig[2] = pl_c;
    diff_vec(orig, normal_z, alf1, bet1);
    alf = alf1;
    bet = bet1;
    ROS_DEBUG_STREAM("Angle diff " << alf << "," << bet);
    if (alf < 0.2 && alf > -0.2 && bet < 0.2 && bet > -0.2) {
      distance = plane_d;
    } else {
      int cnt = dist_hist[max_dist_i];
      int j = max_dist_i;
      while (j > 2 && cnt < 2000) {
        j--;
        cnt += dist_hist[j];
      }
      ROS_DEBUG_STREAM("2 - Minimal # points:" << pt_count2 << " reset from:" << distance << " to:" << odom_dist << " hist_dist:" << (j * koef) << " j:" << j
                                               << " hist[j]:" << dist_hist[j] << " poc nek:" << num_infi);
      normal_z(0) = pl_a;
      normal_z(1) = pl_b;
      normal_z(2) = pl_c;
      plane_d = j * koef;
      distance = plane_d;
      alf = -10.0;
      bet = -10.0;
    }
  } else { */
    int cnt = dist_hist[max_dist_i];
    int j = max_dist_i;
    while (j > 2 && cnt < 1000) {
      j--;
      cnt += dist_hist[j];
    }
    ROS_DEBUG_STREAM("Odom dist:" << odom_dist << " hist_dist:" << (j * koef) << " j:" << j
                                         << " hist[j]:" << dist_hist[j] << " sum:"<<cnt<<" #infi:" << num_infi);
    normal_z(0) = pl_a;
    normal_z(1) = pl_b;
    normal_z(2) = pl_c;
    plane_d = j * koef;
    distance = plane_d;
    alf = -10.0;
    bet = -10.0;
  //}
  /*int cnt = dist_hist[10];
  int j = 10;
  while (j < max_dist_i && cnt < 400) {
    j++;
    cnt += dist_hist[j];
  }
  float closest_obj=(j*koef);
  ROS_DEBUG_STREAM("Closest object:" << (j * koef) << " cnt:" << cnt);
  if (((mode & FIND_WALL) != 0) || (mode == 0)) {
    wall_lvl = (mode >> 3) & 3;
    if (wall_lvl == 0) {
      if (closest_obj<plane_d-_WALL_HIGH-_BRICK_HIGH) {
        ROS_DEBUG_STREAM("Change distance from:" << plane_d << " to:" << (closest_obj+_WALL_HIGH+_BRICK_HIGH));
        plane_d = closest_obj+_WALL_HIGH+_BRICK_HIGH;
        distance = plane_d;
      }
    } else {
      if (closest_obj<plane_d-_WALL_HIGH-2*_BRICK_HIGH) {
        ROS_DEBUG_STREAM("Change distance from:" << plane_d << " to:" << (closest_obj+_WALL_HIGH+2*_BRICK_HIGH));
        plane_d = closest_obj+_WALL_HIGH+2*_BRICK_HIGH;
        distance = plane_d;
      }
    }
  }
  */
  if (distance>3.5) {
    if (garmin+_WALL_HIGH<distance+0.5) {
      garmin+=_WALL_HIGH;
    }
    if (garmin>distance) {
      float koef = (distance-3.5)/2.0;
      if (koef>1.0) {
        koef=1.0;
      }
      distance=garmin*koef+(1.0-koef)*distance;
    }
  }
  normal_x << normal_z(2), 0, -normal_z(0);
  normal_x.normalize();
  normal_y = normal_z.cross(normal_x);
  normal_y.normalize();

  ROS_DEBUG_STREAM("Normal z" << normal_z(0) << "," << normal_z(1) << "," << normal_z(2) << "Normal x" << normal_x(0) << "," << normal_x(1) << "," << normal_x(2) << " Normal y"
                              << normal_y(0) << "," << normal_y(1) << "," << normal_y(2));

  init_big_mat(plane_d);
  if ((mode & FIND_WALL) == 0) {
    find_brick = (mode & 7);
    track_object(plane_d, _BRICK_LIMIT, _BRICK_HIGH);
  }
  if (((mode & FIND_WALL) != 0) || (mode == 0)) {
    find_brick = (mode & 7);
    to_brick = (mode >> 5) & 7;
    wall_lvl = (mode >> 3) & 3;
    if (mode == 0) {
      track_wall(plane_d, _WALL_LIMIT, _WALL_HIGH);
    } else if (wall_lvl == 1) {
      /*if (closest_obj<plane_d-_WALL_HIGH-_BRICK_HIGH) {
        plane_d = closest_obj+_WALL_HIGH+_BRICK_HIGH;
      }*/
      track_wall(plane_d, _WALL_LIMIT, _WALL_HIGH);
    } else {
      /*if (closest_obj<plane_d-_WALL_HIGH-2*_BRICK_HIGH) {
        plane_d = closest_obj+_WALL_HIGH+2*_BRICK_HIGH;
      }*/
      track_wall(plane_d, _WALL_LIMIT, _WALL_HIGH + _BRICK_HIGH);
    }
  }

  double t2 = (double)getTickCount();
  ROS_DEBUG_STREAM("Depth Time " << ((t2 - t) * 1000. / getTickFrequency()) << "ms");
  Mat dbg2;
  if (use_gui) {
    dbg2.create(size, CV_8UC3);

    for (int v = 0; v < src.rows; v++) {
      for (int u = 0; u < src.cols; u++) {
        float val = big_d.at<float>(v / 8, u / 8);  // scale to see details in standard depth image
        if (val < _BRICK_LIMIT) {
          dbg2.at<Vec3b>(v, u) = Vec3b(128, 128, 210);
        } else if (val < _BRICK_HIGH + 2 * _BRICK_LIMIT) {
          dbg2.at<Vec3b>(v, u) = Vec3b(128, 128, 128);
        } else if (val < _WALL_LIMIT) {
          dbg2.at<Vec3b>(v, u) = Vec3b(10, 210, 10);
        } else if (val < _WALL_HIGH + _BRICK_LIMIT) {
          dbg2.at<Vec3b>(v, u) = Vec3b(10, 200, 200);
        } else if (val < _WALL_HIGH + _BRICK_LIMIT + _BRICK_HIGH) {
          dbg2.at<Vec3b>(v, u) = Vec3b(10, 100, 100);
        } else {
          dbg2.at<Vec3b>(v, u) = Vec3b(0, 0, 0);
        }
        if (rs_depth_mask.at<unsigned char>(v / 8, u / 8) < 50) {
          Vec3b c = dbg2.at<Vec3b>(v, u);
          c[0] = 255;
          dbg2.at<Vec3b>(v, u) = c;
        }
      }
    }
    ROS_DEBUG_STREAM("Glob brick num:" << glob_brick_num);
    for (int i = 0; i < glob_brick_num; i++) {
      circle(dbg2, Point(r_cx + brick_arr[i].c_x * r_fx / brick_arr[i].c_z, r_cy + brick_arr[i].c_y * r_fy / brick_arr[i].c_z), 7, brick_colors[(int)brick_arr[i].type - 1], 2);
      line(dbg2, Point(r_cx + brick_arr[i].c_x * r_fx / brick_arr[i].c_z, r_cy + brick_arr[i].c_y * r_fy / brick_arr[i].c_z),
           Point(r_cx + brick_arr[i].c_x * r_fx / brick_arr[i].c_z + 20 * cos(brick_arr[i].yaw), r_cy + brick_arr[i].c_y * r_fy / brick_arr[i].c_z + 20 * sin(brick_arr[i].yaw)),
           brick_colors[(int)brick_arr[i].type], 2, LINE_AA);
    }
    for (int i = 0; i < input_bricks; i++) {
      circle(dbg2, Point(local_brick[i].c_x, local_brick[i].c_y), 5, brick_colors[(int)local_brick[i].type - 1], 1);
    }
    imshow("rsdbg", dbg2);
#ifdef _DEBUG1
    imshow("data", dbg_mini);
#endif
  }
#ifdef _DEBUG_MASK
  imwrite("/home/petr/big/depth.bmp", dbg);
#endif
  return glob_brick_num;
}

}  // namespace brick_detection
