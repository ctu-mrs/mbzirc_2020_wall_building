// clang: PetrFormat

#include "brick.h"

namespace brick_detection
{

#define my_sqr(a) ((a) * (a))

static brick brick_map[BRICK_MAPS][BRICK_MAP_SIZE];
static int time_limit[BRICK_MAPS];

void map_init(int m, int time) {
  for (int i = 0; i < BRICK_MAP_SIZE; i++) {
    brick_map[m][i].last_time = time + 1;
  }
  time_limit[m] = time;
}

void map_update(int m) {
  for (int i = 0; i < BRICK_MAP_SIZE; i++) {
    if (brick_map[m][i].last_time <= time_limit[m]) {
      brick_map[m][i].last_time++;
    }
  }
}

int valid_map_brick(int m, int i) {
  return (brick_map[m][i].last_time < time_limit[m]);
}

int find_map_brick(int m, float x, float y, enum Brick_type type) {
  float min_dist = 10000000.0;
  int min_dist_i = -1;

  for (int i = 0; i < BRICK_MAP_SIZE; i++) {
    if (brick_map[m][i].last_time < time_limit[m] && brick_map[m][i].type == type) {
      float dist = my_sqr(brick_map[m][i].c_x - x) + my_sqr(brick_map[m][i].c_y - y);
      if (dist < min_dist) {
        min_dist = dist;
        min_dist_i = i;
      }
    }
  }
  if (min_dist_i >= 0) {
    brick_map[m][min_dist_i].last_time = 0;
  }
  return min_dist_i;
}

int add_map_brick(int m, float x, float y, float z, float yaw, enum Brick_type t) {
  int max = 0;
  int max_i = -1;
  for (int i = 0; i < BRICK_MAP_SIZE; i++) {
    if (brick_map[m][i].last_time > max) {
      max = brick_map[m][i].last_time;
      max_i = i;
    }
    if (brick_map[m][i].last_time >= time_limit[m]) {
      brick_map[m][i].c_x = x;
      brick_map[m][i].c_y = y;
      brick_map[m][i].c_z = z;
      brick_map[m][i].yaw = yaw;
      brick_map[m][i].type = t;
      brick_map[m][i].last_time = 0;
      return i;
    }
  }
  brick_map[m][max_i].c_x = x;
  brick_map[m][max_i].c_y = y;
  brick_map[m][max_i].c_z = z;
  brick_map[m][max_i].yaw = yaw;
  brick_map[m][max_i].type = t;
  brick_map[m][max_i].last_time = 0;
  return max_i;
}

brick *get_map_brick(int m, int i) {
  if (i < 0 || i >= BRICK_MAP_SIZE) {
    i = 0;
  }
  return &brick_map[m][i];
}

}  // namespace brick_detection
