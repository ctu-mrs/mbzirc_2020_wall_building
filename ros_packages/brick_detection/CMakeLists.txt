cmake_minimum_required(VERSION 3.1.2)
project(brick_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  dynamic_reconfigure
  mrs_msgs
  mrs_lib
  mbzirc_msgs
  nodelet
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  tf2_eigen
  tf2_geometry_msgs
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/brick_detection.cfg
)

catkin_package(
  CATKIN_DEPENDS message_runtime image_transport mrs_lib # roscpp rospy std_msgs
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(BrickDetection src/brick_detection.cpp src/brick.cpp src/brick_depth.cpp src/brick_map.cpp)

add_dependencies(BrickDetection
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(BrickDetection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
