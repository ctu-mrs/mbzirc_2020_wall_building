cmake_minimum_required(VERSION 2.8.3)
project(brick_mapping)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  mrs_msgs
  mbzirc_msgs
  mrs_lib
  message_generation
  actionlib
  nodelet
  tf
  mavros_msgs
  rospy
  )

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

add_message_files(DIRECTORY msg FILES
  MappedArenaObjects.msg  # e.g. Floats.msg HeaderString.msg
  MappedArenaObjectsStamped.msg
)

add_service_files(DIRECTORY srv FILES
  GetMappedArenaObjects.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  mbzirc_msgs
  mrs_msgs)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS mrs_msgs mbzirc_msgs message_runtime roscpp sensor_msgs std_msgs geometry_msgs mrs_lib mavros_msgs rospy
  DEPENDS Eigen
  LIBRARIES BrickGrasping
  )