cmake_minimum_required(VERSION 2.8.3)
project(sim_battery)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  mrs_msgs
  mbzirc_msgs
  mrs_lib
  message_generation
  mavros_msgs
  rospy
  )



catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS mrs_msgs mbzirc_msgs message_runtime roscpp sensor_msgs std_msgs geometry_msgs mrs_lib mavros_msgs rospy
  )
