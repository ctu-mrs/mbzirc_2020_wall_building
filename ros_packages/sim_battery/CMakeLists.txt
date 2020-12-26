cmake_minimum_required(VERSION 3.1.2)
project(sim_battery)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
  CATKIN_DEPENDS mrs_msgs mbzirc_msgs message_runtime roscpp sensor_msgs std_msgs geometry_msgs mrs_lib mavros_msgs rospy
  )
