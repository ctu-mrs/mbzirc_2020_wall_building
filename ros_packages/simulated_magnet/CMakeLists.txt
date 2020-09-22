cmake_minimum_required(VERSION 2.8.3)
project(simulated_magnet)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  mbzirc_msgs
  mrs_gazebo_extras_resources
  mrs_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cross_detector
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime mbzirc_msgs mrs_msgs mrs_gazebo_extras_resources
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ executable
add_executable(simulated_magnet src/simulated_magnet.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(simulated_magnet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(simulated_magnet
   ${catkin_LIBRARIES}
)

