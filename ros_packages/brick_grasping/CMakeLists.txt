cmake_minimum_required(VERSION 3.1.2)
project(brick_grasping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS mrs_msgs mbzirc_msgs roscpp sensor_msgs std_msgs geometry_msgs mrs_lib mavros_msgs
  DEPENDS Eigen
  LIBRARIES BrickGrasping
  )

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# BrickGrasping

add_library(BrickGrasping src/brick_grasping.cpp)

add_dependencies(BrickGrasping
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(BrickGrasping
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  )
