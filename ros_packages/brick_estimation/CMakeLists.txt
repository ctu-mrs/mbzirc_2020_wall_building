cmake_minimum_required(VERSION 2.8.3)
project(brick_estimation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

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
  dynamic_reconfigure
  actionlib
  brick_detection
  nodelet
  tf
  )

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

generate_dynamic_reconfigure_options(
  config/brick_estimation_drs.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS mrs_msgs mbzirc_msgs roscpp sensor_msgs std_msgs geometry_msgs mrs_lib
  DEPENDS Eigen
  LIBRARIES BrickEstimation
  )

find_package(Boost REQUIRED COMPONENTS thread)

# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# BrickEstimation

add_library(BrickEstimation src/brick_estimation.cpp)

add_dependencies(BrickEstimation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(BrickEstimation
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  )
