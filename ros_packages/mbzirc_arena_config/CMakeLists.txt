cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_arena_config)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  mrs_msgs
  mbzirc_msgs
  mrs_lib
  )

#Find Eigen libraries
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

## Generate dynamic parameters which are loaded by the dynamic reconfigure server
# generate_dynamic_reconfigure_options(
#   config/dynparam.cfg
#   )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ArenaPublisher
  CATKIN_DEPENDS roscpp mrs_lib mbzirc_msgs mrs_msgs tf2_geometry_msgs geometry_msgs nav_msgs
  DEPENDS Eigen
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

## Declare a C++ library
add_library(ArenaPublisher src/arena_publisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ArenaPublisher
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
