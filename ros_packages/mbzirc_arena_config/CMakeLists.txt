cmake_minimum_required(VERSION 3.1.2)
project(mbzirc_arena_config)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ArenaPublisher
  CATKIN_DEPENDS roscpp mrs_lib mbzirc_msgs mrs_msgs tf2_geometry_msgs geometry_msgs nav_msgs
  DEPENDS Eigen
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

add_library(ArenaPublisher src/arena_publisher.cpp)

target_link_libraries(ArenaPublisher
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
