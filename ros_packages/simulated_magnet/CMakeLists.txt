cmake_minimum_required(VERSION 3.1.2)
project(simulated_magnet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime mbzirc_msgs mrs_msgs mrs_gazebo_extras_resources
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(simulated_magnet src/simulated_magnet.cpp)

add_dependencies(simulated_magnet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simulated_magnet
   ${catkin_LIBRARIES}
)
