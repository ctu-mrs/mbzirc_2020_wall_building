cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_flyto_planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  mrs_msgs
  mrs_lib
  plan_keeper
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
# )

## Generate services in the 'srv' folder
# add_service_files(
# )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  FlyTo.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  mrs_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
# INCLUDE_DIRS include
 # LIBRARIES mbzirc_planner
 CATKIN_DEPENDS 
  geometry_msgs
  plan_keeper
  std_msgs
  nav_msgs
  actionlib_msgs
  mrs_msgs
 # DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
#  include
)

## Declare a C++ executable
add_executable(flyto_server src/flyto_server.cpp)
add_dependencies(flyto_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flyto_server ${catkin_LIBRARIES} )

add_executable(flyto_client src/flyto_client.cpp)
add_dependencies(flyto_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flyto_client ${catkin_LIBRARIES} )
