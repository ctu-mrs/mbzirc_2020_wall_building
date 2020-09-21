cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_msgs)

set(CMAKE_CXX_FLAGS "-std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  std_srvs
  mrs_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  cmake_modules
  )

###############################################
## Declare ROS messages, services and actions ##
################################################

# messages for BRICK_DETECTION
add_message_files(DIRECTORY msg/brick_detection FILES

  DetectedObject.msg
  ObjectWithType.msg
  ObjectUavOdometry.msg
  ObjectWithTypeArray.msg
  )

# messages for BRICK_ESTIMATION
add_message_files(DIRECTORY msg/brick_estimation FILES

  BrickWithId.msg
  BrickWithIdArray.msg
  MbzircBrick.msg
  EstimatedBrick.msg
  BrickMap.msg
  )

# messages for BRICK_GRASPING
add_message_files(DIRECTORY msg/brick_grasping FILES

  BrickGraspingDiagnostics.msg
  Target.msg
  )

# messages for MBZIRC_ARENA_CONFIG
add_message_files(DIRECTORY msg/mbzirc_arena_config FILES

  ArenaZone.msg
  MbzircArenaParameters.msg
  )
# services for BRICK_GRASPING

add_service_files(DIRECTORY srv/brick_grasping FILES

  BanArea.srv
  StartGrasping.srv
  StartLanding.srv
  MagnetControl.srv
  Drop.srv
  )

  add_service_files(DIRECTORY srv/brick_detection FILES

  DetectionType.srv
  )

  add_service_files(DIRECTORY srv/mbzirc_arena_config FILES

  ArenaZoneSrv.srv
  )

add_action_files(DIRECTORY action_server/brick_grasping FILES
  Grasping.action
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES

  std_msgs
  geometry_msgs
  sensor_msgs
  mrs_msgs
  actionlib_msgs
  )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime sensor_msgs mrs_msgs
  LIBRARIES mbzirc_msgs
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(mbzirc_msgs src/main.cpp)

add_dependencies(mbzirc_msgs
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(mbzirc_msgs
  ${catkin_LIBRARIES}
  )

add_custom_command(TARGET mbzirc_msgs
  PRE_BUILD
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  COMMAND bash -c "${CMAKE_CURRENT_SOURCE_DIR}/link_msg_files.sh"
  )
