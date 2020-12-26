cmake_minimum_required(VERSION 3.1.2)
project(arena_scan_planner)

##################################
## Check c++11 / c++0x support ###
##################################
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
  message(FATAL_ERROR "Compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
endif()
####:##############################

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  std_srvs
  mrs_msgs
  sensor_msgs
  geometry_msgs
  tf
  message_generation
  )

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

## Generate messages in the 'msg' folder
# add_message_files(
#  FILES
#TargetWaypoint.msg
#TargetWaypointArray.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ArenaScanTrajectory.srv
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  mrs_msgs
  )

catkin_package(
  # INCLUDE_DIRS include
  #  LIBRARIES charging
  LIBRARIES dubins_lib
  CATKIN_DEPENDS sensor_msgs std_msgs geometry_msgs roscpp mrs_msgs
  DEPENDS
  )


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#set(dubins_lib_sources
#  src/dubins/angleinterval.cpp
#  src/dubins/arc.cpp
#  src/dubins/circle.cpp
#  src/dubins/dubins.cpp
#  src/dubins/dubinsmaneuver.cpp
#  src/dubins/dubinspath.cpp
#  src/dubins/line.cpp
#  src/dubins/linesegment.cpp
#  src/dubins/path.cpp
#  src/dubins/state.cpp
#  src/dubins/MathCommon.cpp)

set(dubins_lib_sources
  src/dubins/angleinterval.cpp
  src/dubins/arc.cpp
  src/dubins/circle.cpp
  src/dubins/dubins.cpp
  src/dubins/dubinsmaneuver.cpp
  src/dubins/dubinspath.cpp
  src/dubins/line.cpp
  src/dubins/path.cpp
  src/dubins/state.cpp
  src/dubins/MathCommon.cpp)

## Declare a C++ library

add_library(dubins_lib  ${dubins_lib_sources})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
add_executable(arena_scan_planner src/dubins_trajectory.cpp src/arena_scan_planner.cpp)

add_dependencies(arena_scan_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  arena_scan_planner
  dubins_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  )
