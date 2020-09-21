cmake_minimum_required(VERSION 2.8.3)
project(automatic_start_mbzirc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_msgs
  mrs_lib
  nodelet
  std_msgs
  mavros_msgs
  )

catkin_package(
  #  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs mrs_msgs mavros_msgs mrs_lib
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS})

# automatic start MBZIRC

add_library(AutomaticStartMbzirc
  src/automatic_start_mbzirc.cpp)

add_dependencies(AutomaticStartMbzirc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(AutomaticStartMbzirc
  ${catkin_LIBRARIES}
  )
