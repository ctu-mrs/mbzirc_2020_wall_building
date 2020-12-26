cmake_minimum_required(VERSION 3.1.2)
project(automatic_start_mbzirc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_msgs
  mrs_lib
  nodelet
  std_msgs
  mavros_msgs
  )

catkin_package(
  CATKIN_DEPENDS std_msgs mrs_msgs mavros_msgs mrs_lib
  )

###########
## Build ##
###########

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
