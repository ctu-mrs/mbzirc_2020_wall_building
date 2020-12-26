cmake_minimum_required(VERSION 3.1.2)
project(mbzirc_flyto_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  FILES
  FlyTo.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  mrs_msgs
)

catkin_package(
 CATKIN_DEPENDS
  geometry_msgs
  plan_keeper
  std_msgs
  nav_msgs
  actionlib_msgs
  mrs_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(flyto_server src/flyto_server.cpp)
add_dependencies(flyto_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flyto_server ${catkin_LIBRARIES} )

add_executable(flyto_client src/flyto_client.cpp)
add_dependencies(flyto_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flyto_client ${catkin_LIBRARIES} )
