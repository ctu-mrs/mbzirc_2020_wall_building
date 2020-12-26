cmake_minimum_required(VERSION 3.1.2)
project(mrs_gripper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_msgs
  std_msgs
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs mrs_msgs
  DEPENDS Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(mrs_gripper src/mrs_gripper.cpp)
add_dependencies(mrs_gripper ${catkin_EXPORTED_TARGETS})
target_link_libraries(mrs_gripper ${catkin_LIBRARIES} ${Boost_LIBRARIES})
