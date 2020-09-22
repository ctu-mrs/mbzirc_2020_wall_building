cmake_minimum_required(VERSION 2.8.3)
project(mrs_gripper)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_msgs
  std_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# since we need boost search it as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Boost REQUIRED COMPONENTS system thread)

#generate_dynamic_reconfigure_options(
#  cfg/TerarangerOne.cfg
#)

# export the dependencis of this package for who ever depends on us
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs mrs_msgs
  DEPENDS Boost
)

# tell catkin where to find the headers for this project
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# make the executables
add_executable(mrs_gripper src/mrs_gripper.cpp)
add_dependencies(mrs_gripper ${catkin_EXPORTED_TARGETS})
target_link_libraries(mrs_gripper ${catkin_LIBRARIES} ${Boost_LIBRARIES})
