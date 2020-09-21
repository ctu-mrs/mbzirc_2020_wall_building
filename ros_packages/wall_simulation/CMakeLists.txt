cmake_minimum_required(VERSION 2.8.3)
project(wall_simulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  )

catkin_package(
  CATKIN_DEPENDS gazebo_ros
  )

install(DIRECTORY gazebo_files/models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY gazebo_files/worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
