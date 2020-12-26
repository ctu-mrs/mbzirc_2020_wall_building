cmake_minimum_required(VERSION 3.1.2)
project(wall_simulation)

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
