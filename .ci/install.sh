#!/bin/bash

set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

MY_PATH=`pwd`

echo "Starting install preparation"

sudo apt-get -y install git

echo "clone uav_core"
cd
git clone https://github.com/ctu-mrs/uav_core.git
cd uav_core

echo "running the main install.sh"
./installation/install.sh

echo "clone simulation"
cd
git clone https://github.com/ctu-mrs/simulation.git
cd simulation

echo "running the main install.sh"
./installation/install.sh

echo "clone mrs_gazebo_extras_resources"
cd
git clone https://github.com/ctu-mrs/mrs_gazebo_extras_resources
cd mrs_gazebo_extras_resources

sudo apt-get -y install ros-$ROS_DISTRO-flexbe-behavior-engine

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/uav_core
ln -s ~/simulation
ln -s ~/mrs_gazebo_extras_resources
ln -s "$MY_PATH" wall_packages
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws

echo "installation part ended"
