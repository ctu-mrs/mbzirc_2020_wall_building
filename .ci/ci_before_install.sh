#!/bin/bash

set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting install preparation"

openssl aes-256-cbc -K $encrypted_f0fd3ee254e8_key -iv $encrypted_f0fd3ee254e8_iv -in ./.ci/deploy_key_github.enc -out ./.ci/deploy_key_github -d
eval "$(ssh-agent -s)"
chmod 600 ./.ci/deploy_key_github
ssh-add ./.ci/deploy_key_github

sudo apt-get update -qq
sudo apt-mark hold openssh-server
sudo apt -y upgrade --fix-missing
sudo apt-get install dpkg git python-setuptools python3-setuptools python3-pip

echo "clone uav_core"
cd
git clone https://github.com/ctu-mrs/uav_core.git
cd uav_core

echo "running the main install.sh"
./installation/install.sh

gitman update

MY_PATH=`dirname $TRAVIS_BUILD_DIR`

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/uav_core
ln -s "$MY_PATH" wall_packages
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws

echo "install part ended"
