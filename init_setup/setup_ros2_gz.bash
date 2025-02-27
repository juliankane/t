#!/bin/bash

#### Install ROS2 - Jazzy

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings


# Enable required repositories
sudo apt install software-properties-common   # ensure Ubuntu Universe repository
sudo add-apt-repository universe


sudo apt update && sudo apt install curl -y # add repository to sources
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg



sudo apt update && sudo apt install ros-dev-tools  # install development tools

sudo apt update # update your apt repository caches

sudo apt upgrade # upgrade system


sudo apt install ros-jazzy-desktop # install ROS, RVis, demos, tutorials


echo "/opt/ros/jazzy/setup.bash" >> ~/.bashrc  # configure bash file with your ros2 version
source /opt/ros/jazzy/setup.bash



### Install Gazebo

sudo apt-get update
sudo apt-get install curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic



