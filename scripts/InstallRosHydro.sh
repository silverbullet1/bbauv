#!/usr/bin/env sh

#Code to install ros hydro Ubuntu 12.04

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-hydro-desktop-full
sudo apt-get install ros-hydro-camera1394 # ROS drivers for FireWire cameras

#Initialize rosdep
sudo rosdep init
rosdep update

#Set up environment
echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
source ~/.bashrc

#Get rosinstall
sudo apt-get install python-rosinstall

