#!/usr/bin/env sh

#Install uwsim for ros hydro

sudo apt-get update
sudo apt-get install ros-hydro-uwsim
cd ..
catkin_make