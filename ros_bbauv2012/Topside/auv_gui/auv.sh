#!/bin/bash

pkill -f 'republish.*opt'
cd /home/gohew/fuerte_workspace/bbauv/ros_bbauv2012/Topside/auv_gui
ROS_NAMESPACE=/stereo_camera/left rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color_opt &
#ROS_NAMESPACE=/stereo_camera/right rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color_opt &
ROS_NAMESPACE=/bottomcam/camera rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color_opt &
ROS_NAMESPACE=/Vision rosrun image_transport republish compressed in:=image_filter raw out:=image_filter_opt &
rosrun auv_gui auv_gui.py &
