#!/bin/bash

pkill -f 'republish.*opt'
#cd /home/gohew/fuerte_workspace/bbauv/ros_bbauv2012/Topside/auv_gui
ROS_NAMESPACE=/front_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_rect_color_opt_gew &
ROS_NAMESPACE=/bot_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_rect_color_opt_gew &
ROS_NAMESPACE=/Vision rosrun image_transport republish compressed in:=image_filter raw out:=image_filter_opt_gew &
rosrun gui controlpanel.py _front:=/front_camera/camera/image_rect_color_opt_gew _filter:=/Vision/image_filter_opt_gew _bottom:=/bot_camera/camera/image_rect_color_opt_gew  &
