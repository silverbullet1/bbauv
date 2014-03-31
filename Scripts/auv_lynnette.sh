#!/bin/sh

pkill -f 'republish.*lynnette'
#cd /home/gohew/fuerte_workspace/bbauv/ros_bbauv2012/Topside/auv_gui
ROS_NAMESPACE=/front_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_color_lynnette &
#ROS_NAMESPACE=/stereo_camera/right rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color_opt &
ROS_NAMESPACE=/bot_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_color_lynnette &
ROS_NAMESPACE=/Vision rosrun image_transport republish compressed in:=image_filter raw out:=image_filter_opt_lynnette &
#rosrun gui controlpanel.py _front:=/front_camera/camera/image_color_lynnette _bottom:=/bot_camera/camera/image_color_lynnette _filter:=/Vision/image_filter_opt_lynnette  &
rosrun gui controlpanel.py _front:=/front_camera/camera/image_color_lynnette _bottom:=/bot_camera/camera/image_color_lynnette _testing:=True &
