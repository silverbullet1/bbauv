#!/bin/bash

pkill "republish"

# For 2013 robosub bags
ROS_NAMESPACE=/bottomcam/camera rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color &

ROS_NAMESPACE=/stereo_camera/left rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color &
ROS_NAMESPACE=/stereo_camera/right rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color &

# For 2014 sauvc bags
ROS_NAMESPACE=/bot_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_raw_thien &
ROS_NAMESPACE=/Vision rosrun image_transport republish compressed in:=image_filter raw out:=image_filter_thien &

