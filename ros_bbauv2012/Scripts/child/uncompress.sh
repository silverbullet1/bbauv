#!/bin/bash

pkill -f 'republish.*color'
ROS_NAMESPACE=/stereo_camera/left rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color &
ROS_NAMESPACE=/stereo_camera/right rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color &
ROS_NAMESPACE=/bottomcam/camera rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color &

