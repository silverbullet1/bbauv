#!/bin/bash

ROS_NAMESPACE=/debug/stereo_camera/left rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color_opt &
ROS_NAMESPACE=/debug/stereo_camera/right rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color_opt &
ROS_NAMESPACE=/debug/bottomcam/camera rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color_opt &

