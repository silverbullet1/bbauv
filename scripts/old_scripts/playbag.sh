#!/bin/bash

ROS_NAMESPACE=/stereo_camera/left rosrun image_transport republish compressed in:=image_raw raw out:=image_raw &
ROS_NAMESPACE=/stereo_camera/right rosrun image_transport republish compressed in:=image_raw raw out:=image_raw &
ROS_NAMESPACE=/bottomcam/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_raw &

ROS_NAMESPACE=stereo_camera rosrun stereo_image_proc stereo_image_proc _camera_info_url_left:=file:///home/gohew/fuerte_workspace/bbauv/ros_bbauv2012/launch/bumblebee_left.yaml _camera_info_url_right:=file:///home/gohew/fuerte_workspace/bbauv/ros_bbauv2012/launch/bumblebee_right.launch &
ROS_NAMESPACE=bottomcam/camera rosrun image_proc image_proc _camera_info_url:=file:///home/gohew/fuerte_workspace/bbauv/ros_bbauv2012/launch/firefly.yaml &

rxbag "$1"
