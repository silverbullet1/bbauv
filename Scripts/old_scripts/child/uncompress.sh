#!/bin/bash

pkill -f 'republish.*color'
ROS_NAMESPACE=/front_camera/camera rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color_opt &
ROS_NAMESPACE=/bot_camera/camera rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color_opt &
#ROS_NAMESPACE=/bottomcam/camera rosrun image_transport republish compressed in:=image_rect_color raw out:=image_rect_color &
#ROS_NAMESPACE=/Vision rosrun image_transport republish compressed in:=image_filter raw out:=image_filter &

