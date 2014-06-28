#!/bin/bash

pkill -f 'republish.*lynnette'

cd ~/Code/bbauv/src/gui/controlpanel/src
ROS_NAMESPACE=/front_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_color_lynnette &
ROS_NAMESPACE=/bot_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_color_lynnette &
rosrun image_transport republish compressed in:=sonar_image raw out:=sonar_image_lynnette &
ROS_NAMESPACE=/Vision rosrun image_transport republish compressed in:=image_filter raw out:=image_filter_lynnette &

echo $1
if [[ $1 == "True" ]];
then
rosrun gui controlpanel.py _front:=/front_camera/camera/image_color_lynnette _bottom:=/bot_camera/camera/image_color_lynnette _sonar:=/sonar_image_lynnette _testing:=True _filter:=/Vision/image_filter &
else
rosrun gui controlpanel.py _front:=/front_camera/camera/image_color_lynnette _bottom:=/bot_camera/camera/image_color_lynnette _filter:=/Vision/image_filter_lynnette _sonar:=/sonar_image _lynnette &
fi
