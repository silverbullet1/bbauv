#!/bin/bash

ROS_NAMESPACE=/Vision rosrun image_transport republish raw in:=image_filter compressed out:=image_filter &

ROS_NAMESPACE=/ rosrun image_transport republish raw in:=sonar_image compressed out:=sonar_image &
