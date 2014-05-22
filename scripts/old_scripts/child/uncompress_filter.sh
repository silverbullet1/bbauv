#!/bin/bash

pkill -f 'republish.*filter'
ROS_NAMESPACE=/Vision rosrun image_transport republish compressed in:=image_filter raw out:=image_filter_opt &

