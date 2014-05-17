#!/bin/bash

ROS_NAMESPACE=/Vision rosrun image_transport republish raw in:=image_filter compressed out:=image_filter
