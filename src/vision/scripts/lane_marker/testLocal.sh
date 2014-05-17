#!/bin/bash

rosrun vision run.py lane_marker.states _alone:=True _image:=/bottomcam/camera/image_rect_color
