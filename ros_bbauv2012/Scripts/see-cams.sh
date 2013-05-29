#!/bin/bash -i

rosrun image_view image_view _image_transport:=compressed image:=/debug/stereo_camera/left/image_color &
rosrun image_view image_view _image_transport:=compressed image:=/debug/stereo_camera/right/image_color &
rosrun image_view image_view _image_transport:=compressed image:=/debug/bottomcam/camera/image_color &
