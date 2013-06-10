#!/bin/bash

# Bag everything from the compressed cameras
cd /media/0052-4D54
rosbag record -b 0 -o vid-sensors --split --size=3076 /bottomcam/camera/camera_info /bottomcam/camera/image_raw/compressed{,/parameter_descriptions,/parameter_updates} /stereo_camera/{left,right}/camera_info /stereo_camera/{left,right}/image_raw/compressed{,/parameter_descriptions,/parameter_updates} /AHRS8_data_{e,q} /WH_DVL_data /WH_DVL/parameter_{descriptions,updates} /depth /euler
