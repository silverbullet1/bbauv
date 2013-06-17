#!/bin/bash

# Bag everything from the compressed cameras
cd /media/0052-4D54/AutonomousRuns
rosbag record -b 0 -o autonom-runclear --split --size=3076 /Vision/image_filter /bottomcam/camera/camera_info /bottomcam/camera/image_raw/compressed{,/parameter_descriptions,/parameter_updates} /stereo_camera/{left,right}/camera_info /stereo_camera/{left,right}/image_raw/compressed{,/parameter_descriptions,/parameter_updates} /depth /AHRS8_data_e /AHRS8_data_q /LocomotionServer/feedback /controller_points /thruster_speed /euler /WH_DVL_data/twist/twist/linear /WH_DVL_data/pose/pose/position /earth_odom/twist/twist/linear /earth_odom/pose/pose/position /cmd_vel /cmd_position /openups /hull_status /rosout
