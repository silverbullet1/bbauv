#!/bin/bash

# Bag everything from the compressed cameras
cd /media/0052-4D54/AutonomousRuns
rosbag record -b 0 -o autonom-runclear --split --size=3076 /Vision/image_filter /debug/stereo_camera/left/image_rect_color /debug/bottomcam/camera/image_rect_color /depth /AHRS8_data_e /AHRS8_data_q /LocomotionServer/feedback /controller_points /thruster_speed /WH_DVL_data/twist/twist/linear /WH_DVL_data/pose/pose/position /earth_odom/twist/twist/linear /earth_odom/pose/pose/position /cmd_vel /cmd_position /openups /hull_status /rosout
