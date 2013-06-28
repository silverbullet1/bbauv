#!/bin/bash

# Bag everything from the compressed cameras
cd /media/sda1/AutonomousRuns
rosbag record -b 0 -o autonomous-small /Vision/image_filter /debug/stereo_camera/left/image_rect_color /debug/stereo_camera/right/image_rect_color /debug/bottomcam/camera/image_rect_color /depth /euler /AHRS8_data_e /AHRS8_data_q /LocomotionServer/feedback /controller_points /thruster_speed /WH_DVL_data /earth_odom /cmd_vel /cmd_position /openups /hull_status /rosout &

rosrun mission_planner v3_mission_planner.py

