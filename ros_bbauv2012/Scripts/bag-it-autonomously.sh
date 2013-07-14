#!/bin/bash

# Bag everything from the compressed cameras
cd /media/sda1/AutonomousRuns

rosbag record --split --size=10240 -b 0 -o big-autonom /hydrophone/time_diff /Vision/image_filter/compressed /bottomcam/camera/camera_info /bottomcam/camera/image_rect_color /stereo_camera/{left,right}/camera_info /stereo_camera/{left,right}/image_rect_color /depth /altitude /euler /AHRS8_data_e /AHRS8_data_q /LocomotionServer/feedback /controller_points /thruster_speed /WH_DVL_data /earth_odom /cmd_vel /cmd_position /openups_stats /hull_status /rosout & 

rosrun mission_planner v4_mission_planner.py
