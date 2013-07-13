#!/bin/bash

# Bag everything from the compressed cameras
cd /media/sda1/AutonomousRuns

rosbag record --split --size=10240 -b 0 -o small-autonom /hydrophone/time_diff /Vision/image_filter/compressed /debug/stereo_camera/left/image_rect_color /debug/bottomcam/camera/image_rect_color /depth /altitude /euler /LocomotionServer/feedback /controller_points /WH_DVL_data /earth_odom /cmd_vel /cmd_position /openups_stats /hull_status /rosout &

rosrun mission_planner v4_mission_planner.py

