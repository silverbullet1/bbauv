#!/bin/bash

NAME="bb"
MISSION=0

while getopts ":mhn:" opt; do
  case $opt in
    n)
      NAME+="-$OPTARG"
      ;;
    m)
      MISSION=1
      ;;
    h)
      echo "usage: bagit [-m] [-n name]"
      exit 1
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done
# Bag everything from the compressed cameras
cd /media/sda1/AutonomousRuns

rosbag record --split --duration=600 -b 0 -o $NAME /hydrophone/time_diff /Vision/image_filter/compressed /bottomcam/camera/camera_info /bottomcam/camera/image_rect_color/compressed /stereo_camera/{left,right}/camera_info /stereo_camera/{left,right}/image_rect_color/compressed /depth /altitude /euler /AHRS8_data_e /AHRS8_data_q /LocomotionServer/feedback /controller_points /thruster_speed /odom /task_visualization /WH_DVL_data /earth_odom /tf /map /openups_stats /pressure_data /hull_status /manipulators /rosout &

if [ $MISSION -eq 1 ]
then
rosrun mission_planner v4_mission_planner.py
fi

