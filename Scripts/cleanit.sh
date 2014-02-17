#!/bin/bash

NAME="bb"
MISSION=0

while getopts ":hn:" opt; do
  case $opt in
    n)
      NAME+="-$OPTARG"
      ;;
    h)
      echo "usage: bagit [-n name]"
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
cd ~/bags

rosbag record --split --duration=600 -b 0 -o $NAME /bot_camera/camera/camera_info /bot_camera/camera/image_raw/compressed /front_camera/camera/camera_info /front_camera/camera/image_raw/compressed /depth /altitude /euler /AHRS8_data_e /AHRS8_data_q /LocomotionServer/feedback /controller_points /thruster_speed /odom /task_visualization /WH_DVL_data /map /pressure_data /hull_status /manipulators /rosout
