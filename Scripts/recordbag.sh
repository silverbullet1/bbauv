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
      echo "usage: recordbag [-m] [-n name]"
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

cd ~/bags

echo $NAME
rosbag record -O $NAME /Vision/image_filter/compressed /bot_camera/camera/image_raw/compressed /front_camera/camera/image_raw/compressed /depth /altitude /euler /AHRS8_data_e /AHRS8_data_q /LocomotionServer/feedback /controller_points /thruster_speed /WH_DVL_data /earth_odom /pressure_data /hull_status /manipulators

#rosbag record -O $NAME /Vision/image_filter/compressed

