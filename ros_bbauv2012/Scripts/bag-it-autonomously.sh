#!/bin/bash

# Bag everything from the compressed cameras
cd /media/sda1/AutonomousRuns
rosbag record -b 0 -o autonom-runclear /Vision/image_filter /bottomcam/camera/camera_info /bottomcam/camera/image_raw/compressed{,/parameter_descriptions,/parameter_updates} /stereo_camera/{left,right}/camera_info /stereo_camera/{left,right}/image_raw/compressed{,/parameter_descriptions,/parameter_updates} /depth /euler /AHRS8_data_e /AHRS8_data_q /LocomotionServer/feedback /controller_points /thruster_speed /euler /WH_DVL_data /earth_odom /cmd_vel /cmd_position /openups /hull_status /rosout
