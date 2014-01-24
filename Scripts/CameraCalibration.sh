#!/bin/bash

# Grid setup of the internal points of the chessboard
INTERNAL_PTS=9x6
# Width of each square (in metres)
SQUARE_WIDTH=0.024

rosrun camera_calibration cameracalibrator.py --size $INTERNAL_PTS --square $SQUARE_WIDTH image:=bot_camera/camera/image_raw camera:=/bot_camera/camera
