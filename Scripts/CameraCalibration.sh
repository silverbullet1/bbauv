#!/bin/bash

# Grid setup of the internal points of the chessboard
INTERNAL_PTS=28x20
# Width of each square (in metres)
SQUARE_WIDTH=0.01

rosrun camera_calibration cameracalibrator.py --size $INTERNAL_PTS --square $SQUARE_WIDTH image:=/camera/image_raw camera:=/camera
