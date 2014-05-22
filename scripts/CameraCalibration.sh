#!/bin/bash

# Grid setup of the internal points of the chessboard
INTERNAL_PTS=13x7
# Width of each square (in metres)
SQUARE_WIDTH=0.017

rosrun camera_calibration cameracalibrator.py --size $INTERNAL_PTS --square $SQUARE_WIDTH image:=/camera/image_raw camera:=/camera
