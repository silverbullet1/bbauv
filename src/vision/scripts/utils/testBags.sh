#!/bin/bash

roslaunch launch uncompressbags.launch &
rosrun controls PID_Controller &
rqt &
