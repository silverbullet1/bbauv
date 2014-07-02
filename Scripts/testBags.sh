#!/bin/bash

roscore &
#roslaunch launch uncompressbags.launch &
./lynnette_repub.sh &
rosrun controls PID_Controller &
rqt &
