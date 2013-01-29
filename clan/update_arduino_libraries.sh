#!/bin/sh

rosrun rosserial_client make_library.py . bbauv_msgs

sudo cp -r ros_lib/bbauv_msgs/ /usr/share/arduino/libraries/
rm -r ros_lib
