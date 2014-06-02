#!/bin/sh
echo "connecting to BBAUV..."
sleep 1
#ssh -X -t bbauvsbc1@bbauv "sh /home/bbauvsbc1/bbauv_workspace/bbauv/ros_bbauv2012/Scripts/child/ros.sh"
ssh -X -t bbauvsbc1@bbauv "tmux attach"
