#!/bin/sh
sshfs -o idmap=user -o gid=`id --group` bbauvsbc1@bbauv:bbauv_workspace/bbauv/ros_bbauv2012/Logic/Vision/nodes /home/gohew/fuerte_workspace/bbauv/ros_bbauv2012/Logic/Vision/mnt

