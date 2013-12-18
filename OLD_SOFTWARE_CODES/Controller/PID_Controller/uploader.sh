#!/bin/bash
echo "Transferring code now..."
sftp bbauvsbc1@bbauv <<EOF
put /home/gohew/fuerte_workspace/bbauv/ros_bbauv2012/Controller/PID_Controller/src/PID_Controller.cpp /home/bbauvsbc1/bbauv_workspace/bbauv/ros_bbauv2012/Controller/PID_Controller/src/PID_Controller.cpp
put /home/gohew/fuerte_workspace/bbauv/ros_bbauv2012/Controller/PID_Controller/cfg/PID_Controller.cfg /home/bbauvsbc1/bbauv_workspace/bbauv/ros_bbauv2012/Controller/PID_Controller/cfg/PID_Controller.cfg
bye
EOF
ssh bbauvsbc1@bbauv "bash -i -c 'rosmake PID_Controller;roslaunch launch controller.launch'"
