#!/bin/sh
#cd ~/fuerte_workspace/bbauv/ros_bbauv2012/
echo "Connected to Remote AUV System."
sleep 1
echo "Launching Bumblebee AUV Systems..."
sleep 5
tmux start-server
tmux new-session -d -s bbauv
tmux rename-window 'MISSION CONTROL'
tmux split-window -h
tmux new-window -tbbauv:1
tmux rename-window 'rosbag playback'
tmux split-window -h
tmux new-window -tbbauv:2
tmux rename-window 'roscore'
tmux send-keys 'roscore' C-m
sleep 3
tmux new-window -tbbauv:3
tmux rename-window 'launch'
tmux send-keys 'roslaunch launch drivers.launch' C-m
sleep 4
tmux new-window -tbbauv:4
tmux rename-window 'arduino'
tmux send-keys 'roslaunch launch embedded.launch' C-m
tmux new-window -tbbauv:5
tmux rename-window 'cameras'
#tmux send-keys 'rostopic pub -1 /lcd_commands std_msgs/Int8 -- 1' C-m
#tmux send-keys 'roslaunch vision cameras.launch' C-m
tmux send-keys 'roslaunch vision cameraswusb.launch' C-m
tmux new-window -tbbauv:6
tmux rename-window 'batteries'
tmux send-keys 'roslaunch launch battery.launch' C-m
tmux new-window -tbbauv:7
tmux rename-window 'dvl'
tmux send-keys 'roslaunch launch dvl.launch' C-m

tmux set-option -s mouse-resize-pane on
tmux set-option -s mouse-select-pane on
tmux set-option -s pane-border-fg blue
tmux attach-session -d -tbbauv
