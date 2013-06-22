#!/bin/sh
#cd ~/fuerte_workspace/bbauv/ros_bbauv2012/
echo "Connected to Remote AUV System."
sleep 1
echo "Launching Bumblebee AUV Systems..."
tmux start-server
tmux new-session -d -s bbauv
tmux rename-window 'Scripts'
tmux new-window -tbbauv:1
tmux rename-window 'Scripts 2'
tmux new-window -tbbauv:2
tmux rename-window 'roscore'
tmux send-keys 'roscore' C-m
sleep 3
tmux new-window -tbbauv:3
tmux rename-window 'launch'
tmux send-keys 'pls' C-m
sleep 2
tmux new-window -tbbauv:4
tmux rename-window 'boot'
tmux send-keys 'rostopic pub -1 /lcd_commands std_msgs/Int8 -- 1' C-m
tmux set-option -s mouse-resize-pane on
tmux set-option -s mouse-select-pane on
tmux set-option -s pane-border-fg blue
tmux attach-session -d -tbbauv
