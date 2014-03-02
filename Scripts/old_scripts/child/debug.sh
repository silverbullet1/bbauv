#!/bin/sh
echo "Launching BBAUV Debug ..."
export PROFILE=debug
sleep 4
#cd ~/fuerte_workspace/bbauv/ros_bbauv2012/
tmux start-server
tmux new-session -d -s debug
tmux rename-window 'Debug window'
tmux new-window -tdebug:1
tmux rename-window 'System diagnostics'
tmux new-window -tdebug:2
tmux rename-window 'reconfigure'
tmux split-window -h
tmux send-keys 'rosdebug && rosrun dynamic_reconfigure reconfigure_gui' C-m
tmux select-pane -L
#tmux send-keys 'rosrun joy joy_node' C-m
#tmux new-window -t Pool:3
#tmux rename-window 'rqt_gui'
#tmux send-keys 'rosrun rqt_gui rqt_gui' C-m
tmux new-window -t Pool:3
tmux rename-window 'arduino'
tmux send-keys 'rosdebug && cd ~/sketchbook/uploader' C-m
tmux split-window -h
tmux new-window -t Pool:4
tmux rename-window 'rqt_console'
tmux send-keys 'rosdebug && rqt_console' C-m
tmux new-window -t Pool:5
tmux rename-window 'filezilla'
tmux send-keys 'filezilla &' C-m
tmux  new-window -t Pool:7
tmux rename-window 'control panel'
tmux send-keys 'cd ~/fuerte _workspace/bbauv/ros_bbauv2012/Topside/auv_gui/ && ./debug_uncompress.sh' C-m
tmux split-window -h
tmux send-keys 'cd ~/fuerte_workspace/bbauv/ros_bbauv2012/Topside/auv_gui/ && rosrun auv_gui auv_gui.py' C-m
tmux new-window -t Pool:8
tmux rename-window 'scripts'
tmux select-window -tPool:0
tmux split-window -h
tmux select-pane -L
tmux split-window -h
tmux split-window -v
tmux select-pane -L
tmux split-window -v
tmux select-pane -t0

tmux set-option -s mouse-resize-pane on
tmux set-option -s mouse-select-pane on
tmux set-option -s pane-border-bg yellow
tmux attach-session -d -tdebug

