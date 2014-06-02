#!/bin/sh
echo "Launching Local BBAUV Development..."
cd ~/bbauv
tmux start-server
tmux new-session -d -s Poolbag
tmux select-pane -L
tmux new-window -t Poolbag:0
tmux rename-window 'nodes'
tmux send-keys 'rosrun earth_odom earth_odom' C-m
tmux split-window -h
tmux send-keys 'rosrun controls PID_Controller' C-m
tmux new-window -t Poolbag:1
tmux rename-window 'rosbag'
tmux rename-window 'rosbags'
tmux split-window -h
tmux send-keys 'cd ~/bags' C-m
tmux new-window -t Poolbag:2
tmux rename-window 'rqt'
tmux send-keys 'rqt' C-m
tmux new-window -t Poolbag:3
tmux rename-window 'auv_gui'
tmux send-keys 'auv' C-m
tmux new-window -tPoolbag:4
tmux rename-window 'eclipse'
#tmux send-keys 'roseclipse' C-m
tmux split-window -h 
tmux new-window -tPoolbag:5
tmux rename-window 'scripts'
tmux split-window -h
tmux split-window -v
tmux select-window -tPoolbag:0

tmux set-option -s mouse-resize-pane on
tmux set-option -s mouse-select-pane on
tmux set-option -s pane-border-bg yellow
tmux attach-session -d -tPoolbag

