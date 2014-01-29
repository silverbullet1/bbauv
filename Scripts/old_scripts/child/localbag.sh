#!/bin/sh
echo "Launching Local BBAUV Development..."
#cd ~/fuerte_workspace/bbauv/ros_bbauv2012/
#tmux start-server
tmux new-session -d -s Poolbag
tmux rename-window 'reconfigure'
tmux split-window -h
tmux send-keys 'rosrun dynamic_reconfigure reconfigure_gui' C-m
tmux select-pane -L
#tmux send-keys 'rosrun joy joy_node' C-m
#tmux new-window -t Pool:3
#tmux rename-window 'rqt_gui'
#tmux send-keys 'rosrun rqt_gui rqt_gui' C-m
tmux new-window -t Poolbag:0
tmux rename-window 'image_view'
tmux send-keys 'rosrun rqt_image_view rqt_image_view' C-m
tmux new-window -t Poolbag:1
tmux rename-window 'rxconsole'
tmux send-keys 'rxconsole' C-m
tmux new-window -t Poolbag:2
tmux rename-window 'auv_gui'
#tmux send-keys 'cd ~/fuerte_workspace/bbauv/ros_bbauv2012/Topside/auv_gui/ && ./debug_uncompress.sh' C-m
tmux send-keys 'cd ~/fuerte_workspace/bbauv/ros_bbauv2012/Topside/auv_gui/ && rosrun auv_gui auv_gui.py' C-m
tmux new-window -tPoolbag:3
tmux rename-window 'eclipse'
tmux send-keys 'roseclipse' C-m
tmux split-window -h 
tmux new-window -tPoolbag:4
tmux rename-window 'scripts'
tmux split-window -h
tmux split-window -v
tmux select-window -tPoolbag:0

tmux set-option -s mouse-resize-pane on
tmux set-option -s mouse-select-pane on
tmux set-option -s pane-border-bg yellow
tmux attach-session -d -tPoolbag

