#!/bin/sh
echo "Launching BBAUV Local System Diagnostics..."
sleep 4
#cd ~/fuerte_workspace/bbauv/ros_bbauv2012/
tmux start-server
tmux new-session -d -s Pool
tmux rename-window 'terminals'
tmux new-window -tPool:1
tmux rename-window 'Control node'
tmux send-keys 'rosrun gui Control_node' C-m 
tmux new-window -tPool:2
tmux rename-window 'rqt'
tmux send-keys 'rqt' C-m
tmux new-window -t Pool:3
tmux rename-window 'arduino'
tmux send-keys 'cd ~/sketchbook/uploader' C-m
tmux split-window -h
tmux new-window -t Pool:4
tmux rename-window 'eclipse'
tmux send-keys 'roseclipse' C-m
tmux split-window -h
tmux new-window -t Pool:5
tmux rename-window 'filezilla'
tmux send-keys 'filezilla &' C-m
tmux new-window -t Pool:7
tmux rename-window 'auv'
tmux send-keys 'auv' C-m
tmux new-window -t Pool:8
tmux rename-window 'scripts'
#tmux send-keys 'sshfs -o idmap=user -o gid=`id --group` bbauvsbc1@bbauv:bbauv_workspace/bbauv/ros_bbauv2012/Logic/Vision/nodes /home/gohew/fuerte_workspace/bbauv/ros_bbauv2012/Logic/Vision/mnt' C-m
tmux select-window -tPool:0
tmux split-window -h
tmux select-pane -L
tmux split-window -v
tmux select-pane -t2
tmux split-window -v
tmux select-window -tPool:1
tmux split-window -h

tmux select-pane -t0
#tmux send-keys 'rostopic echo /hull_status' C-m
tmux select-pane -t1
#tmux send-keys 'rostopic echo /openups' C-m
tmux select-pane -t2
#tmux send-keys 'rostopic echo /manipulators' C-m
tmux select-window -tPool:0

tmux set-option -s mouse-resize-pane on
tmux set-option -s mouse-select-pane on
tmux set-option -s pane-border-bg yellow
tmux attach-session -d -tPool

