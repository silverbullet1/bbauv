#!/bin/sh
echo "Launching BBAUV Local System Diagnostics..."
sleep 4
#cd ~/fuerte_workspace/bbauv/ros_bbauv2012/
tmux start-server
tmux new-session -d -s Pool
tmux rename-window 'Control diagnostics'
tmux new-window -tPool:1
tmux rename-window 'System diagnostics'
sleep 2
tmux new-window -tPool:2
tmux rename-window 'reconfigure-joy'
tmux split-window -h
tmux send-keys 'rosrun dynamic_reconfigure reconfigure_gui' C-m
tmux select-pane -L
tmux send-keys 'rosrun joy joy_node' C-m
tmux new-window -t Pool:3
tmux rename-window 'rqt_gui'
tmux send-keys 'rosrun rqt_gui rqt_gui' C-m
tmux new-window -t Pool:4
tmux rename-window 'scripts'
tmux select-window -tPool:0
tmux split-window -h
tmux split-window -h
tmux select-pane -L
tmux select-pane -L
tmux split-window -h
tmux split-window -v
tmux select-pane -L
tmux split-window -v
tmux select-pane -R
tmux select-pane -R
tmux split-window -v
tmux select-pane -R
tmux split-window -v

tmux select-pane -t0
tmux send-keys 'rostopic echo /euler' C-m
tmux select-pane -t1
tmux send-keys 'rostopic echo /WH_DVL_data/twist/twist/linear' C-m
tmux select-pane -t2
tmux send-keys 'rostopic echo /depth' C-m
tmux select-pane -t3
tmux send-keys 'rostopic echo /thruster_speed' C-m
tmux select-pane -t4
tmux send-keys 'rostopic echo /teleop_controller' C-m
tmux select-pane -t5
tmux send-keys 'rostopic echo /manipulators' C-m
tmux select-window -tPool:1
tmux split-window -h
tmux split-window -h
tmux select-pane -L
tmux select-pane -L
tmux split-window -h
tmux split-window -v
tmux select-pane -L
tmux split-window -v
tmux select-pane -R
tmux select-pane -R
tmux split-window -v
tmux select-pane -R
tmux split-window -v

tmux select-pane -t0
tmux send-keys 'rostopic echo /hull_status' C-m
tmux select-pane -t1
tmux send-keys 'rostopic echo /openups' C-m
tmux select-window -tPool:0

tmux set-option -s mouse-resize-pane on
tmux set-option -s mouse-select-pane on
tmux set-option -s pane-border-bg yellow
tmux attach-session -d -tPool
