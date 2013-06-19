#!/bin/sh
echo "Launching BBAUV Local System Diagnostics..."
sleep 4
#cd ~/fuerte_workspace/bbauv/ros_bbauv2012/
tmux start-server
tmux new-session -d -s Pool
tmux rename-window 'Control diagnostics'
tmux new-window -tPool:1
tmux rename-window 'System diagnostics'
tmux new-window -tPool:2
tmux rename-window 'reconfigure-joy'
tmux split-window -h
tmux send-keys 'rosrun dynamic_reconfigure reconfigure_gui' C-m
tmux select-pane -L
tmux send-keys 'rosrun joy joy_node' C-m
#tmux new-window -t Pool:3
#tmux rename-window 'rqt_gui'
#tmux send-keys 'rosrun rqt_gui rqt_gui' C-m
tmux new-window -t Pool:3
tmux rename-window 'image_view'
tmux send-keys 'sh child/see-cams.sh' C-m
tmux new-window -t Pool:4
tmux rename-window 'rxconsole'
tmux send-keys 'rxconsole' C-m
tmux new-window -t Pool:5
tmux rename-window 'filezilla'
tmux send-keys 'filezilla &' C-m
tmux new-window -t Pool:7
tmux rename-window 'auv_gui'
tmux rosrun auv_gui auv_gui.py
tmux new-window -t Pool:8
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
tmux select-pane -t5
tmux split-window -h
tmux select-pane -D
tmux split-window -h
tmux select-pane -t0
tmux send-keys 'rostopic echo /AHRS8_data_e' C-m
tmux select-pane -t1
tmux send-keys 'rostopic echo /depth' C-m
tmux select-pane -t2
tmux send-keys 'rostopic echo /WH_DVL_data/twist/twist/linear' C-m
tmux select-pane -t3
tmux send-keys 'rostopic echo /WH_DVL_data/pose/pose/position' C-m
tmux select-pane -t4
tmux send-keys 'rostopic echo /controller_points' C-m
tmux select-pane -t6
tmux send-keys 'rostopic echo /LocomotionServer/feedback' C-m
tmux select-pane -t5
tmux send-keys 'rostopic echo /thruster_speed' C-m
tmux select-pane -t7
tmux send-keys 'rostopic echo /teleop_controller' C-m

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
tmux select-pane -t2
tmux send-keys 'rostopic echo /manipulators' C-m
tmux select-window -tPool:0

tmux set-option -s mouse-resize-pane on
tmux set-option -s mouse-select-pane on
tmux set-option -s pane-border-bg yellow
tmux attach-session -d -tPool

