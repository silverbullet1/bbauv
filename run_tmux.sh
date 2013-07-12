#!/bin/sh
tmux start-server
tmux new-session -d -s Pool
#tmux new-window -t Pool:2
#tmux rename-window 'ROSCORE'
#tmux send-keys 'roscore' C-m
tmux new-window -t Pool:1
tmux rename-window 'KILL TMUX'
tmux send-keys 'tmux kill-server' 
tmux select-window -t Pool:0
tmux rename-window 'Acoustic'
tmux send-keys 'roscd acoustic_nav' C-m 
tmux send-keys 'clear' C-m 
tmux set-option -s mouse-resize-pane on
tmux set-option -s mouse-select-pane on
tmux set-option -s pane-border-bg yellow
tmux attach-session -d -tPool

