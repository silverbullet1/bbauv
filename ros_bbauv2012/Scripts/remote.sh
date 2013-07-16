#!/bin/sh
gnome-terminal --tab --title="tmux_LOCAL" -e "sh child/local.sh" --tab --title="tmux_BBAUV" -e "sh child/remote_ssh.sh"