#!/bin/bash

DAY=$(date "+%Y-%m-%d")
LOGDIR="/home/field/project11/logs"
mkdir -p "$LOGDIR"

source /home/field/.bashrc

tmux new -d -s project11_ui
tmux send-keys -t project11_ui "source /home/field/.bashrc" C-m
tmux send-keys -t project11_ui "source /home/field/project11/catkin_ws/src/drix_project11/scripts/robobox_as_core.bash" C-m
tmux send-keys -t project11_ui "rosrun rosmon rosmon --name=rosmon_p11_operator_ui drix_project11 operator_ui.launch dual_camp:=true drixNumber:=8" C-m
tmux a -t project11_ui
