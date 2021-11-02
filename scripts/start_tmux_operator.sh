#!/bin/bash

DAY=$(date "+%Y-%m-%d")
LOGDIR="/home/field/project11/log/${DAY}"
mkdir -p "$LOGDIR"

source /home/field/.bashrc

tmux new -d -s project11
tmux send-keys -t project11 "source /home/field/project11/catkin_ws/src/drix_project11/scripts/robobox_as_core.bash" C-m
tmux send-keys "rosrun rosmon rosmon --name=rosmon_operator drix_project11 operator_core.launch drixNumber:=8 logDirectory:=${LOGDIR}" C-m
tmux splitw -p 50
tmux send-keys -t project11 "source /home/field/project11/catkin_ws/src/drix_project11/scripts/robobox_as_core.bash" C-m
tmux send-keys "rosrun rosmon rosmon --name=rosmon_operator_ui project11 operator_ui.launch dual_camp:=true" C-m
tmux a -t project11
