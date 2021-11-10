#!/bin/bash

DAY=$(date "+%Y-%m-%d")
LOGDIR="/home/field/project11/log/${DAY}"
mkdir -p "$LOGDIR"

source /home/field/.bashrc

tmux new -d -s project11
tmux send-keys -t project11 "source /home/field/.bashrc" C-m
tmux send-keys -t project11 "source /home/field/project11/catkin_ws/src/drix_project11/scripts/robobox_as_core.bash" C-m
tmux send-keys -t project11 "rosrun rosmon rosmon --name=rosmon_p11_operator_core drix_project11 operator_core.launch drixNumber:=8 logDirectory:=${LOGDIR}" C-m
tmux a -t project11
