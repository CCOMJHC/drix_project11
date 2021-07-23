#!/bin/bash

DAY=$(date "+%Y-%m-%d")
LOGDIR="/home/field/project11/log/${DAY}"
mkdir -p "$LOGDIR"

export ROS_MASTER_URI=http://192.168.0.201:11311
export ROS_IP=$ROBOBOX_ROS_IP 

tmux new -d -s robobox
tmux send-keys "rosrun rosmon rosmon --name=rosmon_p11_robobox drix_project11 robobox_interface.launch logDirectory:=${LOGDIR}" C-m

tmux a -t robobox
