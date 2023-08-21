#!/bin/bash

DAY=$(date "+%Y-%m-%d")
NOW=$(date "+%Y-%m-%dT%H.%M.%S.%N")
LOGDIR="/home/field/project11/logs/${DAY}"
mkdir -p "$LOGDIR"
LOG_FILE="${LOGDIR}/autostart_robobox_interface_${NOW}.txt"

{

printenv
set -x

echo ""
echo "#############################################"
echo "Starting robobox interface"
date
echo "#############################################"
echo ""

source /opt/ros/noetic/setup.bash
source /home/field/project11/catkin_ws/devel/setup.bash

set -v

export ROS_MASTER_URI=http://robobox:11311
export ROS_IP=$ROBOBOX_ROS_IP

/usr/bin/tmux new -d -s robobox
/usr/bin/tmux send-keys -t robobox "rosrun rosmon rosmon --name=rosmon_p11_robobox drix_project11 robobox_interface.launch logDirectory:=${LOGDIR}" C-m

#tmux a -t robobox

set +x

} >> "${LOG_FILE}" 2>&1
