#!/bin/bash

DAY=$(date "+%Y-%m-%d")
NOW=$(date "+%Y-%m-%dT%H.%M.%S.%N")
LOGDIR="/home/field/project11/log/${DAY}"
mkdir -p "$LOGDIR"
LOG_FILE="${LOGDIR}/autostart_${NOW}.txt"

{

echo ""
echo "#############################################"
echo "Running autostart_mystique.bash"
date
echo "#############################################"
echo ""
tmux new -d -s robobox
tmux send-keys -t robobox "source /home/field/.bashrc" C-m
tmux send-keys -t robobox "source /home/field/project11/catkin_ws/src/drix_project11/scripts/robobox_as_core.bash" C-m
tmux send-keys -t robobox "rosrun rosmon rosmon --name=rosmon_p11_robobox drix_project11 robobox_interface.launch logDirectory:=${LOGDIR}" C-m

#tmux a -t robobox

} >> "${LOG_FILE}" 2>&1
