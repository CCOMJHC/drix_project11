#!/bin/bash

DAY=$(date "+%Y-%m-%d")
NOW=$(date "+%Y-%m-%dT%H.%M.%S.%N")
LOGDIR="/home/field/project11/logs"
mkdir -p "$LOGDIR"
LOG_FILE="${LOGDIR}/autostart_drixcloud_${NOW}.txt"

{

set -x

echo ""
echo "#############################################"
echo "Starting operator drixcloud"
date
echo "#############################################"
echo ""

sleep 1

/usr/bin/tmux new -d -s project11
/usr/bin/tmux send-keys -t project11 "rosrun rosmon rosmon --name=rosmon_p11_operator_drixcloud drix_project11 drixcloud.launch drixNumber:=8" C-m

sleep 1

/usr/bin/tmux new -d -s roscore
/usr/bin/tmux send-keys -t roscore "source /home/field/.bashrc" C-m
/usr/bin/tmux send-keys -t roscore "roscore" C-m

sleep 1

/usr/bin/tigervncserver -nolisten tcp -localhost no  :1

sleep 1

/usr/bin/tigervncserver -nolisten tcp -localhost no  :2

set +x

} >> "${LOG_FILE}" 2>&1
ln -s -f ${LOG_FILE} ${LOGDIR}/autostart_drixcloud_latest.txt
