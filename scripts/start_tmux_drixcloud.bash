#!/bin/bash

DAY=$(date "+%Y-%m-%d")
NOW=$(date "+%Y-%m-%dT%H.%M.%S.%N")
LOGDIR="/home/field/project11/logs"
mkdir -p "$LOGDIR"
LOG_FILE="${LOGDIR}/autostart_drixcloud_${NOW}.txt"

{

printenv
set -x

echo ""
echo "#############################################"
echo "Starting operator drixcloud"
date
echo "#############################################"
echo ""

while ! ping -c 1 -W 1 robocloud; do
	sleep 1
done

source /home/field/.bashrc

/usr/bin/tmux new -d -s project11
/usr/bin/tmux send-keys -t project11 "rosrun rosmon rosmon --name=rosmon_p11_operator_drixcloud drix_project11 drixcloud.launch drixNumber:=8" C-m

set +x

} >> "${LOG_FILE}" 2>&1

