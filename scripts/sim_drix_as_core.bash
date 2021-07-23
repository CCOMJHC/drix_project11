#!/bin/bash

if [ -z "$DRIX_NUMBER" ]; then DRIX_NUMBER=1; fi

export ROS_MASTER_URI=http://172.16.$DRIX_NUMBER.200:11311
export ROS_IP=172.16.0.1