#!/bin/bash +x

SCRIPT_DIR=$(cd $(dirname $0); pwd)

# start roscore on background
PORT=11311
roscore -p ${PORT} &
# wait roscore launch
sleep 5

pushd ${SCRIPT_DIR}/catkin_ws
source devel/setup.bash
PACAKGE=subscriber
NODE=subscriber.py
rosrun ${PACAKGE} ${NODE}
popd