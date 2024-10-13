#!/bin/bash +x

SCRIPT_DIR=$(cd $(dirname $0); pwd)

pushd ${SCRIPT_DIR}/catkin_ws
source devel/setup.bash

PACAKGE=subscriber
LAUNCH_FILE=subscriber.launch
roslaunch ${PACAKGE} ${LAUNCH_FILE}
popd