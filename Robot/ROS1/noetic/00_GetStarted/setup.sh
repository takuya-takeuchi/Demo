#!/bin/bash +x

SCRIPT_DIR=$(cd $(dirname $0); pwd)

mkdir -p ${SCRIPT_DIR}/catkin_ws/src

pushd ${SCRIPT_DIR}/catkin_ws/src
catkin_create_pkg hello std_msgs roscpp
popd

pushd ${SCRIPT_DIR}/catkin_ws
catkin build
popd