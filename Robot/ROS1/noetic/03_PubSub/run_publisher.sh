#!/bin/bash +x

SCRIPT_DIR=$(cd $(dirname $0); pwd)

pushd ${SCRIPT_DIR}/catkin_ws
source devel/setup.bash

PACAKGE=publisher
NODE=publisher
rosrun ${PACAKGE} ${NODE} test 1234
popd