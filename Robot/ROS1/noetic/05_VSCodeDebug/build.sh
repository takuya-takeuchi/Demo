#!/bin/bash +x

SCRIPT_DIR=$(cd $(dirname $0); pwd)

pushd ${SCRIPT_DIR}/catkin_ws
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug
popd