#!/bin/bash +x

SCRIPT_DIR=$(cd $(dirname $0); pwd)

mkdir -p ${SCRIPT_DIR}/catkin_ws/src
cd ${SCRIPT_DIR}/catkin_ws
catkin build

# echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
# source ~/.bashrc