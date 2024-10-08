# Hello, world

## Abstracts

* Setup sample project to start development

## Requirements

### Common

* ROS Noetic

## Dependencies

* [ROS](https://github.com/ros/ros)
  * Noetic
  * BSD-3-Clause license

## How to build?

````bash
$ ./build.sh
````

## How to setup?

````bash
$ ./build.sh
$ ./run.sh 
... logging to /home/t-takeuchi/.ros/log/74194ecc-8595-11ef-9cfd-c753b2340ae8/roslaunch-esxi-vm03-1492734.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://esxi-vm03:42341/
ros_comm version 1.17.0


SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.17.0

NODES

auto-starting new master
process[master]: started with pid [1492743]
ROS_MASTER_URI=http://esxi-vm03:11311/

setting /run_id to 74194ecc-8595-11ef-9cfd-c753b2340ae8
process[rosout-1]: started with pid [1492753]
started core service [/rosout]
~/Work/Demo/Robot/ROS1/noetic/01_HelloWorld/catkin_ws ~/Work/Demo/Robot/ROS1/noetic/01_HelloWorld
[ INFO] [1728406246.698506573]: Hello world
[ INFO] [1728406246.798569128]: Hello world
[ INFO] [1728406246.898631276]: Hello world
[ INFO] [1728406246.998661933]: Hello world
[ INFO] [1728406247.098625566]: Hello world
[ INFO] [1728406247.198643709]: Hello world
[ INFO] [1728406247.298661272]: Hello world
[ INFO] [1728406247.398641372]: Hello world
[ INFO] [1728406247.498618994]: Hello world
[ INFO] [1728406247.598651730]: Hello world
[ INFO] [1728406247.698660442]: Hello world
[ INFO] [1728406247.798670565]: Hello world
[ INFO] [1728406247.898639097]: Hello world
````
