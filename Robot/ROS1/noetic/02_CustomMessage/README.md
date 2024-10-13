# Custom message

## Abstracts

* Define custom message pacakge and reference it from other project 

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
~/Work/Demo/Robot/ROS1/noetic/02_CustomMessage/catkin_ws ~/Work/Demo/Robot/ROS1/noetic/02_CustomMessage
Initialized new catkin workspace in `/home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/02_CustomMessage/catkin_ws`
------------------------------------------------------------------------------------------------------------
Profile:                     default
Extending:             [env] /opt/ros/noetic
Workspace:                   /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/02_CustomMessage/catkin_ws
------------------------------------------------------------------------------------------------------------
Build Space:        [exists] /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/02_CustomMessage/catkin_ws/build
Devel Space:        [exists] /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/02_CustomMessage/catkin_ws/devel
Install Space:      [unused] /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/02_CustomMessage/catkin_ws/install
Log Space:         [missing] /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/02_CustomMessage/catkin_ws/logs
Source Space:       [exists] /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/02_CustomMessage/catkin_ws/src
DESTDIR:            [unused] None
------------------------------------------------------------------------------------------------------------
Devel Space Layout:          linked
Install Space Layout:        None
------------------------------------------------------------------------------------------------------------
Additional CMake Args:       None
Additional Make Args:        None
Additional catkin Make Args: None
Internal Make Job Server:    True
Cache Job Environments:      False
------------------------------------------------------------------------------------------------------------
Buildlisted Packages:        None
Skiplisted Packages:         None
------------------------------------------------------------------------------------------------------------
Workspace configuration appears valid.

NOTE: Forcing CMake to run for each package.
------------------------------------------------------------------------------------------------------------
[build] Found 2 packages in 0.0 seconds.                                                                                                                                               
[build] Updating package table.                                                                                                                                                        
Starting  >>> catkin_tools_prebuild                                                                                                                                                    
Finished  <<< catkin_tools_prebuild                [ 2.2 seconds ]                                                                                                                     
Starting  >>> sample_msgs                                                                                                                                                              
Finished  <<< sample_msgs                          [ 3.5 seconds ]                                                                                                                     
Starting  >>> server                                                                                                                                                                   
Finished  <<< server                               [ 5.9 seconds ]                                                                                                                     
[build] Summary: All 3 packages succeeded!                                                                                                                                             
[build]   Ignored:   None.                                                                                                                                                             
[build]   Warnings:  None.                                                                                                                                                             
[build]   Abandoned: None.                                                                                                                                                             
[build]   Failed:    None.                                                                                                                                                             
[build] Runtime: 11.6 seconds total.                                                                                                                                                   
[build] Note: Workspace packages have changed, please re-source setup files to use them.
~/Work/Demo/Robot/ROS1/noetic/02_CustomMessage
````

## How to run?

````bash
$ ./run.sh 
... logging to /home/xxxxxxxxxx/.ros/log/74194ecc-8595-11ef-9cfd-c753b2340ae8/roslaunch-esxi-vm03-1492734.log
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
~/Work/Demo/Robot/ROS1/noetic/02_CustomMessage/catkin_ws ~/Work/Demo/Robot/ROS1/noetic/02_CustomMessage
[rosrun] Couldn't find executable named server below /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/02_CustomMessage/catkin_ws/src/server
[rosrun] Found the following, but they're either not files,
[rosrun] or not executable:
[rosrun]   /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/02_CustomMessage/catkin_ws/src/server
[rosrun]   /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/02_CustomMessage/catkin_ws/src/server/include/server
~/Work/Demo/Robot/ROS1/noetic/02_CustomMessage
````
