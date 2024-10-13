# Pub&Sub (Python)

## Abstracts

* Publish and subscribe custom message
* Use Python 3 instead of Python 2

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
~/Work/Demo/Robot/ROS1/noetic/04_PubSubPython/catkin_ws ~/Work/Demo/Robot/ROS1/noetic/04_PubSubPython
-----------------------------------------------------------------------------------------------------------
Profile:                     default
Extending:          [cached] /opt/ros/noetic
Workspace:                   /home/xxxxxxxxx/Work/Demo/Robot/ROS1/noetic/04_PubSubPython/catkin_ws
-----------------------------------------------------------------------------------------------------------
Build Space:        [exists] /home/xxxxxxxxx/Work/Demo/Robot/ROS1/noetic/04_PubSubPython/catkin_ws/build
Devel Space:        [exists] /home/xxxxxxxxx/Work/Demo/Robot/ROS1/noetic/04_PubSubPython/catkin_ws/devel
Install Space:      [unused] /home/xxxxxxxxx/Work/Demo/Robot/ROS1/noetic/04_PubSubPython/catkin_ws/install
Log Space:          [exists] /home/xxxxxxxxx/Work/Demo/Robot/ROS1/noetic/04_PubSubPython/catkin_ws/logs
Source Space:       [exists] /home/xxxxxxxxx/Work/Demo/Robot/ROS1/noetic/04_PubSubPython/catkin_ws/src
DESTDIR:            [unused] None
-----------------------------------------------------------------------------------------------------------
Devel Space Layout:          linked
Install Space Layout:        None
-----------------------------------------------------------------------------------------------------------
Additional CMake Args:       None
Additional Make Args:        None
Additional catkin Make Args: None
Internal Make Job Server:    True
Cache Job Environments:      False
-----------------------------------------------------------------------------------------------------------
Buildlisted Packages:        None
Skiplisted Packages:         None
-----------------------------------------------------------------------------------------------------------
Workspace configuration appears valid.
-----------------------------------------------------------------------------------------------------------
[build] Found 3 packages in 0.0 seconds.                                                                                                                                               
[build] Package table is up to date.                                                                                                                                                   
Starting  >>> sample_msgs                                                                                                                                                              
Finished  <<< sample_msgs                [ 0.4 seconds ]                                                                                                                               
Starting  >>> publisher                                                                                                                                                                
Starting  >>> subscriber                                                                                                                                                               
Finished  <<< publisher                  [ 0.2 seconds ]                                                                                                                               
Finished  <<< subscriber                 [ 0.2 seconds ]                                                                                                                               
[build] Summary: All 3 packages succeeded!                                                                                                                                             
[build]   Ignored:   None.                                                                                                                                                             
[build]   Warnings:  None.                                                                                                                                                             
[build]   Abandoned: None.                                                                                                                                                             
[build]   Failed:    None.                                                                                                                                                             
[build] Runtime: 0.8 seconds total.                                                                                                                                                    
~/Work/Demo/Robot/ROS1/noetic/04_PubSubPython
````

## How to run?

````bash
$ ./run_subscriber.sh 
... logging to /home/xxxxxxxxx/.ros/log/29051768-8987-11ef-9cfd-c753b2340ae8/roslaunch-esxi-vm03-1970891.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://esxi-vm03:38075/
ros_comm version 1.17.0


SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.17.0

NODES

auto-starting new master
process[master]: started with pid [1970903]
ROS_MASTER_URI=http://esxi-vm03:11311/

setting /run_id to 29051768-8987-11ef-9cfd-c753b2340ae8
process[rosout-1]: started with pid [1970913]
started core service [/rosout]
~/Work/Demo/Robot/ROS1/noetic/04_PubSubPython/catkin_ws ~/Work/Demo/Robot/ROS1/noetic/04_PubSubPython
````

After launch subscriber, kick next script to launch publisher on other shell.

````bash
$ ./run_publisher.sh 
~/Work/Demo/Robot/ROS1/noetic/04_PubSubPython/catkin_ws ~/Work/Demo/Robot/ROS1/noetic/04_PubSubPython
[INFO] [1728840057.049207]: Conection started...
[INFO] [1728840057.050261]: publish: name: test, id: 1234
[INFO] [1728840057.149627]: publish: name: test, id: 1234
````

Then, subscriber recerives topics on previous shell.

````shell
[INFO] [1728840058.450166]: subscribe: name: test, id: 1234
[INFO] [1728840058.550177]: subscribe: name: test, id: 1234
````
