# Pub&Sub

## Abstracts

* Publish and subscribe custom message

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
~/Work/Demo/Robot/ROS1/noetic/03_PubSub/catkin_ws ~/Work/Demo/Robot/ROS1/noetic/03_PubSub
-----------------------------------------------------------------------------------------------------
Profile:                     default
Extending:          [cached] /opt/ros/noetic
Workspace:                   /home/xxxxxxxxx/Work/Demo/Robot/ROS1/noetic/03_PubSub/catkin_ws
-----------------------------------------------------------------------------------------------------
Build Space:        [exists] /home/xxxxxxxxx/Work/Demo/Robot/ROS1/noetic/03_PubSub/catkin_ws/build
Devel Space:        [exists] /home/xxxxxxxxx/Work/Demo/Robot/ROS1/noetic/03_PubSub/catkin_ws/devel
Install Space:      [unused] /home/xxxxxxxxx/Work/Demo/Robot/ROS1/noetic/03_PubSub/catkin_ws/install
Log Space:          [exists] /home/xxxxxxxxx/Work/Demo/Robot/ROS1/noetic/03_PubSub/catkin_ws/logs
Source Space:       [exists] /home/xxxxxxxxx/Work/Demo/Robot/ROS1/noetic/03_PubSub/catkin_ws/src
DESTDIR:            [unused] None
-----------------------------------------------------------------------------------------------------
Devel Space Layout:          linked
Install Space Layout:        None
-----------------------------------------------------------------------------------------------------
Additional CMake Args:       None
Additional Make Args:        None
Additional catkin Make Args: None
Internal Make Job Server:    True
Cache Job Environments:      False
-----------------------------------------------------------------------------------------------------
Buildlisted Packages:        None
Skiplisted Packages:         None
-----------------------------------------------------------------------------------------------------
Workspace configuration appears valid.
-----------------------------------------------------------------------------------------------------
[build] Found 3 packages in 0.0 seconds.                                                                                                                                               
[build] Package table is up to date.                                                                                                                                                   
Starting  >>> sample_msgs                                                                                                                                                              
Finished  <<< sample_msgs                [ 0.4 seconds ]                                                                                                                               
Starting  >>> publisher                                                                                                                                                                
Starting  >>> subscriber                                                                                                                                                               
Finished  <<< publisher                  [ 0.3 seconds ]                                                                                                                               
Finished  <<< subscriber                 [ 0.3 seconds ]                                                                                                                               
[build] Summary: All 3 packages succeeded!                                                                                                                                             
[build]   Ignored:   None.                                                                                                                                                             
[build]   Warnings:  None.                                                                                                                                                             
[build]   Abandoned: None.                                                                                                                                                             
[build]   Failed:    None.                                                                                                                                                             
[build] Runtime: 0.9 seconds total.                                                                                                                                                    
~/Work/Demo/Robot/ROS1/noetic/03_PubSub
````

## How to run?

````bash
$ ./run_subscriber.sh
... logging to /home/xxxxxxxxx/.ros/log/1388c704-8982-11ef-9cfd-c753b2340ae8/roslaunch-esxi-vm03-1962780.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://esxi-vm03:40485/
ros_comm version 1.17.0


SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.17.0

NODES

auto-starting new master
process[master]: started with pid [1962789]
ROS_MASTER_URI=http://esxi-vm03:11311/

setting /run_id to 1388c704-8982-11ef-9cfd-c753b2340ae8
process[rosout-1]: started with pid [1962802]
started core service [/rosout]
~/Work/Demo/Robot/ROS1/noetic/03_PubSub/catkin_ws ~/Work/Demo/Robot/ROS1/noetic/03_PubSub
````

After launch subscriber, kick next script to launch publisher on other shell.

````bash
$ ./run_publisher.sh 
~/Work/Demo/Robot/ROS1/noetic/03_PubSub/catkin_ws ~/Work/Demo/Robot/ROS1/noetic/03_PubSub
[ INFO] [1728837762.211411360]: publish: name: test, id: 1234
[ INFO] [1728837762.311467461]: publish: name: test, id: 1234
[ INFO] [1728837762.411511324]: publish: name: test, id: 1234
[ INFO] [1728837762.511460660]: publish: name: test, id: 1234
[ INFO] [1728837762.611504024]: publish: name: test, id: 1234
[ INFO] [1728837762.711496530]: publish: name: test, id: 1234
[ INFO] [1728837762.811613505]: publish: name: test, id: 1234
````

Then, subscriber recerives topics on previous shell.

````shell
[ INFO] [1728837762.511957472]: subscribe: name: test, id: 1234
[ INFO] [1728837762.611939774]: subscribe: name: test, id: 1234
[ INFO] [1728837762.711942642]: subscribe: name: test, id: 1234
[ INFO] [1728837762.812151483]: subscribe: name: test, id: 1234
[ INFO] [1728837762.912033432]: subscribe: name: test, id: 1234
[ INFO] [1728837763.012092525]: subscribe: name: test, id: 1234
[ INFO] [1728837763.111864578]: subscribe: name: test, id: 1234
[ INFO] [1728837763.211949630]: subscribe: name: test, id: 1234
````
