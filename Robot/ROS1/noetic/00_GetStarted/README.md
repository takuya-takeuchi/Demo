# Get started

## Abstracts

* Setup sample project to start development

## Requirements

### Common

* ROS Noetic

## Dependencies

* [ROS](https://github.com/ros/ros)
  * Noetic
  * BSD-3-Clause license

## How to use?

````bash
$ ./setup.sh 
~/Work/Demo/Robot/ROS1/noetic/00_GetStarted/catkin_ws/src ~/Work/Demo/Robot/ROS1/noetic/00_GetStarted
Created file hello/package.xml
Created file hello/CMakeLists.txt
Created folder hello/include/hello
Created folder hello/src
Successfully created files in /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/00_GetStarted/catkin_ws/src/hello. Please adjust the values in package.xml.
~/Work/Demo/Robot/ROS1/noetic/00_GetStarted
~/Work/Demo/Robot/ROS1/noetic/00_GetStarted/catkin_ws ~/Work/Demo/Robot/ROS1/noetic/00_GetStarted
Initialized new catkin workspace in `/home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/00_GetStarted/catkin_ws`
---------------------------------------------------------------------------------------------------------
Profile:                     default
Extending:             [env] /opt/ros/noetic
Workspace:                   /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/00_GetStarted/catkin_ws
---------------------------------------------------------------------------------------------------------
Build Space:        [exists] /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/00_GetStarted/catkin_ws/build
Devel Space:        [exists] /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/00_GetStarted/catkin_ws/devel
Install Space:      [unused] /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/00_GetStarted/catkin_ws/install
Log Space:         [missing] /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/00_GetStarted/catkin_ws/logs
Source Space:       [exists] /home/xxxxxxxxxx/Work/Demo/Robot/ROS1/noetic/00_GetStarted/catkin_ws/src
DESTDIR:            [unused] None
---------------------------------------------------------------------------------------------------------
Devel Space Layout:          linked
Install Space Layout:        None
---------------------------------------------------------------------------------------------------------
Additional CMake Args:       None
Additional Make Args:        None
Additional catkin Make Args: None
Internal Make Job Server:    True
Cache Job Environments:      False
---------------------------------------------------------------------------------------------------------
Buildlisted Packages:        None
Skiplisted Packages:         None
---------------------------------------------------------------------------------------------------------
Workspace configuration appears valid.

NOTE: Forcing CMake to run for each package.
---------------------------------------------------------------------------------------------------------
[build] Found 1 packages in 0.0 seconds.                                                                                                                               
[build] Updating package table.                                                                                                                                        
Starting  >>> catkin_tools_prebuild                                                                                                                                    
Finished  <<< catkin_tools_prebuild                [ 2.3 seconds ]                                                                                                     
Starting  >>> hello                                                                                                                                                    
Finished  <<< hello                                [ 2.7 seconds ]                                                                                                     
[build] Summary: All 2 packages succeeded!                                                                                                                             
[build]   Ignored:   None.                                                                                                                                             
[build]   Warnings:  None.                                                                                                                                             
[build]   Abandoned: None.                                                                                                                                             
[build]   Failed:    None.                                                                                                                                             
[build] Runtime: 5.0 seconds total.                                                                                                                                    
[build] Note: Workspace packages have changed, please re-source setup files to use them.
~/Work/Demo/Robot/ROS1/noetic/00_GetStarted
````
