cmake_minimum_required(VERSION 3.0.2)

set(PROJ_NAME publisher)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

find_package(catkin REQUIRED COMPONENTS
    rospy
    tf2_ros
    std_msgs
    tf2_sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy tf2_ros tf2_sensor_msgs std_msgs
)