cmake_minimum_required(VERSION 3.0.2)

set(PROJ_NAME subscriber)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    sample_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy std_msgs sample_msgs
)