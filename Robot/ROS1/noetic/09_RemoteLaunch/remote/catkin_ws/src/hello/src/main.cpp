#include <ros/ros.h>

#define NODE_NAME "hello"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    ros::Rate r(10);

    while (ros::ok())
    {
        ROS_INFO("Hello world");
        r.sleep();
    }

    return 0;
}