#include <ros/ros.h>

#include <sample_msgs/MyMessage.h>

#define NODE_NAME "server"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;

    sample_msgs::MyMessage msg;
    msg.id = 1;
    msg.name = "Example";

    ROS_INFO("ID: %d, Name: %s", msg.id, msg.name.c_str());

    ros::spin();

    return 0;
}