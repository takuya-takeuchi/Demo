#include <cstdlib>

#include <ros/ros.h>

#include <sample_msgs/greeting.h>

#define NODE_NAME "client"
#define TOPIC_NAME "greeting"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    if (argc != 2)
    {
        ROS_INFO("usage: client <message>");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<sample_msgs::greeting>(TOPIC_NAME);
    sample_msgs::greeting msg;
    // msg.message = argv[1];
    // if (client.call(srv))
    // {
    //     ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service add_two_ints");
    //     return 1;
    // }

    return 0;
}