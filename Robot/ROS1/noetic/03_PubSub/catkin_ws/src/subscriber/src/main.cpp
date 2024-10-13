#include <ros/ros.h>

#include <sample_msgs/MyMessage.h>

#define NODE_NAME "subscriber"
#define TOPIC_NAME "custom"

void callback(const sample_msgs::MyMessage& msg)
{
    ROS_INFO("subscribe: name: %s, id: %d", msg.name.c_str(), msg.id);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;
    const uint32_t queue_size = 5;
    ros::Subscriber subscriber = nh.subscribe(TOPIC_NAME, queue_size, callback);

    ros::spin();

    return 0;
}