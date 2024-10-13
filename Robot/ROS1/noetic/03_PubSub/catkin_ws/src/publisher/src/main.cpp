#include <cstdlib>

#include <ros/ros.h>

#include <sample_msgs/MyMessage.h>

#define NODE_NAME "publisher"
#define TOPIC_NAME "custom"

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        ROS_INFO("usage: client <name> <id>");
        return 1;
    }

    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;
    const uint32_t queue_size = 5;
    ros::Publisher publisher = nh.advertise<sample_msgs::MyMessage>(TOPIC_NAME, queue_size);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        sample_msgs::MyMessage msg;
        msg.name = argv[1];
        msg.id = atoi(argv[2]);
        
        ROS_INFO("publish: name: %s, id: %d", msg.name.c_str(), msg.id);
        publisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}