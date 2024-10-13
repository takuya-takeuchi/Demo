#include <cstdlib>

#include <ros/ros.h>

#include <sample_msgs/MyMessage.h>

#define NODE_NAME "publisher"
#define TOPIC_NAME "custom"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");
    std::string name;
    if (!pnh.getParam("name", name))
    {
        ROS_ERROR("Failed to get 'name' parameter");
        return -1;
    }

    int id;
    if (!pnh.getParam("id", id))
    {
        ROS_ERROR("Failed to get 'name' parameter");
        return -1;
    }

    const uint32_t queue_size = 5;
    ros::Publisher publisher = nh.advertise<sample_msgs::MyMessage>(TOPIC_NAME, queue_size);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        sample_msgs::MyMessage msg;
        msg.name = name;
        msg.id = id;
        
        ROS_INFO("publish: name: %s, id: %d", msg.name.c_str(), msg.id);
        publisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}