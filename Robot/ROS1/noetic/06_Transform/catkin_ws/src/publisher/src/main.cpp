#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#define NODE_NAME "publisher"
#define TOPIC_NAME "custom"

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, NODE_NAME);
        ros::NodeHandle nh;

        ros::Rate rate(10);

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);

        geometry_msgs::TransformStamped transform;
        while (ros::ok())
        {
            try
            {
                transform = tf_buffer.lookupTransform("map", "sensor", ros::Time(0));
                break;
            }
            catch (tf2::TransformException& ex)
            {
                ROS_WARN("Transform not available yet: %s", ex.what());
                rate.sleep();
            }
        }

        ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(TOPIC_NAME, 10);
        ROS_INFO("Connection started...");

        std::vector<geometry_msgs::Point32> points;
        geometry_msgs::Point32 point;
        point.x = 1.0;
        point.y = 1.0;
        point.z = -1.0;
        points.push_back(point);

        while (ros::ok())
        {
            sensor_msgs::PointCloud2 cloud;
            cloud.header.stamp = ros::Time::now();
            cloud.header.frame_id = "sensor";

            sensor_msgs::PointCloud2Modifier modifier(cloud);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(points.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

            for (const auto& point : points)
            {
                *iter_x = point.x;
                *iter_y = point.y;
                *iter_z = point.z;
                ++iter_x;
                ++iter_y;
                ++iter_z;
            }

            // Publish transformed cloud
            pub.publish(cloud);

            // Transform PointCloud2
            sensor_msgs::PointCloud2 transformed_cloud;
            try
            {
                tf2::doTransform(cloud, transformed_cloud, transform);
            }
            catch (tf2::TransformException& ex)
            {
                ROS_WARN("Failed to transform cloud: %s", ex.what());
                continue;
            }

            // Log transformed points
            sensor_msgs::PointCloud2ConstIterator<float> t_iter_x(transformed_cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> t_iter_y(transformed_cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> t_iter_z(transformed_cloud, "z");

            for (; t_iter_x != t_iter_x.end(); ++t_iter_x, ++t_iter_y, ++t_iter_z)
                ROS_INFO("x: %.3f, y: %.3f, z: %.3f", *t_iter_x, *t_iter_y, *t_iter_z);

            rate.sleep();
        }
    }
    catch (const ros::Exception& ex)
    {
        ROS_ERROR("ROS Exception: %s", ex.what());
    }

    return 0;
}