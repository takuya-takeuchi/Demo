#!/usr/bin/env python3
## coding: UTF-8

import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

def run(node_name:str, topic_name:str):
    rospy.init_node(node_name)

    rate = rospy.Rate(10)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform("map", "sensor", rospy.Time(0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform not available yet")
            rate.sleep()

    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)

    rospy.loginfo("Conection started...")

    points = [
        [1.0, 1.0, -1.0],  # (x, y, z)
    ]

    while not rospy.is_shutdown():
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "sensor"
        cloud = pc2.create_cloud_xyz32(header, points)

        pub.publish(cloud)

        # confirm transformed points
        transformed_cloud = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(cloud, transform)
        point_list = list(pc2.read_points(transformed_cloud, skip_nans=True, field_names=("x", "y", "z")))
        for point in point_list:
            rospy.loginfo(f"x: {point[0]:.3f}, y: {point[1]:.3f}, z: {point[2]:.3f}")

        rate.sleep()

if __name__ == "__main__":
    try:
        run("publisher", "custom")
    except rospy.ROSInterruptException:
        pass
