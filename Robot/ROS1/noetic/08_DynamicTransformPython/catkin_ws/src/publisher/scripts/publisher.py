#!/usr/bin/env python3
## coding: UTF-8

import math
import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

def run(node_name:str, topic_name:str):
    rospy.init_node(node_name)

    rate = rospy.Rate(10)

    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)

    rospy.loginfo("Conection started...")

    points = [
        [3.0, 0, 0],  # (x, y, z)
    ]

    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        sec = stamp.to_sec()
        omega = 10 * (math.pi / 30) # x10 speed
        radian = (omega * sec) % (2 * math.pi)

        # static transform (map -> base_link)
        map_to_base_link = TransformStamped()
        map_to_base_link.header.stamp = stamp
        map_to_base_link.header.frame_id = "map"
        map_to_base_link.child_frame_id = "base_link"
        map_to_base_link.transform.translation.x = 0.0
        map_to_base_link.transform.translation.y = 0.0
        map_to_base_link.transform.translation.z = 0.0
        map_to_base_link.transform.rotation.x = 0.0
        map_to_base_link.transform.rotation.y = 0.0
        map_to_base_link.transform.rotation.z = 0.0
        map_to_base_link.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(map_to_base_link)

        # transform sensor (base_link -> sensor)
        base_link_to_sensor = TransformStamped()
        base_link_to_sensor.header.stamp = stamp
        base_link_to_sensor.header.frame_id = "base_link"
        base_link_to_sensor.child_frame_id = "sensor"
        base_link_to_sensor.transform.translation.x = 3.0 * math.cos(radian)
        base_link_to_sensor.transform.translation.y = 3.0 * math.sin(radian)
        base_link_to_sensor.transform.translation.z = 3.0
        base_link_to_sensor.transform.rotation.x = 0.0
        base_link_to_sensor.transform.rotation.y = math.sin(math.radians(90 / 2))
        base_link_to_sensor.transform.rotation.z = 0.0
        base_link_to_sensor.transform.rotation.w = math.cos(math.radians(90 / 2))
        tf_broadcaster.sendTransform(base_link_to_sensor)

        # Published base_link transform
        header = std_msgs.msg.Header()
        header.stamp = stamp
        header.frame_id = "sensor"
        cloud = pc2.create_cloud_xyz32(header, points)
        pub.publish(cloud)

        transformed_cloud = tf2_sensor_msgs.do_transform_cloud(cloud, base_link_to_sensor)
        transformed_cloud = tf2_sensor_msgs.do_transform_cloud(transformed_cloud, map_to_base_link)
        point_list = list(pc2.read_points(transformed_cloud, skip_nans=True, field_names=("x", "y", "z")))
        rospy.loginfo(f"    sensor - x: {base_link_to_sensor.transform.translation.x:.3f}, y: {base_link_to_sensor.transform.translation.y:.3f}, z: {base_link_to_sensor.transform.translation.z:.3f}")
        for point in point_list:
            rospy.loginfo(f"pointcloud - x: {point[0]:.3f}, y: {point[1]:.3f}, z: {point[2]:.3f}")

        rate.sleep()

if __name__ == "__main__":
    try:
        run("publisher", "custom")
    except rospy.ROSInterruptException:
        pass
