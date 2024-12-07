#!/usr/bin/env python3
## coding: UTF-8

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

def run(node_name:str, topic_name:str):
    rospy.init_node(node_name)

    tf_buffer = tf2_ros.Buffer()
    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform("map", "sensor", rospy.Time(0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform not available yet")

    pub = rospy.Publisher(topic_name, PointStamped, queue_size=10)
    rate = rospy.Rate(10)

    rospy.loginfo("Conection started...")

    while not rospy.is_shutdown():
        msg = PointStamped()
        msg.header.frame_id = "sensor"
        msg.header.stamp = rospy.Time.now()
        msg.point.x = 10.0
        msg.point.y = -10.0
        msg.point.z = 10.0

        transformed_point = tf2_geometry_msgs.do_transform_point(msg, transform)

        pub.publish(transformed_point)
        rate.sleep()

if __name__ == "__main__":
    try:
        run("publisher", "custom")
    except rospy.ROSInterruptException:
        pass
