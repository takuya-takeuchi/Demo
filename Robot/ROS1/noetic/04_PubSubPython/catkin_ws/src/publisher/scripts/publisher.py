#!/usr/bin/env python3
## coding: UTF-8

import rospy

from sample_msgs.msg import MyMessage

def run(node_name:str, topic_name:str):
    rospy.init_node(node_name)

    name = rospy.get_param("~name")
    id   = rospy.get_param("~id")

    rospy.loginfo("Arguments")
    rospy.loginfo(f"\t  name: {name}")
    rospy.loginfo(f"\t    id: {id}")

    pub = rospy.Publisher(topic_name, MyMessage, queue_size=10)
    rate = rospy.Rate(10)

    rospy.loginfo("Conection started...")

    while not rospy.is_shutdown():
        message = MyMessage()
        message.name = name
        message.id = id

        pub.publish(message)
        rospy.loginfo(f"publish: name: {message.name}, id: {message.id}")

        rate.sleep()

if __name__ == '__main__':
    run("publisher", "custom")
