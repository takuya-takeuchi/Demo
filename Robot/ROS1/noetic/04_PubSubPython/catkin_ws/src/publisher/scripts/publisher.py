#!/usr/bin/env python3
## coding: UTF-8

import argparse

import rospy

from sample_msgs.msg import MyMessage

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", type=str, required=True)
    parser.add_argument("--id", type=int, required=True)
    return parser.parse_args()

def run(node_name:str, topic_name:str, name:str, id:int):
    rospy.init_node(node_name)
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
    args = get_args()
    name = args.name
    id   = args.id

    rospy.loginfo("Arguments")
    rospy.loginfo(f"\t  name: {name}")
    rospy.loginfo(f"\t    id: {id}")

    run("publisher", "custom", name, id)
