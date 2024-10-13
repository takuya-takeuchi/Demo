#!/usr/bin/env python3
## coding: UTF-8

import rospy

from sample_msgs.msg import MyMessage

def callback(message):
    rospy.loginfo(f"subscribe: name: {message.name}, id: {message.id}")

if __name__ == '__main__':
    rospy.init_node("subscriber")
    sub = rospy.Subscriber("custom", MyMessage, callback)
    rospy.spin()
