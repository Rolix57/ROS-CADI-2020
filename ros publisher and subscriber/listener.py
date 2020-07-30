#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32

def callback(data):
    print(data)

def num_callback(data):
    print(float(data.data)*11)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.Subscriber('chatter_num', Float32, num_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()