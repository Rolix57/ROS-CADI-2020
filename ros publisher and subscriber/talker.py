#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32
import numpy as np

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub_num = rospy.Publisher('chatter_num', Float32, queue_size=10)
    
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)

    i = 1
    while not rospy.is_shutdown():
        hello_str = 'hello_world!!'
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        if i % 10 == 0:
            pub_num.publish(np.pi/i)
            #i = 0
        i += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except:
        pass