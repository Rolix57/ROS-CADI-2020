#! /usr/bin/env python

import rospy
import numpy as np
import csv
from geometry_msgs.msg import Vector3

def imu_talker():
    pub_accel = rospy.Publisher('acceleration', Vector3, queue_size=10)
    pub_gyro = rospy.Publisher('angular_rate', Vector3, queue_size=10)
    pub_mag = rospy.Publisher('magnetic_field', Vector3, queue_size=10)

    Acc = Vector3()
    Omg = Vector3()
    Mag = Vector3()

    with open('sensorlog.csv', mode='r') as csvfile:
        data = np.array(list(csv.reader(csvfile))).astype(np.float)
    
    rospy.init_node('imu_talker', anonymous=True)
    rate = rospy.Rate(100)

    i = 0

    while not rospy.is_shutdown():
        Acc.x = data[i,1]
        Acc.y = data[i,2]
        Acc.z = data[i,3]
        Omg.x = data[i,4]
        Omg.y = data[i,5]
        Omg.z = data[i,6]
        Mag.x = data[i,7]
        Mag.y = data[i,8]
        Mag.z = data[i,9]
        i = i + 1
        if i >= len(data[:,0]):
            i = 0
        
        pub_accel.publish(Acc)
        pub_gyro.publish(Omg)
        pub_mag.publish(Mag)

        rate.sleep()

if __name__ == '__main__':
    try:
        imu_talker()
    except rospy.ROSInterruptException:
        pass