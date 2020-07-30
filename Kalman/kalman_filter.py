#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Vector3

class KalmanFilt:
    def __init__(self):
        # Sampling time
        self.Ts = 0.01
        # State vector
        self.x = np.zeros(6)
        # Measurement vector
        self.z = np.zeros(6)
        # Sensor variables
        self.ax = 0.
        self.ay = 0.
        self.az = 0.
        self.wx = 0.
        self.wy = 0.
        self.wz = 0.
        self.Bx = 0.
        self.By = 0.
        self.Bz = 0.

        # State matrix
        self.F = np.array([[1., self.Ts, 0., 0., 0., 0.],
                        [0., 1., 0., 0., 0., 0.],
                        [0., 0., 1., self.Ts, 0., 0.],
                        [0., 0., 0., 1., 0., 0.],
                        [0., 0., 0., 0., 1., self.Ts],
                        [0., 0., 0., 0., 0., 1.]])
        
        # Process noise
        sigma_w = 1.
        self.Q = np.array([[1/4*self.Ts ** 4, 1/2*self.Ts ** 3, 0., 0., 0., 0.],
                        [1/2*self.Ts ** 3, self.Ts ** 2, 0., 0., 0., 0.],
                        [0., 0., 1/4*self.Ts ** 4, 1/2*self.Ts ** 3, 0., 0.],
                        [0., 0., 1/2*self.Ts ** 3, self.Ts ** 2, 0., 0.],
                        [0., 0., 0., 0., 1/4*self.Ts ** 4, 1/2*self.Ts ** 3],
                        [0., 0., 0., 0., 1/2*self.Ts ** 3, self.Ts ** 2]]) * sigma_w ** 2
        
        # Measurement matrix
        self.H = np.eye(6)

        # Measurement noise
        sigma_v = 0.05
        self.R = np.eye(6) * sigma_v ** 2

        # Covariance matrix
        self.P = np.eye(6) * 1e3

    def callbackAcc(self, Acc):
        self.ax = Acc.x
        self.ay = Acc.y
        self.az = Acc.z

    def callbackOmg(self, Omg):
        self.wx = Omg.x
        self.wy = Omg.y
        self.wz = Omg.z

    def callbackMag(self, Mag):
        self.Bx = Mag.x
        self.By = Mag.y
        self.Bz = Mag.z

    def executeKalman(self):
        # Measurement extraction
        phi = self.x[0]
        theta = self.x[2]
        Bx_hat = self.Bx * np.cos(theta) + self.By * np.sin(theta) * np.sin(phi) + \
            self.Bz * np.sin(theta) * np.cos(phi)
        By_hat = self.By * np.cos(phi) - self.Bz * np.sin(phi)

        acc_roll = np.arctan2(self.ay, np.hypot(self.ax, self.az))
        acc_pitch = -np.arctan2(self.ax, np.hypot(self.ay, self.az))
        mag_yaw = -np.arctan2(By_hat, Bx_hat)

        # Angle unwrap
        acc_roll = np.unwrap(np.array([self.z[0], acc_roll]))[-1]
        acc_pitch = np.unwrap(np.array([self.z[2], acc_pitch]))[-1]
        mag_yaw = np.unwrap(np.array([self.z[4], mag_yaw]))[-1]

        self.z = [acc_roll, self.wx, acc_pitch, self.wy, mag_yaw, self.wz]

        # Prediction
        self.x = self.F.dot(self.x)
        self.P = self.F.dot(self.P.dot(self.F.T)) + self.Q

        # Correction
        K = self.P.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P.dot(self.H.T)) + self.R))
        self.x = self.x + K.dot(self.z - self.H.dot(self.x))
        self.P = self.P - K.dot(self.H.dot(self.P))

    def main(self):
        rospy.init_node('kalman_filter', anonymous=True)
        rospy.Subscriber('acceleration', Vector3, self.callbackAcc)
        rospy.Subscriber('angular_rate', Vector3, self.callbackOmg)
        rospy.Subscriber('magnetic_field', Vector3, self.callbackMag)
        pub_rpy = rospy.Publisher('rpy', Vector3, queue_size=10)

        rpy = Vector3()

        rate = rospy.Rate(1/self.Ts)

        while not rospy.is_shutdown():
            # Execute Kalman algorithm
            self.executeKalman()

            # Assign rpy variables
            rpy.x = np.rad2deg(self.x[0])
            rpy.y = np.rad2deg(self.x[2])
            rpy.z = np.rad2deg(self.x[4])
            
            # Publish rpy topic
            pub_rpy.publish(rpy)

            rate.sleep()

if __name__ == '__main__':
    try:
        my_kalman = KalmanFilt()
        my_kalman.main()
    except rospy.ROSInterruptException:
        pass