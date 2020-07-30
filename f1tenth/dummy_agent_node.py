#!/usr/bin/env python

# all the used libraries are imported
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry
import csv
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# The class is defined 
class Agent(object):

    def __init__(self):

        # a list is created to store the previously created path
        self.waypoint = list()
        self.waypoint = np.genfromtxt('/home/rolando/catkin_ws/src/f1tenth_gym_ros/scripts/odom_data.dat', dtype=None, delimiter = ' ')
        self.waypoint = self.waypoint[10:].astype(np.float)
       
        self.goal_idx = 0

        # odometry variables are created
        self.current_x = 0
        self.current_y = 0
        self.current_v_x = 0
        self.current_v_y = 0
        self.current_v = (self.current_v_x**2 + self.current_v_y**2) ** 0.5
        self.current_roll = 0
        self.current_pitch = 0
        self.current_yaw = 0

        # control variables are created
        self.dt = 0.01
        self.e_1 = 0
        self.d_t_g = 3.0

        # publishers and subscribers
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

    # this function is accessed every 0.01 seconds
    def timer_callback(self, event):
        # which waypoint is closest to where I am
        dist_to_goal = np.hypot(self.current_x-self.waypoint[self.goal_idx, 0],self.current_y-self.waypoint[self.goal_idx, 1])
        
        # according to the distance to goal, the next goal is chosen or kept
        self.d_t_g = 0.3 * (self.waypoint[self.goal_idx, 2])
        if dist_to_goal < self.d_t_g:
            self.goal_idx += 1

        # a horizon of 10 waypoints is chosen
        waypoints = self.waypoint[self.goal_idx:self.goal_idx+10,:]

        #########################################################################
        ######## STANLEY
        '''
        # first two points 
        x_0, y_0 = waypoints[0][0], waypoints[0][1]
        x_1, y_1 = waypoints[1][0], waypoints[1][1]
        
        # computation of the crosstrack error
        if x_1 == x_0:
            # this statement works when the function is undefined and goes to inf
            e = self.current_x
        else:
            # distance to the desired path
            a = (y_1 - y_0) / (x_1 - x_0)
            b = -1
            c = y_0 - a * x_0
            e = (a*self.current_x + b*self.current_y + c) / ((a**2 + b**2)**0.5)
        
        # heading error
        gamma = np.arctan2(waypoints[-1][1]-waypoints[0][1], 
                            waypoints[-1][0]-waypoints[0][0])
        # heading error relative to the car orientation
        h_e = gamma - self.current_yaw
        
        # saturation of the heading error to keep it in the [0,pi] or [-pi,0] range
        h_e = h_e - (2.0 * np.pi) if h_e > np.pi else h_e 
        h_e = h_e + (2.0 * np.pi) if h_e < -np.pi else h_e
        
        # angle between the vehicle and the vehicle
        beta = np.arctan2(self.current_y - waypoints[0][1], self.current_x - waypoints[0][0])
        epsilon = gamma - beta
        # saturation of the angle to keep it in the [0,pi] or [-pi,0] range
        epsilon = epsilon - (2.0 * np.pi) if epsilon > np.pi else epsilon 
        epsilon = epsilon + (2.0 * np.pi) if epsilon < -np.pi else epsilon 
        
        # the crosstrack error sign is modified according to the angle
        e = abs(e) if epsilon >= 0 else -abs(e)
        
        # pid calculations
        p_e = 2.6 * e #3
        i_e = 0.1 * (e + self.e_1) * self.dt #1.5
        d_e = 0.4 * (e - self.e_1) / self.dt #1
        
        # steering angle calculations
        theta_2 = np.arctan((p_e + i_e + d_e)/ (0.1 + self.current_v))
        
        # correction
        delta = h_e + theta_2
        delta = 1.22 if delta > 1.22 else delta
        delta = -1.22 if delta < -1.22 else delta
        self.e_1 = e
        
        
        #########################################################################
        ######## PURE PURSUIT
        '''
        # vehicle constants
        L = 0.35
        Kdd = 0.4

        # next position objective
        x_next = waypoints[-1][0]
        y_next = waypoints[-1][1]

        # distance to the next point
        ld = np.hypot(self.current_x-x_next,self.current_y-y_next)
        
        # steering angle correction
        alpha = np.arctan2(y_next - self.current_y, x_next - self.current_x) - self.current_yaw                    
        delta = np.arctan2(2 * L * np.sin(alpha), Kdd * self.current_v)
        
        # speed and steering angle are published
        drive = AckermannDriveStamped()
        drive.drive.speed = waypoints[0][2]
        drive.drive.steering_angle = delta
        self.drive_pub.publish(drive)

    # odometry data is collected
    def odom_callback(self, odom_msg):
        
        # orientation data is acquired - data is transformed from quat to euler matrix
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.current_roll, self.current_pitch, self.current_yaw) = euler_from_quaternion (orientation_list)

        # linear odometry is acquired
        # + STANLEY  - PURE PURSUIT
        self.current_x = odom_msg.pose.pose.position.x - 0.17 * np.cos(self.current_yaw)
        self.current_y = odom_msg.pose.pose.position.y - 0.17 * np.sin(self.current_yaw)
        self.current_v_x = odom_msg.twist.twist.linear.x
        self.current_v_y = odom_msg.twist.twist.linear.y
        self.current_v = (self.current_v_x**2 + self.current_v_y**2) ** 0.5
        

if __name__ == '__main__':
    rospy.init_node('dummy_agent')
    dummy_agent = Agent()
    rospy.spin()