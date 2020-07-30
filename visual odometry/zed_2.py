# libraries are imported
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# cv bridge is used as a static variable
bridge = CvBridge()

class Zed_tut():
    def __init__(self):
        rospy.init_node('Zed_tutorial')

        # camera variables
        self.left_img = None
        self.flag = 0
        self.counter = 0

        # odometry linear variables
        self.x_list = list()
        self.y_list = list()
        self.z_list = list()

        # odometry linear velocity variables
        self.v_x_list = list()
        self.v_y_list = list()
        self.v_z_list = list()

        self.counter_odom = 0

        # visual odometry variables
        self.cv2_img_1 = None
        self.depth = None

        self.trajectory = np.zeros((3,1))
        self.full_Rot = np.eye(3)
        self.trans = list()
        self.traj = list()

        # camera matrix - this depends on your camera
        self.k = np.array([[686.07, 0.0, 608.0], [0.0, 686.07, 348.51], [0.0, 0.0, 1.0]], dtype=np.float32)

        self.t1 = time.time()
        self.t2 = 0

        # subscriptors
        self.depth_sub = rospy.Subscriber('/zed/zed_node/depth/depth_registered', Image, self.depth_callback, queue_size=10)
        self.left_img_sub = rospy.Subscriber('/zed/zed_node/left/image_rect_color', Image, self.left_cam_callback, queue_size=10)
        self.odom_sub = rospy.Subscriber('/zed/zed_node/odom/', Odometry, self.odom_callback, queue_size=1)
                
    def left_cam_callback(self, left_msg):
        # image is changed to be used by open cv
        cv2_img = bridge.imgmsg_to_cv2(left_msg, 'bgr8')

        if self.counter < 20:
            # the first images of the camera are ignored
            pass
        else:
            # images with errors are left away
            try:
                cv2.imwrite('rgb/' + str(self.counter) + '.png', cv2_img)
                
                # features are extracted for the actual and previous images
                kp_1, des_1 = self.extract_features(self.cv2_img_1)
                kp, des = self.extract_features(cv2_img)

                # matching elements of both images are detected and the highest matches are chosen 
                matches = self.match_features(des_1, des)
                matches = self.filter_matches_distance(matches, 0.2)

                # motion is estimated and the rotation and translation vectors are acquired 
                rmat, tvec, img1_pts, img2_pts = self.estimate_motion(matches, kp_1, kp, self.k, self.depth)

                # motion is calculated
                self.trans.append(self.full_Rot.dot(tvec))
                self.full_Rot = self.full_Rot.dot(np.linalg.pinv(rmat))
                
                # trajectory is computed and stored
                new_coords = np.add((self.full_Rot.dot(tvec)).T, self.trajectory[:,-1].T)  
                self.trajectory = np.concatenate((self.trajectory, new_coords.T), axis=1)
                np.savetxt('trajectory.dat', self.trajectory)
                np.savetxt('trajectory_x.dat', self.trajectory[0,:])
                np.savetxt('trajectory_y.dat', self.trajectory[1,:])
                np.savetxt('trajectory_z.dat', self.trajectory[2,:])
            except:
                pass
                        

        self.cv2_img_1 = cv2_img

    def depth_callback(self, depth_msg):

        # depth information is acquired and stored
        cv2_dpth = bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        cv2_dpth = np.array(cv2_dpth, dtype = np.dtype('f16'))

        # out of range data is corrected
        cv2_dpth[np.isnan(cv2_dpth)] = 10
        cv2_dpth[np.isinf(cv2_dpth)] = 10
        self.depth = cv2_dpth

        self.counter += 1
        self.flag = 1

    def odom_callback(self, odom_msg):

        if self.counter > 20:
            self.current_x = odom_msg.pose.pose.position.x
            self.current_y = odom_msg.pose.pose.position.y
            self.current_z = odom_msg.pose.pose.position.z

            self.x_list.append(self.current_x)
            self.y_list.append(self.current_y)
            self.z_list.append(self.current_z)

            self.current_v_x = odom_msg.twist.twist.linear.x
            self.current_v_y = odom_msg.twist.twist.linear.y
            self.current_v_z = odom_msg.twist.twist.linear.z

            self.v_x_list.append(self.current_x)
            self.v_y_list.append(self.current_y)
            self.v_z_list.append(self.current_z)


            np.savetxt('odom/cam_odom_x.dat', self.x_list)
            np.savetxt('odom/cam_odom_y.dat', self.y_list)
            np.savetxt('odom/cam_odom_z.dat', self.z_list)

            np.savetxt('odom/cam_odom_v_x.dat', self.v_x_list)
            np.savetxt('odom/cam_odom_v_y.dat', self.v_y_list)
            np.savetxt('odom/cam_odom_v_z.dat', self.v_z_list)

        self.counter_odom += 1

    # ORB is used to detect features
    def extract_features(self, image):
        orb = cv2.ORB_create(nfeatures=1500)
        kp, des = orb.detectAndCompute(image, None)
        return kp, des
        
    # Brute Force algorithm is used to connect matches
    def match_features(self, des1, des2):
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key = lambda x:x.distance)
        return matches

    # a percentage of the matches is chosen
    def filter_matches_distance(self, match, dist_threshold = 0.25):
        filtered_match = match[:int(dist_threshold * len(match))]
        return filtered_match

    def estimate_motion(self, match, kp1, kp2, k, depth1=None):
        rmat = np.eye(3)
        tvec = np.zeros((3, 1))
        image1_points = []
        image2_points = []
        
        objectpoints = list()
        
        for m in match:
            qidx = m.queryIdx
            tidx = m.trainIdx
            
            x1, y1 = kp1[qidx].pt
            x2, y2 = kp2[tidx].pt
        
            Z = depth1[int(y1), int(x1)]
            
            if Z != 10:
                scaled_coord = np.dot(np.linalg.pinv(k), np.array([x1, y1, 1]))
                cali_coord = float(Z)/scaled_coord[2]*scaled_coord
                objectpoints.append(cali_coord)
                image1_points.append([x1, y1])
                image2_points.append([x2, y2])
            else:
                continue
        
                
        objPoints = np.array(objectpoints)
        imgpoints = np.array(image2_points)
        
        try:
            _, rmat, tvec, _  = cv2.solvePnPRansac(objPoints, imgpoints, k, None, flags=cv2.SOLVEPNP_EPNP)
            rmat = cv2.Rodrigues(rmat)[0]
        except:
            pass        
        return rmat, tvec, image1_points, image2_points

if __name__ == '__main__':
    dummy_agent = Zed_tut()
    rospy.spin()