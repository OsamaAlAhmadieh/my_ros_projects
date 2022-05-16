#!/usr/bin/python

import rospy 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
import cv2, cv_bridge
import numpy as np

class Follower:

    def __init__(self):
        self.image_sub = rospy.Subscriber('/robot/camera1/image_raw', Image, self.image_cb)
        cv2.namedWindow('window1', 1)
        self.bridge = cv_bridge.CvBridge()
        self.thersholds = {'lower_yellow': np.array([10, 10, 10]), 
                        'upper_yellow': np.array([255, 255, 250])}
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        if rospy.has_param('Kp'):
            self.Kp = rospy.get_param('Kp')
        else:
            rospy.logwarn('gain not set, using default instead')
            self.Kp = 1/200

    def image_cb(self,msg):
        # converting ros msg to opencv format
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except AttributeError:
            rospy.sleep(3)
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # converting the color from RGB to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.thersholds['lower_yellow'], self.thersholds['upper_yellow'])

        self.create_target_point(image, mask)
        
    def create_target_point(self, image, mask):
        h, w, d = image.shape
        search_top = int((3/4) * h)
        search_bot = int(search_top + 20)
        # defining the search area 
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        # calculating the centroid using the moments function
        M = cv2.moments(mask)
        # calculating the coordinate of the centroid
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        # plotting the circle
        cv2.circle(image, (cx, cy), 10, (0,0,255),-1)
        self.create_mid_line(image)
        cv2.imshow('window1', image)
        self.run_controller(cx, w)
        cv2.waitKey(3)

    def run_controller(self, cx, w):
        self.twist.linear.x = 0.5
        err = cx - w/2
        self.twist.angular.z = -self.Kp * err
        self.cmd_vel_pub.publish(self.twist)

    def create_mid_line(self, image):
        h,w,d = image.shape
        cv2.line(image, (int(w/2), h), (int(w/2), 0), (255,0,0), 2)




if __name__ == '__main__':
    rospy.init_node('follow_dot', anonymous=True)
    follower = Follower()
    rospy.spin()
