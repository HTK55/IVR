#!/usr/bin/env python3
import math

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import message_filters


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send messages to a topic named image_topic

        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)

        # initialize publishers to send joints' angular position to a topic called joints_pos
        self.joint2_pub = rospy.Publisher("joints_pos/joint_angle_2", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("joints_pos/joint_angle_3", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("joints_pos/joint_angle_4", Float64, queue_size=10)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback
        # function to recieve data
        # initialize a subscriber to recieve messages rom a topic named /robot/camera2/image_raw and use callback
        # function to recieve data
        self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw", Image)
        self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw", Image)
        self.ts = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 10)
        self.ts.registerCallback(self.callback)
        self.rate = rospy.Rate(1)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        self.lastyellow = np.array([0, 0, 0])
        self.lastblue = np.array([0, 0, 0])
        self.lastred = np.array([0, 0, 0])

    # In this method you can focus on detecting the centre of the red circle
    # In this method you can focus on detecting the centre of the red circle
    def detect_red(self, c1, c2):
        # Isolate the blue colour in the image as a binary image
        mask1 = cv2.inRange(c1, (0, 0, 100), (0, 0, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask1 = cv2.dilate(mask1, kernel, iterations=3)
        M1 = cv2.moments(mask1)
        if M1['m00'] == 0:
            cy = self.lastred[0]
            cz1 = 0
        else:
            cy = int(M1['m10'] / M1['m00'])
            cz1 = int(M1['m01'] / M1['m00'])
        mask2 = cv2.inRange(c2, (0, 0, 100), (0, 0, 255))
        mask2 = cv2.dilate(mask2, kernel, iterations=3)
        M2 = cv2.moments(mask2)
        if M2['m00'] == 0:
            cx = self.lastred[0]
            cz2 = 0
        else:
            cx = int(M2['m10'] / M2['m00'])
            cz2 = int(M2['m01'] / M2['m00'])
        if cz1 == 0:
            cz = cz2
        elif cz2 == 0:
            cz = cz1
        else:
            cz = (cz1 + cz2) / 2
        return np.array([cx, cy, cz])

    # Detecting the centre of the green circle
    def detect_green(self, c1, c2):
        mask1 = cv2.inRange(c1, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask1 = cv2.dilate(mask1, kernel, iterations=3)
        M1 = cv2.moments(mask1)
        cy = int(M1['m10'] / M1['m00'])
        cz1 = int(M1['m01'] / M1['m00'])
        mask2 = cv2.inRange(c2, (0, 100, 0), (0, 255, 0))
        mask2 = cv2.dilate(mask2, kernel, iterations=3)
        M2 = cv2.moments(mask2)
        cx = int(M2['m10'] / M2['m00'])
        cz2 = int(M2['m01'] / M2['m00'])
        cz = (cz1 + cz2) / 2
        return np.array([cx, cy, cz])

    # Detecting the centre of the blue circle
    def detect_blue(self, c1, c2):
        mask1 = cv2.inRange(c1, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask1 = cv2.dilate(mask1, kernel, iterations=3)
        M1 = cv2.moments(mask1)
        if M1['m00'] == 0:
            cy = self.lastblue[1]
            cz1 = 0
        else:
            cy = int(M1['m10'] / M1['m00'])
            cz1 = int(M1['m01'] / M1['m00'])

        mask2 = cv2.inRange(c2, (100, 0, 0), (255, 0, 0))
        mask2 = cv2.dilate(mask2, kernel, iterations=3)
        M2 = cv2.moments(mask2)
        if M2['m00'] == 0:
            cx = self.lastblue[0]
            cz2 = 0
        else:
            cx = int(M2['m10'] / M2['m00'])
            cz2 = int(M2['m01'] / M2['m00'])
        if cz1 == 0:
            cz = cz2
        elif cz2 == 0:
            cz = cz1
        else:
            cz = (cz1 + cz2) / 2
        return np.array([cx, cy, cz])

    # Detecting the centre of the yellow circle
    def detect_yellow(self, c1, c2):
        mask1 = cv2.inRange(c1, (0, 100, 100), (0, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask1 = cv2.dilate(mask1, kernel, iterations=3)
        M1 = cv2.moments(mask1)
        if M1['m00'] == 0:
            cy = self.lastyellow[1]
            cz1 = 0
        else:
            cy = int(M1['m10'] / M1['m00'])
            cz1 = int(M1['m01'] / M1['m00'])

        mask2 = cv2.inRange(c2, (0, 100, 100), (0, 255, 255))
        mask2 = cv2.dilate(mask2, kernel, iterations=3)
        M2 = cv2.moments(mask2)
        if M2['m00'] == 0:
            cx = self.lastyellow[0]
            cz2 = 0
        else:
            cx = int(M2['m10'] / M2['m00'])
            cz2 = int(M2['m01'] / M2['m00'])
        if cz1 == 0:
            cz = cz2
        elif cz2 == 0:
            cz = cz1
        else:
            cz = (cz1 + cz2) / 2
        return np.array([cx, cy, cz])

    # Calculate the conversion from pixel to meter
    def pixel2meter(self):
        # Obtain the centre of each coloured blob
        green = self.detect_green(self.cv_image1, self.cv_image2)
        yellow = self.detect_yellow(self.cv_image1, self.cv_image2)
        # find the distance between two circles
        dist = np.sum((green - yellow) ** 2)
        return 4 / dist

    def angle(self, v1, v2):
        d = np.dot(v1, v2)
        m1 = np.linalg.norm(v1)
        m2 = np.linalg.norm(v2)
        angle = np.arccos((d / (m1 * m2)))
        return angle

    # Calculate the relevant joint angles from the image
    def detect_joint_angles(self):

        ##unit vectors
        x = np.array([1, 0, 0])
        y = np.array([0, 1, 0])

        yellow = self.detect_yellow(self.cv_image1, self.cv_image2)
        self.lastyellow = yellow
        blue = self.detect_blue(self.cv_image1, self.cv_image2)
        self.lastblue = blue
        red = self.detect_red(self.cv_image1, self.cv_image2)
        self.lastred = red


        link2 = blue - yellow

        blue2 = self.detect_blue(self.cv_image1, self.cv_image2)
        blue2[1] = 400

        yellow2 = self.detect_yellow(self.cv_image1, self.cv_image2)
        yellow2[1]= 400

        link2b = blue2-yellow2
        crossprod = np.cross(y,link2b)
        ja2= self.angle(crossprod,x)
        if link2[0] > 0:
            ja2= ja2 - np.pi
        if link2[0] < 0:
            ja2 = (np.pi) - ja2
        if ja2 >np.pi/2:
            ja2 = np.pi/2
        if ja2 < -np.pi/2:
            ja2 = -np.pi/2


        ja3 = np.pi/2 - self.angle(link2, y)


        blue3 = self.detect_blue(self.cv_image1, self.cv_image2)
        blue3[1] = 400

        red3 = self.detect_red(self.cv_image1, self.cv_image2)
        red3[1]=400

        link3 = red3 - blue3
        ja4 = self.angle(link2, link3)
        if ja4 > np.pi / 2:
            ja4 = np.pi - ja4
        if ja4 < (- np.pi / 2):
            ja4 = ja4 + np.pi
        project = np.dot(link3, crossprod)
        if project < 0:
           ja4 = -ja4
        ja4 = -ja4

        return np.array([ja2, ja3, ja4])

    def callback(self, img1, img2):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(img1, "bgr8")
            self.cv_image2 = self.bridge.imgmsg_to_cv2(img2, "bgr8")
        except CvBridgeError as e:
            print(e)

        data = self.detect_joint_angles()
        self.joint1 = Float64()
        self.joint1 = data[0]
        self.joint2 = Float64()
        self.joint2 = data[1]
        self.joint4 = Float64()
        self.joint4 = data[2]

        im1 = cv2.imshow('window1', self.cv_image1)
        cv2.waitKey(1)


        im2 = cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(1)

        # Publish the results
        try:
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            self.joint2_pub.publish(self.joint2)
            self.joint3_pub.publish(self.joint3)
            self.joint4_pub.publish(self.joint4)
        except CvBridgeError as e:
            print(e)


# call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
