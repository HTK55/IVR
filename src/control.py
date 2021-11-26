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
        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)
        # initialize a publisher to send robot end-effector position
        self.end_effector_pub = rospy.Publisher("end_effector_prediction", Float64MultiArray, queue_size=10)
        # initialize a publisher to send desired trajectory
        self.trajectory_pub = rospy.Publisher("trajectory", Float64MultiArray, queue_size=10)
        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback
        # function to recieve data
        # initialize a subscriber to recieve messages rom a topic named /robot/camera2/image_raw and use callback
        # function to recieve data
        self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw", Image)
        self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw", Image)
        self.joint1_sub = message_filters.Subscriber("joints_pos/joint_angle_1", Float64)
        self.joint3_sub = message_filters.Subscriber("joints_pos/joint_angle_3", Float64)
        self.joint4_sub = message_filters.Subscriber("joints_pos/joint_angle_4", Float64)
        self.ts = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2, self.joint1_sub, self.joint3_sub, self.joint4_sub], 10)
        self.ts.registerCallback(self.callback)
        self.rate = rospy.Rate(1)
        # record the begining time
        self.time_trajectory = rospy.get_time()
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
        # initialize error and derivative of error for trajectory tracking
        self.error = np.array([0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0], dtype='float64')

    # detect robot end-effector from the image
    def detect_end_effector(self,image):
        a = self.pixel2meter(image)
        endPos = a * (self.detect_yellow(image) - self.detect_red(image))
        return endPos

    # Define a circular trajectory
    def trajectory(self):
        # get current time
        cur_time = np.array([rospy.get_time() - self.time_trajectory])
        x_d = float(5.5* np.cos(cur_time * np.pi/100))
        y_d = float(5.5 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
        return np.array([x_d, y_d])

      # Calculate the forward kinematics
    def forward_kinematics(self):
        end_effector_x = np.array([[np.cos(self.joint1),-np.sin(self.joint1),0,0],[np.sin(self.joint1),np.cos(self.joint1),0,0],[0,0,4,0],[0,0,0,1]])
        end_effector_y = np.array([[np.cos(self.joint3),0,np.sin(self.joint3),0],[0,1,0,0],[-np.sin(self.joint3),0,np.cos(self.joint3),0],[0,0,0,1],])
        end_effector_z = np.array([[1,0,0,3.2],[0,np.cos(self.joint4),-np.sin(self.joint4),0],[0,np.sin(self.joint4),np.cos(self.joint4),0],[0,0,0,1]])

        end_effector = np.matmul(end_effector_x,end_effector_y,end_effector_z)
        return end_effector


    def callback(self, img1, img2, joint1, joint3, joint4):
        # Recieve the images
        try:
           self.cv_image1 = self.bridge.imgmsg_to_cv2(img1, "bgr8")
           self.cv_image2 = self.bridge.imgmsg_to_cv2(img2, "bgr8")
        except CvBridgeError as e:
           print(e)


        # compare the estimated position of robot end-effector calculated from images with forward kinematics(lab 3)
        x_e = self.forward_kinematics(self.cv_image1, self.cv_image2)
        x_e_image = self.detect_end_effector(self.cv_image1, self.cv_image2)
        self.end_effector=Float64MultiArray()
        self.end_effector.data= x_e_image

        # send control commands to joints (lab 3)
        q_d = self.control_open(self.cv_image1, self.cv_image2)
        self.joint1=Float64()
        self.joint1.data= q_d[0]
        self.joint2=Float64()
        self.joint2.data= q_d[1]
        self.joint3=Float64()
        self.joint3.data= q_d[2]

        # Publishing the desired trajectory on a topic named trajectory(lab 3)
        x_d = self.trajectory()    # getting the desired trajectory
        self.trajectory_desired= Float64MultiArray()
        self.trajectory_desired.data=x_d

        # Publish the results
        try:
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            self.joints_pub.publish(self.joints)
            self.end_effector_pub.publish(self.end_effector)
            self.trajectory_pub.publish(self.trajectory_desired)
            self.robot_joint1_pub.publish(self.joint1)
            self.robot_joint3_pub.publish(self.joint2)
            self.robot_joint4_pub.publish(self.joint3)
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
