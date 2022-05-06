#!/usr/bin/env python3

import rospy
import numpy as np
import os
from random import choice
import time
import csv


from q_learning_project.msg import QLearningReward
from q_learning_project.msg import QMatrix
from q_learning_project.msg import QMatrixRow
from q_learning_project.msg import RobotMoveObjectToTag
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

class Action(object):
    
    def __init__ (self):
        rospy.init_node("actions")

        #fetch pre-build action matrix
        self.action_matrix =  np.loadtxt(path_prefix + "action_matrix.txt")
        
        #open csv file ?
        with open('q_matrix.csv','r') as file:
            converged_matrix = csv.reader(file)
        

        #fetch actions (fix this?)
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))

         # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
        Image, self.image_callback)

        # cmd_vel publisher 
        self.pub_twist = rospy.Publisher('/cmd_vel',Twist,queue_size = 10)
        def image_callback(self, msg):

        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
        if self.actions.colors == "pink":
            lower_pink = numpy.array([164, 63, 159])
            upper_pink = numpy.array([164, 192, 255])
        elif self.actions.colors == "green"
            lower_green = numpy.array([60, 63, 192])
            upper_green = numpy.array([60, 192, 255])
        else self.actions.colors == "blue"
            lower_blue = numpy.array([90, 63, 255])
            upper_blue = numpy.array([104, 255, 255])

        # this erases all pixels that aren't pink,green,blue
        mask0 = cv2.inRange(hsv, lower_pink, upper_pink)             
        mask1 = cv2.inRange(hsv, lower_green, upper_green)
        mask2 = cv2.inRange(hsv, lower_blue, upper_blue)





