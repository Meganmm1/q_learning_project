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

        self.action = -1
        self.action_complete = False


    def image_callback(self,msg)
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if self.action != -1:
            action = (self.actions[self.action]) #get corresponding action for number
            color = action['object']
            ar_tag = action['tag']
            #i think we would only need one lower and upper bound as we will only be 
            #be looking at one color at a time
            if color == "pink":
                lower_pink = numpy.array([164, 63, 159])
                upper_pink = numpy.array([164, 192, 255])
            elif color == "green"
                lower_green = numpy.array([60, 63, 192])
                upper_green = numpy.array([60, 192, 255])
            else color == "blue"
                lower_blue = numpy.array([90, 63, 255])
                upper_blue = numpy.array([104, 255, 255])

            # this erases all pixels that aren't pink,green,blue
            mask0 = cv2.inRange(hsv, lower_pink, upper_pink)             
            mask1 = cv2.inRange(hsv, lower_green, upper_green)
            mask2 = cv2.inRange(hsv, lower_blue, upper_blue)

            #have robot turn towards correct colored object
            #have robot get to proper distance to colored object
            #have robot pickup object
            #have robot turn towards correct ar tag
            #have robot go towards ar tag
            #have robot place object
            self.action_complete = True 

    def choose_actions(self):
        matrix = np.loadtxt(open("q_matrix.csv", "rb"), delimiter=",", skiprows=1)
        
        curr_state = 0
        for i in range(3):
            action = 0
            max_q = 0
            for i in range(9):
                if matrix[curr_state][i] > max_q:
                    action = i
                    max_q = matrix[curr_state][i]
            self.action = action #set action to maximum
            self.move_to_color()
            self.action_complete = False
            while not self.action_complete:  #wait for action to be completed
                var = 1
            next_state = 0
            while self.action_matrix[curr_state][next_state] != action and next_state < 64:
                next_state += 1
            curr_state = next_state
        



