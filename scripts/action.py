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

        #declares node as a subscriber to scan topic and makes a callback
        # function named interpret_scan
        rospy.Subscriber("/scan", LaserScan, self.scanner_callback)


        # cmd_vel publisher 
        self.pub_twist = rospy.Publisher('/cmd_vel',Twist,queue_size = 10)

        self.action = -1
        self.action_complete = False
        self.image


    def image_callback(self,msg)
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
    def scanner_callback(self,data)
        self.distance_object = data.ranges[0]


    def find_color(self)    
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if self.action != -1:
            action = (self.actions[self.action]) #get corresponding action for number
            color = action['object']
            ar_tag = action['tag']
            #i think we would only need one lower and upper bound as we will only be 
            #be looking at one color at a time
            if color == "pink":
                lower = numpy.array([164, 63, 159])
                upper = numpy.array([164, 192, 255])
            elif color == "green"
                lower = numpy.array([60, 63, 192])
                upper = numpy.array([60, 192, 255])
            else color == "blue"
                lower = numpy.array([90, 63, 255])
                upper = numpy.array([104, 255, 255])

            # this erases all pixels that aren't pink,green,blue
            mask0 = cv2.inRange(hsv, lower, upper)             
           
            #set the dimensions of the image
            h,w,d = image.shape

            # using moments() function, the center of the yellow pixels is determined
            M = cv2.moments(final_mask)
            #im pretty sure this find the center of any of the colored pixels based off the mask
            # if there are any pink,green,blue pixels found
                if M['m00'] > 0:
                        # center of the pixels in the image
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])

            #this is if we want the circle to visualize
            #cv2.circle(image, (cx, cy), 20, (0,0,255), -1) 
            while new_ang != 0
                kp = .1/w
                new_ang = kp*(w*.5 - cx)
                move_towards_color= Vector3(0,0,new_ang)
                self.pub_twist.publish(linear =0, angular = move_towards_color)

            for i in range (360):
            #Will loop through all the angles and store the angle at which 
            #the closest object to the robot is located
            if data.ranges[i] < shortest_distance and not data.ranges[i]==0:
                shortest_distance = data.ranges[i]
                min_i = i

            if min_i < 10 or min_i > 350:
                new_lin = kp * (self.distance_object-.097)
                move_foward = Vector3(new_lin, 0,0)
                self.pub_twist.publish(linear = move_foward, angular = 0)
            
                





            #Will loop through all the angles and store the angle at which 
            #the closest object to the robot is located

         

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
        
    def run(self):
        rospy.spin()


if __name__== '__main__':
    node = Action()
    node.run()
