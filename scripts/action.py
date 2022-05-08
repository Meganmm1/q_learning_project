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

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

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
        self.matrix = np.loadtxt(open("q_matrix.csv", "rb"), delimiter=",", skiprows=0)
        print(self.matrix)
        self.image_process = False
        while not self.image_process:
            i = 1
        if self.image_process == True:
            print("hi")
            self.choose_actions()
        
        
            

    def image_callback(self,msg):
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        #print("image_recieved")
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        self.image = image

        self.image_process = True
        #self.choose_actions()
        #print(self.image_process)
        return
        
    
    def scanner_callback(self,data):
        sum = 0
        for i in [-3,3]:
            sum += data.ranges[i]
        self.distance_object = sum/7
        
        #print("scanner: " + str(self.distance_object))



    def find_color(self):  
        print("look for color")
        if self.action != -1:
            image = self.image
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            action = (self.actions[self.action]) #get corresponding action for number
            print("action",action)
            color = action['object']    
            ar_tag = action['tag']
            #i think we would only need one lower and upper bound as we will only be 
            #be looking at one color at a time
            if color == "pink":
                    lower = numpy.array([147, 63, 159])
                    upper = numpy.array([168, 192, 255])
            elif color == "green":
                    lower = numpy.array([37, 63, 192])
                    upper = numpy.array([65, 192, 255])
            else:
                    lower = numpy.array([85, 63, 255])
                    upper = numpy.array([104, 255, 255])
                
            print("COLOR: " + str(color))
            print("ar tag: " + str(ar_tag))
                # this erases all pixels that aren't pink,green,blue
            mask = cv2.inRange(hsv, lower, upper)             
            
                #set the dimensions of the image
            h,w,d = image.shape
            
            self.w = w
            print("width = " + str(self.w))
                # using moments() function, the center of the yellow pixels is determined
            M = cv2.moments(mask)
                #im pretty sure this find the center of any of the colored pixels based off the mask
                # if there are any pink,green,blue pixels found
                
            if M['m00'] > 0:
                # center of the pixels in the image
                self.cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            
                w = self.w
                cx = self.cx
                #this is if we want the circle to visualize
                #cv2.circle(image, (cx, cy), 20, (0,0,255), -1) 
                kp = .2/w
                new_ang = kp*(w*.5 - cx)
                if abs(new_ang) > 0.005:
                    new_ang = kp*(w*.5 - cx)
                    print(new_ang)
                    move_towards_color= Vector3(0,0,new_ang)
                    my_twist = Twist(
                        linear = Vector3(0,0,0), 
                        angular = move_towards_color
                    )
                    rospy.sleep(1)
                    self.pub_twist.publish(my_twist)
                    
                    print("first publishing")
                else:
                    kp = 0.1
                    while self.distance_object  > 0.097:
                        print(self.distance_object)
                        new_lin = kp * (self.distance_object-.097)
                        move_foward = Vector3(new_lin, 0,0)
                        rospy.sleep(1)
                        self.pub_twist.publish(linear = move_foward, angular = Vector3(0,0,0))
                        print("second publishing")
                    #pick up object
                    print("close enough")
                    return True
            else:
                print("color not showing up")
                my_twist = Twist(
                    linear=Vector3(0.0, 0, 0),
                    angular=Vector3(0, 0, 0.15)
                )
                # allow the publisher enough time to set up before publishing the first msg
                rospy.sleep(1)
                # publish the message
                self.pub_twist.publish(my_twist)
                print("turn published")
                rospy.sleep(1)
                stop = Twist()
                self.pub_twist.publish(stop)
                print("stop")
                return False
                
            
            
        return False



            #Will loop through all the angles and store the angle at which 
            #the closest object to the robot is located

         

            #have robot turn towards correct colored object
            #have robot get to proper distance to colored object
            #have robot pickup object
            #have robot turn towards correct ar tag
            #have robot go towards ar tag
            #have robot place object
        print("done")

    def choose_actions(self):
        print("choose_action")
        matrix = self.matrix
        curr_state = 0
        for i in range(3):
            action = 0
            max_q = 0
            for i in range(9):
                if matrix[curr_state][i] > max_q:
                    action = i
                    max_q = matrix[curr_state][i]
            self.action = action #set action to maximum
            print("action choosen " + str(action))
            action_completed = False
            while not action_completed:
                action_completed = self.find_color()
                
            next_state = 0
            while self.action_matrix[curr_state][next_state] != action and next_state < 64:
                next_state += 1
            curr_state = next_state
            print("updated state")
        
    def run(self):
        rospy.spin()


if __name__== '__main__':
    node = Action()
    node.run()
