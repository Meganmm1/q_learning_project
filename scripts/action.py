#!/usr/bin/env python3

import rospy
import numpy as np
import os
from random import choice
import time
import moveit_commander

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
        #initialize node
        rospy.init_node("actions")

        #fetch pre-build action matrix
        self.action_matrix =  np.loadtxt(path_prefix + "action_matrix.txt")        

        #fetch actions 
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

        #import ar tag library
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        
        #ROBOT ARM CODE
        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        #self.move_group_arm.go(np.deg2rad([-1,-39,13,-29]))
        self.move_group_arm.go(np.deg2rad([-1,15,18,-29]))
        print("ready")

        self.found_color = False   

        #ACTION CODE
        #initialize action as -1
        self.action = -1
        #initialize angle of minimum distance as 0
        self.min_ang = 0
        
        self.matrix = np.loadtxt(open("q_matrix.csv", "rb"), delimiter=",", skiprows=0) #read in q-matrix
        print(self.matrix)
        self.image_process = False
        while not self.image_process: #wait for first image to be processed for actions to start
            i = 1
        if self.image_process == True:
            self.choose_actions()
        
        

        
        
            

    def image_callback(self,msg):
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        #print("image_recieved")
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        self.image = image #save image to self.image so other functions can access

        self.image_process = True 
        return
    
    
    def scanner_callback(self,data):
        non_zero = [] #make array with array of all non zero values between -10 and 10 degrees
        min_dist = 1000
        min_angle = 0
        for i in range(-10,11):
            if data.ranges[i] != 0:
                non_zero.append(data.ranges[i])
                if data.ranges[i] < min_dist:
                    min_angle = i
                    min_dist = data.ranges[i]
        if len(non_zero) > 0: 
            #print(non_zero)
            self.distance_object = min_dist #set the distance to the object as the minimum of non zero values
            #self.distance_object = np.average(non_zero)#data.ranges[0]#sum/7
        else:
            self.distance_object = 1
        self.min_ang = min_angle #set minimum angle

    def find_color(self):  
        print("look for color")
        if self.action != -1: #only run code after action has been initialized
            image = self.image 
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            #read image in
            
            action = (self.actions[self.action]) #get corresponding action for number
            print("action",action)
            color = action['object']    
            ar_tag = action['tag']
            #set bounds for the correct color
            if color == "pink":
                    lower = numpy.array([147, 50, 160])
                    upper = numpy.array([168, 255, 255])
            elif color == "green":
                    lower = numpy.array([30, 50, 160])
                    upper = numpy.array([60, 255, 255])
            else:
                    lower = numpy.array([85, 50, 160])
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
                
                kp = .20/w
                new_ang = kp*(w*.5 - 5 - cx)
                print("new ang " + str(new_ang))
                print("distance " + str(self.distance_object))
                print(new_ang)
                if abs(new_ang) < 0.008 and not self.found_color:
                    self.found_color = True
                    stop = Twist()
                    self.pub_twist.publish(stop)
                    rospy.sleep(1)
                    self.move_arm(0)
                    rospy.sleep(1)
                elif abs(new_ang) > 0.5:
                    self.found_color = False
                if self.found_color: #if it has found the color and gotten close then start moving forward
                    print("has found color")
                    kp2 = 0.15
                    rospy.sleep(1)
                    if self.distance_object > 0.23: #greater than stopping distance
                        #rospy.sleep(1)
                        if self.distance_object > 0.5:
                            print("far away")
                            new_lin = 0.05
                        else:
                            new_lin = kp2 * (self.distance_object-0.16)
                    else:
                        #close to object
                        stop = Twist()
                        self.pub_twist.publish(stop)
                        #pick up object
                        print("close enough")
                        #self.final_rotate()
                        self.move_arm(1)
                        #move back a bit
                        myTwist = Twist(linear = Vector3(-0.13,0,0), angular = Vector3(0,0,0))
                        self.pub_twist.publish(myTwist)
                        rospy.sleep(1)
                        #self.find_ar()
                        return True
                else:
                    new_lin = 0
                if self.found_color:
                    kp = .4*.017
                    print("adjustments, ang: " + str(self.min_ang+3))
                    new_ang = kp*(self.min_ang+1)
                else:
                    new_ang = kp*(w*.5 - 5 - cx)
                move_towards_color= Vector3(0,0,new_ang)
                my_twist = Twist(
                        linear = Vector3(new_lin,0,0), 
                        angular = move_towards_color
                )
                
                rospy.sleep(1)
                self.pub_twist.publish(my_twist)
                return False    

            else:
                print("color not showing up")
                #if color not sensed, keep turning
                my_twist = Twist(
                    linear=Vector3(0.0, 0, 0),
                    angular=Vector3(0, 0, 0.15)
                )
                # allow the publisher enough time to set up before publishing the first msg
                rospy.sleep(1)
                # publish the message
                self.pub_twist.publish(my_twist)
                print("turn published")
                
                print("stop")
                return False
        else:
            return False
    
    def move_arm(self,direction):
        pickup_object = np.deg2rad([-1,15,18,-29])
        lift_object = np.deg2rad([-1,-39,13,-29])
        #convert to rad
        gripper_open = [0.018,0.018]
        gripper_closed = [-0.002,-0.002]

        if direction == 0: #position arm to pick up
            #open gripper
            
            self.move_group_gripper.go(gripper_open)
            self.move_group_gripper.stop()
            print("opened gripper")
            rospy.sleep(2)
          

        elif direction == 1: #grap object and lift up
            
            #close gripper
            self.move_group_gripper.go(gripper_closed)
            self.move_group_gripper.stop()
            print("closed gripper")
            rospy.sleep(2)

            #move arm up
            self.move_group_arm.go(lift_object)
            rospy.sleep(2)
            self.move_group_arm.stop()
            print("arm_up")

            `
        elif direction == 2: #put object down
            
            #move arm down
            self.move_group_arm.go(pickup_object)
            rospy.sleep(1)
            self.move_group_arm.stop()
            print("put_down")
            rospy.sleep(2)

            #open gripper
            self.move_group_gripper.go(gripper_open)
            self.move_group_gripper.stop()
            print("opened gripper")
            rospy.sleep(1)

            #self.move_group_arm.go(np.deg2rad([-1,-39,13,-29]))
            #rospy.sleep(1)
            print("ready")


    def find_ar(self):
        rospy.sleep(1) #sleep to allow robot to reset
        distance_from_ar = .3 #set ideal distance to ar
        kp = 0.1 #set kp value
        print("look for ar tag")
        image= self.image
        # turn the image into a grayscale
        grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #initializes the ar tag
        ar_tag = self.actions[self.action]['tag']
        aruco_dict = self.aruco_dict
        # search for tags from DICT_4X4_50 in a GRAYSCALE image
        corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, aruco_dict)
        num_tag = 0
        #intdex into the corners based off the ids and then you index into the zeros column and
        #take the average of the x values
        w = self.w
        if len(corners) > 0:
            #if length corners greater than 0, at least one tag found
            print("at least one tag found")
            for i in range (len(corners)):
                print(ids[i])
                if ids[i][0] == ar_tag: #check if the ar tag is correct
                    print("correct ar tag")
                    x_sum = 0
                    for j in corners[i][0]:
                        x_sum += j[0]
                    x_avg = x_sum / 4 #average the x value of all four corners of the ar tag
                    kp = .075/w
                    #use proportional control to calculate angle to turn towards ar tag
                    new_ang = kp*(w*.5 - x_avg) 
                    if abs(new_ang) > 0.0075: 
                        #if angle required to turn towards tag is large than have robot turn
                        print("turning towards ar")
                        print(new_ang)
                        move_towards_ar= Vector3(0,0,new_ang)
                        my_twist = Twist(
                            linear = Vector3(0,0,0), 
                            angular = move_towards_ar
                        )
                        rospy.sleep(1)
                        #publish turn
                        self.pub_twist.publish(my_twist)
                        return False #return false because still not facing
                    else:
                        #if angle is small, than have robot move forward towards tag
                        kp = 0.1
                        rospy.sleep(1)
                        while self.distance_object > 0.45:
                            #use porportional control to move closer to object in front 
                            print(self.distance_object)
                            new_lin = kp * (self.distance_object-0.3)
                            move_foward = Vector3(new_lin, 0,0)
                            rospy.sleep(1)
                            self.pub_twist.publish(linear = move_foward, angular = Vector3(0,0,0))
                            print("moving towards ar")
                        stop = Twist()
                        self.pub_twist.publish(stop)
                        #put down object
                        print("close enough")
                        self.move_arm(2)
                        myTwist = Twist(linear = Vector3(-0.2,0,0), angular = Vector3(0,0,0))
                        rospy.sleep(1)
                        self.pub_twist.publish(myTwist)
                        self.found_color = False
                        return True #return true after ar completed
                else:
                    print("ar not showing up")
                    #if ar code not found keep spining
                    my_twist = Twist(
                        linear=Vector3(0.0, 0, 0),
                        angular=Vector3(0, 0, 0.05)
                    )
                    # allow the publisher enough time to set up before publishing the first msg
                    rospy.sleep(1)
                    # publish the message
                    self.pub_twist.publish(my_twist)
                    print("turn published")
                    print("stop")
                    return False
        else:
                    print("ar not showing up")
                    my_twist = Twist(
                        linear=Vector3(0.0, 0, 0),
                        angular=Vector3(0, 0, 0.15)
                    )
                    # allow the publisher enough time to set up before publishing the first msg
                    rospy.sleep(1)
                    # publish the message
                    self.pub_twist.publish(my_twist)
                    print("turn published")
                    #rospy.sleep(1)
                    #stop = Twist()
                    #self.pub_twist.publish(stop)
                    print("stop")
                    return False
        #pink goes to 3
        #green goes to 1
        #blue to 2

    def choose_actions(self):
        print("choose_action")
        
        matrix = self.matrix
        curr_state = 0 #start state at origin
        for i in range(3): #robot must complete three actions
            action = 0
            max_q = 0 
            #find what the maximum q value for the currect state is and save corresponding action
            for i in range(9):
                if matrix[curr_state][i] > max_q:
                    action = i
                    max_q = matrix[curr_state][i]
            self.action = action #set action to maximum action
            print("action choosen " + str(action))
            color_completed = False
            while not color_completed:
                color_completed = self.find_color()
                #call function to find and pick up correct color
            print("picked up color")
            ar_completed = False
            while not ar_completed:
                ar_completed = self.find_ar()
                #call function to find correct ar tag and place object in front of it
            print("placed at ar tag")
            ## initalize a list and then pop them off the list to store the action values

            next_state = 0
            #update next state based on which action was chosen
            while self.action_matrix[curr_state][next_state] != action and next_state < 64:
                next_state += 1
            curr_state = next_state
            print("updated state")
        #stop robot
        myTwist = Twist()
        self.pub_twist.publish(myTwist)
    def run(self):
        rospy.spin()


if __name__== '__main__':
    node = Action()
    node.run()
