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

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        
        #ROBOT ARM CODE
        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go(np.deg2rad([-1,-39,13,-29]))
        print("ready")

        self.found_color = False   

        #ACTION CODE
        self.action = -1
        self.min_ang = 0
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
        non_zero = []
        min_dist = 1000
        min_angle = 0
        for i in range(-10,10):
            if data.ranges[i] != 0:
                non_zero.append(data.ranges[i])
                if data.ranges[i] < min_dist:
                    min_angle = i
                    min_dist = data.ranges[i]
        if len(non_zero) > 0:
            #print(non_zero)
            self.distance_object = min_dist
            #self.distance_object = np.average(non_zero)#data.ranges[0]#sum/7
        else:
            self.distancce_object = 0.3
        self.min_angle = min_angle
        '''else:
            self.distance_object = 0.5
        #print("scanner: " + str(self.distance_object))
        non_zero = []
        for i in [-1,1]:
            if data.ranges[i] != 0:
                non_zero.append(data.ranges[i])
        self.distance_object = np.average(non_zero)
        self.min_ang = 0 '''

    def final_rotate(self):
        print("final rotate")
        print(self.min_ang)
        kp = 1.2*.017
        while self.min_ang != 0:
            print("final adjustments")
            new_ang = kp*(self.min_ang)
            myTwist = Twist(linear = Vector3(),angular = Vector3(0,0,new_ang))
            self.pub_twist.publish(new_ang)

    def find_color(self):  
        print("look for color")
        if self.action != -1:
            image = self.image
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            action = (self.actions[self.action]) #get corresponding action for number
            print("action",action)
            color = action['object']    
            ar_tag = action['tag']
            #set bounds for each color
            if color == "pink":
                    lower = numpy.array([147, 63, 160])
                    upper = numpy.array([168, 255, 255])
            elif color == "green":
                    lower = numpy.array([30, 50, 160])
                    upper = numpy.array([60, 255, 255])
            else:
                    lower = numpy.array([85, 63, 160])
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
                
                kp = .23/w
                new_ang = kp*(w*.5 - 5 - cx)
                print("new ang " + str(new_ang))
                print("distance " + str(self.distance_object))
                print(new_ang)
                if abs(new_ang) > 0.006:
                    self.found_color = True
                elif abs(new_ang) > 0.05:
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
                        self.final_rotate()
                        self.move_arm(1)
                        r = rospy.Rate(2)
                        myTwist = Twist(linear = Vector3(-0.,0,0), angular = Vector3(0,0,0))
                        self.pub_twist.publish(myTwist)
                        rospy.sleep(1)
                        #self.find_ar()
                        return True
                else:
                    new_lin = 0
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
        else:
            return False
            #Will loop through all the angles and store the angle at which 
            #the closest object to the robot is located

         

            #have robot turn towards correct colored object
            #have robot get to proper distance to colored object
            #have robot pickup object
            #have robot turn towards correct ar tag
            #have robot go towards ar tag
            #have robot place object
    
    def move_arm(self,direction):
        pickup_object = [-1,15,18,-29]
        lift_object = [-1,-39,13,-29]
        #convert to rad
        for i in range(4):
            pickup_object[i] = np.deg2rad(pickup_object[i])
            lift_object[i] = np.deg2rad(lift_object[i])
        gripper_open = [0.018,0.018]
        gripper_closed = [-0.002,-0.002]

        if direction == 1:
            #open gripper
            
            self.move_group_gripper.go(gripper_open)
            self.move_group_gripper.stop()
            print("opened gripper")
            rospy.sleep(2)

            #move arm down
            
            self.move_group_arm.go(pickup_object)
            print("picked_up")
            rospy.sleep(1)
            self.move_group_arm.stop()
            rospy.sleep(2)
            print("moved_arm")

            #close gripper
            self.move_group_gripper.go(gripper_closed)
            self.move_group_gripper.stop()
            print("closed gripper")
            rospy.sleep(2)

            #move arm up
            self.move_group_arm.go(lift_object)
            rospy.sleep(1)
            self.move_group_arm.stop()
            print("arm_down")
            rospy.sleep(2)

            #code to make it move down
        elif direction == 2:
            
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

            self.move_group_arm.go(np.deg2rad([-1,-39,13,-29]))
            rospy.sleep(1)
            print("ready")


    def find_ar(self):
        rospy.sleep(1)
        distance_from_ar = .3
        kp = 0.1
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
            print("at least one tag found")
            for i in range (len(corners)):
                print(ids[i])

                if ids[i][0] == ar_tag:
                    print("correct ar tag")
                    x_sum = 0
                    for j in corners[i][0]:
                        x_sum += j[0]
                    x_avg = x_sum / 4
                    kp = .075/w
                    new_ang = kp*(w*.5 - x_avg)
                    if abs(new_ang) > 0.0075:
                        print("turning towards ar")
                        new_ang = kp*(w*.5 - x_avg)
                        print(new_ang)
                        move_towards_color= Vector3(0,0,new_ang)
                        my_twist = Twist(
                            linear = Vector3(0,0,0), 
                            angular = move_towards_color
                        )
                        rospy.sleep(1)
                        self.pub_twist.publish(my_twist)
                        return False
                    else:
                        kp = 0.1
                        rospy.sleep(1)
                        while self.distance_object > 0.4:
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
                        r = rospy.Rate(2)
                        myTwist = Twist(linear = Vector3(-0.1,0,0), angular = Vector3(0,0,0))
                        self.pub_twist.publish(myTwist)
                        rospy.sleep(1)
                        return True
                else:
                    print("ar not showing up")
                    my_twist = Twist(
                        linear=Vector3(0.0, 0, 0),
                        angular=Vector3(0, 0, 0.05)
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

        #distance to x_average ?
        while self.distance_object  > 0.2:
            new_lin = kp * (self.distance_object-distance_from_ar)
            move_foward = Vector3(new_lin, 0,0)
            rospy.sleep(1)
            self.pub_twist.publish(linear = move_foward, angular = Vector3(0,0,0))
        self.move_arm(2)
                        
        #pink goes to 3
        #green goes to 1
        #blue to 2

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
            color_completed = False
            while not color_completed:
                color_completed = self.find_color()
            print("picked up color")
            ar_completed = False
            while not ar_completed:
                ar_completed = self.find_ar()
            print("placed at ar tag")
            ## initalize a list and then pop them off the list to store the action values

            next_state = 0
            while self.action_matrix[curr_state][next_state] != action and next_state < 64:
                next_state += 1
            curr_state = next_state
            print("updated state")
        myTwist = Twist()
        self.pub_twist.publish(myTwist)
    def run(self):
        rospy.spin()


if __name__== '__main__':
    node = Action()
    node.run()