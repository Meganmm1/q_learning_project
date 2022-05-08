#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math
import numpy as np



class ArmMove(object):

    def __init__(self):
         # initialize this node
        rospy.init_node('move_arm')

        # Traffic status subscriber
        #rospy.Subscriber("/q_learning/arm_movement", QMatrixRow, self.movement_received)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go(np.deg2rad([-1,-39,13,-29]))
        print("ready")
   
    #move into the action file 
    
    def move_arm(self,direction):
        pickup_object = [-1,15,18,-29]
        lift_object = [-1,-39,13,-29]
        #convert to rad
        for i in range(4):
            pickup_object[i] = np.deg2rad(pickup_object[i])
            lift_object[i] = np.deg2rad(lift_object[i])
        gripper_open = [0.006,0.006]
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
            print("moved_arm")
            rospy.sleep(2)
           
            #close gripper
            self.move_group_gripper.go(gripper_closed)
            self.move_group_gripper.stop()
            print("closed gripper")
            rospy.sleep(2)

            #move arm up
            self.move_group_arm.go(lift_object)
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
            rospy.sleep(2)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    robot = ArmMove()
    rospy.sleep(2)
    robot.move_arm(1)
    rospy.sleep(2)
    robot.move_arm(2)
    robot.run()



