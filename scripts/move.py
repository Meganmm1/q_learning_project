#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math
import numpy as np

from q_learning_project.msg import RobotMoveObjectToTag
from q_learning_project.msg import RobotArmMovement



class ArmMove(object):

    def __init__(self):
         # initialize this node
        rospy.init_node('move arm')

        # Traffic status subscriber
        rospy.Subscriber("/q_learning/arm_movement", RobotArmMovement, self.movement_received)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)
        print("ready")

    def movement_received(self,data):
        
        pickup_object = [-1,15,18,-29]
        lift_object = [-1,39,13,-29]
        gripper_open = [0.005,0.005]
        gripper_closed = [-0.002,-0.002]

        if data.direction == 1:
            #open gripper
            self.move_group_gripper.go(gripper_open)
            self.move_group_gripper.stop()

            #move arm down
            self.move_group_arm.go(pickup_object,wait=True)
            self.move_group_arm.stop()

            #close gripper
            self.move_group_gripper.go(gripper_closed)
            self.move_group_gripper.stop()

            #move arm down
            self.move_group_arm.go(lift_object,wait=True)
            self.move_group_arm.stop()
        
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    robot = ArmMove()
    robot.run()


