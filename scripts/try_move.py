#!/usr/bin/env python3

import rospy
from q_learning_project.msg import RobotArmMovement
import numpy as np

class Direct(object):
    def __init__(self):
        # Set up traffic status publisher
        self.arm_pub = rospy.Publisher("/q_learning/arm_movement", RobotArmMovement)

        # Counter to loop publishing direction with
        rospy.sleep(1)

    def run(self):
        # Once every 10 seconds
        rate = rospy.Rate(0.1)
        move = 1
        self.arm_pub.publish(move)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('traffic_controller')
    traffic_node = Direct()
    traffic_node.run()