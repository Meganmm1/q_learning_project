#!/usr/bin/env python3

import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class SpinCircles(object):
    """ This node publishes ROS messages containing the 3D coordinates of a single point """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('spin_circles')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        # setup the Twist message we want to send
        my_twist = Twist(
            linear=Vector3(0.1, 0, 0),
            angular=Vector3(0, 0, 0.25)
        )

        # allow the publisher enough time to set up before publishing the first msg
        rospy.sleep(1)

        # publish the message
        self.robot_movement_pub.publish(my_twist)
        print("published")

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = SpinCircles()
    node.run()