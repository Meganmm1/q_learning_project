#!/usr/bin/env python3

import rospy
import numpy as np
import os
#added this
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import QMatrix
from q_learning_project.msg import QMatrixRow
from q_learning_project.msg import RobotMoveObjectToTag

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"




class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))
        print("see states", self.states)
    
    def get_reward(self, data):
        #callback function?
        #get what the reward was and based off this we update the Q MATRIX
        self.actions = data
    
    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        
        # subscribe to the rewards
        rospy.Subscriber(self.actions, QLearningReward, self.get_reward)

        #pusblish to the robot action (not sure about the first one if we have to define)
        self.robot_action_pub = rospy.Publisher("/action_states/", RobotMoveObjectToTag, queue_size=10 )
        
        #publish to the q matrix
        self.q_matrix_pub = rospy.Publisher("q_matrix", RobotMoveObjectToTag, queue_size=10 )
        
        #writing some stuff because idk how to create a matrix lmao 
        starter_matrix = np.array[]
        for x in range 64:
            columns= []
            for y in range 9:
                 columns.append[]
            starter_matrix.append(columns)

        
        #subscribe to the reward and publish to the action
        #update the q matrix based of the algorithm where Q(st)



        #the q matrix updates the q values which are dependnet on the reward.
        
        return

if __name__ == "__main__":
    node = QLearning()
