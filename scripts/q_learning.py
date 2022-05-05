#!/usr/bin/env python3

import rospy
import numpy as np
import os
from random import choice
import time
import csv
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

        self.reward = 0
        # subscribe to the rewards
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.get_reward)

        #pusblish to the robot action (not sure about the first one if we have to define)
        #self.robot_action_pub = rospy.Publisher("/action_states/", RobotMoveObjectToTag, queue_size=10 )
        self.robot_action_pub = rospy.Publisher("q_learning/robot_action", RobotMoveObjectToTag, queue_size=10 )
        
        #publish to the q matrix
        self.q_matrix_pub = rospy.Publisher("q_learning/q_matrix", QMatrix, queue_size=10 )


        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix =  np.loadtxt(path_prefix + "action_matrix.txt")

        #print(self.action_matrix)

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
    
        print(self.actions)


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))
        self.reward_updated = False
        self.q_matrix = self.init_matrix()
        self.update_q_matrix()
        self.save_q_matrix()
        self.choose_path()
       # print("see states", self.states)
            
        
        
    
    def get_reward(self, data):
        #callback function?
        #get what the reward was and based off this we update the Q MATRIX
        #self.actions = data
         
        print("reward!")
        print(data.reward)
        self.reward_updated = True
        self.reward = data.reward
        return 
    
    def init_matrix(self):
        #initializes q-matrix with 0's or -1 if action is invalid

        #makes a matrix of 64 rows and 9 colums filled with -1
        rows = 64
        cols = 9
        starter_matrix = np.full((rows,cols),-1)
        
        #print(starter_matrix)

        #if an action is valid for that state change to 0
        for i in range (64):
            for j in range(64):
                #j for us is an action
                #j for action matrix is opposite state
                action = int(self.action_matrix[i][j])
                #print(action)
                if action != -1:
                    starter_matrix[i][action]= 0
        print(starter_matrix)
        return(starter_matrix)

    def first_action(self):
        self.publish_action(0)
        self.publish_action(4)
        self.publish_action(8)
        

    def publish_action(self,num_act):
        robot_act = (self.actions[num_act]) #get corresponding action
        #print(robot_act)
        time.sleep(1)            
        print("about to publish")
        self.robot_action_pub.publish(RobotMoveObjectToTag
            (robot_object = robot_act['object'],tag_id = robot_act['tag']))
        print("published")

    def choose_action(self, curr_state):
        #choose a random action
        matrix = self.q_matrix
        rand_act = -1
        all_actions = matrix[curr_state] #all actions (possible or not) from current state
        possible_actions = []
        for i in range(9):
            if all_actions[i] != -1: #check if any of the 9 actions is possible (not -1)
                possible_actions.append(i) # if possible add to array
        if len(possible_actions) > 0:
            rand_act = choice(possible_actions) #randomly choose one of possible actions
        return rand_act

    def update_q_matrix(self):
        #implement Q ALGORITHM 
        matrix = self.q_matrix
        gamma = 0.8
        alpha = 1
        num_constant = 0 #num of times matrix has not changed
        past_matrix = np.full((64,9),-1)
        self.first_action()
        while num_constant < 150: #continue looping until matrix has not changed for set num of trajectories
            trajectory = True
            #set initial state to origin
            curr_state = 0
            while trajectory:            
                rand_act = self.choose_action(curr_state)
                if rand_act != -1:
                    #print(rand_act)
                    #publish rand action
                    self.publish_action(rand_act)
                    time.sleep(1)
                    while not self.reward_updated:
                        var = 3
                        print("waiting")
                    #get reward??
                    #reward = QLearningReward()
                    reward = self.reward #just arbitrarily set it as 10 for now
                    #how do you get next state based on action?
                    if reward != 0:
                        print("not zero!!!")
                    next_state = 0
                    while self.action_matrix[curr_state][next_state] != rand_act and next_state < 64:
                        next_state += 1
                    print("next state" +str(next_state))
                    
                    old_q = matrix[curr_state][rand_act]
                    matrix[curr_state][rand_act] = old_q + alpha * (reward + gamma * max(matrix[next_state]) - old_q)
                    #when alpha = 1
                    #matrix[curr_state][rand_act] = reward + gamma * max(matrix[next_state])
                    
                    print("updating row: " +str(curr_state) + " col: " + str(rand_act))
                    print("new value: " + str(matrix[curr_state][rand_act]))
                    if matrix[curr_state][rand_act]:
                        print(matrix)
                    self.q_matrix = matrix
                    #print(matrix)
                    curr_state = next_state
                else:
                    if matrix.all() == past_matrix.all():
                        num_constant += 1
                        print(num_constant)
                    past_matrix = matrix
                    trajectory = False
                    print("end of trajectory")
        self.q_matrix = matrix

    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining

        #subscribe to the reward and publish to the action
        #update the q matrix based of the algorithm where Q(st)

        matrix = self.q_matrix
        print("saving")
        with open("q_matrix.csv","w+") as my_csv:
            csvWriter = csv.writer(my_csv,delimiter=',')
            csvWriter.writerows(matrix)
        #the q matrix updates the q values which are dependnet on the reward.
        return




if __name__ == "__main__":
    node = QLearning()
    node.save_q_matrix
