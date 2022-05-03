# q_learning_project

## Emilia Lim and Megan Morales

# Writeup
## Goal
- The goal of this project is to be able to correctly implement a Q-learning algorithm to get our robot to determine the best path to place three different objects into set locations. Another goal of this project is to get oriented with coding the robot's arm so that it can use the Q-learning alogrithm to place said objects. 

## High-level Description
- In order to determine which colored object goes into which tag we first initalized a 64 by 9 matrix that represented the possible states and actions that the robot could take. We randomly generated different actions where based off of these actions we subscribed to the possible rewards that can be recieved. Using the Q learning algorithm we updated our initalized matrix so that eventually the actions that contain the greatest reward values will be represented in the matrix and accurately represent the path that the robot should take. 
## Q-learning algorithm Description

### Selecting and executing actions for the robot (or phantom robot) to take
- We select and determine which action the robot should take in the choose_action function. 
- First we determine which action is possible based on the state that the robot is currently in. In order to select which action for the robot to take we used the function random.choice to randomly select an action for the robot to take. We used the publish_action function to publish the action into the robot's action function. Once we published this action we subscribed to the QLearningReward topic to determine the reward the robot would recieve upon moving from its current state to the next. This is repeated for multiple iterations up until the matrix converges. 

### Updating the Q-matrix
- We update the Q matrix in the update_q_matrix function. 
- After we select and execute the possible action for the robot to take we use the reward we found to update the matrix. If the reward is non-zero then we update the Q-matrix with a value using the Q-learning algorithm. If the reward is zero we still update the matrix with the zero value. Eventually, the matrix will converge to represent the Q values of the robots actions where the highest Q value represent the path that the robot should take to get the highest reward. 

### Determining when to stop iterating through the Q-learning algorithm
- We determine when to stop iterating through the Q-learning algorithm in the update_q_matrix function. 
- In order to determine this value we ran our code, which updated and printed our a matrix after each update, several times. We found that the value we set (150) was sufficient to update all the states that would lead to possible rewards and eventually allow our matrix to converge.




# Q-learning Algorithm
### Executing the Q-learning algorithm
- To execute the Q-learning algorithm, we will first create a Q-matrix with rows that represent all possible states (64) and each column representing all possible actions (9) which can be found in self.actions in q_learning.py. To fill in the matrix, we will start all of our trajectories in state 0 with all objects at the origin. From there we will choose a random action to transition into another state and use this to calculate an updated reward value in our matrix. 
- To test this, we will use the phantom robot node that was already made for us and we will continuously print out our matrix to check if it is updating properly.

### Determining when the Q-matrix has converged
- To determine when the matrix has converged, we will test if the values in the Q-matrix are constant for multiple iterations. If after multiple iterations, the Q-matrix has not changed, we will conclude that it has converged. Through trial and error we will determine what number of iterations seems reasonable, for now we could start with 10 iterations. 
- To test this, we will see if after 10 iterations with constant values in the Q-matrix results in accurate reward values. We will print out our matrix to see these values.

### Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
- Based on the converged Q-matrix, for whatever state the robot is currently in, we will use the Q-matrix to determine the highest reward value for the corresponding row for that state. Based on whatever column has the highest reward value, we will have the robot take that action. We will then use the action_matrix to update the robots current state and repeat. 
- We will test this code by seeing if given a set matrix the robot is able to choose the best action.  

# Robot perception
### Determining the identities and locations of the three colored objects
- In order to determine the identities and location of the objects, we will make use of the robot’s RGB camera to determine the color of the objects. To determine the location we will use the fiducials topic which makes use of AR tags as fiducial markers to determine the location of objects.
- To test this, we can visualize the camera to make sure the robot is correctly sensing the right objects. We can also see if the robot is able to correctly determine the location of the different objects in different set-ups. 

### Determining the identities and locations of the three AR tags
- We will use the imported ArUco dictionary and the robot’s camera to recognize the AR tags. To determine the location, we will use the estimated position generated by the aruco package. 
- To test this, we can test if the robot is able to correctly identify each individual AR tag. We will also test if the robot is able to correctly determine the location of the different tags in different set-ups. 

# Robot manipulation & movement
### Picking up and putting down the colored objects with the OpenMANIPULATOR arm
- We will use the information about manipulating the robot’s arm in lab F to determine how to control the robot’s arm using the MoveIt ROS package.
- We will test this by seeing if the robot is able to pick up and put down the objects. We will also use RVis to help visualize the way the arm is moving. 

### Navigating to the appropriate locations to pick up and put down the colored objects
- We will have the robot approach the determined position for a colored object and stop at a set distance that accounts for the arm’s motion in front of the object. After picking up the object, the robot will lift the object and go to the determined position for the AR tag. To get from one position to the next, we will have the robot take a direct path by determining the distance between the two locations and orienting itself to face the object it is going to. 
- We will test this by seeing if the robot is able to move to a set location using our code. 


### Timeline
- We would like to have a version of the Q-matrix code by Friday, April 29th, which would allow us to debug over the weekend. We will then turn in the intermediate deliverable on Tuesday.
- We would like to have robot perception working by May 5th and the robot manipulation and movement by the 8th, leaving us a few days to debug.

