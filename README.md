# q_learning_project

## Emilia Lim and Megan Morales

# Writeup
## Goal
- The goal of this project is to be able to correctly implement a Q-learning algorithm to get our robot to determine the best path to place three different objects into set locations. Another goal of this project is to get oriented with coding the robot's arm so that it can use the Q-learning alogrithm to place said objects. 

## High-level Description
- In order to determine which colored object goes into which tag we first initalized a 64 by 9 matrix that represented the possible states and actions that the robot could take with cells set to 0 at possible action state pairs and -1 for pairs that are impossible. We then randomly selected actions for the robot to take and filled our matrix based off of which actions recieved rewards. Using the Q learning algorithm we updated our initalized matrix so that eventually the actions that contain the greatest reward values will be represented in the matrix and accurately represent the path that the robot should take. 

## Q-learning algorithm Description

### Selecting and executing actions for the robot (or phantom robot) to take
- We select and determine which action the robot should take in the choose_action function. 
- First we determine which action is possible based on the state that the robot is currently in by seeing which actions do not contain a value of -1 in our q-matrix. In order to select which action for the robot to take we used the function random.choice to randomly select an action for the robot to take. We used the publish_action function to publish the action into the robot's action function. Once we published this action we subscribed to the QLearningReward topic to determine the reward the robot would recieve upon moving from its current state to the next. This is repeated for multiple iterations up until the matrix converges. 

### Updating the Q-matrix
- We update the Q matrix in the update_q_matrix function. 
- After we select and execute the possible action for the robot to take, we use the reward we found to update the matrix. We then update the Q-matrix with a value using the Q-learning algorithm. If the reward is zero we still update the matrix with the zero value. Eventually, the matrix will converge to represent the Q values of the robots actions where the highest Q value represent the path that the robot should take to get the highest reward. 

### Determining when to stop iterating through the Q-learning algorithm
- We determine when to stop iterating through the Q-learning algorithm in the update_q_matrix function. 
- In order to determine this value we ran our code, which updated and printed our a matrix after each update, several times to find a value that allowed the q-matrix to converge on values. We found that the value we set (150) was sufficient to update all the states that would lead to possible rewards and eventually allow our matrix to converge.

## Robot perception description
### Identifying locations and identities of each colored object
- We determine the identities and location of the colored objects in the find_color function
- In order to find the colored object we subscribed to the RGB camera topic and determined the HSV values for pink, green, and blue. We isolated the color by creating a mask to get rid of every colored pixel that isn't the color we want and then center the pixels in the image using the cv2 moment function. Once we have these bounds we can subscribe to the laser scan topic and determine the location of these colored objects. We use proprtional control to rotate the robot so that the colored image is directly infront of the robot and then move the robot foward until it gets to a set distance. 
### Identifying locations and identities of AR tags.
- We determine the identities and location of the AR tags in the __ function.
- In order to find the location of the AR tag we 

## Robot manipulation and movement
### Moving to the right spot in order to pick up a colored object
- We determine the distance to the right colored object in the find_color function. 
- Previously to writing the code we predetermined a fix distance to take into account the arm movemenmt and the laser scan distance. We used this fix distance with proportional control to ensure that when the robot gets to this distance infront of the colored object it stops to pick it up. 

### Picking up the colored object
- We pick up the colored object using the move_arm function.
- In order to determine the right paramaters for the joints of the arm, we used Rviz to help isolate the right angles. Within the move_arm function we hardcoded these angles and split the arm movement into 4 seperate motions consisting of opening the gripper, moving the arm up and down, and then closing the gripper. 

### Moving to the desired destination (AR tag) with the colored object
- We determine where to move with the colored object in the __ function
- In order to 

### Putting the object back down at the desired destination
- We determine how to move the object back down to the desired destination in the move_arm function.
- In order to


## Challanges
- One of the main challanges that we encountered in this project was malfunction issues with the robot. There were many instances in which we spent debugging out code yet the real issue was that the robot had either lost connection or had stoped reciving instructions. We determined that to help combat this issue it worked to close out of our terminal and reconnect to the robot. Once we realized that the issue relied with the robot it was much easier to get our code to run.
- Another challange that we faced was also getting all the functions to execute in a cohesive manner. Many times our robot would move to the colored object and then begin to find the ar tag without moving the arm or move to the colored object and execute a different part of the arm code. We were able to fine tune our paramaters and get the functions to execute in a linear manner by using many print statement to follow along with the messages being publish as well as introducing substatntial sleep statements between function calls. 
- Another challange was that the code to read the converged matrix wasnt working on Megans computer. We initially though the issue was with the code we had written but once we realized that the code was able to run efficiently on Emilias computer we just continued to run code on hers. This was a challange considering that we were only able to run and test code on one computer. We were not able to fix Megan's computer but since team work make the dream work we were still able to work and finish the projet.

## Future Work
- If we had more time we would try to implement odometry to move the robot back to its original location so that it can be in better position to scan to colors and AR tags. We found that when the robot was a little far from the tag it would sense the tag but would loose connection and would keep turning to find it again rather than moving towards it. By having the robot move to the middle I believe we can minimize the room for error to ensure that the robot moves foward when the tag is located. 
- Another thing we could implement is that we begin with the arm in an upward position where we realized that it might be more efficient to have the robot go in with a an opened gripper and 

## Takeaways



# Implementation Plan
## Q-learning Algorithm
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
