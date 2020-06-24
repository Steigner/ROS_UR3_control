#! /usr/bin/env python3

"""
1. We define register for ai gym, we set id, entry point, timestep, neccesary setups for using open ai gym.
2. TargetReaching class where we define publisher for action data to ~/testos/srv/planning_script.py, and ServiceProxy for call reset_robot node from ~/testos/srv/reset_robot.py
3. The main functions are reset, step, get_state.
a) Reset function calls rosservice /reset_robot, to reset environment.
b) Step function which publishes actions and evaluates state/reward. This function publish data to planning_script.py and collect data from script run.py.
c) Get state Function to call to get current state from script: current_pose_publisher.py
"""

import numpy as np
import rospy
import gym
import std_msgs.msg
import geometry_msgs.msg
from gym import error, spaces, utils
from gym.utils import seeding
from gym.envs.registration import register
from std_srvs.srv import Trigger
import rospkg

register(
    id= 'UR3_RL-v0',
    entry_point='env.target_reaching:TargetReaching',  
    max_episode_steps=50,
    #reward_threshold=50.0,
    )

class TargetReaching(gym.Env):
    def __init__(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('testos')
        
        pathq_pose = pkg_path + '/src/traj_qpose'
        paths_pose = pkg_path + '/src/traj_spose'
        pathdqn_pose = pkg_path + '/src/traj_dqnpose'
        pathdsarsa_pose = pkg_path + '/src/traj_dsarsapose'

        self.path = pathq_pose

        #Setup publisher to publish actions
        self.action_publisher = rospy.Publisher('/action_move_data', std_msgs.msg.Int8, queue_size=1)
        
        #Setup rosservice to reset robot
        self.reset_robot = rospy.ServiceProxy('/reset_robot', Trigger)
        
        #Define action and reward range {x+, x-, y+, y-, z+, z-}
        self.action_space = spaces.Discrete(6)
        self.reward_range = (-np.inf, np.inf)
        self._seed()

    def reset(self):
        print("Simulation reset")        
        #Call rosservice to reset robot.
        reset_succes = None
        while reset_succes == None:
            try:
                reset_succes = self.reset_robot.call()
            except:
                print ("ERROR: Can't reset the simulation!")
        # Get new pose state after reset and return it.
        current_position = None
        while current_position == None:
            try:
                current_position = rospy.wait_for_message('/current_position', geometry_msgs.msg.PoseStamped, 1)
            except:
                print ("ERROR: Can't read /current_position!")
        return self.get_state(current_position) 

    def step(self, action):
        current_position = None
        while current_position == None:
            try:
                current_position = rospy.wait_for_message('/current_position', geometry_msgs.msg.PoseStamped, 1)
            except:
                print ("ERROR: Can't read /current_position!")

        #Execute action by publishing action to "/action_move"
        self.action_publisher.publish(action)

        #Move_group can't execute actions quickly enough. Delay of 0.5s is needed.
        rospy.sleep(15)

        #Get new state after action, and check if episode is done
        state = self.get_state(current_position)
        done = self.done_state(state, current_position)

        #Define distance, which is use as reward
        distance = self.calculate_distance(current_position)

        rev=[0,5000,110,100,90,80,70,60,50,40,30,20,10]
        rospy.sleep(5)

        if not done:  
            print ('Distance:',round(distance,3), 'm')
            for i in range (12,0,-1):
                if  distance < i*0.01:
                    reward = rev[i]
                elif distance > 0.12:
                    reward = rev[0]         
        else:
            #TUNA
            reward = -1000

        return state, reward, done,{}

    #Set current position from node /current_position
    def get_state(self, current_pose):
        path = self.path
        # Loop to get current pose:
        curr_pos = None
        while curr_pos == None:
            try:
                curr_pos = rospy.wait_for_message('/current_position', geometry_msgs.msg.PoseStamped, timeout=1)
            except:
                print ("ERROR: Couldn't read /current_position!")

        x = curr_pos.pose.position.x
        y = curr_pos.pose.position.y
        z = curr_pos.pose.position.z
        
    
        with open (path,'a') as file:           
            file.write(str (x) + "," + str (y) + "," + str (z) + "\n")
        file.close()
        

        # Collecting the data read above into a state dict, and returning it  
        state = {}
        state['x'] = x
        state['y'] = y
        state['z'] = z

        return self.discretise_data(state)

    # This function discretises the state values.
    def discretise_data(self, state):
        newstate = {}
        newstate['x'] = round(state['x'], 4)
        newstate['y'] = round(state['y'], 4)
        newstate['z'] = round(state['z'], 4)
        return newstate

    #If robot gets to bounderies, episode is done.     
    def done_state(self, state, current_position):
        done = False
        if (self.out_of_bounderies(current_position) == True):
            done = True
        return done

    #Calculate Vector distance from target
    def calculate_distance(self, current_position):
        X=0.25
        Y=0.0
        Z=0.6
        
        a = np.array((current_position.pose.position.x, current_position.pose.position.y,current_position.pose.position.z))
        b = np.array((X, Y, Z))
        distance = np.linalg.norm(a - b)
        return distance    
 
    #in this function we define obstacle
    def out_of_bounderies(self, current_position):
        distance_out = self.calculate_distance(current_position)
        if current_position.pose.position.z < 0.4 or current_position.pose.position.z > 0.7:
            return True
        else:
            return False

    # We initialize the random seed, required to generate random numbers. Those are used by the learning algorithm when generating random actions
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
        

    
