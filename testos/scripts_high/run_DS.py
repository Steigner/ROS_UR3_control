#!/usr/bin/env python3

import gym
import numpy as np
import rospkg
import rospy
import time

import env.target_reaching
import graph
import deep_sarsa
import json

from keras.models import load_model
from gym import wrappers

if __name__ == "__main__":
    rospy.init_node('TESTOS', anonymous=True, log_level=rospy.INFO)

    env = gym.make('UR3_RL-v0')

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('testos')
    outdir = pkg_path + '/training_results2'
    path = pkg_path + '/training_results2/main'
    
    pathdsarsa_joints = pkg_path + '/src/traj_dsarsajoints'
    pathdsarsa_pose = pkg_path + '/src/traj_dsarsapose'

    name = "DSARSA"

    #if u want to load trained network 
    load = False

    if load:    
        with open(path + name + '.json') as outfile:
            d = json.load(outfile)
            total_episodes = d.get('total_episodes')
            episode_steps = d.get('episode_steps')
            epsilon = d.get('epsilon')
            epsilon_min = d.get('epsilon_min')
            epsilon_discount = d.get('epsilon_discount')
            batch_size = d.get('batch_size')
            learn_start = d.get('learn_start')
            alpha = d.get('alpha')
            gamma = d.get('gamma')
            memory_size = d.get('memory_size')
            action_size= d.get('action_size')
            state_size = d.get('state_size')
        
        
        deepS = deep_sarsa.DEEPSARSA(state_size, action_size, memory_size,learn_start, alpha, gamma, epsilon,epsilon_min,epsilon_discount)
        deepS.load(path + name + ".h5")

    else:
        total_episodes=50
        episode_steps=50
        
        action_size = env.action_space.n
        state_size = 3

        alpha = 0.001
        gamma = 0.95
        
        epsilon = 1.0
        epsilon_discount = 0.995
        epsilon_min = 0.01 
        
        memory_size = 2000
        batch_size = 64
        learn_start=64

        deepS = deep_sarsa.DEEPSARSA(state_size, action_size, memory_size,learn_start, alpha, gamma, epsilon,epsilon_min,epsilon_discount)
    
    parameter_keys = ['total_episodes','episode_steps','epsilon','epsilon_min','epsilon_discount','batch_size','learn_start','alpha','gamma','memory_size','action_size','state_size']
    parameter_values = [total_episodes, episode_steps, epsilon, epsilon_min, epsilon_discount, batch_size, learn_start, alpha, gamma, memory_size, action_size, state_size]
    parameter_dictionary = dict(zip(parameter_keys, parameter_values))
    with open(path + name + '.json', 'w') as outfile:
        json.dump(parameter_dictionary, outfile)

    stepCounter = 0
    start_time = time.time()
    highest_reward = 0
    done = False

    env._max_episode_steps = episode_steps
    env = gym.wrappers.Monitor(env, outdir,force=True)
    plot_scatter = graph.Graph(outdir,total_episodes, 'DQN')

    for x in range(total_episodes):
        with open (pathdsarsa_joints,'a') as f:           
            f.write(str(x) + "\n")
        f.close()

        with open (pathdsarsa_pose,'a') as f2:           
            f2.write(str(x) + "\n")
        f2.close()

        print ("Start episode:"+str(x))

        cumulated_reward = 0

        observation = env.reset()
        
        state = np.array([observation["x"], observation["y"], observation["z"]])
        state = deepS.getQ(state,state_size)

        #epsilon discount method
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_discount

        for i in range(500):

            stepCounter += 1

            # env.render()
            action = deepS.chooseAction(state)
            nextState, reward, done, info = env.step(action)
            
            cumulated_reward += reward
            if highest_reward < reward:
                highest_reward = reward
            
            nextState = np.array([nextState["x"], nextState["y"], nextState["z"]])
            nextState = deepS.getQ(nextState,state_size)

            nextAction = deepS.chooseAction(nextState)

            deepS.memorize(state, action, reward, nextState, nextAction, done)
            
            state = nextState

            if done:
                state = nextState
                m, s = divmod(int(time.time() - start_time), 60)
                h, m = divmod(m, 60)
                print ("EPISODE: "+str(x+1)+" : [alpha: "+str(round(deepS.learning_rate,2))+" - gamma: "+str(round(deepS.gamma,2))+" - epsilon: "+str(round(deepS.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s))
                break

            if stepCounter > batch_size:
                deepS.learn(batch_size)
        
        #every 10 iteration save 
        if x % 10 == 0:
            deepS.save(path + name + ".h5")

            env._flush(force=True)

    plot_scatter.scatter(env)

    print (highest_reward)

    env.close()
        