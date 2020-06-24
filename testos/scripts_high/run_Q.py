#!/usr/bin/env python3

import time
import gym
import rospkg
from gym import wrappers
import rospy

import env.target_reaching
import graph
import qlearn

if __name__ == '__main__':
    rospy.init_node('TESTOS', anonymous=True, log_level=rospy.INFO)
    
    epsilon_discount=0.999
    total_episodes=500
    episode_steps=50

    # Create the Gym environment
    env = gym.make('UR3_LR-v0')
    env._max_episode_steps = episode_steps

    print ("Gym environment done")

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('testos')
    outdir = pkg_path + '/training_results'
    pathq_joints = pkg_path + '/src/traj_qjoints'
    pathq_pose = pkg_path + '/src/traj_qpose'

    env = wrappers.Monitor(env, outdir, force=True)

    plot_scatter = graph.Graph(outdir,total_episodes, 'Q-learning')
   
    # Initialise qlearn and define hyperparameters

    qlearn = qlearn.QLearn(actions=range(env.action_space.n), alpha=0.3, gamma=0.75, epsilon=0.9)

    initial_epsilon = qlearn.epsilon
    start_time = time.time()
    highest_reward = 0

    for x in range(total_episodes):
        
        with open (pathq_joints,'a') as f:           
            f.write(str(x) + "\n")
        f.close()

        with open (pathq_pose,'a') as f2:           
            f2.write(str(x) + "\n")
        f2.close()

        
        print ("Start episode:"+str(x))
        done = False
        cumulated_reward = 0


        observation=env.reset()

        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount

        state = ''.join(map(str, observation))

        for i in range(episode_steps):
            
            # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            
            # Execute action and get feedback

            observation, reward, done, info = env.step(action)

            cumulated_reward += reward
            
            if highest_reward < reward:
                highest_reward = reward

            nextState = ''.join(map(str, observation))
            qlearn.learn(state, action, reward, nextState)

            # Reset OpenAI environement before new action
            env._flush(force=True)
        

            if not(done):
                state = nextState
            else:
                break
        
        #cumulated_reward=round(cumulated_reward,4)
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print ("EPISODE: "+str(x+1)+" : [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s))
    
    
    plot_scatter.scatter(env)

    print (highest_reward)

    env.close()
