#!/usr/bin/env python3

import numpy
import time
import gym
import rospkg
from gym import wrappers
from functools import reduce
import rospy

import env.target_reaching
import graph
import sarsa

if __name__ == '__main__':

    rospy.init_node('TESTOS', anonymous=True, log_level=rospy.INFO)
 
    epsilon_discount=0.999
    total_episodes=500
    episode_steps=50

    # Create the Gym environment
    env = gym.make('UR3_RL-v0')
    env._max_episode_steps = episode_steps
    print ("Gym environment done")

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('testos')
    outdir = pkg_path + '/training_results'
 
    env = wrappers.Monitor(env, outdir, force=True)


    last_time_steps = numpy.ndarray(0)
    plot_scatter = graph.Graph(outdir,total_episodes, 'SARSA')

    # Initialise SARSA and define hyperparameters

    sarsa = sarsa.SARSA(actions=range(env.action_space.n),alpha=0.3, gamma=0.75, epsilon=0.9)

    initial_epsilon = sarsa.epsilon
    start_time = time.time()
    highest_reward = -5000

    for x in range(total_episodes):
        print ("Start episode:"+str(x))
        done = False
        cumulated_reward = 0
        observation=env.reset()

        if sarsa.epsilon > 0.05:
            sarsa.epsilon *= epsilon_discount

        state = ''.join(map(str, observation))
        #issue with env.render call back errors, searched on web and people have same problem
        #env.render()

        for i in range(episode_steps):

            # Pick an action based on the current state
            action = sarsa.chooseAction(state)

            # Execute action and get feedback
            observation, reward, done, info = env.step(action)

            cumulated_reward += reward

            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            nextState = ''.join(map(str, observation))

            nextAction = sarsa.chooseAction(nextState)

            sarsa.learn(state, action, reward, nextState, nextAction)

            # Reset OpenAI environement before new action
            env._flush(force=True)

            if not(done):
                state = nextState
            else:
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print ("EP: "+str(x+1)+" - [alpha: "+str(round(sarsa.alpha,2))+" - gamma: "+str(round(sarsa.gamma,2))+" - epsilon: "+str(round(sarsa.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s))
    
    l = last_time_steps.tolist()
    l.sort()
    
    plot_scatter.scatter(env)

    print (-highest_reward)

    #print("Overall score: {:0.2f}".format(last_time_steps.mean()))
    #print("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:]) ))
    env.close()
