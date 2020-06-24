#!/usr/bin/env python3

import random

class SARSA:
    def __init__(self, actions, epsilon, alpha, gamma):
        self.q = {}
        self.epsilon = epsilon  # exploration constant
        self.alpha = alpha      # discount constant
        self.gamma = gamma      # discount factor
        self.actions = actions

    def getQ(self, state, action):
        return self.q.get((state, action), 0.0)

    def learnQ(self, state, action, reward, rgQsa):
        '''
        Q-learning:
            Q(s, a) += alpha * (reward(s,a) + max(Q(s') - Q(s,a))
        SARSA                
            Q(s, a) += alpha * (reward() + gama*(Q(s',alpha') - Q(s,a))
        '''
        
        Qsa = self.q.get((state, action), None)

        if Qsa is None:
            self.q[(state, action)] = reward
        else:
            #rgQsa = (reward() + gama*(Q(s',alpha') 
            self.q[(state, action)] = Qsa + self.alpha * (rgQsa - Qsa)

    def chooseAction(self, state, return_q=False):
        if random.random() < self.epsilon:
            action = random.choice(self.actions)
        else:
            q = [self.getQ(state, a) for a in self.actions]
            maxQ = max(q)
            count = q.count(maxQ)
            if count > 1:
                best = [i for i in range(len(self.actions)) if q[i] == maxQ]
                i = random.choice(best)
            else:
                i = q.index(maxQ)

            action = self.actions[i]
        return action

    def learn(self, state1, action1, reward, state2, action2):
        qnew = self.getQ(state2,action2)
        rgQsa=reward + self.gamma*qnew
        self.learnQ(state1, action1, reward, rgQsa)