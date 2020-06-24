#!/usr/bin/env python3
"""
In this script is define architecture of neural netowrk for target reaching task.
1) part build sequential network, where is used Dense layer(fully connected cn layer)
    as optimizer is choosen ADAM 
2) part of this script are functions which work with states, action, rewards, 
3) part are functions save/load builded neural network
"""

import random
import numpy as np

import time 

from collections import deque
from keras.models import Sequential
from keras.layers import Dense, Flatten, Activation, Dropout
from keras.utils import plot_model
from tensorflow.keras import initializers
from tensorflow.keras.callbacks import TensorBoard
from keras import optimizers

import memory_dqn

class DEEPQ:
    def __init__(self, state_size, action_size, memory_size, learnStart, alpha, gamma, epsilon, epsilon_min,epsilon_discount):
        self.state_size = state_size
        self.action_size = action_size

        #self.memory = deque(maxlen=2000)
        self.memory = memory_dqn.Memory(memory_size)

        self.gamma = gamma    # discount rate
        self.epsilon = epsilon  # exploration rate
        
        self.epsilon_min = epsilon_min
        self.epsilon_discount = epsilon_discount
        
        self.learning_rate = alpha
        self.learnStart = learnStart

        self.NAME = "DEEPQNETWORK {}".format(int(time.time()))
        self.tensorboard = TensorBoard(log_dir='log/{}'.format(self.NAME))

        self.model = self._build_model()

    #architecture of neural network fullyconected layers [32,32,32,6]
    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        model = Sequential()
        model.add(Dense(32, input_dim=self.state_size))
        model.add(Activation('relu'))
        model.add(Dense(32))
        model.add(Activation('relu'))
        model.add(Dense(32))
        model.add(Activation('relu'))
        model.add(Dense(self.action_size, activation='linear'))

        adatela = optimizers.Adadelta(learning_rate=self.learning_rate, rho=0.95, epsilon=1e-07)
        adagrad = optimizers.Adagrad(learning_rate=self.learning_rate,epsilon=1e-07,)
        adam = optimizers.Adam(learning_rate=self.learning_rate,beta_1=0.9,beta_2=0.999,epsilon=1e-07,amsgrad=False)
        
        #best oprimizer for task : ADAM
        model.summary()
        model.compile(loss='mse',
                    optimizer=adam,
                    metrics=['accuracy']
                    )

        plot_model(model, to_file='model_neural_network.png', show_shapes=True)  # vizualizace modelu SitA
        return model

    #extended version of save parameters for learning
    def memorize(self, state, action, reward, nextState, done):
        #self.memory.append((state, action, reward, next_state, done))
        self.memory.addMemory(state, action, reward, nextState, done)
      
    def getQ(self, state, size):
        return state.reshape([1,size])
    
    def calculateTarget(self, nextState, reward, done):
        if done:
            return reward
        else:
            return reward + self.gamma * np.amax(self.model.predict(nextState)[0])
    
    def chooseAction(self, state):
        rand = random.random()
        if rand <= self.epsilon:
            action = random.randrange(self.action_size)
        else:
            act_values = self.model.predict(state)
            action = np.argmax(act_values[0])

        return action

    def learn(self, batch_size):
        if self.memory.getCurrentSize() > self.learnStart:
            miniBatch = self.memory.getMiniBatch(batch_size)        
            for sample in miniBatch:
                done = sample['isFinal']
                state = sample['state']
                action = sample['action']
                reward = sample['reward']
                nextState = sample['nextState']
                
                target = self.calculateTarget(nextState, reward, done)

                target_f = self.model.predict(state)
                target_f[0][action] = target

                self.model.fit(state, target_f, batch_size = len(miniBatch), epochs=1, verbose=0,) #callbacks = [self.tensorboard])
                #save model for TensorBoard analysis
                #self.model.fit(state, target_f, batch_size = len(miniBatch), epochs=1, verbose=0, callbacks = [self.tensorboard])
        
    #load and save model functions            
    def load(self, path):
        self.model.load_weights(path)

    def save(self, path):
        self.model.save_weights(path)
    
    