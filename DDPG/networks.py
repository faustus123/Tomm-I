# Author: Kishansingh Rajput
# Script: DDPG Networks

import os
import tensorflow as tf
import tensorflow.keras as keras
from tensorflow.keras.layers import Dense

class CriticNetwork(keras.Model):
    def __init__(self, fc1_dims=512, fc2_dims=512, fc3_dims=512, fc4_dims=256, fc5_dims=128,
            name='critic', chkpt_dir='tmp/ddpg'):
        super(CriticNetwork, self).__init__()
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.fc3_dims = fc3_dims
        self.fc4_dims = fc4_dims
        self.fc5_dims = fc5_dims

        self.model_name = name
        self.checkpoint_dir = chkpt_dir
        self.checkpoint_file = os.path.join(self.checkpoint_dir, 
                    self.model_name+'_ddpg.h5')

        self.fc1 = Dense(self.fc1_dims, activation='tanh')
        self.fc2 = Dense(self.fc2_dims, activation='tanh')
        self.fc3 = Dense(self.fc3_dims, activation='tanh')
        self.fc4 = Dense(self.fc4_dims, activation='tanh')
        self.fc5 = Dense(self.fc5_dims, activation='tanh')
        self.q = Dense(1, activation="tanh")

    def call(self, state, action):
        action_value = self.fc1(tf.concat([state, action], axis=1))
        action_value = tf.keras.layers.BatchNormalization()(action_value)
        action_value = self.fc2(action_value)
        action_value = tf.keras.layers.BatchNormalization()(action_value)
        action_value = self.fc3(action_value)
        action_value = tf.keras.layers.BatchNormalization()(action_value)
        action_value = self.fc4(action_value)
        action_value = tf.keras.layers.BatchNormalization()(action_value)
        action_value = self.fc5(action_value)

        q = self.q(action_value)

        return q

class ActorNetwork(keras.Model):
    def __init__(self, fc1_dims=512, fc2_dims=512, fc3_dims=512, fc4_dims=512, fc5_dims=128, n_actions=8, name='actor',
            chkpt_dir='tmp/ddpg'):
        super(ActorNetwork, self).__init__()
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.fc3_dims = fc3_dims
        self.fc4_dims = fc4_dims
        self.fc5_dims = fc5_dims
        self.n_actions = n_actions

        self.model_name = name
        self.checkpoint_dir = chkpt_dir
        self.checkpoint_file = os.path.join(self.checkpoint_dir, 
                    self.model_name+'_ddpg.h5')

        self.fc1 = Dense(self.fc1_dims, activation='tanh')
        self.fc2 = Dense(self.fc2_dims, activation='tanh')
        self.fc3 = Dense(self.fc2_dims, activation='tanh')
        self.fc4 = Dense(self.fc2_dims, activation='tanh')
        self.fc5 = Dense(self.fc2_dims, activation='tanh')
        self.mu = Dense(self.n_actions, activation='tanh')

    def call(self, state):
        prob = self.fc1(state)
        prob = tf.keras.layers.BatchNormalization()(prob)
        prob = self.fc2(prob)
        prob = tf.keras.layers.BatchNormalization()(prob)
        prob = self.fc3(prob)
        prob = tf.keras.layers.BatchNormalization()(prob)
        prob = self.fc4(prob)
        prob = tf.keras.layers.BatchNormalization()(prob)
        prob = self.fc5(prob)

        mu = self.mu(prob)

        return mu