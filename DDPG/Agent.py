# Author: Kishansingh Rajput
# Script: RL Agent

import os
import numpy as np
import tensorflow as tf
import tensorflow.keras as keras
from tensorflow.keras.optimizers import Adam
from buffer import ReplayBuffer
from networks import ActorNetwork, CriticNetwork

class Agent:
    def __init__(self, input_dims, alpha=0.001, beta=0.002, env=None,
            gamma=0.99, n_actions=8, max_size=1000000, tau=0.005, 
            batch_size=64, noise=0.1):
        self.gamma = gamma
        self.tau = tau
        self.memory = ReplayBuffer(max_size, input_dims, n_actions)
        self.batch_size = batch_size
        self.n_actions = n_actions
        self.noise = noise
        self.max_action = env.action_space.high
        self.min_action = env.action_space.low
        
        self.actor = ActorNetwork(n_actions=n_actions, name='actor')
        self.critic = CriticNetwork(name='critic')
        self.target_actor = ActorNetwork(n_actions=n_actions, name='target_actor')
        self.target_critic = CriticNetwork(name='target_critic')

        self.actor.compile(optimizer=Adam(learning_rate=alpha))
        self.critic.compile(optimizer=Adam(learning_rate=beta))
        self.target_actor.compile(optimizer=Adam(learning_rate=alpha))
        self.target_critic.compile(optimizer=Adam(learning_rate=beta))

        self.update_network_parameters(tau=1)

    def update_network_parameters(self, tau=None):
        if tau is None:
            tau = self.tau

        weights = []
        targets = self.target_actor.weights
        for i, weight in enumerate(self.actor.weights):
            weights.append(weight * tau + targets[i]*(1-tau))
        self.target_actor.set_weights(weights)

        weights = []
        targets = self.target_critic.weights
        for i, weight in enumerate(self.critic.weights):
            weights.append(weight * tau + targets[i]*(1-tau))
        self.target_critic.set_weights(weights)

    def remember(self, state, action, reward, new_state, done):
        self.memory.store_transition(state, action, reward, new_state, done)

    #--------------------------------------
    # save_one_model
    #
    # This will save the model using the TF saved model format
    # and the model.model_dir path to the model. It should be called
    # with either an ActorNetwork or CriticNetwork model as the
    # argument. 
    def save_one_model(self, model):
        print("  saving model to: {}".format(model.model_dir))
        model.save(model.model_dir)

    #--------------------------------------
    # save_models
    #
    # Save all 4 models in TF saved model format
    def save_models(self):
        print('... saving models ...')
        self.save_one_model(self.actor)
        self.save_one_model(self.target_actor)
        self.save_one_model(self.critic)
        self.save_one_model(self.target_critic)
        # self.actor.save(self.actor.model_dir)
        # self.target_actor.save(self.target_actor.model_dir)
        # self.critic.save(self.critic.model_dir)
        # self.target_critic.save(self.target_critic.model_dir)
    
    #--------------------------------------
    # load_one_model
    #
    # This will load the model assuming the TF saved model format
    # and the model.model_dir path to the model. It should be called
    # with either an ActorNetwork or CriticNetwork model as the
    # argument. The model layer architecture is assumed to already
    # be defined (which it is in the constructor in networks.py).
    # It would match the architecture of the saved model.
    def load_one_model(self, model):
        if not os.path.exists(model.model_dir):
            print("  no model to load so will train from scratch (missing {})".format(model.model_dir))
            return

        print("  loading model from: {}".format(model.model_dir))
        loaded_model = tf.keras.models.load_model(model.model_dir)
        print("--- LOADED MODEL ---")
        loaded_model.summary()
        print("--- BUILT MODEL ---")
        model.build()
        model.summary()
        model.set_weights(loaded_model.get_weights())
        model.compile(
            optimizer=loaded_model.optimizer,
            loss=loaded_model.loss,
            metrics=loaded_model.metrics,
        )
        return model

    #--------------------------------------
    # load_models
    #
    # Load all 4 models from saved versions
    def load_models(self):
        print('... loading models ...')
        self.load_one_model(self.actor)
        self.load_one_model(self.target_actor)
        self.load_one_model(self.critic)
        self.load_one_model(self.target_critic)
        # self.actor.load_weights(self.actor.checkpoint_file)
        # self.target_actor.load_weights(self.target_actor.checkpoint_file)
        # self.critic.load_weights(self.critic.model_dir)
        # self.target_critic.load_weights(self.target_critic.model_dir)

    #--------------------------------------
    # choose_action
    #
    def choose_action(self, observation, add_noise=True):
        state = tf.convert_to_tensor([observation], dtype=tf.float32)
        actions = self.actor(state)
        #sactions = ["{:+.2f} ".format(v) for v in actions[0]]
        #print("Model output: {} x={:.3f} y={:.3f} pitch={:.3f}".format(sactions, observation[0], observation[1], observation[3]))
        # for i in range(8):
        #     tf.print(actions[0][i])
        if add_noise:
            actions += tf.random.normal(shape=[self.n_actions],
                    mean=0.0, stddev=self.noise)
       
        # Disable back legs for now
        mask = tf.constant([1., 1., 0., 0., 1., 1., 0., 0.])
        actions = tf.multiply(actions, mask)
        
        actions = tf.clip_by_value(actions, self.min_action, self.max_action)
        # tf.print(actions)
        return actions[0]

    def learn(self):
        if self.memory.mem_cntr < self.batch_size*2:
            return

        state, action, reward, new_state, done = \
                self.memory.sample_buffer(self.batch_size)

        states = tf.convert_to_tensor(state, dtype=tf.float32)
        states_ = tf.convert_to_tensor(new_state, dtype=tf.float32)
        rewards = tf.convert_to_tensor(reward, dtype=tf.float32)
        actions = tf.convert_to_tensor(action, dtype=tf.float32)

        with tf.GradientTape() as tape:
            target_actions = self.target_actor(states_)
            #######################################################
            # actions1 = 110 + target_actions[:4]*10
            # actions2 = 120 + target_actions[4:]*10
            # target_actions = tf.concat([actions1, actions2], axis=0)
            #######################################################
            critic_value_ = tf.squeeze(self.target_critic(
                                states_, target_actions), 1)
            critic_value = tf.squeeze(self.critic(states, actions), 1)
            target = rewards + self.gamma*critic_value_*(1-done)
            critic_loss = keras.losses.MSE(target, critic_value)

        critic_network_gradient = tape.gradient(critic_loss,
                                            self.critic.trainable_variables)
        self.critic.optimizer.apply_gradients(zip(
            critic_network_gradient, self.critic.trainable_variables))

        with tf.GradientTape() as tape:
            new_policy_actions = self.actor(states)
            #######################################################
            # actions1 = 110 + new_policy_actions[:4]*10
            # actions2 = 120 + new_policy_actions[4:]*10
            # new_policy_actions = tf.concat([actions1, actions2], axis=0)
            #######################################################
            actor_loss = -self.critic(states, new_policy_actions)
            actor_loss = tf.math.reduce_mean(actor_loss)

        actor_network_gradient = tape.gradient(actor_loss, 
                                    self.actor.trainable_variables)
        self.actor.optimizer.apply_gradients(zip(
            actor_network_gradient, self.actor.trainable_variables))

        self.update_network_parameters()
