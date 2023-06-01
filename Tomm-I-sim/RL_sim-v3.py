#!/usr/bin/env python3
#
#
# David Lawrence  May 28, 2023
# (with lots of input from Kishan's DDPG implmentation)
#
#
# The following is based on an example generated from ChatGPT. It is
# clearly based on the cartpole example found on the web.

import sys
import math
import collections
import statistics
import threading
import numpy as np
import tensorflow as tf
import tqdm
from typing import Any, List, Sequence, Tuple
import TommIsim


joints = ["FR_hip", "FL_hip", "BR_hip", "BL_hip", "FR_foot", "FL_foot", "BR_foot", "BL_foot"]

# Set seed for experiment reproducibility
seed = 42
tf.random.set_seed(seed)
np.random.seed(seed)

# Small epsilon value for stabilizing division operations
eps = np.finfo(np.float32).eps.item()

# Hyperparameters
gamma = 0.99
lamda = 0.95

istep_episode = int(0)

# Set up optimizer and loss functions
optimizer = tf.keras.optimizers.Adam(learning_rate=0.01)
huber_loss = tf.keras.losses.Huber(reduction=tf.keras.losses.Reduction.SUM)

# This is used to synchronize the simulation thread (main thread)
# and the AI/ML training thread. The training thread will wait on
# the condition and the simulation thread will notify it. 
training_condition = threading.Condition()
simulation_condition = threading.Condition()

#----------------------------------------------------------------------------
# NormalizeSimulationState
#
# Convert values provided by the simulation as the robot state into normalized
# values for use in the AI/ML model training. 
# n.b. The foot and hip conversions must be kept in sync with the
# UnnormalizeSimulationActions procedure below.
def NormalizeSimulationState( status ):
    for key in status.keys():
        if "foot"    in key: status[key]  = (status[key] - 130)/10
        elif "hip"   in key: status[key]  = (status[key] - 120)/10
        elif "x"     == key: status[key] /= 10 
        elif "y"     == key: status[key] /= 10 
        elif "x"     == key: status[key] /= 10 
        elif "yaw"   == key: status[key] /= 180 
        elif "pitch" == key: status[key] /= 180 
        elif "roll"  == key: status[key] /= 180 
    
    return status

#----------------------------------------------------------------------------
# UnnormalizeSimulationActions
#
# Convert values provided by the AI/ML model as actions into units that can
# be used by the robot simulation simulation. 
# n.b. These must be kept in sync with the NormalizeSimulationState procedure
# above.
def UnnormalizeSimulationActions( actions: np.ndarray ) -> np.ndarray:
    global joints
    actions = actions[0]
    for i, key in enumerate(joints):
        if "foot"    in key: actions[i]  = (10*actions[i]) + 130
        elif "hip"   in key: actions[i]  = (10*actions[i]) + 120
    return actions

#----------------------------------------------------------------------------
# Create the actor-critic models
class ActorCriticModel(tf.keras.Model):
    def __init__(self):
        super(ActorCriticModel, self).__init__()
        
        self.critic1 = tf.keras.layers.Dense(512, activation='tanh')
        self.critic1_batch_norm = tf.keras.layers.BatchNormalization()
        self.critic2 = tf.keras.layers.Dense(512, activation='tanh')
        self.critic2_batch_norm = tf.keras.layers.BatchNormalization()
        self.critic3 = tf.keras.layers.Dense(512, activation='tanh')
        self.critic3_batch_norm = tf.keras.layers.BatchNormalization()
        self.critic4 = tf.keras.layers.Dense(256, activation='tanh')
        self.critic4_batch_norm = tf.keras.layers.BatchNormalization()
        self.critic5 = tf.keras.layers.Dense(128, activation='tanh')
        self.critic_out = tf.keras.layers.Dense(1, activation="tanh")

        self.actor1 = tf.keras.layers.Dense(512, activation='tanh')
        self.actor1_batch_norm = tf.keras.layers.BatchNormalization()
        self.actor2 = tf.keras.layers.Dense(512, activation='tanh')
        self.actor2_batch_norm = tf.keras.layers.BatchNormalization()
        self.actor3 = tf.keras.layers.Dense(512, activation='tanh')
        self.actor3_batch_norm = tf.keras.layers.BatchNormalization()
        self.actor4 = tf.keras.layers.Dense(512, activation='tanh')
        self.actor4_batch_norm = tf.keras.layers.BatchNormalization()
        self.actor5 = tf.keras.layers.Dense(128, activation='tanh')
        self.actor_out = tf.keras.layers.Dense(8, activation="tanh")

    def call(self, inputs):
        x = self.actor1(inputs)
        x = self.actor1_batch_norm(x)
        x = self.actor2(x)
        x = self.actor2_batch_norm(x)
        x = self.actor3(x)
        x = self.actor3_batch_norm(x)
        x = self.actor4(x)
        x = self.actor4_batch_norm(x)
        x = self.actor5(x)
        actions = self.actor_out(x)

        x = self.critic1(tf.concat([inputs, actions], axis=1))
        x = self.critic1_batch_norm(x)
        x = self.critic2(x)
        x = self.critic2_batch_norm(x)
        x = self.critic3(x)
        x = self.critic3_batch_norm(x)
        x = self.critic4(x)
        x = self.critic4_batch_norm(x)
        x = self.critic5(x)
        critics = self.critic_out(x)

        return actions, critics



#----------------------------------------------------------------------------
# env_step
#
# Given an action (set of motor settings), apply them and have the simulation
# take one more time step. Return the new state, reward, and done flag.
#
# Upon entry, the simulation should be waiting on the simulation_condition
# lock waiting for us to notify them to take another step.
def env_step(action: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns state, reward and done flag given an action."""

    global istep_episode

    # Capture state of before we take next simulation step
    prev_state = TommIsim.GetStatus()

    # Apply the given action to the simulation
    sim_action = UnnormalizeSimulationActions( action )
    motors = {}
    for i,key in enumerate(joints): motors[key] = sim_action[i]
    TommIsim.SetMotors( motors )

    # Have simulation take one time step
    with simulation_condition:
        simulation_condition.notify()  # Tell simulation to take a step
    with training_condition:
        training_condition.wait()      # wait until simulation completes the step

    # Get the current state
    state = TommIsim.GetStatus()

    # Calculate reward
    rewards = {}
    rewards["move in +x"] = 400.0*(state['x'] - prev_state['x'])
    rewards["don't move in y"] = -0.05*abs(state['y']) 
    rewards["stay upright"] = 0.001

    # reward  = (state['x'] - prev_state['x'])    # reward: Advances in x direction
    # reward -= 0.05*abs(state['y'])              # penalty: straying in y-directio
    # reward += 0.001                             # reward: staying up longer (i.e. not falling over)

    # Reward motion that results in large changes of the joint positions
    rewards["big joint motion"] = 0
    # for joint in joints:
    #     rewards["big joint motion"] += math.log(float(istep_episode))*0.001*(state[joint] - prev_state[joint])**2  # n.b. these are unnormalized values in degrees!

    # Add all reward elements together
    reward = sum(list(rewards.values()))

    # Occasionally print reward elements
    if istep_episode<=5:
        for k,v in rewards.items(): print("{} {}: {:.4f}".format(istep_episode,k,v))

    # Convert current state into normalized values as np array
    normalized_state = NormalizeSimulationState( state )
    npstate = np.array(list(normalized_state.values()), dtype=np.float32)

    # Check if we are done with the episode and need to reset the simulation
    done = False
    if state['z'] < 0.15: done = True  # Check if robot has fallen over

    # state, reward, done, truncated, info = env.step(action)
    return (npstate, 
          np.array(reward, np.float32), 
          np.array(done, np.int32))


#----------------------------------------------------------------------------
# tf_env_step
#
# Wrap the above as an operation in a TensorFlow function.
# This would allow it to be included in a callable TensorFlow graph.
def tf_env_step(action: tf.Tensor) -> List[tf.Tensor]:
  return tf.numpy_function(env_step, [action], 
                           [tf.float32, tf.float32, tf.int32])


#----------------------------------------------------------------------------
# run_episode
#
def run_episode(
    initial_state: tf.Tensor,  
    model: tf.keras.Model, 
    max_steps: int) -> Tuple[tf.Tensor, tf.Tensor, tf.Tensor]:
    """Runs a single episode to collect training data."""

    global istep_episode

    action_probs = tf.TensorArray(dtype=tf.float32, size=0, dynamic_size=True)
    values = tf.TensorArray(dtype=tf.float32, size=0, dynamic_size=True)
    rewards = tf.TensorArray(dtype=tf.float32, size=0, dynamic_size=True)

    initial_state_shape = initial_state.shape
    state = initial_state

    istep_episode = 0
    for t in tf.range(max_steps):

        istep_episode += 1

        # Convert state into a batched tensor (batch size = 1)
        state = tf.expand_dims(state, 0)

        # Run the model and to get action probabilities and critic value
        action_logits_t, value = model(state)

        # In the original example (cartpole) there are only two possible
        # actions: move left and move right. These are mutally exclusive
        # and the model does not provide a speed of movement, just the
        # direction. In fact the model does not directly decide which
        # direction to go in, but rather a pair of "logits" that represent
        # the logs of the probablities it associates with each action.
        # It then uses the tf.random.categorical() function to randomly
        # sample which of the actions to perfrom given those probabilities.
        # 
        # The logits are also converted from their unnormalized
        # probabilities into actual probabilities using the tf.nn.softmax()
        # function. Only the softmax probability for the chosen action
        # is saved in the action_probs tensor.
        #
        # For this project, we want the model to provide settings for each
        # of the 8 motors. These are not mutually exclusive categories,
        # but rather 8 continuous values that are highly coupled. In
        # the original DDPG version Kishan made, the randomness of choosing
        # an action was replaced by random tweaks to each of the motor
        # settings suggested by the model. Here, I will forego the randomness
        # in hopes the model will learn the torques based purely on the
        # position of the legs.

        # Sample next action from the action probability distribution
        # action = tf.random.categorical(action_logits_t, 1)[0, 0]
        # action_probs_t = tf.nn.softmax(action_logits_t)

        action_logits_t += tf.random.normal(shape=action_logits_t.shape, mean=0.0, stddev=0.1)

        # Store critic values
        values = values.write(t, tf.squeeze(value))

        # Store log probability of the action chosen
        # action_probs = action_probs.write(t, action_probs_t[0, action])
        action_probs = action_probs.write(t, action_logits_t[0])

        # Apply action to the environment to get next state and reward
        state, reward, done = tf_env_step(action_logits_t)
        state.set_shape(initial_state_shape)

        # Store reward
        rewards = rewards.write(t, reward)
        print('istep: {} reward:{:.3f}'.format(istep_episode, reward))

        if tf.cast(done, tf.bool):
            break

    action_probs = action_probs.stack()
    values = values.stack()
    rewards = rewards.stack()

    return action_probs, values, rewards


#----------------------------------------------------------------------------
# get_expected_return
#
def get_expected_return(
    rewards: tf.Tensor, 
    gamma: float, 
    standardize: bool = True) -> tf.Tensor:
    """Compute expected returns per timestep."""

    n = tf.shape(rewards)[0]
    returns = tf.TensorArray(dtype=tf.float32, size=n)

    # Start from the end of `rewards` and accumulate reward sums
    # into the `returns` array
    rewards = tf.cast(rewards[::-1], dtype=tf.float32)
    discounted_sum = tf.constant(0.0)
    discounted_sum_shape = discounted_sum.shape
    for i in tf.range(n):
        reward = rewards[i]
        discounted_sum = reward + gamma * discounted_sum
        discounted_sum.set_shape(discounted_sum_shape)
        returns = returns.write(i, discounted_sum)
    returns = returns.stack()[::-1]

    if standardize:
        returns = ((returns - tf.math.reduce_mean(returns)) / 
                (tf.math.reduce_std(returns) + eps))

    return returns



#----------------------------------------------------------------------------
# compute_loss
#
def compute_loss(
    action_probs: tf.Tensor,  
    values: tf.Tensor,  
    returns: tf.Tensor) -> tf.Tensor:
    """Computes the combined Actor-Critic loss."""

    # Here, it computes the total loss based on both the actor and
    # critic performances. The original example scales the actor
    # loss by the probability of the action taken. I guess this is
    # to downgrade how much the models are changed when an action
    # is selected that the model is not confident about.
    #
    # For this application, the actor model outputs are used directly 
    # and not interpreted as probabilities. Thus using the values 
    # to scale the advantage does not make much sense. Therefore,
    # the advantage is the only thing included in the actor loss.
    # This probably means the actor and critic losses are essentially
    # the same (except for the huber function). Maybe we don't need
    # an explicit actor loss (??)

    advantage = returns - values

    # action_log_probs = tf.math.log(action_probs)
    # actor_loss = -tf.math.reduce_sum(action_log_probs * advantage)
    actor_loss = -tf.math.reduce_sum(advantage)

    critic_loss = huber_loss(values, returns)

    return actor_loss + critic_loss



#----------------------------------------------------------------------------
# train_step
#
#@tf.function
def train_step(
    initial_state: tf.Tensor, 
    model: tf.keras.Model, 
    optimizer: tf.keras.optimizers.Optimizer, 
    gamma: float, 
    max_steps_per_episode: int) -> tf.Tensor:
    """Runs a model training step."""

    with tf.GradientTape() as tape:

        # Run the model for one episode to collect training data
        action_probs, values, rewards = run_episode( initial_state, model, max_steps_per_episode) 

        # Calculate the expected returns
        returns = get_expected_return(rewards, gamma)

        # Convert training data to appropriate TF tensor shapes
        action_probs, values, returns = [tf.expand_dims(x, 1) for x in [action_probs, values, returns]] 

        # Calculate the loss values to update our network
        loss = compute_loss(action_probs, values, returns)

    # Compute the gradients from the loss
    grads = tape.gradient(loss, model.trainable_variables)

    # Apply the gradients to the model's parameters
    optimizer.apply_gradients(zip(grads, model.trainable_variables))

    episode_reward = tf.math.reduce_sum(rewards)
    print( rewards )
    print('episode_reward{:.3f}'.format(episode_reward))

    return episode_reward


#----------------------------------------------------------------------------
# TrainModels
#
def TrainModels():
    global model

    min_episodes_criterion = 100
    max_episodes = 10000
    max_steps_per_episode = 200

    # `CartPole-v1` is considered solved if average reward is >= 475 over 500 
    # consecutive trials
    reward_threshold = 475
    running_reward = 0

    # The discount factor for future rewards
    gamma = 0.99

    # Keep the last episodes reward
    episodes_reward: collections.deque = collections.deque(maxlen=min_episodes_criterion)

    t = tqdm.trange(max_episodes)
    for i in t:
        # Wait for simulation thread to complete current step
        with simulation_condition:
            simulation_condition.notify()
        with training_condition:
            training_condition.wait()

        # Reset simulation to initial state
        TommIsim.Reset()
        state = TommIsim.GetStatus()
        initial_state = NormalizeSimulationState( state )
        # initial_state, info = env.reset()
        initial_state = tf.constant(list(initial_state.values()), dtype=tf.float32)
        episode_reward = float(train_step(initial_state, model, optimizer, gamma, max_steps_per_episode))

        episodes_reward.append(episode_reward)
        running_reward = statistics.mean(episodes_reward)


        t.set_postfix(  episode_reward=episode_reward, running_reward=running_reward )

        # Show the average episode reward every 10 episodes
        if i % 10 == 0:
            pass # print(f'Episode {i}: average reward: {avg_reward}')

        if running_reward > reward_threshold and i >= min_episodes_criterion:  
            break

    print(f'\nSolved at episode {i}: average reward: {running_reward:.2f}!')



#----------------------------------------------------------------------------
# MyPythonCallback
#
# This routine is called from the simulation after every step. It is used
# to stall the simulation while the training thread processes the step.
def MyPythonCallback():
    with training_condition:
        training_condition.notify()  # Tell training thread to process current step
    with simulation_condition:
        simulation_condition.wait()  # Wait for training thread to complete step


#======================================================================
#                   main


# Initialize the actor-critic model
model = ActorCriticModel()

# Launch training thread (it will block on training_condition.wait() when 
# ready for input from simulation)
training_thread = threading.Thread(target=TrainModels)
training_thread.start()

# Run simulation in main thread
TommIsim.SetRunRealTime( False )
TommIsim.Setup()
TommIsim.RegisterCallback(MyPythonCallback)
TommIsim.SetUseGraphics( True )

# n.b. this will block until the simulation is complete
TommIsim.Run()

# Join the training thread back in
training_thread.join()

print("Done.")
