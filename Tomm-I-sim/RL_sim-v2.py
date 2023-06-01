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
import threading
import tensorflow as tf
import TommIsim


joints = ["FR_hip", "FL_hip", "BR_hip", "BL_hip", "FR_foot", "FL_foot", "BR_foot", "BL_foot"]

# Hyperparameters
num_episodes = 100
gamma = 0.99
lamda = 0.95

# Set up optimizer and loss functions
optimizer = tf.keras.optimizers.Adam(learning_rate=0.001)
huber_loss = tf.keras.losses.Huber()

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
def UnnormalizeSimulationActions( actions ):
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
# train_step
#
# This is called at the end of an episode to update the model(s) based on the
# arrays of states, actions, and returns accumulated during the episode in
# the training loop below.
def train_step(states, actions, returns):
    with tf.GradientTape() as tape:
        # Forward pass through the model
        model_actions, values = model(states)
        
        # Compute advantages
        advantages = returns - values
        
        # Compute the actor and critic losses
        # actor_loss = tf.reduce_sum(tf.nn.sparse_softmax_cross_entropy_with_logits(labels=actions, logits=logits) * advantages)
        actor_loss = huber_loss(model_actions, actions)
        critic_loss = huber_loss(values, returns)
        total_loss = actor_loss + critic_loss
        
    # Compute and apply gradients
    grads = tape.gradient(total_loss, model.trainable_variables)
    optimizer.apply_gradients(zip(grads, model.trainable_variables))

#----------------------------------------------------------------------------
# TrainModels
#
# Train the actor and critic models using the running simulation.
# This is called from the MyPythonCallback routine which is run in
# a dedicated thread. The reason for using multiple threads is because
# the simulation needs to communicate its state and apply any actions
# in response at a point deep inside the main simulation loop. Similarly,
# the AI/ML training needs to access the state and apply actions from
# deep within its own traning loop. i.e. both want to control the other
# using callbacks.
#
# At the point where the training needs the simulation state, it will
# block
def TrainModels():
    global joints

    # Training loop
    for episode in range(num_episodes):

        # Wait for simulation thread to complete current step
        with simulation_condition:
            simulation_condition.notify()
        with training_condition:
            training_condition.wait()

        # Reset simulation to initial state
        TommIsim.Reset()
        state = TommIsim.GetStatus()
        state = NormalizeSimulationState( state )
        episode_reward = 0
        trajectory = []  # Store (state, action, reward) tuples
        itime_steps = 0

        while True:
            # Wait for simulation thread to complete current step
            with simulation_condition:
                simulation_condition.notify()  # Tell simulation to take a step
            with training_condition:
                training_condition.wait()      # wait until simulation completes the step

            # Run the actor's policy to choose an action (based on state from previous step)
            state_tensor = tf.convert_to_tensor([list(state.values())], dtype=tf.float32)
            model_actions, _ = model(state_tensor)  # returns actor,critic (we ignore the critic model's response here)
            action = model_actions.numpy()
            # action = tf.random.categorical(model_actions, 1)[0, 0].numpy()
            
            # Apply the chosen action to the simulation
            sim_action = UnnormalizeSimulationActions( action )
            motors = {}
            for i,key in enumerate(joints): motors[key] = sim_action[i]
            TommIsim.SetMotors( motors )
            print(motors)

            # Calculate state, reward, and done status of current step 
            next_state = TommIsim.GetStatus()
            next_state = NormalizeSimulationState( next_state )
            reward = next_state['x'] - 0.25*abs(next_state['y'])
            episode_reward += reward
            done = False
            if next_state['z'] < 0.015: done = True
            itime_steps += 1
            if itime_steps >= 200: done = True

            sactions = ["{:+.2f} ".format(float(v)) for v in list(model_actions[0])[:8]]
            status = TommIsim.GetStatus()
            # print("Model output: {} x={:.3f} y={:.3f} yaw={:.3f} reward={:.3f}".format(sactions, status['x'], status['y'], status['yaw'], reward))
            
            # Store the (state, action, reward) tuple in the trajectory
            state_vals = list(state.values())
            action_vals = action.tolist()[0]
            trajectory.append((state_vals, action_vals, reward))
            
            if done:
                # Accumulate rewards and update actor-critic model
                states, actions, rewards = zip(*trajectory)
                states_tensor = tf.convert_to_tensor(states, dtype=tf.float32)
                actions_tensor = tf.convert_to_tensor(actions, dtype=tf.float32)
                rewards_tensor = tf.convert_to_tensor(rewards, dtype=tf.float32)
                
                next_state_tensor = tf.convert_to_tensor([list(next_state.values())], dtype=tf.float32)
                _, next_state_value = model(next_state_tensor)  # Get critic response only for current simulation state
                next_state_value = next_state_value[0, 0]
                
                # Compute advantages
                returns = []
                adv = tf.zeros_like(rewards_tensor)
                for t in reversed(range(len(rewards))):
                    next_value = rewards_tensor[t] + gamma * next_state_value
                    returns.insert(0, next_value)
                    _ , critic_val = model(states_tensor[t][None, :])
                    delta = rewards_tensor[t] + gamma * next_state_value - critic_val[0]
                    adv = delta + gamma * lamda * adv
                returns_tensor = tf.convert_to_tensor(returns, dtype=tf.float32)
                
                # Update actor-critic model
                train_step(states_tensor, actions_tensor, returns_tensor)
                break
            
            state = next_state

        # Print the episode reward
        print("Episode {}: Reward = {}".format(episode + 1, episode_reward))


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
TommIsim.SetUseGraphics( False )

# n.b. this will block until the simulation is complete
TommIsim.Run()

# Join the training thread back in
training_thread.join()

print("Done.")
