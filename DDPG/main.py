# Author: Kishansingh Rajput, David Lawrence
# Script: Driver script for RL agent

import gym
import numpy as np
from Agent import Agent
from utils import plot_learning_curve
import env
import tensorflow as tf

if __name__ == '__main__':
    # env = gym.make('Pendulum-v0')
    agent = Agent(input_dims=env.observation_space.shape, env=env, n_actions=env.action_space.shape)
    n_games = 100

    figure_file = 'learning_curve.png'

    best_score = env.reward_range[0]
    score_history = []

    # Setting this to true will add a small amount of noise to the actions
    # the model suggests. (This is done in the Agent.choose_action() call).
    # The intent is to give the model a chance to wiggle out of a local
    # minimum it may fall into during training.
    add_noise = True

    # This will attempt to load model parameters from the saved model directories.
    # If a directory does not exist, it will skip it and use the default values
    # from when the model was created during instantiation of the Agent object.
    agent.load_models()

    # Loop over "games".
    for i in range(n_games):
        observation = env.reset()
        done = False
        score = 0
        prev_x = 0
        prev_y = 0
        prev_yaw = 0
        nSteps = 0
        while not done and nSteps < 200:
            nSteps += 1
            action = agent.choose_action(observation, add_noise)
            observation_, reward, done, info = env.step(action, prev_x, prev_y, prev_yaw)
            prev_x = info[0]
            prev_y = info[1]
            prev_yaw = info[3]
            score += reward
            agent.remember(observation, action, reward, observation_, done)
            agent.learn()
            observation = observation_

        # Add total reward score to history and print average score for last
        # 100 games.
        score_history.append(score)
        avg_score = np.mean(score_history[-100:])
        print("avg_score={}".format(avg_score))

        # Save the best model
        if avg_score > best_score:
            best_score = avg_score
            agent.save_models()

        # For every 10th game, evaluate performance without adding any additional
        # noise to the actions.
        if i%10 == 0:
            prev_x = 0
            observation = env.reset()
            total_reward = 0
            for j in range(200):
                #print(" ... Evaluating ... ")
                action = agent.choose_action(observation, False)
                observation_, reward, done, info = env.step(action, prev_x, prev_y, prev_yaw)
                prev_x = info[0]
                prev_y = info[1]
                prev_yaw = info[3]
                total_reward += reward
                # tf.print(action_record.shape)
                observation = observation_
                if done:
                    break
            print("Evaluation reward: ", total_reward)


        print('game ', i, 'score %.3f' % score, 'avg score %.3f' % avg_score)

    # All Done. Plot learning curve
    x = [i+1 for i in range(n_games)]
    plot_learning_curve(x, score_history, figure_file)
