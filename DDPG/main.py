# Author: Kishansingh Rajput
# Script: Driver script for RL agent

import gym
import numpy as np
from Agent import Agent
from utils import plot_learning_curve
import env
import tensorflow as tf

if __name__ == '__main__':
    # env = gym.make('Pendulum-v0')
    agent = Agent(input_dims=env.observation_space.shape, env=env,
            n_actions=env.action_space.shape)
    n_games = 3

    figure_file = 'pendulum.png'

    best_score = env.reward_range[0]
    score_history = []
    load_checkpoint = True

    if load_checkpoint:
        n_steps = 0
        while n_steps <= agent.batch_size:
            observation = env.reset()
            # action = env.action_space.sample()
            action = []
            observation_, reward, done, info = env.step(action)
            agent.remember(observation, action, reward, observation_, done)
            n_steps += 1
        agent.learn()
        agent.load_models()
        evaluate = True
    else:
        evaluate = False

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
            action = agent.choose_action(observation, evaluate)
            observation_, reward, done, info = env.step(action, prev_x, prev_y, prev_yaw)
            prev_x = info[0]
            prev_y = info[1]
            prev_yaw = info[3]
            score += reward
            agent.remember(observation, action, reward, observation_, done)
            if not load_checkpoint:
                agent.learn()
            observation = observation_

        score_history.append(score)
        avg_score = np.mean(score_history[-100:])
        print("avg_score={}".format(avg_score))

        if avg_score > best_score:
            best_score = avg_score
            if not load_checkpoint:
                agent.save_models()
        if i%10 == 0:
            prev_x = 0
            observation = env.reset()
            total_reward = 0
            for j in range(200):
                #print(" ... Evaluating ... ")
                action = agent.choose_action(observation, True)
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


        print('episode ', i, 'score %.1f' % score, 'avg score %.1f' % avg_score)

    if not load_checkpoint:
        x = [i+1 for i in range(n_games)]
        plot_learning_curve(x, score_history, figure_file)
