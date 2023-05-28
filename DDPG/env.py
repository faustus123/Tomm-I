# Author: Kishansingh Rajput, David Lawrence
# Script: Interface to the environment


import zmq
import json

# ZeroMQ Context
context = zmq.Context()

# Define the socket using the "Context"
sock = context.socket(zmq.REQ)
sock.connect("tcp://127.0.0.1:5678")
action = [120, 120, 120, 120, 120, 120, 120, 120]

# Global counter to keep track of how many times step is called since last reset.
count = 0

class action_space_class:
    def __init__(self):
        # props = {'high':[140, 140, 140, 140, 160, 160, 160, 160], 'low':[90, 90,90,90,110,110,110,110], 'shape':8}
        # props = {'high':[140, 140, 140, 140, 180, 180, 180, 180], 'low':[45, 45,45, 45,50,50,50,50], 'shape':8}
        props = {'high':[1,1,1,1,1,1,1,1], 'low':[-1,-1,-1,-1,-1,-1,-1,-1], 'shape':8}
        self.shape = props['shape']
        self.high = props['high']
        self.low = props['low']

class observation_space_class:
    def __init__(self):
        self.shape = 11

action_space = action_space_class()
observation_space = observation_space_class()
reward_range = [0, -400]


def convertToDict(action):
    keys = ["FR_hip", "FL_hip", "BR_hip", "BL_hip", "FR_foot", "FL_foot", "BR_foot", "BL_foot"]
    motor = {}
    if len(keys) != len(action):
        #breakpoint()
        print("WARNING in env.py:converToDict:  len(keys)!=len(action)   {}!={}".format(len(keys),len(action)))
    else:
        for i in range(len(keys)):
            motor[keys[i]] = float(action[i])
    return motor

def getObservations(status):
    keys = ["FR_hip", "FL_hip", "BR_hip", "BL_hip", "FR_foot", "FL_foot", "BR_foot", "BL_foot"]
    obs = []
    for i in range(len(keys)):
        obs.append(status[keys[i]])
    for key in ["yaw", "pitch", "roll"]:
        obs.append(status[key])
    return obs

def step(action, prev_x=0, prev_y=0, prev_yaw=0):
    global count
    if not isinstance(action, str):
        action = convertToDict(action)
    # print("sending: ", action)
    content = json.dumps(action)
    sock.send_string(content)
    msg = sock.recv().decode('utf-8')
    status = json.loads(msg)
    obs = getObservations(status)
    #reward = (status['x'] - prev_x-0.2)*20 - abs(status['y']-prev_y)*2 #- max(0, abs(status['roll'])-0.01)*10 - max(0, abs(status['yaw'])-0.01)*10 - max(0, abs(status['pitch'])-0.01)*10
    reward = status['x'] - 0.25*abs(status['y'])
    if isinstance(action, dict):
        sactions = ["{:+.2f} ".format(float(v)) for v in list(action.values())[:8]]
        print("Model output: {} x={:.3f} y={:.3f} yaw={:.3f} reward={:.3f}".format(sactions, status['x'], status['y'], status['yaw'], reward))
    done = False
    count += 1
    if count > 2000:
        done = True
        print("Greater than 2000 calls to env.step(). Ending current game ...")
    if status['z'] < 0.015:
        done = True
        reward -= 500
        print("Robot z indicates robot fell over. Ending current game ...")
        # if status['z'] == -10:
        #     reward += 10
    axis = [status['x'], status['y'], status['z'], status['yaw']]
    return obs, reward, done, axis


def reset():
    global count
    print("Reset Called!", count)
    count = 0
    action = "Reset"
    obs, reward, done, axis = step(action)
    return obs


step(action)
