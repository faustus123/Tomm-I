#!/usr/bin/env python3
#
#  Make sure the Tomm-I/Tomm-I-sim/python_modules directory is
# in your PYTHONPATH.
#
#  e.g.
#
#     setenv PYTHONPATH $HOME/builds/Tomm-I/Tomm-I-sim/python_modules
#
#

import TommIsim
import zmq
import json
import time

context = zmq.Context()

# Define the socket using the "Context"
sock = context.socket(zmq.REP)
sock.bind("tcp://127.0.0.1:5678")


keys = ["FR_hip", "FL_hip", "BR_hip", "BL_hip", "FR_foot", "FL_foot", "BR_foot", "BL_foot"]
motor = {}
for key in keys:
    motor[key] = 120

message = sock.recv().decode("utf-8")

def checkStatus(motor):
    global keys
    status = TommIsim.GetStatus()
    margin = 0.2
    for i,key in enumerate(motor.keys()):
        if key not in status.keys(): continue
        if abs(motor[key] - status[key]) > margin:
            return False, i

    return True, -1
    
def printComparision(status, motor):
    global keys
    for key in keys:
        print(status[key], motor[key])

def softReset():
    global motor
    global count 
    status = TommIsim.GetStatus()
    motor['BL_hip'] = status['BL_hip']
    motor['BR_hip'] = status['BR_hip']
    motor['BL_foot'] =  status['BL_foot']
    motor['BR_foot'] = status['BR_foot']
    motor['FL_hip'] = status['FL_hip']
    motor['FR_hip'] = status['FR_hip']
    motor['FL_foot'] = status['FL_foot']
    motor['FR_foot'] = status['FR_foot']

            

count = 0
metaCount = 0
# status = {}
prev_status = {}
def MyPythonCallback():
    global motor
    global count
    global metaCount
    # global status
    global keys
    global prev_status
    if metaCount > 50:
        done, where = checkStatus(motor)
        # flag = False
        # for key in prev_status:
        #     if status[key] - prev_status[key] > 0.001:
        #         flag = True
        #         prev_status = status
        #         # print(key)
        #         break

        # if not flag:
        if done:
            # time.sleep(5)

            status = TommIsim.GetStatus()
            for key in keys:
                if "foot" in key: # or "FR_" in key:
                    status[key] = (status[key] - 130)/10
                else:
                    status[key] = (status[key] - 120)/10
            for key in ['x', 'y', 'z']:
                status[key] = status[key]/10
            for key in ['yaw', 'pitch', 'roll']:
                status[key] = status[key]/180

            # prev_status = status
            # print(status, prev_status)
            # for key in keys:
            #     status[key] = (status[key] - 110)/10
            # for key in ['x', 'y', 'z']:
            #     status[key] = status[key]/10
            # for key in ['yaw', 'pitch', 'roll']:
            #     status[key] = status[key]/180
            content = json.dumps(status)
            sock.send_string(content)
            message = sock.recv().decode("utf-8") 
            if "Reset" in message:
                TommIsim.Reset()
                metaCount = 0
                motor = {'BL_foot': 120, 'BL_hip': 120, 'BR_foot': 120, 'BR_hip': 120, 'FL_foot': 120, 'FL_hip': 120, 'FR_foot': 120, 'FR_hip': 120}
            else:
                motor = json.loads(message)
                for key in motor.keys():
                    if "foot" in key: # or "FR_" in key:
                        motor[key] = 130 + motor[key]*10
                    else:
                        motor[key] = 120 + motor[key]*10
                TommIsim.SetMotors(motor)
           
        else:
            # print(status, prev_status)
            count += 1
            if count > 1000:
                status = TommIsim.GetStatus()
                print("################## DEAD END #####################", keys[where], status[keys[where]], motor[keys[where]])
                softReset()
                count = 0
                
                TommIsim.SetMotors(motor)
                # status = TommIsim.GetStatus()
                # print("Where: ", where)
                # printComparision(status, motor)
    else:
        metaCount += 1
        TommIsim.SetMotors(motor)

#======================================================================
TommIsim.SetRunRealTime( False )
# TommIsim.SetUsePreprogrammedActions( True ) # comment this line to turn off the pre-programmed sequence used for the initial video
TommIsim.Setup()
TommIsim.RegisterCallback(MyPythonCallback)

# n.b. this will block until the simulation is complete
TommIsim.Run()

print("Done.")
