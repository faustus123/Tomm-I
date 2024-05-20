#!/usr/bin/env python3
#
# This is the program used to control the robot. 
#
# This reads the robot status from the pipe at /tmp/robot_status and controls
# the robot actuators via the Raspberry Pi I2C interface (among others)
#
#

import os
import select
import time
import sys
import threading
import json
from adafruit_servokit import ServoKit

Done = False
DataSource = "pipe"  # "zmq", "pipe", "serial"  (zmq and pipe require robot_statsd.py to be running)
arduino_state = {}
servo2adc_map = {
    'BL1':'A0',
    'BL2':'A1',
    'BR2':'A2',
    'BR1':'A3',

    'FR1':'A4',
    'FR2':'A5',
    'FL2':'A6',
    'FL1':'A7'
      }
servo_map = {
    'BL1':15,
    'BL2':13,
    'BR2':2,
    'BR1':0,

    'FR1':1,
    'FR2':3,
    'FL2':12,
    'FL1':14
      }

last_update_actuators_time = time.time()


#----------------- Servos 

# Parameters
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
MIN_ANG  =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG  =[180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]
pca = ServoKit(channels=16)
for i in range(16): pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])


class TommIservo:
    def __init__():
        pass
    

#-----------------------------------
# ServosOff
#-----------------------------------
def ServosOff():
    # Turn all servos off
    for servo,chan in servo_map.items(): pca.servo[chan].fraction = None

#-----------------------------------
# WaitForState
#
# Wait for the arduino_state dictionary to be filled by
# the read thread. This is used at program start to ensure
# arduino_state has values in it before proceeding.
#-----------------------------------
def WaitForState():
    global arduino_state
    global Done
    
    for i in range(150):
        if len(arduino_state.keys()) >= 10: return
        time.sleep(0.010)
    
    Done = True;
    print('Timeout waiting for state to be read')
    time.sleep(0.5)
    sys.exit(-1)

    
#-----------------------------------
# testing
#-----------------------------------
def testing():
    global pca
    global servo_map
    
    all = ['FR1', 'FR2', 'BR1', 'BR2', 'FL1', 'FL2', 'BL1', 'BL2']

    print('Setting all servos to 90 degrees ...')
    for s in all:
        servo = pca.servo[servo_map[s]]
        servo.angle = 90.0
        
    time.sleep(1)
    print('complete.')

#-----------------------------------
# stand
#-----------------------------------
def stand():
    global arduino_state
    global pca
    global servo_map
    global Done
    global cycle_time
    
    all = {'FR1':40, 'FR2':90, 'BR1':140, 'BR2':65, 'FL1':140, 'FL2':30, 'BL1':20, 'BL2':90}

    print('Setting all servos to 90 degrees ...')
    for s,a in all.items():
        servo = pca.servo[servo_map[s]]
        servo.angle = a
        time.sleep(0.75)
        
    time.sleep(1)
    print('complete.')


#-----------------------------------
# calibrate_servos
#-----------------------------------
def calibrate_servos():
    global arduino_state
    global pca
    global servo_map
    global Done
    global cycle_time
    
    print('Calibrating servos ...')
    
    
    # Turn all servos off
    ServosOff()
    
    # Determine the 
    
    
    s = 'FR2'
    adc = servo2adc_map[s]
    servo = pca.servo[servo_map[s]]
    min_angle = 15
    max_angle = 160
    
    # calibrate max speed
    servo.angle = min_angle
    time.sleep(1.2)
    min_pos = arduino_state[adc]
    servo.angle = max_angle
    time.sleep(1.2)
    max_pos = arduino_state[adc]
    start_time = time.time()
    end_time = 0.0
    servo.angle = min_angle
    for i in range(1500):
        if abs(arduino_state[adc] - min_pos) < 8 :
            end_time = time.time()
            break
        time.sleep(0.010)
    if end_time > start_time:
        deg_per_sec_backward = (max_angle - min_angle)/(end_time - start_time)
    else:
        print('Timeout waiting for angle to return to min!')

    time.sleep(0.5)
    start_time = time.time()
    end_time = 0.0
    servo.angle = max_angle
    for i in range(1500):
        if abs(arduino_state[adc] - max_pos) < 8 :
            end_time = time.time()
            break
        time.sleep(0.010)
    if end_time > start_time:
        deg_per_sec_forward = (max_angle - min_angle)/(end_time - start_time)
    else:
        print('Timeout waiting for angle to return to max!')

    deg_per_50ms = min(deg_per_sec_forward, deg_per_sec_backward)*0.050
    print('{} degrees per second: forward={:4.1f} backward={:4.1f} max degrees/50ms={:4.2f}'.format(s, deg_per_sec_forward, deg_per_sec_backward, deg_per_50ms))
   
    
    # for A in range(15,160,1):
    #     my_state = arduino_state
    #     if adc in my_state.keys():
    #         print('chan={} pos={}'.format(s, my_state[adc]))
    #     servo.angle = A
    #     time.sleep(0.04)
    
    # time.sleep(1)
    # print('chan={} pos={}'.format(s, arduino_state[adc]))

    # Turn all servos off
    for servo,chan in servo_map.items(): pca.servo[chan].fraction = None
    
    Done = True

#-----------------------------------
# update_actuators
#-----------------------------------
def update_actuators():
    global arduino_state
    global last_update_actuators_time
    
    
    
    if (time.time() - last_update_actuators_time) > 1.0:
        print('New state: millis={}'.format(arduino_state['millis']))
        last_update_actuators_time = time.time()
    
#-----------------------------------
# arduino_state_read_thread_pipe
#-----------------------------------
def arduino_state_read_thread_pipe():
    global arduino_state
    global Done
    global cycle_time
    
    path = "/tmp/robot_status"
    fifo_fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK )
    if fifo_fd >= 0:
        print('Opened fifo: {}'.format(path))    
    else:
        print('Unable to open fifo: {}'.format(path))
        Done = True
        return

    fifo = os.fdopen(fifo_fd)

    last_time = time.time()
    cycle_time = 0.050 # intialize to something reasonable
    while fifo and not Done:
        
        try:
            ready = select.select([fifo_fd], [], [], 1)
            if ready[0] :
                data = fifo.readline().strip()
                if len(data)>2 :
                    arduino_state = json.loads(data)
                    if not Done:
                        update_actuators()
                        delta_time = time.time() - last_time
                        cycle_time = 0.93*cycle_time + 0.07*delta_time
                        last_time = time.time()
            if (time.time()-last_time) >= 0.5 :
                print('WARNING: unable to read robot status from pipe!')
                last_time = time.time()
        except Exception as e:
            print(e)
    print("arduino_state_read_thread_pipe stopping")


#------------------------------
# StartAllThreads
#------------------------------
def StartAllThreads():
    global last_tread_thread_start_time
    for t in all_threads:
        t['proc'] = threading.Thread( target=t['target'] )
        print('Starting thread for '+t['name'])
        t['proc'].start()
    last_tread_thread_start_time = time.time()

#------------------------------
# StopAllThreads
#------------------------------
def StopAllThreads():
    global Done
    print('Stopping all threads ...')
    Done = True
    for t in all_threads:
        print('  Joining thread for '+t['name'])
        t['proc'].join()
    print("All threads stopped.")

#------------------------------
# Quit
#------------------------------
def Quit():
    global root
    StopAllThreads()
    root.destroy()

#==============================================================

# Turn off all servo motors
ServosOff()

# Currently will accept a single argument specifying the
# source to use. No argument defaults to pipe.
if len(sys.argv)>1: DataSource = sys.argv[1]

# Specify the reader thread depending on the data source
all_threads = []
if DataSource == 'zmq':
    #all_threads.append( {'name':'arduino'  , 'target':arduino_state_read_thread_zmq    , 'proc':None})
    pass
elif DataSource == 'serial':
    #all_threads.append( {'name':'arduino'  , 'target':arduino_state_read_thread_serial , 'proc':None})
    pass
elif DataSource == 'pipe':
    all_threads.append( {'name':'arduino'  , 'target':arduino_state_read_thread_pipe   , 'proc':None})
else:
    print('DataSource {} is unknown. Should be one of "zmq", "serial", or "pipe"'.format( DataSource ))
    sys.exit()



# Start threads
StartAllThreads()

# Allow threads to start up and for the cycle_time to become primed
WaitForState()

# stand()

testing()

# Calibrate if specified
#calibrate_servos()

#while not Done: time.sleep(1)

# Turn off all servo motors
ServosOff()
