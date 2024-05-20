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
#import socket
import time
#import serial
import sys
#import subprocess
import threading
import json
#import zmq
from adafruit_servokit import ServoKit

import tkinter as tk


Done = False
DataSource = "pipe"  # "zmq", "pipe", "serial"  (zmq and pipe require robot_statsd.py to be running)
arduino_state = {}
LABEL = {}
servo_control = {} # tkinter slider that holders current set value. Key is servo name. Filled in create_control_frame
servo_enable = {}  # tkinter variable holding state of on/off setting. Key is servo name. Filled in create_control_frame
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
metadata_items = []

last_update_actuators_time = time.time()


#----------------- Servos 

# Parameters
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
MIN_ANG  =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG  =[180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]
pca = ServoKit(channels=16)
for i in range(16): pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])


#-----------------------------------
# create_control_frame
#-----------------------------------
def create_control_frame(parent):
    control_frame = tk.Frame(parent)

    # Function to create a row for a servo control
    def create_servo_control(frame, name, row):
        global servo_control
        global servo_enable
        label = tk.Label(frame, text=name)
        label.grid(row=row, column=0, padx=5, pady=5)

        slider = tk.Scale(frame, from_=500, to=2500, orient=tk.HORIZONTAL)
        servo_control[name] = slider
        slider.grid(row=row, column=1, columnspan=4, padx=5, pady=5)

        btn_dec_10 = tk.Button(frame, text="-10", command=lambda: adjust_servo(slider, -10))
        btn_dec_10.grid(row=row, column=5, padx=2, pady=2)

        btn_dec_1 = tk.Button(frame, text="-1", command=lambda: adjust_servo(slider, -1))
        btn_dec_1.grid(row=row, column=6, padx=2, pady=2)

        btn_inc_1 = tk.Button(frame, text="+1", command=lambda: adjust_servo(slider, 1))
        btn_inc_1.grid(row=row, column=7, padx=2, pady=2)

        btn_inc_10 = tk.Button(frame, text="+10", command=lambda: adjust_servo(slider, 10))
        btn_inc_10.grid(row=row, column=8, padx=2, pady=2)

        toggle_var = tk.IntVar()
        servo_enable[name] = toggle_var
        toggle = tk.Checkbutton(frame, text="On/Off", variable=toggle_var, command=lambda: toggle_controls(slider, btn_dec_10, btn_dec_1, btn_inc_1, btn_inc_10, label, toggle_var))
        toggle.grid(row=row, column=9, padx=5, pady=5)

        # Initialize controls state
        toggle_controls(slider, btn_dec_10, btn_dec_1, btn_inc_1, btn_inc_10, label, toggle_var)

    # Function to adjust the servo value
    def adjust_servo(slider, delta):
        new_value = slider.get() + delta
        new_value = max(500, min(2500, new_value))
        slider.set(new_value)

    # Function to toggle control state
    def toggle_controls(slider, btn_dec_10, btn_dec_1, btn_inc_1, btn_inc_10, label, toggle_var):
        state = tk.NORMAL if toggle_var.get() else tk.DISABLED
        color = "black" if toggle_var.get() else "grey"
        slider.config(state=state)
        btn_dec_10.config(state=state)
        btn_dec_1.config(state=state)
        btn_inc_1.config(state=state)
        btn_inc_10.config(state=state)
        label.config(foreground=color)

    # Servo motor names
    servos = ["FL1", "FL2", "FR1", "FR2", "BL1", "BL2", "BR1", "BR2"]

    # Create buttons for Resting, Stand, All On, All Off
    button_frame = tk.Frame(control_frame)
    button_frame.grid(row=0, column=0, columnspan=10, pady=10)

    tk.Button(button_frame, text="Rest Pos.", command=resting).grid(row=0, column=0, padx=5)
    tk.Button(button_frame, text="Stand", command=stand).grid(row=0, column=1, padx=5)
    tk.Button(button_frame, text="All On", command=lambda: set_all_on_off(True)).grid(row=0, column=2, padx=5)
    tk.Button(button_frame, text="All Off", command=lambda: set_all_on_off(False)).grid(row=0, column=3, padx=5)

    # Create servo controls
    for idx, servo in enumerate(servos):
        create_servo_control(control_frame, servo, idx + 1)  # Shift rows down by 1

    return control_frame

#-----------------------------------
# resting
#-----------------------------------
def resting():
    global servo_map

    print("Resting position activated.")

    for name in servo_map.keys():
        SetServoDegrees( name, 90.0 )    
    
    # Implement resting position logic here

#-----------------------------------
# stand
#-----------------------------------
def stand():
    print("Stand position activated.")
    # Implement stand position logic here

#-----------------------------------
# set_all_on_off
#-----------------------------------
def set_all_on_off(state):
    global servo_map
 
    print(f"Setting all servos to {'on' if state else 'off'}.")
    if state == 'off':
        for servo,chan in servo_map.items():
            pca.servo[chan].fraction = None

#-----------------------------------
# CreateGUI
#-----------------------------------
def CreateGUI():
    
    global root
    global LABEL
    global metadata_items
    global orientation_items

    # create main window
    root = tk.Tk()
    
    # set window title
    root.title("Robot Control")
    
    # System Info.
    systeminfo_frame = tk.Frame(root)
    
    # Data Source
    datasource_frame = tk.Frame(systeminfo_frame)
    datasource_frame.grid(row=0, column=0, sticky='we')
    datasource_title = tk.Label(datasource_frame, text="Data Source:")
    LABEL['datasource'] = tk.Label(datasource_frame, text="----")
    datasource_title.grid(row=0, column=0)
    LABEL['datasource'].grid(row=0, column=1)

    # Battery
    battery_frame = tk.Frame(systeminfo_frame)
    battery_frame.grid(row=1, column=0)
    battery_title = tk.Label(battery_frame, text="Battery:")
    LABEL['battery_voltage'] = tk.Label(battery_frame, text="----")
    LABEL['battery_percent'] = tk.Label(battery_frame, text="----")
    battery_title.grid(row=0, column=0)
    LABEL['battery_voltage'].grid(row=0, column=1)
    LABEL['battery_percent'].grid(row=0, column=2)
       
    # Metadata
    metadata_items = ['millis', 'device_read_time_ms', 'loop_time_ms', 'last_data_size']
    metadata_frame = tk.Frame(root, pady=5, padx=5, borderwidth=2, relief='groove')
    metadata_title = tk.Label(metadata_frame, text="Metadata", borderwidth=2, relief='raised')
    metadata_title.grid(row=0, column=0, columnspan=2, sticky='we')
    row = 1
    for m in metadata_items:
        title = tk.Label(metadata_frame, text='{}:'.format(m), anchor='e')
        LABEL[m] = tk.Label(metadata_frame, text="----")
        title.grid(row=row, column=0, sticky='we')
        LABEL[m].grid(row=row, column=1)
        row += 1
 
    # Orientation
    orientation_items = ['yaw_front', 'pitch_front', 'roll_front', 'dist_front']
    orientation_frame = tk.Frame(root, pady=5, padx=5, borderwidth=2, relief='groove')
    orientation_title = tk.Label(orientation_frame, text="Orientation", borderwidth=2, relief='raised')
    orientation_title.grid(row=0, column=0, columnspan=2, sticky='we')
    row = 1
    for m in orientation_items:
        title = tk.Label(orientation_frame, text='{}:'.format(m), anchor='e')
        LABEL[m] = tk.Label(orientation_frame, text="----")
        title.grid(row=row, column=0, sticky='we')
        LABEL[m].grid(row=row, column=1)
        row += 1
   
     
    # Servos
    servo_frame = tk.Frame(root, pady=5, padx=5, borderwidth=2, relief='groove')
    servo_title = tk.Label(servo_frame, text="SERVOS", borderwidth=2, relief='raised')
    servo_title.grid(column=0, row=0, columnspan=4, sticky='we')
    
    # create left column labels with titles
    fl1 = tk.Label(servo_frame, text="FL1")
    fl1_relief = tk.Label(servo_frame, text="--------", relief="sunken")
    fl2 = tk.Label(servo_frame, text="FL2")
    fl2_relief = tk.Label(servo_frame, text="--------", relief="sunken")
    bl1 = tk.Label(servo_frame, text="BL1")
    bl1_relief = tk.Label(servo_frame, text="--------", relief="sunken")
    bl2 = tk.Label(servo_frame, text="BL2")
    bl2_relief = tk.Label(servo_frame, text="--------", relief="sunken")
    
    # create right column labels with titles
    fr1_relief = tk.Label(servo_frame, text="--------", relief="sunken")
    fr1 = tk.Label(servo_frame, text="FR1")
    fr2_relief = tk.Label(servo_frame, text="--------", relief="sunken")
    fr2 = tk.Label(servo_frame, text="FR2")
    br1_relief = tk.Label(servo_frame, text="--------", relief="sunken")
    br1 = tk.Label(servo_frame, text="BR1")
    br2_relief = tk.Label(servo_frame, text="--------", relief="sunken")
    br2 = tk.Label(servo_frame, text="BR2")
    
    # create Quit button to close the window
    quit_button = tk.Button(root, text="Quit", command=Quit)
    
    # use grid layout to arrange the labels and button
    fl1.grid(row=1, column=0)
    fl1_relief.grid(row=1, column=1)
    fr1_relief.grid(row=1, column=2)
    fr1.grid(row=1, column=3)
    fl2.grid(row=2, column=0)
    fl2_relief.grid(row=2, column=1)
    fr2_relief.grid(row=2, column=2)
    fr2.grid(row=2, column=3)
    bl1.grid(row=3, column=0)
    bl1_relief.grid(row=3, column=1)
    br1_relief.grid(row=3, column=2)
    br1.grid(row=3, column=3)
    bl2.grid(row=4, column=0)
    bl2_relief.grid(row=4, column=1)
    br2_relief.grid(row=4, column=2)
    br2.grid(row=4, column=3)

    # Keep references to the labels we wish to update
    LABEL['FL1'] =  fl1_relief
    LABEL['FL2'] =  fl2_relief
    LABEL['BL1'] =  bl1_relief
    LABEL['BL2'] =  bl2_relief
    LABEL['FR1'] =  fr1_relief
    LABEL['FR2'] =  fr2_relief
    LABEL['BR1'] =  br1_relief
    LABEL['BR2'] =  br2_relief
    
    control_frame = create_control_frame(root)

    # ------ Pack -------
    metadata_frame.grid(row=0, column=0, sticky='we')
    #battery_frame.grid(row=0, column=1)
    systeminfo_frame.grid(row=0, column=1)
    orientation_frame.grid(row=1, column=1, sticky='we')
    servo_frame.grid(row=1, column=0, sticky='we')
    control_frame.grid(row=2, column=0, columnspan=2, sticky='we')
    quit_button.grid(row=3, column=0, columnspan=2)

#------------------------------
# update_labels
#------------------------------
def update_labels():
    global arduino_state
    global LABEL
    global servo2adc_map
    global metadata_items
    global orientation_items
    
    #print( arduino_state )

    # dummy check that at least some data is in arduino_state
    if len(arduino_state.keys()) < 2 : return
    
    LABEL['battery_voltage'].config( text='{}V'.format(arduino_state['battery_voltage']), width=6, bg='white', fg='green')
    LABEL['battery_percent'].config( text='({}%)'.format(arduino_state['battery_percent']), width=8, bg='white', fg='blue')
    for (s,a) in servo2adc_map.items():
        LABEL[s].config( text=arduino_state[a], width=8, bg='black', fg='yellow') 
    
    for m in metadata_items:
        if m in arduino_state.keys():
            LABEL[m].config( text=arduino_state[m], width=12, bg='grey', fg='black')
            
    for m in orientation_items:
        if m in arduino_state.keys():
            LABEL[m].config( text=arduino_state[m], width=10, bg='yellow', fg='blue')

#-----------------------------------
# SetServoDegrees
#-----------------------------------
def SetServoDegrees(name, angle):
    global servo_map
    global pca
    global servo_control
    global servo_enable

    if servo_enable[name]:
        servo = pca.servo[servo_map[name]]
        servo.angle = angle
    
    
#-----------------------------------
# calibrate_servos
#-----------------------------------
def calibrate_servos():
    global arduino_state
    global pca
    global servo_map
    global Done
    
    print('Calibrating servos ...')
    
    # Turn all servos off
    for servo,chan in servo_map.items(): pca.servo[chan].fraction = None
    
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
    
    path = "/tmp/robot_status"
    LABEL['datasource'].config(text=path)
    fifo_fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK )
    if fifo_fd >= 0:
        print('Opened fifo: {}'.format(path))    
    else:
        print('Unable to open fifo: {}'.format(path))
        Done = True
        return

    fifo = os.fdopen(fifo_fd)

    last_time = time.time()
    while fifo and not Done:
        
        try:
            ready = select.select([fifo_fd], [], [], 1)
            if ready[0] :
                data = fifo.readline().strip()
                if len(data)>2 :
                    arduino_state = json.loads(data)
                    if not Done:
                        update_actuators()
                        # last_time = time.time()
            if (time.time()-last_time) >= 0.1 :
                # print('WARNING: unable to read robot status from pipe!')
                if not Done : update_labels()
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

# Currently will accept a single argument specifying the
# source to use. No argument defaults to zmq.
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



# Create GUI and run it
CreateGUI()
StartAllThreads()
root.mainloop()

# Calibrate if specified
# calibrate_servos()

#while not Done: time.sleep(1)
