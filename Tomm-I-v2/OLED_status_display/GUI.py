#
# This script will provide a monitoring GUI for the robot status.
# It is capable of reading from all 3 sources of the status information:
#
#  zmq    - PUB-SUB socket on port 71400 (default)
#  fifo   - (i.e. named pipe /tmp/robot_status)
#  serial - directly from arduino serial connection on /dev/ttyS0
#
# The zmq and serial methods require that the robot_statusd.py script
# be running. That script is automatically run at system start up
# to update the OLED display on the robot and to publish the robot
# status to the zmq and fifo methods.
#

import os
import select
import socket
import time
import serial
import sys
import subprocess
import threading
import json
import zmq


import tkinter as tk

port = "71400"  # For zmq commands
Done = False
DataSource = "zmq"  # "zmq", "pipe", "serial"  (zmq and pipe require robot_statsd.py to be running)
arduino_state = {}
LABEL = {}
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
metadata_items = []

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
    root.title("Robot Status")
    
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

    # ------ Pack -------
    metadata_frame.grid(row=0, column=0, sticky='we')
    #battery_frame.grid(row=0, column=1)
    systeminfo_frame.grid(row=0, column=1)
    orientation_frame.grid(row=1, column=1, sticky='we')
    servo_frame.grid(row=1, column=0, sticky='we')
    quit_button.grid(row=2, column=0, columnspan=2)
    
#-----------------------------------
# arduino_state_read_thread_serial
#-----------------------------------
def arduino_state_read_thread_serial():
    global arduino_state
    global Done
    
    path = '/dev/ttyS0' #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
    LABEL['datasource'].config(text=path)

    ser = serial.Serial(
        port=path, 
        baudrate = 115200,
        #baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1)

    ser.reset_input_buffer()

    last_time = time.time()
    while ser and not Done:
        
        try:
            data = ser.readline().decode('utf-8').strip()
            if len(data)>2 : arduino_state = json.loads(data)
            if (time.time()-last_time) >= 0.1 :
                if not Done : update_labels()
                last_time = time.time()
        except Exception as e:
            print(e)
    print("arduino_state_read_thread_serial stopping")


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
            #data = ser.readline().decode('utf-8').strip()
            ready = select.select([fifo_fd], [], [], 1)
            if ready[0] :
                data = fifo.readline().strip()
                if len(data)>2 : arduino_state = json.loads(data)
            if (time.time()-last_time) >= 0.1 :
                if not Done : update_labels()
                last_time = time.time()
        except Exception as e:
            print(e)
    print("arduino_state_read_thread_pipe stopping")

#-----------------------------------
# arduino_state_read_thread_zmq
#-----------------------------------
def arduino_state_read_thread_zmq():
    global arduino_state
    global Done
    global port
    
    # Socket to talk to server
    context = zmq.Context()
    zmqsocket = context.socket(zmq.SUB)
    zmqsocket.connect("tcp://localhost:%s" % port)
    LABEL['datasource'].config(text="ZMQ port {}".format(port))
    zmqsocket.subscribe("") # Subscribe to everything

    last_time = time.time()
    while zmqsocket and not Done:
        
        try:
            data = zmqsocket.recv(flags=zmq.NOBLOCK).strip()
            arduino_state = json.loads(data)
            if (time.time()-last_time) >= 0.1 :
                if not Done : update_labels()
                last_time = time.time()
        except zmq.Again as e:
            time.sleep( 0.010 )  # No message available. Sleep a bit

    print("arduino_state_read_thread_zmq stopping")
    
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
    all_threads.append( {'name':'arduino'  , 'target':arduino_state_read_thread_zmq    , 'proc':None})
elif DataSource == 'serial':
    all_threads.append( {'name':'arduino'  , 'target':arduino_state_read_thread_serial , 'proc':None})
elif DataSource == 'pipe':
    all_threads.append( {'name':'arduino'  , 'target':arduino_state_read_thread_pipe   , 'proc':None})
else:
    print('DataSource {} is unknown. Should be one of "zmq", "serial", or "pipe"'.format( DataSource ))
    sys.exit()


# Create GUI and run it
CreateGUI()
StartAllThreads()
root.mainloop()

