#!/usr/bin/env python3
#
#  This is run automatically at start-up and runs continuously.
#
# This script will open the /dev/ttyS0 serial connection to the
# Arduino Mega board and continually read the status from it.
# It then distributes this information to 3 places:
#
# 1. It updates the onboard OLED status screen. It does this at a
#    limited rate.
#
# 2. It creates a fifo (pipe) /tmp/robot_status that it writes the
#    most recent JSON status string to at high rate. This can be
#    used by one other program that consumes the information. That
#    program may then send messages back to the arduino directly
#    via the /dev/serial1 device to complete a full control loop.
#
# 3. It sends the JSON string to a zeroMQ PUB-SUB socket at a
#    limited rate. This is intended for the GUI monitor. Being
#    PUB-SUB also allows for other potential programs to access
#    the robot status information.
#
# Use the RobotStatus.py script to launch a GUI that monitors
# telemetry from the robot.

import os
import zmq
import time
import serial
import subprocess
import threading
import json

import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


# Reduces the jitter on the servos
os.environ["GPIOZERO_PIN_FACTORY"] = "pigpio"
os.system("sudo pigpiod")


import RPi.GPIO as GPIO


port = "71400"  # For zmq commands
Done = False
ShutdownIntiated = False;  # Gets set to true when a shutdown is initiated due to low battery


raspi_status = []


# Initialize onboard display
RST = None     # on the PiOLED this pin isnt used
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0
disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST) # 128x64 display with hardware I2C:
disp.begin()
disp.clear()
disp.display()
padding=-2
top = padding
width = disp.width
height = disp.height
image = Image.new('1', (width, height)) # Create blank image for drawing. '1' for 1-bit color.
draw = ImageDraw.Draw(image) # Get drawing object to draw on image.
draw.rectangle((0,0,width,height), outline=0, fill=0) # Clear screen
font = ImageFont.load_default() # Load default font.
draw.text((10, top+8),  '... starting up ....', font=font, fill=255)
disp.image(image)
disp.display()

arduino_state = {}

#------------------------------
# arduino_state_read_thread
#------------------------------
def arduino_state_read_thread():
    global arduino_state_json
    global arduino_state
    global Done

    ser = serial.Serial(
        port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 115200,
        #baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1)

    ser.reset_input_buffer()
    while not Done:
        
        try:
            arduino_state_json = ser.readline().decode('utf-8').strip()
            arduino_state = json.loads(arduino_state_json)
        except:
            pass


#------------------------------
# onboard_display_update_thread
#------------------------------
def onboard_display_update_thread():
    global raspi_status
    global arduino_state
    global draw, disp, width, height
    global Done

    while not Done:
        # Draw a black filled box to clear the image.
        draw.rectangle((0,0,width,height), outline=0, fill=0)

        # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
        cmd = "hostname -I"
        IP = subprocess.check_output(cmd, shell = True )
        #cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
        cmd = "top -bn1 | grep Cpu | awk '{printf \"%.2f\", $(NF-9)}'"
        CPU = subprocess.check_output(cmd, shell = True )
        CPU = "CPU: %.2f%%" % (100.0-float(str(CPU,'utf-8')))
        cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
        MemUsage = subprocess.check_output(cmd, shell = True )
        cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
        Disk = subprocess.check_output(cmd, shell = True )
        cmd = "vcgencmd measure_temp |cut -f 2 -d '='"
        temp = subprocess.check_output(cmd, shell = True )
        
        # Copy lines into global list so it can be sent to remote host upon request
        raspi_status = [
            "IP: " + str(IP,'utf-8'),
            str(CPU) + " " + str(temp,'utf-8'),
            str(MemUsage,'utf-8'),
            str(Disk,'utf-8'),
        ]
        
        battery_str = "battery: <unavailable>"
        if "battery_voltage" in arduino_state:
            battery_str = "battery: {}V  {}%".format( arduino_state["battery_voltage"], arduino_state["battery_percent"] )
        raspi_status.append(battery_str)

        # Write all lines of text.
        x=0
        y=top
        for line in raspi_status:
            draw.text((x, y),  line, font=font, fill=255)
            y += 8

        # Display image.
        disp.image(image)
        disp.display()
        time.sleep(1.)
        
        if float(arduino_state["battery_percent"]) < 90.0: Shutdown()

#------------------------------
# Shutdown
#------------------------------
def Shutdown():
    
    # This is called by the onboard_display_update_thread when the battery
    # level has dropped too low and we need to shut the robot down. This
    # is primarily to save the SD card from getting corrupted.
    global raspi_status
    global arduino_state
    global draw, disp, width, height
    global Done
    
    cmd = "sudo shutdown --poweroff +1"   # Shutdown in 1 min.
    subprocess.run(cmd, shell = True )

    start_time = time.time()
    while (time.time() - start_time)<=60:

        # Draw a black filled box to clear the image.
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        
        raspi_status = ['*** WARNING!!! ***']
        raspi_status.append('low battery: {}%'.format( arduino_state["battery_percent"] ))
        raspi_status.append(' ')
        raspi_status.append('SHUTTING DOWN!')
        raspi_status.append(' ')
        raspi_status.append('{} seconds remaining'.format(int(60 - (time.time() - start_time))))
        
         # Write all lines of text.
        x=0
        y=top
        for line in raspi_status:
            draw.text((x, y),  line, font=font, fill=255)
            y += 8

        # Display image.
        disp.image(image)
        disp.display()
        time.sleep(1.0)
    Done = True

#------------------------------
# pipe_update_thread
#------------------------------
def pipe_update_thread():
    global Done
    global arduino_state_json
    global arduino_state
    
    #fifo_delay = 0.1 # seconds to delay between fifo writes

    while not Done:
        
        path = "/tmp/robot_status"
        if not os.path.exists(path):
            try:
                os.mkfifo(path)
            except OSError as e:
                print('Failed to create FIFO: {}'.format(e))
        
        # n.b. we can't use the the "with open..." syntax here since 
        # it will cause an exception within an exception when the pipe
        # is closed by the client.
        print("Opening high speed FIFO: {}".format(path))
        fifo = open(path, 'w')
        if fifo:
            print("High speed FIFO connected on {}".format(path))
        
            last_millis = 0
            fifo_good = True
            while fifo_good and not Done:
                if 'millis' in arduino_state.keys() and arduino_state['millis'] != last_millis:
                    try:
                        fifo.write( arduino_state_json+'\n' )
                        fifo.flush()
                        last_millis = arduino_state['millis']
                        #time.sleep( fifo_delay )
                    except:
                        print('FIFO broken. Restarting.')
                        fifo_good = False
                else:
                    # The ardunio state has not been updated since we last wrote it
                    # Sleep for small amount of time to free up CPU.
                    time.sleep(0.001)
            


#------------------------------
# zmq_update_thread
#------------------------------
def zmq_update_thread():
    global Done
    global arduino_state_json
    global arduino_state
    global port
    
    # Minimum time to sleep between publishes
    publish_delay = 0.2

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:%s" % port)
    
    print("Publishing on port " + port + " ...")
    
    last_millis = 0
    while not Done:
        if 'millis' in arduino_state.keys() and arduino_state['millis'] != last_millis:
            socket.send_string(arduino_state_json)
            last_millis = arduino_state['millis']
            time.sleep( publish_delay )
            


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Make list of threads to run
#
all_threads = []
all_threads.append( {'name':'arduino'  , 'target':arduino_state_read_thread,     'proc':None})
all_threads.append( {'name':'display'  , 'target':onboard_display_update_thread, 'proc':None})
all_threads.append( {'name':'pipe'     , 'target':pipe_update_thread,            'proc':None})
all_threads.append( {'name':'zmq'      , 'target':zmq_update_thread,             'proc':None})
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


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



#=============================================================================


StartAllThreads()


# Server loop
while not Done:
    #  Wait for next request from client
    # command = socket.recv_string()
    # print("Received request: " + command)
    # if command.startswith('Hello'):
    #     mess = "Hola'!"

    # elif command.startswith('quit'):
    #     Done = True
    #     mess = "Quitting ..."

    # elif command.startswith('get_raspi_status'):
    #     mess = '\n'.join(raspi_status)

    # else:
    #     mess = 'Unknown command: ' + command
    
    # print(mess)
    # socket.send_string(mess)
    time.sleep(1.0)

# Cleanup
StopAllThreads();
GPIO.cleanup()