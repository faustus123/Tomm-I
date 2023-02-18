#!/usr/bin/env python3
#
#  Run this on the robot (raspberry pi)
#

import os
import zmq
import time
import serial
import sys
import subprocess
import threading
import json

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


# Reduces the jitter on the servos
os.environ["GPIOZERO_PIN_FACTORY"] = "pigpio"
os.system("sudo pigpiod")


import RPi.GPIO as GPIO
from gpiozero import Servo


port = "71400"  # For zmq commands
Done = False


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

def arduino_state_read_thread():
	global arduino_state

	ser = serial.Serial(
        port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 115200,
        #baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1)

	ser.reset_input_buffer()
	last_time = time.time()
	while not Done:
        
		try:
			data = ser.readline().decode('utf-8').strip()
			arduino_state = json.loads(data)
			if (time.time()-last_time) >= 1.0 :
				print( arduino_state )
				last_time = time.time()
		except:
			pass


#------------------------------
# onboard_display_update_thread
#------------------------------
def onboard_display_update_thread():
	global raspi_status
	global arduino_state
	global draw, disp, width, height

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

all_threads = []
all_threads.append( {'name':'display'  , 'target':onboard_display_update_thread , 'proc':None})
all_threads.append( {'name':'arduino'  , 'target':arduino_state_read_thread , 'proc':None})


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
	print('Stopping all threads ...')
	Done = True
	for t in all_threads:
		print('  Joining thread for '+t['name'])
		t['proc'].join()



#=============================================================================

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:%s" % port)

print("Listening on port " + port + " ...")

StartAllThreads()


# Server loop
while not Done:
	#  Wait for next request from client
	command = socket.recv_string()
	print("Received request: " + command)
	if command.startswith('Hello'):
		mess = "Hola'!"

	elif command.startswith('quit'):
		Done = True
		mess = "Quitting ..."

	elif command.startswith('get_raspi_status'):
		mess = '\n'.join(raspi_status)

	else:
		mess = 'Unknown command: ' + command
	
	print(mess)
	socket.send_string(mess)

# Cleanup
StopAllThreads();
GPIO.cleanup()