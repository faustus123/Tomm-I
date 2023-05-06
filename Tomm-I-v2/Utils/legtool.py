import tkinter as tk


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

#----------------- Servos 

# Parameters
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
MIN_ANG  =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG  =[180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]
pca = ServoKit(channels=16)
for i in range(16): pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])

#-----------------------------------
# ServosOff
#-----------------------------------
def ServosOff():
    # Turn all servos off
    for servo,chan in servo_map.items(): pca.servo[chan].fraction = None


def increase_value(slider, name):
    current_value = slider.get()
    if current_value < 180.0:
        slider.set(current_value + 1.0)

def decrease_value(slider, name):
    current_value = slider.get()
    if current_value > 0.0:
        slider.set(current_value - 1.0)

def UpdateServo(name, val):
    global pca
    global servo_map
    print("updating {} to {}".format(name, val))
    servo = pca.servo[servo_map[name]]
    servo.angle = float(val)


root = tk.Tk()
root.title("Slider Demo")

slider_order = ["FL2","FL1","BL1","BL2","FR2","FR1","BR1","BR2"]
sliders = {}

for i, label_text in enumerate(slider_order[:4]):
    label = tk.Label(root, text=label_text)
    label.grid(row=0, column=i*2, columnspan=2, padx=10)

    slider = tk.Scale(root, from_=0, to=180, resolution=1.0, orient=tk.HORIZONTAL, command=lambda v, n=label_text: UpdateServo(n, v))
    slider.grid(row=2, column=i*2, columnspan=2, padx=10)
    sliders[label_text] = slider

    increase_button = tk.Button(root, text="+", command=lambda s=slider, n=label_text: increase_value(s,n))
    increase_button.grid(row=1, column=i*2+1, padx=5, sticky="e")

    decrease_button = tk.Button(root, text="-", command=lambda s=slider, n=label_text: decrease_value(s,n))
    decrease_button.grid(row=1, column=i*2+0, padx=5, sticky="w")

for i, label_text in enumerate(slider_order[4:]):
    label = tk.Label(root, text=label_text)
    label.grid(row=3, column=(i)*2, columnspan=2, padx=10)

    slider = tk.Scale(root, from_=0, to=180, resolution=1.0, orient=tk.HORIZONTAL, command=lambda v, n=label_text: UpdateServo(n, v))
    slider.grid(row=5, column=(i)*2, columnspan=2, padx=10)
    sliders[label_text] = slider

    increase_button = tk.Button(root, text="+", command=lambda s=slider, n=label_text: increase_value(s,n))
    increase_button.grid(row=4, column=(i)*2+1, padx=5, sticky="e")

    decrease_button = tk.Button(root, text="-", command=lambda s=slider, n=label_text: decrease_value(s,n))
    decrease_button.grid(row=4, column=(i)*2+0, padx=5, sticky="w")

# Set all sliders to 90
for (label_text, slider) in sliders.items():
    slider.set(90)


root.mainloop()

ServosOff()
