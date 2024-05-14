#!/usr/bin/env python3
#
# This is a simple utility to switch off all servos
#
#

import os
import select
import time
import sys
import threading
import json
from adafruit_servokit import ServoKit

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

print("Getting PCA device ... ")
pca = ServoKit(channels=16)

print("Turning off servos: ")
for servo,chan in servo_map.items():
    print(f"  {servo} : {chan}")
    pca.servo[chan].fraction = None

print("Done.")
