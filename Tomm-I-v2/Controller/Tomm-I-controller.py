#!/usr/bin/env python3
#
# This is the program used to control the robot. 

import tkinter as tk
from tkinter import ttk
from adafruit_servokit import ServoKit
import json
import os
import time

#----------------- Servos 
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

# Parameters
# TODO: Make the slider limits and angle to pulse width calculations match these
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
pca = ServoKit(channels=16)
for i in range(16): pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])

# Calibration file path
CALIBRATION_FILE = "calibration.json"

class ServoControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Tomm-I Controls")
        
        self.calibrations = self.load_calibrations()
        
        self.create_title_label()
        self.create_position_buttons()
        self.create_servo_controls()
        self.create_control_buttons()
        self.update_servo_labels()  # Ensure initial label colors are set correctly
        
        self.SetLaydownPosition()

    def create_title_label(self):
        title_label = tk.Label(self.root, text="Tomm-I Controls", font=("Arial", 24))
        title_label.pack(pady=10)

    def create_position_buttons(self):
        frame = tk.Frame(self.root)
        frame.pack(fill=tk.X)
        
        buttons = [
            ("Rest Pos.", self.SetRestPosition),
            ("Stand", self.SetStandPosition),
            ("Sit", self.SetSitPosition),
            ("Test", self.SetTestPosition),
            ("Laydown", self.SetLaydownPosition)
        ]
        
        for btn_text, command in buttons:
            btn = tk.Button(frame, text=btn_text, command=command)
            btn.pack(side=tk.LEFT, padx=5, pady=5)
    
    def create_servo_controls(self):
        self.servos = ["FL1", "FL2", "FR1", "FR2", "BL1", "BL2", "BR1", "BR2"]
        self.servo_vars = {}
        self.enable_vars = {}
        self.servo_labels = {}
        self.buttons = {}
        
        frame = tk.Frame(self.root)
        frame.pack(fill=tk.BOTH, expand=True)
        
        for idx, servo in enumerate(self.servos):
            servo_frame = tk.Frame(frame, borderwidth=2, relief=tk.GROOVE)
            servo_frame.pack(fill=tk.X, padx=5, pady=5)
            
            enable_var = tk.BooleanVar(value=False)  # Initialize checkboxes as unchecked
            self.enable_vars[servo] = enable_var
            enable_check = tk.Checkbutton(servo_frame, text=servo, variable=enable_var, command=lambda s=servo: self.toggle_servo(s))
            enable_check.grid(row=0, column=0, padx=5, pady=5)
            
            slider = ttk.Scale(servo_frame, from_=0, to=180, orient=tk.HORIZONTAL)
            slider.set(90)  # Set to default 90 degrees
            slider.grid(row=0, column=1, padx=5, pady=5)
            slider.bind("<Motion>", lambda event, s=servo: self.update_slider_value(event, s))
            slider.bind("<ButtonRelease-1>", lambda event, s=servo: self.slider_released(s))
            self.servo_vars[servo] = slider
            
            value_label = tk.Label(servo_frame, text=str(slider.get()))
            value_label.grid(row=0, column=2, padx=5, pady=5)
            self.servo_labels[servo] = value_label
            
            btn_frame = tk.Frame(servo_frame)
            btn_frame.grid(row=0, column=3, padx=5, pady=5)
            
            buttons = {
                "-10": tk.Button(btn_frame, text="-10", command=lambda s=slider, l=value_label, v=-10, srv=servo: self.change_slider_value(s, l, v, srv)),
                "-1": tk.Button(btn_frame, text="-1", command=lambda s=slider, l=value_label, v=-1, srv=servo: self.change_slider_value(s, l, v, srv)),
                "+1": tk.Button(btn_frame, text="+1", command=lambda s=slider, l=value_label, v=1, srv=servo: self.change_slider_value(s, l, v, srv)),
                "+10": tk.Button(btn_frame, text="+10", command=lambda s=slider, l=value_label, v=10, srv=servo: self.change_slider_value(s, l, v, srv))
            }
            for btn in buttons.values():
                btn.pack(side=tk.LEFT)
            self.buttons[servo] = buttons

    def create_control_buttons(self):
        frame = tk.Frame(self.root)
        frame.pack(fill=tk.X, pady=5)
        
        tk.Button(frame, text="Enable All", command=self.enable_all_servos).pack(side=tk.LEFT, padx=5)
        tk.Button(frame, text="Disable All", command=self.disable_all_servos).pack(side=tk.LEFT, padx=5)
        tk.Button(frame, text="Capture Calibration", command=self.CaptureCalibration).pack(side=tk.LEFT, padx=5)
        tk.Button(frame, text="Quit", command=self.root.quit).pack(side=tk.RIGHT, padx=5)
    
    def change_slider_value(self, slider, value_label, delta, servo):
        value = slider.get() + delta
        value = max(0, min(180, value))
        slider.set(value)
        value_label.config(text=str(int(value)))
        print(f"change_slider_value: servo={servo}, enabled={self.enable_vars[servo].get()}")
        if self.enable_vars[servo].get():
            self.SetServoAngle(servo, int(value))
    
    def update_slider_value(self, event, servo):
        value = self.servo_vars[servo].get()
        self.servo_labels[servo].config(text=str(int(value)))
    
    def slider_released(self, servo):
        value = int(self.servo_vars[servo].get())
        if self.enable_vars[servo].get():
            self.SetServoAngle(servo, value)
    
    def enable_all_servos(self):
        for servo, var in self.enable_vars.items():
            var.set(True)
            value = int(self.servo_vars[servo].get())
            self.SetServoAngle(servo, value)
        self.update_servo_labels()
    
    def disable_all_servos(self):
        for servo, var in self.enable_vars.items():
            var.set(False)
            self.DisableServo(servo)
        self.update_servo_labels()
    
    def toggle_servo(self, servo):
        if self.enable_vars[servo].get():
            value = int(self.servo_vars[servo].get())
            self.SetServoAngle(servo, value)
        else:
            self.DisableServo(servo)
        self.update_servo_labels()
    
    def update_servo_labels(self):
        for servo, var in self.enable_vars.items():
            color = "black" if var.get() else "grey"
            
            self.servo_labels[servo].config(fg=color)
            servo_frame = self.servo_vars[servo].master
            
            for widget in servo_frame.winfo_children():
                widget_type = widget.winfo_class()
                if widget_type == "Label":
                    widget.config(fg=color)
                elif widget_type == "Frame":
                    for btn in self.buttons[servo].values():
                        btn.config(fg=color)

    def SetServoAngle(self, servo, angle):
        self.servo_vars[servo].set(angle)   # To update GUI if motors disabled
        self.servo_labels[servo].config(text=f'{angle}') # To update GUI if motors disabled
        if self.enable_vars[servo].get():
            print(f"SetServoAngle called with servo: {servo}, angle: {angle}")
            calibration_offset = self.calibrations.get(servo, 0)
            actual_angle = angle + calibration_offset
            actual_angle = max(0, min(180, actual_angle))
            pca.servo[servo_map[servo]].angle = actual_angle
    
    def GetServoAngle(self, servo):
        return self.servo_vars[servo].get()
    
    def DisableServo(self, servo):
        print(f"DisableServo called with servo: {servo}")
        pca.servo[servo_map[servo]].fraction = None
    
    def CaptureCalibration(self):
        print("CaptureCalibration called")
        self.calibrations = {servo: self.calibrations.get(servo, 0) + int(self.servo_vars[servo].get()) - 90 for servo in self.servos}
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(self.calibrations, f)
        print(f"Calibration saved: {self.calibrations}")
    
    def load_calibrations(self):
        if os.path.exists(CALIBRATION_FILE):
            with open(CALIBRATION_FILE, 'r') as f:
                return json.load(f)
        return {}

    def SetServoControlAngle(self, servo, angle):
        slider = self.servo_vars[servo]
        value_label = self.servo_labels[servo]
        slider.set(angle)
        value_label.config(text=str(int(angle)))
        if self.enable_vars[servo].get():
            self.SetServoAngle(servo, int(angle))
    
    def MoveToPosition(self, target_angle):
        # target_angle is dictionary with keys being servo names and values the angles to move to
        print(f"MoveToPosition called: {target_angle}")
        
        # Get current positions
        last_set_angle = {}
        for servo in servo_map.keys():
            last_set_angle[servo] = self.GetServoAngle(servo)
        
        # Loop so the stand is implemented over many small steps
        max_angle_delta = 1
        min_angle_delta = -1
        sleep_seconds = 0.010
        for i in range(100):
            Nchanged = 0
            for servo in target_angle.keys():
                curr = last_set_angle[servo]
                if curr == target_angle[servo] : continue
                delta = target_angle[servo] - curr
                delta = min(max_angle_delta, max(min_angle_delta, delta))
                last_set_angle[servo] = curr + delta
                self.SetServoAngle(servo, last_set_angle[servo])    # Actually update motors (if enabled)
                Nchanged += 1
                
            if Nchanged == 0: break
            # self.root.update_idletasks()
            time.sleep( sleep_seconds)
            
    def SetRestPosition(self):
        print("SetRestPosition called")
        end_angle = {}
        for servo in self.servos: end_angle[servo] = 90
        self.MoveToPosition(end_angle)

    def SetStandPosition(self):
        print("SetStandPosition called")
        
        # Laydown first since standing from even the rest position can
        # cause gears to slip
        self.SetLaydownPosition()
        
        # Target positions
        end_angle = {}
        end_angle['FL1'] = 130
        end_angle['FR1'] = 50
        end_angle['BL1'] = 50
        end_angle['BR1'] = 130
   
        end_angle['FL2'] = 50
        end_angle['FR2'] = 130
        end_angle['BL2'] = 130
        end_angle['BR2'] = 50
        
        self.MoveToPosition(end_angle)
    
    def SetSitPosition(self):
        print("SetSitPosition called")
        self.SetLaydownPosition()
        # Target positions
        end_angle = {}
        end_angle['FL1'] = 100
        end_angle['FR1'] = 80
        # end_angle['BL1'] = 50
        # end_angle['BR1'] = 130
   
        end_angle['FL2'] = 0
        end_angle['FR2'] = 180
        end_angle['BL2'] = 70
        end_angle['BR2'] = 110
        self.MoveToPosition(end_angle)

    def SetTestPosition(self):
        print("SetTestPosition called")

        # Stand positions
        stand_pos = {}
        stand_pos['FL1'] = 130
        stand_pos['FR1'] = 50
        stand_pos['BL1'] = 50
        stand_pos['BR1'] = 130
        stand_pos['FL2'] = 50
        stand_pos['FR2'] = 130
        stand_pos['BL2'] = 130
        stand_pos['BR2'] = 50
        
        # Front left, Back right up
        step_pos1 = {}
        # step_pos1['FL1'] = 150
        # step_pos1['BR1'] = 150
        step_pos1['FL2'] = 90
        step_pos1['BR2'] = 90
        
        # Front right, Back left up
        step_pos2 = {}
        # step_pos2['FR1'] = 30
        # step_pos2['BL1'] = 30
        step_pos2['FR2'] = 90
        step_pos2['BL2'] = 90

        self.SetStandPosition()
        
        for i in range(4):
            self.MoveToPosition(stand_pos)
            time.sleep(0.500)
            self.MoveToPosition(step_pos1)
            self.MoveToPosition(stand_pos)
            time.sleep(0.500)
            self.MoveToPosition(step_pos2)
            
        
        self.MoveToPosition(stand_pos)        
   
    
    def SetLaydownPosition(self):
        print("SetLaydownPosition called")
        # Target positions
        end_angle = {}
        end_angle['FL1'] = 150
        end_angle['FR1'] = 30
        end_angle['BL1'] = 30
        end_angle['BR1'] = 150
   
        end_angle['FL2'] = 150
        end_angle['FR2'] = 30
        end_angle['BL2'] = 30
        end_angle['BR2'] = 150
        
        self.MoveToPosition(end_angle)
if __name__ == "__main__":
    root = tk.Tk()
    app = ServoControlApp(root)
    root.mainloop()
