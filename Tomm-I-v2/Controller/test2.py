#!/usr/bin/env python3
#
# This is the program used to control the robot. 

import tkinter as tk
from tkinter import ttk
from adafruit_servokit import ServoKit


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
MIN_ANG  =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG  =[180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]
pca = ServoKit(channels=16)
for i in range(16): pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])


class ServoControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Tomm-I Controls")
        
        self.create_title_label()
        self.create_position_buttons()
        self.create_servo_controls()
        self.create_control_buttons()
        self.update_servo_labels()  # Ensure initial label colors are set correctly

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
            
            slider = ttk.Scale(servo_frame, from_=500, to=2500, orient=tk.HORIZONTAL)
            slider.set(1500)
            slider.grid(row=0, column=1, padx=5, pady=5)
            slider.bind("<Motion>", lambda event, s=servo: self.update_slider_value(event, s))
            slider.bind("<ButtonRelease-1>", lambda event, s=servo: self.slider_released(s))
            self.servo_vars[servo] = slider
            
            value_label = tk.Label(servo_frame, text="1500")
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
        tk.Button(frame, text="Quit", command=self.root.quit).pack(side=tk.RIGHT, padx=5)
    
    def change_slider_value(self, slider, value_label, delta, servo):
        value = slider.get() + delta
        value = max(500, min(2500, value))
        slider.set(value)
        value_label.config(text=str(int(value)))
        print(f"change_slider_value: servo={servo}, enabled={self.enable_vars[servo].get()}")
        if self.enable_vars[servo].get():
            self.SetServoPWM(servo, int(value))
    
    def update_slider_value(self, event, servo):
        value = self.servo_vars[servo].get()
        self.servo_labels[servo].config(text=str(int(value)))
    
    def slider_released(self, servo):
        value = int(self.servo_vars[servo].get())
        if self.enable_vars[servo].get():
            self.SetServoPWM(servo, value)
    
    def enable_all_servos(self):
        for servo, var in self.enable_vars.items():
            var.set(True)
            value = int(self.servo_vars[servo].get())
            self.SetServoPWM(servo, value)
        self.update_servo_labels()
    
    def disable_all_servos(self):
        for servo, var in self.enable_vars.items():
            var.set(False)
            self.DisableServo(servo)
        self.update_servo_labels()
    
    def toggle_servo(self, servo):
        if self.enable_vars[servo].get():
            value = int(self.servo_vars[servo].get())
            self.SetServoPWM(servo, value)
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

    def SetServoPWM(self, servo, val):
        print(f"SetServoPWM called with servo: {servo}, value: {val}")
        angle = (val - 500) * 180.0 / 2000.0
        if angle < 0.0: angle = 0.0
        if angle > 180.0: angle = 180.0
        pca.servo[servo_map[servo]].angle = angle
    
    def DisableServo(self, servo):
        print(f"DisableServo called with servo: {servo}")
        pca.servo[servo_map[servo]].fraction = None
    
    def SetServoAngle(self, servo, angle):
        pwm_value = int(500 + (angle / 180.0) * 2000)
        self.servo_vars[servo].set(pwm_value)
        self.servo_labels[servo].config(text=str(pwm_value))
        if self.enable_vars[servo].get():
            self.SetServoPWM(servo, pwm_value)
    
    def SetRestPosition(self):
        print("SetRestPosition called")
        # Add your code to set the rest position for each servo
        for servo in self.servos:
            self.SetServoAngle(servo, 90)  # Example angle for rest position
    
    def SetStandPosition(self):
        print("SetStandPosition called")
        # Add your code to set the stand position for each servo
        for servo in self.servos:
            self.SetServoAngle(servo, 180)  # Example angle for stand position
    
    def SetSitPosition(self):
        print("SetSitPosition called")
        # Add your code to set the sit position for each servo
        for servo in self.servos:
            self.SetServoAngle(servo, 45)  # Example angle for sit position
    
    def SetLaydownPosition(self):
        print("SetLaydownPosition called")
        # Add your code to set the laydown position for each servo
        for servo in self.servos:
            self.SetServoAngle(servo, 0)  # Example angle for laydown position

if __name__ == "__main__":
    root = tk.Tk()
    app = ServoControlApp(root)
    root.mainloop()
