#!/usr/bin/env python3
"""
leg_calibrator.py

GUI tool for calibrating an 8‐servo robot leg.
Stores offsets and 2nd‐order readback fits in YAML and displays live ADC readbacks via ZMQ.
"""

#----------------- Servos (pin & channel mappings)
servo2adc_map = {
    'BL1': 'A0',
    'BL2': 'A1',
    'BR2': 'A2',
    'BR1': 'A3',

    'FR1': 'A4',
    'FR2': 'A5',
    'FL2': 'A6',
    'FL1': 'A7'
}
servo_map = {
    'BL1': 15,
    'BL2': 13,
    'BR2': 2,
    'BR1': 0,

    'FR1': 1,
    'FR2': 3,
    'FL2': 12,
    'FL1': 14
}

import os
import time
import threading
import json
import zmq
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import yaml  # PyYAML
from adafruit_servokit import ServoKit

# Constants & defaults
default_calib_path = os.path.expanduser('~/calibrations/leg_servo_calibs.yaml')
JOINT_NAMES = ['FR1', 'FR2', 'BR1', 'BR2', 'BL1', 'BL2', 'FL1', 'FL2']
# ZMQ port for ADC state updates
ZMQ_PORT = 71400

class CalibrationAbort(Exception):
    """Raised when user aborts calibration."""
    pass

class CalibrationGUI(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.master.title("Leg Servo Calibration")
        self.pack(fill='both', expand=True, padx=10, pady=10)
        self.master.protocol("WM_DELETE_WINDOW", self._quit)

        # Servo controller
        self.kit = ServoKit(channels=16)
        for ch in range(16):  # disable at start
            self.kit.servo[ch].angle = None

        # State
        self.calib_data = {}
        self.offsets = {}
        self.fit_params = {}
        self.specs = {'servo_range': 270, 'shoulder_ratio': 2, 'foot_ratio': 20/14}

        # Live ADC state
        self.arduino_state = {}
        self._Done = False

        # GUI setup
        self._build_specs_frame()
        self._load_calibration_file(default_calib_path)
        self.offsets = self.calib_data.get('offsets', {})
        self._build_offsets_frame()
        self._build_controls_frame()

        # Start ZMQ read thread
        threading.Thread(target=self._arduino_read_thread_zmq, daemon=True).start()

    def _build_specs_frame(self):
        frame = ttk.LabelFrame(self, text="Servo & Joint Specs")
        frame.pack(fill='x', pady=5)
        for row, (key, label) in enumerate([
            ('servo_range', 'Servo range (°):'),
            ('shoulder_ratio', 'Shoulder gear ratio:'),
            ('foot_ratio', 'Foot sprocket ratio:')]):
            ttk.Label(frame, text=label).grid(column=0, row=row, sticky='e')
            ent = ttk.Entry(frame, width=10)
            ent.insert(0, str(self.specs[key]))
            ent.grid(column=1, row=row, sticky='w')
            setattr(self, f'ent_{key}', ent)

    def _build_offsets_frame(self):
        frame = ttk.LabelFrame(self, text="Current Offsets & Readbacks")
        frame.pack(fill='x', pady=5)
        # headers
        ttk.Label(frame, text="Joint").grid(column=0, row=0, sticky='e')
        ttk.Label(frame, text="Offset").grid(column=1, row=0, sticky='w')
        ttk.Label(frame, text="Readback").grid(column=2, row=0, sticky='w')
        self.offset_vars = {}
        self.readback_vars = {}
        for idx, joint in enumerate(JOINT_NAMES, start=1):
            ttk.Label(frame, text=f"{joint}:").grid(column=0, row=idx, sticky='e')
            # Offset displayed in readonly Entry for border
            off_val = self.offsets.get(joint, None)
            off_var = tk.StringVar(value=str(off_val))
            ent_off = ttk.Entry(frame, textvariable=off_var, width=8, state='readonly', justify='center')
            ent_off.grid(column=1, row=idx, sticky='w', padx=(0,10))
            self.offset_vars[joint] = off_var
            # Readback displayed similarly
            rd_var = tk.StringVar(value='N/A')
            ent_rd = ttk.Entry(frame, textvariable=rd_var, width=8, state='readonly', justify='center')
            ent_rd.grid(column=2, row=idx, sticky='w')
            self.readback_vars[joint] = rd_var

    def _build_controls_frame(self):
        frame = ttk.Frame(self)
        frame.pack(fill='x', pady=10)
        self.btn_begin = ttk.Button(frame, text="Begin Calibration", command=self.begin_calibration)
        self.btn_begin.pack(side='left')
        self.btn_save = ttk.Button(frame, text="Save Calibration", command=self.save_calibration)
        self.btn_save.pack(side='left', padx=(10, 0))
        quit_frame = ttk.Frame(self)
        quit_frame.pack(fill='x', pady=5)
        self.btn_quit = ttk.Button(quit_frame, text="Quit", command=self._quit)
        self.btn_quit.pack()

    def _load_calibration_file(self, path):
        try:
            with open(path) as f:
                self.calib_data = yaml.safe_load(f) or {}
            messagebox.showinfo("Loaded", f"Calibration loaded from {path}")
        except FileNotFoundError:
            self.calib_data = {}
            messagebox.showwarning("Not found", f"No calibration file at {path}, starting fresh.")

    def save_calibration(self):
        self.specs['servo_range'] = float(self.ent_servo_range.get())
        self.specs['shoulder_ratio'] = float(self.ent_shoulder_ratio.get())
        self.specs['foot_ratio'] = float(self.ent_foot_ratio.get())
        out = {'specs': self.specs, 'offsets': self.offsets, 'readback_fits': self.fit_params}
        with open(default_calib_path, 'w') as f:
            yaml.safe_dump(out, f)
        messagebox.showinfo("Saved", f"Calibration written to {default_calib_path}")

    def begin_calibration(self):
        self.btn_begin.config(state='disabled')
        try:
            for joint in JOINT_NAMES:
                self._calibrate_joint(joint)
            if messagebox.askyesno("Readback?", "Proceed to readback calibration?"):
                for joint in JOINT_NAMES:
                    self._readback_calibration(joint)
            messagebox.showinfo("Done", "Calibration procedure complete.")
        except CalibrationAbort:
            messagebox.showwarning("Aborted", "Calibration aborted by user.")
        finally:
            self.btn_begin.config(state='normal')

    def _ask_ok_skip(self, title, message):
        dlg = tk.Toplevel(self.master)
        dlg.title(title)
        ttk.Label(dlg, text=message, wraplength=300).pack(padx=20, pady=10)
        result = {'value': 'skip'}
        def on_ok(): result.update(value='ok'); dlg.destroy()
        def on_skip(): dlg.destroy()
        def on_abort(): result.update(value='abort'); dlg.destroy()
        top = ttk.Frame(dlg); top.pack(pady=(0,10))
        ttk.Button(top, text="Skip", command=on_skip).pack(side='left', padx=5)
        ttk.Button(top, text="Abort", command=on_abort).pack(side='left', padx=5)
        bot = ttk.Frame(dlg); bot.pack(pady=(0,10))
        ttk.Button(bot, text="OK", command=on_ok).pack()
        dlg.transient(self.master); dlg.grab_set(); self.master.wait_window(dlg)
        choice = result['value']
        if choice == 'abort': raise CalibrationAbort()
        return (choice == 'ok')

    def _calibrate_joint(self, joint):
        chan = servo_map[joint]
        if not self._ask_ok_skip(joint, f"Disconnect servo for {joint}. Press OK to proceed, Skip to skip, or Abort."):
            self.offsets[joint] = None; self.offset_vars[joint].set('None'); self.kit.servo[chan].angle = None; return
        nominal = self.specs['servo_range']/2
        self.kit.servo[chan].angle = nominal
        if not self._ask_ok_skip(joint, f"Reattach servo for {joint}. Press OK to continue, Skip to skip, or Abort."):
            self.offsets[joint] = None; self.offset_vars[joint].set('None'); self.kit.servo[chan].angle = None; return
        adj = self._fine_tune_dialog(joint, nominal)
        self.offsets[joint] = adj - nominal; self.offset_vars[joint].set(str(self.offsets[joint])); self.kit.servo[chan].angle = None

    def _fine_tune_dialog(self, joint, start_angle):
        dlg = tk.Toplevel(self.master)
        dlg.title(f"Fine‐tune {joint}")
        ttk.Label(dlg, text="Fine tune the joint so it is at the 90° rest position.", wraplength=300).pack(padx=20, pady=(10,5))
        angle_var = tk.DoubleVar(dlg, value=start_angle)
        ttk.Label(dlg, textvariable=angle_var, font=('TkDefaultFont', 16)).pack(padx=10, pady=10)
        def move(delta):
            new_val = angle_var.get() + delta
            angle_var.set(new_val)
            self.kit.servo[servo_map[joint]].angle = new_val
        btns = ttk.Frame(dlg)
        btns.pack(pady=5)
        for txt, d in [('−5°', -5), ('−1°', -1), ('+1°', 1), ('+5°', 5)]:
            ttk.Button(btns, text=txt, command=lambda d=d: move(d)).pack(side='left', padx=5)
        done = ttk.Frame(dlg)
        done.pack(pady=(5,10))
        ttk.Button(done, text="Complete", command=dlg.destroy).pack()
        dlg.transient(self.master)
        dlg.grab_set()
        self.master.wait_window(dlg)
        return angle_var.get()

    def _readback_calibration(self, joint):
        """
        Perform readback calibration for a single joint:
        - Determine range (±45° for shoulder, ±90° for foot)
        - Step servo through range in 2° increments
        - At each step, collect 5 readings with 50ms delay,
          retrying oldest if RMS>3 until RMS≤3 or 10s elapsed
        - On timeouts, allow user to retry, abort joint, or abort calibration
        - Fit angle vs readback to 2nd-order polynomial and save params
        """
        chan = servo_map[joint]
        # determine nominal servo setting
        offset = self.offsets.get(joint, 0) or 0
        center = self.specs['servo_range']/2 + offset
        # choose half-range
        half_range = 45 if joint.endswith('1') else 90
        angles = []
        readings = []
        for step in range(-half_range, half_range+1, 2):
            target = center + step
            self.kit.servo[chan].angle = target
            # collect 5 stable readings
            vals = []
            start = time.time()
            # initial 5
            while len(vals) < 5 and time.time() - start < 10:
                raw = self.arduino_state.get(servo2adc_map[joint])
                if raw is not None:
                    vals.append(float(raw))
                time.sleep(0.05)
            # check stability
            def rms(xs):
                m = sum(xs)/len(xs)
                return (sum((x-m)**2 for x in xs)/len(xs))**0.5
            # retry oldest readings until stable or timeout
            while time.time() - start < 10 and rms(vals) > 3:
                vals.pop(0)
                raw = self.arduino_state.get(servo2adc_map[joint])
                if raw is not None:
                    vals.append(float(raw))
                time.sleep(0.05)
            # if still unstable/time expired
            if rms(vals) > 3:
                # prompt user
                choice = self._ask_retry_abort(f"{joint} stability", \
                    f"Readback RMS {rms(vals):.1f} >3 at step {step}°. Choose action:")
                if choice == 'retry':
                    return self._readback_calibration(joint)
                elif choice == 'abort_joint':
                    self.fit_params[joint] = None
                    return
                elif choice == 'abort_all':
                    raise CalibrationAbort()
            # record
            angles.append(target)
            readings.append(sum(vals)/len(vals))
        # fit 2nd-order polynomial
        import numpy as np
        coeffs = np.polyfit(angles, readings, 2)
        self.fit_params[joint] = coeffs.tolist()

    def _ask_retry_abort(self, title, message):
        dlg = tk.Toplevel(self.master)
        dlg.title(title)
        ttk.Label(dlg, text=message, wraplength=300).pack(padx=20, pady=10)
        result = {'value': None}
        def on_retry(): result['value'] = 'retry'; dlg.destroy()
        def on_abort_joint(): result['value'] = 'abort_joint'; dlg.destroy()
        def on_abort_all(): result['value'] = 'abort_all'; dlg.destroy()
        frm = ttk.Frame(dlg)
        frm.pack(pady=10)
        ttk.Button(frm, text="Try Again", command=on_retry).pack(side='left', padx=5)
        ttk.Button(frm, text="Abort Joint", command=on_abort_joint).pack(side='left', padx=5)
        ttk.Button(frm, text="Abort Calibration", command=on_abort_all).pack(side='left', padx=5)
        dlg.transient(self.master)
        dlg.grab_set()
        self.master.wait_window(dlg)
        return result['value']

    def update_labels(self):
        for joint, var in self.readback_vars.items():
            adc = servo2adc_map[joint]
            val = self.arduino_state.get(adc)
            var.set(str(val) if val is not None else 'N/A')

    def _arduino_read_thread_zmq(self):
        # Socket to talk to server
        context = zmq.Context()
        zmqsocket = context.socket(zmq.SUB)
        zmqsocket.connect(f"tcp://localhost:{ZMQ_PORT}")
        zmqsocket.subscribe("")
        last_time = time.time()
        while zmqsocket and not self._Done:
            try:
                data = zmqsocket.recv(flags=zmq.NOBLOCK).strip()
                self.arduino_state = json.loads(data)
                if time.time() - last_time >= 0.1:
                    if not self._Done:
                        self.update_labels()
                    last_time = time.time()
            except zmq.Again:
                time.sleep(0.01)
        print("arduino_state_read_thread_zmq stopping")

    def _quit(self):
        self._Done = True
        self.master.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    CalibrationGUI(root)
    root.mainloop()
