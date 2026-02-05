import can
import time
import struct
import math
import matplotlib.pyplot as plt
from collections import deque

# --- CONFIGURATION ---
MOTOR_PITCH_ID = 104
MOTOR_ROLL_ID = 1
INTERFACE = 'can0'

class DualPlotter:
    def __init__(self):
        self.bus = can.interface.Bus(channel=INTERFACE, bustype='socketcan')
        
        # Data Buffers (Last 100 points)
        self.maxlen = 100
        self.times = deque(maxlen=self.maxlen)
        
        # Pitch Data
        self.pitch_wave_data = deque(maxlen=self.maxlen)
        self.pitch_motor_data = deque(maxlen=self.maxlen)
        
        # Roll Data
        self.roll_wave_data = deque(maxlen=self.maxlen)
        self.roll_motor_data = deque(maxlen=self.maxlen)
        
        # Setup Plot
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Plot 1: Pitch
        self.ax1.set_title(f"Pitch (ID {MOTOR_PITCH_ID}) - Should be MIRROR IMAGES")
        self.l_p_wave, = self.ax1.plot([], [], 'r--', label='Wave (Disturbance)')
        self.l_p_motor, = self.ax1.plot([], [], 'b-', label='Motor (Reaction)')
        self.ax1.set_ylim(-30, 30)
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True)
        
        # Plot 2: Roll
        self.ax2.set_title(f"Roll (ID {MOTOR_ROLL_ID}) - Should be MIRROR IMAGES")
        self.l_r_wave, = self.ax2.plot([], [], 'r--', label='Wave (Disturbance)')
        self.l_r_motor, = self.ax2.plot([], [], 'b-', label='Motor (Reaction)')
        self.ax2.set_ylim(-20, 20)
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True)
        
    def send_vesc(self, motor_id, angle):
        # VESC Set Position Command
        cmd_id = (4 << 8) | motor_id
        pos_int = int(angle * 1000000.0)
        msg = can.Message(
            arbitration_id=cmd_id, 
            data=struct.pack('>i', pos_int), 
            is_extended_id=True
        )
        self.bus.send(msg)

    def run(self):
        print("Starting Dual Dashboard...")
        print("Blue Line should be the OPPOSITE of Red Line.")
        start_time = time.time()
        
        try:
            while True:
                t = time.time() - start_time
                
                # --- MATH (The Brain) ---
                # 1. Pitch: Slow Ocean Swell (+/- 20 deg)
                p_wave = 20.0 * math.sin(1.0 * t)
                p_cmd = -p_wave # Invert to cancel
                
                # 2. Roll: Faster Chop (+/- 10 deg)
                r_wave = 10.0 * math.sin(3.0 * t)
                r_cmd = -r_wave # Invert to cancel
                
                # --- ACTUATION (The Hands) ---
                self.send_vesc(MOTOR_PITCH_ID, p_cmd)
                self.send_vesc(MOTOR_ROLL_ID, r_cmd)
                
                # --- VISUALIZATION (The Eyes) ---
                self.times.append(t)
                self.pitch_wave_data.append(p_wave)
                self.pitch_motor_data.append(p_cmd)
                self.roll_wave_data.append(r_wave)
                self.roll_motor_data.append(r_cmd)
                
                # Update Plot (Every 5th frame to prevent lag)
                if int(t * 100) % 5 == 0:
                    x_vals = range(len(self.times))
                    
                    # Update Pitch
                    self.l_p_wave.set_data(x_vals, self.pitch_wave_data)
                    self.l_p_motor.set_data(x_vals, self.pitch_motor_data)
                    self.ax1.set_xlim(0, len(self.times))
                    
                    # Update Roll
                    self.l_r_wave.set_data(x_vals, self.roll_wave_data)
                    self.l_r_motor.set_data(x_vals, self.roll_motor_data)
                    self.ax2.set_xlim(0, len(self.times))
                    
                    self.fig.canvas.flush_events()
                
                time.sleep(0.01) # 100Hz Loop

        except KeyboardInterrupt:
            print("\nStopping...")

if __name__ == "__main__":
    plotter = DualPlotter()
    plotter.run()
