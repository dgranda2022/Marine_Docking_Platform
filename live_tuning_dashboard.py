import can
import time
import struct
import math
import matplotlib.pyplot as plt
from collections import deque

# --- CONFIGURATION ---
MOTOR_ID = 1
INTERFACE = 'can0'

# INITIAL GAINS (We will change these with Arrow Keys!)
kp = 1.0
kd = 0.1

# VESC CONSTANTS
CAN_PACKET_SET_POS = 4

class RealTimePlotter:
    def __init__(self):
        self.bus = can.interface.Bus(channel=INTERFACE, bustype='socketcan')
        
        # Data Buffers (Last 100 points)
        self.maxlen = 100
        self.times = deque(maxlen=self.maxlen)
        self.wave_data = deque(maxlen=self.maxlen)
        self.motor_data = deque(maxlen=self.maxlen)
        
        # Setup Plot
        plt.ion() # Interactive Mode ON
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.line_wave, = self.ax.plot([], [], 'r--', label='Virtual Wave (Disturbance)', linewidth=2)
        self.line_motor, = self.ax.plot([], [], 'b-', label='Motor Correction (Reaction)', linewidth=2)
        
        self.ax.set_ylim(-50, 50)
        self.ax.set_xlim(0, 5)
        self.ax.set_title("Real-Time PID Tuning (Use Arrow Keys!)")
        self.ax.set_xlabel("Time Window (s)")
        self.ax.set_ylabel("Angle (Degrees)")
        self.ax.grid(True)
        self.ax.legend(loc='upper right')
        
        # Text for Gains
        self.text_gains = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes, 
                                       bbox=dict(facecolor='white', alpha=0.8))

        # Keyboard Listeners
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
    def on_key(self, event):
        global kp, kd
        step = 0.1
        if event.key == 'up': kp += step
        if event.key == 'down': kp = max(0, kp - step)
        if event.key == 'right': kd += step
        if event.key == 'left': kd = max(0, kd - step)

    def send_vesc(self, position_deg):
        # Construct VESC Extended ID for Position Control
        cmd_id = (CAN_PACKET_SET_POS << 8) | MOTOR_ID
        pos_int = int(position_deg * 1000000.0)
        data = struct.pack('>i', pos_int)
        msg = can.Message(arbitration_id=cmd_id, data=data, is_extended_id=True)
        self.bus.send(msg)

    def run(self):
        print("Starting Dashboard...")
        print("PRESS ARROW KEYS to Tune Gains Live!")
        start_time = time.time()
        last_time = start_time
        last_error = 0
        
        try:
            while True:
                t = time.time() - start_time
                dt = time.time() - last_time
                if dt < 0.001: continue # Avoid divide by zero

                # 1. GENERATE VIRTUAL WAVE (The "Problem")
                # A slow sine wave + some fast jitters
                wave_angle = (25.0 * math.sin(2.0 * t)) + (2.0 * math.sin(10.0 * t))
                
                # 2. PID LOGIC (The "Brain")
                # Goal: Platform = 0. So Motor must move OPPOSITE to Wave.
                error = 0.0 - wave_angle
                d_error = (error - last_error) / dt
                
                # Calculate Response
                p_term = kp * error
                d_term = kd * d_error
                motor_command = p_term + d_term
                
                # 3. MOVE MOTOR (The "Action")
                self.send_vesc(motor_command)
                
                # 4. UPDATE PLOT (Visuals)
                self.times.append(t)
                self.wave_data.append(wave_angle)
                self.motor_data.append(motor_command)
                
                # Only redraw every 5th frame to keep motor fast (20Hz UI update)
                if int(t * 100) % 5 == 0:
                    self.line_wave.set_data(range(len(self.times)), self.wave_data)
                    self.line_motor.set_data(range(len(self.times)), self.motor_data)
                    self.ax.set_xlim(0, len(self.times))
                    
                    # Update Text
                    status = f"Kp: {kp:.1f} (Stiffness)\nKd: {kd:.1f} (Damping)"
                    self.text_gains.set_text(status)
                    
                    self.fig.canvas.flush_events()
                
                last_error = error
                last_time = time.time()
                time.sleep(0.01) # 100Hz loop

        except KeyboardInterrupt:
            print("\nStopping...")
            # Relax motor
            stop_id = (1 << 8) | MOTOR_ID # Set Current 0
            self.bus.send(can.Message(arbitration_id=stop_id, data=struct.pack('>i', 0), is_extended_id=True))

if __name__ == "__main__":
    plotter = RealTimePlotter()
    plotter.run()
