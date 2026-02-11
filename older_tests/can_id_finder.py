import can
import struct
import time
import math
import csv
import os
import matplotlib.pyplot as plt
from collections import deque

# --- CONFIGURATION ---
MOTOR_ID = 1
INTERFACE = 'can0'
POKE_ID = 21   
LOG_PREFIX = "capstone_final"

# --- SETUP CAN ---
try:
    bus = can.interface.Bus(channel=INTERFACE, bustype='socketcan')
    print("[OK] CAN Bus Connected")
except Exception as e:
    print(f"[ERR] CAN Error: {e}")
    exit()

# --- SETUP LOGGING ---
def get_next_filename():
    i = 1
    while True:
        fname = f"{LOG_PREFIX}_{i}.csv"
        if not os.path.exists(fname):
            return fname
        i += 1

log_filename = get_next_filename()
print(f"[OK] Logging to: {log_filename}")

log_file = open(log_filename, mode='w', newline='')
csv_writer = csv.writer(log_file)
csv_writer.writerow(['Timestamp', 'Target_Deg', 'Actual_Pos', 'Current_A', 'Voltage_V'])

# --- SETUP PLOT ---
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

times = deque(maxlen=100)
targets = deque(maxlen=100)
positions = deque(maxlen=100)
currents = deque(maxlen=100)

line_target, = ax1.plot([], [], 'r--', linewidth=2, label='Target (Deg)')
line_pos, = ax1.plot([], [], 'b-', linewidth=2, label='Actual (Deg)')
line_current, = ax2.plot([], [], 'g-', linewidth=2, label='Current (A)')

ax1.set_title(f"Active Stabilization - {log_filename}")
ax1.set_ylabel("Angle (Degrees)")
ax1.legend()
ax1.grid(True)
ax1.set_ylim(-30, 30)

ax2.set_title("Motor Current")
ax2.set_ylabel("Current (Amps)")
ax2.grid(True)

# --- FUNCTIONS ---
def get_signed_int16(data, offset):
    return struct.unpack_from('>h', data, offset)[0]

def send_motor_command(pos_deg):
    cmd_id = (4 << 8) | MOTOR_ID
    val = int(pos_deg * 1000000.0)
    data = struct.pack('>i', val)
    try:
        bus.send(can.Message(arbitration_id=cmd_id, data=data, is_extended_id=True))
    except can.CanError:
        pass

def send_poke():
    cmd_id = (POKE_ID << 8) | MOTOR_ID
    try:
        bus.send(can.Message(arbitration_id=cmd_id, data=b'', is_extended_id=True))
    except can.CanError:
        pass

# --- MAIN LOOP ---
print("--- RUNNING DASHBOARD ---")
start_time = time.time()
last_poke = 0

# State Variables
current_pos = 0.0
current_amp = 0.0
current_volts = 0.0

try:
    while True:
        t = time.time() - start_time
        
        # 1. MOTION (Ocean Swell 0.1Hz)
        target = 20.0 * math.sin(2.0 * math.pi * 0.1 * t)
        send_motor_command(target)
        
        # 2. POKE (Keepalive)
        if time.time() - last_poke > 0.1:
            send_poke()
            last_poke = time.time()
            
        # 3. READ & DECODE (CORRECTED MAP)
        while True:
            msg = bus.recv(timeout=0)
            if not msg: break
            
            cmd = msg.arbitration_id >> 8
            
            # --- CUBEMARS ID 41 (0x29) DECODER ---
            if cmd == 41 and len(msg.data) >= 8:
                # Bytes 0-1: Position (Offset 0)
                # Bytes 2-3: Voltage (Offset 2)
                # Bytes 4-5: Current (Offset 4)
                # Bytes 6-7: Temp (Offset 6)
                
                raw_p = get_signed_int16(msg.data, 0)
                raw_v = get_signed_int16(msg.data, 2)
                raw_c = get_signed_int16(msg.data, 4)
                
                # Corrected Scaling
                current_pos = raw_p / 100.0
                current_volts = raw_v / 100.0
                current_amp = raw_c / 100.0

        # 4. LOGGING
        csv_writer.writerow([f"{t:.4f}", f"{target:.4f}", f"{current_pos:.4f}", f"{current_amp:.4f}", f"{current_volts:.4f}"])

        # 5. VISUALS (10Hz)
        if int(t * 100) % 10 == 0:
            times.append(t)
            targets.append(target)
            positions.append(current_pos)
            currents.append(current_amp)
            
            line_target.set_data(times, targets)
            line_pos.set_data(times, positions)
            line_current.set_data(times, currents)
            
            ax1.set_xlim(max(0, t-10), t)
            ax2.set_xlim(max(0, t-10), t)
            
            if len(currents) > 0:
                y_min = min(currents) - 0.5
                y_max = max(currents) + 0.5
                ax2.set_ylim(y_min, y_max)

            plt.pause(0.001)

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopping...")
    log_file.close()
    stop_id = (1 << 8) | MOTOR_ID
    bus.send(can.Message(arbitration_id=stop_id, data=struct.pack('>i', 0), is_extended_id=True))