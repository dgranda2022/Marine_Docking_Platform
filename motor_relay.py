import socket
import json
import can
import struct
import matplotlib.pyplot as plt
from collections import deque
import time

# --- CONFIGURATION ---
UDP_IP = "0.0.0.0"   # Listen on all interfaces
UDP_PORT = 5005
MOTOR_ID = 1         # Must match your hardware
# CORRECTION: Changed from 16 to 4 to match your working script
CAN_PACKET_SET_POS = 4 

# --- SETUP CAN ---
try:
    bus = can.interface.Bus(channel='can0', interface='socketcan')
    print("[OK] Laptop CAN Interface Connected")
except Exception as e:
    print(f"[ERR] CAN Failed: {e}")
    # We exit because without CAN, the script is useless
    exit()

# --- SETUP NETWORK ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# --- GRAPHING SETUP ---
plt.ion()
fig, ax = plt.subplots(figsize=(10, 5))
times = deque(maxlen=50)
rolls = deque(maxlen=50)
line, = ax.plot([], [], 'r-', linewidth=2, label="IMU Roll / Motor Target")
ax.set_ylim(-60, 60)
ax.set_title("Remote Balance Monitor")
ax.set_ylabel("Degrees")
ax.legend()
ax.grid(True)

def send_vesc_cmd(position):
    """Pass-through: Sends the command to the physical motor"""
    # VESC Protocol: ID in upper 8 bits, Motor ID in lower 8 bits
    arbitration_id = (CAN_PACKET_SET_POS << 8) | MOTOR_ID
    
    # Scale: VESC usually expects 1000000.0 for position
    value = int(position * 1000000.0)
    data = struct.pack(">i", value)
    
    msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
    bus.send(msg)

print(f"Listening on {UDP_PORT}...")
print("Ready for Jetson commands!")

packet_count = 0

try:
    while True:
        try:
            # 1. Listen for Jetson (Non-blocking)
            data, addr = sock.recvfrom(1024)
            packet = json.loads(data.decode())
            
            # 2. Extract Data
            roll = packet.get('roll', 0)
            target = packet.get('motor', 0)
            
            # 3. DRIVE THE MOTOR (The Relay)
            # We do this IMMEDIATELY after receiving for lowest latency
            send_vesc_cmd(target)
            
            # 4. Update Graph (Decimated)
            # Only draw every 10th packet to prevent lag
            packet_count += 1
            if packet_count % 10 == 0:
                rolls.append(roll)
                times.append(len(rolls)) # Just a dummy x-axis
                
                line.set_data(range(len(rolls)), rolls)
                ax.set_xlim(0, len(rolls))
                
                # Use a tiny pause just to trigger the GUI update
                plt.pause(0.001)
            
        except BlockingIOError:
            # No data waiting, just loop
            pass
        except Exception as e:
            print(f"Error: {e}")

except KeyboardInterrupt:
    print("\nStopping...")
    # Safety: Send 0 position (or current 0) on exit
    stop_id = (1 << 8) | MOTOR_ID # Set Current = 1
    bus.send(can.Message(arbitration_id=stop_id, data=struct.pack('>i', 0), is_extended_id=True))
    print("Motor released.")