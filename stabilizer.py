import time
import math
import socket
import json
import board
import busio
import can
import os
import struct
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

# --- CONFIGURATION ---
LAPTOP_IP = "192.168.55.100"    # Your Laptop's IP
UDP_PORT = 5005
LIVE_MOTOR = True               # Enabled
SAFETY_LIMIT = 45.0             # Kill switch angle
MOTOR_ID = 1                    # <--- CONFIRMED ID 1

# --- VESC CONSTANTS ---
CAN_PACKET_SET_POS = 16         # VESC Position Command

# --- HARDWARE SETUP ---
print("Initializing Hardware (VESC Mode @ 1M)...")

# 1. Setup CAN Bus (Forcing 1M to match Laptop success)
# We run the 'down' command first to prevent "Device Busy" errors
os.system('sudo ip link set can0 down 2>/dev/null')
os.system('sudo ip link set can0 type can bitrate 1000000')
os.system('sudo ip link set can0 up')
time.sleep(1) # Wait for it to settle

try:
    bus = can.interface.Bus(channel='can0', interface='socketcan')
except OSError:
    print(" [ERR] CAN Bus failed to open. Did you run the sudo commands?")
    exit()

# 2. Setup IMU
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    print("  [OK] IMU Connected")
except Exception as e:
    print(f"  [ERR] IMU Failed: {e}")
    exit()

# 3. Network
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- HELPER FUNCTIONS ---
def get_imu_roll():
    try:
        quat = bno.quaternion
        if not quat: return None
        # Quaternion to Euler (Roll)
        sinr_cosp = 2 * (quat[3] * quat[0] + quat[1] * quat[2])
        cosr_cosp = 1 - 2 * (quat[0] * quat[0] + quat[1] * quat[1])
        roll = math.atan2(sinr_cosp, cosr_cosp)
        return math.degrees(roll)
    except OSError:
        return None

def send_vesc_position(position_deg):
    """Sends VESC Position Command (ID 16)."""
    arbitration_id = (CAN_PACKET_SET_POS << 8) | MOTOR_ID
    
    # VESC Scaling: Degrees * 1,000,000
    value = int(position_deg * 1000000.0)
    
    data = struct.pack(">i", value)
    msg = can.Message(
        arbitration_id=arbitration_id, 
        data=data, 
        is_extended_id=True
    )
    bus.send(msg)

def stop_motor():
    """Sends 0 Current to relax motor."""
    arbitration_id = (1 << 8) | MOTOR_ID # ID 1 = Set Current
    data = struct.pack(">i", 0)
    msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
    bus.send(msg)

# --- SAFETY STARTUP ---
print("\n--- SAFETY CHECK ---")
while True:
    roll = get_imu_roll()
    if roll is None: continue
    
    print(f"Current Angle: {roll:5.1f}°", end='\r')
    
    if abs(roll) < 5.0:
        print(f"\n[OK] Level ({roll:.1f}°). Press ENTER to ARM MOTOR.")
        input()
        break
    else:
        time.sleep(0.1)

# --- MAIN LOOP ---
print("--- ROBOT ARMED ---")

try:
    while True:
        roll = get_imu_roll()
        if roll is None: continue

        # 1. Kill Switch
        if abs(roll) > SAFETY_LIMIT:
            print(f" [KILL] TIPPED OVER! ({roll:.1f}°)")
            if LIVE_MOTOR: stop_motor()
            break

        # 2. Control Logic
        motor_target = -1.0 * roll

        # 3. Move Motor
        if LIVE_MOTOR:
            send_vesc_position(motor_target)

        # 4. Telemetry
        packet = {"roll": round(roll, 2), "motor": round(motor_target, 2)}
        sock.sendto(json.dumps(packet).encode(), (LAPTOP_IP, UDP_PORT))

        time.sleep(0.01) # 100Hz

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    if LIVE_MOTOR: stop_motor()
    print("Clean Exit.")