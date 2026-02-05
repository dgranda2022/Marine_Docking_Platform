import time
import math
import socket
import json
import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

# --- CONFIGURATION ---
LAPTOP_IP = "192.168.55.100"  # Target IP (Laptop)
UDP_PORT = 5005
SAFETY_LIMIT = 45.0

# --- SETUP IMU ---
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    print("[OK] IMU Connected")
except Exception as e:
    print(f"[ERR] IMU Failed: {e}")
    exit()

# --- NETWORK ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def get_imu_roll():
    try:
        quat = bno.quaternion
        if not quat: return None
        sinr_cosp = 2 * (quat[3] * quat[0] + quat[1] * quat[2])
        cosr_cosp = 1 - 2 * (quat[0] * quat[0] + quat[1] * quat[1])
        roll = math.atan2(sinr_cosp, cosr_cosp)
        return math.degrees(roll)
    except:
        return None

print("--- JETSON BRAIN ACTIVE ---")
print("Move the sensor to drive the Laptop's motor.")

while True:
    roll = get_imu_roll()
    if roll is None: continue

    # 1. Safety Logic
    if abs(roll) > SAFETY_LIMIT:
        motor_target = 0 # Kill
    else:
        # 2. The Logic: Mirror the angle
        motor_target = -1.0 * roll

    # 3. Send Command to Laptop
    packet = {
        "roll": round(roll, 2),
        "motor": round(motor_target, 2)
    }
    sock.sendto(json.dumps(packet).encode(), (LAPTOP_IP, UDP_PORT))

    time.sleep(0.01) # 100Hz