
# Active Stabilization System - Integration & Setup Guide

**Project:** Mechanical Engineering Capstone II  
**Role:** Controls & Software  
**Date:** February 2026

---

## 1. System Architecture (The "Workaround")
Due to USB-to-CAN driver incompatibility on the Jetson Orin Nano, the system uses a **Network Bridge Architecture**:

* **Brain (Jetson Orin Nano):** Reads IMU (I2C) $\rightarrow$ Calculates Control Signal $\rightarrow$ Sends UDP Packet.
* **Bridge (Laptop/Ground Station):** Listens for UDP $\rightarrow$ Graphic Visualization $\rightarrow$ Writes to CAN Bus $\rightarrow$ Motor.

---

## 2. Hardware "Golden" Configuration
**WARNING:** These values are hard-coded in the scripts. Do not change without updating code.

| Component | Setting | Value | Notes |
| :--- | :--- | :--- | :--- |
| **Motor** | Hardware ID | `1` | CubeMars AK60-39 |
| **Motor** | CAN Command ID | `4` | **Critical:** ID 4 is "Set Position". ID 16 will fail. |
| **IMU** | Model | BNO085 | Adafruit Breakout |
| **IMU** | Interface | I2C | Address `0x4A` (default) or `0x4B` |
| **Network**| Jetson IP | `192.168.55.1` | USB Device Mode IP |
| **Network**| Laptop IP | `192.168.55.100` | USB Host IP |

---

## 3. Software Dependencies
If setting up a new machine, install these first.

**On the Jetson:**
```bash
sudo apt-get install i2c-tools
pip3 install adafruit-circuitpython-bno08x board busio

```

**On the Laptop:**

```bash
pip3 install python-can matplotlib

```

---

## 4. Operations Manual (Start Here)

### Step 1: Internet Sharing (Run on Laptop)

*Why:* The Jetson cannot access the internet or install packages without this. Run this every time you reboot or switch Wi-Fi networks.

1. Enable IP Forwarding

```bash
sudo sysctl -w net.ipv4.ip_forward=1
```

2. Identify your current Internet Source (Auto-detects Wifi/Ethernet)

```bash
WIFI_IF=$(ip route | grep default | awk '{print $5}' | head -n 1)
echo "Sharing Internet from Interface: $WIFI_IF"
```

3. Setup NAT (The "Masquerade")

```bash
sudo iptables -t nat -A POSTROUTING -s 192.168.55.0/24 -o $WIFI_IF -j MASQUERADE
sudo iptables -A FORWARD -s 192.168.55.0/24 -o $WIFI_IF -j ACCEPT
sudo iptables -A FORWARD -m state --state ESTABLISHED,RELATED -j ACCEPT
```

### Step 2: Establish Route (Run on Jetson)

*Why:* The Jetson knows it is connected to the laptop, but doesn't know the laptop is its gateway to the internet.

```bash
# 1. Set the Laptop (192.168.55.100) as the Gateway
sudo route add default gw 192.168.55.100

# 2. Set Google DNS (Temporary)
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf > /dev/null

# 3. Test Connection
ping -c 3 google.com

```

---

## 5. Hardware Interface Setup

### A. IMU Setup (Jetson Side)

*The Bus Hunt:* On the Jetson Orin Nano, the I2C pins (Pins 3 & 5) are not always Bus 1.

1. Run discovery:
```bash
sudo i2cdetect -y -r 1
sudo i2cdetect -y -r 7
sudo i2cdetect -y -r 8

```


2. Look for `4a` or `4b` in the grid. Update the Python script `board.SCL_1` vs `board.SCL` accordingly.

### B. Motor Setup (Laptop Side)

1. **Bring up CAN Interface:**
```bash
sudo ip link set can0 up type can bitrate 1000000

```


2. **Verify Connection:**
```bash
ip link show can0
# You should see "state UP"

```



---

## 6. Startup Sequence (The "Launch")

### 1. SSH into Jetson

Connect the USB-C cable from Laptop to Jetson (Data Port).

```bash
ssh edg5@192.168.55.1
# Password: Docking_station!

```

*(Note: Replace `<your_username>` with your actual user)*

### 2. Start the Ground Station (Laptop)

This must run first to listen for the Jetson.

```bash
python3 laptop_relay.py
# Expected Output: "Listening on 5005..."

```

### 3. Start the Brain (Jetson)

```bash
python3 jetson_brain.py
# Expected Output: "IMU Connected... Sending packets..."

```

---

## 7. Troubleshooting

* **"OSError: [Errno 121] Remote I/O error" (Jetson):**
* The IMU wiring is loose, or you are targeting the wrong I2C Bus. Re-run `i2cdetect`.


* **Motor doesn't move but Graph works:**
* Check the **Motor ID** (Is it 1?).
* Check the **CAN ID** (Is it 4?).
* Ensure the battery is on (Voltage > 40V).


* **"Network is unreachable" (Jetson):**
* You forgot to run the **Step 1** (iptables) on laptop or **Step 2** (route) on Jetson.
* Your Laptop's Wi-Fi interface name might have changed. Rerunning the auto-detect script fixes this.



---

## Appendix: Reference Scripts

### A. `motor_relay.py` (Fixed CAN ID)

```python
import socket
import json
import can
import struct
import matplotlib.pyplot as plt
from collections import deque
import time

# --- CONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
MOTOR_ID = 1
CAN_PACKET_SET_POS = 4  # FIXED: Changed from 16 to 4

# --- SETUP CAN ---
try:
    bus = can.interface.Bus(channel='can0', interface='socketcan')
    print("[OK] Laptop CAN Interface Connected")
except Exception as e:
    print(f"[ERR] CAN Failed: {e}")
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
line, = ax.plot([], [], 'r-', linewidth=2, label="IMU Roll")
ax.set_ylim(-60, 60)
ax.set_title("Remote Balance Monitor")
ax.grid(True)

def send_vesc_cmd(position):
    arbitration_id = (CAN_PACKET_SET_POS << 8) | MOTOR_ID
    value = int(position * 1000000.0)
    data = struct.pack(">i", value)
    msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
    bus.send(msg)

print(f"Listening on {UDP_PORT}...")
packet_count = 0

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            packet = json.loads(data.decode())
            
            # DRIVE MOTOR IMMEDIATELY
            target = packet.get('motor', 0)
            send_vesc_cmd(target)
            
            # UPDATE GRAPH (Decimated to avoid lag)
            packet_count += 1
            if packet_count % 10 == 0:
                roll = packet.get('roll', 0)
                rolls.append(roll)
                line.set_data(range(len(rolls)), rolls)
                ax.set_xlim(0, len(rolls))
                plt.pause(0.001)
                
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"Error: {e}")

except KeyboardInterrupt:
    # Safety Stop
    stop_id = (1 << 8) | MOTOR_ID
    bus.send(can.Message(arbitration_id=stop_id, data=struct.pack('>i', 0), is_extended_id=True))
    print("Motor released.")

```
### B. `brain.py` (This is on jetson)

```python
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
```