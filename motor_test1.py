# This is for update presentation 02/10/2026
# The goal is to prove we can command motor using python
# and plot the data being sent to the motor and its response. 

import can
import struct
import time
import math
import matplotlib.pyplot as plt
from collections import deque

# ---CONFIGURATION---
MOTOR_ID = 1
INTERFACE = 'can0'

# VESC CAN ID's (Shifted by 8bits + Motor ID)

# STATUS 1 (ID 9) contains RPM & Current

CAN_ID_STATUS_1 = (9 << 8) | MOTOR_ID

# STATUS 4, contains PID position (from 0-360)
CAN_ID_STATUS_5 = (16 << 8) | MOTOR_ID

def get_signed_int16(data, offset):
    """Helper : Unpack 2 bytes into a signed integer"""
    # '>' means Big Endian, 'h' means short (2bytes)
    return struct.unpack_from('>h', data, offset)[0]

def get_signed_int32(data, offset):
    """Helper : Unpack 4 bytes into a signed integer"""
    # 'i' means int (4bytes)
    return struct.unpack_from('>i', data, offset)[0]

'''
Everything above here is setting up the communication between the computer
and the motor. 

The motor id and interface is like a key, it tells it what the 
motor is called, and where it is. 

The CAN_ID_STATUS variables are "Filters". They tell the computer:
"When you hear a message with this specific ID, it contains the RPM/Position 
data we are looking for." They identify the "envelope" (The Label).

The get_signed_int methods are the "Decoders". They tell the code HOW to 
read the data inside that envelope. VESC packs multiple numbers into a 
single 8-byte message, and these methods slice that message up to extract 
specific values (like RPM or Current).
'''
def send_position (bus, position_deg):
    """Sends position target to the motor"""
    # ID 4 = set position
    cmd_id = (4 << 8) | MOTOR_ID

    #Scale: VESC expects degrees * 1,000,000
    val = int(position_deg*1000000.0)

    data = struct.pack('>i', val)
    msg= can.Message(arbitration_id=cmd_id, data=data, is_extended_id = True)
    bus.send(msg)
if __name__ == "__main__":
    print ("Starting Motor Diagnostics...")

    #1. setup CAN
    try:
        bus = can.interface.Bus(channel=INTERFACE, bustype='socketcan')
    except Exception as e:
        print(f"Error opening CAN: {e}")
        exit()

    #2. Setup plot
    plt.ion() #interactive mode
    fig, (ax1, ax2) = plt.subplots(2,1,figsize=(8,8), sharex=True)

    #Buffers for plotting
    times = deque(maxlen=50)
    targets = deque(maxlen=50)
    actuals = deque(maxlen=50)
    currents = deque(maxlen=50)

    #Lines
    line_target, = ax1.plot([],[], 'r--', label = 'Target (Deg)')
    line_actual, = ax1.plot([],[], 'b-', label = 'Actual (Deg)')
    line_current, = ax2.plot([],[], 'g-', label = 'Current (A)')

    ax1.set_title("Position Tracking")
    ax1.legend()
    ax1.set_ylim(-45,45) #+/- 45 degree test

    ax2.set_title("Motor Effort (Current)")
    ax2.set_ylabel("amps")
    ax2.set_ylim(-10, 10) #10A safety limit for visualization)

    start_time = time.time()
    current_val = 0
    actual_pos = 0

    try:
        while True:
            t = time.time()-start_time

            # --A. Generate a Test signal ---
            target_pos = 20.0*math.sin(2.0*math.pi*0.5*t)
            # --B. Send Command ---
            send_position(bus, target_pos)
            # ---C. Read Feedback ---
            # Drain buffer so its latest data
            msg = bus.recv(timeout=0.001)

            if msg:
                #Decode status 1 (current)
                if msg.arbitration_id == CAN_ID_STATUS_1:
                    # Bytes 4-5 are current * 10
                    raw_current = get_signed_int16(msg.data, 4)
                    current_val = raw_current /10.0

                # Decode status 4 (position)

                elif msg.arbitration_id == CAN_ID_STATUS_5:
                    # Bytes 4-5 are PID Pos * 50 (Standard VESC scaling)
                    # NOTE: This scaling varies by firmware! 
                    # If graphs look weird, we might need Status 5 instead.
                    raw_pose = get_signed_int16(msg.data, 4)
                    actual_pos = raw_pose/50.0

            #--D. UPdate plot
            # ONly update every 10th loop
            if int(t*100) %5 == 0:
                times.append(t)
                targets.append(target_pos)

                #We append last known values (for simplicity)
                #For real logger you match timestamps perfectly
                actuals.append(actual_pos)
                currents.append(current_val)

                line_target.set_data(list(times), list(targets))
                line_actual.set_data(list(times), list(actuals))
                line_current.set_data(list(times), list(currents))
                
                ax1.set_xlim(max(0, t-5), t) #scroll window
                plt.pause(0.001)
    except KeyboardInterrupt:
        print("\nStopping Test...")
        # Send 0 Current (Release Motor)
        stop_id = (1 << 8) | MOTOR_ID
        bus.send(can.Message(arbitration_id=stop_id, data=struct.pack('>i', 0), is_extended_id=True))