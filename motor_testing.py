import os
import time
import can
import struct

def setup_can(bitrate):
    print(f"  > Switching CAN to {bitrate}...")
    os.system('sudo ip link set can0 down 2>/dev/null')
    os.system(f'sudo ip link set can0 type can bitrate {bitrate}')
    os.system('sudo ip link set can0 up')
    time.sleep(1) # Let it settle
    return can.interface.Bus(channel='can0', interface='socketcan')

def try_vesc_ping(bus, motor_id):
    # VESC "Get Firmware" command (ID 0)
    # Frame: [Command:0][ID]
    arbitration_id = (0 << 8) | motor_id
    msg = can.Message(arbitration_id=arbitration_id, data=[], is_extended_id=True)
    bus.send(msg)
    
    # Wait for reply
    listener = bus.recv(timeout=0.2)
    if listener and (listener.arbitration_id & 0xFF) == motor_id:
        return True
    return False

def try_mit_ping(bus, motor_id):
    # MIT "Enable" command (0xFC)
    # Cubmars usually replies with its ID
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    bus.send(msg)
    
    listener = bus.recv(timeout=0.2)
    if listener and listener.arbitration_id == 0x00: # Master ID reply
        return True
    return False

print("--- STARTING MOTOR SCANNER ---")

# COMBINATION 1: VESC @ 500k
print("\n[TEST 1] VESC Protocol @ 500k")
bus = setup_can(500000)
for mid in [1, 104]:
    print(f"  Pinging ID {mid}...", end="")
    if try_vesc_ping(bus, mid):
        print(" FOUND! (VESC Firmware)")
        print(f"*** SUCCESS: Use ID {mid} and Bitrate 500000 ***")
        exit()
    print(" No reply.")

# COMBINATION 2: MIT Protocol @ 1M (Cubmars Default)
print("\n[TEST 2] MIT Protocol @ 1M")
bus = setup_can(1000000)
for mid in [1, 104]:
    print(f"  Pinging ID {mid}...", end="")
    if try_mit_ping(bus, mid):
        print(" FOUND! (MIT Firmware)")
        print(f"*** SUCCESS: Use ID {mid} and Bitrate 1000000 ***")
        exit()
    print(" No reply.")

print("\n[FAIL] No motors found.")
print("Troubleshooting:")
print("1. Are CAN High/Low swapped? (Try swapping wires)")
print("2. Is the motor power on? (Green light?)")
print("3. Are phase wires shorted? (Unplug motor power to check stiffness)")