import can
import struct
import time

# --- CONFIGURATION ---
MOTOR_ID = 1          # PROVEN by your logs
INTERFACE = 'can0'
RECV_SCALE = 0.1      # PROVEN by your logs

def send_current(bus, current_ma):
    # Command 1 = Current Control
    arb_id = (1 << 8) | MOTOR_ID
    data = struct.pack('>i', int(current_ma))
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
    try:
        bus.send(msg)
    except can.CanError:
        pass

print(f"--- TORQUE TEST (ID {MOTOR_ID}) ---")
print("1. Increase buffer: sudo ip link set can0 txqueuelen 1000 up")
print("2. Motor will spin for 3 seconds.")

try:
    bus = can.interface.Bus(channel=INTERFACE, bustype='socketcan', bitrate=1000000)
    
    # WAKE UP
    for _ in range(10):
        bus.send(can.Message(arbitration_id=(0 << 8) | MOTOR_ID, data=b'\0'*4, is_extended_id=True))
        time.sleep(0.01)

    start_time = time.time()
    
    # SPIN PHASE
    while time.time() - start_time < 3.0:
        # Send 2 Amps (Stronger than before to be visible)
        send_current(bus, 2000) 
        
        # Read Feedback & Print
        while True:
            msg = bus.recv(timeout=0)
            if not msg: break
            
            # ID 41 (Status) from Motor 1
            if (msg.arbitration_id >> 8) == 41 and (msg.arbitration_id & 0xFF) == MOTOR_ID:
                raw_pos = struct.unpack_from('>h', msg.data, 0)[0]
                current_ma = struct.unpack_from('>h', msg.data, 4)[0]
                
                print(f"Position: {raw_pos * RECV_SCALE:6.1f}° | Current: {current_ma/100.0:4.1f}A")
        
        time.sleep(0.01)

except KeyboardInterrupt:
    pass
finally:
    print("Stopping...")
    send_current(bus, 0)