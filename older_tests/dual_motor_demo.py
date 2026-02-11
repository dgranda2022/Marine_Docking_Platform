import can
import time
import struct
import math

# --- CONFIGURATION ---
MOTOR_PITCH_ID = 104   # Your First Motor
MOTOR_ROLL_ID = 1      # Your Second Motor (CHANGE THIS if needed)
INTERFACE = 'can0'

def send_vesc(bus, motor_id, angle):
    # Command: Set Position (ID 4)
    cmd_id = (4 << 8) | motor_id
    pos_int = int(angle * 1000000.0)
    msg = can.Message(
        arbitration_id=cmd_id, 
        data=struct.pack('>i', pos_int), 
        is_extended_id=True
    )
    bus.send(msg)

def main():
    bus = can.interface.Bus(channel=INTERFACE, bustype='socketcan')
    print(f"Dual Motor Demo: Pitch({MOTOR_PITCH_ID}) vs Roll({MOTOR_ROLL_ID})")
    
    start_time = time.time()
    
    try:
        while True:
            t = time.time() - start_time
            
            # 1. PITCH: Fights a slow Ocean Swell (+/- 20 deg)
            pitch_wave = 20.0 * math.sin(1.0 * t)
            pitch_cmd = -pitch_wave  # Kp = 1.0 (Perfect Cancel)
            
            # 2. ROLL: Fights a chopped wave (+/- 10 deg)
            roll_wave = 10.0 * math.cos(3.0 * t)
            roll_cmd = -roll_wave    # Kp = 1.0
            
            # 3. Send Commands
            send_vesc(bus, MOTOR_PITCH_ID, pitch_cmd)
            send_vesc(bus, MOTOR_ROLL_ID,  roll_cmd)
            
            # Print Status
            print(f"Pitch: {pitch_cmd:5.1f}° | Roll: {roll_cmd:5.1f}°", end='\r')
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")

if __name__ == "__main__":
    main()

