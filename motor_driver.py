import can
import struct
import time
import csv
import datetime

class MotorDriver:
    def __init__(self, motor_id=104, interface='can0'):
        self.motor_id = motor_id
        self.interface = interface
        self.bus = None
        # Scaling from Manual V3.0.1
        self.POS_SCALE = 0.1
        self.VEL_SCALE = 10.0
        self.CUR_SCALE = 0.01
        
    def connect(self):
        try:
            self.bus = can.interface.Bus(channel=self.interface, bustype='socketcan', bitrate=1000000)
            print(f"[Motor {self.motor_id}] Connected.")
            return True
        except Exception as e:
            print(f"[Motor {self.motor_id}] Connection Failed: {e}")
            return False

    def arm(self):
        """Sends zero commands to wake up the controller."""
        print("Arming motor...")
        for _ in range(10):
            self.send_torque(0)
            time.sleep(0.01)

    def send_torque(self, current_amps):
        """
        Sets motor torque (Current).
        Range: -10A to 10A (Safe limits)
        """
        # Command 1 = Set Current
        arb_id = (1 << 8) | self.motor_id
        
        # VESC expects milliamps (Int32)
        current_ma = int(current_amps * 1000.0)
        data = struct.pack('>i', current_ma)
        
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
        try:
            self.bus.send(msg)
        except can.CanError:
            pass # Skip if buffer full

    def get_state(self):
        """
        Returns (Position_Deg, Speed_RPM, Current_A) or None
        """
        # Drain buffer to get latest
        last_msg = None
        while True:
            msg = self.bus.recv(timeout=0)
            if not msg: break
            
            cmd = msg.arbitration_id >> 8
            dev = msg.arbitration_id & 0xFF
            
            if cmd == 41 and dev == self.motor_id: # ID 41 = Status
                last_msg = msg
        
        if last_msg:
            # Parse Data (8 bytes)
            # Unpack 4 signed 16-bit integers
            # Mapping based on your can_id_finder.py results:
            # 0-1: Position, 2-3: Voltage, 4-5: Torque/Current, 6-7: Speed/Temp
            vals = struct.unpack('>hhhh', last_msg.data)
            
            return {
                'pos': vals[0] * self.POS_SCALE,
                'voltage': vals[1] * 0.01,      # Assuming 0.01 scale from can_id_finder
                'torque': vals[2] * self.CUR_SCALE,
                'speed': vals[3] * 1.0          # Scale unknown, logging raw
            }
        return None

    def stop(self):
        self.send_torque(0)
        print("Motor Stopped.")

# --- EXAMPLE USAGE ---
if __name__ == "__main__":
    motor = MotorDriver(motor_id=104)
    if motor.connect():
        motor.arm()
        
        # Setup Logging
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"motor_log_{timestamp}.csv"
        print(f"Logging data to {filename}...")
        
        start = time.time()
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Time', 'Command_Torque', 'Position', 'Actual_Torque', 'Speed', 'Voltage'])
                
                while time.time() - start < 5.0:
                    # Sine Wave Torque (Swing back and forth)
                    import math
                    t = time.time() - start
                    cmd_torque = 1.0 * math.sin(t * 2.0)
                    
                    motor.send_torque(cmd_torque)
                    
                    state = motor.get_state()
                    if state:
                        print(f"Cmd: {cmd_torque:.2f}A | Act: {state['torque']:.2f}A | Pos: {state['pos']:.1f}°", end='\r')
                        writer.writerow([f"{t:.4f}", f"{cmd_torque:.4f}", 
                                         f"{state['pos']:.2f}", f"{state['torque']:.2f}", 
                                         f"{state['speed']:.2f}", f"{state['voltage']:.2f}"])
                    
                    time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            motor.stop()