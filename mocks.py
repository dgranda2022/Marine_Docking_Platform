import random
import time

class MockIMUReader:
    """Simulates the IMU by returning random noise."""
    def __init__(self):
        print("[MOCK] IMU Connected")
    
    def get_angles(self):
        # Simulate a slight wobble so you can see numbers changing
        return {
            "roll": random.uniform(-0.5, 0.5), 
            "pitch": random.uniform(-0.5, 0.5), 
            "yaw": 0.0
        }

class MockMotorDriver:
    """Simulates a CubeMars motor."""
    def __init__(self, motor_id=1, interface="can0"):
        self.motor_id = motor_id
        print(f"[MOCK] Motor {motor_id} Instantiated")

    def connect(self):
        # Always succeed
        return True

    def arm(self):
        print(f"[MOCK] Motor {self.motor_id} Armed")

    def send_torque(self, current_amps):
        # We don't need to do anything, just don't crash
        pass

    def get_state(self):
        # Return a fake state so the loop has data to log
        return {"pos": 0.0, "torque": 0.0}

    def stop(self):
        print(f"[MOCK] Motor {self.motor_id} Stopped")