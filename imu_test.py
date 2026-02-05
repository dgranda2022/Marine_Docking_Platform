import time
import math
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

def quaternion_to_euler(i, j, k, real):
    sinr_cosp = 2 * (real * i + j * k)
    cosr_cosp = 1 - 2 * (i * i + j * j)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (real * j - k * i)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (real * k + i * j)
    cosy_cosp = 1 - 2 * (j * j + k * k)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def main():
    print("Initializing BNO085 (with Error Handling)...")
    
    # Try creating the I2C bus
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        bno = BNO08X_I2C(i2c)
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    except Exception as e:
        print(f"Startup Error: {e}")
        return

    print("IMU Active! Move the sensor...")
    print("--------------------------------------")

    while True:
        try:
            # --- THE DANGER ZONE ---
            # This is where the read usually fails
            quat = bno.quaternion
            
            if quat:
                # Unpack tuple (i, j, k, real)
                roll, pitch, yaw = quaternion_to_euler(quat[0], quat[1], quat[2], quat[3])
                
                # Print nicely formatted
                print(f"Roll: {roll:6.2f}°  |  Pitch: {pitch:6.2f}°  |  Yaw: {yaw:6.2f}°", end='\r')
            
        except OSError as e:
            # Check for "Remote I/O Error" (121)
            if e.errno == 121:
                # Just skip this frame and try again immediately
                continue
            else:
                # If it's a different error, print it but keep going
                print(f"\nBus Error: {e}")
                time.sleep(0.1)
                
        except RuntimeError as e:
            # BNO085 sometimes throws RuntimeErrors if it gets confused
            print(f"\nSensor Sync Error, resetting... {e}")
            time.sleep(0.1)
            
        except KeyboardInterrupt:
            print("\nStopping...")
            break

        # Small delay to let the bus breathe
        time.sleep(0.01)

if __name__ == "__main__":
    main()