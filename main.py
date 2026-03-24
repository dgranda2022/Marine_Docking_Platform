"""Main control loop — UART edition.

Orchestrates IMU reading, PD control, and motor output over VESC serial
links.  Drop-in replacement for the CAN-based main.py.

Hardware:
    - IMU:   BNO08x on I2C bus 7, address 0x4A
    - Pitch: CubeMars AK60-39 V3.0 on /dev/ttyUSB0  (confirmed)
    - Roll:  CubeMars AK60-39 V3.0 on /dev/ttyUSB1  (not yet connected)

Usage:
    python3 main.py
"""

import sys
import time

from imu_sensor import IMUReader
from pd_controller import PDController
from serial_motor_driver import SerialMotorDriver

# ── Port Configuration ───────────────────────────────────────────────────────
# Change these if your USB-to-UART adapters enumerate differently.
# Run `ls /dev/ttyUSB*` or check `dmesg | grep ttyUSB` after plugging in.

PORT_ROLL: str = "/dev/ttyUSB1"   # not connected — fails gracefully
PORT_PITCH: str = "/dev/ttyUSB0"  # confirmed pitch motor

# ── Control Parameters ───────────────────────────────────────────────────────

LOOP_PERIOD: float = 0.02     # 50 Hz
TARGET_ANGLE: float = 0.0     # Level-hold setpoint (degrees)

# ── Master State Dictionary ──────────────────────────────────────────────────

State: dict = {
    "IMU": {
        "roll": 0.0,
        "pitch": 0.0,
    },
    "Motors": {
        1: {"torque": 0.0, "pos": 0.0},
        2: {"torque": 0.0, "pos": 0.0},
    },
    "Tuning": {
        "Kp": 0.1,
        "Kd": 0.01,
        "max_torque": 3.0,
    },
}

# ── Hardware Initialisation ──────────────────────────────────────────────────

imu = IMUReader()

motor_roll = SerialMotorDriver(port=PORT_ROLL, motor_id=1)
motor_pitch = SerialMotorDriver(port=PORT_PITCH, motor_id=2)

if not motor_roll.connect():
    print(f"[FATAL] Failed to open UART for motor 1 (roll) on {PORT_ROLL}.",
          file=sys.stderr)
    #sys.exit(1)

if not motor_pitch.connect():
    print(f"[FATAL] Failed to open UART for motor 2 (pitch) on {PORT_PITCH}.",
          file=sys.stderr)
    motor_roll.stop()
    #sys.exit(1)

if motor_roll.ser is not None:
    print(f"[INFO] Roll  motor connected on {PORT_ROLL}")
else:
    print(f"[WARN] Roll  motor NOT connected ({PORT_ROLL}) — roll axis disabled.")
if motor_pitch.ser is not None:
    print(f"[INFO] Pitch motor connected on {PORT_PITCH}")
else:
    print(f"[WARN] Pitch motor NOT connected ({PORT_PITCH}) — pitch axis disabled.")

motor_roll.arm()
motor_pitch.arm()
print("[INFO] Both motors armed (10× zero-current wake-up sent).")

# ── Controller Instantiation ─────────────────────────────────────────────────

ctrl_roll = PDController(
    Kp=State["Tuning"]["Kp"],
    Kd=State["Tuning"]["Kd"],
    max_torque=State["Tuning"]["max_torque"],
)

ctrl_pitch = PDController(
    Kp=State["Tuning"]["Kp"],
    Kd=State["Tuning"]["Kd"],
    max_torque=State["Tuning"]["max_torque"],
)

# ── 50 Hz Control Loop ──────────────────────────────────────────────────────

print(f"[INFO] Entering control loop at {1.0 / LOOP_PERIOD:.0f} Hz. Press Ctrl+C to stop.")

try:
    prev_time: float = time.perf_counter()
    loop_count: int = 0

    while True:
        loop_start: float = time.perf_counter()
        dt: float = loop_start - prev_time
        prev_time = loop_start

        # ── IMU Read ─────────────────────────────────────────────────
        angles = imu.get_angles()
        State["IMU"]["roll"] = angles["roll"]
        State["IMU"]["pitch"] = angles["pitch"]

        # ── PD Control ───────────────────────────────────────────────
        cmd_roll: float = ctrl_roll.calculate(TARGET_ANGLE, angles["roll"], dt)
        cmd_pitch: float = ctrl_pitch.calculate(TARGET_ANGLE, angles["pitch"], dt)

        # ── Motor Output ─────────────────────────────────────────────
        # Sign is negated for pitch: confirmed empirically — positive RPM/current
        # reduces pitch angle, so the control output must be inverted.
        motor_roll.send_torque(cmd_roll)
        motor_pitch.send_torque(-cmd_pitch)

        # ── Request Telemetry for Next Cycle ─────────────────────────
        motor_roll.request_telemetry()
        motor_pitch.request_telemetry()

        # ── Motor Feedback ───────────────────────────────────────────
        state_roll = motor_roll.get_state()
        if state_roll is not None:
            State["Motors"][1]["torque"] = state_roll["torque"]
            State["Motors"][1]["pos"] = state_roll["pos"]

        state_pitch = motor_pitch.get_state()
        if state_pitch is not None:
            State["Motors"][2]["torque"] = state_pitch["torque"]
            State["Motors"][2]["pos"] = state_pitch["pos"]

        # ── Terminal Status ──────────────────────────────────────────
        print(
            f"t={loop_count * LOOP_PERIOD:07.2f}s | "
            f"Roll:{State['IMU']['roll']:+7.2f}° cmd:{cmd_roll:+6.3f}A | "
            f"Pitch:{State['IMU']['pitch']:+7.2f}° cmd:{cmd_pitch:+6.3f}A | "
            f"dt:{dt * 1000:5.1f}ms",
            end="\r",
        )

        loop_count += 1

        # ── Rate Limiting ────────────────────────────────────────────
        used: float = time.perf_counter() - loop_start
        sleep_time: float = LOOP_PERIOD - used
        if sleep_time > 0.0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\n[INFO] Interrupted by operator. Shutting down...")

finally:
    # Zero-torque both motors and release serial ports
    motor_roll.stop()
    motor_pitch.stop()
    print("[INFO] Motors stopped. Serial ports released.")