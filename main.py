"""Main control loop with integrated stability monitoring and keyboard input."""

import sys
import time
from collections import deque

#from imu_sensor import IMUReader
from mocks import MockIMUReader as IMUReader      # <--- Aliased mock
from keyboard_listener import KeyboardListener
#from motor_driver import MotorDriver
from mocks import MockMotorDriver as MotorDriver  # <--- Aliased mock
from pd_controller import PDController

# ── Master state dictionary ──────────────────────────────────────────────────

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
    "Landing_Logic": {
        "armed": False,
        "is_stable": False,
        "stability_timer": 0.0,
        "ready_signal": False,
    },
}

# ── Hardware initialisation ──────────────────────────────────────────────────

imu = IMUReader()

motor_roll = MotorDriver(motor_id=1)
motor_pitch = MotorDriver(motor_id=2)

if not motor_roll.connect():
    print("[FATAL] Failed to open CAN bus for motor 1 (roll).", file=sys.stderr)
    sys.exit(1)

if not motor_pitch.connect():
    print("[FATAL] Failed to open CAN bus for motor 2 (pitch).", file=sys.stderr)
    motor_roll.stop()
    sys.exit(1)

motor_roll.arm()
motor_pitch.arm()

# ── Controller instantiation ─────────────────────────────────────────────────

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

# ── Keyboard listener ────────────────────────────────────────────────────────

listener = KeyboardListener(State)
listener.start()

# ── Stability buffer (1 second window at 50 Hz) ─────────────────────────────

error_buffer: deque = deque(maxlen=50)

# ── 50 Hz control loop ──────────────────────────────────────────────────────

LOOP_PERIOD: float = 0.02  # 50 Hz
TARGET_ANGLE: float = 0.0

try:
    prev_time: float = time.perf_counter()
    loop_count: int = 0

    while True:
        loop_start: float = time.perf_counter()
        dt: float = loop_start - prev_time
        prev_time = loop_start

        # ── IMU read ─────────────────────────────────────────────────
        angles = imu.get_angles()
        State["IMU"]["roll"] = angles["roll"]
        State["IMU"]["pitch"] = angles["pitch"]

        # ── PD control ───────────────────────────────────────────────
        cmd_roll: float = ctrl_roll.calculate(TARGET_ANGLE, angles["roll"], dt)
        cmd_pitch: float = ctrl_pitch.calculate(TARGET_ANGLE, angles["pitch"], dt)

        # ── Motor output ─────────────────────────────────────────────
        motor_roll.send_torque(cmd_roll)
        motor_pitch.send_torque(cmd_pitch)

        # ── Motor feedback ───────────────────────────────────────────
        state_roll = motor_roll.get_state()
        if state_roll is not None:
            State["Motors"][1]["torque"] = state_roll["torque"]
            State["Motors"][1]["pos"] = state_roll["pos"]

        state_pitch = motor_pitch.get_state()
        if state_pitch is not None:
            State["Motors"][2]["torque"] = state_pitch["torque"]
            State["Motors"][2]["pos"] = state_pitch["pos"]

        # ── Sliding window stability logic ───────────────────────────
        pitch_error: float = abs(TARGET_ANGLE - State["IMU"]["pitch"])
        error_buffer.append(pitch_error)

        landing = State["Landing_Logic"]

        # Gate 1: Arming check
        if not landing["armed"]:
            landing["stability_timer"] = 0.0
            landing["is_stable"] = False
            landing["ready_signal"] = False
        else:
            # Gate 2: Delta check over the sliding window
            if len(error_buffer) > 0:
                delta: float = max(error_buffer) - min(error_buffer)
            else:
                delta = float("inf")

            if delta < 2.0:
                landing["stability_timer"] += dt
                landing["is_stable"] = True
            else:
                landing["stability_timer"] = 0.0
                landing["is_stable"] = False

            # Gate 3: Verdict
            landing["ready_signal"] = landing["stability_timer"] > 5.0

        # ── Terminal status ──────────────────────────────────────────
        if landing["ready_signal"]:
            land_status = "[*** LAND ***]"
        elif landing["armed"]:
            land_status = f"[ARMED: {landing['stability_timer']:.1f}s]"
        else:
            land_status = "[DISARMED]"

        elapsed: float = time.perf_counter() - loop_start
        print(
            f"t={loop_count * LOOP_PERIOD:07.2f}s | "
            f"Roll:{State['IMU']['roll']:+7.2f}° cmd:{cmd_roll:+6.3f}A | "
            f"Pitch:{State['IMU']['pitch']:+7.2f}° cmd:{cmd_pitch:+6.3f}A | "
            f"dt:{dt * 1000:5.1f}ms {land_status}",
            end="\r",
        )

        loop_count += 1

        # ── Rate limiting ────────────────────────────────────────────
        used: float = time.perf_counter() - loop_start
        sleep_time: float = LOOP_PERIOD - used
        if sleep_time > 0.0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\n[INFO] Interrupted by operator. Shutting down...")

finally:
    listener.stop()
    motor_roll.stop()
    motor_pitch.stop()
    print("[INFO] Listener stopped. Motors stopped. CAN bus released.")