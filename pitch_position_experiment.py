#!/usr/bin/env python3
"""
pitch_position_experiment.py
============================
EXPERIMENT — compares MIT Force Control vs Servo Position Loop for pitch axis.
Do not merge into main.py without review.

Usage:
    python3 pitch_position_experiment.py mit      # MIT Force Control (0x60)
    python3 pitch_position_experiment.py servo    # Servo Position Loop (0x4A)

Pre-condition: run `python3 motor_test.py recover` first so pitch is near 0°.
Both modes call COMM_SET_POS_ORIGIN at startup to zero the encoder at the
current physical position, so position 0 = level regardless of power-on state.

Comparison:
    MIT mode   — motor's internal position + velocity servo via 0x60 frame.
                 Gains sent each cycle: Kp (A/rad), Kd (A/(rad/s)).
                 Motor closes its own loop at high rate; Jetson just sends target.

    Servo mode — motor's built-in position PID via COMM_SET_POS (0x4A).
                 Motor runs its own PID (tuned in CubeMarsTool); Jetson sends target.
                 Simpler — no gains to tune over serial.

Roll axis: unchanged software PD + COMM_SET_CURRENT, same as main.py.

For context — main.py uses COMM_SET_CURRENT (0x47): pure torque commands with
the entire PD control loop running in Python on the Jetson. The motor is dumb
torque actuation; the Jetson is the controller.

Files modified:  NONE (main.py and serial_motor_driver.py are untouched)
Log files:       pitch_mit_experiment.csv  or  pitch_servo_experiment.csv
"""

import csv
import struct
import sys
import time

import serial

from imu_sensor import IMUReader
from pd_controller import PDController
from serial_motor_driver import _build_frame, _parse_frame

# ── Mode selection ─────────────────────────────────────────────────────────────

MODE = sys.argv[1].lower() if len(sys.argv) > 1 else "mit"
if MODE not in ("mit", "servo"):
    print("Usage: python3 pitch_position_experiment.py [mit|servo]")
    sys.exit(1)

# ── Ports ─────────────────────────────────────────────────────────────────────

PORT_ROLL  = "/dev/ttyUSB1"
PORT_PITCH = "/dev/ttyUSB0"
BAUD       = 921600
LOOP_HZ    = 50
LOOP_DT    = 1.0 / LOOP_HZ
LOG_FILE   = f"pitch_{MODE}_experiment.csv"

# ── CubeMars command IDs ───────────────────────────────────────────────────────

CMD_GET_VALUES     = 0x45
CMD_SET_CURRENT    = 0x47   # roll axis — current/torque loop
CMD_SET_POS        = 0x4A   # servo position loop (motor internal PID)
CMD_SET_POS_ORIGIN = 0x40   # set current encoder position as origin (= 0)
CMD_MIT            = 0x60   # MIT force control (position + velocity + torque + kp + kd)

# ── MIT gains (corrected from first experiment) ───────────────────────────────
#
# AK60-39 parameter ranges (manual p.42):
#   Kp: 0–500 A/rad    Kd: 0–5 A/(rad/s)
#   Torque coefficient Kt = 3.4616 N·m/A
#
# First experiment used Kp=0.5 → only 0.12 A per radian of error → 0.43 N·m.
# Gravity easily overwhelmed this. New values:
#   Kp=20: 0.25 rad (14°) error → 5 A → 17.3 N·m  — strong hold
#   Kd=1.5: good damping without overshoot at Kp=20

MIT_KP = 20.0   # A/rad
MIT_KD =  1.5   # A/(rad/s)

# ── Roll PD gains (software loop — unchanged from main.py) ────────────────────

ROLL_KP    = 0.20
ROLL_KD    = 0.030
MAX_TORQUE = 3.0
TARGET_DEG = 0.0

# ── Frame builders ─────────────────────────────────────────────────────────────

def mit_frame(pos_rad: float = 0.0,
              vel_rad_s: float = 0.0,
              torque_Nm: float = 0.0,
              kp: float = 0.0,
              kd: float = 0.0) -> bytes:
    """MIT Force Control frame (0x60).

    All values in SI units; stored as int32 big-endian scaled ×1000.
    Layout: [pos_rad×1000][vel_rad_s×1000][torque_Nm×1000][kp×1000][kd×1000]
    Confirmed from manual p.52/58 examples, e.g.:
        pos=6 rad → 0x00001770, Kp=2 → 0x000007D0
    """
    data = struct.pack(
        ">iiiii",
        int(pos_rad   * 1000),
        int(vel_rad_s * 1000),
        int(torque_Nm * 1000),
        int(kp        * 1000),
        int(kd        * 1000),
    )
    return _build_frame(CMD_MIT, data)


def servo_pos_frame(deg: float = 0.0) -> bytes:
    """Servo position loop command (0x4A).

    Target position in degrees, encoded as int32 big-endian × 1,000,000.
    Confirmed from manual p.52: 180° → 0x0ABA9500 = 180,000,000.
    """
    return _build_frame(CMD_SET_POS, struct.pack(">i", int(deg * 1_000_000)))


def set_origin_frame() -> bytes:
    """COMM_SET_POS_ORIGIN (0x40) — sets current encoder position as 0."""
    return _build_frame(CMD_SET_POS_ORIGIN)


def set_current_frame(amps: float) -> bytes:
    return _build_frame(CMD_SET_CURRENT, struct.pack(">i", int(amps * 1000)))


def get_values_frame() -> bytes:
    return _build_frame(CMD_GET_VALUES)


# ── Serial helpers ─────────────────────────────────────────────────────────────

def open_port(port: str) -> serial.Serial | None:
    try:
        ser = serial.Serial(
            port=port, baudrate=BAUD,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0.01, write_timeout=0.01,
        )
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser
    except (serial.SerialException, OSError) as e:
        print(f"  [WARN] Cannot open {port}: {e}")
        return None


def drain_telemetry(ser: serial.Serial, rx_buf: bytearray) -> dict | None:
    try:
        w = ser.in_waiting
        if w:
            rx_buf.extend(ser.read(w))
    except (serial.SerialException, OSError):
        return None

    last = None
    while True:
        result = _parse_frame(bytes(rx_buf))
        if result is None:
            break
        cmd, data = result
        if cmd == CMD_GET_VALUES and len(data) >= 26:
            last = {
                "curr": struct.unpack_from(">i", data, 4)[0] / 100.0,
                "rpm":  struct.unpack_from(">i", data, 22)[0],
            }
        idx = bytes(rx_buf).find(0xAA)
        if idx == -1:
            break
        length = rx_buf[idx + 1] if idx + 1 < len(rx_buf) else 0
        frame_end = idx + 2 + length + 3
        rx_buf[:] = rx_buf[frame_end:]
    return last


# ── Startup ───────────────────────────────────────────────────────────────────

print()
print(f"  pitch_position_experiment.py  MODE={MODE.upper()}")
if MODE == "mit":
    print(f"  Pitch: MIT Force Control (0x60)")
    print(f"         Kp={MIT_KP} A/rad, Kd={MIT_KD} A/(rad/s)")
else:
    print(f"  Pitch: Servo Position Loop (0x4A) — motor internal PID, target=0°")
print(f"  Roll:  Current control (software PD, same as main.py)")
print()

imu = IMUReader()
time.sleep(0.4)

angles = imu.get_angles()
print(f"  IMU at startup — roll: {angles['roll']:+.2f}°  pitch: {angles['pitch']:+.2f}°")
if abs(angles["pitch"]) > 3.0:
    print("  [WARN] Pitch not near 0°. Run `python3 motor_test.py recover` first.")
    print("  Encoder origin will be set at the current (non-level) position.")
print()

ser_pitch = open_port(PORT_PITCH)
ser_roll  = open_port(PORT_ROLL)

# Arm both motors with zero-current frames
print("  Arming motors ...")
for _ in range(10):
    if ser_pitch:
        ser_pitch.write(set_current_frame(0.0))
    if ser_roll:
        ser_roll.write(set_current_frame(0.0))
    time.sleep(0.01)

# Set encoder origin: tells the motor "current physical position = position 0"
# This is critical — without it, position 0 is wherever the motor powered on,
# not the physical level position returned by motor_test.py recover.
if ser_pitch:
    print("  Setting pitch encoder origin to current position ...")
    for _ in range(5):
        ser_pitch.write(set_origin_frame())
        time.sleep(0.02)
    print(f"  Origin set at IMU pitch = {imu.get_angles()['pitch']:+.2f}°")

ctrl_roll = PDController(Kp=ROLL_KP, Kd=ROLL_KD, max_torque=MAX_TORQUE)
rx_pitch  = bytearray()
rx_roll   = bytearray()

# Pre-build the pitch hold frame (sent every cycle)
if MODE == "mit":
    PITCH_HOLD_FRAME = mit_frame(pos_rad=0.0, vel_rad_s=0.0, torque_Nm=0.0,
                                  kp=MIT_KP, kd=MIT_KD)
    print(f"  MIT hold frame ({len(PITCH_HOLD_FRAME)} bytes): "
          f"{PITCH_HOLD_FRAME.hex(' ').upper()}")
else:
    PITCH_HOLD_FRAME = servo_pos_frame(0.0)
    print(f"  Servo pos frame ({len(PITCH_HOLD_FRAME)} bytes): "
          f"{PITCH_HOLD_FRAME.hex(' ').upper()}")

print()
print(f"  Logging to {LOG_FILE}")
print("  Press Ctrl+C to stop.")
print()

# ── Log setup ─────────────────────────────────────────────────────────────────

log_fh  = open(LOG_FILE, "w", newline="")
log_csv = csv.writer(log_fh)
log_csv.writerow([
    "t_s", "roll_deg", "cmd_roll_A",
    "pitch_deg", "pitch_motor_rpm", "pitch_motor_curr_A", "dt_ms",
])

# ── 50 Hz control loop ────────────────────────────────────────────────────────

prev_time  = time.perf_counter()
loop_count = 0
pitch_rpm  = 0
pitch_curr = 0.0

try:
    while True:
        t0 = time.perf_counter()
        dt = t0 - prev_time
        prev_time = t0

        angles = imu.get_angles()

        # ── Pitch: position hold (MIT or Servo) ───────────────────────────
        if ser_pitch:
            ser_pitch.write(PITCH_HOLD_FRAME)
            if loop_count % 5 == 0:
                ser_pitch.write(get_values_frame())
            state_p = drain_telemetry(ser_pitch, rx_pitch)
            if state_p:
                pitch_rpm  = state_p["rpm"]
                pitch_curr = state_p["curr"]

        # ── Roll: software PD + current control (same as main.py) ────────
        cmd_roll = 0.0
        if ser_roll:
            cmd_roll = ctrl_roll.calculate(TARGET_DEG, angles["roll"], dt)
            ser_roll.write(set_current_frame(cmd_roll))
            if loop_count % 5 == 0:
                ser_roll.write(get_values_frame())
            drain_telemetry(ser_roll, rx_roll)

        # ── Status line ───────────────────────────────────────────────────
        print(
            f"t={loop_count * LOOP_DT:07.2f}s | "
            f"Roll:{angles['roll']:+6.2f}° cmd:{cmd_roll:+5.2f}A | "
            f"Pitch:{angles['pitch']:+6.2f}° [{MODE.upper()}] "
            f"rpm:{pitch_rpm:+5d} I:{pitch_curr:+5.2f}A",
            end="\r",
        )

        # ── CSV log ───────────────────────────────────────────────────────
        log_csv.writerow([
            f"{loop_count * LOOP_DT:.3f}",
            f"{angles['roll']:.4f}",  f"{cmd_roll:.4f}",
            f"{angles['pitch']:.4f}", f"{pitch_rpm}",
            f"{pitch_curr:.4f}",      f"{dt * 1000:.2f}",
        ])

        loop_count += 1

        used  = time.perf_counter() - t0
        slack = LOOP_DT - used
        if slack > 0:
            time.sleep(slack)

except KeyboardInterrupt:
    print("\n  Stopped by operator.")

finally:
    print("  Zeroing motors ...")
    for _ in range(15):
        if ser_pitch:
            ser_pitch.write(set_current_frame(0.0))
        if ser_roll:
            ser_roll.write(set_current_frame(0.0))
        time.sleep(0.01)
    if ser_pitch:
        ser_pitch.close()
    if ser_roll:
        ser_roll.close()
    log_fh.close()
    print(f"  Data saved to {LOG_FILE}")
