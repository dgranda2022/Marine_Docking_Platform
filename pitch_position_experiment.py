#!/usr/bin/env python3
"""
pitch_position_experiment.py
============================
EXPERIMENT — do not merge into main.py without review.

Pitch axis: MIT-mode position control via the motor's own internal servo
            (COMM_MIT 0x60 — position + velocity + feedforward torque + kp + kd)
Roll  axis: unchanged torque/current control with IMU feedback (same as main.py)

The pitch motor sends a position target of 0 (its power-on encoder reference)
with a stiffness Kp and damping Kd that the *motor's* controller executes
at its own high internal rate — no software PD loop needed for pitch.

Pre-condition: run `python3 motor_test.py recover` first so pitch is at 0°.

Usage:
    python3 pitch_position_experiment.py

Files modified:  NONE (main.py and serial_motor_driver.py are untouched)
New log file:    pitch_pos_experiment.csv
"""

import csv
import struct
import sys
import time

import serial

from imu_sensor import IMUReader
from pd_controller import PDController
# Import frame primitives only — motor driver files are not modified
from serial_motor_driver import _build_frame, _parse_frame

# ── Ports ─────────────────────────────────────────────────────────────────────

PORT_ROLL  = "/dev/ttyUSB1"
PORT_PITCH = "/dev/ttyUSB0"
BAUD       = 921600
LOOP_HZ    = 50
LOOP_DT    = 1.0 / LOOP_HZ
LOG_FILE   = "pitch_pos_experiment.csv"

# ── CubeMars command IDs ───────────────────────────────────────────────────────

CMD_GET_VALUES  = 0x45
CMD_SET_CURRENT = 0x47   # roll axis (unchanged)
CMD_MIT         = 0x60   # pitch axis — MIT position/velocity/torque control

# ── MIT mode frame builder ─────────────────────────────────────────────────────
#
# Frame: AA 15 60 [20 bytes data] CRC BB
# Data layout (5 × int32, big-endian, each value scaled × 1000):
#   pos_deg   — target position in degrees  (motor encoder frame, 0 = power-on ref)
#   vel_dps   — target velocity  deg/s      (0 = position hold)
#   torque_A  — feedforward torque in amps  (0 for pure position servo)
#   kp        — position stiffness  A/deg
#   kd        — velocity damping    A/(deg/s)
#
# Example from manual p.57: Kd=2 rad/s → 0x000007D0 = 2000 (Kd=2.0 * 1000)

def mit_frame(pos_deg: float = 0.0,
              vel_dps: float = 0.0,
              torque_A: float = 0.0,
              kp: float = 0.0,
              kd: float = 0.0) -> bytes:
    data = struct.pack(
        ">iiiii",
        int(pos_deg  * 1000),
        int(vel_dps  * 1000),
        int(torque_A * 1000),
        int(kp       * 1000),
        int(kd       * 1000),
    )
    return _build_frame(CMD_MIT, data)


def set_current_frame(amps: float) -> bytes:
    return _build_frame(CMD_SET_CURRENT, struct.pack(">i", int(amps * 1000)))


def get_values_frame() -> bytes:
    return _build_frame(CMD_GET_VALUES)


# ── MIT position control gains (motor-side) ────────────────────────────────────
#
# These are gains inside the *motor's* controller, not a software PD loop.
# kp: how many amps per degree of position error  (stiffness)
# kd: how many amps per deg/s of velocity error   (damping)
#
# Start conservatively — the motor has almost no load.
# Increase MIT_KP for stiffer hold; increase MIT_KD to suppress any bounce.

MIT_KP = 0.5    # A/deg  — start light, raise toward 1.0-2.0 if hold is weak
MIT_KD = 0.05   # A/(deg/s) — damping; raise if overshoot/bounce appears

# ── Roll PD gains (software loop, same as main.py) ────────────────────────────

ROLL_KP      = 0.20
ROLL_KD      = 0.030
MAX_TORQUE   = 3.0

TARGET_DEG   = 0.0   # level-hold setpoint for both axes

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
        print(f"[WARN] Cannot open {port}: {e}")
        return None


def drain_telemetry(ser: serial.Serial, rx_buf: bytearray) -> dict | None:
    """Non-blocking: drain RX, return parsed GET_VALUES payload or None."""
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
        if cmd == CMD_GET_VALUES and len(data) >= 48:
            last = {
                "curr": struct.unpack_from(">i", data, 4)[0] / 100.0,
                "rpm":  struct.unpack_from(">i", data, 22)[0],
                "tacho": struct.unpack_from(">i", data, 44)[0],
            }
        # Advance buffer past this frame
        idx = bytes(rx_buf).find(0xAA)
        if idx == -1:
            break
        length = rx_buf[idx + 1] if idx + 1 < len(rx_buf) else 0
        frame_end = idx + 2 + length + 3
        rx_buf[:] = rx_buf[frame_end:]
    return last


# ── Startup ───────────────────────────────────────────────────────────────────

print()
print("  pitch_position_experiment.py")
print("  Pitch: MIT position control (motor-internal servo)")
print("  Roll:  torque/current control (software PD + IMU)")
print()

imu = IMUReader()
time.sleep(0.4)

angles = imu.get_angles()
print(f"  IMU at startup — roll: {angles['roll']:+.2f}°  pitch: {angles['pitch']:+.2f}°")
if abs(angles["pitch"]) > 3.0:
    print("  [WARN] Pitch is not near 0°. Run `python3 motor_test.py recover` first.")
    print("  Continuing anyway — motor will servo to its encoder zero.")
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

ctrl_roll = PDController(Kp=ROLL_KP, Kd=ROLL_KD, max_torque=MAX_TORQUE)

rx_pitch = bytearray()
rx_roll  = bytearray()

# ── MIT position hold frame (pre-built, sent every cycle) ─────────────────────
# pos=0 → motor encoder position at power-on (should be ~level if recover was run)
PITCH_HOLD_FRAME = mit_frame(pos_deg=0.0, vel_dps=0.0, torque_A=0.0,
                              kp=MIT_KP, kd=MIT_KD)

print(f"  MIT hold frame: {PITCH_HOLD_FRAME.hex(' ').upper()}")
print(f"  MIT_KP={MIT_KP}  MIT_KD={MIT_KD}  (motor-side gains)")
print(f"  Roll  KP={ROLL_KP}  KD={ROLL_KD}  (software PD)")
print()
print("  Press Ctrl+C to stop.")
print()

# ── Log setup ─────────────────────────────────────────────────────────────────

log_fh  = open(LOG_FILE, "w", newline="")
log_csv = csv.writer(log_fh)
log_csv.writerow([
    "t_s", "roll_deg", "cmd_roll_A",
    "pitch_deg", "pitch_motor_rpm", "pitch_motor_curr_A",
    "dt_ms",
])
print(f"  Logging to {LOG_FILE}")

# ── 50 Hz loop ────────────────────────────────────────────────────────────────

prev_time = time.perf_counter()
loop_count = 0
pitch_rpm  = 0
pitch_curr = 0.0

try:
    while True:
        t0  = time.perf_counter()
        dt  = t0 - prev_time
        prev_time = t0

        angles = imu.get_angles()

        # ── Pitch: MIT position hold ──────────────────────────────────────
        if ser_pitch:
            ser_pitch.write(PITCH_HOLD_FRAME)
            # Request telemetry every 5 cycles (~10 Hz) to monitor
            if loop_count % 5 == 0:
                ser_pitch.write(get_values_frame())
            state_p = drain_telemetry(ser_pitch, rx_pitch)
            if state_p:
                pitch_rpm  = state_p["rpm"]
                pitch_curr = state_p["curr"]

        # ── Roll: software PD + current control ──────────────────────────
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
            f"Pitch:{angles['pitch']:+6.2f}° [MIT] rpm:{pitch_rpm:+5d} I:{pitch_curr:+5.2f}A",
            end="\r",
        )

        # ── Log ───────────────────────────────────────────────────────────
        log_csv.writerow([
            f"{loop_count * LOOP_DT:.3f}",
            f"{angles['roll']:.4f}",   f"{cmd_roll:.4f}",
            f"{angles['pitch']:.4f}",  f"{pitch_rpm}",
            f"{pitch_curr:.4f}",       f"{dt * 1000:.2f}",
        ])

        loop_count += 1

        used = time.perf_counter() - t0
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
