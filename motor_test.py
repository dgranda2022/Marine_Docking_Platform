#!/usr/bin/env python3
"""
motor_test.py — IMU-guided motor test for CubeMars AK60-39.

Modes:
  python3 motor_test.py recover   — drive pitch back to 0° using IMU feedback
  python3 motor_test.py sweep     — controlled ±N° sweep once 0° is confirmed

Uses IMU pitch as the real feedback so tachometer calibration doesn't matter.
"""

import struct
import sys
import time

import serial
from imu_sensor import IMUReader
from serial_motor_driver import _build_frame, _parse_frame

PORT     = "/dev/ttyUSB0"
BAUD     = 921600
CMD_GET_VALUES  = 0x45
CMD_SET_CURRENT = 0x47
CMD_SET_RPM     = 0x49

RESET = "\033[0m"; BOLD = "\033[1m"
GREEN = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"
CYAN = "\033[96m";  DIM = "\033[2m"
def c(col, txt): return f"{col}{txt}{RESET}"


# ── Motor helpers ─────────────────────────────────────────────────────────────

def open_port() -> serial.Serial:
    return serial.Serial(
        port=PORT, baudrate=BAUD,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE, timeout=0.05, write_timeout=0.05,
    )

def set_rpm(ser, rpm: int):
    ser.write(_build_frame(CMD_SET_RPM, struct.pack(">i", rpm)))

def set_current(ser, amps: float):
    ser.write(_build_frame(CMD_SET_CURRENT, struct.pack(">i", int(amps * 1000))))

def zero_motor(ser, n: int = 15):
    for _ in range(n):
        set_current(ser, 0.0)
        time.sleep(0.01)


# ── Mode: recover — bring pitch back to 0° ───────────────────────────────────

def recover():
    print(c(BOLD + CYAN, "\n  RECOVER — driving pitch to 0°"))
    print(c(CYAN, "═" * 55))

    imu = IMUReader()
    time.sleep(0.4)

    angles = imu.get_angles()
    pitch = angles["pitch"]
    print(f"  Current IMU pitch: {pitch:+.2f}°")

    if abs(pitch) < 1.5:
        print(c(GREEN, "  Already at 0° — nothing to do."))
        return

    # CONFIRMED: positive RPM reduces pitch, negative RPM increases pitch
    direction = +1 if pitch > 0 else -1
    RPM = 200   # motor-shaft RPM (~3 deg/s at output based on calibration)

    print(f"  Applying {direction * RPM:+d} RPM until |pitch| < 1.5°")
    print(f"  (Ctrl-C to emergency stop)")
    print()

    ser = open_port()
    zero_motor(ser)

    t_start = time.monotonic()
    MAX_TIME = 15.0   # hard time limit — never run more than 15 s

    try:
        while True:
            angles = imu.get_angles()
            pitch  = angles["pitch"]
            elapsed = time.monotonic() - t_start

            print(f"  t={elapsed:5.1f}s  pitch={pitch:+7.2f}°", end="\r")

            if abs(pitch) < 1.5:
                print()
                print(c(GREEN, f"  Reached {pitch:+.2f}° — stopping."))
                break

            if elapsed > MAX_TIME:
                print()
                print(c(RED, f"  Time limit reached at pitch={pitch:+.2f}° — stopping."))
                break

            # Slow down when close to avoid overshoot
            if abs(pitch) < 8.0:
                rpm_cmd = direction * 80
            else:
                rpm_cmd = direction * RPM

            set_rpm(ser, rpm_cmd)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print()
        print(c(RED, "  Emergency stop!"))
    finally:
        zero_motor(ser)
        ser.close()

    # Final IMU reading
    time.sleep(0.3)
    angles = imu.get_angles()
    print(f"  Final pitch: {angles['pitch']:+.2f}°")
    print(c(CYAN, "═" * 55))


# ── Mode: sweep — ±N° around current 0 ───────────────────────────────────────

def sweep(target_deg: float = 10.0):
    print(c(BOLD + CYAN, f"\n  SWEEP ±{target_deg:.0f}° (IMU-guided)"))
    print(c(CYAN, "═" * 55))

    imu = IMUReader()
    time.sleep(0.4)

    angles = imu.get_angles()
    pitch0 = angles["pitch"]
    print(f"  Start pitch: {pitch0:+.2f}°")

    if abs(pitch0) > 3.0:
        print(c(YELLOW, "  Pitch not near 0° — run 'recover' first."))
        return

    ser = open_port()
    zero_motor(ser)

    RPM_SLOW = 80    # motor-shaft RPM near limits (~0.8°/s)
    RPM_FAST = 300   # motor-shaft RPM in the middle (~3°/s, confirmed in recovery)

    def drive_to(target: float, label: str):
        """Drive motor until IMU pitch reaches target, with hard time limit."""
        # INVERTED: positive RPM reduces pitch, negative RPM increases pitch
        direction = -1 if target > imu.get_angles()["pitch"] else +1
        print(c(BOLD, f"  {label}: target pitch {target:+.1f}°"))
        t0 = time.monotonic()
        while time.monotonic() - t0 < 20.0:
            pitch = imu.get_angles()["pitch"]
            err   = target - pitch
            print(f"    pitch={pitch:+7.2f}°  target={target:+6.1f}°  err={err:+6.2f}°",
                  end="\r")

            if abs(err) < 1.0:
                print()
                print(c(GREEN, f"    Reached {pitch:+.2f}°"))
                return

            rpm = RPM_SLOW if abs(err) < 5.0 else RPM_FAST
            set_rpm(ser, direction * rpm)
            time.sleep(0.02)
        print()
        print(c(YELLOW, f"    Time limit — ended at {imu.get_angles()['pitch']:+.2f}°"))

    try:
        drive_to(+target_deg, "Phase 1 forward")
        time.sleep(0.5)
        drive_to(-target_deg, "Phase 2 reverse")
        time.sleep(0.5)
        drive_to(0.0, "Phase 3 return")
    except KeyboardInterrupt:
        print(c(RED, "\n  Emergency stop!"))
    finally:
        zero_motor(ser)
        ser.close()

    time.sleep(0.3)
    angles = imu.get_angles()
    print(f"  Final pitch: {angles['pitch']:+.2f}°")
    print(c(CYAN, "═" * 55))


# ── Entry point ───────────────────────────────────────────────────────────────

mode = sys.argv[1] if len(sys.argv) > 1 else "recover"

if mode == "recover":
    recover()
elif mode == "sweep":
    target = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    sweep(target)
else:
    print(f"Usage: python3 motor_test.py [recover | sweep [deg]]")
