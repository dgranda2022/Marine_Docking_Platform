#!/usr/bin/env python3
"""
system_check_uart.py — UART Pre-Flight Diagnostic for Marine Docking Platform
===============================================================================
Jetson Orin Nano  |  CubeMars AK60-39 V3.0  |  USB-to-UART Adapters

Run with:  python3 system_check_uart.py

This script replaces the CAN-specific system_check.py for the new UART
architecture.  It verifies:
    1. Each USB-to-UART adapter is physically present and openable.
    2. A VESC COMM_GET_VALUES request produces a parseable response from
       each motor, confirming end-to-end wiring and protocol correctness.

No sudo required — serial ports only need dialout group membership:
    sudo usermod -aG dialout $USER   (then log out / log back in)
"""

import struct
import sys
import time

import serial

# ── Imports from the project driver module ───────────────────────────────────
# We reuse the CRC and framing utilities so the diagnostic exercises the exact
# same code path the control loop will use at runtime.

from serial_motor_driver import (
    COMM_GET_VALUES,
    _build_vesc_frame,
    _parse_vesc_frame,
    vesc_crc16,
)

# ── Configuration ────────────────────────────────────────────────────────────

PORTS: list[dict] = [
    {"port": "/dev/ttyUSB0", "label": "Roll  (Motor 1)", "motor_id": 1},
    {"port": "/dev/ttyUSB1", "label": "Pitch (Motor 2)", "motor_id": 2},
]

BAUDRATE: int = 921600
RESPONSE_TIMEOUT_S: float = 0.5   # How long to wait for motor reply
NUM_RETRIES: int = 3              # Retry the request if first attempt fails

# ── ANSI Colour Helpers ──────────────────────────────────────────────────────

RESET  = "\033[0m"
BOLD   = "\033[1m"
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
DIM    = "\033[2m"


def c(colour: str, text: str) -> str:
    return f"{colour}{text}{RESET}"


def header(title: str) -> None:
    width = 70
    print()
    print(c(CYAN, "─" * width))
    print(c(BOLD + CYAN, f"  {title}"))
    print(c(CYAN, "─" * width))


def ok(msg: str) -> None:
    print(f"  {c(GREEN, '[ PASS ]')}  {msg}")


def fail(msg: str) -> None:
    print(f"  {c(RED,   '[ FAIL ]')}  {msg}")


def warn(msg: str) -> None:
    print(f"  {c(YELLOW, '[ WARN ]')}  {msg}")


def info(msg: str) -> None:
    print(f"  {c(DIM,    '[  --  ]')}  {msg}")


def hint(lines: list[str]) -> None:
    for line in lines:
        print(f"           {c(YELLOW, '▶')} {line}")


# ── Result Tracking ──────────────────────────────────────────────────────────

results: dict[str, bool] = {}


# ═══════════════════════════════════════════════════════════════════════════════
# TEST — Iterate through each configured UART port
# ═══════════════════════════════════════════════════════════════════════════════

print()
print(c(BOLD + CYAN, "  UART PRE-FLIGHT DIAGNOSTIC"))
print(c(CYAN,        "  Marine Docking Platform — CubeMars AK60-39 V3.0 (VESC Serial)"))

for idx, motor in enumerate(PORTS, start=1):
    port_path: str = motor["port"]
    label: str = motor["label"]
    motor_id: int = motor["motor_id"]

    header(f"TEST {idx} / {len(PORTS)}  —  {label}  ({port_path})")

    # ── Step 1: Open the serial port ─────────────────────────────────────
    ser = None
    try:
        ser = serial.Serial(
            port=port_path,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.05,
        )
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ok(f"Serial port {port_path} opened at {BAUDRATE} baud, 8N1.")
    except serial.SerialException as exc:
        fail(f"Cannot open {port_path}: {exc}")
        hint([
            f"Check that the USB-to-UART adapter is plugged into the Jetson.",
            f"Run:  ls -la /dev/ttyUSB*",
            f"If the device exists but you get 'Permission denied':",
            f"  sudo usermod -aG dialout $USER   (then log out / back in)",
            f"Or run this script with sudo for a quick test.",
        ])
        results[label] = False
        continue

    # ── Step 2: Send COMM_GET_VALUES and listen for a response ───────────
    request_payload = struct.pack(">B", COMM_GET_VALUES)
    request_frame = _build_vesc_frame(request_payload)

    info(f"COMM_GET_VALUES frame ({len(request_frame)} bytes): "
         f"{request_frame.hex().upper()}")

    response_ok = False

    for attempt in range(1, NUM_RETRIES + 1):
        info(f"Attempt {attempt}/{NUM_RETRIES} — sending request...")

        try:
            ser.reset_input_buffer()
            ser.write(request_frame)
            ser.flush()
        except (serial.SerialException, OSError) as exc:
            fail(f"Write failed: {exc}")
            continue

        # Collect bytes over the timeout window
        deadline = time.monotonic() + RESPONSE_TIMEOUT_S
        rx_buf = bytearray()

        while time.monotonic() < deadline:
            try:
                waiting = ser.in_waiting
                if waiting > 0:
                    rx_buf.extend(ser.read(waiting))
                else:
                    time.sleep(0.005)
            except (serial.SerialException, OSError):
                break

        if len(rx_buf) == 0:
            info(f"  No bytes received (0 bytes in {RESPONSE_TIMEOUT_S}s).")
            continue

        info(f"  Received {len(rx_buf)} bytes.")

        # ── Step 3: Parse the response ───────────────────────────────────
        payload = _parse_vesc_frame(bytes(rx_buf))

        if payload is None:
            info(f"  No valid VESC frame found in response.")
            info(f"  Raw hex (first 64 bytes): {bytes(rx_buf[:64]).hex().upper()}")
            continue

        if len(payload) < 1:
            info(f"  Frame found but payload is empty.")
            continue

        cmd_id = payload[0]
        if cmd_id != COMM_GET_VALUES:
            info(f"  Frame parsed but command ID is 0x{cmd_id:02X}, expected 0x04.")
            continue

        # Successfully received COMM_GET_VALUES response
        ok(f"Valid COMM_GET_VALUES response received ({len(payload)} byte payload).")

        # Try to extract and display key telemetry fields
        if len(payload) >= 45:
            try:
                temp_fet = struct.unpack_from(">h", payload, 1)[0] / 10.0
                temp_motor = struct.unpack_from(">h", payload, 3)[0] / 10.0
                avg_current = struct.unpack_from(">i", payload, 5)[0] / 100.0
                rpm = struct.unpack_from(">i", payload, 29)[0]
                tachometer = struct.unpack_from(">i", payload, 41)[0]
                vin = struct.unpack_from(">h", payload, 33)[0] / 10.0

                info(f"  FET Temp   : {temp_fet:.1f} °C")
                info(f"  Motor Temp : {temp_motor:.1f} °C")
                info(f"  Avg Current: {avg_current:.2f} A")
                info(f"  Input V    : {vin:.1f} V")
                info(f"  RPM        : {rpm}")
                info(f"  Tachometer : {tachometer}")

                if vin < 5.0:
                    warn(f"Input voltage is low ({vin:.1f}V). Is the battery connected?")

            except struct.error:
                info(f"  Could not unpack all telemetry fields (payload may be shorter than expected).")
        else:
            info(f"  Payload shorter than expected ({len(payload)}B < 45B) — partial decode skipped.")

        response_ok = True
        break

    # ── Record result ────────────────────────────────────────────────────
    if response_ok:
        results[label] = True
    else:
        fail(f"No valid response from {label} after {NUM_RETRIES} attempts.")
        hint([
            "Check the following:",
            "  • Motor power supply is connected and ON.",
            "  • USB-to-UART adapter TX→Motor RX, adapter RX→Motor TX (crossover).",
            "  • GND is shared between the adapter and the motor.",
            "  • Motor is configured for VESC UART mode at 921600 baud (use CubeMars app).",
            "  • The correct /dev/ttyUSBx is assigned to this motor.",
            f"  • Try:  screen {port_path} 921600   to check for raw traffic.",
        ])
        results[label] = False

    # ── Close port ───────────────────────────────────────────────────────
    try:
        ser.close()
        info("Serial port closed.")
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════════════════════
# FINAL SUMMARY
# ═══════════════════════════════════════════════════════════════════════════════

print()
print(c(CYAN, "═" * 70))
print(c(BOLD + CYAN, "  UART DIAGNOSTIC SUMMARY"))
print(c(CYAN, "═" * 70))

all_passed = True
for motor_cfg in PORTS:
    label = motor_cfg["label"]
    port_path = motor_cfg["port"]
    passed = results.get(label, False)
    if passed:
        badge = c(GREEN, "  PASS  ")
    else:
        badge = c(RED, "  FAIL  ")
        all_passed = False
    print(f"  [{badge}]  {label}  ({port_path})")

print(c(CYAN, "─" * 70))
if all_passed:
    print(c(BOLD + GREEN, "  ✔  All UART links verified. Ready for main control loop."))
else:
    fail_count = sum(1 for v in results.values() if not v)
    print(c(BOLD + RED,
            f"  ✘  {fail_count} link(s) failed. Fix wiring/config before running main.py."))
print(c(CYAN, "═" * 70))
print()

sys.exit(0 if all_passed else 1)