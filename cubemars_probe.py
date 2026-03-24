#!/usr/bin/env python3
"""
cubemars_probe.py — CubeMars AK series UART protocol probe
===========================================================
Tests the correct CubeMars UART framing from the AK series manual v3.2.0.

CubeMars UART frame format (manual section 4.3.2):
  [0xAA] [Length] [Cmd] [Data...] [CRC16_H] [CRC16_L] [0xBB]

  Length = 1 (cmd byte) + len(data)
  CRC16  = CRC16-CCITT (poly 0x1021, init 0x0000, MSB-first, no reflection)
           computed over [cmd + data] bytes only

Known-good frames verified against manual page 57-58:
  Get motor params (COMM_GET_VALUES 0x45, no data):
    AA 01 45 18 61 BB
  Set current 5A (COMM_SET_CURRENT 0x47, data = int32(5000)):
    AA 05 47 00 00 13 88 30 1C BB
"""

import struct
import sys
import time
import serial

PORT    = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD    = 921600
LISTEN  = 1.0   # seconds to wait for response per test

RESET  = "\033[0m";  BOLD = "\033[1m"
GREEN  = "\033[92m"; RED  = "\033[91m"
YELLOW = "\033[93m"; CYAN = "\033[96m"
DIM    = "\033[2m"

def c(col, txt): return f"{col}{txt}{RESET}"


# ── CRC16-CCITT (poly 0x1021, init 0x0000, MSB-first, no reflection) ─────────
# Verified: crc16([0x45]) = 0x1861  (matches manual page 57 example)
#           crc16([0x47, 0x00, 0x00, 0x13, 0x88]) = 0x301C  (5A frame)
def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        for _ in range(8):
            bit = (byte >> 7) & 1
            byte = (byte << 1) & 0xFF
            feedback = (crc >> 15) & 1
            crc = (crc << 1) & 0xFFFF
            if bit ^ feedback:
                crc ^= 0x1021
    return crc


# ── Frame builder ─────────────────────────────────────────────────────────────

def ak_frame(cmd: int, data: bytes = b'') -> bytes:
    """Build a CubeMars AK UART frame.

    Frame: [0xAA] [Length] [Cmd] [Data...] [CRC16_H] [CRC16_L] [0xBB]
    Length = 1 + len(data)
    CRC over [cmd byte] + [data bytes]
    """
    payload = bytes([cmd]) + data
    length = len(payload)
    chk = crc16(payload)
    return bytes([0xAA, length]) + payload + struct.pack(">H", chk) + bytes([0xBB])


# ── Verify known-good frames from manual ─────────────────────────────────────

_get_vals = ak_frame(0x45)
_set_5a   = ak_frame(0x47, struct.pack(">i", 5000))

expected_get = bytes.fromhex("AA014518 61BB".replace(" ", ""))
expected_5a  = bytes.fromhex("AA0547000013 8830 1CBB".replace(" ", ""))

assert _get_vals == expected_get, \
    f"CRC bug: get_vals = {_get_vals.hex(' ').upper()}, expected AA 01 45 18 61 BB"
assert _set_5a == expected_5a, \
    f"CRC bug: set_5a = {_set_5a.hex(' ').upper()}, expected AA 05 47 00 00 13 88 30 1C BB"


# ── Test list ─────────────────────────────────────────────────────────────────

TESTS = [
    # Safest read commands first — no motion
    ("COMM_GET_VALUES 0x45  (get motor state)",
     ak_frame(0x45)),

    ("COMM_SET_CURRENT 0x47 zero-current (safe)",
     ak_frame(0x47, struct.pack(">i", 0))),

    ("COMM_SET_CURRENT 0x47 0.1 A (100 mA, tiny motion test)",
     ak_frame(0x47, struct.pack(">i", 100))),

    ("COMM_SET_CURRENT 0x47 -0.1 A (-100 mA, reverse)",
     ak_frame(0x47, struct.pack(">i", -100))),

    ("Passive listen — no TX",
     b''),
]


# ── Run ───────────────────────────────────────────────────────────────────────

print()
print(c(BOLD + CYAN, f"  CUBEMARS AK PROBE (correct protocol) — {PORT} @ {BAUD:,}"))
print(c(CYAN, f"  {len(TESTS)} tests  |  known-good frames verified against manual"))
print(c(CYAN, "═" * 70))

hits = []

try:
    ser = serial.Serial(
        port=PORT, baudrate=BAUD,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE, timeout=0.05, write_timeout=0.1,
    )
except serial.SerialException as e:
    print(c(RED, f"  Cannot open {PORT}: {e}"))
    sys.exit(1)

for name, tx in TESTS:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.05)

    if tx:
        print(f"  TX ({len(tx)}B): {tx.hex(' ').upper()}")
        try:
            ser.write(tx)
            ser.flush()
        except Exception as e:
            print(c(RED, f"  Write failed: {e}"))
            continue

    deadline = time.monotonic() + LISTEN
    rx = bytearray()
    while time.monotonic() < deadline:
        w = ser.in_waiting
        if w:
            rx.extend(ser.read(w))
        else:
            time.sleep(0.01)

    if rx:
        print(c(BOLD + GREEN, f"  [ HIT! ] {name}"))
        print(f"           {len(rx)} bytes: {bytes(rx).hex(' ').upper()[:120]}")
        hits.append((name, bytes(rx)))
    else:
        print(c(DIM, f"  [      ] {name}"))
    print()

ser.close()

print(c(CYAN, "═" * 70))
if hits:
    print(c(BOLD + GREEN, f"  {len(hits)} RESPONSE(S) — motor is alive on {PORT}!"))
    for name, rx in hits:
        print(f"    ● {name}")
        print(f"      {rx.hex(' ').upper()}")
else:
    print(c(BOLD + RED, "  NO RESPONSE"))
    print()
    print(c(YELLOW, "  Checklist:"))
    print(c(YELLOW, "    1. Motor power on?  (LED should be lit)"))
    print(c(YELLOW, "    2. TX/RX not swapped?  Try flipping the two data wires."))
    print(c(YELLOW, "    3. Ground shared between Jetson and motor?"))
    print(c(YELLOW, "    4. Baud rate 921600 confirmed in AK Config UI?"))
    print(c(YELLOW, "    5. Correct JST port on motor? (UART port, not CAN)"))
print(c(CYAN, "═" * 70))
