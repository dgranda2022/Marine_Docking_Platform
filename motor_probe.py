#!/usr/bin/env python3
"""
motor_probe.py — Brute-Force Protocol & Baud Rate Scanner
==========================================================
Tries every reasonable combination of baud rate and command protocol
against a CubeMars AK60-39 on a USB-to-UART adapter to find what
makes it respond.

Protocols tested:
    1. VESC COMM_GET_VALUES (0x04) — standard VESC serial framing
    2. VESC COMM_FW_VERSION (0x00) — lightest possible VESC request
    3. CubeMars "Enter Motor Mode" — MIT-mode enter frame over UART
    4. Raw 0x00 ping — some bootloaders echo on null bytes
    5. CubeMars periodic feedback trigger — zero-current set command

Usage:
    python3 motor_probe.py [port]
    Default port: /dev/ttyUSB0
"""

import struct
import sys
import time

import serial

# ── Configuration ────────────────────────────────────────────────────────────

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

BAUD_RATES = [
    115200,     # CubeMars factory default
    921600,     # High-speed VESC
    500000,     # Common VESC default
    256000,     # Some VESC builds
    1000000,    # Occasional CubeMars config
    460800,     # FTDI common
    57600,      # Legacy
    9600,       # Fallback / bootloader
]

LISTEN_TIME_S = 0.6   # Time to wait for a response per test
INTER_TEST_PAUSE = 0.1

# ── ANSI ─────────────────────────────────────────────────────────────────────

RESET  = "\033[0m"
BOLD   = "\033[1m"
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
DIM    = "\033[2m"

def c(colour, text):
    return f"{colour}{text}{RESET}"

# ── VESC CRC16 ───────────────────────────────────────────────────────────────

_CRC16_TAB = [
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
    0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
    0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x54A5,
    0xA54A,0xB56B,0x8508,0x9529,0xE5CE,0xF5EF,0xC58C,0xD5AD,
    0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
    0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
    0x4864,0x5845,0x6826,0x7807,0x08E0,0x18C1,0x28A2,0x38A3,
    0xC94C,0xD96D,0xE90E,0xF92F,0x89C8,0x99E9,0xA98A,0xB9AB,
    0x5A55,0x4A74,0x7A17,0x6A36,0x1AD1,0x0AF0,0x3A93,0x2AB2,
    0xDB5D,0xCB7C,0xFB1F,0xEB3E,0x9BD9,0x8BF8,0xBB9B,0xAB9A,
    0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
    0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
    0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
    0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
    0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
    0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
    0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
    0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
    0xB5EA,0xA5CB,0x95A8,0x85A9,0xF54E,0xE56F,0xD50C,0xC52D,
    0x34C2,0x24E3,0x1480,0x04A1,0x7446,0x6467,0x5404,0x4425,
    0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
    0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
    0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
    0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
    0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
    0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
    0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
    0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
    0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
    0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0,
]

def vesc_crc16(data):
    crc = 0
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC16_TAB[((crc >> 8) ^ b) & 0xFF]
    return crc

def vesc_frame(payload):
    """Build a short-format VESC serial frame."""
    length = len(payload)
    crc = vesc_crc16(payload)
    return bytes([0x02, length]) + payload + struct.pack(">H", crc) + bytes([0x03])

def vesc_long_frame(payload):
    """Build a long-format VESC serial frame (start byte 0x03, 2-byte length)."""
    length = len(payload)
    crc = vesc_crc16(payload)
    return bytes([0x03]) + struct.pack(">H", length) + payload + struct.pack(">H", crc) + bytes([0x03])

# ── Test Payloads ────────────────────────────────────────────────────────────

TESTS = []

# 1. VESC COMM_FW_VERSION (0x00) — lightest request, always supported
fw_payload = struct.pack(">B", 0x00)
TESTS.append(("VESC COMM_FW_VERSION (0x00)", vesc_frame(fw_payload)))

# 2. VESC COMM_GET_VALUES (0x04)
gv_payload = struct.pack(">B", 0x04)
TESTS.append(("VESC COMM_GET_VALUES (0x04)", vesc_frame(gv_payload)))

# 3. VESC COMM_SET_CURRENT (0x06) with 0.0A — might trigger a status echo
sc_payload = struct.pack(">Bi", 0x06, 0)
TESTS.append(("VESC COMM_SET_CURRENT 0.0A (0x06)", vesc_frame(sc_payload)))

# 4. VESC COMM_ALIVE (0x1E / 30) — keepalive, some FW echoes
alive_payload = struct.pack(">B", 0x1E)
TESTS.append(("VESC COMM_ALIVE (0x1E)", vesc_frame(alive_payload)))

# 5. VESC COMM_GET_VALUES using LONG frame format (start=0x03, 2-byte len)
TESTS.append(("VESC COMM_GET_VALUES long-frame", vesc_long_frame(gv_payload)))

# 6. CubeMars MIT-mode Enter Motor (used in your CAN code, adapted to UART)
#    8-byte payload: FF FF FF FF FF FF FF FC
mit_enter = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
TESTS.append(("CubeMars MIT Enter Motor (raw 8-byte)", mit_enter))

# 7. CubeMars MIT-mode Enter wrapped in VESC frame
TESTS.append(("CubeMars MIT Enter (VESC-framed)", vesc_frame(mit_enter)))

# 8. Raw null byte probe
TESTS.append(("Raw null bytes (bootloader probe)", b'\x00\x00\x00\x00'))

# 9. Passive listen — send nothing, just check if motor is broadcasting
TESTS.append(("Passive listen (no TX, check for periodic broadcast)", b''))


# ── Run All Tests ────────────────────────────────────────────────────────────

print()
print(c(BOLD + CYAN, f"  MOTOR PROBE — {PORT}"))
print(c(CYAN, f"  Testing {len(BAUD_RATES)} baud rates × {len(TESTS)} protocols"))
print(c(CYAN, "═" * 70))

hits = []

for baud in BAUD_RATES:
    print()
    print(c(BOLD + YELLOW, f"  ── Baud Rate: {baud:,} ──"))

    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=0.1,
        )
    except serial.SerialException as exc:
        print(f"    {c(RED, '[ FAIL ]')} Cannot open port at {baud}: {exc}")
        continue

    for test_name, tx_data in TESTS:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.02)   # Let buffers settle

        # Send the test data (if any)
        if len(tx_data) > 0:
            try:
                ser.write(tx_data)
                ser.flush()
            except (serial.SerialException, OSError) as exc:
                print(f"    {c(RED, '[ ERR  ]')} {test_name}: write failed: {exc}")
                continue

        # Collect response
        deadline = time.monotonic() + LISTEN_TIME_S
        rx_buf = bytearray()
        while time.monotonic() < deadline:
            try:
                waiting = ser.in_waiting
                if waiting > 0:
                    rx_buf.extend(ser.read(waiting))
                else:
                    time.sleep(0.01)
            except (serial.SerialException, OSError):
                break

        if len(rx_buf) > 0:
            hex_preview = bytes(rx_buf[:48]).hex(" ").upper()
            print(f"    {c(GREEN, '[ HIT! ]')} {test_name}")
            print(f"             {len(rx_buf)} bytes: {hex_preview}")
            hits.append({
                "baud": baud,
                "test": test_name,
                "rx_len": len(rx_buf),
                "rx_hex": bytes(rx_buf[:64]).hex().upper(),
            })
        else:
            print(f"    {c(DIM, '[      ]')} {test_name} — no response")

        time.sleep(INTER_TEST_PAUSE)

    ser.close()


# ── Summary ──────────────────────────────────────────────────────────────────

print()
print(c(CYAN, "═" * 70))
if len(hits) == 0:
    print(c(BOLD + RED, "  NO RESPONSE AT ANY BAUD RATE / PROTOCOL"))
    print()
    print(c(YELLOW, "  This means the motor is NOT transmitting on its UART TX pin."))
    print(c(YELLOW, "  Likely causes:"))
    print(c(YELLOW, "    1. TX/RX are swapped — try flipping the two data wires"))
    print(c(YELLOW, "    2. UART is not enabled on the motor — need R-Link + CubeMars app"))
    print(c(YELLOW, "       to set App → UART under 'General → App to use'"))
    print(c(YELLOW, "    3. The motor UART port uses 3.3V logic but your adapter is 5V"))
    print(c(YELLOW, "       (or vice versa) — check voltage levels"))
    print(c(YELLOW, "    4. Motor firmware doesn't support UART at all (older FW)"))
    print()
else:
    print(c(BOLD + GREEN, f"  {len(hits)} RESPONSE(S) DETECTED!"))
    print()
    for h in hits:
        print(f"  {c(GREEN, '●')} Baud={h['baud']:>7,}  Test={h['test']}")
        print(f"    {h['rx_len']} bytes: {h['rx_hex'][:80]}")
        print()

print(c(CYAN, "═" * 70))
print()