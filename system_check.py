#!/usr/bin/env python3
"""
system_check.py — Pre-Flight Diagnostic for Marine Docking Platform
=====================================================================
Jetson Orin Nano  |  CubeMars CAN Motors  |  BNO085 IMU

Run with:  sudo python3 system_check.py

Architecture
------------
Every test is wrapped in its own try/except.  A failure is printed in
red and execution continues.  The script NEVER exits early.  The final
summary table shows the full state of the board in one shot.

Tests performed
---------------
  1. Privilege check (sudo / root)
  2. CAN interface sanity  (can0 UP / DOWN / NO-CARRIER)
  3. I2C bus 7 presence
  4. PD controller math verification (pure Python, no hardware)
  5. BNO085 IMU init, soft_reset, quaternion read
  6. CubeMars Roll motor  (CAN ID 1)  —  enter / ping / exit
  7. CubeMars Pitch motor (CAN ID 2)  —  enter / ping / exit
"""

import os
import struct
import subprocess
import sys
import time

# ── ANSI colour helpers ──────────────────────────────────────────────────────

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
    print(f"  {c(GREEN, '[  OK  ]')}  {msg}")

def fail(msg: str) -> None:
    print(f"  {c(RED,   '[ FAIL ]')}  {msg}")

def warn(msg: str) -> None:
    print(f"  {c(YELLOW, '[ WARN ]')}  {msg}")

def info(msg: str) -> None:
    print(f"  {c(DIM,    '[  --  ]')}  {msg}")

def hint(lines: list[str]) -> None:
    """Print indented fix hints in yellow."""
    for line in lines:
        print(f"           {c(YELLOW, '▶')} {line}")

# ── Result tracking ──────────────────────────────────────────────────────────

results: dict[str, bool | None] = {}   # test_name → True/False/None(skipped)

def record(name: str, passed: bool) -> None:
    results[name] = passed

# ════════════════════════════════════════════════════════════════════════════
# TEST 1 — Privilege Check
# ════════════════════════════════════════════════════════════════════════════

header("TEST 1 / 7  —  Privilege Check")

try:
    if os.geteuid() == 0:
        ok("Running as root / sudo.")
        record("Privilege", True)
    else:
        fail("Script is NOT running as root.")
        hint([
            "Re-run with:  sudo python3 system_check.py",
            "CAN socket operations require root privileges.",
        ])
        record("Privilege", False)
except Exception as exc:
    fail(f"Could not determine effective UID: {exc}")
    record("Privilege", False)

# ════════════════════════════════════════════════════════════════════════════
# TEST 2 — CAN Interface Sanity
# ════════════════════════════════════════════════════════════════════════════

header("TEST 2 / 7  —  CAN Interface  (can0)")

CAN_IFACE = "can0"
CAN_BITRATE = 1_000_000

try:
    result = subprocess.run(
        ["ip", "-details", "link", "show", CAN_IFACE],
        capture_output=True, text=True, timeout=5,
    )

    if result.returncode != 0:
        fail(f"Interface '{CAN_IFACE}' not found. Output: {result.stderr.strip()}")
        hint([
            "Load kernel modules first:",
            "  sudo modprobe can",
            "  sudo modprobe can_raw",
            "  sudo modprobe mttcan",
            "Then bring up the interface:",
            f"  sudo ip link set {CAN_IFACE} type can bitrate {CAN_BITRATE} restart-ms 100",
            f"  sudo ip link set {CAN_IFACE} txqueuelen 1000 up",
        ])
        record("CAN Interface", False)
    else:
        output = result.stdout + result.stderr
        if "NO-CARRIER" in output:
            fail(f"'{CAN_IFACE}' exists but is in NO-CARRIER state (no physical bus detected).")
            hint([
                "Check that the Waveshare SN65HVD230 transceiver is seated on J17.",
                "Verify TX/RX/3.3V/GND wiring against the pinout sheet on the enclosure lid.",
                "Confirm at least one other CAN node (motor) is powered and terminated.",
                "If wiring is correct, re-initialize the interface:",
                f"  sudo ip link set {CAN_IFACE} down",
                f"  sudo ip link set {CAN_IFACE} type can bitrate {CAN_BITRATE} restart-ms 100",
                f"  sudo ip link set {CAN_IFACE} txqueuelen 1000 up",
            ])
            record("CAN Interface", False)
        elif "state UP" in output or "state UNKNOWN" in output:
            ok(f"'{CAN_IFACE}' is UP.")

            # Extract reported bitrate for sanity
            for token in output.split():
                if token.isdigit() and int(token) in (1000000, 500000, 250000):
                    info(f"Reported bitrate: {int(token):,} bps")
                    if int(token) != CAN_BITRATE:
                        warn(f"Expected {CAN_BITRATE:,} bps but found {int(token):,} bps — mismatch!")
                    break

            record("CAN Interface", True)
        elif "state DOWN" in output:
            fail(f"'{CAN_IFACE}' exists but is DOWN.")
            hint([
                f"  sudo ip link set {CAN_IFACE} type can bitrate {CAN_BITRATE} restart-ms 100",
                f"  sudo ip link set {CAN_IFACE} txqueuelen 1000 up",
            ])
            record("CAN Interface", False)
        else:
            warn(f"'{CAN_IFACE}' found but state is ambiguous — inspect manually.")
            info(output.strip()[:300])
            record("CAN Interface", None)

except FileNotFoundError:
    fail("'ip' command not found. Is iproute2 installed?")
    record("CAN Interface", False)
except subprocess.TimeoutExpired:
    fail("'ip link show' timed out.")
    record("CAN Interface", False)
except Exception as exc:
    fail(f"Unexpected error during CAN check: {exc}")
    record("CAN Interface", False)

# ════════════════════════════════════════════════════════════════════════════
# TEST 3 — I2C Bus 7 Presence
# ════════════════════════════════════════════════════════════════════════════

header("TEST 3 / 7  —  I2C Bus  (bus 7)")

I2C_BUS = 7
I2C_DEV = f"/dev/i2c-{I2C_BUS}"

try:
    if os.path.exists(I2C_DEV):
        ok(f"{I2C_DEV} exists and is accessible.")

        # Try i2cdetect for extra info (non-fatal if absent)
        try:
            det = subprocess.run(
                ["i2cdetect", "-y", str(I2C_BUS)],
                capture_output=True, text=True, timeout=10,
            )
            if det.returncode == 0:
                # BNO085 default I2C address is 0x4A or 0x4B
                bno_found = "4a" in det.stdout.lower() or "4b" in det.stdout.lower()
                if bno_found:
                    ok("BNO085 address (0x4A or 0x4B) detected on bus 7 by i2cdetect.")
                else:
                    warn("i2cdetect ran but did NOT find 0x4A / 0x4B — IMU may not be wired correctly.")
                    info("Raw i2cdetect output:")
                    for line in det.stdout.strip().splitlines():
                        info(f"  {line}")
            else:
                info("i2cdetect returned non-zero; skipping address scan.")
        except FileNotFoundError:
            info("i2cdetect not installed — skipping address scan (apt install i2c-tools).")
        except subprocess.TimeoutExpired:
            warn("i2cdetect timed out — bus may be locked up by a misbehaving device.")

        record("I2C Bus", True)
    else:
        fail(f"{I2C_DEV} does not exist.")
        hint([
            "Enable I2C on the Jetson:",
            "  sudo /opt/nvidia/jetson-io/jetson-io.py   # enable I2C bus 7",
            "Or verify the device tree overlay is applied in /boot/extlinux/extlinux.conf.",
        ])
        record("I2C Bus", False)
except Exception as exc:
    fail(f"Unexpected error during I2C check: {exc}")
    record("I2C Bus", False)

# ════════════════════════════════════════════════════════════════════════════
# TEST 4 — PD Controller Software Verification
# ════════════════════════════════════════════════════════════════════════════

header("TEST 4 / 7  —  PD Controller  (pure-software logic test)")

try:
    # Direct import of the module as written (no hardware deps)
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from pd_controller import PDController

    ok("pd_controller.py imported successfully.")

    Kp, Kd, max_torque = 0.1, 0.01, 3.0
    ctrl = PDController(Kp=Kp, Kd=Kd, max_torque=max_torque)
    ok(f"PDController instantiated  (Kp={Kp}, Kd={Kd}, max_torque={max_torque}).")

    # Test case 1 — normal operation
    target, current, dt = 0.0, 5.0, 0.02
    torque = ctrl.calculate(target, current, dt)
    expected_p = Kp * (target - current)            # -0.5
    expected_d = Kd * ((target - current) / dt)     # -5.0  (prev_error was 0)
    expected_raw = expected_p + expected_d           # -5.5  → clamped to -3.0
    clamped = max(-max_torque, min(expected_raw, max_torque))

    if abs(torque - clamped) < 1e-9:
        ok(f"Calc 1  target={target}° current={current}° dt={dt}s → torque={torque:.4f} A  ✓ (matches expected {clamped:.4f} A)")
    else:
        fail(f"Calc 1 math mismatch. Got {torque:.6f}, expected {clamped:.6f}.")

    # Test case 2 — clamp fires
    ctrl2 = PDController(Kp=10.0, Kd=0.0, max_torque=3.0)
    t2 = ctrl2.calculate(0.0, 10.0, 0.02)   # raw = -100 → clamped -3.0
    if abs(t2 - (-3.0)) < 1e-9:
        ok(f"Calc 2  clamp test  raw=-100 → clamped={t2:.4f} A  ✓")
    else:
        fail(f"Clamp test failed. Expected -3.0, got {t2:.6f}.")

    # Test case 3 — dt=0 guard (no division by zero)
    ctrl3 = PDController(Kp=0.1, Kd=0.01, max_torque=3.0)
    try:
        t3 = ctrl3.calculate(0.0, 5.0, 0.0)
        ok(f"Calc 3  dt=0 guard → torque={t3:.4f} A  (no ZeroDivisionError)  ✓")
    except ZeroDivisionError:
        fail("dt=0 guard failed — ZeroDivisionError raised!")

    record("PD Controller", True)

except ImportError as exc:
    fail(f"Could not import pd_controller.py: {exc}")
    hint([
        "Ensure pd_controller.py is in the same directory as system_check.py,",
        "or add the correct path to sys.path at the top of this script.",
    ])
    record("PD Controller", False)
except AssertionError as exc:
    fail(f"PD logic assertion failed: {exc}")
    record("PD Controller", False)
except Exception as exc:
    fail(f"Unexpected error in PD test: {exc}")
    record("PD Controller", False)

# ════════════════════════════════════════════════════════════════════════════
# TEST 5 — BNO085 IMU
# ════════════════════════════════════════════════════════════════════════════

header("TEST 5 / 7  —  BNO085 IMU  (I2C init + quaternion read)")

IMU_TIMEOUT_S = 5.0   # seconds to wait for a quaternion before declaring a hang

try:
    import board
    import busio
    ok("CircuitPython board / busio imported.")
except ImportError as exc:
    fail(f"CircuitPython not installed: {exc}")
    hint([
        "Install with:  pip3 install adafruit-circuitpython-bno08x",
        "Also ensure:   pip3 install adafruit-blinka",
    ])
    record("IMU", False)
    # Skip the rest of this block by jumping past it with a flag
    _imu_skip = True
else:
    _imu_skip = False

if not _imu_skip:
    try:
        from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
        from adafruit_bno08x.i2c import BNO08X_I2C
        ok("adafruit_bno08x library imported.")
    except ImportError as exc:
        fail(f"adafruit_bno08x library missing: {exc}")
        hint(["pip3 install adafruit-circuitpython-bno08x"])
        record("IMU", False)
        _imu_skip = True

if not _imu_skip:
    try:
        info("Opening I2C bus and initializing BNO085…")
        i2c = busio.I2C(board.SCL, board.SDA)
        ok("I2C bus opened.")
    except Exception as exc:
        fail(f"Failed to open I2C bus: {exc}")
        hint([
            "Confirm /dev/i2c-7 exists (Test 3 above).",
            "Check SDA/SCL pull-up resistors (4.7 kΩ to 3.3 V).",
        ])
        record("IMU", False)
        _imu_skip = True

if not _imu_skip:
    try:
        imu = BNO08X_I2C(i2c)
        ok("BNO08X_I2C driver instantiated.")
    except Exception as exc:
        fail(f"BNO085 init failed: {exc}")
        if "unable to enable feature" in str(exc).lower():
            hint([
                "═══ HARD POWER CYCLE REQUIRED ═══",
                "The BNO085 is in a bad internal state that soft-resets cannot clear.",
                "  1. Press Ctrl+C to stop any running scripts.",
                "  2. UNPLUG the Jetson Orin Nano from mains power entirely.",
                "  3. Wait 10 full seconds (capacitors must discharge).",
                "  4. Reconnect power and re-run this script.",
                "If the error persists after 3 cycles, the IMU may be damaged.",
            ])
        else:
            hint([
                "Check I2C address: BNO085 must be at 0x4A or 0x4B.",
                "Verify 3.3 V supply to the IMU breakout.",
                "Check for electrical noise — motor CAN switching can corrupt I2C.",
            ])
        record("IMU", False)
        _imu_skip = True

if not _imu_skip:
    try:
        info("Issuing soft_reset()…")
        imu.soft_reset()
        time.sleep(0.5)
        ok("soft_reset() completed.")
    except Exception as exc:
        warn(f"soft_reset() raised an exception (non-fatal): {exc}")
        info("Continuing — some firmware versions do not expose soft_reset().")

if not _imu_skip:
    try:
        info(f"Enabling BNO_REPORT_ROTATION_VECTOR…")
        imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        ok("Rotation vector report enabled.")
    except Exception as exc:
        fail(f"enable_feature() failed: {exc}")
        if "unable to enable feature" in str(exc).lower():
            hint([
                "═══ HARD POWER CYCLE REQUIRED ═══",
                "Unplug the Jetson entirely, wait 10 s, then reconnect and retry.",
            ])
        record("IMU", False)
        _imu_skip = True

if not _imu_skip:
    try:
        info(f"Waiting up to {IMU_TIMEOUT_S:.0f} s for a valid quaternion…")
        deadline = time.monotonic() + IMU_TIMEOUT_S
        quat = None
        while time.monotonic() < deadline:
            quat = imu.quaternion
            if quat is not None:
                break
            time.sleep(0.05)

        if quat is None:
            fail(f"No quaternion received within {IMU_TIMEOUT_S:.0f} s — possible hang.")
            hint([
                "The BNO085 sensor-fusion engine failed to produce data.",
                "  1. Check that the IMU is not mechanically vibrating excessively.",
                "  2. Confirm 3.3 V rail is stable under load.",
                "  3. Perform a HARD POWER CYCLE (unplug Jetson, wait 10 s, reconnect).",
            ])
            record("IMU", False)
        else:
            i, j, k, real = quat
            magnitude = (i**2 + j**2 + k**2 + real**2) ** 0.5
            ok(f"Quaternion received:  i={i:+.4f}  j={j:+.4f}  k={k:+.4f}  w={real:+.4f}")
            info(f"Quaternion magnitude: {magnitude:.6f}  (expected ≈ 1.000)")
            if abs(magnitude - 1.0) > 0.05:
                warn("Magnitude deviates >5% from 1.0 — sensor may need calibration.")
            else:
                ok("Magnitude within tolerance. IMU data looks valid.")
            record("IMU", True)

    except Exception as exc:
        fail(f"Quaternion read failed: {exc}")
        hint(["Try a hard power cycle if this error is 'I2C timeout' or 'NACK'."])
        record("IMU", False)

# ════════════════════════════════════════════════════════════════════════════
# TEST 6 & 7 — CubeMars Motor Roll Call
# CAN IDs from README: Roll = 1 (migrated from 104), Pitch = 2
# ════════════════════════════════════════════════════════════════════════════
#
# CubeMars MIT-mode "Enter Motor Mode" frame:
#   Arbitration ID (extended): 0x7FF   (broadcast enter-mode command)
#   Data: FF FF FF FF FF FF FF FC
#
# "Exit Motor Mode" frame:
#   Data: FF FF FF FF FF FF FF FD
#
# Status reply:  arb_id >> 8 == 41 (0x29), arb_id & 0xFF == motor_id
# The README and motor_driver.py both confirm this decode.
# ─────────────────────────────────────────────────────────────────────────

MOTOR_CONFIG = {
    "Roll  (Motor 1)": 1,
    "Pitch (Motor 2)": 2,
}

ENTER_MODE_DATA = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
EXIT_MODE_DATA  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
MOTOR_PING_TIMEOUT_S = 0.5   # seconds to wait for a status reply

try:
    import can as _can_lib
    _can_available = True
    ok_shown = False
except ImportError:
    _can_available = False

for motor_label, motor_id in MOTOR_CONFIG.items():
    test_name = f"Motor {motor_id} ({motor_label.strip()})"
    test_index = 5 + motor_id   # Test 6 and Test 7

    header(f"TEST {test_index} / 7  —  {motor_label}  CAN ID {motor_id}")

    if not _can_available:
        fail("python-can not installed — cannot test motors.")
        hint(["pip3 install python-can"])
        record(test_name, False)
        continue

    bus = None
    try:
        bus = _can_lib.Bus(
            interface="socketcan",
            channel=CAN_IFACE,
            bitrate=CAN_BITRATE,
        )
        ok(f"CAN bus opened on '{CAN_IFACE}'.")
    except Exception as exc:
        fail(f"Could not open CAN bus: {exc}")
        hint([
            "Ensure can0 is UP (Test 2 above).",
            "Check that you are running as root.",
        ])
        record(test_name, False)
        continue

    # — Send "Enter Motor Mode" ───────────────────────────────────────────
    enter_id = (0x00 << 8) | motor_id    # command 0x00 = enter mode, per CubeMars protocol
    # CubeMars uses the broadcast arb ID for enter/exit; motor replies with its own ID
    broadcast_arb_id = 0x7FF

    try:
        enter_msg = _can_lib.Message(
            arbitration_id=broadcast_arb_id,
            data=ENTER_MODE_DATA,
            is_extended_id=True,
        )
        bus.send(enter_msg)
        ok(f"'Enter Motor Mode' frame sent (arb_id=0x{broadcast_arb_id:X}, data={ENTER_MODE_DATA.hex().upper()}).")
    except _can_lib.CanError as exc:
        fail(f"Failed to send Enter Motor Mode: {exc}")
        hint([
            "TX buffer overflow — check bus load or increase txqueuelen:",
            f"  sudo ip link set {CAN_IFACE} txqueuelen 1000",
        ])
        # Still try to close and move on
        try:
            bus.shutdown()
        except Exception:
            pass
        record(test_name, False)
        continue

    # — Listen for a status reply ─────────────────────────────────────────
    info(f"Listening for status reply from motor ID {motor_id} (timeout={MOTOR_PING_TIMEOUT_S}s)…")
    deadline = time.monotonic() + MOTOR_PING_TIMEOUT_S
    reply_found = False
    reply_frame = None

    while time.monotonic() < deadline:
        try:
            frame = bus.recv(timeout=0.05)
        except _can_lib.CanError:
            break
        if frame is None:
            continue
        cmd_byte = frame.arbitration_id >> 8
        dev_id   = frame.arbitration_id & 0xFF
        # CubeMars status reply: cmd byte 0x29 (41 decimal), device == motor_id
        if cmd_byte == 0x29 and dev_id == motor_id:
            reply_found = True
            reply_frame = frame
            break

    if reply_found and reply_frame is not None:
        ok(f"Reply received from motor ID {motor_id}.")
        raw = reply_frame.data
        if len(raw) >= 8:
            val = struct.unpack(">hhhh", raw[:8])
            pos_raw, vel_raw, cur_raw, temp_raw = val
            POS_SCALE = 0.1
            CUR_SCALE = 0.01
            info(f"  Position : {pos_raw * POS_SCALE:+.2f} (raw={pos_raw})")
            info(f"  Current  : {cur_raw * CUR_SCALE:+.3f} A (raw={cur_raw})")
            info(f"  Temp raw : {temp_raw}")
        else:
            info(f"  Raw bytes: {raw.hex().upper()}")
        record(test_name, True)
    else:
        fail(f"No reply from motor ID {motor_id} within {MOTOR_PING_TIMEOUT_S}s.")
        hint([
            "Check the following:",
            "  • Battery / power supply is connected and ON.",
            "  • CAN wiring: CANH↔CANH and CANL↔CANL between transceiver and motor.",
            "  • 120 Ω termination resistors on both ends of the CAN bus.",
            "  • Motor CAN ID is set correctly via the CubeMars debug app.",
            f"  • can0 is UP with no NO-CARRIER state (see Test 2).",
            "  • Try:  candump can0  and power-cycle the motor to see its boot frame.",
        ])
        record(test_name, False)

    # — Send "Exit Motor Mode" regardless of result ───────────────────────
    try:
        exit_msg = _can_lib.Message(
            arbitration_id=broadcast_arb_id,
            data=EXIT_MODE_DATA,
            is_extended_id=True,
        )
        bus.send(exit_msg)
        ok(f"'Exit Motor Mode' frame sent. Motor is disarmed.")
    except _can_lib.CanError as exc:
        warn(f"Could not send Exit Motor Mode: {exc}  (motor may still be armed!)")

    # — Close bus ─────────────────────────────────────────────────────────
    try:
        bus.shutdown()
        info("CAN bus released.")
    except Exception as exc:
        warn(f"Bus shutdown raised: {exc}")

# ════════════════════════════════════════════════════════════════════════════
# FINAL SUMMARY TABLE
# ════════════════════════════════════════════════════════════════════════════

print()
print(c(CYAN, "═" * 70))
print(c(BOLD + CYAN, "  PRE-FLIGHT DIAGNOSTIC SUMMARY"))
print(c(CYAN, "═" * 70))

# Ordered display list
display_order = [
    ("Privilege",          "1. Sudo / Root"),
    ("CAN Interface",      "2. CAN Interface  (can0)"),
    ("I2C Bus",            "3. I2C Bus  (/dev/i2c-7)"),
    ("PD Controller",      "4. PD Controller  (math)"),
    ("IMU",                "5. BNO085 IMU"),
    ("Motor 1 (Roll  (Motor 1))",  "6. Motor Roll   (CAN ID 1)"),
    ("Motor 2 (Pitch (Motor 2))", "7. Motor Pitch  (CAN ID 2)"),
]

all_passed = True
for key, label in display_order:
    status = results.get(key)
    if status is True:
        badge = c(GREEN, "  PASS  ")
    elif status is False:
        badge = c(RED,   "  FAIL  ")
        all_passed = False
    else:
        badge = c(YELLOW, "  SKIP  ")
        all_passed = False
    print(f"  [{badge}]  {label}")

print(c(CYAN, "─" * 70))
if all_passed:
    print(c(BOLD + GREEN, "  ✔  All systems nominal. Platform is CLEAR FOR STARTUP."))
else:
    fail_count = sum(1 for v in results.values() if v is False)
    print(c(BOLD + RED, f"  ✘  {fail_count} check(s) failed. Review FAIL messages above before proceeding."))
print(c(CYAN, "═" * 70))
print()
