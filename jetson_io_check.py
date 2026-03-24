#!/usr/bin/env python3
"""
jetson_io_check.py — Jetson Orin Nano UART + I2C diagnostic script
Tests: USB-to-UART adapters, native Jetson UARTs, I2C bus scan, BNO085 IMU
"""

import subprocess
import sys
import glob
import time

SEP = "-" * 60

def tag(label, msg):
    print(f"[ {label:<4} ] {msg}")

def run_cmd(cmd, shell=True):
    try:
        result = subprocess.run(
            cmd, shell=shell, capture_output=True, text=True, timeout=10
        )
        return result.stdout.strip(), result.stderr.strip(), result.returncode
    except subprocess.TimeoutExpired:
        return "", "TIMEOUT", -1
    except Exception as e:
        return "", str(e), -1


# ─────────────────────────────────────────────────────────
print(SEP)
print("TEST 1 — USB-to-UART Adapter Detection")
print(SEP)

usb_devices = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))

if usb_devices:
    for dev in usb_devices:
        tag("INFO", f"Found USB serial device: {dev}")
else:
    tag("INFO", "No /dev/ttyUSB* or /dev/ttyACM* devices found")

# Try to open each device and write 3 bytes
import serial

for dev in usb_devices:
    try:
        with serial.Serial(dev, baudrate=921600, timeout=1) as s:
            s.write(b'\xAA\x55\xFF')
            tag("PASS", f"{dev} opened at 921600, TX 3 bytes OK")
    except PermissionError:
        tag("FAIL", f"{dev} — Permission denied. Try: sudo usermod -aG dialout edg5")
    except serial.SerialException as e:
        tag("FAIL", f"{dev} — SerialException: {e}")
    except Exception as e:
        tag("FAIL", f"{dev} — {type(e).__name__}: {e}")

# dmesg tty
print()
tag("INFO", "dmesg | grep -i tty (last 20 lines):")
stdout, stderr, rc = run_cmd("dmesg | grep -i tty | tail -20")
if stdout:
    for line in stdout.splitlines():
        print(f"         {line}")
else:
    tag("INFO", "(no tty lines in dmesg or permission denied)")

# dmesg usb
print()
tag("INFO", "dmesg | grep -i usb (last 20 lines):")
stdout, stderr, rc = run_cmd("dmesg | grep -i usb | tail -20")
if stdout:
    for line in stdout.splitlines():
        print(f"         {line}")
else:
    tag("INFO", "(no usb lines in dmesg or permission denied)")


# ─────────────────────────────────────────────────────────
print()
print(SEP)
print("TEST 2 — Jetson Native UART (ttyTHS*)")
print(SEP)

native_uarts = sorted(glob.glob("/dev/ttyTHS*"))

if not native_uarts:
    tag("INFO", "No /dev/ttyTHS* devices found (may need device-tree overlay or different kernel config)")
else:
    for dev in native_uarts:
        tag("INFO", f"Found native UART: {dev}")
        try:
            with serial.Serial(dev, baudrate=921600, timeout=1) as s:
                s.write(b'\xAA\x55\xFF')
                tag("PASS", f"{dev} opened at 921600, TX 3 bytes OK")
        except PermissionError:
            tag("FAIL", f"{dev} — Permission denied. Try: sudo usermod -aG dialout edg5")
        except serial.SerialException as e:
            tag("FAIL", f"{dev} — SerialException: {e}")
        except Exception as e:
            tag("FAIL", f"{dev} — {type(e).__name__}: {e}")


# ─────────────────────────────────────────────────────────
print()
print(SEP)
print("TEST 3 — I2C Bus Scan")
print(SEP)

i2c_buses = sorted(glob.glob("/dev/i2c-*"))
bno085_bus = None  # will be set if 0x4A or 0x4B is found

if not i2c_buses:
    tag("INFO", "No /dev/i2c-* devices found")
else:
    for bus_dev in i2c_buses:
        bus_num = bus_dev.split("-")[-1]
        tag("INFO", f"Scanning {bus_dev} with i2cdetect -y -r {bus_num}:")
        stdout, stderr, rc = run_cmd(f"i2cdetect -y -r {bus_num}")
        if rc != 0:
            tag("FAIL", f"i2cdetect failed on bus {bus_num}: {stderr}")
        else:
            for line in stdout.splitlines():
                print(f"         {line}")
            # Flag BNO085 addresses
            if " 4a" in stdout.lower() or "\t4a" in stdout.lower() or stdout.lower().count("4a") > 0:
                # confirm it's a real device entry not just UU
                lines = stdout.lower().splitlines()
                for l in lines:
                    if "4a" in l and "uu" not in l.split("4a")[0].split()[-1:]:
                        tag("PASS", f"Bus {bus_num}: Address 0x4A detected — BNO085 IMU found!")
                        bno085_bus = int(bus_num)
                        break
            if " 4b" in stdout.lower():
                tag("PASS", f"Bus {bus_num}: Address 0x4B detected — BNO085 IMU found!")
                if bno085_bus is None:
                    bno085_bus = int(bus_num)
        print()


# ─────────────────────────────────────────────────────────
print(SEP)
print("TEST 4 — BNO085 IMU Read Attempt")
print(SEP)

try:
    import board
    import busio
    tag("PASS", "Imported board and busio (CircuitPython)")
except ImportError as e:
    tag("FAIL", f"Missing CircuitPython library: {e}")
    tag("INFO", "Install with: pip3 install adafruit-circuitpython-bno08x")
    board = None

try:
    import adafruit_bno08x
    from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
    tag("PASS", "Imported adafruit_bno08x")
except ImportError as e:
    tag("FAIL", f"Missing adafruit_bno08x: {e}")
    tag("INFO", "Install with: pip3 install adafruit-circuitpython-bno08x")
    adafruit_bno08x = None

if board and adafruit_bno08x:
    if bno085_bus is None:
        tag("FAIL", "BNO085 not found on any I2C bus — skipping read. Check wiring.")
    else:
        tag("INFO", f"BNO085 detected on bus {bno085_bus}, attempting init...")
        try:
            from adafruit_extended_bus import ExtendedI2C as ExtI2C
            i2c = ExtI2C(bno085_bus)
            tag("PASS", f"Opened I2C bus {bno085_bus} via ExtendedI2C")
        except ImportError:
            tag("INFO", "adafruit_extended_bus not found, falling back to busio.I2C (may use wrong bus)")
            import busio
            i2c = busio.I2C(board.SCL, board.SDA)
        try:
            from adafruit_bno08x.i2c import BNO08X_I2C
            bno = BNO08X_I2C(i2c)
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            tag("PASS", "BNO085 initialized, waiting for rotation vector (up to 5s)...")
            deadline = time.time() + 5.0
            quaternion = None
            while time.time() < deadline:
                try:
                    quaternion = bno.quaternion
                    if quaternion and any(v != 0.0 for v in quaternion):
                        break
                    quaternion = None
                except Exception:
                    pass
                time.sleep(0.05)
            if quaternion:
                qi, qj, qk, real = quaternion
                tag("PASS", f"Quaternion read OK: i={qi:.4f}, j={qj:.4f}, k={qk:.4f}, real={real:.4f}")
            else:
                tag("FAIL", "IMU initialized but quaternion stayed all-zero for 5s — sensor may need reset")
        except Exception as e:
            tag("FAIL", f"BNO085 init/read failed: {type(e).__name__}: {e}")
else:
    tag("INFO", "Skipping IMU read — required libraries not available")


# ─────────────────────────────────────────────────────────
print()
print(SEP)
print("TEST 5 — USB Adapter Details (lsusb + udevadm)")
print(SEP)

stdout, stderr, rc = run_cmd("lsusb")
if rc == 0 and stdout:
    tag("INFO", "lsusb output:")
    for line in stdout.splitlines():
        print(f"         {line}")
else:
    tag("FAIL", f"lsusb failed: {stderr}")

for dev in ["/dev/ttyUSB0", "/dev/ttyUSB1"]:
    devs = glob.glob(dev)
    if devs:
        print()
        tag("INFO", f"udevadm info for {dev}:")
        stdout, stderr, rc = run_cmd(
            f"udevadm info --name={dev} --attribute-walk 2>/dev/null | head -30"
        )
        if stdout:
            for line in stdout.splitlines():
                print(f"         {line}")
        else:
            tag("INFO", f"No udevadm output for {dev}")

print()
print(SEP)
print("Diagnostic complete.")
print(SEP)
