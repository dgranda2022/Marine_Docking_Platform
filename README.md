# feature/serial-uart-arch

## Marine Docking Platform — UART Serial Architecture

This branch migrates the motor interface from CAN bus to **USB-to-UART adapters** running the native CubeMars serial protocol. The IMU, PD controller, and 50 Hz orchestrator loop are unchanged. Only the motor HAL and hardware setup differ.

If you are pulling this branch for the first time, read the full setup procedure below before powering on the motors.

---

## What Changed From the CAN Branch

### Motor Interface: CAN → UART

The original architecture used SocketCAN (`motor_driver.py`) with a Waveshare SN65HVD230 transceiver on the J17 header. This branch replaces that entirely with USB-to-UART adapters (CP210x chipset) plugged into the Jetson's USB-A ports. Each motor gets its own adapter.

**Why the change:** The CubeMars AK60-39 V3.0 motors on this build have only the UART JST port accessible — the CAN lines are hardwired into the power connector and unavailable. UART is the only viable interface.

### Protocol: VESC → CubeMars Native

The previous `serial_motor_driver.py` used **VESC serial framing** (`0x02`/`0x03` headers, VESC command IDs). This was wrong for these motors. After reading the AK series manual v3.2.0 and brute-force probing the motor, the correct protocol was identified and verified:

| Field | Value |
|---|---|
| Frame format | `[0xAA][Len][Cmd][Data...][CRC16_H][CRC16_L][0xBB]` |
| Length byte | Counts cmd byte + data bytes only |
| CRC algorithm | CRC16-CCITT, poly `0x1021`, init `0x0000`, MSB-first, no reflection |
| CRC covers | `[Cmd]` + `[Data]` bytes only |
| Baud rate | 921600, 8N1 |
| COMM_GET_VALUES | `0x45` — request full motor telemetry |
| COMM_SET_CURRENT | `0x47` — set phase current, data = `int32` (amps × 1000, big-endian) |

Known-good frames verified against manual page 57–58:
- `AA 01 45 18 61 BB` — get motor state (no data)
- `AA 05 47 00 00 13 88 30 1C BB` — set current to 5 A

### Motor Direction Sign

During hardware testing the pitch motor was confirmed to have an **inverted torque direction** relative to the IMU convention: a positive current command reduces pitch angle. `main.py` negates the pitch command output (`-cmd_pitch`) to correct for this.

### IMU I2C Bus Fix

The BNO085 IMU was found on **I2C bus 7** (address `0x4A`), confirmed via `i2cdetect`. The original code used `busio.I2C(board.SCL, board.SDA)` which defaulted to the wrong bus and returned all-zeros. `imu_sensor.py` now uses `adafruit_extended_bus.ExtendedI2C(7)` to target bus 7 directly. Auto-reinit after 5 consecutive read failures was also added to recover from I2C bus glitches.

---

## Software Architecture

### `main.py` — Orchestrator

Runs a **50 Hz control loop**. Reads IMU, computes PD commands for roll and pitch, sends torque commands to both motors, requests telemetry, and reads feedback. On startup it prints a warning (not a fatal error) if either motor fails to connect, so a single-motor configuration works without code changes.

Key constants to adjust:
```python
PORT_ROLL  = "/dev/ttyUSB1"   # roll motor adapter
PORT_PITCH = "/dev/ttyUSB0"   # pitch motor adapter (confirmed)
Kp = 0.1                       # proportional gain
Kd = 0.01                      # derivative gain
max_torque = 3.0               # current clamp (amps)
```

### `serial_motor_driver.py` — UART Motor HAL

One instance per motor. Public interface identical to the old CAN driver:

| Method | Description |
|---|---|
| `connect()` | Open the serial port at 921600 8N1 |
| `arm()` | Send 10× zero-current frames to wake controller |
| `send_torque(amps)` | Send `COMM_SET_CURRENT` frame |
| `request_telemetry()` | Send `COMM_GET_VALUES` request |
| `get_state()` | Drain RX buffer, parse latest telemetry reply |
| `stop()` | Zero torque and close port |

`get_state()` maintains a persistent receive buffer so partial frames across calls are not lost.

### `imu_sensor.py` — IMU HAL

BNO085 over I2C bus 7. Returns `{"roll": float, "pitch": float, "yaw": float}` in degrees. Falls back to last good reading on I2C errors; reinitialises the sensor after 5 consecutive failures.

### `pd_controller.py` — PD Math

Unchanged from CAN branch. Stateless except for `previous_error`. Clamps output to `±max_torque`. Suppresses derivative term when `dt ≤ 0`.

---

## Diagnostic and Utility Scripts

These scripts are for hardware bring-up and are not needed during normal operation.

| Script | Purpose |
|---|---|
| `jetson_io_check.py` | Full diagnostic: scans USB-UART ports, native UARTs, I2C buses, and attempts a BNO085 read |
| `cubemars_probe.py` | Sends correct CubeMars frames and prints any motor response — use to verify a motor is alive before running main |
| `motor_probe.py` | Brute-force baud-rate × protocol scanner (VESC and MIT-mode) — kept for reference |
| `motor_test.py` | IMU-guided motor movement tests: `recover` mode returns pitch to 0°, `sweep` mode does a controlled ±N° sweep |

### Checking if the motor is responding

```bash
python3 cubemars_probe.py /dev/ttyUSB0
```

Expected output when motor is live:
```
[ HIT! ] COMM_GET_VALUES 0x45  (get motor state)
         90 bytes: AA 55 45 ...
```

### Returning the motor to 0° after a runaway

```bash
python3 motor_test.py recover
```

---

## Hardware Setup

### Wiring

| Connection | Details |
|---|---|
| Pitch motor | JST UART port → CP210x USB adapter → Jetson `/dev/ttyUSB0` |
| Roll motor | JST UART port → CP210x USB adapter → Jetson `/dev/ttyUSB1` |
| IMU (BNO085) | SDA/SCL/3.3V/GND → Jetson I2C bus 7, address `0x4A` |
| Motor power | 22 V supply (confirmed via telemetry Vin = 22.1 V) |

**Ground:** The UART adapter GND and motor GND must share a common reference. Floating ground will cause no response or garbage frames.

### USB Port Enumeration

`/dev/ttyUSBx` numbers are assigned by the kernel in plug-in order. If you unplug and replug in a different order, the numbers may change. After replugging, verify with:

```bash
ls /dev/ttyUSB*
# or
dmesg | grep ttyUSB | tail -5
```

Update `PORT_PITCH` and `PORT_ROLL` in `main.py` if the numbers changed.

### SSH into the Jetson

```bash
ssh edg5@192.168.55.1
```

### Sharing Internet to the Jetson

**On the laptop:**
```bash
export WIFI_IF=wlo1
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -s 192.168.55.0/24 -o $WIFI_IF -j MASQUERADE
sudo iptables -A FORWARD -i $WIFI_IF -o l4tbr0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i l4tbr0 -o $WIFI_IF -j ACCEPT
```

**On the Jetson:**
```bash
sudo route add default gw 192.168.55.100
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf > /dev/null
```

These rules do not survive a reboot — re-run after every power cycle.

---

## Hardware Parameter Map

| Parameter | Value |
|---|---|
| Pitch motor port | `/dev/ttyUSB0` (confirmed) |
| Roll motor port | `/dev/ttyUSB1` (not yet connected) |
| Baud rate | 921,600 bps, 8N1 |
| Control loop frequency | 50 Hz (20 ms period) |
| Default Kp | 0.1 |
| Default Kd | 0.01 |
| Max torque clamp | ±3.0 A |
| IMU | BNO085, I2C bus 7, address `0x4A` |
| Motor supply voltage | 22.1 V (measured) |
| Pitch motor direction | **Inverted** — positive current reduces pitch angle |

---

## Quick Start

```bash
cd ~/capstone_project/Marine_Docking_Platform
python3 main.py
```

The terminal will show a single updating line:
```
t=0000.02s | Roll:  -0.17° cmd:+0.017A | Pitch:  -0.01° cmd:+0.001A | dt: 20.1ms
```

Press `Ctrl+C` to stop. The shutdown handler zeros both motors and closes the serial ports automatically.

If the pitch motor is not at 0° before starting, run the recovery script first:
```bash
python3 motor_test.py recover
python3 main.py
```

---

## Known Issues — Mechanical Slip (Pending Repair)

**Both motors currently have slip in the motor-to-platform connection.** This is the root cause of instability observed during closed-loop testing and must be resolved before any meaningful gain tuning can be done.

**How slip affects each axis differently:**

- **Pitch axis** — Slip causes the platform to lurch unpredictably rather than move smoothly in response to motor commands. This appeared in data collection as 2 Hz oscillation with ±3° amplitude and 23% command saturation (hitting the 3A clamp), but these are symptoms of the mechanical problem, not a tuning issue.

- **Roll axis** — Carries the full weight of the pitch axis in addition to its own load, making slip significantly worse and the effective load much harder to model. Roll will likely need higher gains and possibly a different control strategy (e.g. feedforward gravity compensation) compared to pitch once repaired.

**What needs to happen before tuning:**
1. Repair the motor-platform connection on both axes to eliminate slip
2. Re-run data collection on pitch (`python3 main.py`, Ctrl+C after ~45s, inspect `pitch_data.csv`)
3. Tune pitch gains first (lighter, more consistent load)
4. Tune roll separately — expect it to need different `Kp`/`Kd` values and possibly a gravity feedforward term given the extra load

---

## Project Roadmap

### Phase 1 — Gravity Compensation (Feedforward Term)

The platform currently has a small steady-state offset (~2–3°) because the PD controller is reactive — it only produces torque after an error already exists. A feedforward term of the form:

$$\tau_{ff} = K_g \cdot \cos(\theta)$$

proactively cancels the gravitational torque at any angle, letting the PD terms focus purely on dynamic stabilization. $K_g$ will be determined experimentally by increasing it until the platform holds level with PD gains zeroed.

### Phase 2 — Live Tuner (Terminal UI)

A `curses`-based dashboard running in a separate thread alongside the 50 Hz loop, allowing `Kp`, `Kd`, and `Kg` to be nudged live with keyboard bindings. Changes take effect on the next 20 ms cycle — no restart, no re-arm.

### Phase 3 — "Clear to Land" Stability Monitor

A sliding-window variance check over the last 1 second of error data (50 samples). If variance stays below 2° for 5 continuous seconds, a CLEAR TO LAND flag is asserted — eventually on a GPIO pin readable by the drone's flight controller.

### Phase 4 — Roll Motor Integration

Wire the second CP210x adapter to `/dev/ttyUSB1`, confirm direction sign, and verify both axes stabilize simultaneously. The software already supports both motors; only the physical wiring remains.
