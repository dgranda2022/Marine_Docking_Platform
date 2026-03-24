# Marine Docking Platform

Active stabilisation platform for drone landing at sea. Two CubeMars AK60-39 V3.0 actuators (pitch and roll axes) are driven by a Jetson running a 50 Hz software PD control loop over USB-to-UART serial. A BNO085 IMU provides angle feedback.

---

## Quick Start

```bash
cd ~/capstone_project/Marine_Docking_Platform
python3 motor_test.py recover     # return pitch to 0° if needed
python3 main.py                   # start closed-loop stabilisation
```

Terminal output while running:
```
t=0000.02s | Roll:  -0.17° cmd:+0.017A | Pitch:  -0.01° cmd:+0.001A | dt: 20.1ms
```

Press `Ctrl+C` to stop. The shutdown handler zeros both motors and closes serial ports automatically. Data is saved to `pitch_data.csv`.

---

## Hardware Setup

### Wiring

| Connection | Details |
|---|---|
| Pitch motor | JST UART port → CP210x USB adapter → Jetson `/dev/ttyUSB0` |
| Roll motor | JST UART port → CP210x USB adapter → Jetson `/dev/ttyUSB1` |
| IMU (BNO085) | SDA/SCL/3.3V/GND → Jetson I2C bus 7, address `0x4A` |
| Motor power | 22 V supply (confirmed via telemetry Vin = 22.1 V) |

**Ground:** The UART adapter GND and motor GND must share a common reference. Floating ground causes no response or garbage frames.

### USB Port Enumeration

`/dev/ttyUSBx` numbers are assigned by the kernel in plug-in order. If you unplug and replug in a different order, the numbers may shift. After replugging, verify with:

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

## Software Architecture

### `main.py` — Control Loop

Runs a **50 Hz PD control loop**. Reads IMU angles, computes current commands for both axes, sends them to the motors, and logs data to `pitch_data.csv`. Starts cleanly with one or both motors connected.

**Control mode:** `COMM_SET_CURRENT (0x47)` — the motor applies whatever current it is commanded. All control logic (the PD loop) runs in Python on the Jetson. The motor is dumb torque actuation; the Jetson is the controller.

Key parameters in the `State["Tuning"]` dict:

```python
"roll_Kp":    0.20,   # A/deg — confirmed stable, 0.44 Hz, no oscillation
"roll_Kd":    0.030,  # A/(deg/s)
"pitch_Kp":   0.10,   # A/deg — pitched back from 0.20 which oscillated at 2.14 Hz
"pitch_Kd":   0.025,  # A/(deg/s)
"max_torque": 3.0,    # A — current clamp applied to both axes
```

**Motor direction:** Pitch motor is electrically inverted — positive current reduces pitch angle. `main.py` negates the pitch command (`-cmd_pitch`) to correct for this. Roll is not inverted.

### `serial_motor_driver.py` — UART Motor HAL

One instance per motor. Implements the CubeMars native serial protocol (AK series manual v3.2.0):

| Method | Description |
|---|---|
| `connect()` | Open serial port at 921600 8N1 |
| `arm()` | Send 10× zero-current frames to wake controller |
| `send_torque(amps)` | Send `COMM_SET_CURRENT` (0x47) frame |
| `request_telemetry()` | Send `COMM_GET_VALUES` (0x45) request |
| `get_state()` | Drain RX buffer, return latest telemetry as `{pos, torque}` |
| `stop()` | Zero current and close port |

**Frame format:** `[0xAA][Len][Cmd][Data...][CRC16_H][CRC16_L][0xBB]`
- Length byte counts cmd + data bytes only
- CRC16-CCITT: poly `0x1021`, init `0x0000`, MSB-first, no reflection
- Baud rate: 921600, 8N1

Known-good frames (verified against manual p.57–58):
- `AA 01 45 18 61 BB` — request motor state
- `AA 05 47 00 00 13 88 30 1C BB` — set 5 A

### `imu_sensor.py` — IMU HAL

BNO085 over I2C bus 7 (address `0x4A`). Returns `{"roll": float, "pitch": float, "yaw": float}` in degrees. Falls back to last good reading on I2C errors; reinitialises sensor after 5 consecutive failures.

**Bus note:** The BNO085 is on I2C bus 7, confirmed via `i2cdetect`. Using `adafruit_extended_bus.ExtendedI2C(7)` directly — the default `busio.I2C` targets the wrong bus and returns all-zeros.

### `pd_controller.py` — PD Math

Stateless except for `previous_error`. Output = `Kp * error + Kd * (error - prev_error) / dt`. Clamps to `±max_torque`. Suppresses derivative term when `dt ≤ 0`.

### `motor_test.py` — Pre-Run Utility

IMU-guided motor movement for aligning pitch before a run.

```bash
python3 motor_test.py recover        # drive pitch back to 0° using IMU feedback
python3 motor_test.py sweep          # controlled ±10° sweep to verify motor response
python3 motor_test.py sweep 15       # sweep ±15°
```

**Recover mode:** Applies RPM commands until IMU pitch < 1.5°. Direction confirmed: positive RPM reduces pitch angle. Slows to 80 RPM within 8° of target to avoid overshoot. Hard time limit of 15 s.

### `system_check_uart.py` — Pre-Flight Diagnostics

Checks USB-UART ports, motor response, IMU, and I2C buses before a run. Run this if something seems wrong:

```bash
python3 system_check_uart.py
```

---

## Data Collection

Every `main.py` run writes `pitch_data.csv` in the working directory (overwritten each run — copy it if you want to keep it).

### CSV Columns

| Column | Unit | Description |
|---|---|---|
| `t_s` | seconds | Elapsed time from loop start |
| `roll_deg` | degrees | IMU roll angle (positive = right side down) |
| `cmd_roll_A` | amps | Current commanded to roll motor |
| `pitch_deg` | degrees | IMU pitch angle (positive = nose up) |
| `cmd_pitch_A` | amps | Current commanded to pitch motor (after negation) |
| `dt_ms` | milliseconds | Actual loop iteration time (target: 20 ms) |

### Analysing a Run

**Check for oscillation:**
```python
import csv, math
rows = list(csv.DictReader(open('pitch_data.csv')))
pitch = [float(r['pitch_deg']) for r in rows]
crossings = [i for i in range(1, len(pitch)) if pitch[i-1]*pitch[i] < 0]
# Oscillation frequency = 1 / average half-period pair
```

**Check command saturation** (high saturation = gains too aggressive or mechanical slip):
```python
cmd = [float(r['cmd_pitch_A']) for r in rows]
sat_pct = 100 * sum(1 for c in cmd if abs(c) >= 2.95) / len(cmd)
print(f"Saturation: {sat_pct:.1f}%")   # target < 5%
```

**Check loop timing** (should stay close to 20 ms):
```python
dt = [float(r['dt_ms']) for r in rows]
print(f"dt mean: {sum(dt)/len(dt):.1f} ms  max: {max(dt):.1f} ms")
```

**Check steady-state offset** (non-zero mean = gravity bias, needs feedforward):
```python
print(f"Pitch mean: {sum(pitch)/len(pitch):+.2f}°")
```

### Gain Tuning Reference

The axes were tuned independently. Key observations from logged data:

| Condition | Symptom in log | Fix |
|---|---|---|
| Kp too high | Rapid zero-crossings, high saturation % | Reduce Kp |
| Kd too low | Oscillation grows over time (underdamped) | Raise Kd |
| Kd too high | Large dt spikes, jerky cmd trace | Reduce Kd |
| Gravity offset | Non-zero pitch mean, asymmetric cmd | Add feedforward term |

Confirmed working values and their history:

| Axis | Kp | Kd | Notes |
|---|---|---|---|
| Roll | 0.20 | 0.030 | Stable, 0.44 Hz response, no oscillation |
| Pitch | 0.10 | 0.025 | Kp=0.20 caused 2.14 Hz oscillation; stepped back |

---

## Communication Protocol Reference

### CubeMars Serial Protocol (AK Series v3.2.0)

The motors respond to the following serial commands over 921600 8N1 UART:

| Command | ID | Data | Description |
|---|---|---|---|
| `COMM_GET_VALUES` | `0x45` | none | Request full telemetry |
| `COMM_SET_CURRENT` | `0x47` | int32 = amps × 1000 | Set phase current (torque mode) |
| `COMM_SET_RPM` | `0x49` | int32 = ERPM | Velocity loop |
| `COMM_SET_POS` | `0x4A` | int32 = deg × 1,000,000 | Position loop (motor internal PID) |

**Note on position/MIT modes:** `COMM_SET_POS` (0x4A) is active on this firmware but the motor's internal position PID gains are at factory defaults (very low). MIT Force Control (0x60) does not respond over serial on this firmware version — it likely requires CAN or a firmware update. For reliable position hold, use the software PD loop in `main.py`.

### Telemetry Response Layout (`COMM_GET_VALUES`)

Response data offsets (after cmd byte stripped):

| Offset | Type | Scale | Field |
|---|---|---|---|
| 0–1 | int16 | ÷10 | MOS temperature (°C) |
| 2–3 | int16 | ÷10 | Motor temperature (°C) |
| 4–7 | int32 | ÷100 | Output current (A) |
| 8–11 | int32 | ÷100 | Input current (A) |
| 20–21 | int16 | ÷1000 | Duty cycle |
| 22–25 | int32 | raw | Motor speed (ERPM) |
| 26–27 | int16 | ÷10 | Input voltage (V) |

---

## Hardware Parameter Map

| Parameter | Value |
|---|---|
| Pitch motor port | `/dev/ttyUSB0` (confirmed) |
| Roll motor port | `/dev/ttyUSB1` (confirmed) |
| Baud rate | 921,600 bps, 8N1 |
| Control loop frequency | 50 Hz (20 ms period) |
| Roll Kp / Kd | 0.20 / 0.030 A/deg |
| Pitch Kp / Kd | 0.10 / 0.025 A/deg |
| Max torque clamp | ±3.0 A |
| IMU | BNO085, I2C bus 7, address `0x4A` |
| Motor supply voltage | 22.1 V (measured) |
| Pitch motor direction | **Inverted** — positive current reduces pitch angle |
| Roll motor direction | Normal — positive current increases roll angle |
| Motor torque coefficient | 3.4616 N·m/A (AK60-39 spec) |

---

## Known Issues

### Mechanical Slip (Pending Repair)

Both motors have some slip in the motor-to-platform coupling. Slip introduces a dead zone and stick-slip dynamics that can cause low-frequency oscillation independent of gain settings. Symptoms:

- Platform lurches rather than moving smoothly
- Oscillation that does not respond predictably to Kd changes
- Non-zero steady-state offset even with correct gains

**Fix:** Tighten or re-secure the motor coupling on both axes before final gain tuning.

### Gravity Offset (~1–3° Steady-State Error)

The PD controller is reactive — it only produces torque after an error exists. At rest, gravity pulls the platform slightly off level and the P term produces just enough torque to balance it, leaving a permanent small offset. This is not a tuning problem; it requires a feedforward term (see Roadmap Phase 1).

---

## Roadmap

### Phase 1 — Gravity Compensation (Feedforward Term)

Add a feedforward term to cancel gravitational torque at any angle:

$$\tau_{ff} = K_g \cdot \sin(\theta)$$

$K_g$ is determined experimentally: zero the PD gains, then increase $K_g$ until the platform holds level. Once found, restore PD gains — they then handle only dynamic disturbances, not gravity.

### Phase 2 — Live Tuner (Terminal UI)

A `curses`-based dashboard in a separate thread alongside the 50 Hz loop. Keyboard bindings nudge `Kp`, `Kd`, and `Kg` live — changes take effect on the next 20 ms cycle without restarting.

### Phase 3 — "Clear to Land" Stability Monitor

A sliding-window variance check over the last 1 second of IMU data (50 samples). If variance on both axes stays below a threshold for 5 continuous seconds, assert a CLEAR TO LAND flag — eventually driven to a GPIO pin readable by the drone's flight controller.

### Phase 4 — PID Upgrade

Replace the PD controller with a full PID. The integral term eliminates the gravity-induced steady-state offset without needing a manually-tuned feedforward $K_g$, and handles any slow drift from unmodelled disturbances.
