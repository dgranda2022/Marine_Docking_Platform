# feature/modular-pd-arch

## Marine Docking Platform — Modular PD Control Architecture

This branch replaces the previous single-script test approach with a modular, contract-based Hardware Abstraction Layer (HAL) architecture for the Jetson Orin Nano stabilization system. Each module owns exactly one responsibility and communicates through a centralized State Dictionary managed by a single orchestrator loop.

If you are pulling this branch for the first time, read the full setup procedure below before powering on the motors.

---

## Software Architecture

The codebase is organized around a strict principle: **separation of concerns**. No module knows how any other module works internally. They exchange data exclusively through a centralized, JSON-style State Dictionary that the orchestrator reads and writes on every loop iteration.

There are four files, each with a single job.

### `main.py` — The Orchestrator

This is the only file that imports the other three. It runs a **50 Hz control loop** (`dt = 0.02s`) and is responsible for the following:

- Initializing all hardware (IMU, both motors) and verifying connections at startup.
- Computing a precise time delta (`dt`) on every iteration using `time.perf_counter()`. This compensates for Linux OS scheduling jitter, which on a non-RT kernel can cause individual loop periods to drift by several milliseconds.
- Reading sensor data, passing it to the controllers, sending commands to the motors, reading motor feedback, and updating the State Dictionary — in that exact order, every cycle.
- Printing a single overwriting status line to the terminal for real-time monitoring.
- Catching `KeyboardInterrupt` and guaranteeing that both motors receive a zero-torque command and the CAN bus is released in the `finally` block.

The orchestrator does not contain any math, any byte packing, or any I2C transactions. It only moves data between modules.

### `pd_controller.py` — Module A (Pure Math)

A stateless-except-for-derivative PD controller class. It accepts a target angle, a measured angle, and a `dt`, then returns a clamped torque command.

Key characteristics:

- **State retention**: The class stores `previous_error` internally so it can compute the derivative term `(error - previous_error) / dt` across consecutive calls.
- **Safety clamping**: The output is hard-limited to `±max_torque` (default 3.0 Amps). This is the last line of defense before current hits the motors.
- **Zero-dt guard**: If `dt` is zero or negative (which can happen on the first iteration or under extreme scheduling anomalies), the derivative term is suppressed entirely rather than producing a division-by-zero fault.
- **No hardware imports**: This file imports nothing beyond Python builtins. It can be unit-tested on any laptop without a Jetson.

### `motor_driver.py` — Module B (CAN Bus HAL)

The SocketCAN interface to the CubeMars actuators. Each instance manages one motor.

Key characteristics:

- **Byte-level protocol**: Torque commands are scaled to milliamps and packed as 32-bit big-endian signed integers (`>i`). Feedback frames carry four signed 16-bit integers (`>hhhh`) encoding position, velocity, current, and temperature.
- **Buffer draining**: The `get_state()` method uses a `while True` loop with `recv(timeout=0)` to aggressively drain the Linux kernel's CAN receive buffer. This ensures the returned feedback is always the most recent frame, not a stale one sitting in the queue. Without this pattern, feedback latency accumulates and the control loop operates on outdated data.
- **Arming sequence**: CubeMars motors require a wake-up handshake — ten consecutive zero-current commands at 10 ms intervals — before they accept real torque commands.
- **Graceful error handling**: `can.CanError` exceptions on send (typically caused by a full transmit buffer) are caught and suppressed so a single dropped frame does not crash the loop.

> **⚠️ Critical Note:** CubeMars motors require **Extended CAN Frames** (`is_extended_id=True`). Verify this flag is set correctly in `motor_driver.py` before running on hardware. Sending standard frames to a CubeMars controller will be silently ignored.

### `imu_sensor.py` — Module C (I2C HAL)

The interface to the BNO085 9-axis IMU over the Jetson's I2C bus.

Key characteristics:

- **Hardware Kalman filter**: The driver enables `BNO_REPORT_ROTATION_VECTOR`, which activates the BNO085's onboard sensor-fusion engine. This gives us drift-corrected quaternion orientation without running our own filter.
- **Quaternion-to-Euler conversion**: The raw quaternion `(i, j, k, real)` is converted to Roll, Pitch, and Yaw in degrees using the standard ZYX aerospace convention. The pitch calculation includes a clamp to `[-1, 1]` before `asin` to prevent `NaN` near gimbal lock.
- **I2C fault tolerance**: Every sensor read is wrapped in a `try/except` block. If the I2C bus returns garbage (common with long wire runs, electrical noise from motors, or marginal pull-up resistor values), the class returns the last known good reading rather than crashing or returning zeros.

---

## Hardware Setup

### Connecting to the Jetson via USB-C

Plug a USB-C cable into the Jetson Orin Nano's **data** USB-C port (not the power barrel jack). The Jetson exposes a static IP over USB networking.

```bash
ssh edg5@192.168.55.1
```

**Windows users:** Use WSL (Windows Subsystem for Linux) for SSH. The native Windows OpenSSH client has known routing bugs with USB-gadget network interfaces that cause intermittent connection drops.

### Sharing Internet to the Jetson

The Jetson needs internet access to install Python packages and system updates. You must forward your laptop's Wi-Fi connection over the USB link.

**On your laptop (Linux):**

First, enable IP forwarding:

```bash
sudo sysctl -w net.ipv4.ip_forward=1
```

Identify your Wi-Fi interface name (commonly `wlan0` or `wlp2s0`):

```bash
ip route | grep default
```

Set up NAT masquerading, replacing `<WIFI_INTERFACE>` with your interface name:

```bash
sudo iptables -t nat -A POSTROUTING -o <WIFI_INTERFACE> -j MASQUERADE
sudo iptables -A FORWARD -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -s 192.168.55.0/24 -j ACCEPT
```

**On the Jetson:**

```bash
sudo route add default gw 192.168.55.100
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf > /dev/null
```

Verify connectivity with `ping 8.8.8.8`. If it works but `ping google.com` does not, the DNS line did not stick — re-run the `tee` command.

### Setting Up the CAN Bus (J17 Header)

The Waveshare SN65HVD230 CAN transceiver is wired to the Jetson's **J17 expansion header**. The four connections are TX, RX, 3.3V, and GND. Refer to the Jetson Orin Nano pinout sheet for exact pin numbers.

**Load the kernel modules:**

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
```

**Bring up the interface at 1 Mbit/s:**

```bash
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000 up
```

The `restart-ms 100` parameter tells the driver to automatically recover from bus-off errors after 100 ms. The `txqueuelen 1000` value prevents transmit buffer overflows during sustained 50 Hz operation.

Verify the interface is active:

```bash
ip link show can0
```

You should see `state UP` and `qlen 1000` in the output.

---

## Hardware Parameter Map

| Parameter              | Value                                                                 |
|------------------------|-----------------------------------------------------------------------|
| Motor 1 (Roll Axis)    | CAN ID `104` — being migrated to ID `1` in the final build           |
| Motor 2 (Pitch Axis)   | CAN ID `2`                                                           |
| CAN Bitrate            | 1,000,000 bps (1 Mbit/s)                                             |
| Control Loop Frequency | 50 Hz (20 ms period)                                                 |
| Default Kp             | 0.1                                                                  |
| Default Kd             | 0.01                                                                 |
| Max Torque (Clamp)     | ±3.0 Amps                                                            |
| IMU                    | BNO085 over I2C (`board.SCL` / `board.SDA`)                          |
| CAN Transceiver        | Waveshare SN65HVD230 on J17 header                                   |

---

## Quick Start

After completing the hardware setup above:

```bash
cd ~/catamaran/feature-modular-pd-arch
python3 main.py
```

The terminal will display a single updating line showing real-time roll, pitch, commanded torques, and loop timing. Press `Ctrl+C` to stop. The shutdown handler will zero both motors and release the CAN bus automatically.
