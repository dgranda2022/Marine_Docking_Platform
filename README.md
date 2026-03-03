# feature/modular-pd-arch

## Marine Docking Platform — Modular PD Control Architecture

This branch replaces the previous single-script test approach with a modular, contract-based Hardware Abstraction Layer (HAL) architecture for the Jetson Orin Nano stabilization system. Each module owns exactly one responsibility and communicates through a centralized State Dictionary managed by a single orchestrator loop.

If you are pulling this branch for the first time, read the full setup procedure below before powering on the motors.

---

## Software Architecture

The codebase is organized around a strict principle: **separation of concerns**. No module knows how any other module works internally. They exchange data exclusively through a centralized, JSON-style State Dictionary that the orchestrator reads and writes on every loop iteration.

There are four files, each with a single job.

### `main.py` — The Orchestrator

This is the only file that imports the other three. It runs a **50 Hz control loop** and is responsible for the following:

- Initializing all hardware (IMU, both motors) and verifying connections at startup.
- Computing a precise time delta (`dt`) on every iteration using `time.perf_counter()`.
- Reading sensor data, passing it to the controllers, sending commands to the motors, reading motor feedback, and updating the State Dictionary — in that exact order, every cycle.
- Printing a single overwriting status line to the terminal for real-time monitoring.
- Catching `KeyboardInterrupt` and guaranteeing that both motors receive a zero-torque command and the CAN bus is released in the `finally` block.

The orchestrator does not contain any math, any byte packing, or any I2C transactions. It only moves data between modules.

> **Design Rationale — Why calculate `dt` dynamically?**
>
> Linux on the Jetson is not a real-time operating system. When we request a 20 ms sleep, the kernel might actually give us 20 ms on one iteration and 25 ms on the next, depending on what else is running. If we assumed a fixed `dt = 0.02` but the OS actually took 0.025 s to come back to us, the derivative term would compute `(error - previous_error) / 0.02` when the true elapsed time was 0.025 s — a 25% overshoot that produces a torque spike out of thin air. By measuring the real wall-clock delta with `time.perf_counter()` every loop, the physics stays correct regardless of OS scheduling jitter. The controller sees the world as it actually is, not as we wish it were.

### `pd_controller.py` — Module A (Pure Math)

A stateless-except-for-derivative PD controller class. It accepts a target angle, a measured angle, and a `dt`, then returns a clamped torque command.

Key characteristics:

- **State retention**: The class stores `previous_error` internally so it can compute the derivative term `(error - previous_error) / dt` across consecutive calls.
- **Safety clamping**: The output is hard-limited to `±max_torque` (default 3.0 Amps).
- **Zero-dt guard**: If `dt` is zero or negative (which can happen on the first iteration or under extreme scheduling anomalies), the derivative term is suppressed entirely rather than producing a division-by-zero fault.
- **No hardware imports**: This file imports nothing beyond Python builtins. It can be unit-tested on any laptop without a Jetson.

> **Design Rationale — Why clamp at 3 A when the motors can do 17 A?**
>
> This is a deliberate safety sandbox. At 3 A the platform has enough authority to stabilize itself, but not enough to injure someone standing next to it or tear a mechanical joint apart if the control logic has a sign error or a runaway derivative spike. During initial testing, a software bug should result in a gentle wobble, not a violent snap. We will raise this limit incrementally as confidence in the system grows, and only after mechanical hard-stops are installed.

> **Design Rationale — Why separate modules instead of one script?**
>
> The PD controller is a pure-math module with zero hardware imports. This means anyone on the team can clone the repo, open `pd_controller.py` on their laptop, and write unit tests against it without needing a Jetson, a CAN transceiver, or a motor. The same applies to verifying the quaternion-to-Euler math in the IMU module — the static method can be called directly with known quaternion values and checked against a reference table. Bugs in math should be caught on a laptop at a desk, not discovered on the dock with live hardware.

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

The Jetson needs internet access to install Python packages and system updates. Daniel's laptop is the designated network bridge for the team. The Wi-Fi interface on this machine is `wlo1` and the Jetson-side bridge interface is `l4tbr0`. These are the exact commands we validated — run them in order.

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

Verify connectivity with `ping 8.8.8.8`. If it works but `ping google.com` does not, the DNS line did not stick — re-run the `tee` command. Note that the `resolv.conf` entry and the `iptables` rules do not survive a reboot on either machine; you will need to re-run them after every power cycle.

### Setting Up the CAN Bus (J17 Header)

The Waveshare SN65HVD230 CAN transceiver is wired to the Jetson's **J17 expansion header**. The four connections are TX, RX, 3.3V, and GND. Refer to the Jetson Orin Nano pinout sheet taped to the inside of the enclosure lid for exact pin numbers.

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

## Project Roadmap

The modular architecture on this branch was designed with the following three phases in mind. Each phase adds a new capability without rewriting existing modules.

### Phase 1 — Gravity Compensation (Feedforward Term)

**Problem:** The platform currently sags under its own weight. A PD controller is purely reactive — it only produces torque in response to an error that already exists. This means the system must first drift away from level before the controller pushes back, resulting in a persistent steady-state offset where the platform hangs slightly below horizontal.

**Solution:** We will add a feedforward gravity compensation term to the control output. If we know the angle of the platform, we know the component of gravitational torque acting on it. We cancel that torque proactively, before it creates an error.

The feedforward term takes the form:

$$\tau_{ff} = K_g \cdot \cos(\theta)$$

where θ is the current joint angle and $K_g$ is a gain that encodes the mass and moment arm of the platform. The total motor command becomes:

$$\tau_{cmd} = \underbrace{K_p \cdot e + K_d \cdot \dot{e}}_{\text{PD (stabilization)}} + \underbrace{K_g \cdot \cos(\theta)}_{\text{Feedforward (weight holding)}}$$

This lets the PD terms focus entirely on dynamic stabilization (responding to waves, wind, and landing loads) rather than wasting authority fighting gravity. In practice, the platform should sit at level with near-zero steady-state error even before a disturbance arrives.

$K_g$ will be determined experimentally by slowly increasing the value until the platform holds itself level with the PD gains set to zero.

### Phase 2 — Live Tuner (Terminal UI)

**Problem:** Tuning $K_p$, $K_d$, and $K_g$ currently requires stopping the script, editing the State Dictionary in `main.py`, and restarting. Every restart triggers the motor arming sequence and resets controller state, making it impossible to do fine iterative tuning under realistic conditions.

**Solution:** We will build a `curses`-based terminal dashboard that runs in a **separate thread** alongside the 50 Hz control loop. The dashboard will display the State Dictionary in real time (angles, torques, loop timing) and allow the operator to nudge gain values up or down with keyboard bindings (e.g., arrow keys to select a parameter, `+`/`-` to adjust).

The tuner thread will write directly to the shared State Dictionary. Because the control loop reads its gains from the State Dictionary on every iteration, changes take effect on the very next 20 ms cycle — no restart, no re-arm, no interruption. This gives us true closed-loop tuning: adjust a gain, immediately see the platform respond, adjust again.

Thread safety will be handled with a single `threading.Lock` guarding the Tuning sub-dictionary.

### Phase 3 — "Clear to Land" Stability Monitor

**Problem:** The platform needs to signal the drone that it is stable enough to land. A single angle reading is not sufficient — the platform could pass through level momentarily while oscillating violently.

**Solution:** We will implement a **Sliding Window Variance Check** that evaluates stability over time, not at a single instant.

The logic works as follows:

- Maintain a rolling buffer of the last 1 second of error data (50 samples at 50 Hz).
- On every loop iteration, compute the variance of the buffer.
- If the variance remains below **2.0 degrees** for **5 continuous seconds**, the system asserts a "CLEAR TO LAND" signal.
- If the variance exceeds the threshold at any point during the 5-second window, the countdown resets.

In the initial implementation, the signal will be a printed status line and a flag in the State Dictionary. In the final build, it will drive a **GPIO pin** on the Jetson's J17 header, providing a hardware-level binary signal that the drone's flight controller can read directly without any software integration.

---

## Quick Start

After completing the hardware setup above:

```bash
cd ~/catamaran/feature-modular-pd-arch
python3 main.py
```

The terminal will display a single updating line showing real-time roll, pitch, commanded torques, and loop timing. Press `Ctrl+C` to stop. The shutdown handler will zero both motors and release the CAN bus automatically.