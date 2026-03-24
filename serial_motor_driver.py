"""UART-based motor driver for CubeMars AK60-39 V3.0 actuators over VESC serial protocol.

Replaces the CAN-based MotorDriver for use with USB-to-UART adapters
connected to the CubeMars motors configured for 'Periodic Feedback' (VESC).

Hardware setup:
    - Two USB-to-UART adapters at /dev/ttyUSB0 (Roll) and /dev/ttyUSB1 (Pitch)
    - Baud rate: 921600, 8N1
    - VESC serial framing: [0x02] [len] [payload] [CRC16-CCITT (2B)] [0x03]

Protocol reference:
    https://github.com/vedderb/bldc/blob/master/comm/comm_uart.c
"""

import struct
import time
from typing import Optional

import serial


# ── VESC CRC16-CCITT Lookup Table ────────────────────────────────────────────
# Polynomial 0x1021, used by the VESC firmware for all serial frame checksums.

_CRC16_TAB: list[int] = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x54A5,
    0xA54A, 0xB56B, 0x8508, 0x9529, 0xE5CE, 0xF5EF, 0xC58C, 0xD5AD,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x4864, 0x5845, 0x6826, 0x7807, 0x08E0, 0x18C1, 0x28A2, 0x38A3,
    0xC94C, 0xD96D, 0xE90E, 0xF92F, 0x89C8, 0x99E9, 0xA98A, 0xB9AB,
    0x5A55, 0x4A74, 0x7A17, 0x6A36, 0x1AD1, 0x0AF0, 0x3A93, 0x2AB2,
    0xDB5D, 0xCB7C, 0xFB1F, 0xEB3E, 0x9BD9, 0x8BF8, 0xBB9B, 0xAB9A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x85A9, 0xF54E, 0xE56F, 0xD50C, 0xC52D,
    0x34C2, 0x24E3, 0x1480, 0x04A1, 0x7446, 0x6467, 0x5404, 0x4425,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
]


def vesc_crc16(data: bytes) -> int:
    """Compute the VESC CRC16-CCITT checksum over a byte sequence.

    This is the standard CRC used by the VESC firmware for serial frame
    integrity checking (polynomial 0x1021, init 0x0000).

    Args:
        data: Raw payload bytes to checksum.

    Returns:
        16-bit unsigned CRC value.
    """
    crc: int = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC16_TAB[((crc >> 8) ^ byte) & 0xFF]
    return crc


def _build_vesc_frame(payload: bytes) -> bytes:
    """Wrap a VESC payload into a complete serial frame.

    Frame format (for payloads <= 256 bytes):
        [0x02] [length: 1 byte] [payload] [CRC16: 2 bytes big-endian] [0x03]

    For payloads > 256 bytes the start byte would be 0x03 with a 2-byte
    length field, but our command payloads are always short so we only
    implement the single-byte-length variant.

    Args:
        payload: The command payload (command ID + data).

    Returns:
        Complete framed byte string ready for serial transmission.
    """
    length = len(payload)
    if length > 256:
        raise ValueError(f"Payload too long for short-frame format: {length} bytes")

    crc = vesc_crc16(payload)
    frame = bytearray()
    frame.append(0x02)                          # Start byte (short frame)
    frame.append(length & 0xFF)                 # Payload length
    frame.extend(payload)                       # Payload data
    frame.append((crc >> 8) & 0xFF)             # CRC high byte
    frame.append(crc & 0xFF)                    # CRC low byte
    frame.append(0x03)                          # Stop byte
    return bytes(frame)


def _parse_vesc_frame(buf: bytes) -> Optional[bytes]:
    """Attempt to extract a valid VESC serial frame from a byte buffer.

    Searches for a start byte (0x02), reads the length, verifies the CRC,
    and confirms the stop byte.  Returns the payload if valid, or None if
    no valid frame could be found.

    Args:
        buf: Raw bytes read from the serial port.

    Returns:
        The payload bytes (excluding framing) if a valid frame is found,
        or None otherwise.
    """
    # Scan for start byte
    idx = buf.find(0x02)
    if idx == -1:
        return None

    # Need at least: start(1) + len(1) + payload(>=1) + crc(2) + stop(1)
    if idx + 2 > len(buf):
        return None

    payload_len = buf[idx + 1]
    # Total frame size: 1 (start) + 1 (len) + payload_len + 2 (CRC) + 1 (stop)
    frame_end = idx + 2 + payload_len + 3
    if frame_end > len(buf):
        return None

    payload = buf[idx + 2 : idx + 2 + payload_len]
    crc_received = (buf[idx + 2 + payload_len] << 8) | buf[idx + 2 + payload_len + 1]
    stop_byte = buf[idx + 2 + payload_len + 2]

    if stop_byte != 0x03:
        return None

    crc_computed = vesc_crc16(payload)
    if crc_received != crc_computed:
        return None

    return payload


# ── VESC Command IDs ─────────────────────────────────────────────────────────

COMM_SET_CURRENT: int = 0x06       # Set motor current (amps × 1000 → int32)
COMM_GET_VALUES: int = 0x04        # Request full telemetry packet


class SerialMotorDriver:
    """UART-based driver for a CubeMars AK60-39 V3.0 motor via VESC protocol.

    Drop-in replacement for the CAN-based MotorDriver.  Each motor gets
    its own USB-to-UART adapter and its own instance of this driver.

    Attributes:
        port: OS device path for the serial adapter (e.g., '/dev/ttyUSB0').
        motor_id: Logical motor identifier used only for logging/debug.
        baudrate: UART baud rate (921600 for CubeMars VESC mode).
        ser: The underlying pyserial Serial instance, set by `connect()`.
    """

    BAUDRATE: int = 921600

    def __init__(self, port: str, motor_id: int = 1) -> None:
        """Initialize the serial motor driver.

        Args:
            port: Device path for the USB-to-UART adapter
                  (e.g., '/dev/ttyUSB0').
            motor_id: Logical motor ID for logging (default 1).
        """
        self.port: str = port
        self.motor_id: int = motor_id
        self.ser: Optional[serial.Serial] = None

    def connect(self) -> bool:
        """Open the UART port at 921600 baud, 8N1.

        Uses a very short read timeout (10 ms) so that get_state() never
        blocks the real-time control loop.

        Returns:
            True if the port was opened successfully, False otherwise.
        """
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,       # 10 ms read timeout — non-blocking enough for 50 Hz
                write_timeout=0.01,
            )
            # Flush any stale data sitting in the OS buffer
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            return True
        except (serial.SerialException, OSError) as exc:
            print(f"[ERROR] Motor {self.motor_id}: Could not open {self.port}: {exc}")
            self.ser = None
            return False

    def arm(self) -> None:
        """Wake the motor by sending a burst of zero-current commands.

        Mirrors the CAN-based MotorDriver.arm() behaviour: ten zero-torque
        packets at ~10 ms intervals satisfy the VESC watchdog so the
        controller transitions from idle to active current-control mode.
        """
        for _ in range(10):
            self.send_torque(0.0)
            time.sleep(0.01)

    def send_torque(self, current_amps: float) -> None:
        """Command a torque-producing current to the motor.

        Constructs a VESC COMM_SET_CURRENT frame:
            Payload = [0x06] [current_mA as int32 big-endian]

        The VESC firmware interprets the value as milliamps (amps × 1000).

        Args:
            current_amps: Desired phase current in amperes.  Positive =
                          forward torque, negative = reverse torque.
        """
        if self.ser is None or not self.ser.is_open:
            return

        current_mA: int = int(current_amps * 1000.0)
        payload = struct.pack(">Bi", COMM_SET_CURRENT, current_mA)
        frame = _build_vesc_frame(payload)

        try:
            self.ser.write(frame)
        except (serial.SerialException, OSError):
            pass

    def request_telemetry(self) -> None:
        """Send a COMM_GET_VALUES request to the motor.

        The motor will respond asynchronously with a telemetry packet that
        can be read on the next call to get_state().  Call this once per
        control loop iteration *before* get_state() to keep feedback fresh.
        """
        if self.ser is None or not self.ser.is_open:
            return

        payload = struct.pack(">B", COMM_GET_VALUES)
        frame = _build_vesc_frame(payload)

        try:
            self.ser.write(frame)
        except (serial.SerialException, OSError):
            pass

    def get_state(self) -> Optional[dict]:
        """Read and parse the most recent COMM_GET_VALUES response.

        Drains all available bytes from the OS serial buffer, finds the
        latest valid VESC frame, and extracts motor position and current.

        VESC COMM_GET_VALUES response payload layout (partial, big-endian):
            Byte  0     : Command ID (0x04)
            Bytes 1-2   : temp_fet (int16, ×10)
            Bytes 3-4   : temp_motor (int16, ×10)
            Bytes 5-8   : avg_motor_current (int32, ×100)
            Bytes 9-12  : avg_input_current (int32, ×100)
            ...
            Bytes 29-32 : rpm (int32)
            ...
            Bytes 41-44 : tachometer_value (int32) — proportional to position
            ...

        We extract avg_motor_current (as torque proxy) and tachometer
        (as position proxy) to match the CAN-based MotorDriver.get_state()
        return format: {"pos": float, "torque": float}.

        Returns:
            Dictionary with 'pos' and 'torque' keys, or None if the buffer
            is empty, corrupted, or no valid frame was found.
        """
        if self.ser is None or not self.ser.is_open:
            return None

        try:
            waiting = self.ser.in_waiting
            if waiting == 0:
                return None
            raw = self.ser.read(waiting)
        except (serial.SerialException, OSError):
            return None

        if len(raw) == 0:
            return None

        # There may be multiple frames queued — find the last valid one
        payload: Optional[bytes] = None
        search_buf = raw
        while True:
            found = _parse_vesc_frame(search_buf)
            if found is None:
                break
            if len(found) > 0 and found[0] == COMM_GET_VALUES:
                payload = found
            # Advance past this frame to look for more
            idx = search_buf.find(0x02)
            if idx == -1:
                break
            length_byte = search_buf[idx + 1] if idx + 1 < len(search_buf) else 0
            next_start = idx + 2 + length_byte + 3
            if next_start >= len(search_buf):
                break
            search_buf = search_buf[next_start:]

        if payload is None:
            return None

        # Minimum COMM_GET_VALUES response is ~70 bytes; we need at least
        # through tachometer at offset 41-44 (so 45 bytes of payload).
        if len(payload) < 45:
            return None

        try:
            # Command ID is payload[0] == 0x04, already verified
            # avg_motor_current: int32 at payload offset 5..8, scaled ×100
            avg_motor_current_raw = struct.unpack_from(">i", payload, 5)[0]
            motor_current_amps = avg_motor_current_raw / 100.0

            # tachometer_value: int32 at payload offset 41..44
            tachometer_raw = struct.unpack_from(">i", payload, 41)[0]
            # Convert tachometer ticks to a rough position value.
            # For the AK60-39: 3 pole pairs, 6 ticks per erev → divide by 6×3=18
            # for mechanical revolutions, then ×360 for degrees.
            # Adjust this constant if your gear ratio or pole count differs.
            TICKS_PER_MECH_REV = 18.0
            position_deg = (tachometer_raw / TICKS_PER_MECH_REV) * 360.0

            return {
                "pos": position_deg,
                "torque": motor_current_amps,
            }
        except struct.error:
            return None

    def stop(self) -> None:
        """Send a zero-torque command and close the serial port.

        Safe to call even if the port was never opened or has already
        been closed.
        """
        if self.ser is not None:
            try:
                self.send_torque(0.0)
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None