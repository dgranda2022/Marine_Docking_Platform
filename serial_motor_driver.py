"""UART motor driver for CubeMars AK60-39 V3.0 actuators.

Uses the native CubeMars serial protocol from the AK series manual v3.2.0
(NOT VESC framing).  Each motor gets its own USB-to-UART adapter instance.

Hardware setup:
    - USB-to-UART adapters at /dev/ttyUSB0 and /dev/ttyUSB1
    - Baud rate: 921600, 8N1
    - CubeMars frame: [0xAA][Len][Cmd][Data...][CRC16_H][CRC16_L][0xBB]

Protocol reference: AK series product manual v3.2.0, section 4.3.2
"""

import struct
import time
from typing import Optional

import serial


# ── CRC16-CCITT ───────────────────────────────────────────────────────────────
# poly=0x1021, init=0x0000, MSB-first, no reflection, no final XOR.
# Verified: crc16([0x45])                     == 0x1861
#           crc16([0x47,0x00,0x00,0x13,0x88]) == 0x301C
#           (both confirmed against manual page 57-58 example frames)

def _crc16(data: bytes) -> int:
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


# ── Frame codec ───────────────────────────────────────────────────────────────

def _build_frame(cmd: int, data: bytes = b'') -> bytes:
    """Build a CubeMars UART frame.

    Frame layout:
        [0xAA] [Len] [Cmd] [Data...] [CRC_H] [CRC_L] [0xBB]
    Len  = 1 + len(data)   (counts cmd byte + data; excludes AA, Len, CRC, BB)
    CRC  = crc16([cmd] + data)
    """
    payload = bytes([cmd]) + data
    chk = _crc16(payload)
    return bytes([0xAA, len(payload)]) + payload + struct.pack(">H", chk) + bytes([0xBB])


def _parse_frame(buf: bytes) -> Optional[tuple[int, bytes]]:
    """Find and validate the first CubeMars frame in buf.

    Returns:
        (cmd, data) if a valid frame is found, else None.
        data does not include cmd, CRC, or framing bytes.
    """
    idx = buf.find(0xAA)
    if idx == -1 or idx + 4 > len(buf):      # need at least AA Len Cmd CRC_H CRC_L BB
        return None

    length = buf[idx + 1]
    if length < 1:
        return None

    # Full frame: AA(1) + Len(1) + payload(length) + CRC(2) + BB(1)
    frame_end = idx + 2 + length + 3
    if frame_end > len(buf):
        return None

    if buf[frame_end - 1] != 0xBB:
        return None

    payload = buf[idx + 2 : idx + 2 + length]
    crc_rx = struct.unpack_from(">H", buf, idx + 2 + length)[0]
    if _crc16(payload) != crc_rx:
        return None

    cmd  = payload[0]
    data = payload[1:]
    return cmd, data


# ── CubeMars command IDs (from AK manual section 4.3.2) ──────────────────────

_CMD_GET_VALUES:  int = 0x45    # Request full motor state telemetry
_CMD_SET_CURRENT: int = 0x47    # Set phase current (data: int32 = amps × 1000)


class SerialMotorDriver:
    """UART driver for a CubeMars AK60-39 V3.0 motor.

    Uses the CubeMars native serial protocol (AA/BB framing, CRC16-CCITT).
    Drop-in replacement for the CAN-based MotorDriver with the same
    public interface: connect / arm / send_torque / request_telemetry /
    get_state / stop.

    Attributes:
        port:     OS device path (e.g., '/dev/ttyUSB0').
        motor_id: Logical motor ID used for logging only.
        ser:      Underlying pyserial.Serial, set after connect().
    """

    BAUDRATE: int = 921600

    def __init__(self, port: str, motor_id: int = 1) -> None:
        self.port:     str                   = port
        self.motor_id: int                   = motor_id
        self.ser:      Optional[serial.Serial] = None
        self._rx_buf:  bytearray             = bytearray()

    # ── Connection management ─────────────────────────────────────────────────

    def connect(self) -> bool:
        """Open the serial port at 921600 8N1.

        Returns True on success, False on error.
        """
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,
                write_timeout=0.01,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self._rx_buf.clear()
            return True
        except (serial.SerialException, OSError) as exc:
            print(f"[ERROR] Motor {self.motor_id}: cannot open {self.port}: {exc}")
            self.ser = None
            return False

    def arm(self) -> None:
        """Send 10 × zero-current frames to wake the motor controller."""
        for _ in range(10):
            self.send_torque(0.0)
            time.sleep(0.01)

    # ── Control commands ──────────────────────────────────────────────────────

    def send_torque(self, current_amps: float) -> None:
        """Command a phase current (torque-producing) to the motor.

        Args:
            current_amps: Desired current in amperes.  Positive = forward
                          torque direction, negative = reverse.
        """
        if self.ser is None or not self.ser.is_open:
            return
        current_ma = int(current_amps * 1000.0)
        frame = _build_frame(_CMD_SET_CURRENT, struct.pack(">i", current_ma))
        try:
            self.ser.write(frame)
        except (serial.SerialException, OSError):
            pass

    def request_telemetry(self) -> None:
        """Send a COMM_GET_VALUES request; motor replies with its state."""
        if self.ser is None or not self.ser.is_open:
            return
        frame = _build_frame(_CMD_GET_VALUES)
        try:
            self.ser.write(frame)
        except (serial.SerialException, OSError):
            pass

    # ── Feedback ──────────────────────────────────────────────────────────────

    def get_state(self) -> Optional[dict]:
        """Drain the RX buffer and parse the latest COMM_GET_VALUES reply.

        The CubeMars COMM_GET_VALUES (0x45) response data layout mirrors the
        VESC COMM_GET_VALUES payload (the AK series runs VESC firmware with
        CubeMars framing):

            Byte offset in data (after cmd byte stripped):
              0- 1  temp_fet        int16  ×10  (°C)
              2- 3  temp_motor      int16  ×10  (°C)
              4- 7  avg_motor_curr  int32  ×100 (A)
              8-11  avg_input_curr  int32  ×100 (A)
             12-15  (reserved)
             16-19  (reserved)
             20-21  duty_cycle      int16  ×1000
             22-25  rpm             int32
             26-27  v_in            int16  ×10  (V)
             28-31  amp_hours       int32  ×10000
             32-35  amp_hours_chgd  int32  ×10000
             36-39  wh_used         int32  ×10000
             40-43  wh_charged      int32  ×10000
             44-47  tachometer      int32  (ticks)
             48-51  tachometer_abs  int32  (ticks)
             52     fault_code      uint8

        If the actual layout differs, the byte offsets above will need
        adjustment once we capture a real response frame.

        Returns:
            {'pos': float (degrees), 'torque': float (amps)}, or None.
        """
        if self.ser is None or not self.ser.is_open:
            return None

        # Drain everything available into the persistent RX buffer
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self._rx_buf.extend(self.ser.read(waiting))
        except (serial.SerialException, OSError):
            return None

        # Parse all valid frames, keep the last GET_VALUES payload
        last_data: Optional[bytes] = None
        while True:
            result = _parse_frame(bytes(self._rx_buf))
            if result is None:
                break
            cmd, data = result
            if cmd == _CMD_GET_VALUES:
                last_data = data
            # Advance buffer past this frame
            idx = bytes(self._rx_buf).find(0xAA)
            if idx == -1:
                break
            length = self._rx_buf[idx + 1] if idx + 1 < len(self._rx_buf) else 0
            frame_end = idx + 2 + length + 3
            self._rx_buf = self._rx_buf[frame_end:]

        if last_data is None:
            return None

        # Need at least 52 bytes of data for tachometer field
        if len(last_data) < 48:
            return None

        try:
            avg_motor_curr_raw = struct.unpack_from(">i", last_data, 4)[0]
            motor_current_amps = avg_motor_curr_raw / 100.0

            tachometer_raw = struct.unpack_from(">i", last_data, 44)[0]
            # AK60-39: 21 pole pairs → 21 erev/mrev; tachometer in erev ticks
            # Adjust TICKS_PER_REV if empirical calibration suggests otherwise.
            TICKS_PER_MECH_REV = 21.0 * 6.0   # 6 ticks per erev
            position_deg = (tachometer_raw / TICKS_PER_MECH_REV) * 360.0

            return {
                "pos":    position_deg,
                "torque": motor_current_amps,
            }
        except struct.error:
            return None

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def stop(self) -> None:
        """Zero-current the motor and close the serial port."""
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
