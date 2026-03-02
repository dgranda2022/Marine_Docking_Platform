"""CAN-based motor driver for CubeMars actuators over SocketCAN."""

import struct
import time

import can


class MotorDriver:
    """Low-level driver for a CubeMars motor on a SocketCAN interface.

    Handles connection lifecycle, arming sequence, torque commands, and
    state feedback parsing over a raw CAN bus.

    Attributes:
        motor_id: CAN device ID of the target motor.
        interface: SocketCAN channel name (e.g., 'can0').
        POS_SCALE: Multiplier to convert raw position ticks to user units.
        CUR_SCALE: Multiplier to convert raw current ticks to amperes.
        bus: The underlying python-can Bus instance, set by `connect()`.
    """

    POS_SCALE: float = 0.1
    CUR_SCALE: float = 0.01

    def __init__(self, motor_id: int = 1, interface: str = "can0") -> None:
        """Initialize the motor driver.

        Args:
            motor_id: CAN node ID of the CubeMars motor (default 1).
            interface: SocketCAN channel to use (default 'can0').
        """
        self.motor_id: int = motor_id
        self.interface: str = interface
        self.bus: can.Bus | None = None

    def connect(self) -> bool:
        """Open the SocketCAN bus at 1 Mbit/s.

        Returns:
            True if the bus was opened successfully, False otherwise.
        """
        try:
            self.bus = can.Bus(
                interface="socketcan",
                channel=self.interface,
                bitrate=1000000,
            )
            return True
        except (can.CanError, OSError):
            self.bus = None
            return False

    def arm(self) -> None:
        """Wake the CubeMars motor by sending ten zero-current commands.

        This satisfies the controller's watchdog requirement so it
        transitions from idle to active mode.
        """
        for _ in range(10):
            self.send_torque(0.0)
            time.sleep(0.01)

    def send_torque(self, current_amps: float) -> None:
        """Command a torque-producing current to the motor.

        The current value is scaled to milliamps, packed as a 32-bit
        big-endian signed integer, and transmitted on the CAN bus.

        Args:
            current_amps: Desired phase current in amperes.
        """
        arb_id: int = (1 << 8) | self.motor_id
        payload: bytes = struct.pack(">i", int(current_amps * 1000.0))
        msg = can.Message(arbitration_id=arb_id, data=payload, is_extended_id=True)
        try:
            self.bus.send(msg)
        except can.CanError:
            pass

    def get_state(self) -> dict | None:
        """Drain the receive buffer and return the latest motor feedback.

        The OS-level CAN socket may queue multiple frames. This method
        reads all pending frames and keeps only the most recent one that
        matches this driver's motor ID with command byte 41 (status
        report).

        Returns:
            A dictionary with 'pos' (scaled position) and 'torque'
            (scaled current) if a valid frame was found, or None if no
            matching frame was available.
        """
        latest: can.Message | None = None

        while True:
            frame = self.bus.recv(timeout=0)
            if frame is None:
                break
            cmd: int = frame.arbitration_id >> 8
            dev: int = frame.arbitration_id & 0xFF
            if cmd == 41 and dev == self.motor_id:
                latest = frame

        if latest is None:
            return None

        val = struct.unpack(">hhhh", latest.data)
        return {
            "pos": val[0] * self.POS_SCALE,
            "torque": val[2] * self.CUR_SCALE,
        }

    def stop(self) -> None:
        """Send a zero-torque command and shut down the CAN bus.

        Safe to call even if the bus was never opened or has already
        been shut down.
        """
        if self.bus is not None:
            self.send_torque(0.0)
            try:
                self.bus.shutdown()
            except (can.CanError, OSError):
                pass
            self.bus = None