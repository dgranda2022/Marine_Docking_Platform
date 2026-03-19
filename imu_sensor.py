"""BNO08x IMU reader with quaternion-to-Euler conversion over I2C."""

import math

import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C


class IMUReader:
    """High-level interface to a BNO08x IMU on the default I2C bus.

    Provides fused orientation as Euler angles (degrees) with automatic
    fallback to the last good reading when the I2C bus glitches.

    Attributes:
        i2c: The underlying busio.I2C peripheral instance.
        imu: The BNO08X_I2C sensor driver.
        last_angles: Most recent valid Euler angle reading, used as a
            fallback when a read fails due to I2C noise.
    """

    def __init__(self) -> None:
        """Initialize the I2C bus, BNO08x sensor, and enable rotation vector reports."""
        self.i2c: busio.I2C = busio.I2C(board.SCL, board.SDA)
        self.imu: BNO08X_I2C = BNO08X_I2C(self.i2c)
        
        # CRITICAL FIX: Reset the sensor to clear any previous "hang" states
        self.imu.soft_reset()
        
        self.imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.last_angles: dict = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    @staticmethod
    def _quaternion_to_euler(
        i: float, j: float, k: float, real: float
    ) -> tuple[float, float, float]:
        """Convert a unit quaternion to Euler angles in degrees.

        Uses the ZYX (aerospace) convention: Yaw around Z, Pitch around
        Y, Roll around X.

        Args:
            i: Quaternion x component.
            j: Quaternion y component.
            k: Quaternion z component.
            real: Quaternion w (scalar) component.

        Returns:
            A tuple of (roll, pitch, yaw) in degrees.
        """
        # Roll (rotation about X)
        sinr_cosp = 2.0 * (real * i + j * k)
        cosr_cosp = 1.0 - 2.0 * (i * i + j * j)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (rotation about Y) — clamped to avoid NaN near poles
        sinp = 2.0 * (real * j - k * i)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        # Yaw (rotation about Z)
        siny_cosp = 2.0 * (real * k + i * j)
        cosy_cosp = 1.0 - 2.0 * (j * j + k * k)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        )

    def get_angles(self) -> dict:
        """Read the current orientation from the IMU as Euler angles.

        Drains the latest rotation-vector quaternion from the BNO08x and
        converts it to roll / pitch / yaw in degrees.  If the I2C read
        fails for any reason (bus noise, NACK, CRC error) the method
        silently returns the last successfully read angles so that the
        control loop always has a usable value.

        Returns:
            Dictionary with 'roll', 'pitch', and 'yaw' in degrees.
        """
        try:
            quat = self.imu.quaternion
            if quat is None:
                return self.last_angles.copy()

            i, j, k, real = quat
            roll_deg, pitch_deg, yaw_deg = self._quaternion_to_euler(i, j, k, real)

            self.last_angles = {
                "roll": roll_deg,
                "pitch": pitch_deg,
                "yaw": yaw_deg,
            }
        except Exception:
            return self.last_angles.copy()

        return self.last_angles.copy()