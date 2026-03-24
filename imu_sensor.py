"""BNO08x IMU reader with quaternion-to-Euler conversion over I2C."""

import math
import time

from adafruit_extended_bus import ExtendedI2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C


class IMUReader:
    """High-level interface to a BNO08x IMU on a specific I2C bus.

    Provides fused orientation as Euler angles (degrees) with automatic
    fallback to the last good reading when the I2C bus glitches, and
    automatic re-initialisation if the sensor locks up.

    Attributes:
        i2c: The underlying ExtendedI2C peripheral instance.
        imu: The BNO08X_I2C sensor driver.
        last_angles: Most recent valid Euler angle reading, used as a
            fallback when a read fails due to I2C noise.
    """

    I2C_BUS: int = 7          # /dev/i2c-7 — confirmed by i2cdetect
    MAX_CONSEC_ERRORS: int = 5 # reinit after this many consecutive failures

    def __init__(self, i2c_bus: int = I2C_BUS) -> None:
        """Initialize the I2C bus, BNO08x sensor, and enable rotation vector reports.

        Args:
            i2c_bus: Linux I2C bus number (default 7, confirmed on this board).
        """
        self._bus_num: int = i2c_bus
        self._consec_errors: int = 0
        self.last_angles: dict = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self._init_sensor()

    def _init_sensor(self) -> None:
        """Open the I2C bus, bind to the BNO08x, and enable rotation vector."""
        self.i2c: ExtendedI2C = ExtendedI2C(self._bus_num)
        self.imu: BNO08X_I2C = BNO08X_I2C(self.i2c)
        # BNO08x needs up to 5 s after cold power-on before accepting feature
        # enable commands.  Retry with backoff rather than failing immediately.
        for attempt in range(10):
            try:
                self.imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                break
            except RuntimeError:
                if attempt == 9:
                    raise
                time.sleep(0.5)
        self._consec_errors = 0

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
            self._consec_errors = 0
        except Exception:
            self._consec_errors += 1
            if self._consec_errors >= self.MAX_CONSEC_ERRORS:
                try:
                    self._init_sensor()
                except Exception:
                    pass
            return self.last_angles.copy()

        return self.last_angles.copy()