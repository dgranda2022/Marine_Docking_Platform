"""PD (Proportional-Derivative) controller for joint torque computation."""


class PDController:
    """A stateful PD controller that outputs clamped torque commands.

    This is a pure-math module with no hardware dependencies, suitable for
    use in a modular robotics control pipeline (e.g., Jetson Orin Nano).

    Attributes:
        Kp: Proportional gain.
        Kd: Derivative gain.
        max_torque: Absolute torque saturation limit (symmetric).
        previous_error: Error from the last call to `calculate`, used for
            the derivative term. Initialized to 0.0.
    """

    def __init__(self, Kp: float, Kd: float, max_torque: float) -> None:
        """Initialize the PD controller.

        Args:
            Kp: Proportional gain coefficient.
            Kd: Derivative gain coefficient.
            max_torque: Maximum absolute torque the controller may output.
                The output is clamped to [-max_torque, +max_torque].
        """
        self.Kp: float = Kp
        self.Kd: float = Kd
        self.max_torque: float = max_torque
        self.previous_error: float = 0.0

    def calculate(self, target_angle: float, current_angle: float, dt: float) -> float:
        """Compute the clamped torque command for a single control cycle.

        Args:
            target_angle: Desired joint angle (radians or degrees — unit
                must be consistent with gains).
            current_angle: Measured joint angle in the same units.
            dt: Time elapsed since the previous control cycle, in seconds.
                If dt is zero or negative the derivative term is suppressed
                to avoid division-by-zero / nonsensical values.

        Returns:
            Torque command clamped to [-max_torque, +max_torque].
        """
        error: float = target_angle - current_angle

        p_term: float = self.Kp * error

        if dt > 0.0:
            d_term: float = self.Kd * ((error - self.previous_error) / dt)
        else:
            d_term = 0.0

        raw_torque: float = p_term + d_term

        clamped_torque: float = max(-self.max_torque, min(raw_torque, self.max_torque))

        self.previous_error = error

        return clamped_torque