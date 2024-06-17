class PIDController:
    """
    A Proportional-Integral-Derivative (PID) Controller class.

    Args:
        Kp (float): Proportional gain.
        Ki (float): Integral gain.
        Kd (float): Derivative gain.
        output_max (float): Maximum output value.
        output_min (float): Minimum output value.

    Attributes:
        Kp (float): Proportional gain.
        Ki (float): Integral gain.
        Kd (float): Derivative gain.
        output_max (float): Maximum output value.
        output_min (float): Minimum output value.
        prev_error (float): Previous error value.
        accumulative_error (float): Accumulated error value.

    Methods:
        get_control_command: Calculates the control command based on the current error.

    """

    def __init__(self, Kp: float, Ki: float, Kd: float, output_min: float, output_max: float):
        """
        Initialize the PIDController with the specified gains and output limits.

        Args:
            Kp (float): Proportional gain.
            Ki (float): Integral gain.
            Kd (float): Derivative gain.
            output_max (float): Maximum output value.
            output_min (float): Minimum output value.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max
        self.prev_error = 0.0
        self.accumulative_error = 0.0

    def get_control_command(self, current_error: float, dt: float) -> float:
        """
        Calculate the control command based on the current error and time difference.

        Args:
            current_error (float): The current error value.
            dt (float): The time difference since the last control command.

        Returns:
            float: The calculated control command.
        """
        P = self.Kp * current_error
        self.accumulative_error += current_error * dt
        I = self.Ki * self.accumulative_error
        # To prevent dividing by zero
        if dt != 0:
            D = self.Kd * (current_error - self.prev_error) / dt
        else:
            D = 0
        control_command = P + I + D
        # limit output to be between output_min and output_max
        control_command = max(min(control_command, self.output_max), self.output_min)
        self.prev_error = current_error

        return control_command