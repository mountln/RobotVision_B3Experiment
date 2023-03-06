"""PID class"""
import time


class PID(object):
    """A PID controller."""

    def __init__(self, Kp=0.1, Ki=0.0, Kd=0.0, output_limits=(None, None)):
        """
        Initializes a new PID controller.

        :param Kp: The coefficient of the proportional term
        :param Ki: The coefficient of the integral term
        :param Kd: The coefficient of the derivative term
        :param output_limits: The (lower_bound, upper_bound) of output
        :type output_limits: (float, float)
        """
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.output_limits = output_limits
        self.setpoint = 0

        self._last_integral_term = 0.0
        self._last_PV = None
        self._last_CV = None
        self._last_time = time.monotonic()

    def compute(self, PV):
        """
        Using process variable (PV) to calculate and return the control
        variable (CV).

        :param PV: The process variable
        :return: The control variable (CV)
        """
        # get current time and calculate delta time
        current_time = time.monotonic()
        d_time = current_time - self._last_time

        # calculate the current error and delta input
        error = self.setpoint - PV
        d_PV = PV - self._last_PV if self._last_PV is not None else 0

        # calculate the control variable(CV)
        proportional_term = self.Kp * error
        integral_term = self._last_integral_term + self.Ki * error * d_time
        derivative_term = -self.Kd * d_PV / d_time
        CV = proportional_term + integral_term + derivative_term

        # bound the CV
        min_output, max_output = self.output_limits
        if min_output is not None and CV < min_output:
            CV = min_output
        elif max_output is not None and CV > max_output:
            CV = max_output

        self._last_integral_term = integral_term
        self._last_CV = CV
        self._last_PV = PV
        self._last_time = current_time

        return CV

    def clear(self):
        """Clears saved calculation status."""
        self._last_integral_term = 0.0
        self._last_PV = None
        self._last_CV = None
        self._last_time = time.monotonic()
