import time


__all__ = ["PIDController"]


# def input_modulus(input_, minimumInput, maximumInput):
#     modulus = maximumInput - minimumInput
#
#     num_max = math.floor((input_ - minimumInput) / modulus)
#     input_ -= num_max * modulus
#
#     num_min = math.floor((input_ - maximumInput) / modulus)
#     input_ -= num_min * modulus
#
#     # return input_ % modulus
#     return input_

def input_modulus(input_, minimumInput, maximumInput):
    diff = maximumInput - minimumInput
    while input_ > maximumInput:
        input_ -= diff
    while input_ < minimumInput:
        input_ += diff
    return input_


class PIDController:
    def __init__(self, p: float, i: float, d: float):
        # TODO: enable continuous input
        """
        Creates a PID controller with the given P/I/D values
        :param p: P value
        :param i: I value
        :param d: D value
        """
        self.p = p
        self.i = i
        self.d = d
        self._current_setpoint = 0
        self._prev_err = 0
        self._accumulated_err = 0
        self._last_time = time.time()
        self._continuous_input = False
        self._max = None
        self._min = None

    def enable_continuous_input(self, low: float, high: float):
        self._continuous_input = True
        if low >= high:
            raise ValueError("Low must be < high value")
        self._max = high
        self._min = low

    def calculate(self, current, goal):
        """
        Calculates the PID output for a given goal using the given setpoint. Updates the stored setpoint
        :param current: current position
        :param goal: desired position
        :return: PID output
        """
        self.set_goal(goal)
        if self._continuous_input:
            # what the maximum error can be
            max_err = self._max - self._min
            max_err /= 2
            # use fancy bs (copied from WPILib) to find the closest err
            error = input_modulus(goal - current, -max_err, max_err)
        else:
            error = goal - current
        dt = time.time() - self._last_time
        # TODO: remove (simulations fast go brrrrrr)
        if dt == 0:
            dt = 0.000001
        # TODO: this can grow infinitely if we never reach the setpoint, clamp
        self._accumulated_err += error * dt
        p_out = self.p * error
        i_out = self.i * self._accumulated_err
        d_out = self.d * (error - self._prev_err) / dt
        self._prev_err = error
        # shhhh this won't be off by some tiny amount
        self._last_time = time.time()
        return p_out + i_out + d_out

    def reset(self):
        """
        Resets the current setpoint, previous error, and accumulated error.
        """
        self._current_setpoint = 0
        self._prev_err = 0
        self._accumulated_err = 0
        self._last_time = time.time()

    def set_goal(self, setpoint):
        """
        Sets the desired setpoint. Called by calculate() automatically
        :param setpoint: the desired setpoint
        """
        self._current_setpoint = setpoint

    # def calculate(self, goal):
    #     """
    #     Calculates the PID output for a given goal using the stored setpoint
    #     :param goal: the desired position
    #     :return: PID output
    #     """
    #     return self.calculate(self._current_setpoint, goal)
