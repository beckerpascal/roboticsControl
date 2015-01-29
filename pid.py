#!/usr/bin/env python


class PID:
    """
    A PID controller sporting an option to do accumulator min/max anti-windup.
    """

    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, reference=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.reference = reference
        self.previous_error = 0.0
        self.accumulated_error = 0.0
        self.anti_windup = False

    def control(self, input, reference=None):
        """
        Compute a control value for @input. If no @reference is used, fall
        back to self.reference. 
        """
        # Calculate new error and accumulate
        error = (reference if reference else self.reference) - input
        self.accumulated_error += error
        # Check for accumulator limits
        if (self.anti_windup):
            if self.accumulated_error < self.accumulator_min:
                self.accumulated_error = self.accumulator_min
            elif self.accumulated_error > self.accumulator_max:
                self.accumulated_error = self.accumulated_max
        # Calculate control output
        P_term = self.Kp * self.error
        D_term = self.Kd * (error - self.previous_error)
        I_term = self.Ki * self.accumulated_error
        control = P_term + I_term + D_term
        # Store current error
        self.previous_error = error
        # Return control value
        return control

    def anti_windup(self, acc_min, acc_max=None):
        """
        @acc_min false for disabling
        @acc_max defaults to -@acc_min
        """
        self.anti_windup = True if acc_min is not False else False
        self.accumulator_min = acc_min
        self.accumulator_max = acc_max if acc_max is not None else -acc_min


if __name__ == "__main__":
    print("No tests, man!")
