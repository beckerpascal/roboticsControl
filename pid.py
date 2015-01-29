#!/usr/bin/env python


class PID:

    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, reference=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.reference = reference
        self.previous_error = 0.0
        self.accumulated_error = 0.0

    def control(self, input, reference=None):
        # Calculate new error and accumulate
        error = (reference if reference else self.reference) - input
        self.accumulated_error += error
        # Calculate control output
        P_term = self.Kp * self.error
        D_term = self.Kd * (error - self.previous_error)
        I_term = self.Ki * self.accumulated_error
        control = P_term + I_term + D_term

        self.previous_error = error
        # Return control value
        return control
