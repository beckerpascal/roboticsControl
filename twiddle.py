#!/usr/bin/env python


class Twiddle:
    """
    Twiddler class
    """

    def __init__(self, params=[0, 0, 0], deltas=[1, 1, 1], tolerance=0.001):
        self.params = params
        self.deltas = deltas
        self.tolerance = tolerance
        self.error_min = float("inf")

    def tune(self, error_function):
        """
        Calculate the tuned parameters
        """
        while self.best_error > self.tolerance:
            for i in range(len(self.params)):
                self.params[i] += self.deltas[i]
                error = error_function(self.params)
                if error < self.error_min:
                    # New best result
                    self.error_min = error
                    self.deltas[i] *= 1.1
                else:
                    # Was not better, try other way around
                    self.params[i] -= 2*self.deltas[i]
                    error = error_function(self.params)
                    if error < self.error_min:
                        # Was best when going the other way
                        self.deltas[i] *= 1.1
                    else:
                        # Didn't get better results using previous delta.
                        # Thus, make it smaller and try again
                        self.params[i] += self.deltas[i]
                        self.deltas[i] *= 0.9

        return self.params
