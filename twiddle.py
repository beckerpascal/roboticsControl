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
        # Initialize a buffered vector for the best parameters found for
        # cleaner keyboard interrupt handling
        best_params = [-1 for p in self.params]
        # Catch ctrl-Cs here since the interesting data (best params) are here
        try:
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
                best_params = self.params
        except KeyboardInterrupt:
            pass  # Do nothing and then pass the params
        return best_params
