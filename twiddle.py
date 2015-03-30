#!/usr/bin/env python


class Twiddle:
    """
    Twiddler class
    """

    def __init__(self, params=[0, 0, 0], deltas=[2, 1, 1], tolerance=10**-6):
        self.params = params
        self.deltas = deltas
        self.tolerance = tolerance

    def setup(self, **kwargs):
        self.params = kwargs.get("params", self.params)
        self.deltas = kwargs.get("deltas", self.deltas)
        self.tolerance = kwargs.get("tolerance", self.tolerance)

    def tune(self, error_function):
        """
        Calculate the tuned parameters
        """

        # Catch ctrl-Cs here since the interesting data (best params) are here
        try:
            # Initialize a buffered vector for the best parameters found for
            # cleaner keyboard interrupt handling
            best_params = self.params[:]
            # Initial run
            error_min, error_min_time = error_function(self.params)
            # Coordinate descent -> tolerance will be on the param deltas
            while sum(map(abs, self.deltas)) > self.tolerance:  # error_min > self.tolerance
                for i in range(len(self.params)):
                    self.params[i] += self.deltas[i]
                    error, sim_time = error_function(self.params)
                    if error < error_min:
                        # New best result
                        error_min = float(error)
                        error_min_time = int(sim_time)
                        best_params = self.params[:]  # Deep-ish copy
                        self.deltas[i] *= 1.1
                    else:
                        # Was not better, try other way around
                        self.params[i] -= 2*self.deltas[i]
                        error, sim_time = error_function(self.params)
                        if error < error_min:
                            # Was best when going the other way
                            error_min = float(error)
                            error_min_time = int(sim_time)
                            best_params = self.params[:]  # Deep-ish copy
                            self.deltas[i] *= 1.1
                        else:
                            # Didn't get better results using previous delta.
                            # Thus, make it smaller and try again
                            self.params[i] += self.deltas[i]
                            self.deltas[i] *= 0.9
        except KeyboardInterrupt:
            print "ctrl-C!"  # Do nothing and then pass the params
        return (best_params, error_min, error_min_time)
