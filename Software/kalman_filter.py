# kalman_filter.py

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimated_measurement_variance, initial_estimate=0.0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.estimate = initial_estimate
        self.error_estimate = 1.0
        self.kalman_gain = 0.0

    def update(self, measurement):
        # Prediction update
        self.error_estimate += self.process_variance

        # Measurement update
        self.kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_variance)
        self.estimate = self.estimate + self.kalman_gain * (measurement - self.estimate)
        self.error_estimate = (1.0 - self.kalman_gain) * self.error_estimate

        return self.estimate
