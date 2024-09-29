import ulab.numpy as np  # Import ulab numpy-like functionality


class ExtendedKalmanFilter:
    def __init__(self, initial_state, process_covariance, measurement_covariance):
        """
        Initialize the EKF with the initial state, process covariance, and measurement covariance.

        :param initial_state: List of floats [altitude, velocity, acceleration].
        :param process_covariance: 2D list representing the process covariance matrix (Q).
        :param measurement_covariance: List representing the measurement covariance matrix (R).
        """
        self.state = np.array(initial_state)

        self.process_covariance = np.array(process_covariance)
        self.measurement_covariance = np.array(measurement_covariance)

        # Error covariance matrix (P): Initial uncertainty in state estimates (identity matrix)
        self.error_covariance = np.eye(len(self.state))

        # To store the previous altitude for derivative calculations
        self.previous_altitude = self.state[0]  # Altitude
        self.previous_velocity = self.state[1]  # Velocity

    def predict(self, dt):
        """
        Prediction step: Predicts the next state based on the current state and the time step (dt).

        :param dt: Time step as a float (seconds).
        """
        self.state = self.state_transition_function(self.state, dt)
        F = self.state_transition_jacobian(dt)

        # Update the error covariance matrix: P = F * P * F' + Q
        self.error_covariance = (
            np.dot(F, np.dot(self.error_covariance, F.T)) + self.process_covariance
        )

    def update(self, measurement, dt):
        """
        Update step: Incorporate the new measurement into the state estimate.
        Better handle velocity and acceleration as derived from altitude.

        :param measurement: The altitude measurement.
        :param dt: Time step as a float (seconds).
        """
        if dt == 0:
            self.state[1] = 0  # Set velocity to 0
            self.state[2] = 0  # Set acceleration to 0
            self.previous_altitude = self.state[0]
            return

        predicted_measurement = self.measurement_function(self.state)
        residual = np.array(measurement) - predicted_measurement

        H = self.measurement_jacobian()

        # Calculate Kalman gain: K = P * H' * inv(H * P * H' + R)
        S = np.dot(H, np.dot(self.error_covariance, H.T)) + self.measurement_covariance
        S_inv = np.linalg.inv(S)
        kalman_gain = np.dot(np.dot(self.error_covariance, H.T), S_inv)

        # Update the state: state = state + K * residual
        self.state = self.state + np.dot(kalman_gain, residual)

        # Update the error covariance matrix: P = (I - K * H) * P
        I = np.eye(len(self.state))
        self.error_covariance = np.dot(
            (I - np.dot(kalman_gain, H)), self.error_covariance
        )

        # Update velocity and acceleration based on altitude differences
        altitude = self.state[0]

        self.state[1] = (altitude - self.previous_altitude) / dt
        self.state[2] = (self.state[1] - self.previous_velocity) / dt

        self.previous_altitude = altitude
        self.previous_velocity = self.state[1]

    def get_state(self):
        """
        Return the current state estimate: List of floats [altitude, velocity, acceleration].
        """
        return self.state

    @staticmethod
    def state_transition_function(state, dt):
        """
        Predict the next state based on the current state and time step (dt).
        Assuming a simple model where altitude depends on velocity, and velocity depends on acceleration.

        :param state: List of floats representing the current state [altitude, velocity, acceleration].
        :param dt: Time step as a float (seconds).
        :return: List of floats representing the predicted next state [altitude, velocity, acceleration].
        """
        altitude, velocity, acceleration = state
        next_altitude = altitude + velocity * dt + 0.5 * acceleration * dt**2
        next_velocity = velocity + acceleration * dt
        next_acceleration = acceleration  # Assuming constant acceleration
        return np.array([next_altitude, next_velocity, next_acceleration])

    @staticmethod
    def state_transition_jacobian(dt):
        """
        Jacobian matrix of the state transition function.

        :param dt: Time step as a float (seconds).
        :return: 2D list representing the Jacobian matrix of partial derivatives.
        """
        return np.array(
            [
                [
                    1,
                    dt,
                    0.5 * dt**2,
                ],  # Partial derivatives of altitude with respect to state variables
                [0, 1, dt],  # Partial derivatives of velocity
                [0, 0, 1],  # Partial derivatives of acceleration
            ]
        )

    @staticmethod
    def measurement_function(state):
        """
        Predict the measurement (altitude) from the current state.

        :param state: List of floats representing the current state [altitude, velocity, acceleration].
        :return: List of floats representing the predicted measurement (altitude).
        """
        altitude, velocity, acceleration = state
        return np.array([altitude])

    @staticmethod
    def measurement_jacobian():
        """
        Jacobian matrix of the measurement function.

        :return: 2D list representing the Jacobian matrix for the measurement function.
        """
        return np.array(
            [[1, 0, 0]]
        )  # We only measure altitude, so the derivative is 1 with respect to altitude
