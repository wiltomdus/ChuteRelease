import ulab.numpy as np  # Import ulab numpy-like functionality


class ExtendedKalmanFilter:
    def __init__(self):
        """
        Initialize the EKF with the initial state, process covariance, and measurement covariance.

        :param initial_state: List of floats [altitude, velocity].
        :param process_covariance: 2D list representing the process covariance matrix (Q).
        :param measurement_covariance: List representing the measurement covariance matrix (R).
        """
        self.state = np.array([0, 0])  # Initial state: [altitude, velocity]

        self.process_covariance = np.array([[1, 0], [0, 1]])

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

    def update(self, measurement, dt, speed_of_sound_threshold=274.4):
        """
        Update step: Incorporate the new measurement into the state estimate.

        :param measurement: The altitude measurement.
        :param dt: Time step as a float (seconds).
        """
        if dt == 0:
            self.state[1] = 0  # Set velocity to 0
            self.previous_altitude = self.state[0]
            return

        # Calculate velocity to detect supersonic flight (rough threshold)
        if abs(self.state[1]) > speed_of_sound_threshold:
            # Increase measurement covariance (less trust in measurements)
            self.measurement_covariance = np.array([[5.0]])
            print("Supersonic flight detected!")
        else:
            # Reset measurement covariance (normal mode)
            self.measurement_covariance = np.array([[0.5]])
            print("Normal flight mode.")

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

        self.previous_altitude = altitude
        self.previous_velocity = self.state[1]

    def get_state(self):
        """
        Return the current state estimate: List of floats [altitude, velocity].
        """
        return self.state

    @staticmethod
    def state_transition_function(state, dt):
        """
        Predict the next state based on the current state and time step (dt).
        Altitude is updated based on velocity, and velocity remains constant.
        :param state: List of floats representing the current state [altitude, velocity].
        :param dt: Time step as a float (seconds).
        :return: List of floats representing the predicted next state [altitude, velocity].
        """
        altitude, velocity = state
        next_altitude = altitude + velocity * dt
        next_velocity = velocity  # Assuming no external forces change velocity
        return np.array([next_altitude, next_velocity])

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
                ],  # Partial derivatives of altitude with respect to state variables
                [0, 1],  # Partial derivatives of velocity
            ]
        )

    @staticmethod
    def measurement_function(state):
        """
        Predict the measurement (altitude) from the current state.
        :param state: List of floats representing the current state [altitude, velocity].
        :return: List of floats representing the predicted measurement (altitude).
        """
        altitude, velocity = state
        return np.array([altitude])

    @staticmethod
    def measurement_jacobian():
        """
        Jacobian matrix of the measurement function.
        :return: 2D list representing the Jacobian matrix for the measurement function.
        """
        return np.array([[1, 0]])  # We only measure altitude
