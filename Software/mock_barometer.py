class MockBarometer:
    """Mock class to simulate a simple linear altitude change with a controlled descent"""

    def __init__(self, max_altitude=1000, ascent_duration=10, descent_rate=10):
        """
        :param max_altitude: Maximum altitude reached in meters.
        :param ascent_duration: Time to reach the maximum altitude in seconds.
        :param descent_rate: Descent rate in meters per second.
        """
        self.max_altitude = max_altitude
        self.ascent_duration = ascent_duration
        self.descent_rate = descent_rate
        self.current_time = 0  # Time starts at 0
        self.altitude = 0
        self.vertical_velocity = 0
        self.ground_pressure = 1013.25  # Pressure at sea level (hPa)
        self.calibration_points = 10

    def simulate_flight(self):
        """Simulate a simple flight with a linear ascent and constant descent."""

        if self.calibration_points >= 0:
            # Calibration phase: return ground values
            self.calibration_points -= 1
            return self.ground_pressure, 0, self.simulate_temperature(0)

        time_step = 0.01  # Time step for simulation (seconds)

        if self.current_time <= self.ascent_duration:
            # Ascent phase: parabolic increase in altitude
            self.altitude = (
                (self.current_time / self.ascent_duration) ** 3
            ) * self.max_altitude
        else:
            # Descent phase: constant descent rate
            self.altitude -= self.descent_rate * time_step
            self.altitude = max(
                0, self.altitude
            )  # Ensure altitude doesn't go below ground level

        # Increment time for the next reading
        self.current_time += time_step

    def simulate_pressure(self, altitude):
        """Simulate pressure based on altitude using the ISA barometric formula"""
        if altitude < 11000:  # Troposphere (up to 11km)
            T = (
                288.15 - 0.0065 * altitude
            )  # Temperature decreases linearly with altitude
            P = self.ground_pressure * (T / 288.15) ** (-9.81 / (0.0065 * 287.05))
        else:
            P = 1013.25  # Use default ground pressure if altitude exceeds normal ranges

        return P

    def simulate_temperature(self, altitude):
        """Simulate temperature based on altitude"""
        return 15.0  # Simplified constant temperature for this basic simulation

    def get_sensor_data(self) -> tuple[float, float, float]:
        """Get current simulated sensor data"""

        self.simulate_flight()
        pressure = self.simulate_pressure(self.altitude)
        temperature = self.simulate_temperature(self.altitude)

        # Return pressure, altitude, and temperature
        return pressure, self.altitude, temperature

    def init_barometer(self):
        """Mock initialization process"""
        print("Mock Barometer initialized for simplified flight simulation.")
