import math


class MockBarometer:
    """Mock class to simulate the DPS310 sensor during a model rocket flight"""

    def __init__(self, boost_duration=1, coast_duration=5, descent_velocity=100):
        """
        :param boost_duration: Duration of the boost phase (seconds).
        :param coast_duration: Duration of the coast phase (seconds).
        :param descent_velocity: Descent velocity (m/s).
        """
        self.boost_duration = boost_duration
        self.coast_duration = coast_duration
        self.descent_velocity = descent_velocity
        self.current_time = 0  # Time starts at 0
        self.max_altitude = 0
        self.vertical_velocity = 0
        self.altitude = 0
        self.ground_pressure = 1013.25  # Pressure at sea level (hPa)
        self.ground_temperature = 288.15  # Temperature at sea level (Kelvin)

    def simulate_flight(self):
        """Simulate the flight trajectory based on phases"""
        if self.current_time <= self.boost_duration:
            # Boost phase (constant acceleration of 10Gs)
            acceleration = 98.1  # 10 Gs
            self.vertical_velocity += (
                acceleration * 0.1
            )  # Increment velocity every 0.1 second
            self.altitude += (
                self.vertical_velocity * 0.1
            )  # Update altitude based on velocity
        elif (
            self.boost_duration
            < self.current_time
            <= self.boost_duration + self.coast_duration
        ):
            # Coast phase (only gravity slowing down the rocket)
            acceleration = -9.81  # Gravity acting downward
            self.vertical_velocity += (
                acceleration * 0.1
            )  # Decrease velocity due to gravity
            self.altitude += (
                self.vertical_velocity * 0.1
            )  # Update altitude based on velocity
            if self.vertical_velocity <= 0:  # Reached apogee
                self.max_altitude = self.altitude  # Mark max altitude
        else:
            # Descent phase (constant descent velocity)
            if self.altitude > 0:
                self.vertical_velocity = (
                    -self.descent_velocity
                )  # Constant descent velocity
                self.altitude += self.vertical_velocity * 0.1  # Update altitude
                self.altitude = max(
                    0, self.altitude
                )  # Ensure altitude doesn't go below ground level

    def simulate_pressure(self, altitude):
        """Simulate pressure based on altitude using the ISA barometric formula"""
        if altitude < 11000:  # Troposphere (up to 11km)
            T = (
                288.15 - 0.0065 * altitude
            )  # Temperature decreases linearly with altitude
            P = self.ground_pressure * (T / 288.15) ** (-9.81 / (0.0065 * 287.05))
        elif 11000 <= altitude < 20000:  # Lower stratosphere (11km to 20km)
            T = 216.65  # Constant temperature in this layer
            P = (
                self.ground_pressure
                * 0.22336
                * math.exp(-9.81 * (altitude - 11000) / (287.05 * T))
            )
        else:
            T = 216.65 + 0.001 * (altitude - 20000)  # Gradual increase in temperature
            P = (
                self.ground_pressure
                * 0.054032
                * math.exp(-9.81 * (altitude - 20000) / (287.05 * T))
            )

        return P

    def simulate_temperature(self, altitude):
        """Simulate temperature based on the ISA model"""
        if altitude < 11000:  # Troposphere (up to 11km)
            temperature = 288.15 - 0.0065 * altitude  # Lapse rate: -6.5°C per km
        elif 11000 <= altitude < 20000:  # Lower stratosphere (11km to 20km)
            temperature = 216.65  # Constant temperature in this layer
        else:
            temperature = 216.65 + 0.001 * (
                altitude - 20000
            )  # Gradual increase above 20km

        return temperature

    def get_sensor_data(self) -> tuple[float, float, float]:
        """Get current simulated sensor data"""
        self.simulate_flight()
        pressure = self.simulate_pressure(self.altitude)
        temperature = self.simulate_temperature(self.altitude)

        # Increment time for the next reading
        self.current_time += 0.1  # Simulate a time step (e.g., 100ms per reading)

        return pressure, self.altitude, temperature

    def init_barometer(self):
        """Mock initialization process"""
        print("Mock Barometer initialized for rocket flight simulation.")
