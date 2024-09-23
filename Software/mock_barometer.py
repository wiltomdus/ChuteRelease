import math

class MockBarometer:
    """Mock class to simulate the DPS310 sensor during a model rocket flight"""

    def __init__(self, max_altitude=500, flight_duration=20):
        """
        :param max_altitude: Maximum altitude in meters for the parabolic flight.
        :param flight_duration: Duration of the flight in seconds.
        """
        self.max_altitude = max_altitude
        self.flight_duration = flight_duration
        self.current_time = 0  # Time starts at 0
        self.ground_pressure = 1013.25  # Pressure at sea level (hPa)
        self.ground_temperature = 25.0  # Temperature at launch (Celsius)

    def simulate_altitude(self):
        """Simulate altitude based on a parabolic flight trajectory"""
        # Simple parabolic equation to simulate a flight (y = -ax^2 + bx)
        # x is the current time, scaled between 0 and 1 (0: launch, 1: end of flight)
        normalized_time = self.current_time / self.flight_duration
        if normalized_time > 1:
            normalized_time = 1

        # Parabolic altitude curve: maximum at the midpoint (apogee)
        altitude = -4 * self.max_altitude * (normalized_time - 0.5)**2 + self.max_altitude
        return max(0, altitude)  # Altitude should never be negative

    def simulate_pressure(self, altitude):
        """Simulate pressure based on altitude"""
        # Barometric formula approximation for pressure based on altitude
        pressure = self.ground_pressure * math.exp(-altitude / 8500)  # Scale height ~8500 meters
        return pressure

    def simulate_temperature(self, altitude):
        """Simulate temperature drop with altitude (rough estimation)"""
        # Approximate temperature lapse rate in the troposphere: -6.5°C per km
        temperature = self.ground_temperature - (altitude / 1000 * 6.5)
        return max(-56.5, temperature)  # Temperature should not go below the tropopause (~ -56.5°C)

    def get_sensor_data(self) -> tuple[float, float, float]:
        """Get current simulated sensor data"""
        altitude = self.simulate_altitude()
        pressure = self.simulate_pressure(altitude)
        temperature = self.simulate_temperature(altitude)

        # Increment time for the next reading
        self.current_time += 0.1  # Simulate a time step (e.g., 100ms per reading)

        return pressure, altitude, temperature

    def init_barometer(self):
        """Mock initialization process"""
        print("Mock Barometer initialized for rocket flight simulation.")
