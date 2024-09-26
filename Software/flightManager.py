import time
import asyncio
import os
import pwmio
import board
import math
import digitalio

from kalman_filter import KalmanFilter
from adafruit_motor import servo

FLIGHT_STAGES = ("LAUNCHPAD", "ASCENT", "APOGEE", "DESCENT", "LANDED")
RELEASE_ALTITUDE = {
    "0": 30.48,  # 100 feet
    "1": 60.96,  # 200 feet
    "2": 91.44,  # 300 feet
    "3": 121.92,  # 400 feet
    "4": 152.4,  # 500 feet
    "5": 182.88,  # 600 feet
    "6": 213.36,  # 700 feet
    "7": 243.84,  # 800 feet
    "8": 274.32,  # 900 feet
    "9": 304.8,  # 1000 feet
    "10": 335.28,  # 1100 feet
    "11": 365.76,  # 1200 feet
    "12": 396.24,  # 1300 feet
    "13": 426.72,  # 1400 feet
    "14": 457.2,  # 1500 feet
    "15": 487.68,  # 1600 feet
}
LAUNCH_DETECT_ALTITUDE = (
    60.96  # Altitude threshold for launch detection in meters (200 ft)
)


class FlightManager:
    """The flight manager uses data from the altimeter and accelerometer to determine the flight stage"""

    def __init__(
        self, flight_data_filename: str = None, is_development: bool = None
    ) -> None:
        self.flight_data_filename = (
            flight_data_filename if flight_data_filename else None
        )
        self.is_development = is_development if is_development is not None else True

        self.setup_barometer()
        self.setup_servo()
        self.setup_rotary_switch()

        self.initialize_flight_variables()

    def setup_barometer(self):
        """Set up the barometer depending on development or production mode."""
        if self.is_development:
            from mock_barometer import MockBarometer

            self.barometer = MockBarometer()
            print("Using mock barometer for development")
        else:
            from barometer import Barometer

            self.barometer = Barometer()
            print("Using real barometer for production")

    def setup_servo(self):
        """Set up the servo motor, with error handling."""
        try:
            pwm = pwmio.PWMOut(board.GP15, frequency=50)
            servo_range = 120
            self.my_servo = servo.Servo(
                pwm, min_pulse=900, max_pulse=2100, actuation_range=servo_range
            )
            self.my_servo.angle = 0
        except Exception as e:
            print(f"Error initializing servo: {e}")

    def setup_rotary_switch(self):
        """Set up the rotary switch pins."""
        self.pins = {
            "pin0": digitalio.DigitalInOut(board.GP21),
            "pin1": digitalio.DigitalInOut(board.GP20),
            "pin2": digitalio.DigitalInOut(board.GP19),
            "pin3": digitalio.DigitalInOut(board.GP18),
        }

        for pin in self.pins.values():
            pin.direction = digitalio.Direction.INPUT
            pin.pull = digitalio.Pull.UP

    def initialize_flight_variables(self):
        """Initialize flight variables and state."""
        self.flight_stage = FLIGHT_STAGES[
            0
        ]  # Set the initial flight stage to LAUNCHPAD
        self.ground_altitude = 0.0
        self.max_altitude = 0.0

        self.initial_pressure = self.barometer.get_sensor_data()[0]
        self.previous_velocity = 0.0
        self.previous_acceleration = 0.0

        self.release_enabled = False
        self.is_flight_started = False

        self.vertical_velocity = 0.0
        self.vertical_acceleration = 0.0
        self.speed_of_sound = 343.0  # Speed of sound in air at 20°C (m/s)

        # Kalman filters for altitude, velocity, and acceleration
        self.kalman_filter_altitude = KalmanFilter(50, 0.5, 0.5)
        self.kalman_filter_velocity = KalmanFilter(0.1, 1.0, 1.0)
        self.kalman_filter_acceleration = KalmanFilter(0.01, 10.0, 10.0)

    async def main(self) -> None:
        self.ground_altitude = await self.collect_ground_altitude()
        self.open_flight_data_file()

        previous_altitude = self.ground_altitude
        previous_time = time.monotonic()
        previous_velocity = 0.0
        previous_smoothed_altitude = self.ground_altitude
        smoothed_altitude = self.ground_altitude

        while True:
            current_time = time.monotonic()
            pressure, altitude, temperature = self.barometer.get_sensor_data()
            altitude_agl = altitude - self.ground_altitude
            delta_time = current_time - previous_time

            smoothed_altitude = self.kalman_filter_altitude.update(altitude_agl)
            self.speed_of_sound = self.calculate_speed_of_sound(temperature)

            self.calculate_velocity(previous_altitude, smoothed_altitude, delta_time)
            # TODO: Implement mach lockout
            self.calculate_acceleration(
                previous_velocity, self.vertical_velocity, delta_time
            )

            if self.update_flight_stage(smoothed_altitude, previous_smoothed_altitude):
                break

            previous_smoothed_altitude = smoothed_altitude

            # Log the flight data to a file or print it
            if not self.is_development:
                stat_result = os.stat(self.flight_data_filename)
                file_size = stat_result[6]

                # Log the flight data if the file size is less than 12MB and the flight has started
                if file_size <= 12000000 and self.is_flight_started:
                    self.log_flight_data(
                        altitude_agl,
                        smoothed_altitude,
                        pressure,
                        temperature,
                        self.vertical_velocity,
                        self.vertical_acceleration,
                        self.flight_stage,
                        delta_time,
                    )
            else:
                print(
                    f"{time.monotonic()},{altitude_agl:.2f},{smoothed_altitude:.2f},{pressure:.2f},{temperature:.2f},{self.vertical_velocity:.2f},{self.vertical_acceleration:.2f},{self.flight_stage},{delta_time:.2f}"
                )

            self.update_max_altitude(smoothed_altitude)

            previous_altitude, previous_time, previous_velocity = (
                smoothed_altitude,
                current_time,
                self.vertical_velocity,
            )

            await asyncio.sleep(0.1)  # TODO Remove when ready to run on hardware

    async def collect_ground_altitude(self) -> float:
        """Collect ground altitude by averaging 10 samples over 5 seconds."""
        samples = []
        for _ in range(10):
            # Collect a single altitude sample
            samples.append(self.barometer.get_sensor_data()[1])
            # Wait for 0.5 seconds before collecting the next sample
            await asyncio.sleep(0.5)
        # Calculate the average of the collected samples
        return sum(samples) / len(samples)

    def open_flight_data_file(self):
        """Open the flight data file for logging, if in production mode."""
        if not self.is_development:
            try:
                with open(self.flight_data_filename, "w") as file:
                    file.write(
                        "Timestamp,Altitude AGL,Altitude Smoothed,Pressure,Temperature,Vertical Velocity,Vertical Acceleration,FlightStage, Delta Time\n"
                    )
                print(
                    f"Opened flight data file: {self.flight_data_filename} for logging"
                )
            except OSError as err:
                print("Unable to open flight data file:", err)
                print("Switching to development mode...")
                self.is_development = True
        else:
            print("Running in development mode, skipping file logging")

    def update_flight_stage(self, smoothed_altitude, previous_smoothed_altitude):
        """Update the flight stage based on altitude and velocity."""
        if self.flight_stage == FLIGHT_STAGES[0]:  # LAUNCHPAD
            self.handle_launchpad_stage(smoothed_altitude)
        elif self.flight_stage == FLIGHT_STAGES[1]:  # ASCENT
            if smoothed_altitude < previous_smoothed_altitude:  #! Apogee detected
                self.flight_stage = FLIGHT_STAGES[2]  # * SET APOGEE
                self.release_enabled = True
        elif self.flight_stage == FLIGHT_STAGES[2]:  # APOGEE
            if smoothed_altitude < previous_smoothed_altitude:  #! Descent detected
                self.flight_stage = FLIGHT_STAGES[3]  # * SET DESCENT
        elif self.flight_stage == FLIGHT_STAGES[3]:  # DESCENT
            self.handle_descent_stage(smoothed_altitude)
        elif self.flight_stage == FLIGHT_STAGES[4]:  # LANDED
            print("Rocket has landed")
            print(f"Max altitude: {self.max_altitude:.2f} m")
            return True
        return False

    def handle_launchpad_stage(self, smoothed_altitude):
        """Handle the launchpad stage, detect launch, and lock the release altitude."""
        if not self.is_flight_started:
            self.selected_release_altitude = self.read_rotary_switch()
            print(f"Current release altitude: {self.selected_release_altitude} meters")
        if smoothed_altitude > LAUNCH_DETECT_ALTITUDE:  #! Launch detected
            self.flight_stage = FLIGHT_STAGES[1]  # * SET ASCENT
            self.is_flight_started = True
            print(f"Release altitude locked: {self.selected_release_altitude} meters")

    def handle_descent_stage(self, smoothed_altitude):
        """Handle the descent stage and trigger parachute release if needed."""
        if (
            -0.25 <= self.vertical_velocity <= 0.25
            and smoothed_altitude <= LAUNCH_DETECT_ALTITUDE
        ):
            self.flight_stage = FLIGHT_STAGES[4]  # * SET LANDED
        elif (
            self.release_enabled and smoothed_altitude <= self.selected_release_altitude
        ):
            self.trigger_chute_release()
            self.release_enabled = False

    def calculate_speed_of_sound(self, temperature_celsius: float) -> float:
        """
        Calculates the speed of sound in air based on the temperature in Celsius.

        Parameters:
        temperature_celsius (float): The temperature in Celsius.

        Returns:
        float: Speed of sound in meters per second.
        """
        # Constants
        gamma = 1.4  # Adiabatic index for air
        R = 287.05  # Specific gas constant for dry air (J/(kg·K))

        # Convert Celsius to Kelvin
        temperature_kelvin = temperature_celsius + 273.15

        # Speed of sound formula
        speed = math.sqrt(gamma * R * temperature_kelvin)

        return speed

    def update_max_altitude(self, altitude):
        """Update the maximum altitude reached during the flight"""
        self.max_altitude = max(self.max_altitude, altitude)

    def calculate_velocity(self, previous_altitude, current_altitude, delta_time):
        """
        Calculate the vertical velocity based on altitude readings and apply Kalman filtering.

        :param previous_altitude: Altitude at the previous time step (in meters)
        :param current_altitude: Altitude at the current time step (in meters)
        :param delta_time: Time difference between the two altitude readings (in seconds)
        :return: None
        """
        if delta_time == 0:
            return  # Avoid division by zero if time difference is zero

        # Calculate the raw vertical velocity (change in altitude over time)
        raw_velocity = (current_altitude - previous_altitude) / delta_time

        # Update Kalman filter for velocity
        self.vertical_velocity = self.kalman_filter_velocity.update(raw_velocity)

    def calculate_acceleration(self, previous_velocity, current_velocity, delta_time):
        """
        Calculate the vertical acceleration based on velocity readings and apply filtering.

        :param previous_velocity: Velocity at the previous time step (in meters/second)
        :param current_velocity: Velocity at the current time step (in meters/second)
        :param delta_time: Time difference between the two velocity readings (in seconds)
        :return: None
        """
        if delta_time == 0:
            return  # Avoid division by zero if time difference is zero

        # Calculate the raw acceleration as the change in velocity over time
        raw_acceleration = (current_velocity - previous_velocity) / delta_time

        # Update Kalman filter for acceleration
        self.vertical_acceleration = self.kalman_filter_acceleration.update(
            raw_acceleration
        )

    def log_flight_data(
        self,
        altitude_agl,
        smoothed_altitude,
        pressure,
        temperature,
        vertical_velocity,
        vertical_acceleration,
        flight_stage,
        delta_time,
    ):
        try:
            # Log the flight data to a file
            with open(self.flight_data_filename, "a") as file:
                file.write(
                    f"{time.monotonic()},{altitude_agl:.2f},{smoothed_altitude:.2f},{pressure:.2f},{temperature:.2f},{vertical_velocity:.2f},{vertical_acceleration:.2f},{flight_stage},{delta_time:.2f}\n"
                )
        except OSError:  # Typically when the filesystem isn't writable...
            print("Filesystem not writable, skipping flight data logging")
        except Exception as e:
            print(f"Error writing to filesystem: {e}")
            raise e

    def trigger_chute_release(self):
        """Trigger the parachute release mechanism"""
        print("Triggering parachute release...")
        self.my_servo.angle = 45

    def read_rotary_switch(self):
        """Reads the rotary switch and returns the corresponding altitude"""
        # Read the binary value from the rotary switch
        switch_value = (
            (self.pins["pin3"].value << 3)
            | (self.pins["pin2"].value << 2)
            | (self.pins["pin1"].value << 1)
            | self.pins["pin0"].value
        )
        # Invert the switch value to correct the offset
        switch_value = switch_value ^ 0xF
        # Map the switch value to a release altitude
        release_altitude = RELEASE_ALTITUDE.get(
            str(switch_value), RELEASE_ALTITUDE["0"]
        )
        return release_altitude
