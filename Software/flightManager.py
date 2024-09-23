import time
import asyncio
import os
import pwmio
import board

from kalman_filter import KalmanFilter
from adafruit_motor import servo


FLIGHT_STAGES = (
    "LAUNCHPAD",
    "ASCENT",
    "APOGEE",
    "DESCENT",
    "LANDED",
)

RELEASE_ALTITUDE = {
    "100": 30.48,
    "200": 60.96,
    "300": 91.44,
    "400": 121.92,
    "500": 152.4,
    "600": 182.88,
    "700": 213.36,
    "800": 243.84,
    "900": 274.32,
    "1000": 304.8,
    "1100": 335.28,
    "1200": 365.76,
    "1300": 396.24,
    "1400": 426.72,
    "1500": 457.2,
}

LAUNCH_DETECT_ALTITUDE = (
    60.96  # Altitude threshold for launch detection in meters (200 ft)
)


class FlightManager:
    """The flight manager uses data from the altimeter and accelerometer to determine the flight stage"""

    def __init__(
        self, flight_data_filename: str = None, is_development: bool = None
    ) -> None:
        if flight_data_filename is not None:
            self.flight_data_filename = flight_data_filename

        if is_development is not None:
            self.is_development = is_development
        else:
            self.is_development = True

        if self.is_development:
            from Software.mock_barometer import MockBarometer

            self.barometer = MockBarometer()
            print("Using mock barometer for development")
        else:
            from barometer import Barometer

            self.barometer = Barometer()
            print("Using real barometer for production")

        self.flight_stage = FLIGHT_STAGES[
            0
        ]  # Set the initial flight stage to LAUNCHPAD
        self.ground_altitude: float = 0.0
        self.max_altitude: float = 0.0

        self.initial_pressure = self.barometer.get_sensor_data()[0]
        self.previous_velocity = 0.0
        self.previous_acceleration = 0.0
        self.alpha = 0.1  # Smoothing factor for EMA

        self.release_enabled = False
        self.is_flight_started = False

        self.vertical_velocity = 0.0
        self.vertical_acceleration = 0.0

        self.altitude_buffer = []  # Buffer to hold recent altitude readings
        self.buffer_size = 5  # Number of readings to average (adjust as needed)

        # Kalman filters for altitude and velocity
        self.kalman_filter_altitude = KalmanFilter(
            process_variance=50,
            measurement_variance=0.5,
            estimated_measurement_variance=0.5,
        )

        self.kalman_filter_velocity = KalmanFilter(
            process_variance=0.1,
            measurement_variance=1.0,
            estimated_measurement_variance=1.0,
        )

        self.kalman_filter_acceleration = KalmanFilter(
            process_variance=0.01,
            measurement_variance=10.0,
            estimated_measurement_variance=10.0,
        )

        pwm = pwmio.PWMOut(board.GP15, frequency=50)
        servo_range = 120
        self.my_servo = servo.Servo(
            pwm,
            min_pulse=900,  # Minimum pulse width in microseconds
            max_pulse=2100,  # Maximum pulse width in microseconds
            actuation_range=servo_range,  # Set to 120 degrees
        )
        self.my_servo.angle = 0

    async def fly(self) -> None:
        """State machine to determine the current flight stage"""
        # Get the ground altitude by averaging 10 samples over 5 seconds
        samples = []
        for _ in range(10):
            samples.append(self.barometer.get_sensor_data()[1])
            await asyncio.sleep(0.5)

        self.ground_altitude = sum(samples) / len(samples)

        # Create and open the flight data file
        if not self.is_development:
            try:
                with open(self.flight_data_filename, "w") as file:
                    file.write(
                        "Timestamp,Altitude,Pressure,Temperature,Vertical Velocity, Vertical Acceleration,FlightStage\n"
                    )
                print(
                    f"Opened flight data file: {self.flight_data_filename} for logging"
                )
            except OSError as err:
                # if the filesystem isn't writable, skip logging to file
                print("Unable to open flight data file:", err)
                print("Switching to development mode...")
                self.is_development = True

        previous_altitude = self.ground_altitude
        previous_time = time.monotonic()
        previous_velocity = 0.0
        previous_smoothed_altitude = self.ground_altitude
        smoothed_altitude = self.ground_altitude

        while True:

            current_time = time.monotonic()

            # Get the altimeter data
            pressure, altitude, temperature = self.barometer.get_sensor_data()

            altitude_agl = (
                altitude - self.ground_altitude
            )  # Adjust altitude to ground level

            # Calculate the time delta
            delta_time = current_time - previous_time

            # Update Kalman filter for altitude
            smoothed_altitude = self.kalman_filter_altitude.update(altitude_agl)

            # Calculate and filter velocity
            self.calculate_velocity(previous_altitude, smoothed_altitude, delta_time)

            # Calculate and filter acceleration based on the new velocity
            self.calculate_acceleration(
                previous_velocity, self.vertical_velocity, delta_time
            )

            # Update the previous values
            previous_altitude = smoothed_altitude
            previous_time = current_time
            previous_velocity = self.vertical_velocity

            self.update_max_altitude(altitude_agl)

            # Log the flight data to a file or print it
            if not self.is_development:
                stat_result = os.stat(self.flight_data_filename)
                file_size = stat_result[6]

                if file_size <= 12000000 and self.is_flight_started:
                    self.log_flight_data(
                        altitude_agl,
                        pressure,
                        temperature,
                        self.vertical_velocity,
                        self.vertical_acceleration,
                        self.flight_stage,
                    )
            else:
                print(
                    f"{time.monotonic()},{altitude_agl:.2f},{smoothed_altitude:.2f},{pressure:.2f},{temperature:.2f},{self.vertical_velocity:.2f},{self.vertical_acceleration:.2f},{self.flight_stage},{delta_time:.2f}"
                )

            # Update the maximum values
            self.max_altitude = max(self.max_altitude, smoothed_altitude)

            # Determine the current flight stage
            if self.flight_stage == FLIGHT_STAGES[0]:  # LAUNCHPAD
                if smoothed_altitude > LAUNCH_DETECT_ALTITUDE:  # Launch detected
                    self.flight_stage = FLIGHT_STAGES[
                        1
                    ]  # Set the flight stage to ASCENT
                    self.is_flight_started = True
                    continue
                await asyncio.sleep(0)

            elif self.flight_stage == FLIGHT_STAGES[1]:  # ASCENT
                if (
                    smoothed_altitude < previous_smoothed_altitude
                ):  # Apogee detected when altitude starts to decrease
                    self.flight_stage = FLIGHT_STAGES[
                        2
                    ]  # Set the flight stage to APOGEE
                    self.release_enabled = True
                    # Sleep for 1 second to prevent false detection of descent due to separation
                    # await asyncio.sleep(1)
                    continue
                await asyncio.sleep(0)

            elif self.flight_stage == FLIGHT_STAGES[2]:  # APOGEE
                if (
                    smoothed_altitude < previous_smoothed_altitude
                ):  # Confirm descent has started
                    self.flight_stage = FLIGHT_STAGES[
                        3
                    ]  # Set the flight stage to DESCENT
                    continue
                await asyncio.sleep(0)

            elif self.flight_stage == FLIGHT_STAGES[3]:  # DESCENT
                if -0.25 <= self.vertical_velocity <= 0.25:
                    self.flight_stage = FLIGHT_STAGES[
                        4
                    ]  # Set the flight stage to LANDED
                    continue
                elif self.release_enabled:
                    if smoothed_altitude <= RELEASE_ALTITUDE["300"]:
                        self.trigger_chute_release()
                        self.release_enabled = False
                await asyncio.sleep(0)

            elif self.flight_stage == FLIGHT_STAGES[4]:  # LANDED
                print("Rocket has landed")
                print(f"Max altitude: {self.max_altitude:.2f} m")
                break

            # Update previous smoothed altitude for the next iteration
            previous_smoothed_altitude = smoothed_altitude
            await asyncio.sleep(0.1)

    def trigger_chute_release(self):
        """Trigger the parachute release mechanism"""
        print("Triggering parachute release...")
        self.my_servo.angle = 45

    def log_flight_data(
        self,
        altitude,
        pressure,
        temperature,
        vertical_velocity,
        vertical_acceleration,
        flight_stage,
    ):
        try:
            # Log the flight data to a file
            with open(self.flight_data_filename, "a") as file:
                file.write(
                    f"{time.monotonic()},{altitude:.2f},{pressure:.2f},{temperature:.2f},{vertical_velocity:.2f},{vertical_acceleration:.2f},{flight_stage}\n"
                )
        except OSError:  # Typically when the filesystem isn't writable...
            print("Filesystem not writable, skipping flight data logging")
        except Exception as e:
            print(f"Error writing to filesystem: {e}")
            raise e

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

    def update_max_altitude(self, altitude):
        """Update the maximum altitude reached during the flight"""
        self.max_altitude = max(self.max_altitude, altitude)
