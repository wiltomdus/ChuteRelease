import time
import asyncio
import os


from barometer import Barometer
from kalman_filter import KalmanFilter

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
  "1500": 457.2
}

# LAUNCH_DETECT_ALTITUDE = 60.96  # Altitude threshold for launch detection in meters (200 ft)
LAUNCH_DETECT_ALTITUDE = 1 #! DEBUGGING

class FlightManager:
    """The flight manager uses data from the altimeter and accelerometer to determine the flight stage"""

    def __init__(self) -> None:
        self.barometer = Barometer()
        
        self.flight_stage = FLIGHT_STAGES[0]  # Set the initial flight stage to LAUNCHPAD
        self.ground_altitude: float = 0.0
        self.max_altitude: float = 0.0
        
        self.initial_pressure = self.barometer.get_sensor_data()[0]
        self.previous_velocity = 0.0
        self.previous_acceleration = 0.0
        self.alpha = 0.1  # Smoothing factor for EMA
        
        self.release_enabled = False
        
        self.vertical_velocity = 0.0
        self.vertical_acceleration = 0.0
        
        self.altitude_buffer = []  # Buffer to hold recent altitude readings
        self.buffer_size = 5  # Number of readings to average (adjust as needed)

        # Kalman filters for altitude and velocity
        self.kalman_filter_altitude = KalmanFilter(
            process_variance=1e-3,  # Increase process variance
            measurement_variance=0.1**2,
            estimated_measurement_variance=0.1
        )

        self.kalman_filter_velocity = KalmanFilter(
            process_variance=1e-2,  # Increase process variance
            measurement_variance=0.1**2,
            estimated_measurement_variance=0.1
)


    async def fly(self, is_development) -> None:
        """State machine to determine the current flight stage"""
        
       
        # Get the ground altitude by averaging 10 samples over 5 seconds
        samples = []
        for _ in range(10):
            samples.append(self.barometer.get_sensor_data()[1])
            await asyncio.sleep(0.5)
            
        self.ground_altitude = sum(samples) / len(samples)

        # Create and open the flight data file
        if not is_development:
            try:
                self.flight_data_file = open("/data/flight-data.csv", "w")
                self.flight_data_file.write(
                    "Timestamp,Altitude,Pressure,Temperature,Vertical Velocity, Vertical Acceleration,FlightStage\n"
                )
                print("Opened flight data file...")
            except OSError as err:
                # if the filesystem isn't writable, skip logging to file
                print("Unable to open flight data file:", err)
                print("Switching to development mode...")
                is_development = True
                
        previous_altitude = self.ground_altitude
        previous_time = time.monotonic()
        previous_velocity = 0.0
        previous_smoothed_altitude = self.ground_altitude
        smoothed_altitude = self.ground_altitude
        
        while True:

            current_time = time.monotonic()
            
            # Get the altimeter data
            pressure, altitude, temperature = self.barometer.get_sensor_data()
            
            altitude_agl = altitude - self.ground_altitude  # Adjust altitude to ground level
                      
            # Calculate the time delta
            delta_time = current_time - previous_time
        
            # Update Kalman filter for altitude
            smoothed_altitude = self.kalman_filter_altitude.update(altitude_agl)
            
            # Calculate and filter velocity
            self.calculate_velocity(previous_altitude, smoothed_altitude, delta_time)
            
            # Calculate and filter acceleration based on the new velocity
            self.calculate_acceleration(previous_velocity, self.vertical_velocity, delta_time)
                      
            # Update the previous values
            previous_altitude = altitude_agl
            previous_time = current_time
            previous_velocity = self.vertical_velocity
            
            # Log the flight data to a file or print it
            if not is_development and os.stat("/data/flight-data.csv").st_size <= 12000000:
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
                    f"{time.monotonic()},{altitude_agl:.2f},{pressure:.2f},{temperature:.2f},{self.vertical_velocity:.2f},{self.vertical_acceleration:.2f},{self.flight_stage},{delta_time:.2f}"
                )

            # Update the maximum values
            self.max_altitude = max(self.max_altitude, altitude_agl)

            # Determine the current flight stage
            if self.flight_stage == FLIGHT_STAGES[0]:  # LAUNCHPAD
                if altitude_agl > LAUNCH_DETECT_ALTITUDE:  # Launch detected
                    self.flight_stage = FLIGHT_STAGES[1]  # Set the flight stage to ASCENT
                    print(f"Flight stage: {self.flight_stage}")
                    continue
                await asyncio.sleep(0)

            elif self.flight_stage == FLIGHT_STAGES[1]:  # ASCENT
                if smoothed_altitude < previous_smoothed_altitude:  # Apogee detected when altitude starts to decrease
                    print(f"Apogee detected at {previous_smoothed_altitude}m")
                    self.flight_stage = FLIGHT_STAGES[2]  # Set the flight stage to APOGEE
                    self.release_enabled = True
                    # Sleep for 1 second to prevent false detection of descent due to separation
                    #await asyncio.sleep(1)
                    continue
                await asyncio.sleep(0)

            elif self.flight_stage == FLIGHT_STAGES[2]:  # APOGEE
                if smoothed_altitude < previous_smoothed_altitude:  # Confirm descent has started
                    self.flight_stage = FLIGHT_STAGES[3]  # Set the flight stage to DESCENT
                    print(f"Flight stage: {self.flight_stage}")
                    continue
                await asyncio.sleep(0)

            elif self.flight_stage == FLIGHT_STAGES[3]:  # DESCENT
                if (-0.5 <= altitude <= 0.5):
                    self.flight_stage = FLIGHT_STAGES[4]  # Set the flight stage to LANDED
                    print(f"Flight stage: {self.flight_stage}")
                    break
                elif self.release_enabled:
                    if altitude_agl <= 0.5:
                        print("Release chute at 100ft/30.48m")
                        self.trigger_chute_release()
                        self.release_enabled = False
                await asyncio.sleep(0)

            # Update previous smoothed altitude for the next iteration
            previous_smoothed_altitude = smoothed_altitude

        if not is_development:
            try:
                self.flight_data_file.close()
            except OSError as err:
                print("Unable to close flight data file:", err)
        
    def trigger_chute_release(self):
        """Trigger the parachute release mechanism"""
        print("Triggering parachute release...")
        pass

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
            self.flight_data_file.write(
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
        
        # Apply EMA filter to the acceleration
        self.vertical_acceleration = self.alpha * raw_acceleration + (1 - self.alpha) * self.vertical_acceleration

    def update_altitude_buffer(self, altitude_agl):
        """Update the altitude buffer and calculate the moving average."""
        if len(self.altitude_buffer) >= self.buffer_size:
            self.altitude_buffer.pop(0)  # Remove the oldest reading if buffer is full
        self.altitude_buffer.append(altitude_agl)  # Add the new altitude reading
        
        # Calculate the moving average
        smoothed_altitude = sum(self.altitude_buffer) / len(self.altitude_buffer)
        
        return smoothed_altitude
