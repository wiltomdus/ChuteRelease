import json


class FlightData:
    def __init__(
        self,
        flight_time,
        altitude_agl,
        smoothed_altitude_agl,
        pressure,
        temperature,
        vertical_velocity,
        flight_stage,
        delta_time,
    ):
        self.flight_time = round(flight_time, 2)
        self.altitude_agl = round(altitude_agl, 2)
        self.smoothed_altitude_agl = round(smoothed_altitude_agl, 2)
        self.pressure = round(pressure, 2)
        self.temperature = round(temperature, 2)
        self.vertical_velocity = round(vertical_velocity, 2)
        self.flight_stage = flight_stage
        self.delta_time = round(delta_time, 2)

    def as_dict(self):
        """Convert the flight data to a dictionary for easy access or serialization."""
        return {
            "flight_time": self.flight_time,
            "altitude_agl": self.altitude_agl,
            "smoothed_altitude_agl": self.smoothed_altitude_agl,
            "pressure": self.pressure,
            "temperature": self.temperature,
            "vertical_velocity": self.vertical_velocity,
            "flight_stage": self.flight_stage,
            "delta_time": self.delta_time,
        }

    def as_json(self):
        """Convert the flight data to a JSON string for easy access or serialization."""
        return {
            "flight_time": self.flight_time,
            "altitude_agl": self.altitude_agl,
            "smoothed_altitude_agl": self.smoothed_altitude_agl,
            "pressure": self.pressure,
            "temperature": self.temperature,
            "vertical_velocity": self.vertical_velocity,
            "flight_stage": self.flight_stage,
            "delta_time": self.delta_time,
        }

    def as_list(self):
        """Convert the flight data to a list for easy access or serialization."""
        return [
            self.flight_time,
            self.altitude_agl,
            self.smoothed_altitude_agl,
            self.pressure,
            self.temperature,
            self.vertical_velocity,
            self.flight_stage,
            self.delta_time,
        ]
