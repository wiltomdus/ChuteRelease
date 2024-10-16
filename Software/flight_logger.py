import storage
from flight_data import FlightData


class FlightLogger:

    def __init__(self, is_developement):
        self.is_development = is_developement

    def store_flight_data(self, filename, data: FlightData):
        """Save the flight data to a CSV file in onboard flash."""
        if self.is_development:
            print(f"flight data: {data.as_json()}")
        else:
            try:
                with open(filename, "a") as file:
                    if file.tell() == 0:
                        # Write header if the file is empty
                        file.write(
                            "altitude_agl,smoothed_altitude,pressure,temperature,vertical_velocity,vertical_acceleration,flight_stage,delta_time\n"
                        )

                    # Convert data list to a comma-separated string and write to the file
                    file.write(",".join(map(str, data.as_list())) + "\n")
            except OSError as e:
                print(f"Error saving to Flash: {e}")

    def store_data(self, filename, data: str):
        """Save the data to a CSV file in onboard flash."""
        if self.is_development:
            print(data)
        elif self.is_flash_full():
            try:
                with open(filename, "a") as file:
                    if file.tell() == 0:
                        # Write header if the file is empty
                        file.write(
                            "altitude_agl,smoothed_altitude,pressure,temperature,vertical_velocity,vertical_acceleration,flight_stage,delta_time\n"
                        )

                    # Write data to the file
                    file.write(data + "\n")

            except OSError as e:
                print(f"Error saving to Flash: {e}")

    def is_flash_full(self, threshold=1000000):
        """
        Check if the flash memory is almost full.
        :param threshold: The minimum number of bytes that should remain free. Default is 1000000 bytes (1MB).
        :return: True if the flash is full (below threshold), otherwise False.
        """
        if self.is_development:
            return False

        try:
            mount = storage.getmount("/")
            free_space = mount.free_space

            if free_space < threshold:
                print("Warning: Flash memory is almost full.")
                return True
        except AttributeError:
            print("Error: Unable to determine available flash memory.")
        return False
