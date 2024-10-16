import storage
from flight_data import FlightData


class FlightLogger:

    def __init__(self, is_development, buffer_size=10):
        self.is_development = is_development
        self.filename = self.generate_filename()
        self.file = None
        self.data_buffer = []
        self.buffer_size = buffer_size

    def start_logging(self):
        """Start logging by opening the file."""
        if not self.is_development:
            try:
                self.file = open(self.filename, "a")
                if self.file.tell() == 0:
                    # Write header if the file is empty
                    self.file.write(
                        "flight_time,altitude_agl,smoothed_altitude,pressure,temperature,vertical_velocity,flight_stage,delta_time\n"
                    )
            except OSError as e:
                print(f"Error opening file: {e}")

    def stop_logging(self):
        """Flush and close the file at the end of logging."""
        self.flush_data()
        if self.file:
            self.file.flush()
            self.file.close()

    def get_next_flight_number(self):
        """Retrieve and increment the flight number from a file."""
        try:
            with open("/flight_number.txt", "r") as file:
                flight_number = int(file.read().strip())
        except (OSError, ValueError):
            flight_number = 0

        flight_number += 1

        try:
            with open("/flight_number.txt", "w") as file:
                file.write(str(flight_number))
        except OSError:
            pass

        return flight_number

    def generate_filename(self):
        """Generate a unique filename using the flight number."""
        flight_number = self.get_next_flight_number()
        filename = f"/flight-data-{flight_number}.csv"
        return filename

    def store_flight_data(self, data: FlightData):
        """Buffer data and write to file in batches."""
        if self.is_development:
            print(f"flight data: {data.as_json()}")
        else:
            self.data_buffer.append(data)
            if len(self.data_buffer) >= self.buffer_size:
                self.flush_data()

    def flush_data(self):
        """Write buffered data to the file."""
        if self.data_buffer:
            try:
                if not self.file:
                    self.start_logging()

                for data in self.data_buffer:
                    self.file.write(",".join(map(str, data.as_list())) + "\n")
                self.file.flush()  # Ensure data is written to flash
                self.data_buffer = []  # Clear the buffer after writing
            except OSError as e:
                print(f"Error saving to Flash: {e}")

    def is_flash_full(self, threshold=1000000):
        """Check if the flash memory is almost full."""
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
