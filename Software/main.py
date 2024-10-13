import asyncio
import digitalio
import board
import analogio

from flightManager import FlightManager
import board
import digitalio

from neopixel_manager import NeoPixelManager


def get_next_flight_number():
    """Retrieve and increment the flight number from a file."""
    try:
        with open("/flight_number.txt", "r") as file:
            flight_number = int(file.read().strip())
    except (OSError, ValueError):
        flight_number = 0  # Start with 0 if the file doesn't exist or is corrupted

    flight_number += 1  # Increment the flight number

    # Write the new flight number back to the file
    try:
        with open("/flight_number.txt", "w") as file:
            file.write(str(flight_number))
    except OSError:
        pass

    return flight_number


def generate_filename():
    """Generate a unique filename using a flight number."""
    flight_number = get_next_flight_number()
    filename = f"/flight-data-{flight_number}.csv"
    return filename


async def main():
    is_development = False

    flight_data_filename = generate_filename()

    # Check if filesystem is writable
    try:

        with open(flight_data_filename, "a") as file:
            pass
    except OSError:  # Typically when the filesystem isn't writable...
        is_development = True
    finally:
        if is_development:
            print("Running in dev mode")
        else:
            print("Running in prod mode")

    flight_manager = FlightManager(
        flight_data_filename=flight_data_filename, is_development=is_development
    )

    # Start the flight manager task
    print("Run flight manager task...")
    flight_manager_task = asyncio.create_task(flight_manager.main())

    await flight_manager_task


if __name__ == "__main__":

    BAT_SENSE = analogio.AnalogIn(board.VBAT_SENSE)
    BAT_SENSE_value = BAT_SENSE.value

    conversion_factor = 3 * 3.3 / 65535
    full_battery = 4.2  # reference voltages for a full/empty battery, in volts
    empty_battery = 2.8  # the values could vary by battery size/manufacturer so you might need to adjust them

    # convert the raw ADC read into a voltage, and then a percentage
    voltage = BAT_SENSE_value * conversion_factor
    percentage = 100 * ((voltage - empty_battery) / (full_battery - empty_battery))
    if percentage > 100:
        percentage = 100

    # Initialize NeoPixelManager
    neopixel_manager = NeoPixelManager()

    # Display the battery percentage using NeoPixelManager
    neopixel_manager.display_battery_level(percentage)

    print(f"Battery voltage: {voltage:0.2f}V ({percentage:0.1f}%)")

    asyncio.run(main())
