import asyncio
import board
import analogio
import digitalio
import os


from flightManager import FlightManager


from neopixel_manager import NeoPixelManager


def is_filesystem_writable():
    try:
        # Try to create and delete a temporary file
        with open("/temp_file.txt", "w") as temp_file:
            temp_file.write("test")
        os.remove("/temp_file.txt")
        return True
    except OSError:
        return False


async def main():
    # Initialize NeoPixelManager
    neopixel_manager = NeoPixelManager()
    is_development = False

    # Check if filesystem is writable
    if is_filesystem_writable():
        is_development = False
        neopixel_manager.pixel.fill((0, 0, 255))  # Blue
        print("Running in production mode.")
    else:
        is_development = True
        neopixel_manager.pixel.fill((128, 0, 255))  # Purple
        print("Running in development mode.")

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

    asyncio.create_task(neopixel_manager.display_battery_level(percentage))

    flight_manager = FlightManager(is_development=is_development)

    # Start the flight manager task
    print("Run flight manager task...")
    flight_manager_task = asyncio.create_task(flight_manager.main())

    await flight_manager_task


if __name__ == "__main__":
    asyncio.run(main())
