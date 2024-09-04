"""Main system for Sora"""

import asyncio
import digitalio
import board
import os

from flightManager import FlightManager


async def main():
    is_development = False

    # Check if filesystem is writable
    try:
        with open("/data/flight-data.csv", "a") as file:
            pass
    except OSError:  # Typically when the filesystem isn't writeable...
        is_development = True
    finally:
        if is_development:
            print("Running in dev mode")
            led = digitalio.DigitalInOut(board.GP25)
            led.direction = digitalio.Direction.OUTPUT
            led.value = True
        else:
            print("Running in prod mode")

    flight_manager = FlightManager()

    # Start the flight manager task
    
    print("Run flight manager task...")
    flight_manager_task = asyncio.create_task(flight_manager.fly(is_development))
    asyncio.gather(flight_manager_task)

    # Logging the max altitude
    max_altitude = flight_manager.get_max_values()
    print(f"Max altitude : {max_altitude:.4f}m")
    print("End of flight")


if __name__ == "__main__":
    print(f'version {os.uname()}')
    asyncio.run(main())
