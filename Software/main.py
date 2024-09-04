import asyncio
import digitalio
import board

from flightManager import FlightManager

async def blink_led(led, delay):
    """Blink the LED at a specified delay."""
    while True:
        led.value = not led.value
        await asyncio.sleep(delay)

async def main():
    is_development = False
    
    
    led = digitalio.DigitalInOut(board.GP25)
    led.direction = digitalio.Direction.OUTPUT

    # Check if filesystem is writable
    try:
        with open("/data/flight-data.csv", "a") as file:
            pass
    except OSError:  # Typically when the filesystem isn't writeable...
        is_development = True
    finally:
        if is_development:
            print("Running in dev mode")
            led.value = True
            
        else:
            print("Running in prod mode")

    flight_manager = FlightManager()

    # Start the flight manager task
    print("Run flight manager task...")
    flight_manager_task = asyncio.create_task(flight_manager.fly(is_development))
    
    # Start the LED blinking task
    if not is_development:
        led_blink_task = asyncio.create_task(blink_led(led, 0.5))  # Blink every 0.5 seconds

    await flight_manager_task
    
    print("End of flight")


if __name__ == "__main__":
    asyncio.run(main())
