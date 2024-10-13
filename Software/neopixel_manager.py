import time
import neopixel
import board

# Define the pin where the NeoPixel is connected and the number of pixels (1 in this case)
NEOPIXEL_PIN = board.NEOPIX  # Example pin, adjust to your hardware
NUM_PIXELS = 1

# Define colors for each digit (cycling colors for more digits)
COLORS = [
    (255, 0, 0),  # Red
    (0, 0, 255),  # Blue
    (255, 255, 0),  # Yellow
    (0, 255, 0),  # Green
    (128, 0, 128),  # Purple
    (255, 165, 0),  # Orange
]


class NeoPixelManager:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(NeoPixelManager, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return  # Ensure we only initialize once

        self.pixel = neopixel.NeoPixel(NEOPIXEL_PIN, NUM_PIXELS, brightness=0.05)
        self.pixel.fill((0, 0, 0))  # Turn off initially
        self._initialized = True

    def display_battery_level(self, percentage):
        """
        Display battery level on NeoPixel from green (full) to yellow (half) to red (low).
        :param percentage: Battery percentage (0 to 100)
        """
        if percentage > 50:
            # Interpolate between green and yellow
            ratio = (percentage - 50) / 50
            color = (
                int(0 * (1 - ratio) + 255 * ratio),  # Green to Yellow
                int(255 * (1 - ratio) + 255 * ratio),
                0,
            )
        else:
            # Interpolate between yellow and red
            ratio = percentage / 50
            color = (
                int(255 * (1 - ratio) + 255 * ratio),
                int(255 * (1 - ratio)),
                0,  # Yellow to Red
            )

        self.pixel.fill(color)

    def turn_off(self):
        """Turn off the NeoPixel."""
        self.pixel.fill((0, 0, 0))

    def display_altitude_sequence(self, altitude):
        """
        Dynamically display the altitude by blinking in colors corresponding to digits.
        The number of blinks corresponds to each digit of the altitude, cycling through colors.
        Example: Altitude 1345 -> 1 blink of Red, 3 blinks of Blue, 4 blinks of Purple, 5 blinks of Green.
        """
        # Long fill to indicate the start of the sequence
        self.pixel.fill((255, 255, 255))  # White color for the start of the sequence
        time.sleep(4)  # Keep the LED lit for 2 seconds
        self.pixel.fill((0, 0, 0))  # Turn off before starting the blink sequence
        time.sleep(0.5)  # Short pause before the sequence begins

        digits = [
            int(d) for d in str(altitude)
        ]  # Convert altitude into a list of digits
        num_colors = len(COLORS)

        # Loop through each digit and display corresponding blinks
        for i, digit in enumerate(digits):
            color = COLORS[i % num_colors]  # Cycle through colors for each digit
            for _ in range(digit):
                self.pixel.fill(color)
                time.sleep(0.35)
                self.pixel.fill((0, 0, 0))
                time.sleep(0.35)

            # Short delay between digits
            time.sleep(1)
