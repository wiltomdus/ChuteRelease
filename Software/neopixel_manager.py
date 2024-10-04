import time
import neopixel
import board

# Define the pin where the NeoPixel is connected and the number of pixels (1 in this case)
NEOPIXEL_PIN = board.GP28  # Example pin, adjust to your hardware
NUM_PIXELS = 1

# Define colors for each digit (cycling colors for more digits)
COLORS = [
    (255, 0, 0),  # Red
    (0, 0, 255),  # Blue
    (128, 0, 128),  # Purple
    (0, 255, 0),  # Green
    (255, 255, 0),  # Yellow
]


class NeoPixelManager:
    def __init__(self):
        self.pixel = neopixel.NeoPixel(NEOPIXEL_PIN, NUM_PIXELS, brightness=0.5)
        self.pixel.fill((0, 0, 0))  # Turn off initially

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
        digits = [
            int(d) for d in str(altitude)
        ]  # Convert altitude into a list of digits
        num_colors = len(COLORS)

        # Loop through each digit and display corresponding blinks
        for i, digit in enumerate(digits):
            color = COLORS[i % num_colors]  # Cycle through colors for each digit
            for _ in range(digit):
                self.pixel.fill(color)
                time.sleep(0.5)
                self.pixel.fill((0, 0, 0))
                time.sleep(0.5)

        # Short delay between cycles
        time.sleep(2)
