"""Barometer module"""

import time
import digitalio
import board
import busio
from adafruit_dps310.advanced import DPS310_Advanced as DPS310
from adafruit_dps310.advanced import Rate as DPS310_RATE
from adafruit_dps310.advanced import Mode as DPS310_MODE
from adafruit_dps310.advanced import SampleCount as DPS310_SAMPLE_COUNT


class Barometer(object):
    """Altimeter API using the DPS310 sensor"""

    def __init__(self):
        self.i2c = None
        self._sensor = None
        self.init_barometer()

    def init_barometer(self):
        """Initialize settings for the DPS310 sensor"""
        print("Initializing barometer...")

        # Set up the SDO pin
        sdo = digitalio.DigitalInOut(board.SDO)
        sdo.direction = digitalio.Direction.OUTPUT
        sdo.value = True

        # Initialize the I2C bus if not already initialized
        i2c = busio.I2C(board.SCL, board.SDA)

        # Scan for I2C devices
        print("Scanning for I2C devices...")
        while not i2c.try_lock():
            pass
        try:
            devices = i2c.scan()
            if not devices:
                raise RuntimeError("No I2C devices found.")
            print(
                f"Found I2C devices at addresses: {[hex(device) for device in devices]}"
            )
            device = devices[0]
            print(f"Selected I2C device at address: {hex(device)}")
        finally:
            i2c.unlock()

        time.sleep(0.5)
        # Set the sensor to high precision mode
        self._sensor = DPS310(i2c, address=devices[0])
        self._sensor.initialize()
        self._sensor.pressure_oversample_count = DPS310_SAMPLE_COUNT.COUNT_4
        self._sensor.pressure_rate = DPS310_RATE.RATE_128_HZ
        self._sensor.temperature_oversample_count = DPS310_SAMPLE_COUNT.COUNT_4
        self._sensor.temperature_rate = DPS310_RATE.RATE_128_HZ
        self._sensor.mode = DPS310_MODE.CONT_PRESTEMP
        self._sensor.sea_level_pressure = (
            1013.25  #! Set local sea level pressure for more precision
        )
        self._sensor.wait_temperature_ready()
        self._sensor.wait_pressure_ready()
        print("Barometer initialization finished!")

    def get_sensor_data(self) -> tuple[float, float, float]:
        """Get current sensor data from barometer"""
        return (self._sensor.pressure, self._sensor.altitude, self._sensor.temperature)
