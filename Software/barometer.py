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
        """Initialize settings for the DPS310 sensor with additional debugging"""
        print("Initializing barometer...")

        i2c = busio.I2C(board.SCL, board.SDA)
        time.sleep(0.1)  # Short delay after initializing I2C

        while not i2c.try_lock():
            pass

        try:
            devices = i2c.scan()
            print(
                f"Found I2C devices at addresses: {[hex(device) for device in devices]}"
            )
            if not devices:
                raise RuntimeError("No I2C devices found.")
            device = devices[0]
            print(f"Using device: {hex(device)}")
            time.sleep(1)

        finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
            i2c.unlock()

        try:
            self._sensor = DPS310(i2c, address=device)
            time.sleep(0.5)  # Short delay before initializing the sensor
            print("Basic sensor initialization successful.")
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
            time.sleep(1)
            print("Sensor initialized successfully.")
        except OSError as e:
            print(f"Sensor initialization failed: {e}")
            raise RuntimeError("Failed to initialize sensor.")
        finally:
            i2c.unlock()

    def get_sensor_data(self) -> tuple[float, float, float]:
        """Get current sensor data from barometer"""
        return (self._sensor.pressure, self._sensor.altitude, self._sensor.temperature)
