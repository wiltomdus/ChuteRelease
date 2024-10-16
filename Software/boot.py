"""CircuitPython Essentials Storage logging boot.py file"""

import board
import digitalio
import storage

VBUS_DETECT = digitalio.DigitalInOut(board.VBUS_DETECT)
VBUS_DETECT.direction = digitalio.Direction.INPUT
VBUS_DETECT_value = VBUS_DETECT.value

# If USB is connected VBUS_DETECT_value is True, therefore the flash is readonly, CircuitPython cannot write to the drive but the your PC can (DEV MODE)
# If USB is not connected VBUS_DETECT_value is False, therefore the flash is not readonly, CircuitPython can write to the drive (PRODUCTION MODE)
storage.remount("/", readonly=VBUS_DETECT_value)
