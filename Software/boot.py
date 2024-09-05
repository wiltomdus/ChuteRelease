"""CircuitPython Essentials Storage logging boot.py file"""
import board
import digitalio
import storage

VBUS_DETECT = digitalio.DigitalInOut(board.VBUS_DETECT)
VBUS_DETECT.direction = digitalio.Direction.INPUT
VBUS_DETECT_value = VBUS_DETECT.value

# If USB is connected, CircuitPython can write to the drive (DEV MODE)
# If USB is not connected, CircuitPython cannot write to the drive (PRODUCTION MODE)
storage.remount("/", VBUS_DETECT_value)
