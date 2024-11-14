from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
import logging

logger = logging.getLogger(__name__)


def check_pigpiod():
    """
    Checks if pigpiod is running by attempting to connect with PiGPIOFactory.
    If successful, returns an instance of PiGPIOFactory.
    Otherwise, returns None.
    """
    try:

        factory = PiGPIOFactory()
        return factory
    except (OSError, IOError):
        logger.warning(
            "Warning: pigpiod not running. Using default GPIOZero configuration."
        )
        return None


factory = check_pigpiod()

trigger_pin = 27
echo_pin = 22
sensor = (
    DistanceSensor(
        echo=echo_pin, trigger=trigger_pin, max_distance=3, pin_factory=factory
    )
    if factory is not None
    else DistanceSensor(echo=echo_pin, trigger=trigger_pin)
)


class Ultrasonic:
    def get_distance(self):
        distance_cm = sensor.distance * 100
        return int(distance_cm)


# Main program logic follows:
if __name__ == '__main__':
    pass
