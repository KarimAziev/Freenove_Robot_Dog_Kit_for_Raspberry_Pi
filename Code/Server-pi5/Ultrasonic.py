import logging
from gpiozero import DistanceSensor

logger = logging.getLogger(__name__)


class Ultrasonic:
    def __init__(self, trigger_pin=27, echo_pin=22):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.factory = self.get_factory()

        if self.factory:
            self.sensor = DistanceSensor(
                echo=self.echo_pin,
                trigger=self.trigger_pin,
                max_distance=3,
                pin_factory=self.factory,
            )
        else:
            self.sensor = DistanceSensor(
                echo=self.echo_pin,
                max_distance=3,
                trigger=self.trigger_pin,
            )
            logger.warning(
                "No pin factory detected. Falling back to default configuration."
            )

    def get_distance(self):
        distance_cm = self.sensor.distance * 100
        return int(distance_cm)

    @staticmethod
    def get_factory():
        """
        Attempt to return a PiGPIO factory, if not available,
        return RPiGPIOFactory. If none work, return None.
        """
        factory = Ultrasonic.check_pigpiod()
        if not factory:
            factory = Ultrasonic.check_rpigpio()
        return factory

    @staticmethod
    def check_pigpiod():
        """
        Checks if pigpiod is running by attempting to connect with PiGPIOFactory.
        If successful, returns an instance of PiGPIOFactory.
        Otherwise, returns None.
        """
        try:

            from gpiozero.pins.pigpio import PiGPIOFactory

            factory = PiGPIOFactory()
            return factory
        except (OSError, IOError):
            logger.warning("Warning: pigpiod not running.")
            return None

    @staticmethod
    def check_rpigpio():
        """
        Checks if pigpiod is running by attempting to connect with PiGPIOFactory.
        If successful, returns an instance of PiGPIOFactory.
        Otherwise, returns None.
        """
        try:
            from gpiozero.pins.rpigpio import RPiGPIOFactory

            factory = RPiGPIOFactory()
            return factory
        except (OSError, IOError):
            logger.warning("Warning: RPI factory not running.")
            return None


# Main program logic follows:
if __name__ == '__main__':
    pass
