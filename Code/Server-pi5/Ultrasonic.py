import warnings
import logging

logger = logging.getLogger(__name__)


class Ultrasonic:
    def __init__(self, trigger_pin=27, echo_pin=22):

        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        with warnings.catch_warnings():
            warnings.filterwarnings(
                "ignore", category=UserWarning, message=".*pigpiod.*"
            )
            warnings.filterwarnings(
                "ignore", category=UserWarning, message=".*PWMSoftwareFallback.*"
            )
            self.sensor = self.init_sensor()

    def init_sensor(self):
        """Try to initialize the sensor with whatever pin factory is available."""
        try:
            from gpiozero import DistanceSensor

            return DistanceSensor(
                echo=self.echo_pin,
                trigger=self.trigger_pin,
                max_distance=3,  # 3-meter max range
            )
        except Exception as e:
            logger.error(f"Error initializing DistanceSensor: {e}")
            raise

    def get_distance(self):
        """Return the distance in cm."""
        distance_cm = self.sensor.distance * 100
        return int(distance_cm)


if __name__ == '__main__':
    import time

    ultrasonic = Ultrasonic()

    try:
        while True:
            data = ultrasonic.get_distance()
            logger.info("Obstacle distance is " + str(data) + "CM")
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\nEnd of program")
