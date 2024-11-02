#coding:utf-8
from PCA9685 import PCA9685
import logging

logger = logging.getLogger('Servo')

class Servo:
    def __init__(self):
        self.angle_min = 18
        self.angle_max = 162
        self.pwm = PCA9685(address=0x40, debug=False)
        self.pwm.setPWMFreq(50)
        logger.info("Servo controller initialized")

    def map(self, value: float, from_low: float, from_high: float, to_low: float, to_high: float) -> float:
        return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low

    def setServoAngle(self, channel: int, angle: float):
        if angle < self.angle_min:
            angle = self.angle_min
        elif angle > self.angle_max:
            angle = self.angle_max
        pulse = self.map(angle, 0, 180, 102, 512)
        self.pwm.setPWM(channel, 0, int(pulse))
        logger.info(f"Set servo channel {channel} to angle {angle} (pulse {int(pulse)})")



if __name__ == '__main__':
    import time
    logging.basicConfig(level=logging.INFO)
    servo = Servo()
    try:
        while True:
            for angle in range(18, 163, 15):
                servo.setServoAngle(0, angle)
                time.sleep(0.5)
            for angle in range(162, 17, -15):
                servo.setServoAngle(0, angle)
                time.sleep(0.5)
    except KeyboardInterrupt:
        logger.info("Program terminated by user")