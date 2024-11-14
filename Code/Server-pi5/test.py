import time
from Servo import Servo
from Ultrasonic import Ultrasonic
from Buzzer import BuzzerController
from ADS7830 import ADS7830

ultrasonic = Ultrasonic()
servo = Servo()
adc = ADS7830()
buzzer = BuzzerController()


def test_Ultrasonic(count=10):
    try:
        for _ in range(count):
            data = ultrasonic.get_distance()
            print("Obstacle distance is " + str(data) + "CM")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nEnd of program")


def test_Servo():
    try:
        for i in range(90):
            servo.setServoAngle(4, 90 - i)
            servo.setServoAngle(7, 90 - i)
            servo.setServoAngle(8, 90 + i)
            servo.setServoAngle(11, 90 + i)
            time.sleep(0.01)
        for i in range(90):
            servo.setServoAngle(2, 90 - i)
            servo.setServoAngle(5, 90 - i)
            servo.setServoAngle(10, 90 + i)
            servo.setServoAngle(13, 90 + i)
            time.sleep(0.01)
        for i in range(60):
            servo.setServoAngle(3, 90 - i)
            servo.setServoAngle(6, 90 - i)
            servo.setServoAngle(9, 90 + i)
            servo.setServoAngle(12, 90 + i)
            time.sleep(0.01)
        print("\nEnd of program")
    except KeyboardInterrupt:
        print("\nEnd of program")


def test_Adc():
    try:
        for _ in range(10):
            Power = adc.read_adc(0) / 255.0 * 5.0 * 2
            print("The battery voltage is " + str(Power) + "V")
            time.sleep(1)
            print('\n')
    except KeyboardInterrupt:
        print("\nEnd of program")


def test_Buzzer():
    try:
        buzzer.run('1')
        time.sleep(1)
        print("1S")
        time.sleep(1)
        print("2S")
        time.sleep(1)
        print("3S")
        buzzer.run('0')
        print("\nEnd of program")
    except KeyboardInterrupt:
        buzzer.run('0')
        print("\nEnd of program")


if __name__ == '__main__':
    print('Program is starting ... ')
    import sys

    if len(sys.argv) < 2:
        print("Testing servo:")
        test_Servo()
        print("Testing ADC:")
        test_Adc()
        print("Testing Ultrasonic:")
        test_Ultrasonic()
        print("Testing Buzzer:")
        test_Buzzer()
    elif sys.argv[1] == 'Ultrasonic':
        test_Ultrasonic()
    elif sys.argv[1] == 'Servo':
        test_Servo()
    elif sys.argv[1] == 'ADC':
        test_Adc()
    elif sys.argv[1] == 'Buzzer':
        test_Buzzer()
