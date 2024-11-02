from gpiozero import DistanceSensor

trigger_pin = 27
echo_pin    = 22
sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin ,max_distance=3)

class Ultrasonic:
    def get_distance(self):
        distance_cm = sensor.distance * 100
        return int(distance_cm)

# Main program logic follows:
if __name__ == '__main__':
    pass
