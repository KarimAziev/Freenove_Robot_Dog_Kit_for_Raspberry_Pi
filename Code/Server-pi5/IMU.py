#coding:utf-8
import time
import math
from Kalman import Kalman_filter
from mpu6050 import mpu6050
from typing import Tuple

class IMU:
    def __init__(self):
        """
        Initializes the IMU class, which handles sensor fusion for orientation based on accelerometer and gyroscope data.
        Sets up the MPU6050 sensor, Kalman filters, and the initial quaternion values representing orientation.
        """
        # PID parameters for sensor fusion
        self.Kp = 100
        self.Ki = 0.002

        # Initial integration time
        self.lastUpdate = time.time()

        # Quaternion components representing the orientation
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        # Integral error terms for PI controller
        self.exInt = 0.0
        self.eyInt = 0.0
        self.ezInt = 0.0

        # Euler angles
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        # Initialize the MPU6050 sensor
        self.sensor = mpu6050(address=0x68)
        self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_2G)
        self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)

        # Kalman filters for accelerometer and gyroscope data
        self.kalman_filter_AX = Kalman_filter(0.001, 0.1)
        self.kalman_filter_AY = Kalman_filter(0.001, 0.1)
        self.kalman_filter_AZ = Kalman_filter(0.001, 0.1)

        self.kalman_filter_GX = Kalman_filter(0.001, 0.1)
        self.kalman_filter_GY = Kalman_filter(0.001, 0.1)
        self.kalman_filter_GZ = Kalman_filter(0.001, 0.1)

        # Calculate bias errors by averaging over 100 samples
        self.Error_value_accel_data, self.Error_value_gyro_data = self.average_filter()

    def average_filter(self):
        """
        Collects and averages 100 samples of accelerometer and gyroscope data.

        :returns: A tuple containing two dictionaries:
                  - accel_data: Dictionary with averaged accelerometer data ('x', 'y', 'z')
                  - gyro_data: Dictionary with averaged gyroscope data ('x', 'y', 'z')
        """
        sum_accel_x = 0.0
        sum_accel_y = 0.0
        sum_accel_z = 0.0

        sum_gyro_x = 0.0
        sum_gyro_y = 0.0
        sum_gyro_z = 0.0

        # Initialize sensor data dictionaries
        accel_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        gyro_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Collect 100 samples to calculate the average for bias correction
        for i in range(100):
            try:
                accel = self.sensor.get_accel_data()
                gyro = self.sensor.get_gyro_data()

                sum_accel_x += accel['x']
                sum_accel_y += accel['y']
                sum_accel_z += accel['z']

                sum_gyro_x += gyro['x']
                sum_gyro_y += gyro['y']
                sum_gyro_z += gyro['z']
            except Exception as e:
                print(f"Sensor read error: {e}")
                continue
            time.sleep(0.005)  # Small delay to avoid overwhelming the sensor

        accel_data['x'] = sum_accel_x / 100.0
        accel_data['y'] = sum_accel_y / 100.0
        accel_data['z'] = sum_accel_z / 100.0 - 9.8  # Assuming sensor is stationary and aligned

        gyro_data['x'] = sum_gyro_x / 100.0
        gyro_data['y'] = sum_gyro_y / 100.0
        gyro_data['z'] = sum_gyro_z / 100.0

        return accel_data, gyro_data

    def imuUpdate(self) -> Tuple[float, float, float]:
        """
        Updates the IMU state by reading sensor data, applying Kalman filtering,
        and performing sensor fusion to compute pitch, roll, and yaw.

        :returns: A tuple containing (pitch, roll, yaw) in degrees.
        """
        now = time.time()
        dt = now - self.lastUpdate
        self.lastUpdate = now
        halfT = dt / 2.0

        try:
            accel_data = self.sensor.get_accel_data()
            gyro_data = self.sensor.get_gyro_data()
        except Exception as e:
            print(f"Sensor read error: {e}")
            return self.pitch, self.roll, self.yaw

        # Apply Kalman filter and subtract bias errors
        ax = self.kalman_filter_AX.kalman(accel_data['x'] - self.Error_value_accel_data['x'])
        ay = self.kalman_filter_AY.kalman(accel_data['y'] - self.Error_value_accel_data['y'])
        az = self.kalman_filter_AZ.kalman(accel_data['z'] - self.Error_value_accel_data['z'])
        gx = self.kalman_filter_GX.kalman(gyro_data['x'] - self.Error_value_gyro_data['x'])
        gy = self.kalman_filter_GY.kalman(gyro_data['y'] - self.Error_value_gyro_data['y'])
        gz = self.kalman_filter_GZ.kalman(gyro_data['z'] - self.Error_value_gyro_data['z'])

        # Normalize accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            norm = 1e-5  # Prevent division by zero
        ax /= norm
        ay /= norm
        az /= norm

        # Estimated direction of gravity and vector perpendicular to magnetic flux
        vx = 2 * (self.q1 * self.q3 - self.q0 * self.q2)
        vy = 2 * (self.q0 * self.q1 + self.q2 * self.q3)
        vz = self.q0 * self.q0 - self.q1 * self.q1 - self.q2 * self.q2 + self.q3 * self.q3

        # Error is the cross product between estimated and measured direction of gravity
        ex = (ay * vz - az * vy)
        ey = (az * vx - ax * vz)
        ez = (ax * vy - ay * vx)

        # Integral feedback
        self.exInt += ex * self.Ki * dt
        self.eyInt += ey * self.Ki * dt
        self.ezInt += ez * self.Ki * dt

        # Adjust gyroscope measurements
        gx += self.Kp * ex + self.exInt
        gy += self.Kp * ey + self.eyInt
        gz += self.Kp * ez + self.ezInt

        # Integrate quaternion rate and normalize
        self.q0 += (-self.q1 * gx - self.q2 * gy - self.q3 * gz) * halfT
        self.q1 += (self.q0 * gx + self.q2 * gz - self.q3 * gy) * halfT
        self.q2 += (self.q0 * gy - self.q1 * gz + self.q3 * gx) * halfT
        self.q3 += (self.q0 * gz + self.q1 * gy - self.q2 * gx) * halfT

        # Normalize quaternion
        norm = math.sqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        if norm == 0:
            norm = 1e-5  # Prevent division by zero
        self.q0 /= norm
        self.q1 /= norm
        self.q2 /= norm
        self.q3 /= norm

        # Compute Euler angles
        self.pitch = math.degrees(math.asin(-2 * self.q1 * self.q3 + 2 * self.q0 * self.q2))
        self.roll = math.degrees(math.atan2(2 * self.q2 * self.q3 + 2 * self.q0 * self.q1,
                                            -2 * self.q1 * self.q1 - 2 * self.q2 * self.q2 + 1))
        self.yaw = math.degrees(math.atan2(2 * (self.q1 * self.q2 + self.q0 * self.q3),
                                           self.q0 * self.q0 + self.q1 * self.q1 - self.q2 * self.q2 - self.q3 * self.q3))

        return self.pitch, self.roll, self.yaw


# Main program logic follows:
if __name__ == '__main__':
    pass
