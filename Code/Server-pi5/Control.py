import logging
import time
import math
import threading
from IMU import IMU
from PID import Incremental_PID
import numpy as np
from Servo import Servo
from Command import COMMAND as cmd
from typing import Union, List


class Control:
    def __init__(self):
        """
        Initializes the Control class, setting up logging and all necessary components such as IMU, Servo, and PID.
        Also initiates the necessary variables for controlling servo angles, points, calibration data, safety locks, and flags.
        """
        logging.basicConfig(
            filename='control.log',
            level=logging.DEBUG,
            format='%(asctime)s - %(levelname)s - %(message)s',
        )

        logging.info('Initializing Control Class')
        self.imu = IMU()
        self.servo = Servo()
        self.pid = Incremental_PID(0.5, 0.0, 0.0025)
        self.speed = 8
        self.height = 99
        self.timeout = 0
        self.move_flag = 0
        self.move_count = 0
        self.move_timeout = time.time()
        self.order = ['', '', '', '', '']
        self.point: List[List[Union[int, float]]] = [
            [0, 99, 10],
            [0, 99, 10],
            [0, 99, -10],
            [0, 99, -10],
        ]
        self.calibration_point = self.read_from_txt('point')
        self.angle = [[90, 0, 0], [90, 0, 0], [90, 0, 0], [90, 0, 0]]
        self.calibration_angle = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.relax_flag = True
        self.balance_flag = False
        self.attitude_flag = False
        self.stop_condition_thread = False
        self.lock = threading.Lock()
        self.calibration()
        self.relax(True)
        self.Thread_condition = threading.Thread(target=self.condition)
        self.Thread_condition.daemon = True
        self.Thread_condition.start()

    def read_from_txt(self, filename):
        """
        Reads data from a txt file and returns it as a 2D list of integers.

        Args:
            filename (str): The name of the file (without extension).

        Returns:
            List[List[int]]: The data read from the file as a 2D list of integers.
        """
        logging.info(f'Reading from {filename}.txt')
        try:
            with open(filename + ".txt", "r") as file:
                list_row = file.readlines()
            list_source = []
            for row in list_row:
                column_list = row.strip().split("\t")
                list_source.append([int(i) for i in column_list])
            logging.info(f'Successfully read data from {filename}.txt')
            return list_source
        except Exception as e:
            logging.error(f'Error reading from {filename}.txt: {e}', exc_info=True)
            return [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]

    def save_to_txt(self, list, filename):
        """
        Saves a list of data into a txt file.

        Args:
            list (List[List[Union[int, float]]]): The data to save.
            filename (str): The name of the file (without extension).
        """
        logging.info(f'Saving to {filename}.txt')
        try:
            with open(filename + '.txt', 'w') as file:
                for items in list:
                    file.write("\t".join(map(str, items)) + '\n')
            logging.info(f'Successfully saved data to {filename}.txt')
        except Exception as e:
            logging.error(f'Error saving to {filename}.txt: {e}', exc_info=True)

    def coordinate_to_angle(self, x, y, z, l1=23, l2=55, l3=55):
        """
        Converts XYZ coordinates into servo angles.

        Args:
            x, y, z (float): Coordinates in the 3D plane.
            l1, l2, l3 (int): The lengths of the three segments.

        Returns:
            tuple[int, int, int]: Servo angles for respective coordinates.
        """
        try:
            a = math.pi / 2 - math.atan2(z, y)
            l23 = math.sqrt(
                (z - l1 * math.cos(a)) ** 2 + (y - l1 * math.sin(a)) ** 2 + x**2
            )
            w = x / l23
            v = (l2**2 + l23**2 - l3**2) / (2 * l2 * l23)
            b = math.asin(round(w, 2)) - math.acos(round(v, 2))
            c = math.pi - math.acos(round((l2**2 + l3**2 - l23**2) / (2 * l3 * l2), 2))
            logging.debug(
                f'Coordinate to Angle calculation: [{round(math.degrees(a))}, {round(math.degrees(b))}, {round(math.degrees(c))}]'
            )
            return (
                round(math.degrees(a)),
                round(math.degrees(b)),
                round(math.degrees(c)),
            )
        except Exception as e:
            logging.error(f'Error in coordinate_to_angle: {e}', exc_info=True)
            return 0, 0, 0

    def angle_to_coordinate(
        self, a, b, c, l1=23, l2=55, l3=55
    ) -> tuple[float, float, float]:
        """
        Converts servo angles back into XYZ coordinates.

        Args:
            a, b, c (int): Servo angles.
            l1, l2, l3 (int): The lengths of the three segments.

        Returns:
            tuple[float, float, float]: XYZ coordinates.
        """
        a = math.radians(a)
        b = math.radians(b)
        c = math.radians(c)
        x = l3 * math.sin(b + c) + l2 * math.sin(b)
        y = (
            l3 * math.sin(a) * math.cos(b + c)
            + l2 * math.sin(a) * math.cos(b)
            + l1 * math.sin(a)
        )
        z = (
            l3 * math.cos(a) * math.cos(b + c)
            + l2 * math.cos(a) * math.cos(b)
            + l1 * math.cos(a)
        )
        return x, y, z

    def calibration(self):
        """
        Calibrates the angles based on points for all four legs.
        """
        for i in range(4):
            (
                self.calibration_angle[i][0],
                self.calibration_angle[i][1],
                self.calibration_angle[i][2],
            ) = self.coordinate_to_angle(
                self.calibration_point[i][0],
                self.calibration_point[i][1],
                self.calibration_point[i][2],
            )
        for i in range(4):
            self.angle[i][0], self.angle[i][1], self.angle[i][2] = (
                self.coordinate_to_angle(
                    self.point[i][0], self.point[i][1], self.point[i][2]
                )
            )
        for i in range(4):
            self.calibration_angle[i][0] -= self.angle[i][0]
            self.calibration_angle[i][1] -= self.angle[i][1]
            self.calibration_angle[i][2] -= self.angle[i][2]

    def run(self):
        logging.info('Running Control Logic...')
        if self.check_point():
            try:
                for i in range(4):
                    self.angle[i][0], self.angle[i][1], self.angle[i][2] = (
                        self.coordinate_to_angle(
                            self.point[i][0], self.point[i][1], self.point[i][2]
                        )
                    )
                for i in range(2):
                    self.angle[i][0] = self.restriction(
                        self.angle[i][0] + self.calibration_angle[i][0], 0, 180
                    )
                    self.angle[i][1] = self.restriction(
                        90 - (self.angle[i][1] + self.calibration_angle[i][1]), 0, 180
                    )
                    self.angle[i][2] = self.restriction(
                        self.angle[i][2] + self.calibration_angle[i][2], 0, 180
                    )
                    self.angle[i + 2][0] = self.restriction(
                        self.angle[i + 2][0] + self.calibration_angle[i + 2][0], 0, 180
                    )
                    self.angle[i + 2][1] = self.restriction(
                        90 + self.angle[i + 2][1] + self.calibration_angle[i + 2][1],
                        0,
                        180,
                    )
                    self.angle[i + 2][2] = self.restriction(
                        180 - (self.angle[i + 2][2] + self.calibration_angle[i + 2][2]),
                        0,
                        180,
                    )
                for i in range(2):
                    self.servo.setServoAngle(4 + i * 3, self.angle[i][0])
                    self.servo.setServoAngle(3 + i * 3, self.angle[i][1])
                    self.servo.setServoAngle(2 + i * 3, self.angle[i][2])
                    self.servo.setServoAngle(8 + i * 3, self.angle[i + 2][0])
                    self.servo.setServoAngle(9 + i * 3, self.angle[i + 2][1])
                    self.servo.setServoAngle(10 + i * 3, self.angle[i + 2][2])
                logging.info('Servo angles set successfully')
            except Exception as e:
                logging.error(f'Error running servo control logic: {e}', exc_info=True)
        else:
            logging.warning("This coordinate point is out of the active range")

    def check_point(self):
        flag = True
        leg_length: List[Union[int, float]] = [0, 0, 0, 0]
        for i in range(4):
            leg_length[i] = math.sqrt(
                self.point[i][0] ** 2 + self.point[i][1] ** 2 + self.point[i][2] ** 2
            )
            if leg_length[i] > 130 or leg_length[i] < 25:
                flag = False
        logging.info(f'Leg lengths: {leg_length}, flag {flag}')
        return flag

    def condition(self):
        logging.info('Starting condition thread...')
        while not self.stop_condition_thread:
            try:
                current_time = time.time()
                if (
                    current_time - self.move_timeout > 60
                    and self.move_timeout != 0
                    and self.relax_flag
                ):
                    with self.lock:
                        self.move_count = 0
                    logging.info('Stopping due to inactivity')
                    self.move_timeout = current_time

                if self.move_count < 180:
                    if (
                        (current_time - self.timeout) > 10
                        and self.timeout != 0
                        and not self.relax_flag
                        and self.order[0] == ''
                    ):
                        self.timeout = current_time
                        self.relax_flag = True
                        self.relax(True)
                        self.order = ['', '', '', '', '']
                    if (
                        self.relax_flag
                        and self.order[0] != ''
                        and self.order[0] != cmd.CMD_RELAX
                    ):
                        self.relax(False)
                        self.relax_flag = False
                    if (
                        self.attitude_flag
                        and self.order[0] != cmd.CMD_ATTITUDE
                        and self.order[0] != ''
                    ):
                        self.stop()
                        self.attitude_flag = False
                    if not self.relax_flag:
                        with self.lock:
                            self.move_count += current_time - self.move_timeout
                            self.move_timeout = current_time
                    if self.order[0] == cmd.CMD_MOVE_STOP:
                        self.order = ['', '', '', '', '']
                        self.stop()
                    elif self.order[0] == cmd.CMD_MOVE_FORWARD:
                        self.speed = int(self.order[1])
                        self.forward()
                    elif self.order[0] == cmd.CMD_MOVE_BACKWARD:
                        self.speed = int(self.order[1])
                        self.backward()
                    elif self.order[0] == cmd.CMD_MOVE_LEFT:
                        self.speed = int(self.order[1])
                        self.step_left()
                    elif self.order[0] == cmd.CMD_MOVE_RIGHT:
                        self.speed = int(self.order[1])
                        self.step_right()
                    elif self.order[0] == cmd.CMD_TURN_LEFT:
                        self.speed = int(self.order[1])
                        self.turn_left()
                    elif self.order[0] == cmd.CMD_TURN_RIGHT:
                        self.speed = int(self.order[1])
                        self.turn_right()
                    elif self.order[0] == cmd.CMD_RELAX:
                        if self.relax_flag:
                            self.relax_flag = False
                            self.relax(False)
                        else:
                            self.relax_flag = True
                            self.relax(True)
                        self.order = ['', '', '', '', '']
                    elif self.order[0] == cmd.CMD_HEIGHT:
                        self.up_and_down(int(self.order[1]))
                        self.order = ['', '', '', '', '']
                    elif self.order[0] == cmd.CMD_HORIZON:
                        self.before_and_after(int(self.order[1]))
                        self.order = ['', '', '', '', '']
                    elif self.order[0] == cmd.CMD_ATTITUDE:
                        self.attitude_flag = True
                        self.attitude(self.order[1], self.order[2], self.order[3])
                    elif self.order[0] == cmd.CMD_CALIBRATION:
                        with self.lock:
                            self.move_count = 0
                        if self.order[1] == "one":
                            self.calibration_point[0][0] = int(self.order[2])
                            self.calibration_point[0][1] = int(self.order[3])
                            self.calibration_point[0][2] = int(self.order[4])
                            self.calibration()
                            self.run()
                        elif self.order[1] == "two":
                            self.calibration_point[1][0] = int(self.order[2])
                            self.calibration_point[1][1] = int(self.order[3])
                            self.calibration_point[1][2] = int(self.order[4])
                            self.calibration()
                            self.run()
                        elif self.order[1] == "three":
                            self.calibration_point[2][0] = int(self.order[2])
                            self.calibration_point[2][1] = int(self.order[3])
                            self.calibration_point[2][2] = int(self.order[4])
                            self.calibration()
                            self.run()
                        elif self.order[1] == "four":
                            self.calibration_point[3][0] = int(self.order[2])
                            self.calibration_point[3][1] = int(self.order[3])
                            self.calibration_point[3][2] = int(self.order[4])
                            self.calibration()
                            self.run()
                        elif self.order[1] == "save":
                            self.save_to_txt(self.calibration_point, 'point')
                            self.stop()
                    elif self.order[0] == cmd.CMD_BALANCE and self.order[1] == '1':
                        self.stop_condition_thread = True
                        Thread_IMU = threading.Thread(target=self.IMU6050)
                        Thread_IMU.daemon = True
                        Thread_IMU.start()
                        break
                elif self.move_count > 180:
                    self.relax_flag = True
                    self.relax(True)
                    if self.move_flag != 1:
                        self.move_flag = 1
                    if self.move_count > 240:
                        with self.lock:
                            self.move_count = 0
                            self.move_flag = 0
                    self.order = ['', '', '', '', '']
            except Exception as e:
                logging.error(f"Error in condition thread: {e}", exc_info=True)
                self.stop_condition_thread = True
                return

    def restriction(self, var, v_min, v_max):
        return max(v_min, min(var, v_max))

    def map(self, value, fromLow, fromHigh, toLow, toHigh):
        return (toHigh - toLow) * (value - fromLow) / (fromHigh - fromLow) + toLow

    def change_coordinates(
        self,
        move_order,
        X1: Union[int, float] = 0,
        Y1: Union[int, float] = 96,
        Z1: Union[int, float] = 0,
        X2: Union[int, float] = 0,
        Y2: Union[int, float] = 96,
        Z2: Union[int, float] = 0,
        pos=np.mat(np.zeros((3, 4))),
    ):
        if move_order == 'turn_left':
            for i in range(2):
                self.point[2 * i][0] = ((-1) ** (1 + i)) * X1 + 10
                self.point[2 * i][1] = Y1
                self.point[2 * i][2] = ((-1) ** i) * Z1 + ((-1) ** i) * 10
                self.point[1 + 2 * i][0] = ((-1) ** (1 + i)) * X2 + 10
                self.point[1 + 2 * i][1] = Y2
                self.point[1 + 2 * i][2] = ((-1) ** (1 + i)) * Z2 + ((-1) ** i) * 10
        elif move_order == 'turn_right':
            for i in range(2):
                self.point[2 * i][0] = ((-1) ** i) * X1 + 10
                self.point[2 * i][1] = Y1
                self.point[2 * i][2] = ((-1) ** (1 + i)) * Z1 + ((-1) ** i) * 10
                self.point[1 + 2 * i][0] = ((-1) ** i) * X2 + 10
                self.point[1 + 2 * i][1] = Y2
                self.point[1 + 2 * i][2] = ((-1) ** i) * Z2 + ((-1) ** i) * 10
        elif move_order in ('height', 'horizon'):
            for i in range(2):
                self.point[3 * i][0] = X1 + 10
                self.point[3 * i][1] = Y1
                self.point[1 + i][0] = X2 + 10
                self.point[1 + i][1] = Y2
        elif move_order == 'Attitude Angle':
            for i in range(2):
                self.point[3 - i][0] = pos[0, 1 + 2 * i] + 10
                self.point[3 - i][1] = pos[2, 1 + 2 * i]
                self.point[3 - i][2] = pos[1, 1 + 2 * i]
                self.point[i][0] = pos[0, 2 * i] + 10
                self.point[i][1] = pos[2, 2 * i]
                self.point[i][2] = pos[1, 2 * i]
        else:  # 'backward', 'forward', 'step_right', 'step_left'
            for i in range(2):
                self.point[2 * i][0] = X1 + 10
                self.point[2 * i][1] = Y1
                self.point[2 * i + 1][0] = X2 + 10
                self.point[2 * i + 1][1] = Y2
                self.point[2 * i][2] = Z1 + ((-1) ** i) * 10
                self.point[2 * i + 1][2] = Z2 + ((-1) ** i) * 10
        self.run()

    def backward(self):
        logging.info('Executing backward movement logic...')
        for i in range(450, 89, -self.speed):
            X1 = 12 * math.cos(math.radians(i))
            Y1 = 6 * math.sin(math.radians(i)) + self.height
            X2 = 12 * math.cos(math.radians(i + 180))
            Y2 = 6 * math.sin(math.radians(i + 180)) + self.height
            Y1 = min(Y1, self.height)
            Y2 = min(Y2, self.height)
            self.change_coordinates('backward', X1, Y1, 0, X2, Y2, 0)
            time.sleep(0.01)

    def forward(self):
        logging.info('Executing forward movement logic...')
        for i in range(90, 451, self.speed):
            X1 = 12 * math.cos(math.radians(i))
            Y1 = 6 * math.sin(math.radians(i)) + self.height
            X2 = 12 * math.cos(math.radians(i + 180))
            Y2 = 6 * math.sin(math.radians(i + 180)) + self.height
            Y1 = min(Y1, self.height)
            Y2 = min(Y2, self.height)
            self.change_coordinates('forward', X1, Y1, 0, X2, Y2, 0)
            time.sleep(0.01)

    def turn_left(self):
        logging.info('Executing left turn movement logic...')
        for i in range(0, 361, self.speed):
            X1 = 3 * math.cos(math.radians(i))
            Y1 = 8 * math.sin(math.radians(i)) + self.height
            X2 = 3 * math.cos(math.radians(i + 180))
            Y2 = 8 * math.sin(math.radians(i + 180)) + self.height
            Y1 = min(Y1, self.height)
            Y2 = min(Y2, self.height)
            Z1 = X1
            Z2 = X2
            self.change_coordinates('turn_left', X1, Y1, Z1, X2, Y2, Z2)
            time.sleep(0.01)

    def turn_right(self):
        logging.info('Executing right turn movement logic...')
        for i in range(0, 361, self.speed):
            X1 = 3 * math.cos(math.radians(i))
            Y1 = 8 * math.sin(math.radians(i)) + self.height
            X2 = 3 * math.cos(math.radians(i + 180))
            Y2 = 8 * math.sin(math.radians(i + 180)) + self.height
            Y1 = min(Y1, self.height)
            Y2 = min(Y2, self.height)
            Z1 = X1
            Z2 = X2
            self.change_coordinates('turn_right', X1, Y1, Z1, X2, Y2, Z2)
            time.sleep(0.01)

    def stop(self):
        logging.info('Stopping servo movements')
        p = [
            [10, self.height, 10],
            [10, self.height, 10],
            [10, self.height, -10],
            [10, self.height, -10],
        ]
        for i in range(4):
            p[i][0] = (p[i][0] - self.point[i][0]) / 50
            p[i][1] = (p[i][1] - self.point[i][1]) / 50
            p[i][2] = (p[i][2] - self.point[i][2]) / 50
        for _ in range(50):
            for i in range(4):
                self.point[i][0] += p[i][0]
                self.point[i][1] += p[i][1]
                self.point[i][2] += p[i][2]
            self.run()

    def step_left(self):
        logging.info('Executing left step movement logic...')
        for i in range(90, 451, self.speed):
            Z1 = 10 * math.cos(math.radians(i))
            Y1 = 5 * math.sin(math.radians(i)) + self.height
            Z2 = 10 * math.cos(math.radians(i + 180))
            Y2 = 5 * math.sin(math.radians(i + 180)) + self.height
            Y1 = min(Y1, self.height)
            Y2 = min(Y2, self.height)
            self.change_coordinates('step_left', 0, Y1, Z1, 0, Y2, Z2)
            time.sleep(0.01)

    def step_right(self):
        logging.info('Executing right step movement logic...')
        for i in range(450, 89, -self.speed):
            Z1 = 10 * math.cos(math.radians(i))
            Y1 = 5 * math.sin(math.radians(i)) + self.height
            Z2 = 10 * math.cos(math.radians(i + 180))
            Y2 = 5 * math.sin(math.radians(i + 180)) + self.height
            Y1 = min(Y1, self.height)
            Y2 = min(Y2, self.height)
            self.change_coordinates('step_right', 0, Y1, Z1, 0, Y2, Z2)
            time.sleep(0.01)

    def relax(self, flag=False):
        logging.info('Entering relax mode')
        if flag:
            p: List[List[Union[int, float]]] = [[55, 78, 0] for _ in range(4)]
            for i in range(4):
                p[i][0] = (self.point[i][0] - p[i][0]) / 50
                p[i][1] = (self.point[i][1] - p[i][1]) / 50
                p[i][2] = (self.point[i][2] - p[i][2]) / 50
            for _ in range(50):
                for i in range(4):
                    self.point[i][0] -= p[i][0]
                    self.point[i][1] -= p[i][1]
                    self.point[i][2] -= p[i][2]
                self.run()
            with self.lock:
                self.move_count += time.time() - self.move_timeout
                self.move_timeout = time.time()
        else:
            self.stop()
            self.move_timeout = time.time()

    def up_and_down(self, var):
        self.height = var + 99
        logging.info('Executing height adjustment logic...')
        self.change_coordinates('height', 0, self.height, 0, 0, self.height, 0)

    def before_and_after(self, var):
        logging.info('Executing horizontal movement logic...')
        self.change_coordinates('horizon', var, self.height, 0, var, self.height, 0)

    def attitude(self, r, p, y):
        r = self.map(int(r), -20, 20, -10, 10)
        p = self.map(int(p), -20, 20, -10, 10)
        y = self.map(int(y), -20, 20, -10, 10)
        pos = self.posture_balance(r, p, y, 0)
        self.change_coordinates('Attitude Angle', pos=pos)

    def IMU6050(self):
        logging.info('Starting IMU balancing logic')
        self.balance_flag = True
        self.order = ['', '', '', '', '']
        pos = self.posture_balance(0, 0, 0)
        self.change_coordinates('Attitude Angle', pos=pos)
        time.sleep(2)
        self.imu.Error_value_accel_data, self.imu.Error_value_gyro_data = (
            self.imu.average_filter()
        )

        time.sleep(1)

        while not self.stop_condition_thread:
            with self.lock:
                self.move_count += time.time() - self.move_timeout
                self.move_timeout = time.time()
            r, p, y = self.imu.imu_update()
            logging.info(f'Starting IMU balancing logic', y)
            r = self.pid.PID_compute(r)
            p = self.pid.PID_compute(p)
            pos = self.posture_balance(r, p, 0)
            self.change_coordinates('Attitude Angle', pos=pos)
            if (
                (self.order[0] == cmd.CMD_BALANCE and self.order[1] == '0')
                or (self.balance_flag and self.order[0] != '')
                or (self.move_count > 180)
            ):
                logging.info('Exiting IMU balancing logic...')
                self.balance_flag = False
                if not self.Thread_condition.is_alive():
                    self.stop_condition_thread = False
                    self.Thread_condition = threading.Thread(target=self.condition)
                    self.Thread_condition.daemon = True
                    self.Thread_condition.start()
                break

    def posture_balance(self, r, p, y, h=1):
        logging.info('Executing posture balance calculation...')
        b = 76
        w = 76
        l = 136
        h = self.height if h != 0 else h
        pos = np.mat([0.0, 0.0, h]).T
        rpy = np.radians([r, p, y])
        R, P, Y = rpy[0], rpy[1], rpy[2]
        rotx = np.mat(
            [[1, 0, 0], [0, math.cos(R), -math.sin(R)], [0, math.sin(R), math.cos(R)]]
        )
        roty = np.mat(
            [[math.cos(P), 0, -math.sin(P)], [0, 1, 0], [math.sin(P), 0, math.cos(P)]]
        )
        rotz = np.mat(
            [[math.cos(Y), -math.sin(Y), 0], [math.sin(Y), math.cos(Y), 0], [0, 0, 1]]
        )
        rot_mat = rotx * roty * rotz
        body_struc = np.mat(
            [
                [l / 2, b / 2, 0],
                [l / 2, -b / 2, 0],
                [-l / 2, b / 2, 0],
                [-l / 2, -b / 2, 0],
            ]
        ).T
        footpoint_struc = np.mat(
            [
                [(l / 2), (w / 2) + 10, self.height - h],
                [(l / 2), (-w / 2) - 10, self.height - h],
                [(-l / 2), (w / 2) + 10, self.height - h],
                [(-l / 2), (-w / 2) - 10, self.height - h],
            ]
        ).T
        AB = np.mat(np.zeros((3, 4)))
        for i in range(4):
            AB[:, i] = pos + rot_mat * footpoint_struc[:, i] - body_struc[:, i]
        return AB


if __name__ == '__main__':
    pass
