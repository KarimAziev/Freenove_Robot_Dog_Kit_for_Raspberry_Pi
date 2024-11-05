import io
import os
import time
import fcntl
import socket
import struct
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from picamera2.encoders import Quality
from threading import Thread, Condition, Event, Lock
from typing import Optional
import logging

from Servo import Servo
from Buzzer import BuzzerController
from Control import Control
from ADS7830 import ADS7830
from Ultrasonic import Ultrasonic
from Command import COMMAND as cmd


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame: Optional[bytes] = None
        self.condition = Condition()

    def write(self, buf) -> int:
        with self.condition:
            self.frame = buf
            self.condition.notify_all()
        return len(buf)


class Server:
    def __init__(self):
        self.tcp_flag = False
        self.servo = Servo()
        self.adc = ADS7830()
        self.buzzer = BuzzerController()
        self.control = Control()
        self.sonic = Ultrasonic()
        self.battery_voltage = [8.4] * 5

        # Thread management
        self.video_thread: Optional[Thread] = None
        self.instruction_thread: Optional[Thread] = None
        self.server_socket = socket.socket()
        self.server_socket1 = socket.socket()
        self.connection = None

        self.stop_event = Event()
        self.operation_counter = 0
        self.lock = Lock()

    def get_interface_ip(self) -> str:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ip_addr = socket.inet_ntoa(
            fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', b'wlan0'[:15]))[20:24]
        )
        s.close()
        return ip_addr

    def turn_on_server(self):
        # Get IP address
        HOST = self.get_interface_ip()
        # Port 8001 for video transmission
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.server_socket.bind((HOST, 8001))
        self.server_socket.listen(1)

        # Port 5001 is used for instruction sending and receiving
        self.server_socket1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.server_socket1.bind((HOST, 5001))
        self.server_socket1.listen(1)
        logging.info(f'Server started at {HOST}')

    def turn_off_server(self):
        logging.info("Shutting down the server")
        try:
            if self.connection:
                self.connection.close()
        except Exception as e:
            logging.warning("No client connection for video")
            logging.exception(e)

        try:
            if self.connection1:
                self.connection1.close()
        except Exception as e:
            logging.warning("No client connection for instructions")
            logging.exception(e)

    def reset_server(self):
        logging.info("Resetting the server")
        self.stop_event.set()
        if self.video_thread and self.video_thread.is_alive():
            self.video_thread.join()
        if self.instruction_thread and self.instruction_thread.is_alive():
            self.instruction_thread.join()
        self.turn_off_server()
        self.stop_event.clear()
        self.turn_on_server()
        self.video_thread = Thread(target=self.transmission_video)
        self.instruction_thread = Thread(target=self.receive_instruction)
        self.video_thread.start()
        self.instruction_thread.start()

    def send_data(self, connect: socket.socket, data: str):
        logging.info(f"Sending data: {data.strip()}")
        try:
            connect.sendall(data.encode('utf-8'))
        except Exception as e:
            logging.error(f"Failed to send data: {data.strip()}")
            logging.exception(e)

    def transmission_video(self):
        logging.info("Starting video transmission thread")
        try:
            conn_socket, _ = self.server_socket.accept()
            conn_socket.settimeout(2)
            self.connection = conn_socket.makefile('wb')
        except Exception as e:
            logging.exception(f"Error accepting socket connection: {e}")
            return

        self.server_socket.close()
        logging.info("Video socket connected")

        camera = Picamera2()
        camera.configure(camera.create_video_configuration(main={"size": (400, 300)}))  # type: ignore
        output = StreamingOutput()
        encoder = JpegEncoder(q=95)
        camera.start_recording(encoder, FileOutput(output), quality=Quality.VERY_HIGH)
        logging.info("Video recording started")

        while not self.stop_event.is_set():
            with output.condition:
                output.condition.wait()
                if output.frame is None:
                    logging.warning("Frame is empty, skipping...")
                    continue
                frame = output.frame

            try:
                len_frame = len(frame)
                length_bin = struct.pack('<I', len_frame)

                if self.stop_event.is_set():
                    break

                self.connection.write(length_bin)
                self.connection.write(frame)
                self.connection.flush()

            except Exception as e:
                logging.exception("Error transmitting frame")
                break

        camera.stop_recording()
        camera.close()
        logging.info("Video transmission thread ended")

    def measuring_voltage(self):
        try:
            for i in range(5):
                voltage = self.adc.power(0)
                self.battery_voltage[i] = round(voltage, 2)
            max_voltage = max(self.battery_voltage)
            command = f"{cmd.CMD_POWER}#{max_voltage}\n"
            self.send_data(self.connection1, command)
            self.send_relax_flag()
            self.battery_reminder()
        except Exception as e:
            logging.error("Error measuring voltage")
            logging.exception(e)

    def battery_reminder(self):
        # Adjusted the voltage threshold to appropriate values
        if max(self.battery_voltage) < 6.4:
            logging.error(
                "Battery voltage is too low. Please recharge or replace the batteries."
            )
            self.control.relax(True)
            logging.info("Server shutting down due to low battery.")
            self.turn_off_server()
            os._exit(0)

    def send_relax_flag(self):
        if self.control.move_flag != 2:
            command = f"{cmd.CMD_RELAX}#{self.control.move_flag}\n"
            self.send_data(self.connection1, command)
            self.control.move_flag = 2

    def receive_instruction(self):
        logging.info("Starting instruction reception thread")
        try:
            conn_socket1, client_address1 = self.server_socket1.accept()
            self.connection1 = conn_socket1
            logging.info(f"Connected to client {client_address1}")
        except Exception:
            logging.exception("Error accepting instruction connection")
            return

        self.server_socket1.close()

        while not self.stop_event.is_set():
            try:
                all_data_bytes = self.connection1.recv(1024)
                all_data = all_data_bytes.decode('utf-8')
                if not all_data:
                    logging.warning(
                        "Received empty data, client might have disconnected"
                    )
                    break

                with self.lock:
                    self.operation_counter += 1
                    if self.operation_counter % 10 == 0:
                        logging.info(f"Processed {self.operation_counter} instructions")

                cmd_array = all_data.strip().split('\n')
                for one_cmd in cmd_array:
                    data = one_cmd.strip().split("#")
                    logging.info(f"Received instruction: {one_cmd.strip()}")
                    if not data or data[0] == "":
                        continue

                    command = data[0]
                    args = data[1:]

                    # Process commands
                    if command == cmd.CMD_BUZZER:
                        self.buzzer.run(args[0])
                    elif command == cmd.CMD_HEAD:
                        angle = int(args[0])
                        self.servo.setServoAngle(15, angle)
                    elif command == cmd.CMD_SONIC:
                        distance = self.sonic.get_distance()
                        response = f"{cmd.CMD_SONIC}#{distance}\n"
                        self.send_data(self.connection1, response)
                    elif command == cmd.CMD_POWER:
                        self.measuring_voltage()
                    elif command == cmd.CMD_WORKING_TIME:
                        self.send_working_time()
                    else:
                        self.control.order = data
                        self.control.timeout = time.time()
                        logging.info(f"Control order updated: {data}")

            except Exception:
                logging.exception("Error receiving instructions")
                break

        logging.info("Instruction reception thread ended")

    def send_working_time(self):
        current_time = time.time()
        if self.control.move_timeout != 0 and self.control.relax_flag:
            if self.control.move_count > 180:
                extra_time = round(self.control.move_count - 180)
                command = f"{cmd.CMD_WORKING_TIME}#180#{extra_time}\n"
            else:
                elapsed_time = round(current_time - self.control.move_timeout)
                command = f"{cmd.CMD_WORKING_TIME}#{round(self.control.move_count)}#{elapsed_time}\n"
        else:
            command = f"{cmd.CMD_WORKING_TIME}#{round(self.control.move_count)}#0\n"
        logging.info(f"Sending working time: {command.strip()}")
        self.send_data(self.connection1, command)


if __name__ == '__main__':
    server = Server()
    server.turn_on_server()
    server.video_thread = Thread(target=server.transmission_video)
    server.instruction_thread = Thread(target=server.receive_instruction)
    server.video_thread.start()
    server.instruction_thread.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Server shutting down due to KeyboardInterrupt")
        server.stop_event.set()
        server.video_thread.join()
        server.instruction_thread.join()
        server.turn_off_server()
