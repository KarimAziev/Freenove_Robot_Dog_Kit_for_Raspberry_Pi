# -*- coding: utf-8 -*-
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
from threading import Condition
import threading
from Led import Led
from Servo import Servo
from Thread import stop_thread
from Buzzer import Buzzer
from Control import Control
from ADS7830 import ADS7830
from Ultrasonic import Ultrasonic
from Command import COMMAND as cmd
import logging


# Setup logging
LOG_FORMAT = '%(asctime)s - %(levelname)s - %(message)s'
logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
logger = logging.getLogger('Server')


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()
        return len(buf)

class Server:
    def __init__(self):
        self.tcp_flag = False
        self.led = Led()
        self.servo = Servo()
        self.adc = ADS7830()
        self.buzzer = Buzzer()
        self.control = Control()
        self.sonic = Ultrasonic()
        self.control.Thread_condition.start()
        self.battery_voltage = [8.4, 8.4, 8.4, 8.4, 8.4]

    def get_interface_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(s.fileno(),
                                            0x8915,
                                            struct.pack('256s', b'wlan0'[:15])
                                            )[20:24])

    def turn_on_server(self):
        # ip address
        HOST = self.get_interface_ip()
        # Port 8001 for video transmission
        self.server_socket = socket.socket()
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.server_socket.bind((HOST, 8001))
        self.server_socket.listen(1)

        # Port 5001 is used for instruction sending and receiving
        self.server_socket1 = socket.socket()
        self.server_socket1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.server_socket1.bind((HOST, 5001))
        self.server_socket1.listen(1)
        logger.info(f'Server started at {HOST}')

    def turn_off_server(self):
        try:
            logger.info("Shutting down the server")
            self.connection.close()
            self.connection1.close()
        except Exception as e:
            logger.warning("No client connection")
            logger.exception(e)

    def reset_server(self):
        logger.info("Resetting down the server")
        self.turn_off_server()
        self.turn_on_server()
        self.video = threading.Thread(target=self.transmission_video)
        self.instruction = threading.Thread(target=self.receive_instruction)
        self.video.start()
        self.instruction.start()

    def send_data(self, connect, data):
        logger.info(f"sending data {data}")
        try:
            connect.send(data.encode('utf-8'))
        except Exception as e:
            logger.error(f"Failed to send data: {data}")
            logger.exception(e)

    def transmission_video(self):
        logger.info("transmission_video start")
        try:
            self.connection, self.client_address = self.server_socket.accept()
            self.connection = self.connection.makefile('wb')
        except Exception as e:
            logger.exception(f"Error accepting socket connection {e}")
            return

        self.server_socket.close()
        logger.info("Socket video connected")

        camera = Picamera2()
        camera.configure(camera.create_video_configuration(main={"size": (400, 300)})) # type: ignore
        output = StreamingOutput()
        encoder = JpegEncoder(q=95)
        camera.start_recording(encoder, FileOutput(output), quality=Quality.VERY_HIGH)
        logger.info("Recording started")

        while True:
            with output.condition:
                output.condition.wait()
                if output.frame is None:
                    logger.info("Frame is empty, skipping...")
                    continue
                frame = output.frame
            try:
                lenFrame = len(output.frame)
                lengthBin = struct.pack('<I', lenFrame)
                self.connection.write(lengthBin)
                self.connection.write(frame)
            except Exception as e:
                logger.exception("Error transmitting frame to connection")
                camera.stop_recording()
                camera.close()
                logger.info("End transmit video")
                break

    def measuring_voltage(self, connect):
        try:
            for i in range(5):
                self.battery_voltage[i] = round(self.adc.power(0), 2)
            command = cmd.CMD_POWER+'#'+str(max(self.battery_voltage))+"\n"
            self.send_data(connect, command)
            self.sednRelaxFlag()
            self.battery_reminder()
        except Exception as e:
            logger.error("Error measuring voltage")
            logger.exception(e)

    def battery_reminder(self):
        if max(self.battery_voltage) < -6.4:
            self.turn_off_server()
            self.control.relax(True)
            logger.error("The battery power is too low. Please recharge or replace the batteries.")
            logger.info("Server shutting down due to low battery.")
            os._exit(0)

    def sednRelaxFlag(self):
        if self.control.move_flag != 2:
            command = cmd.CMD_RELAX + "#" + str(self.control.move_flag) + "\n"
            self.send_data(self.connection1, command)
            self.control.move_flag = 2

    def receive_instruction(self):
        try:
            self.connection1, self.client_address1 = self.server_socket1.accept()
            logger.info("Client connection successful!")
        except Exception as e:
            logger.error("Client connect failed")
            return

        self.server_socket1.close()
        global operation_counter
        operation_counter = 0
        while True:
            try:
                global operation_counter
                allData = self.connection1.recv(1024).decode('utf-8')
                operation_counter += 1
                if operation_counter % 10 == 0:
                    logging.info(f"Received {operation_counter} operations")
            except Exception as e:
                logger.error(e)
                if self.tcp_flag:
                    if max(self.battery_voltage) < 6.4:
                        self.turn_off_server()
                        self.control.relax(True)
                        logger.error("The battery power is too low. Please recharge or replace the batteries.")
                        logger.info("Server shutting down due to low battery.")
                        os._exit(0)
                    break
                else:
                    break

            if allData == "" and self.tcp_flag:
                self.reset_server()
                break

            cmdArray = allData.split('\n')
            if cmdArray[-1] != "":
                cmdArray = cmdArray[:-1]

            for oneCmd in cmdArray:
                data = oneCmd.split("#")
                logger.info("receive_instruction oneCmd %s" % oneCmd)
                if not data or data[0] == "":
                    continue

                if cmd.CMD_BUZZER in data:
                    self.buzzer.run(data[1])
                elif cmd.CMD_LED in data or cmd.CMD_LED_MOD in data:
                    thread_led = None
                    if self.led.Ledsupported == 1:
                        try:
                            if thread_led is not None:
                                stop_thread(thread_led)
                        except:
                            pass
                        thread_led = threading.Thread(target=self.led.light, args=(data,))
                        thread_led.start()
                elif cmd.CMD_HEAD in data:
                    logger.info("receive_instruction oneCmd %s" % oneCmd)
                    self.servo.setServoAngle(15, int(data[1]))
                elif cmd.CMD_SONIC in data:
                    logger.info("receive_instruction oneCmd %s" % oneCmd)
                    command = cmd.CMD_SONIC + '#' + str(self.sonic.get_distance()) + "\n"
                    self.send_data(self.connection1, command)
                elif cmd.CMD_POWER in data:
                    logger.info("Measuring power command executed")
                    self.measuring_voltage(self.connection1)
                elif cmd.CMD_WORKING_TIME in data:
                    logger.info("Working time command executed")
                    if self.control.move_timeout != 0 and self.control.relax_flag:
                        if self.control.move_count > 180:
                            logger.info(f"Working time exceeds 180s {self.control.move_count}")
                            command = cmd.CMD_WORKING_TIME + '#' + str(180) + '#' + str(round(self.control.move_count - 180)) + "\n"
                            logger.info(f"Working time exceeds 180s {self.control.move_count}, extra time: {round(self.control.move_count - 180)}")
                        else:
                            if self.control.move_count == 0:
                                command = cmd.CMD_WORKING_TIME + '#' + str(round(self.control.move_count)) + '#' + str(round((time.time() - self.control.move_timeout)+60)) + "\n"
                            else:
                                command = cmd.CMD_WORKING_TIME + '#' + str(round(self.control.move_count)) + '#' + str(round(time.time() - self.control.move_timeout)) + "\n"
                    else:
                        command = cmd.CMD_WORKING_TIME + '#' + str(round(self.control.move_count)) + '#' + str(0) + "\n"
                    logger.info("Working time command response generated: %s" % command)
                    self.send_data(self.connection1, command)
                else:
                    self.control.order = data
                    logger.info("receive_instruction received order: %s" % data)
                    self.control.timeout = time.time()
                    logger.info("receive_instruction complete, control order updated.")

        logger.info("Receiving thread shutting down")
        self.control.relax_flag = False
        self.control.order[0] = cmd.CMD_RELAX


if __name__ == '__main__':
    pass
