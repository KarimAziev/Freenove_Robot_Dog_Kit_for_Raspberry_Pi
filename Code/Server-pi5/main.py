# -*- coding: utf-8 -*-

import logging
import sys, getopt
from ui_server import Ui_server
from PyQt5.QtCore import QCoreApplication
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtGui
import threading
from Server import Server
from Thread import stop_thread


logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    handlers=[logging.StreamHandler(sys.stdout)])


class MyWindow(QMainWindow, Ui_server):
    def __init__(self):
        self.user_ui = True
        self.start_tcp = False
        self.server = Server()

        self.parseOpt()

        logging.info("Server initialized")

        if self.user_ui:
            self.app = QApplication(sys.argv)
            super(MyWindow, self).__init__()
            self.setupUi(self)
            self.pushButton_On_And_Off.clicked.connect(self.on_and_off_server)
            logging.debug("UI setup and button clicked signal connected")
            self.on_and_off_server()

        if self.start_tcp:
            self.server.turn_on_server()
            self.server.tcp_flag = True
            self.video = threading.Thread(target=self.server.transmission_video)
            self.video.start()
            self.instruction = threading.Thread(target=self.server.receive_instruction)
            self.instruction.start()
            logging.info("TCP server started and threads initialized")

            if self.user_ui:
                self.pushButton_On_And_Off.setText('Off')
                self.states.setText('On')

    def parseOpt(self):
        self.opts, self.args = getopt.getopt(sys.argv[1:], "tn")
        for o, _ in self.opts:
            if o in ('-t'):
                logging.info("TCP option selected")
                self.start_tcp = True
            elif o in ('-n'):
                logging.info("Non-UI mode selected")
                self.user_ui = False

    def on_and_off_server(self):
        if self.pushButton_On_And_Off.text() == 'On':
            self.pushButton_On_And_Off.setText('Off')
            self.states.setText('On')
            self.server.turn_on_server()
            self.server.tcp_flag = True
            self.video = threading.Thread(target=self.server.transmission_video)
            self.video.start()
            self.instruction = threading.Thread(target=self.server.receive_instruction)
            self.instruction.start()
            logging.info("Server turned ON")
        else:
            self.pushButton_On_And_Off.setText('On')
            self.states.setText('Off')
            self.server.tcp_flag = False
            try:
                stop_thread(self.video)
                stop_thread(self.instruction)
                logging.debug("Video and Instruction threads stopped")
            except Exception as e:
                logging.error(f"Error stopping threads: {e}")

            self.server.turn_off_server()
            logging.info("Server turned OFF")

    def closeEvent(self, a0: QtGui.QCloseEvent):
        logging.info(f"close event: {a0}")
        try:
            stop_thread(self.video)
            stop_thread(self.instruction)
            logging.debug("Threads stopped on close event")
        except:
            pass
        try:
            self.server.server_socket.shutdown(2)
            self.server.server_socket1.shutdown(2)
            self.server.turn_off_server()
            logging.info("Server sockets shut down")
        except:
            logging.error("Error shutting down server sockets")

        if self.user_ui:
            QCoreApplication.instance().quit()



if __name__ == '__main__':
    myshow = None
    try:
        myshow = MyWindow()
        if myshow.user_ui == True:
            myshow.show()
            logging.debug("UI shown")
            sys.exit(myshow.app.exec_())
        else:
            try:
                pass
            except KeyboardInterrupt:
                myshow.close()
        while True:
            pass
    except KeyboardInterrupt:
        if myshow is not None:
            myshow.close()
        logging.info("Program interrupted and closing")