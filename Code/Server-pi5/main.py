import logging
import sys
import getopt
import time
from ui_server import Ui_server
from PyQt5.QtCore import QCoreApplication
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtGui
import threading
from Server import Server
from typing import Optional

# Configure logging
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    handlers=[logging.StreamHandler(sys.stdout)])


class MyWindow(QMainWindow, Ui_server):
    def __init__(self, server: Server, user_ui: bool = True, start_tcp: bool = False):
        super(MyWindow, self).__init__()
        self.user_ui = user_ui
        self.start_tcp = start_tcp
        self.server = server
        self.video_thread: Optional[threading.Thread] = None
        self.instruction_thread: Optional[threading.Thread] = None

        logging.info("Server initialized")

        if self.user_ui:
            self.setupUi(self)
            self.pushButton_On_And_Off.clicked.connect(self.on_and_off_server)
            logging.debug("UI setup and button clicked signal connected")

        if self.start_tcp:
            self.start_server_threads()
            if self.user_ui:
                self.pushButton_On_And_Off.setText('Off')
                self.states.setText('On')

    def on_and_off_server(self):
        if self.pushButton_On_And_Off.text() == 'On':
            self.pushButton_On_And_Off.setText('Off')
            self.states.setText('On')
            self.start_server_threads()
            logging.info("Server turned ON")
        else:
            self.pushButton_On_And_Off.setText('On')
            self.states.setText('Off')
            self.stop_server_threads()
            logging.info("Server turned OFF")

    def start_server_threads(self):
        logging.info("Starting server threads")
        self.server.turn_on_server()
        self.server.tcp_flag = True
        self.server.stop_event.clear()

        self.video_thread = threading.Thread(target=self.server.transmission_video)
        self.video_thread.start()
        self.instruction_thread = threading.Thread(target=self.server.receive_instruction)
        self.instruction_thread.start()
        logging.info("Server threads started")

    def stop_server_threads(self):
        logging.info("Stopping server threads")
        self.server.tcp_flag = False
        self.server.stop_event.set()
        if self.video_thread and self.video_thread.is_alive():
            self.video_thread.join()
            logging.debug("Video thread stopped")
        if self.instruction_thread and self.instruction_thread.is_alive():
            self.instruction_thread.join()
            logging.debug("Instruction thread stopped")
        self.server.turn_off_server()
        logging.info("Server threads and sockets shut down")

    def closeEvent(self, a0: QtGui.QCloseEvent):
        logging.info("Close event triggered")
        self.stop_server_threads()
        if self.user_ui:
            QCoreApplication.instance().exit()
            logging.info("Application exited")

if __name__ == '__main__':
    user_ui = True
    start_tcp = False
    opts, args = getopt.getopt(sys.argv[1:], "tn")
    for o, _ in opts:
        if o == '-t':
            logging.info("TCP option selected")
            start_tcp = True
        elif o == '-n':
            logging.info("Non-UI mode selected")
            user_ui = False


    server = Server()

    if user_ui:
        app = QApplication(sys.argv)
        window = MyWindow(server=server, user_ui=user_ui, start_tcp=start_tcp)
        window.show()
        logging.debug("UI shown")
        sys.exit(app.exec_())
    else:
        window = MyWindow(server=server, user_ui=user_ui, start_tcp=start_tcp)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logging.info("KeyboardInterrupt received, stopping server")
            window.stop_server_threads()