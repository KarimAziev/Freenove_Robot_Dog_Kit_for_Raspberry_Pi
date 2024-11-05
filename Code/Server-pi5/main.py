import logging
import sys
import time
from ui_server import Ui_server
from PyQt5.QtCore import QCoreApplication
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtGui
import threading
from Server import Server
from typing import Optional


logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
)

logger = logging.getLogger(__name__)


class MyWindow(QMainWindow, Ui_server):
    def __init__(self, server: Server, user_ui: bool = True, start_tcp: bool = False):
        super(MyWindow, self).__init__()
        self.user_ui = user_ui
        self.start_tcp = start_tcp
        self.server = server
        self.video_thread: Optional[threading.Thread] = None
        self.instruction_thread: Optional[threading.Thread] = None

        logger.info("Server initialized")

        if self.user_ui:
            self.setupUi(self)
            self.pushButton_On_And_Off.clicked.connect(self.on_and_off_server)
            logger.debug("UI setup and button clicked signal connected")
            self.on_and_off_server()

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
            logger.info("Server turned ON")
        else:
            self.pushButton_On_And_Off.setText('On')
            self.states.setText('Off')
            self.stop_server_threads()
            logger.info("Server turned OFF")

    def start_server_threads(self):
        logger.info("Starting server threads")
        self.server.turn_on_server()
        self.server.tcp_flag = True
        self.server.stop_event.clear()

        self.video_thread = threading.Thread(target=self.server.transmission_video)
        self.video_thread.start()
        self.instruction_thread = threading.Thread(
            target=self.server.receive_instruction
        )
        self.instruction_thread.start()
        logger.info("Server threads started")

    def stop_server_threads(self):
        logger.info("Stopping server threads")
        self.server.tcp_flag = False
        self.server.stop_event.set()
        if self.video_thread and self.video_thread.is_alive():
            self.video_thread.join()
            logger.info("Video thread stopped")
        if self.instruction_thread and self.instruction_thread.is_alive():
            self.instruction_thread.join()
            logger.info("Instruction thread stopped")
        self.server.turn_off_server()
        logger.info("Server threads and sockets shut down")

    def closeEvent(self, a0: QtGui.QCloseEvent):
        logger.info("Close event triggered %s", a0)
        self.stop_server_threads()
        if self.user_ui:
            QCoreApplication.instance().exit()
            logger.info("Application exited")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Run the application.")
    parser.add_argument("-n", "--no-tcp", action="store_true", help="Disable TCP")
    parser.add_argument("-u", "--ui", action="store_true", help="Allow UI")

    args = parser.parse_args()
    user_ui = args.ui
    start_tcp = not args.no_tcp

    server = Server()

    if user_ui:
        app = QApplication(sys.argv)
        window = MyWindow(server=server, user_ui=user_ui, start_tcp=start_tcp)
        window.show()
        logger.debug("UI shown")
        sys.exit(app.exec_())
    else:
        window = MyWindow(server=server, user_ui=user_ui, start_tcp=start_tcp)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("KeyboardInterrupt received, stopping server")
            window.stop_server_threads()
