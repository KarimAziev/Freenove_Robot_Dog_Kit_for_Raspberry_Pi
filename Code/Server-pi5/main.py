import logging
import sys
import time
import threading
from Server import Server
from typing import Optional


logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    stream=sys.stdout,
)

logger = logging.getLogger(__name__)


class ServerController:
    """
    ServerController - controls the start and stop of the server and its threads
    """

    def __init__(self, server: Server, start_tcp: bool = False):
        self.server = server
        self.video_thread: Optional[threading.Thread] = None
        self.instruction_thread: Optional[threading.Thread] = None

        if start_tcp:
            self.start_server_threads()

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
            logger.info("Stopping video thread")
            self.video_thread.join()
            logger.info("Video thread stopped")

        if self.instruction_thread and self.instruction_thread.is_alive():
            logger.info("Stopping instruction thread")
            self.instruction_thread.join()
            logger.info("Instruction thread stopped")

        self.server.turn_off_server()
        logger.info("Server has been shut down gracefully.")

    def run_forever(self):
        """
        Keeps the server running until interrupted.
        """
        logger.info("Running in non-UI mode.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("KeyboardInterrupt received, stopping the server.")
            self.stop_server_threads()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Run the server without UI.")
    parser.add_argument(
        "-n", "--no-tcp", action="store_true", help="Disable TCP completely"
    )

    args = parser.parse_args()
    start_tcp = not args.no_tcp

    server = Server()

    controller = ServerController(server=server, start_tcp=start_tcp)
    controller.run_forever()
