import socket
import logging

def get_copter_id():
    return socket.gethostname()

def setup_logger():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        handlers=[
            logging.FileHandler(f"{get_copter_id()}.log"),
            logging.StreamHandler()
        ]
    )
    return logging.getLogger("SwarmClient")

logger = setup_logger()