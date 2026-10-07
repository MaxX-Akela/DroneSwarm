import socket
import logging
import collections
import time

ERROR_LOG = collections.deque(maxlen=30)


class ErrorLogHandler(logging.Handler):
    """Keeps the latest warnings/errors so they can be shown on the server."""

    def emit(self, record):
        try:
            ERROR_LOG.append("{} [{}] {}".format(time.strftime("%H:%M:%S"), record.levelname,
                                                 record.getMessage()[:300]))
        except Exception:
            pass


def get_copter_id():
    return socket.gethostname()

def setup_logger():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        handlers=[
            logging.FileHandler(f"{get_copter_id()}.log"),
            logging.StreamHandler(),
            ErrorLogHandler(level=logging.WARNING),
        ]
    )
    return logging.getLogger("SwarmClient")

logger = setup_logger()