import os
import socket
import time

HOSTNAME = socket.gethostname()
LOG_FILE = "{}.log".format(HOSTNAME)

def log(message, level="INFO"):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    formatted_msg = "[{}] [{}] {}".format(timestamp, level, message)
        
    try:
        with open(LOG_FILE, "a") as f:
            f.write(formatted_msg + "\n")
    except Exception as e:
        print("Ошибка записи лога: {}".format(e))

    return formatted_msg