import os
import csv
import copy
import math
import time
import numpy
import logging
import threading

try:
    import modules.flight as flight
except ImportError:
    pass

try:
    import modules.led as led
except ImportError:
    pass

def load_animation(csv_path):
    frames = []

    with open(csv_path, newline='') as f:
        reader = csv.reader(f)
        rows = list(reader)

        for row in rows[1:]:
            if len(row) < 8:
                continue

            frame = {
                "frame": int(row[0]),
                "x": float(row[1]),
                "y": float(row[2]),
                "z": float(row[3]),
                "yaw": float(row[4]),
                "r": int(row[5]),
                "g": int(row[6]),
                "b": int(row[7]),
            }
            frames.append(frame)

    return frames


