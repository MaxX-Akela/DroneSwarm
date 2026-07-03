import logging
import re
import socket
import struct
import subprocess
import time

logger = logging.getLogger(__name__)

CHRONY_TIMEOUT = 2.0
NTP_TIMEOUT = 2.0
NTP_PORT = 123
# NTP epoch (1900-01-01) to Unix epoch (1970-01-01), in seconds.
NTP_UNIX_DELTA = 2208988800

_CHRONY_OFFSET_RE = re.compile(r"System time\s*:\s*([\d.eE+-]+)\s*seconds\s*(fast|slow)")


def get_chrony_offset(timeout=CHRONY_TIMEOUT):
    try:
        output = subprocess.run(
            ["chronyc", "tracking"],
            capture_output=True, text=True, timeout=timeout, check=True,
        ).stdout
    except (OSError, subprocess.SubprocessError) as e:
        logger.debug("chronyc unavailable: %s", e)
        return None

    match = _CHRONY_OFFSET_RE.search(output)
    if not match:
        logger.debug("Could not parse chronyc tracking output")
        return None

    magnitude, direction = match.groups()
    offset = float(magnitude)
    return offset if direction == "fast" else -offset


def get_ntp_offset(server, timeout=NTP_TIMEOUT):
    packet = b"\x1b" + 47 * b"\0"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(timeout)
            t1 = time.time()
            sock.sendto(packet, (server, NTP_PORT))
            reply, _ = sock.recvfrom(48)
            t4 = time.time()
    except OSError as e:
        logger.debug("NTP query to %s failed: %s", server, e)
        return None

    if len(reply) < 48:
        return None

    t2 = struct.unpack("!II", reply[32:40])[0] - NTP_UNIX_DELTA
    t3 = struct.unpack("!II", reply[40:48])[0] - NTP_UNIX_DELTA
    return ((t2 - t1) + (t3 - t4)) / 2.0


def get_time_offset(ntp_server=None):
    offset = get_chrony_offset()
    if offset is not None:
        return {"source": "chrony", "offset_sec": offset, "synced": True}

    if ntp_server:
        offset = get_ntp_offset(ntp_server)
        if offset is not None:
            return {"source": "ntp", "offset_sec": offset, "synced": True}

    return {"source": None, "offset_sec": 0.0, "synced": False}


def corrected_now(offset_sec=0.0):
    return time.time() - offset_sec
