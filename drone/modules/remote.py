"""Server-initiated housekeeping on the drone: files, services, shell, FCU params.

The command channel is unauthenticated, so this assumes a trusted show network.
"""
import configparser
import logging
import os
import subprocess

logger = logging.getLogger(__name__)

# Server menu name -> systemd unit.
SERVICES = {"chrony": "chrony", "ros": "clover", "swarm": "droneswarm"}

COMMAND_TIMEOUT = 60.0
PARAMS_TIMEOUT = 180.0
MAX_OUTPUT = 4000


def _sudo(*cmd, data=None, timeout=COMMAND_TIMEOUT):
    return subprocess.run(["sudo", "-n", *cmd], input=data, capture_output=True,
                          timeout=timeout, check=True)


def write_file(path, data):
    """Write bytes to path atomically; falls back to sudo for root-owned locations."""
    path = os.path.expanduser(path)
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        tmp_path = path + ".tmp"
        with open(tmp_path, "wb") as f:
            f.write(data)
        os.replace(tmp_path, path)
    except PermissionError:
        _sudo("mkdir", "-p", os.path.dirname(path) or ".")
        _sudo("tee", path, data=data)
    return path


def merge_ini(path, ini_text):
    """'Modify' mode: update/add only the keys present in ini_text, keep the rest."""
    current = configparser.ConfigParser(interpolation=None)
    current.read(path, encoding="utf-8")
    incoming = configparser.ConfigParser(interpolation=None)
    incoming.read_string(ini_text)
    for section in incoming.sections():
        if not current.has_section(section):
            current.add_section(section)
        for key, value in incoming.items(section):
            current.set(section, key, value)
    with open(path, "w", encoding="utf-8") as f:
        current.write(f)


def run_command(command, timeout=COMMAND_TIMEOUT):
    """Run a shell command; returns (returncode, combined output)."""
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return -1, f"timed out after {timeout:.0f}s"
    output = (result.stdout + result.stderr).strip()
    return result.returncode, output[-MAX_OUTPUT:]


def restart_service(name):
    unit = SERVICES.get(name)
    if unit is None:
        raise ValueError(f"unknown service '{name}'")
    _sudo("systemctl", "restart", unit)


def restart_service_detached(name):
    """For the service running this very client: don't wait for our own death."""
    subprocess.Popen(["sudo", "-n", "systemctl", "restart", SERVICES[name]])


def reboot():
    subprocess.Popen(["sudo", "-n", "reboot"])


def load_fcu_params(path):
    return run_command(f"rosrun mavros mavparam load '{path}'", timeout=PARAMS_TIMEOUT)
