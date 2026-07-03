#!/usr/bin/python3
import json
import math
import os
import socket
import struct
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState

import modules.checks as checks
import modules.failsafe as failsafe
import modules.network as network
from modules.commander import TaskManager
from modules.config import config
from modules.utils import get_copter_id, setup_logger

CLIENT_VERSION = "0.1.0"

DISCOVERY_PORT = 9000
DISCOVERY_PERIOD = 2.0
TELEMETRY_PERIOD = 0.5
STATUS_CHECK_PERIOD = 5.0
OFFSET_CHECK_PERIOD = 5.0
SOCKET_TIMEOUT = 2.0

logger = setup_logger()


def _yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class NetworkManager:
    def __init__(self, commander, copter_id, watchdog, discovery_port=DISCOVERY_PORT):
        self.commander = commander
        self.copter_id = copter_id
        self.watchdog = watchdog
        self.discovery_port = discovery_port
        self.telemetry_port = discovery_port + 1
        self.running = True
        self.server_ip = None

        self.telemetry = {
            "copter_id": self.copter_id,
            "version": CLIENT_VERSION,
            "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0,
            "frame_id": config.flight_frame_id,
            "bat": 0.0, "armed": False, "mode": "IDLE", "connected": False,
        }
        self.checks_status = {"ok": None, "problems": ["not checked yet"]}
        self.time_offset = {"source": None, "offset_sec": 0.0, "synced": False}

        rospy.Subscriber("mavros/state", State, self._state_cb)
        rospy.Subscriber("mavros/battery", BatteryState, self._bat_cb)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self._pose_cb)

    def _state_cb(self, msg):
        self.telemetry["armed"], self.telemetry["mode"] = msg.armed, msg.mode
        self.telemetry["connected"] = msg.connected

    def _bat_cb(self, msg):
        self.telemetry["bat"] = round(msg.voltage, 2)

    def _pose_cb(self, msg):
        self.telemetry["x"] = round(msg.pose.position.x, 2)
        self.telemetry["y"] = round(msg.pose.position.y, 2)
        self.telemetry["z"] = round(msg.pose.position.z, 2)
        self.telemetry["yaw"] = round(_yaw_from_quaternion(msg.pose.orientation), 2)

    def start(self):
        threading.Thread(target=self._command_loop, daemon=True).start()
        threading.Thread(target=self._telemetry_loop, daemon=True).start()
        threading.Thread(target=self._status_loop, daemon=True).start()
        threading.Thread(target=self._offset_loop, daemon=True).start()

    def _discover_server(self):
        """Broadcast until a server answers; returns (server_ip, tcp_port)."""
        request = json.dumps({"type": "discover", "copter_id": self.copter_id}).encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.settimeout(SOCKET_TIMEOUT)
            while self.running and not rospy.is_shutdown():
                try:
                    sock.sendto(request, ("<broadcast>", self.discovery_port))
                    data, addr = sock.recvfrom(1024)
                    reply = json.loads(data.decode("utf-8"))
                    if reply.get("type") == "discover_reply":
                        return addr[0], reply["tcp_port"]
                except (socket.timeout, OSError, ValueError, KeyError) as e:
                    logger.debug("Discovery attempt failed: %s", e)
                rospy.sleep(DISCOVERY_PERIOD)
        return None, None

    @staticmethod
    def _send_framed(sock, obj):
        data = json.dumps(obj).encode("utf-8")
        sock.sendall(struct.pack("!I", len(data)) + data)

    def _command_loop(self):
        while self.running and not rospy.is_shutdown():
            server_ip, tcp_port = self._discover_server()
            if server_ip is None:
                continue
            self.server_ip = server_ip
            logger.info("Server found at %s, connecting on port %s", server_ip, tcp_port)

            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.connect((server_ip, tcp_port))
                    self._send_framed(sock, {"type": "hello", "copter_id": self.copter_id})
                    logger.info("Connected to server %s", server_ip)
                    while self.running and not rospy.is_shutdown():
                        raw_len = self._recv_exact(sock, 4)
                        if not raw_len:
                            break
                        msg_len = struct.unpack("!I", raw_len)[0]
                        data = self._recv_exact(sock, msg_len)
                        if not data:
                            break
                        msg = json.loads(data.decode("utf-8"))
                        self._handle_command(msg.get("action"), msg.get("params", {}))
            except (OSError, ValueError) as e:
                logger.warning("Command connection error: %s. Rediscovering...", e)

    def _handle_command(self, action, params):
        if action == "check":
            try:
                self.checks_status = checks.self_check()
            except Exception as e:
                self.checks_status = {"ok": False, "problems": [f"self_check crashed: {e!r}"]}
        elif action == "set_config":
            self._set_config(params.get("ini_text", ""))
        else:
            self.commander.do_action(action, **params)

    def _set_config(self, ini_text):
        ini_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drone.ini")
        try:
            with open(ini_path, "w", encoding="utf-8") as f:
                f.write(ini_text)
            config.load(ini_path)
            logger.info("Config updated from server (%d bytes)", len(ini_text))
        except OSError as e:
            logger.error("Failed to write pushed config: %s", e)

    @staticmethod
    def _recv_exact(sock, size):
        chunks = []
        remaining = size
        while remaining > 0:
            chunk = sock.recv(remaining)
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _telemetry_loop(self):
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while self.running and not rospy.is_shutdown():
            if self.server_ip:
                payload = dict(self.telemetry)
                payload["animation_id"] = self.commander.animation.id
                start_frame = self.commander.animation.get_start_frame("fly")
                payload["start_pos"] = start_frame.get_pos() if start_frame else []
                payload["system"] = {"ok": self.telemetry["connected"]}
                payload["sensors"] = {"ok": self.watchdog.sensors_ok()}
                payload["checks"] = self.checks_status
                payload["time_offset"] = self.time_offset
                try:
                    udp_sock.sendto(json.dumps(payload).encode("utf-8"), (self.server_ip, self.telemetry_port))
                except OSError as e:
                    logger.debug("Telemetry send failed: %s", e)
            rospy.sleep(TELEMETRY_PERIOD)

    def _status_loop(self):
        while self.running and not rospy.is_shutdown():
            try:
                self.checks_status = checks.self_check()
            except Exception as e:
                self.checks_status = {"ok": False, "problems": [f"self_check crashed: {e!r}"]}
            rospy.sleep(STATUS_CHECK_PERIOD)

    def _offset_loop(self):
        while self.running and not rospy.is_shutdown():
            self.time_offset = network.get_time_offset(ntp_server=self.server_ip)
            rospy.sleep(OFFSET_CHECK_PERIOD)


def main():
    rospy.init_node("drone_swarm_client", anonymous=True)
    copter_id = get_copter_id()

    rospy.loginfo(f"=== Drone {copter_id} is starting ===")

    ini_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drone.ini")
    config.load(ini_path)

    commander = TaskManager()

    watchdog = failsafe.Watchdog()
    rospy.Timer(rospy.Duration(0.5), watchdog.check)

    net = NetworkManager(commander, copter_id, watchdog)
    net.start()

    rospy.loginfo("System READY")

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
