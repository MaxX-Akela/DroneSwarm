#!/usr/bin/python3
import base64
import configparser
import json
import math
import os
import socket
import struct
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState

import modules.checks as checks
import modules.failsafe as failsafe
import modules.flight as flight
import modules.network as network
import modules.remote as remote
from modules.commander import TaskManager
from modules.config import config
from modules.utils import get_copter_id, setup_logger
from modules.version import get_version

CLIENT_VERSION = "0.1.0"  # fallback when the checkout has no .git

DISCOVERY_PORT = 9000
DISCOVERY_PERIOD = 2.0
TELEMETRY_PERIOD = 0.5
STATUS_CHECK_PERIOD = 5.0
OFFSET_CHECK_PERIOD = 5.0
SOCKET_TIMEOUT = 2.0

logger = setup_logger()

REMOTE_ACTIONS = ("write_file", "run_command", "restart_service", "reboot", "load_fcu_params")


class NetworkManager:
    def __init__(self, commander, copter_id, watchdog, discovery_port=DISCOVERY_PORT):
        self.commander = commander
        self.copter_id = copter_id
        self.watchdog = watchdog
        self.discovery_port = discovery_port
        self.telemetry_port = discovery_port + 1
        self.running = True
        self.server_ip = None
        self._sock = None
        self._send_lock = threading.Lock()

        self.telemetry = {
            "copter_id": self.copter_id,
            "version": get_version(CLIENT_VERSION),
            "x": None, "y": None, "z": None, "yaw": None,
            "frame_id": flight.FRAME_ID,
            "bat": 0.0, "armed": False, "mode": "IDLE", "connected": False,
        }
        self.checks_status = {"ok": None, "problems": ["not checked yet"]}
        self.time_offset = {"source": None, "offset_sec": 0.0, "synced": False}

        rospy.Subscriber("mavros/state", State, self._state_cb)
        rospy.Subscriber("mavros/battery", BatteryState, self._bat_cb)

    def _state_cb(self, msg):
        self.telemetry["armed"], self.telemetry["mode"] = msg.armed, msg.mode
        self.telemetry["connected"] = msg.connected

    def _bat_cb(self, msg):
        self.telemetry["bat"] = round(msg.voltage, 2)

    def _update_pose(self):
        """Pose in aruco_map from the clover tf tree; None fields when the map isn't visible."""
        pose = flight.get_pose()
        if pose is None:
            self.telemetry.update(x=None, y=None, z=None, yaw=None)
        else:
            self.telemetry.update(x=round(pose.x, 2), y=round(pose.y, 2), z=round(pose.z, 2),
                                  yaw=None if math.isnan(pose.yaw) else round(pose.yaw, 2))

    def _states(self):
        """Per-column health, 'ok' / 'warn' / 'fail', shown as cell colors on the server."""
        t = self.telemetry
        sensors = "ok" if self.watchdog.sensors_ok() else "fail"
        if sensors == "ok" and not self.watchdog.rangefinder_ok():
            sensors = "warn"

        if self.watchdog.is_emergency():
            mode = "fail"
        elif t["armed"] and t["mode"] != "OFFBOARD":
            mode = "warn"
        else:
            mode = "ok"

        return {
            "system": "ok" if t["connected"] else "fail",
            "sensors": sensors,
            "battery": checks.battery_state(t["bat"]),
            "mode": mode,
            "position": "ok" if t["x"] is not None else "fail",
        }

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

    def _send_framed(self, sock, obj):
        data = json.dumps(obj).encode("utf-8")
        with self._send_lock:
            sock.sendall(struct.pack("!I", len(data)) + data)

    def _reply(self, text):
        """Show a message in the server console (and in our own log)."""
        logger.info(text)
        sock = self._sock
        if sock is None:
            return
        try:
            self._send_framed(sock, {"type": "log", "text": text})
        except OSError as e:
            logger.debug("Reply to server failed: %s", e)

    def _command_loop(self):
        while self.running and not rospy.is_shutdown():
            server_ip, tcp_port = self._discover_server()
            if server_ip is None:
                continue
            if server_ip != self.server_ip:
                network.set_chrony_server(server_ip, previous_ip=self.server_ip)
            self.server_ip = server_ip
            logger.info("Server found at %s, connecting on port %s", server_ip, tcp_port)

            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.connect((server_ip, tcp_port))
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self._send_framed(sock, {"type": "hello", "copter_id": self.copter_id})
                    self._sock = sock
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
            finally:
                self._sock = None

    def _handle_command(self, action, params):
        if action == "check":
            # self_check can block for seconds on ROS timeouts; keep the command loop free.
            threading.Thread(target=self._run_check, daemon=True).start()
        elif action == "set_config":
            self._set_config(params.get("ini_text", ""), params.get("mode", "rewrite"))
        elif action == "set_animation":
            self._set_animation(params.get("csv_text", ""))
        elif action in REMOTE_ACTIONS:
            # File writes, shell commands and service restarts can take a while:
            # keep the command loop free for land/stop.
            threading.Thread(target=self._run_remote, args=(action, params), daemon=True).start()
        else:
            self.commander.do_action(action, **params)

    def _run_remote(self, action, params):
        needs_ground = action == "reboot" or (action == "restart_service" and params.get("name") != "chrony")
        if needs_ground and self.telemetry["armed"]:
            self._reply(f"REFUSED {action}: drone is armed")
            return
        try:
            if action == "write_file":
                path = remote.write_file(params["path"], base64.b64decode(params["data"]))
                self._reply(f"file written: {path}")
                if params.get("restart"):
                    self._restart(params["restart"])
            elif action == "run_command":
                code, output = remote.run_command(params["command"])
                self._reply(f"$ {params['command']}\n[exit {code}] {output}")
            elif action == "restart_service":
                self._restart(params["name"])
            elif action == "reboot":
                self._reply("rebooting")
                remote.reboot()
            elif action == "load_fcu_params":
                path = remote.write_file(params["path"], base64.b64decode(params["data"]))
                self._reply(f"FCU parameters stored in {path}, loading...")
                code, output = remote.load_fcu_params(path)
                self._reply(f"FCU parameters load [exit {code}] {output}")
        except Exception as e:
            self._reply(f"ERROR in {action}: {e!r}")

    def _restart(self, name):
        if name != "chrony" and self.telemetry["armed"]:
            self._reply(f"REFUSED restart of {name}: drone is armed")
            return
        if name == "swarm":
            self._reply("restarting swarm service")
            remote.restart_service_detached(name)
            return
        remote.restart_service(name)
        self._reply(f"service restarted: {name}")
        if name == "chrony" and self.server_ip:
            # chronyd forgets the runtime-added server on restart.
            threading.Event().wait(2.0)
            network.set_chrony_server(self.server_ip)

    def _run_check(self):
        try:
            self.checks_status = checks.self_check()
        except Exception as e:
            self.checks_status = {"ok": False, "problems": [f"self_check crashed: {e!r}"], "warnings": []}

    def _set_animation(self, csv_text):
        if self.telemetry["armed"]:
            logger.error("Refusing to replace the animation while armed")
            return
        path = os.path.abspath(self.commander.animation.filepath)
        tmp_path = path + ".tmp"
        try:
            with open(tmp_path, "w", encoding="utf-8", newline="") as f:
                f.write(csv_text)
            os.replace(tmp_path, path)
        except OSError as e:
            logger.error("Failed to store animation: %s", e)
            return
        logger.info("Animation received from server (%d bytes)", len(csv_text))
        self.commander.do_action("reload_animation")

    def _set_config(self, ini_text, mode="rewrite"):
        ini_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drone.ini")
        try:
            if mode == "modify":
                remote.merge_ini(ini_path, ini_text)
            else:
                with open(ini_path, "w", encoding="utf-8") as f:
                    f.write(ini_text)
            config.reset()
            config.load(ini_path)
            self._reply(f"config updated ({mode})")
            if self.telemetry["armed"]:
                logger.warning("Armed: animation will use the new config after the next reload")
            else:
                self.commander.animation.on_config_update(config)
        except OSError as e:
            self._reply(f"ERROR: config not applied: {e}")
        except configparser.Error as e:
            self._reply(f"ERROR: bad config: {e}")

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
                try:
                    self._update_pose()
                except Exception as e:
                    logger.warning("Pose update failed: %r", e)
                    self.telemetry.update(x=None, y=None, z=None, yaw=None)
                payload = dict(self.telemetry)
                payload["states"] = self._states()
                payload["animation_id"] = self.commander.animation.id
                payload["animation_state"] = self.commander.animation.state
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
            self._run_check()
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
