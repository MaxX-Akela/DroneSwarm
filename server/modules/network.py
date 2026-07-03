import json
import logging
import socket
import struct
import threading
import time

from PyQt5.QtCore import QObject, pyqtSignal

from modules.config import config

logger = logging.getLogger(__name__)


class NetworkManager(QObject):
    telemetry_received = pyqtSignal(str, dict)
    drone_disconnected = pyqtSignal(str)
    log_message = pyqtSignal(str)

    def __init__(self, discovery_port=None, tcp_port=None, telemetry_port=None, bind_address=None):
        super().__init__()
        self.discovery_port = discovery_port or config.network_discovery_port
        self.tcp_port = tcp_port or config.network_tcp_port
        self.telemetry_port = telemetry_port or config.network_telemetry_port
        self.bind_address = bind_address or config.network_bind_address
        self.running = True

        self._connections_lock = threading.Lock()
        self.connections = {}

    def start(self):
        threading.Thread(target=self._discovery_loop, daemon=True).start()
        threading.Thread(target=self._tcp_accept_loop, daemon=True).start()
        threading.Thread(target=self._telemetry_loop, daemon=True).start()

    def stop(self):
        self.running = False
        with self._connections_lock:
            sockets = list(self.connections.values())
        for sock in sockets:
            try:
                sock.close()
            except OSError:
                pass

    def _discovery_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_address, self.discovery_port))
        sock.settimeout(1.0)
        self.log_message.emit(f"Discovery listener on {self.bind_address}:{self.discovery_port}")
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                msg = json.loads(data.decode("utf-8"))
                if msg.get("type") == "discover":
                    reply = json.dumps({"type": "discover_reply", "tcp_port": self.tcp_port}).encode("utf-8")
                    sock.sendto(reply, addr)
            except socket.timeout:
                continue
            except (OSError, ValueError) as e:
                logger.debug("Discovery error: %s", e)
        sock.close()

    def _tcp_accept_loop(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self.bind_address, self.tcp_port))
        server_sock.listen(16)
        server_sock.settimeout(1.0)
        self.log_message.emit(f"Command listener on {self.bind_address}:{self.tcp_port}")
        while self.running:
            try:
                client_sock, addr = server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._handle_drone_connection, args=(client_sock, addr), daemon=True).start()
        server_sock.close()

    def _handle_drone_connection(self, sock, addr):
        copter_id = None
        try:
            raw_len = self._recv_exact(sock, 4)
            if not raw_len:
                return
            msg_len = struct.unpack("!I", raw_len)[0]
            data = self._recv_exact(sock, msg_len)
            if not data:
                return
            hello = json.loads(data.decode("utf-8"))
            if hello.get("type") != "hello":
                logger.warning("First frame from %s wasn't a hello handshake", addr)
                return
            copter_id = hello["copter_id"]
            with self._connections_lock:
                self.connections[copter_id] = sock
            self.log_message.emit(f"{copter_id} connected from {addr[0]}")

            while self.running:
                chunk = sock.recv(1)
                if not chunk:
                    break
        except (OSError, ValueError, KeyError) as e:
            logger.debug("Connection from %s dropped: %s", addr, e)
        finally:
            if copter_id:
                with self._connections_lock:
                    if self.connections.get(copter_id) is sock:
                        del self.connections[copter_id]
                self.drone_disconnected.emit(copter_id)
                self.log_message.emit(f"{copter_id} disconnected")
            sock.close()

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

    def send_command(self, copter_id, action, params=None):
        with self._connections_lock:
            sock = self.connections.get(copter_id)
        if sock is None:
            logger.warning("No connection to %s, dropping '%s'", copter_id, action)
            return False
        data = json.dumps({"action": action, "params": params or {}}).encode("utf-8")
        try:
            sock.sendall(struct.pack("!I", len(data)) + data)
            return True
        except OSError as e:
            logger.warning("Failed to send '%s' to %s: %s", action, copter_id, e)
            return False

    def broadcast_command(self, copter_ids, action, params=None):
        for copter_id in copter_ids:
            self.send_command(copter_id, action, params)

    def _telemetry_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_address, self.telemetry_port))
        sock.settimeout(1.0)
        self.log_message.emit(f"Telemetry listener on {self.bind_address}:{self.telemetry_port}")
        while self.running:
            try:
                data, addr = sock.recvfrom(4096)
                telemetry = json.loads(data.decode("utf-8"))
                copter_id = telemetry.get("copter_id")
                if copter_id:
                    telemetry["_last_seen"] = time.time()
                    telemetry["_addr"] = addr[0]
                    self.telemetry_received.emit(copter_id, telemetry)
            except socket.timeout:
                continue
            except (OSError, ValueError) as e:
                logger.debug("Telemetry error: %s", e)
        sock.close()
