import json
import logging
import queue
import socket
import struct
import threading
import time

from PyQt5.QtCore import QObject, pyqtSignal

from modules.config import config

logger = logging.getLogger(__name__)

SEND_TIMEOUT = 5.0


class DroneConnection:
    """One drone's TCP socket plus its own sender thread, so a stuck drone
    can't block the GUI thread or commands to the other drones."""

    def __init__(self, copter_id, sock):
        self.copter_id = copter_id
        self.sock = sock
        self.outbox = queue.Queue()
        threading.Thread(target=self._send_loop, daemon=True).start()

    def _send_loop(self):
        while True:
            data = self.outbox.get()
            if data is None:
                return
            try:
                self.sock.sendall(struct.pack("!I", len(data)) + data)
            except OSError as e:
                # A partial frame would corrupt the stream: drop the connection
                # and let the drone rediscover/reconnect.
                logger.warning("Failed to send to %s: %s", self.copter_id, e)
                self.close()
                return

    def send(self, data):
        self.outbox.put(data)

    def close(self):
        self.outbox.put(None)
        try:
            self.sock.close()
        except OSError:
            pass


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
        self._threads = []

    def start(self):
        for target in (self._discovery_loop, self._tcp_accept_loop, self._telemetry_loop):
            thread = threading.Thread(target=target, daemon=True)
            thread.start()
            self._threads.append(thread)

    def stop(self):
        self.running = False
        with self._connections_lock:
            conns = list(self.connections.values())
            self.connections.clear()
        for conn in conns:
            conn.close()
        # Listener sockets must be released before a restart rebinds the same ports.
        for thread in self._threads:
            thread.join(timeout=2.0)

    def _bind(self, sock, port, what):
        try:
            sock.bind((self.bind_address, port))
            return True
        except OSError as e:
            self.log_message.emit(f"ERROR: can't bind {what} {self.bind_address}:{port}: {e}")
            sock.close()
            return False

    def _discovery_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if not self._bind(sock, self.discovery_port, "discovery"):
            return
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
        if not self._bind(server_sock, self.tcp_port, "command listener"):
            return
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
        conn = None
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            sock.settimeout(SEND_TIMEOUT)
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
            conn = DroneConnection(copter_id, sock)
            with self._connections_lock:
                old = self.connections.get(copter_id)
                self.connections[copter_id] = conn
            if old is not None:
                old.close()
            self.log_message.emit(f"{copter_id} connected from {addr[0]}")

            while self.running:
                # Drones only talk back with short {"type": "log"} replies.
                try:
                    raw_len = self._recv_exact(sock, 4)
                    if not raw_len:
                        break
                    data = self._recv_exact(sock, struct.unpack("!I", raw_len)[0])
                except socket.timeout:
                    continue
                if not data:
                    break
                msg = json.loads(data.decode("utf-8"))
                if msg.get("type") == "log":
                    self.log_message.emit(f"[{copter_id}] {msg.get('text', '')}")
        except (OSError, ValueError, KeyError) as e:
            logger.debug("Connection from %s dropped: %s", addr, e)
        finally:
            if conn is not None:
                with self._connections_lock:
                    current = self.connections.get(copter_id) is conn
                    if current:
                        del self.connections[copter_id]
                conn.close()
                # A drone that reconnected already replaced this connection: it isn't offline.
                if current and self.running:
                    self.drone_disconnected.emit(copter_id)
                    self.log_message.emit(f"{copter_id} disconnected")
            else:
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

    def is_connected(self, copter_id):
        with self._connections_lock:
            return copter_id in self.connections

    def send_command(self, copter_id, action, params=None):
        with self._connections_lock:
            conn = self.connections.get(copter_id)
        if conn is None:
            logger.warning("No connection to %s, dropping '%s'", copter_id, action)
            self.log_message.emit(f"WARNING: нет соединения с {copter_id}, команда [{action}] не отправлена")
            return False
        conn.send(json.dumps({"action": action, "params": params or {}}).encode("utf-8"))
        return True

    def broadcast_command(self, copter_ids, action, params=None):
        for copter_id in copter_ids:
            self.send_command(copter_id, action, params)

    def _telemetry_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if not self._bind(sock, self.telemetry_port, "telemetry"):
            return
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
