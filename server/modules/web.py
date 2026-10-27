import json
import logging
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from modules.config import config

logger = logging.getLogger(__name__)

WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "web")
CONTENT_TYPES = {".html": "text/html; charset=utf-8", ".js": "application/javascript"}


def parse_aruco_map(text):
    """Clover map file: `id size x y z z_angle y_angle x_angle` per line, '#' comments."""
    markers = []
    for line in text.splitlines():
        fields = line.split("#", 1)[0].split()
        if len(fields) < 2:
            continue
        try:
            values = [float(v) for v in fields[2:8]]
            marker = {"id": int(fields[0]), "size": float(fields[1])}
        except ValueError:
            continue
        values += [0.0] * (6 - len(values))
        marker.update(zip(("x", "y", "z", "rz", "ry", "rx"), values))
        markers.append(marker)
    return markers


def _finite(v):
    return isinstance(v, (int, float)) and v == v and abs(v) != float("inf")


class SwarmWebServer:
    """Serves the swarm 3D visualization page and its JSON state."""

    def __init__(self, is_connected):
        self._is_connected = is_connected
        self._lock = threading.Lock()
        self._telemetry = {}
        self._map_path = config.web_aruco_map
        self._httpd = None

    def update(self, copter_id, telemetry):
        with self._lock:
            self._telemetry[copter_id] = telemetry

    def set_aruco_map(self, path):
        self._map_path = path

    def _map(self):
        if not self._map_path:
            return []
        try:
            with open(self._map_path, "r", encoding="utf-8") as f:
                return parse_aruco_map(f.read())
        except (OSError, UnicodeDecodeError) as e:
            logger.warning("Can't read aruco map %s: %s", self._map_path, e)
            return []

    def snapshot(self):
        """Every known drone: seen-by-map drones at their real pose, others at the animation start."""
        now = time.time()
        drones = []
        with self._lock:
            items = list(self._telemetry.items())
        for copter_id, t in sorted(items):
            online = self._is_connected(copter_id) and now - t.get("_last_seen", 0) <= config.network_drone_timeout
            pose = [t.get(k) for k in ("x", "y", "z")]
            start = t.get("start_pos") or []
            if all(_finite(v) for v in pose):
                source, pos = "aruco_map", pose
            elif len(start) == 3:
                source, pos = "animation", list(start)
            else:
                source, pos = "none", None
            drones.append({
                "id": copter_id,
                "online": online,
                "armed": bool(t.get("armed")),
                "mode": t.get("mode"),
                "bat": t.get("bat"),
                "source": source,
                "pos": pos,
                "yaw": t.get("yaw") if source == "aruco_map" and _finite(t.get("yaw")) else 0.0,
            })
        return {"time": now, "drones": drones}

    def start(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                logger.debug("web: " + fmt, *args)

            def _send(self, code, body, ctype):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                # The page may be opened from a drone's own web (different origin).
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path == "/api/swarm":
                    return self._send(200, json.dumps(server.snapshot()).encode(), "application/json")
                if path == "/api/map":
                    return self._send(200, json.dumps(server._map()).encode(), "application/json")
                rel = "swarm.html" if path == "/" else path.lstrip("/")
                full = os.path.normpath(os.path.join(WEB_DIR, rel))
                ext = os.path.splitext(full)[1]
                if not full.startswith(WEB_DIR + os.sep) or ext not in CONTENT_TYPES or not os.path.isfile(full):
                    return self._send(404, b"not found", "text/plain")
                with open(full, "rb") as f:
                    self._send(200, f.read(), CONTENT_TYPES[ext])

        try:
            self._httpd = ThreadingHTTPServer((config.network_bind_address, config.web_port), Handler)
        except OSError as e:
            logger.error("Can't start swarm web on port %s: %s", config.web_port, e)
            return False
        self._httpd.daemon_threads = True
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        return True

    def stop(self):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
