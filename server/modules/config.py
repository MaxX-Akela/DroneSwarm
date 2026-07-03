import configparser
import logging

logger = logging.getLogger(__name__)

DEFAULTS = {
    "network_discovery_port": 9000,
    "network_telemetry_port": 9001,
    "network_tcp_port": 9010,
    "network_bind_address": "0.0.0.0",
    "network_drone_timeout": 5.0,
    "server_log_dir": ".",
}


def _coerce(raw, default):
    if isinstance(default, bool):
        return raw.strip().lower() in ("1", "true", "yes", "on")
    if isinstance(default, float):
        return float(raw)
    if isinstance(default, int):
        return int(raw)
    return raw


class Config:
    def __init__(self, path=None):
        self._values = dict(DEFAULTS)
        if path:
            self.load(path)

    def load(self, path):
        parser = configparser.ConfigParser()
        if not parser.read(path, encoding="utf-8"):
            logger.warning("Config file %s not found, using defaults", path)
            return

        for section in parser.sections():
            for key, raw in parser.items(section):
                flat_key = f"{section.lower()}_{key.lower()}"
                if flat_key not in DEFAULTS:
                    logger.warning("Unknown config key [%s] %s, ignoring", section, key)
                    continue
                try:
                    self._values[flat_key] = _coerce(raw, DEFAULTS[flat_key])
                except (ValueError, SyntaxError) as e:
                    logger.warning("Bad value for %s = %r (%s), keeping default", flat_key, raw, e)

    def __getattr__(self, name):
        try:
            return self._values[name]
        except KeyError:
            raise AttributeError(name)


config = Config()
