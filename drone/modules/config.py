import ast
import configparser
import logging

logger = logging.getLogger(__name__)

DEFAULTS = {
    "animation_frame_delay": 0.1,
    "animation_yaw": "animation",
    "animation_common_offset": (0.0, 0.0, 0.0),
    "animation_private_offset": (0.0, 0.0, 0.0),
    "animation_ratio": (1.0, 1.0, 1.0),
    "animation_takeoff_level": 0.3,
    "animation_output_static_begin": False,
    "animation_output_takeoff": True,
    "animation_output_route": True,
    "animation_output_land": True,
    "animation_output_static_end": False,
    "animation_check_ground": True,
    "animation_ground_level": "current",
    "animation_start_action": "auto",

    "led_use": True,
    "led_takeoff_indication": True,
    "led_land_indication": True,

    "flight_frame_id": "map",
    "flight_takeoff_height": 1.5,
    "flight_takeoff_time": 5.0,
    "flight_land_timeout": 5.0,
    "flight_land_delay": 0.0,
    "flight_arming_time": 5.0,
    "flight_reach_first_point_time": 5.0,

    "checks_battery_min_voltage": 3.5,
    "checks_fcu_timeout": 3.0,
    "checks_service_timeout": 2.0,
}


def _coerce(raw, default):
    if isinstance(default, bool):
        return raw.strip().lower() in ("1", "true", "yes", "on")
    if isinstance(default, tuple):
        return tuple(float(v) for v in ast.literal_eval(raw))
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
