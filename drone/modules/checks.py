import logging
import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState

import modules.flight as flight
from modules.config import config

logger = logging.getLogger(__name__)

REQUIRED_SERVICES = ["navigate", "land", "get_telemetry", "led/set_effect"]


def check_fcu(timeout=None):
    timeout = config.checks_fcu_timeout if timeout is None else timeout
    try:
        state = rospy.wait_for_message("mavros/state", State, timeout=timeout)
    except rospy.ROSException:
        return "FCU: no mavros/state message received"
    if not state.connected:
        return "FCU: MAVROS reports no FCU connection"
    return None


def battery_state(voltage):
    """'ok' / 'warn' / 'fail' for a pack voltage, thresholds are per cell."""
    if voltage is None or voltage <= 0:
        return "fail"
    per_cell = voltage / max(1, config.checks_battery_cells)
    if per_cell < config.checks_battery_min_voltage:
        return "fail"
    if per_cell < config.checks_battery_warn_voltage:
        return "warn"
    return "ok"


def check_battery(timeout=None, min_voltage=None):
    timeout = config.checks_fcu_timeout if timeout is None else timeout
    min_voltage = config.checks_battery_min_voltage if min_voltage is None else min_voltage
    cells = max(1, config.checks_battery_cells)
    try:
        battery = rospy.wait_for_message("mavros/battery", BatteryState, timeout=timeout)
    except rospy.ROSException:
        return "Battery: no mavros/battery message received"
    if battery.voltage <= 0:
        return "Battery: invalid voltage reading"
    if battery.voltage / cells < min_voltage:
        return (f"Battery: {battery.voltage / cells:.2f}V per cell "
                f"({battery.voltage:.2f}V / {cells}S) below minimum {min_voltage:.2f}V")
    return None


def check_aruco_map():
    """The copter must see the ArUco map: all flight happens in that frame."""
    if flight.get_pose() is None:
        return f"Frame '{flight.FRAME_ID}': no pose (ArUco map not detected)"
    return None


def battery_warning():
    try:
        battery = rospy.wait_for_message("mavros/battery", BatteryState, timeout=config.checks_fcu_timeout)
    except rospy.ROSException:
        return None
    if battery_state(battery.voltage) == "warn":
        return f"Battery: {battery.voltage:.2f}V is getting low"
    return None


def check_services(services=None, timeout=None):
    services = REQUIRED_SERVICES if services is None else services
    timeout = config.checks_service_timeout if timeout is None else timeout
    problems = []
    for name in services:
        try:
            rospy.wait_for_service(name, timeout=timeout)
        except rospy.ROSException:
            problems.append(f"Service '{name}' is not available")
    return problems


def self_check():
    problems = []

    for check in (check_fcu, check_battery, check_aruco_map):
        try:
            problem = check()
        except Exception as e:
            problem = f"{check.__name__} raised {e!r}"
        if problem:
            problems.append(problem)

    try:
        problems += check_services()
    except Exception as e:
        problems.append(f"check_services raised {e!r}")

    warnings = []
    if not problems:
        try:
            warning = battery_warning()
        except Exception as e:
            warning = f"battery_warning raised {e!r}"
        if warning:
            warnings.append(warning)

    if problems:
        logger.warning("Self-check found problems: %s", problems)

    return {"ok": not problems, "problems": problems, "warnings": warnings}
