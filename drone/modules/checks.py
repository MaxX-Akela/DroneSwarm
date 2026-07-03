import logging
import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState

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


def check_battery(timeout=None, min_voltage=None):
    timeout = config.checks_fcu_timeout if timeout is None else timeout
    min_voltage = config.checks_battery_min_voltage if min_voltage is None else min_voltage
    try:
        battery = rospy.wait_for_message("mavros/battery", BatteryState, timeout=timeout)
    except rospy.ROSException:
        return "Battery: no mavros/battery message received"
    if battery.voltage <= 0:
        return "Battery: invalid voltage reading"
    if battery.voltage < min_voltage:
        return f"Battery: voltage {battery.voltage:.2f}V below minimum {min_voltage:.2f}V"
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

    for check in (check_fcu, check_battery):
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

    if problems:
        logger.warning("Self-check found problems: %s", problems)

    return {"ok": not problems, "problems": problems}
