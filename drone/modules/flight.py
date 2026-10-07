import logging
import math
import threading
import rospy
from clover import srv
from mavros_msgs.srv import CommandBool, CommandLong
from std_srvs.srv import Trigger

logger = logging.getLogger(__name__)

arming = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
navigate = rospy.ServiceProxy("navigate", srv.Navigate)
land_srv = rospy.ServiceProxy("land", Trigger)
command_long = rospy.ServiceProxy("mavros/cmd/command", CommandLong)


MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
MAV_CMD_PREFLIGHT_CALIBRATION = 241

# Every flight command is expressed in the ArUco map frame, never in "map"/"body"
# (except stop(), which only holds the current position).
FRAME_ID = "aruco_map"
LAND_TIMEOUT = 4
TAKEOFF_TIMEOUT = 5
ARM_TIMEOUT = 10
Z_TAKEOFF = 1
TOLERANCE = 0.2
SPEED = 1
FLIGHT_TIMEOUT = 20

_telemetry_lock = threading.Lock()


def get_telemetry_locked(frame_id=""):
    with _telemetry_lock:
        return get_telemetry(frame_id=frame_id)


def _interrupted(interrupter):
    return interrupter is not None and interrupter.is_set()


_last_navto_error = [0.0]


def get_pose():
    """Telemetry in the aruco_map frame, or None if the map isn't visible (NaN pose)."""
    try:
        t = get_telemetry_locked(frame_id=FRAME_ID)
    except rospy.ServiceException:
        return None
    if any(math.isnan(v) for v in (t.x, t.y, t.z)):
        return None
    return t


def navto(x=0, y=0, z=0, yaw=float("nan"), speed=SPEED,
          auto_arm=False, interrupter=None):
    """Non-blocking: publish the setpoint and return immediately.

    For animation frames, where playback timing is driven by the frame
    schedule itself, not by waiting for the copter to arrive.
    """
    try:
        res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=FRAME_ID, auto_arm=auto_arm)
    except rospy.ServiceException as e:
        res = None
        message = str(e)
    else:
        message = res.message
    if res is None or not res.success:
        now = rospy.get_time()
        if now - _last_navto_error[0] > 2.0:
            _last_navto_error[0] = now
            logger.warning("navigate in %s failed: %s", FRAME_ID, message)
        return False
    return True


def reach_point(x=0, y=0, z=0, yaw=float("nan"), speed=SPEED,
                 auto_arm=False, tolerance=TOLERANCE, timeout=FLIGHT_TIMEOUT, interrupter=None):
    """Blocking: publish the setpoint and wait until it's reached (or timeout/interrupt).

    For takeoff and standalone commands that must confirm arrival.
    """
    if not navto(x=x, y=y, z=z, yaw=yaw, speed=speed,
                 auto_arm=auto_arm, interrupter=interrupter):
        return False

    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        if _interrupted(interrupter):
            return False

        telem = get_telemetry_locked(frame_id="navigate_target")
        dist = math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2)

        if dist < tolerance:
            return True

        if timeout and (rospy.get_time() - start_time > timeout):
            return False

        rospy.sleep(0.2)

    return False


navto_wait = reach_point


def stop(speed=SPEED, interrupter=None):
    """Hold the current position: re-target the copter's own position in the aruco_map frame."""
    pose = get_pose()
    if pose is None:
        logger.warning("stop: aruco_map pose unavailable")
        return False
    return reach_point(x=pose.x, y=pose.y, z=pose.z, speed=speed, interrupter=interrupter)


def takeoff(height=Z_TAKEOFF, timeout_takeoff=TAKEOFF_TIMEOUT,
            yaw=float("nan"), emergency_land=False, tolerance=TOLERANCE, interrupter=None):
    start_time = rospy.get_time()

    # Takeoff is relative to the copter ("body"), so it climbs straight up from where it
    # stands; height is then confirmed in aruco_map. navigate() with x=y=0 in a non-body
    # frame would fly the copter to the map origin.
    try:
        res = navigate(z=height, yaw=yaw, frame_id="body", auto_arm=True)
        if not res.success:
            return False
    except rospy.ServiceException:
        return False

    while not rospy.is_shutdown():
        if _interrupted(interrupter):
            return False

        telem = get_telemetry_locked(frame_id=FRAME_ID)
        if math.isnan(telem.z):  # map not visible: fall back to the default (local) frame
            telem = get_telemetry_locked()

        if abs(telem.z - height) < tolerance:
            return True

        if timeout_takeoff and (rospy.get_time() - start_time > timeout_takeoff):
            if emergency_land:
                land_srv()
            return False

        rospy.sleep(0.1)

    return False


def land(z=0, descend=False, timeout_land=LAND_TIMEOUT, interrupter=None):
    if descend:
        reach_point(z=z, interrupter=interrupter)

    try:
        land_srv()
    except rospy.ServiceException:
        return False

    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        if _interrupted(interrupter):
            return False

        telem = get_telemetry_locked()

        if not telem.armed:
            return True

        if rospy.get_time() - start_time > max(timeout_land, ARM_TIMEOUT):
            if telem.z <= 0.3:
                arming(False)
                return True
            return False

        rospy.sleep(0.2)

    return False


def disarm(interrupter=None):
    try:
        arming(False)
        return True
    except rospy.ServiceException as e:
        logger.error("Disarm failed: %s", e)
        return False


def reboot_fcu(interrupter=None):
    try:
        command_long(command=MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1=1)
        return True
    except rospy.ServiceException as e:
        logger.error("Reboot FCU failed: %s", e)
        return False


def calibrate_gyro(interrupter=None):
    try:
        command_long(command=MAV_CMD_PREFLIGHT_CALIBRATION, param1=1)
        return True
    except rospy.ServiceException as e:
        logger.error("Gyro calibration failed: %s", e)
        return False


def calibrate_level(interrupter=None):
    try:
        command_long(command=MAV_CMD_PREFLIGHT_CALIBRATION, param5=2)
        return True
    except rospy.ServiceException as e:
        logger.error("Level calibration failed: %s", e)
        return False
