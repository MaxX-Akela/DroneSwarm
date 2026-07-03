import logging
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped

logger = logging.getLogger(__name__)

VISION_TIMEOUT = 2.0


class Watchdog:
    def __init__(self, vision_timeout=VISION_TIMEOUT):
        self.vision_timeout = vision_timeout
        self.emergency = False
        self.armed = False
        self.mode = ""
        self.laser_range = float("nan")
        self._last_vision_pose = 0.0

        self._set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)

        rospy.Subscriber("mavros/vision_pose/pose", PoseStamped, self._vision_pose_cb)
        rospy.Subscriber("mavros/state", State, self._state_cb)
        rospy.Subscriber("mavros/distance_sensor/rangefinder", Range, self._laser_cb)

    def _vision_pose_cb(self, msg):
        self._last_vision_pose = msg.header.stamp.to_sec()

    def _state_cb(self, msg):
        self.armed, self.mode = msg.armed, msg.mode

    def _laser_cb(self, msg):
        self.laser_range = msg.range

    def check(self, event=None):
        """Callback shape matches rospy.Timer: check(event)."""
        if not (self.armed and self.mode == "OFFBOARD"):
            return

        vision_dt = abs(rospy.get_time() - self._last_vision_pose)
        if vision_dt > self.vision_timeout:
            if not self.emergency:
                logger.warning("Failsafe triggered: vision pose stale for %.1fs", vision_dt)
            self.emergency = True
            try:
                self._set_mode(custom_mode="AUTO.LAND")
            except rospy.ServiceException as e:
                logger.error("Failsafe couldn't set AUTO.LAND: %s", e)

    def is_emergency(self):
        return self.emergency

    def sensors_ok(self):
        """Vision pose freshness, independent of arm state, so the server's
        'sensors' column can show a problem before takeoff too."""
        return abs(rospy.get_time() - self._last_vision_pose) <= self.vision_timeout
