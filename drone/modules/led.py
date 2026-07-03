import rospy
import time
import logging
from clover.srv import SetLEDEffect

logger = logging.getLogger(__name__)

ros_service = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

def set_effect(*args, **kwargs):
    try:
        ros_service(*args, **kwargs)
    except rospy.ServiceException:
        logger.error("Led Error")

def test(interrupter=None):
    for color in ((255, 0, 0), (0, 255, 0), (0, 0, 255)):
        if interrupter is not None and interrupter.is_set():
            return
        set_effect(r=color[0], g=color[1], b=color[2])
        time.sleep(0.5)
    set_effect(r=0, g=0, b=0)