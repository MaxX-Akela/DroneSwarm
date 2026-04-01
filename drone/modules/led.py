import rospy
import logging
from clover.srv import SetLEDEffect

logger = logging.getLogger(__name__)

ros_service = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

def set_effect(*args, **kwargs):
    try:
        ros_service(*args, **kwargs)
    except rospy.ServiceException:
        logger.error("Can't set led effect: service /led/set_effect is unavailable!")