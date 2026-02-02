import rospy 
from clover.srv import SetLEDEffect

set_effect_clover = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

def set_effect(*args, **kwargs):

    try:
        set_effect_clover(*args, **kwargs)
    except rospy.ServiceException:
        pass