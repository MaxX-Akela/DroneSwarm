import rospy 
from clover.srv import SetLEDEffect

set_effect_ros = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service

def set_effect(*args, **kwargs):

    try:
        set_effect_ros(*args, **kwargs)
    except rospy.ServiceException:
        pass