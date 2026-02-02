import rospy
import time
import logging
from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import ParamGet, ParamSet
from mavros_msgs.msg import State, ParamValue, Altitude
from std_msgs.msg import Float64
from pymavlink.dialects.v20 import common as mavlink

send_command_long = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
get_param = rospy.ServiceProxy('/mavros/param/get', ParamGet)
set_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
system_status = -1
heartbeat_sub = None
heartbeat_sub_status = None

def start_subscriber():
    global heartbeat_sub, heartbeat_sub_status
    heartbeat_sub = rospy.Subscriber('/mavros/state', State, state_callback)
    heartbeat_sub_status = True

def check_state_topic(wait_new_status = False):
    global system_status, heartbeat_sub, heartbeat_sub_status
    if (not heartbeat_sub) or (not heartbeat_sub_status):
        start_subscriber()
        system_status = -1
    if wait_new_status:
        system_status = -1
    start_time = time.time()
    while system_status == -1:
        if time.time() - start_time > 1.:
            rospy.loginfo("Not connected to fcu. Check connection.")
            return False
        rospy.sleep(0.1)
    return True

def stop_subscriber():
    global heartbeat_sub, heartbeat_sub_status
    if heartbeat_sub:
        heartbeat_sub.unregister()
        heartbeat_sub_status = False

def reboot_fcu():
    if check_state_topic():
        rospy.loginfo("Send reboot message to fcu")
        send_command_long(False, mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        stop_subscriber()
        return True
    return False


if __name__ == '__main__':
    rospy.init_node('mavros_wrapper')