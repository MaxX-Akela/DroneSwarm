import rospy
import math
import os
import subprocess
import traceback
import threading
import sys
import tf.transformations as t 
from threading import Lock

from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState, Image, CameraInfo, NavSatFix, Imu, Range
from mavros_msgs.msg import State, OpticalFlowRad, Mavlink
from mavros_msgs.srv import ParamGet 
from geometry_msgs.msg import PoseStamped

rospy.init_node('selfcheck', anonymous=True)

param_get = rospy.ServiceProxy('mavros/param/get', ParamGet)

thread_local = threading.local()
reports_lock = Lock()

if sys.stdout.isatty():
    GREY = '\033[90m'
    GREEN = '\033[92m'
    RED = '\033[31m'
    END = '\033[0m'
else:
    GREY = GREEN = RED = END = ''

def check(name):
    def inner(fn):
        def wrapper(*args, **kwargs):
            start = rospy.get_time()
            thread_local.reports = []
            try:
                fn(*args, **kwargs)
            except Exception as e:
                traceback.print_exc()
                rospy.logerr('%s: exception occurred', name)
            with reports_lock:
                for report in thread_local.reports:
                    if 'failure' in report:
                        rospy.logerr('%s: %s', name, report['failure'])
                    elif 'info' in report:
                        rospy.loginfo(GREY + name + END + ': ' + report['info'])
                if not thread_local.reports:
                    rospy.loginfo(GREY + name + END + ': ' + GREEN + 'OK' + END)
        return wrapper
    return inner

def failure(text, *args):
    msg = text % args
    thread_local.reports += [{'failure': msg}]

def info(text, *args):
    msg = text % args
    thread_local.reports += [{'info': msg}]

def is_process_running(name, full=False):
    return True 

def is_on_the_floor():
    try:
        dist = rospy.wait_for_message('rangefinder/range', Range, timeout=0.1).range
        return dist < 0.1
    except:
        return False

class Checks:
    @staticmethod
    @check("ROS")
    def check_ros():
        if rospy.is_shutdown():
            failure('ROS is shutdown')
        else:
            info('ROS is working')

    @staticmethod
    @check("FCU")
    def check_fcu():
        try:
            state = rospy.wait_for_message('mavros/state', State, timeout=3)
            if not state.connected:
                failure('No connection to the FCU (check wiring)')
        except rospy.ROSException:
            failure('No MAVROS state message received')

    @staticmethod
    @check("SimpleOffboard")    
    def check_simpleoffboard():
        try:
            rospy.wait_for_service('navigate', timeout=1)
            rospy.wait_for_service('get_telemetry', timeout=1)
        except rospy.ROSException:
            failure('No simple_offboard services available')

# Пример запуска
if __name__ == '__main__':
    Checks.check_ros()
    Checks.check_fcu()
    Checks.check_simpleoffboard()