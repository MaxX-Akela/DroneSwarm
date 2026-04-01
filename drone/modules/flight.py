import rospy
from clover import srv
from std_srvs.srv import Trigger
import logging
import math
import interrupter
from mavros_msgs.srv import CommandBool

arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

logger = logging.getLogger(__name__)

FRAME_ID = 'map'
LAND_TIMEOUT = 4
TAKEOFF_TIMEOUT = 5
ARM_TIMEOUT = 10
Z_TAKEOFF = 1
TOLERANCE = 0.2
SPEED = 1
FLIGHT_TIMEOUT = 20

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=SPEED, frame_id=FRAME_ID, auto_arm=False, tolerance=TOLERANCE):

    try:
        res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
        if not res.success:
            return False
    except rospy.ServiceException:
        return False
    
    start_time = rospy.get_time()

    while rospy.is_shutdown():
        if interrupter and interrupter.is_set():
            return False

        telem = get_telemetry(frame_id='navigate_target')
        dist = math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2)

        if dist < tolerance:
            return True 

        if FLIGHT_TIMEOUT and (rospy.get_time() - start_time > FLIGHT_TIMEOUT):
            return False

        rospy.sleep(0.2)
        

def stop(frame_id='body', speed=SPEED):
    navigate_wait(frame_id=frame_id, speed=speed, yaw=math.nan())

def land_wait():
    try:
        land()
    except:
        return False
    
    start_time = rospy.get_time()

    while rospy.is_shutdown():

        t = get_telemetry()

        if t.armed:
            return True
        
        if rospy.get_time() - start_time > ARM_TIMEOUT:

            if t.z <= 0.3:
                arming(False)
                return True
            else:
                return False
            
        rospy.sleep(0.2)


def takeoff(z=Z_TAKEOFF, frame_id='body', timeout=TAKEOFF_TIMEOUT, yaw=math.nan, emergency_land=False, tolerance=0.2):
    
    start_time = rospy.get_time()

    try:
        res = navigate(z=z, yaw=yaw, frame_id=frame_id, auto_arm=True)
        if not res.success:
            return False
    except rospy.ServiceException:
        return False
    
    while not rospy.is_shutdown():
        if interrupter and interrupter.is_set():
            return False

        telem = get_telemetry()
        
        if abs(telem.z) < tolerance:
            return True

        if timeout and (rospy.get_time() - start_time > timeout):
            if emergency_land:
                land()
            return False

        rospy.sleep(0.1)


