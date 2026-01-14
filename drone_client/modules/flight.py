import rospy
import math
import time
from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

SWARM_FRAME = "map"
START_Z = 1

def stop_swarm():
    navigate(frame_id="body", yaw=float('nan'), speed=0.5)

def set_pos(x, y, z, yaw=float('nan'), frame_id=SWARM_FRAME, auto_arm=False, **kwargs):

    navigate(x=x, y=y, z=z, yaw=yaw, auto_arm=auto_arm, frame_id=frame_id)

    print(f"x: {x:.2f} | y: {y:.2f} | z: {z:.2f} | yaw: {yaw:.2f}")

    return True

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2, timeout=10):
    
    try:
        nav = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
        if not nav.success:
            return False
    except rospy.ServiceException as e:
        return False

    start_time = rospy.get_time()
    rate = rospy.Rate(10)
    dist = math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')

        if dist < tolerance:
            return True
        
        if timeout is not None and (rospy.get_time() - start_time) >  timeout:
            return False

        rospy.sleep(0.2)

def takeoff(frame_id=SWARM_FRAME, z=START_Z):

    navigate(z=z, frame_id=frame_id, speed=0.5, yaw=math.nan)

    while True:
        telemetry = get_telemetry(frame_id=frame_id)
        if telemetry.z >= z - 0.05:  
            print("TakeOff complete")
            break
        time.sleep(0.1)

def swarm_land(landing=True, landing_z=0.5, timeout=10):

    if landing == True:
        navigate_wait(z=landing_z, frame_id="body")
    land()

    start_time = rospy.get_time()    
    while not rospy.is_shitdown():

        if get_telemetry().armed:
            return True
        
        if (rospy.get_time() - start_time) > timeout:
            arming(False)
            return False
