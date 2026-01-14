import math
import rospy
import time
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Trigger

navigate = rospy.ServiceProxy('/navigate', srv.Navigate)
set_position = rospy.ServiceProxy('/set_position', srv.SetPosition)
set_rates = rospy.ServiceProxy('/set_rates', srv.SetRates)
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
landing = rospy.ServiceProxy('/land', Trigger)
emergency_land = rospy.ServiceProxy('/emergency_land', Trigger)

try:
    import rospy
except ImportError:
    print("rospy is not instaled")
    exit()

try:
    from clover import srv
except ImportError:
        print("Clover is not instaled")
        exit()

try:
    import modules.led as led
except:
    print("")

try:
    import modules.flight as flight
except:
     print("")

try:
    import modules.checks as selfchek
except:
    print()