import socket, json, struct, sys, selectors, threading, rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from modules.flight import takeoff, swarm_land, position
from modules.led import set_effect
from modules.animation import load_animation

class SwarmClient:
    def __init__(self):
        rospy.init_node('drone_client_node')
        self.copter_id = rospy.get_param('~copter_id', socket.gethostname())
        self.animation_frames = load_animation("/home/pi/droneswarm/animations/show.csv")
        self.current_frame = 0
        self.telemetry = {"copter_id": self.copter_id, "bat_p": 0, "mode": "DISCONNECTED"}
        
        rospy.Subscriber('mavros/state', State, self._state_cb)
        rospy.Subscriber('mavros/battery', BatteryState, self._bat_cb)
        
        self.sel = selectors.DefaultSelector()

    def _state_cb(self, msg): self.telemetry["mode"] = msg.mode
    def _bat_cb(self, msg): self.telemetry["bat_p"] = int(msg.percentage * 100)

    def handle_command(self, sock, mask):
        data = sock.recv(1024)
        if not data: sys.exit()
        msg = json.loads(data.decode('utf-8'))
        action = msg.get('action')

        if action == 'takeoff': takeoff()
        elif action == 'land': swarm_land()
        elif action == 'position':
            if self.current_frame < len(self.animation_frames):
                f = self.animation_frames[self.current_frame]
                position(f['x'], f['y'], f['z'], f['yaw'])
                set_effect(effect='fill', r=f['r'], g=f['g'], b=f['b'])
                self.current_frame += 1

    def run(self):
        pass