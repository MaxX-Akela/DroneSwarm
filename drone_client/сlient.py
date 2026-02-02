import socket
import json
import struct
import sys
import selectors
import threading
import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped

try:
    from modules.led import set_effect
    from modules.flight import takeoff, swarm_land, position
    from modules.mavros import reboot_fcu
    from modules.animation import load_animation
    from modules.checks import Checks 
except ImportError:
    print("Ошибка импорта модулей в drone_client/modules/")

class SwarmClient:
    def __init__(self):
        rospy.init_node('drone_client_node', anonymous=True)
        self.copter_id = rospy.get_param('~copter_id', socket.gethostname())
        self.udp_port = 9000
        self.sel = selectors.DefaultSelector()
        self.tcp_sock = None
        
        self.telemetry = {
            "copter_id": self.copter_id,
            "bat_p": 0, "bat_v": 0.0,
            "mode": "DISCONNECTED", "armed": False,
            "checks": "WAIT", "x": 0, "y": 0, "z": 0
        }

        rospy.Subscriber('mavros/state', State, self._state_cb)
        rospy.Subscriber('mavros/battery', BatteryState, self._bat_cb)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self._pose_cb)

    def _state_cb(self, msg):
        self.telemetry["mode"] = msg.mode
        self.telemetry["armed"] = msg.armed
    def _bat_cb(self, msg):
        self.telemetry["bat_p"] = int(msg.percentage * 100)
        self.telemetry["bat_v"] = round(msg.voltage, 2)
    def _pose_cb(self, msg):
        self.telemetry["x"] = round(msg.pose.position.x, 2)
        self.telemetry["y"] = round(msg.pose.position.y, 2)
        self.telemetry["z"] = round(msg.pose.position.z, 2)

    def find_server(self):
        print(f"[{self.copter_id}] Ожидание сервера на порту {self.udp_port}...")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            udp.bind(('', self.udp_port))
            while not rospy.is_shutdown():
                data, addr = udp.recvfrom(1024)
                msg = json.loads(data.decode('utf-8'))
                if 'ip' in msg:
                    print(f"Найден сервер: {msg['ip']}:{msg['port']}")
                    return msg['ip'], msg['port']

    def handle_command(self, sock, mask):
        try:
            header = sock.recv(4)
            if not header: raise ConnectionError
            msg_len = struct.unpack('!I', header)[0]
            data = sock.recv(msg_len)
            msg = json.loads(data.decode('utf-8'))
            
            action = msg.get('action')
            if action == 'takeoff': takeoff()
            elif action == 'land': swarm_land()
            elif action == 'set_color':
                set_effect(effect='fill', r=msg.get('r',0), g=msg.get('g',0), b=msg.get('b',0))
            
            resp = json.dumps({"status": "done", "action": action}).encode('utf-8')
            sock.sendall(struct.pack('!I', len(resp)) + resp)
        except Exception as e:
            print(f"Ошибка связи: {e}")
            self.sel.unregister(sock)
            sock.close()
            sys.exit(1)

    def telemetry_loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            try:
                self.telemetry["checks"] = "OK" if Checks.run_all() else "FAIL" 
                
                body = json.dumps(self.telemetry).encode('utf-8')
                self.tcp_sock.sendall(struct.pack('!I', len(body)) + body)
            except: break
            rate.sleep()

    def run(self):
        ip, port = self.find_server()
        self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_sock.connect((ip, port))
        self.tcp_sock.setblocking(False)
        
        self.sel.register(self.tcp_sock, selectors.EVENT_READ, data=self.handle_command)
        
        threading.Thread(target=self.telemetry_loop, daemon=True).start()
        
        while not rospy.is_shutdown():
            events = self.sel.select(timeout=0.1)
            for key, mask in events:
                callback = key.data
                callback(key.fileobj, mask)

if __name__ == '__main__':
    client = SwarmClient()
    client.run()