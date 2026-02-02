import sys
import json
import socket
import struct
import selectors
import threading
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QTableWidget, QTableWidgetItem, QVBoxLayout, 
    QHBoxLayout, QWidget, QPushButton, QCheckBox, QHeaderView, QMenu, QAction,
    QColorDialog, QMessageBox, QFileDialog
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QColor

TCP_PORT = 8888
UDP_PORT = 9999
BROADCAST_INTERVAL = 2.0

class NetworkWorker(QThread):
    new_client_signal = pyqtSignal(str, str)  
    telemetry_signal = pyqtSignal(str, dict) 
    client_disconnected_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.sel = selectors.DefaultSelector()
        self.clients = {} 
        self.running = True

    def get_local_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
        except: ip = '127.0.0.1'
        finally: s.close()
        return ip

    def run(self):
        threading.Thread(target=self.udp_beacon, daemon=True).start()

        lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        lsock.bind(('0.0.0.0', TCP_PORT))
        lsock.listen()
        lsock.setblocking(False)
        self.sel.register(lsock, selectors.EVENT_READ, data=self.accept_wrapper)

        while self.running:
            events = self.sel.select(timeout=0.1)
            for key, mask in events:
                callback = key.data
                callback(key.fileobj, mask)

    def udp_beacon(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        msg = json.dumps({"ip": self.get_local_ip(), "port": TCP_PORT}).encode('utf-8')
        while self.running:
            try:
                sock.sendto(msg, ('<broadcast>', UDP_PORT))
                self.msleep(int(BROADCAST_INTERVAL * 1000))
            except: pass

    def accept_wrapper(self, sock, mask):
        conn, addr = sock.accept()
        conn.setblocking(False)
        drone_id = f"Drone_{addr[1]}"
        self.clients[conn] = drone_id
        self.sel.register(conn, selectors.EVENT_READ, data=self.read_handler)
        self.new_client_signal.emit(addr[0], drone_id)

    def read_handler(self, conn, mask):
        try:
            header = conn.recv(4)
            if not header: raise Exception("Closed")
            msg_len = struct.unpack('!I', header)[0]
            data = conn.recv(msg_len)
            msg = json.loads(data.decode('utf-8'))
            self.telemetry_signal.emit(self.clients[conn], msg)
        except:
            self.client_disconnected_signal.emit(self.clients[conn])
            self.sel.unregister(conn)
            conn.close()
            del self.clients[conn]

    def send_to_client(self, drone_id, data):
        for sock, d_id in self.clients.items():
            if d_id == drone_id:
                body = json.dumps(data).encode('utf-8')
                header = struct.pack('!I', len(body))
                try: sock.sendall(header + body)
                except: pass

class DroneSwarmServer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DroneSwarm Control Center")
        self.resize(1200, 600)
        
        self.drones = {}
        
        self.init_ui()
        self.init_network()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)

        self.table = QTableWidget()
        self.headers = [
            "ID", "Version", "Config", "Animation ID", "Battery", 
            "System", "Sensors", "Mode", "Checks", "Position", "Start Pos", "dt"
        ]
        self.table.setColumnCount(len(self.headers))
        self.table.setHorizontalHeaderLabels(self.headers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.table, 4)

        btn_layout = QVBoxLayout()
        
        self.btn_takeoff = QPushButton("Takeoff")
        self.btn_land = QPushButton("Land")
        self.btn_color = QPushButton("Set LED Color")
        self.btn_reboot = QPushButton("Reboot FCU")
        self.btn_disarm = QPushButton("Disarm")

        for b in [self.btn_takeoff, self.btn_land, self.btn_color, self.btn_reboot, self.btn_disarm]:
            btn_layout.addWidget(b)
            b.setMinimumHeight(40)

        btn_layout.addStretch()
        layout.addLayout(btn_layout, 1)

        self.btn_takeoff.clicked.connect(lambda: self.broadcast_command({"action": "takeoff"}))
        self.btn_land.clicked.connect(lambda: self.broadcast_command({"action": "land"}))
        self.btn_reboot.clicked.connect(lambda: self.broadcast_command({"action": "reboot_fcu"}))
        self.btn_color.clicked.connect(self.choose_color)

        
        self.create_menus()

    def create_menus(self):
        menubar = self.menuBar()
        
        selected_menu = menubar.addMenu("Selected drones")
        
        send_sub = selected_menu.addMenu("Send")
        send_sub.addAction("Animations")
        send_sub.addAction("Configuration")
        send_sub.addAction("File")
        
        restart_sub = selected_menu.addMenu("Restart Service")
        restart_sub.addAction("chrony")
        restart_sub.addAction("clever")
        restart_sub.addAction("drone_swarm")
        
        server_menu = menubar.addMenu("Server")
        server_menu.addAction("Edit server config")
        server_menu.addAction("Restart server")

    def init_network(self):
        self.network = NetworkWorker()
        self.network.new_client_signal.connect(self.add_drone)
        self.network.telemetry_signal.connect(self.update_telemetry)
        self.network.start()

    def add_drone(self, ip, drone_id):
        row = self.table.rowCount()
        self.table.insertRow(row)
        
        item_id = QTableWidgetItem(drone_id)
        item_id.setFlags(item_id.flags() | Qt.ItemIsUserCheckable | Qt.ItemIsEditable)
        item_id.setCheckState(Qt.Unchecked)
        self.table.setItem(row, 0, item_id)

        for i in range(1, len(self.headers)):
            item = QTableWidgetItem("N/A")
            item.setBackground(QColor(255, 255, 0)) # Yellow
            self.table.setItem(row, i, item)
            
        self.drones[drone_id] = {"row": row}

    def update_telemetry(self, drone_id, data):
        if drone_id not in self.drones: return
        row = self.drones[drone_id]["row"]
        
        self.update_cell(row, 4, f"{data.get('bat_v', 0)}V ({data.get('bat_p', 0)}%)", self.validate_battery(data))
        self.update_cell(row, 5, data.get('state', 'UNKNOWN'), data.get('state') == 'STANDBY')
        self.update_cell(row, 7, data.get('mode', 'NONE'), 'CMODE' not in data.get('mode', ''))
        self.update_cell(row, 8, "OK", True) 

    def update_cell(self, row, col, text, is_valid):
        item = self.table.item(row, col)
        if not item: return
        item.setText(text)
        color = QColor(0, 255, 0) if is_valid else QColor(255, 0, 0) # Green / Red
        item.setBackground(color)

    def validate_battery(self, data):
        return data.get('bat_p', 0) > 20 

    def choose_color(self):
        color = QColorDialog.getColor()
        if color.isValid():
            cmd = {
                "action": "set_color",
                "r": color.red(), "g": color.green(), "b": color.blue()
            }
            self.broadcast_command(cmd)

    def broadcast_command(self, cmd):
        for i in range(self.table.rowCount()):
            item = self.table.item(i, 0)
            if item.checkState() == Qt.Checked:
                drone_id = item.text()
                self.network.send_to_client(drone_id, cmd)

    def validate_drone_status(self, row, data):
        bat_ok = data.get('bat_p', 0) > 20 
        self.update_cell(row, 4, f"{data['bat_v']}V ({data['bat_p']}%)", bat_ok)
    
        sys_ok = data.get('system') == "STANDBY"
        self.update_cell(row, 5, data.get('system'), sys_ok)
    
        mode = data.get('state', 'UNKNOWN')
        mode_ok = mode != "NO_FCU" and "CMODE" not in mode
        self.update_cell(row, 7, mode, mode_ok)
    
        checks_ok = data.get('checks') == "OK"
        self.update_cell(row, 8, data.get('checks'), checks_ok)
    
        is_ready_to_fly = all([bat_ok, sys_ok, mode_ok, checks_ok])
        self.set_row_readiness(row, is_ready_to_fly)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DroneSwarmServer()
    window.show()
    sys.exit(app.exec_())