import sys, json, socket, struct, selectors, threading
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QColor

class NetworkWorker(QThread):
    telemetry_signal = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.sel = selectors.DefaultSelector()
        self.running = True

    def run(self):
        lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        lsock.bind(('0.0.0.0', 8888))
        lsock.listen()
        lsock.setblocking(False)
        self.sel.register(lsock, selectors.EVENT_READ, data=self.accept)
        while self.running:
            for key, mask in self.sel.select(timeout=0.1):
                key.data(key.fileobj, mask)

    def accept(self, sock, mask):
        conn, addr = sock.accept()
        conn.setblocking(False)
        self.sel.register(conn, selectors.EVENT_READ, data=self.read)

    def read(self, conn, mask):
        try:
            header = conn.recv(4)
            if not header: raise Exception()
            msg_len = struct.unpack('!I', header)[0]
            data = json.loads(conn.recv(msg_len).decode('utf-8'))
            self.telemetry_signal.emit(data)
        except:
            self.sel.unregister(conn)
            conn.close()

class DroneSwarmServer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DroneSwarm Control")
        self.table = QTableWidget(0, 12)
        self.table.setHorizontalHeaderLabels(["ID", "Version", "Conf", "Anim", "Battery", "Sys", "Sensors", "Mode", "Checks", "Pos", "Start", "dt"])
        self.setCentralWidget(self.table)
        self.drones = {}

        self.net = NetworkWorker()
        self.net.telemetry_signal.connect(self.update_row)
        self.net.start()

    def update_row(self, data):
        d_id = data.get('copter_id')
        if d_id not in self.drones:
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.drones[d_id] = row
        
        row = self.drones[d_id]
        bat_ok = data['bat_p'] > 20
        self.set_item(row, 4, f"{data['bat_p']}%", QColor(0,255,0) if bat_ok else QColor(255,0,0))
        mode_ok = "CMODE" not in data['mode']
        self.set_item(row, 7, data['mode'], QColor(0,255,0) if mode_ok else QColor(255,0,0))

    def set_item(self, row, col, text, color):
        item = QTableWidgetItem(text)
        item.setBackground(color)
        self.table.setItem(row, col, item)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = DroneSwarmServer()
    ex.show()
    sys.exit(app.exec_())