import os
import sys
import time

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QTableWidget, QTableWidgetItem,
                             QHeaderView, QTextEdit, QMessageBox, QLabel, QGroupBox,
                             QAbstractItemView, QDoubleSpinBox, QCheckBox, QFormLayout,
                             QPlainTextEdit, QDialog, QDialogButtonBox, QMenuBar)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from modules.config import config
from modules.network import NetworkManager

COLUMNS = [
    "copter ID", "version", "animation ID", "battery", "system", "sensors",
    "mode", "checks", "current x y z yaw frame_id", "start x y z", "dt",
]

OK_COLOR = QColor("#ccffcc")
WARN_COLOR = QColor("#ffe0a3")
FAIL_COLOR = QColor("#ff9999")


class ConfigEditorDialog(QDialog):
    def __init__(self, parent=None, initial_text=""):
        super().__init__(parent)
        self.setWindowTitle("Send config to selected drones")
        self.resize(500, 400)
        layout = QVBoxLayout(self)
        self.editor = QPlainTextEdit(self)
        self.editor.setPlainText(initial_text)
        layout.addWidget(self.editor)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def text(self):
        return self.editor.toPlainText()


class DroneDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DroneSwarm")
        self.resize(1400, 700)

        self.drones = {}   
        self.row_of = {}   
        self._last_config_text = self._load_default_config_text()

        self.init_menu()
        self.init_ui()

        self.network = NetworkManager()
        self.network.telemetry_received.connect(self.on_telemetry)
        self.network.drone_disconnected.connect(self.on_drone_disconnected)
        self.network.log_message.connect(self.log)
        self.network.start()

        self.stale_timer = QTimer(self)
        self.stale_timer.timeout.connect(self.check_stale)
        self.stale_timer.start(1000)

    @staticmethod
    def _load_default_config_text():
        example = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                                "drone", "config.example.ini")
        try:
            with open(example, "r", encoding="utf-8") as f:
                return f.read()
        except OSError:
            return ""

    def init_menu(self):
        menubar = self.menuBar()

        selected_menu = menubar.addMenu("Selected drones")
        selected_menu.addAction("Send config...", self.send_config_dialog)
        selected_menu.addAction("Reload animation", lambda: self.send_to_selected("reload_animation"))

        server_menu = menubar.addMenu("Server")
        server_menu.addAction("Restart networking", self.restart_networking)

        table_menu = menubar.addMenu("Table")
        table_menu.addAction("Select all", lambda: self.table.selectAll())
        table_menu.addAction("Deselect all", lambda: self.table.clearSelection())

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        sidebar_layout = QVBoxLayout()
        sidebar_layout.setContentsMargins(0, 0, 10, 0)

        control_group = QGroupBox("Команды управления")
        control_vbox = QVBoxLayout(control_group)

        timing_form = QFormLayout()
        self.start_after = QDoubleSpinBox()
        self.start_after.setSuffix(" s")
        self.start_after.setRange(0, 3600)
        timing_form.addRow("Start after", self.start_after)
        self.music_after = QDoubleSpinBox()
        self.music_after.setSuffix(" s")
        self.music_after.setRange(0, 3600)
        timing_form.addRow("Music after", self.music_after)
        self.play_music = QCheckBox("Play music")
        timing_form.addRow(self.play_music)
        control_vbox.addLayout(timing_form)

        btn_check = QPushButton("Проверка (Preflight check)")
        btn_takeoff = QPushButton("Взлет (Takeoff)")
        self.takeoff_z = QDoubleSpinBox()
        self.takeoff_z.setRange(0.1, 10)
        self.takeoff_z.setValue(1.5)
        self.takeoff_z.setSuffix(" m")
        btn_start_anim = QPushButton("Start animation")
        btn_pause = QPushButton("Pause")
        btn_land = QPushButton("Посадка (Land selected)")
        btn_land_all = QPushButton("Land ALL")
        btn_emergency = QPushButton("Emergency land")
        btn_visual_land = QPushButton("Визуальная посадка")
        btn_disarm_all = QPushButton("Disarm ALL")
        btn_disarm = QPushButton("Disarm selected")
        btn_test_leds = QPushButton("Test leds")
        btn_flip = QPushButton("Flip")
        btn_reboot = QPushButton("Reboot FCU")
        btn_calib_gyro = QPushButton("Calibrate gyro")
        btn_calib_level = QPushButton("Calibrate level")

        btn_takeoff.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold; padding: 10px;")
        btn_land.setStyleSheet("background-color: #f1c40f; color: black; font-weight: bold; padding: 10px;")
        btn_disarm.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; padding: 10px;")

        btn_check.clicked.connect(lambda: self.send_to_selected("check"))
        btn_start_anim.clicked.connect(self.start_animation_selected)
        btn_pause.clicked.connect(lambda: self.send_to_selected("stop"))
        btn_takeoff.clicked.connect(self.takeoff_selected)
        btn_land.clicked.connect(lambda: self.send_to_selected("land"))
        btn_land_all.clicked.connect(lambda: self.send_to_all("land"))
        btn_emergency.clicked.connect(self.emergency_land_all)
        btn_visual_land.clicked.connect(self.visual_land_stub)
        btn_disarm_all.clicked.connect(self.disarm_all)
        btn_disarm.clicked.connect(self.disarm_selected)
        btn_test_leds.clicked.connect(lambda: self.send_to_selected("test_leds"))
        btn_flip.clicked.connect(lambda: self.send_to_selected("flip"))
        btn_reboot.clicked.connect(lambda: self.send_to_selected("reboot_fcu"))
        btn_calib_gyro.clicked.connect(lambda: self.send_to_selected("calibrate_gyro"))
        btn_calib_level.clicked.connect(lambda: self.send_to_selected("calibrate_level"))

        for w in (btn_check, btn_start_anim, btn_pause, btn_land, btn_land_all,
                  btn_emergency, btn_visual_land):
            control_vbox.addWidget(w)
        control_vbox.addWidget(QLabel("Z:"))
        control_vbox.addWidget(self.takeoff_z)
        control_vbox.addWidget(btn_takeoff)
        control_vbox.addWidget(btn_flip)
        for w in (btn_disarm_all, btn_disarm, btn_test_leds, btn_reboot, btn_calib_gyro, btn_calib_level):
            control_vbox.addWidget(w)
        control_vbox.addStretch()

        sidebar_layout.addWidget(control_group)
        main_layout.addLayout(sidebar_layout, 1)

        right_layout = QVBoxLayout()

        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)

        log_group = QGroupBox("Консоль сервера")
        log_layout = QVBoxLayout(log_group)
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: monospace;")
        log_layout.addWidget(self.console)

        right_layout.addWidget(self.table, 3)
        right_layout.addWidget(log_group, 1)

        main_layout.addLayout(right_layout, 4)

    def log(self, message):
        time_str = time.strftime("%H:%M:%S")
        self.console.append(f"[{time_str}] {message}")

    def on_telemetry(self, copter_id, telemetry):
        self.drones[copter_id] = telemetry
        if copter_id not in self.row_of:
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.row_of[copter_id] = row
            item_id = QTableWidgetItem(copter_id)
            item_id.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(row, 0, item_id)
        self.update_row(copter_id)

    def on_drone_disconnected(self, copter_id):
        if copter_id in self.drones:
            self.drones[copter_id]["_offline"] = True
            self.update_row(copter_id)

    def check_stale(self):
        now = time.time()
        timeout = config.network_drone_timeout
        for copter_id, telemetry in self.drones.items():
            if now - telemetry.get("_last_seen", now) > timeout:
                telemetry["_offline"] = True
                self.update_row(copter_id)

    def update_row(self, copter_id):
        row = self.row_of.get(copter_id)
        if row is None:
            return
        t = self.drones[copter_id]
        offline = t.get("_offline", False)

        system_ok = t.get("system", {}).get("ok")
        sensors_ok = t.get("sensors", {}).get("ok")
        checks = t.get("checks", {})
        checks_ok = checks.get("ok")

        values = {
            1: t.get("version", "?"),
            2: t.get("animation_id") or "-",
            3: f"{t.get('bat', 0.0):.1f}V",
            4: "OK" if system_ok else ("FAIL" if system_ok is not None else "?"),
            5: "OK" if sensors_ok else ("FAIL" if sensors_ok is not None else "?"),
            6: t.get("mode", "?"),
            7: "OK" if checks_ok else ("FAIL" if checks_ok is not None else "?"),
            8: "{:.2f} {:.2f} {:.2f} {:.2f} {}".format(
                t.get("x", 0.0), t.get("y", 0.0), t.get("z", 0.0), t.get("yaw", 0.0), t.get("frame_id", "")),
            9: "{:.2f} {:.2f} {:.2f}".format(*(t.get("start_pos") or [0.0, 0.0, 0.0])) if t.get("start_pos") else "",
            10: "{:.3f}".format(t.get("time_offset", {}).get("offset_sec", 0.0)),
        }

        if offline:
            color = FAIL_COLOR
        elif system_ok and sensors_ok and checks_ok:
            color = OK_COLOR
        elif system_ok is None and sensors_ok is None and checks_ok is None:
            color = None
        else:
            color = WARN_COLOR

        for col, text in values.items():
            item = QTableWidgetItem("OFFLINE" if offline and col == 6 else str(text))
            item.setTextAlignment(Qt.AlignCenter)
            if color is not None:
                item.setBackground(color)
            self.table.setItem(row, col, item)

    def get_selected_drones(self):
        selected_rows = set(item.row() for item in self.table.selectedItems())
        ids = []
        for copter_id, row in self.row_of.items():
            if row in selected_rows:
                ids.append(copter_id)
        return ids

    def send_to_selected(self, action, params=None):
        selected = self.get_selected_drones()
        if not selected:
            QMessageBox.warning(self, "Внимание", "Выберите хотя бы одного дрона в таблице!")
            return
        self.network.broadcast_command(selected, action, params)
        self.log(f"> Команда [{action}] отправлена: {', '.join(selected)}")

    def send_to_all(self, action, params=None):
        ids = list(self.row_of.keys())
        if not ids:
            QMessageBox.warning(self, "Внимание", "Нет подключенных дронов!")
            return
        self.network.broadcast_command(ids, action, params)
        self.log(f"> Команда [{action}] отправлена всем ({len(ids)})")

    def takeoff_selected(self):
        selected = self.get_selected_drones()
        if not selected:
            return QMessageBox.warning(self, "Внимание", "Дроны не выбраны!")

        reply = QMessageBox.question(self, "Подтверждение",
                                     f"Внимание! Выбрано {len(selected)} дронов для ВЗЛЕТА.\nПродолжить?",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.network.broadcast_command(selected, "takeoff", {"height": self.takeoff_z.value()})

    def start_animation_selected(self):
        selected = self.get_selected_drones()
        if not selected:
            return QMessageBox.warning(self, "Внимание", "Дроны не выбраны!")
        if self.play_music.isChecked():
            self.log("Play music включен, но воспроизведение музыки не реализовано в этой версии сервера")
        start_time = time.time() + self.start_after.value()
        self.network.broadcast_command(selected, "play", {"start_time": start_time})
        self.log(f"> Старт анимации через {self.start_after.value():.1f}с для {', '.join(selected)}")

    def disarm_selected(self):
        selected = self.get_selected_drones()
        if not selected:
            return QMessageBox.warning(self, "Внимание", "Дроны не выбраны!")
        reply = QMessageBox.critical(self, "КРИТИЧЕСКАЯ ОПЕРАЦИЯ",
                                     "ОТКЛЮЧЕНИЕ МОТОРОВ В ПОЛЕТЕ ПРИВЕДЕТ К ПАДЕНИЮ!\nВы уверены?",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.network.broadcast_command(selected, "disarm")

    def disarm_all(self):
        ids = list(self.row_of.keys())
        if not ids:
            return QMessageBox.warning(self, "Внимание", "Нет подключенных дронов!")
        reply = QMessageBox.critical(self, "КРИТИЧЕСКАЯ ОПЕРАЦИЯ",
                                     "ОТКЛЮЧЕНИЕ МОТОРОВ У ВСЕХ ДРОНОВ В ПОЛЕТЕ ПРИВЕДЕТ К ПАДЕНИЮ!\nВы уверены?",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.network.broadcast_command(ids, "disarm")

    def emergency_land_all(self):
        ids = list(self.row_of.keys())
        if not ids:
            return QMessageBox.warning(self, "Внимание", "Нет подключенных дронов!")
        reply = QMessageBox.critical(self, "Аварийная посадка",
                                     f"Посадить ВСЕ дроны ({len(ids)}) немедленно?",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.network.broadcast_command(ids, "land")

    def visual_land_stub(self):
        self.log("Визуальная посадка не реализована в этой версии сервера")
        QMessageBox.information(self, "Визуальная посадка", "Эта функция пока не реализована.")

    def send_config_dialog(self):
        selected = self.get_selected_drones()
        if not selected:
            return QMessageBox.warning(self, "Внимание", "Дроны не выбраны!")
        dialog = ConfigEditorDialog(self, self._last_config_text)
        if dialog.exec_() == QDialog.Accepted:
            text = dialog.text()
            self._last_config_text = text
            self.network.broadcast_command(selected, "set_config", {"ini_text": text})
            self.log(f"Конфиг отправлен: {', '.join(selected)}")

    def restart_networking(self):
        self.network.stop()
        self.network = NetworkManager()
        self.network.telemetry_received.connect(self.on_telemetry)
        self.network.drone_disconnected.connect(self.on_drone_disconnected)
        self.network.log_message.connect(self.log)
        self.network.start()
        self.log("Сеть перезапущена")

    def closeEvent(self, event):
        self.network.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = DroneDashboard()
    window.show()
    sys.exit(app.exec_())
