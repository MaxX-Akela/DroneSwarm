import base64
import math
import os
import re
import subprocess
import sys
import threading
import time

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QTableWidget, QTableWidgetItem,
                             QHeaderView, QTextEdit, QMessageBox, QLabel, QGroupBox,
                             QAbstractItemView, QDoubleSpinBox, QCheckBox, QFormLayout,
                             QPlainTextEdit, QDialog, QDialogButtonBox, QMenuBar,
                             QFileDialog, QInputDialog)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from modules.config import CONFIG_PATH, config
from modules.network import NetworkManager
from modules.version import get_version, git_pull

COLUMNS = [
    "copter ID", "version", "animation ID", "battery", "system", "sensors",
    "mode", "checks", "current x y z yaw frame_id", "start x y z", "dt",
]
COL_VERSION, COL_ANIMATION, COL_BATTERY, COL_SYSTEM, COL_SENSORS = 1, 2, 3, 4, 5
COL_MODE, COL_CHECKS, COL_POSITION, COL_START, COL_DT = 6, 7, 8, 9, 10

# Time offset limits (seconds) for the dt column.
DT_OK = 0.05
DT_WARN = 0.2
# Distance (m) between the drone and its animation start point before takeoff.
START_OK = 0.3
START_WARN = 1.0
AIRBORNE_Z = 0.3

STATE_ORDER = {"ok": 0, "warn": 1, "fail": 2}


def worst(states):
    return max(states, key=lambda s: STATE_ORDER[s])


def _num(v):
    return isinstance(v, (int, float)) and not math.isnan(v)

OK_COLOR = QColor("#ccffcc")
WARN_COLOR = QColor("#ffe566")
FAIL_COLOR = QColor("#ff9999")
STATE_COLORS = {"ok": OK_COLOR, "warn": WARN_COLOR, "fail": FAIL_COLOR}


class ConfigEditorDialog(QDialog):
    """Plain-text editor for an .ini file; Save writes the file back."""

    def __init__(self, parent=None, title="Edit config", initial_text=""):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(600, 500)
        layout = QVBoxLayout(self)
        self.editor = QPlainTextEdit(self)
        self.editor.setPlainText(initial_text)
        layout.addWidget(self.editor)
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel, self)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def text(self):
        return self.editor.toPlainText()


class DroneDashboard(QMainWindow):
    update_finished = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("DroneSwarm")
        self.resize(1400, 700)

        self.drones = {}
        self.row_of = {}
        self._last_dir = ""
        self.server_version = get_version("unknown")
        self.setWindowTitle(f"DroneSwarm  {self.server_version}")

        self.init_menu()
        self.init_ui()

        self.network = NetworkManager()
        self.network.telemetry_received.connect(self.on_telemetry)
        self.network.drone_disconnected.connect(self.on_drone_disconnected)
        self.network.log_message.connect(self.log)
        self.network.start()
        self.update_finished.connect(self.on_update_finished)

        self.stale_timer = QTimer(self)
        self.stale_timer.timeout.connect(self.check_stale)
        self.stale_timer.start(1000)

    def init_menu(self):
        menubar = self.menuBar()

        selected_menu = menubar.addMenu("Selected drones")
        send_menu = selected_menu.addMenu("Send")
        send_menu.addAction("Animations...", self.send_animations_folder)
        send_menu.addAction("Camera calibrations...", self.send_camera_calibrations)
        send_menu.addAction("Aruco map...", self.send_aruco_map)
        send_menu.addAction("Animation...", self.send_animation_file)
        send_menu.addAction("Configuration...", self.send_configuration)
        send_menu.addAction("Launch files folder...", self.send_launch_files)
        send_menu.addAction("FCU parameters file...", self.send_fcu_params)
        send_menu.addAction("File...", self.send_file)
        send_menu.addAction("Command...", self.send_command_dialog)
        restart_menu = selected_menu.addMenu("Restart service")
        for name in ("chrony", "ros", "swarm"):
            restart_menu.addAction(name, lambda checked=False, n=name: self.restart_service(n))
        selected_menu.addAction("Update (git pull + restart)", self.update_selected_drones)
        selected_menu.addAction("Reload animation", lambda: self.send_to_selected("reload_animation"))
        selected_menu.addAction("Reboot", self.reboot_selected)

        server_menu = menubar.addMenu("Server")
        server_menu.addAction("Edit server config", self.edit_server_config)
        server_menu.addAction("Edit any config", self.edit_any_config)
        server_menu.addAction("Update server git", self.update_server)
        server_menu.addAction("Restart server", self.restart_server)

        table_menu = menubar.addMenu("Table")
        table_menu.addAction("Select all", lambda: self.set_all_checked(True))
        table_menu.addAction("Deselect all", lambda: self.set_all_checked(False))

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
        self.developer_mode = QCheckBox("Developer mode (ignore self-check)")
        timing_form.addRow(self.developer_mode)
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
        # Drones are chosen with the checkbox in the first column, not by row selection.
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.cellDoubleClicked.connect(self.show_drone_details)

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
        # Telemetry (UDP) can outlive the command link (TCP); such a drone can't be commanded.
        telemetry["_offline"] = not self.network.is_connected(copter_id)
        self.drones[copter_id] = telemetry
        if copter_id not in self.row_of:
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.row_of[copter_id] = row
            item_id = QTableWidgetItem(copter_id)
            item_id.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsUserCheckable)
            item_id.setCheckState(Qt.Unchecked)
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

    def version_state(self, version):
        if not version or version == "unknown":
            return "warn"
        if version.rstrip("*") != self.server_version.rstrip("*") or version.endswith("*"):
            return "warn"
        return "ok"

    @staticmethod
    def animation_state(t):
        anim_id = t.get("animation_id")
        if t.get("animation_state") != "OK" or not anim_id:
            return "fail"
        return "warn" if anim_id == "No animation id" else "ok"

    @staticmethod
    def dt_state(offset):
        if not offset.get("synced"):
            return "fail"
        magnitude = abs(offset.get("offset_sec", 0.0))
        if magnitude < DT_OK:
            return "ok"
        return "warn" if magnitude < DT_WARN else "fail"

    @staticmethod
    def start_state(t):
        """Is the drone standing at its animation's start point? None while airborne."""
        pos = [t.get(k) for k in ("x", "y", "z")]
        if t.get("armed") and _num(pos[2]) and pos[2] > AIRBORNE_Z:
            return None
        start = t.get("start_pos")
        if not start:
            return "fail"
        if not all(_num(v) for v in pos):
            return "warn"
        dist = math.dist(pos[:2], start[:2])
        if dist <= START_OK:
            return "ok"
        return "warn" if dist <= START_WARN else "fail"

    def update_row(self, copter_id):
        row = self.row_of.get(copter_id)
        if row is None:
            return
        t = self.drones[copter_id]
        offline = t.get("_offline", False)

        reported = t.get("states", {})
        checks = t.get("checks", {})
        if checks.get("ok") is None:
            checks_state, checks_text = "warn", "?"
        elif not checks["ok"]:
            checks_state, checks_text = "fail", "FAIL ({})".format(len(checks.get("problems", [])))
        elif checks.get("warnings"):
            checks_state, checks_text = "warn", "WARN ({})".format(len(checks["warnings"]))
        else:
            checks_state, checks_text = "ok", "OK"
        checks_tip = "\n".join(checks.get("problems", []) + checks.get("warnings", []))

        offset = t.get("time_offset", {})
        pos_ok = all(_num(t.get(k)) for k in ("x", "y", "z"))
        if pos_ok:
            pos_text = "{:.2f} {:.2f} {:.2f} {} {}".format(
                t["x"], t["y"], t["z"], "-" if t.get("yaw") is None else "{:.2f}".format(t["yaw"]),
                t.get("frame_id", ""))
        else:
            pos_text = "no {}".format(t.get("frame_id", "aruco_map"))
        start = t.get("start_pos")

        def state_text(key):
            state = reported.get(key)
            return {"ok": "OK", "warn": "WARN", "fail": "FAIL"}.get(state, "?")

        # column -> (text, state, tooltip); a missing report from the drone counts as warn.
        cells = {
            COL_VERSION: (t.get("version", "?"), self.version_state(t.get("version")),
                          "server: " + self.server_version),
            COL_ANIMATION: (t.get("animation_id") or "-", self.animation_state(t),
                            "animation: " + str(t.get("animation_state"))),
            COL_BATTERY: ("{:.1f}V".format(t.get("bat") or 0.0), reported.get("battery", "warn"), ""),
            COL_SYSTEM: (state_text("system"), reported.get("system", "warn"),
                         "FCU connected" if t.get("connected") else "no FCU connection"),
            COL_SENSORS: (state_text("sensors"), reported.get("sensors", "warn"),
                          "vision pose / rangefinder"),
            COL_MODE: (t.get("mode", "?") + (" (armed)" if t.get("armed") else ""),
                       reported.get("mode", "warn"), ""),
            COL_CHECKS: (checks_text, checks_state, checks_tip),
            COL_POSITION: (pos_text, reported.get("position", "warn"), ""),
            COL_START: ("{:.2f} {:.2f} {:.2f}".format(*start) if start else "-",
                        self.start_state(t), ""),
            COL_DT: ("{:.3f} ({})".format(offset.get("offset_sec", 0.0), offset.get("source") or "none"),
                     self.dt_state(offset), ""),
        }

        overall = []
        for col, (text, state, tip) in cells.items():
            if offline:
                state = "fail"
                if col == COL_MODE:
                    text = "OFFLINE"
            item = QTableWidgetItem(str(text))
            item.setTextAlignment(Qt.AlignCenter)
            item.setToolTip(tip)
            if state is not None:
                item.setBackground(STATE_COLORS[state])
                item.setForeground(QColor("black"))
                overall.append(state)
            self.table.setItem(row, col, item)

        id_item = self.table.item(row, 0)
        id_item.setBackground(STATE_COLORS[worst(overall)] if overall else QColor("white"))
        id_item.setForeground(QColor("black"))

    def show_drone_details(self, row, column):
        copter_id = self.table.item(row, 0).text()
        t = self.drones.get(copter_id)
        if t is None:
            return
        dlg = QDialog(self)
        dlg.setWindowTitle(f"{copter_id}: config & errors")
        dlg.resize(700, 600)
        layout = QVBoxLayout(dlg)
        layout.addWidget(QLabel("Config"))
        cfg = t.get("config", {})
        table = QTableWidget(len(cfg), 2)
        table.setHorizontalHeaderLabels(["key", "value"])
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        for i, (k, v) in enumerate(sorted(cfg.items())):
            table.setItem(i, 0, QTableWidgetItem(k))
            table.setItem(i, 1, QTableWidgetItem(str(v)))
        layout.addWidget(table, 3)
        layout.addWidget(QLabel("Errors / warnings (latest 30)"))
        errors = QPlainTextEdit("\n".join(t.get("errors", []) + t.get("checks", {}).get("problems", [])))
        errors.setReadOnly(True)
        layout.addWidget(errors, 2)
        dlg.exec_()

    def set_all_checked(self, checked):
        state = Qt.Checked if checked else Qt.Unchecked
        for row in self.row_of.values():
            self.table.item(row, 0).setCheckState(state)

    def get_selected_drones(self):
        return [copter_id for copter_id, row in self.row_of.items()
                if self.table.item(row, 0).checkState() == Qt.Checked]

    def send_to_selected(self, action, params=None):
        selected = self.get_selected_drones()
        if not selected:
            QMessageBox.warning(self, "Внимание", "Отметьте галочкой хотя бы одного дрона в таблице!")
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
        self.network.broadcast_command(selected, "play", {"start_time": start_time,
                                                         "ignore_checks": self.developer_mode.isChecked()})
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

    # ----- sending files / commands to the checked drones -----

    def _require_selected(self):
        selected = self.get_selected_drones()
        if not selected:
            QMessageBox.warning(self, "Внимание", "Отметьте галочкой хотя бы одного дрона в таблице!")
        return selected

    def _pick_folder(self, title):
        folder = QFileDialog.getExistingDirectory(self, title, self._last_dir)
        if folder:
            self._last_dir = folder
        return folder

    def _pick_file(self, title, name_filter="All files (*)"):
        path, _ = QFileDialog.getOpenFileName(self, title, self._last_dir, name_filter)
        if path:
            self._last_dir = os.path.dirname(path)
        return path

    def _match_files(self, selected, folder, extensions):
        """copter id -> file in folder whose name contains that id (as a whole token)."""
        names = [n for n in sorted(os.listdir(folder)) if n.lower().endswith(extensions)]
        paths, missing = {}, []
        for copter_id in selected:
            token = re.compile(r"(?<![0-9a-z])" + re.escape(copter_id.lower()) + r"(?![0-9a-z])")
            match = next((n for n in names if token.search(n.lower())), None)
            if match:
                paths[copter_id] = os.path.join(folder, match)
            else:
                missing.append(copter_id)
        return paths, missing

    def _warn_missing(self, what, missing):
        if missing:
            self.log(f"WARNING: нет файла ({what}) для: " + ", ".join(missing))
            QMessageBox.warning(self, "Нет файлов", f"Не найден файл ({what}) с ID в имени для:\n"
                                + "\n".join(missing))

    def _read_bytes(self, path):
        try:
            with open(path, "rb") as f:
                return f.read()
        except OSError as e:
            self.log(f"ERROR: не удалось прочитать {path}: {e}")
            return None

    def _send_write_file(self, copter_id, path, dest, restart=None):
        data = self._read_bytes(path)
        if data is None:
            return
        params = {"path": dest, "data": base64.b64encode(data).decode("ascii")}
        if restart:
            params["restart"] = restart
        if self.network.send_command(copter_id, "write_file", params):
            self.log(f"> {os.path.basename(path)} -> {copter_id}:{dest}")

    def _send_animation_to(self, copter_id, path):
        data = self._read_bytes(path)
        if data is None:
            return
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError:
            return self.log(f"ERROR: {path} не в кодировке UTF-8")
        if self.network.send_command(copter_id, "set_animation", {"csv_text": text}):
            self.log(f"> Анимация {os.path.basename(path)} отправлена: {copter_id}")

    def send_animations_folder(self):
        selected = self._require_selected()
        folder = selected and self._pick_folder("Папка с анимациями (.csv / .txt)")
        if not folder:
            return
        paths, missing = self._match_files(selected, folder, (".csv", ".txt"))
        for copter_id, path in paths.items():
            self._send_animation_to(copter_id, path)
        self._warn_missing("анимация", missing)

    def send_animation_file(self):
        selected = self._require_selected()
        path = selected and self._pick_file("Файл анимации", "Animation (*.csv *.txt);;All files (*)")
        for copter_id in (selected if path else []):
            self._send_animation_to(copter_id, path)

    def send_camera_calibrations(self):
        selected = self._require_selected()
        folder = selected and self._pick_folder("Папка с калибровками камеры (.yaml)")
        if not folder:
            return
        paths, missing = self._match_files(selected, folder, (".yaml", ".yml"))
        for copter_id, path in paths.items():
            self._send_write_file(copter_id, path, config.paths_camera_calibration)
        self._warn_missing("калибровка", missing)

    def send_aruco_map(self):
        selected = self._require_selected()
        path = selected and self._pick_file("Карта ArUco", "Aruco map (*.txt);;All files (*)")
        for copter_id in (selected if path else []):
            # The map is read by clover at startup, so clover restarts after the upload.
            self._send_write_file(copter_id, path, config.paths_aruco_map, restart="ros")

    def send_configuration(self):
        selected = self._require_selected()
        path = selected and self._pick_file("Конфигурация (.ini)", "Config (*.ini);;All files (*)")
        if not path:
            return
        box = QMessageBox(QMessageBox.Question, "Конфигурация",
                          "Modify: обновить только указанные ключи.\nRewrite: заменить конфиг целиком.",
                          parent=self)
        modify = box.addButton("Modify", QMessageBox.AcceptRole)
        rewrite = box.addButton("Rewrite", QMessageBox.DestructiveRole)
        box.addButton(QMessageBox.Cancel)
        box.exec_()
        if box.clickedButton() not in (modify, rewrite):
            return
        mode = "modify" if box.clickedButton() is modify else "rewrite"
        try:
            with open(path, "r", encoding="utf-8") as f:
                text = f.read()
        except (OSError, UnicodeDecodeError) as e:
            return self.log(f"ERROR: не удалось прочитать {path}: {e}")
        self.network.broadcast_command(selected, "set_config", {"ini_text": text, "mode": mode})
        self.log(f"> Конфигурация ({mode}) отправлена: {', '.join(selected)}")

    def send_launch_files(self):
        selected = self._require_selected()
        folder = selected and self._pick_folder("Папка с .launch и .yaml файлами")
        if not folder:
            return
        names = [n for n in sorted(os.listdir(folder)) if n.lower().endswith((".launch", ".yaml", ".yml"))]
        if not names:
            return self.log("WARNING: в папке нет .launch / .yaml файлов")
        for copter_id in selected:
            for name in names:
                self._send_write_file(copter_id, os.path.join(folder, name),
                                      config.paths_launch_dir.rstrip("/") + "/" + name)

    def send_fcu_params(self):
        selected = self._require_selected()
        path = selected and self._pick_file("Параметры FCU (.params)", "PX4 params (*.params);;All files (*)")
        if not path:
            return
        data = self._read_bytes(path)
        if data is None:
            return
        params = {"path": "/tmp/fcu.params", "data": base64.b64encode(data).decode("ascii")}
        self.network.broadcast_command(selected, "load_fcu_params", params)
        self.log(f"> Параметры FCU {os.path.basename(path)} отправлены, загрузка: {', '.join(selected)}")

    def send_file(self):
        selected = self._require_selected()
        path = selected and self._pick_file("Файл для отправки")
        if not path:
            return
        dest, ok = QInputDialog.getText(self, "Путь на дроне", "Куда положить файл на дроне:",
                                        text="/home/pi/" + os.path.basename(path))
        if ok and dest.strip():
            for copter_id in selected:
                self._send_write_file(copter_id, path, dest.strip())

    def send_command_dialog(self):
        selected = self._require_selected()
        if not selected:
            return
        command, ok = QInputDialog.getText(self, "Команда", "Команда (shell) для выполнения на дронах:")
        if ok and command.strip():
            self.network.broadcast_command(selected, "run_command", {"command": command.strip()})
            self.log(f"> $ {command.strip()}  -> {', '.join(selected)}")

    def restart_service(self, name):
        selected = self._require_selected()
        if not selected:
            return
        reply = QMessageBox.question(self, "Перезапуск службы",
                                     f"Перезапустить службу '{name}' на {len(selected)} дронах?",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.network.broadcast_command(selected, "restart_service", {"name": name})
            self.log(f"> restart {name}: {', '.join(selected)}")

    # Uses run_command only, so drones running an older client can be updated too.
    # Git checkout (image) -> git pull; apt install (/opt/droneswarm) -> apt upgrade of drone-swarm.
    UPDATE_COMMAND = (
        "if [ -d /home/pi/DroneSwarm/.git ]; then "
        "cd /home/pi/DroneSwarm && git -c safe.directory='*' pull --ff-only; "
        "else sudo -n apt-get update && "
        "sudo -n env DRONESWARM_NO_RESTART=1 apt-get install -y --only-upgrade drone-swarm; fi; rc=$?; "
        "if [ $rc -eq 0 ]; then setsid sh -c 'sleep 3; sudo -n systemctl restart droneswarm' "
        ">/dev/null 2>&1 < /dev/null & fi; exit $rc"
    )

    def update_selected_drones(self):
        selected = self._require_selected()
        if not selected:
            return
        armed = [c for c in selected if self.drones.get(c, {}).get("armed")]
        targets = [c for c in selected if c not in armed]
        if armed:
            self.log(f"Update пропущен (armed): {', '.join(armed)}")
        if not targets:
            return
        reply = QMessageBox.question(self, "Обновление",
                                     f"Обновление (git pull / apt) и перезапуск клиента на {len(targets)} дронах?",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.network.broadcast_command(targets, "run_command", {"command": self.UPDATE_COMMAND})
            self.log(f"> update: {', '.join(targets)}")

    def reboot_selected(self):
        selected = self._require_selected()
        if not selected:
            return
        reply = QMessageBox.question(self, "Перезагрузка",
                                     f"Перезагрузить ОС на {len(selected)} дронах? Это займёт 30-60 секунд.",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.network.broadcast_command(selected, "reboot")
            self.log(f"> reboot: {', '.join(selected)}")

    # ----- server menu -----

    def _edit_ini(self, path, title, default_text=""):
        try:
            with open(path, "r", encoding="utf-8") as f:
                text = f.read()
        except FileNotFoundError:
            text = default_text
        except (OSError, UnicodeDecodeError) as e:
            return QMessageBox.critical(self, "Ошибка", f"Не удалось открыть {path}:\n{e}")
        dialog = ConfigEditorDialog(self, title, text)
        if dialog.exec_() != QDialog.Accepted:
            return
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                f.write(dialog.text())
        except OSError as e:
            return QMessageBox.critical(self, "Ошибка", f"Не удалось сохранить {path}:\n{e}")
        self.log(f"Сохранён {path}")

    def edit_server_config(self):
        example = os.path.join(os.path.dirname(CONFIG_PATH), "server.example.ini")
        try:
            with open(example, "r", encoding="utf-8") as f:
                default_text = f.read()
        except OSError:
            default_text = ""
        self._edit_ini(CONFIG_PATH, "Server config", default_text)
        self.log("Конфиг сервера применится после перезапуска сервера")

    def edit_any_config(self):
        path = self._pick_file("Конфигурационный файл", "Config (*.ini);;All files (*)")
        if path:
            self._edit_ini(path, path)

    def update_server(self):
        reply = QMessageBox.question(
            self, "Обновление сервера",
            f"Текущая версия: {self.server_version}\n\nВыполнить git fetch и git pull --rebase?",
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        self.log("Обновление сервера: git fetch && git pull --rebase...")
        threading.Thread(target=lambda: self.update_finished.emit(*git_pull()), daemon=True).start()

    def on_update_finished(self, ok, output):
        self.log(("git: " if ok else "git FAILED: ") + output)
        if not ok:
            return QMessageBox.critical(self, "Обновление не удалось", output)
        old_version, self.server_version = self.server_version, get_version("unknown")
        self.setWindowTitle(f"DroneSwarm  {self.server_version}")
        for copter_id in self.drones:
            self.update_row(copter_id)
        if self.server_version.rstrip("*") == old_version.rstrip("*"):
            return QMessageBox.information(self, "Обновление сервера", "Уже последняя версия.\n\n" + output)
        reply = QMessageBox.question(
            self, "Обновление сервера",
            f"Обновлено: {old_version} -> {self.server_version}\n\nПерезапустить сервер сейчас?",
            QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self._restart_process()

    def restart_server(self):
        reply = QMessageBox.question(self, "Перезапуск сервера", "Перезапустить сервер?",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self._restart_process()

    def _restart_process(self):
        self.network.stop()
        subprocess.Popen([sys.executable] + sys.argv)
        QApplication.quit()

    def closeEvent(self, event):
        self.network.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = DroneDashboard()
    window.show()
    sys.exit(app.exec_())
