#!/usr/bin/env python3
"""
Integrated Visual Logger

Merges functionalities of sensor_visual_logger.py and eye_tracker_visual_logger.py.
Uses PyQt5. Start/Stop manages both devices simultaneously. Data is logged to 2 separate CSV files.
"""

import argparse
import configparser
import csv
import os
import queue
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np
import serial

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen, QPixmap
from PyQt5.QtCore import Qt, QRectF
import pyqtgraph as pg

# ---- Eye Tracker SDK Imports ----
from sdk_types import PY_7I_ENVIRONMENT, PY_7I_RESOLUTION
from sdk_wrapper import wrapper


# ===================== Constants =====================
# Sensor
PACKET_SIZE = 64
NUM_FLOATS = 14
SYNC_MARKER = bytes([0x55, 0xAA, 0x55, 0xAA])
SYNC_LEN = 4

DISPLAY_WINDOW_SECONDS = 10.0
PLOT_UPDATE_INTERVAL_MS = 33
METRIC_UPDATE_INTERVAL_MS = 250
Y_LIMIT_UPDATE_INTERVAL_MS = 500
SERIAL_STARTUP_SETTLE_SECONDS = 2.0
RATE_WINDOW_SECONDS = 3.0

SENSOR_COLUMN_NAMES = [
    'Red', 'IR', 'Green',
    'accX', 'accY', 'accZ',
    'gyrX', 'gyrY', 'gyrZ',
    'magX', 'magY', 'magZ',
    'temp', 'timestamp', 'pc_timestamp'
]

EYE_COLUMN_NAMES = [
    "pc_timestamp", "device_timestamp", "gaze_x", "gaze_y", "gaze_z",
    "left_pupil_x", "left_pupil_y", "right_pupil_x", "right_pupil_y",
    "left_pupil_diameter_mm", "right_pupil_diameter_mm",
    "left_openness", "right_openness", "left_blink", "right_blink"
]


# ===================== Theme =====================
THEME_DARK = {
    'bg': '#12161C', 'fg': '#EAF0F7', 'grid_alpha': 0.2, 'label_color': '#EAF0F7',
    'group_border': '#3A3F4B', 'group_title': '#EAF0F7', 'value_bg': '#1E2330',
    'value_fg': '#4ED1A6', 'btn_bg': '#2A3040', 'btn_hover': '#3A4050',
    'btn_text': '#EAF0F7', 'btn_disabled_bg': '#1A1F2A', 'btn_disabled_text': '#5A6070',
    'separator': '#3A3F4B',
}
THEME_LIGHT = {
    'bg': '#FFFFFF', 'fg': '#1A1A2E', 'grid_alpha': 0.3, 'label_color': '#1A1A2E',
    'group_border': '#D0D5DD', 'group_title': '#1A1A2E', 'value_bg': '#F5F7FA',
    'value_fg': '#0D7C66', 'btn_bg': '#E8ECF0', 'btn_hover': '#D0D5DD',
    'btn_text': '#1A1A2E', 'btn_disabled_bg': '#F0F2F5', 'btn_disabled_text': '#A0A8B4',
    'separator': '#D0D5DD',
}

def detect_dark_theme():
    palette = QtWidgets.QApplication.instance().palette()
    window_color = palette.color(QtGui.QPalette.Window)
    luminance = (0.299 * window_color.redF() + 0.587 * window_color.greenF() + 0.114 * window_color.blueF())
    return luminance < 0.5


# ===================== Utility Classes =====================
class SceneImageLabel(QtWidgets.QLabel):
    button_clicked_signal = QtCore.pyqtSignal(int, int)
    def __init__(self, parent=None):
        super().__init__(parent)
    def mousePressEvent(self, ev):
        if ev.buttons() == Qt.LeftButton:
            self.button_clicked_signal.emit(ev.x(), ev.y())
    def connect_customized_slot(self, slot_func):
        self.button_clicked_signal.connect(slot_func)


class PacketRateTracker:
    def __init__(self, window_seconds=RATE_WINDOW_SECONDS):
        self.window_seconds = window_seconds
        self.timestamps = deque()
        self.total_packets = 0

    def push(self, timestamp):
        self.timestamps.append(timestamp)
        self.total_packets += 1
        cutoff = timestamp - self.window_seconds
        while self.timestamps and self.timestamps[0] < cutoff:
            self.timestamps.popleft()

    def current_rate(self):
        if len(self.timestamps) < 2: return 0.0
        duration = self.timestamps[-1] - self.timestamps[0]
        if duration <= 0: return 0.0
        return (len(self.timestamps) - 1) / duration


class RingSeries:
    def __init__(self, sample_rate_hz, window_seconds, use_numpy=False):
        self.use_numpy = use_numpy
        self.sample_rate_hz = sample_rate_hz
        self.window_seconds = window_seconds
        self.size = max(128, int(round(sample_rate_hz * window_seconds)))
        if use_numpy:
            self.values = np.zeros(self.size, dtype=np.float64)
        else:
            self.values = [0.0] * self.size
        self.write_index = 0
        self.count = 0

    def append(self, batch):
        if not batch: return
        if self.use_numpy:
            values = np.asarray(batch, dtype=np.float64)
            if len(values) >= self.size: values = values[-self.size:]
            n = len(values)
            first = min(n, self.size - self.write_index)
            second = n - first
            self.values[self.write_index:self.write_index + first] = values[:first]
            if second > 0: self.values[:second] = values[first:]
            self.write_index = (self.write_index + n) % self.size
            self.count = min(self.size, self.count + n)
        else:
            if len(batch) >= self.size: batch = batch[-self.size:]
            for value in batch:
                self.values[self.write_index] = float(value)
                self.write_index = (self.write_index + 1) % self.size
                self.count = min(self.size, self.count + 1)

    def ordered(self):
        if self.count == 0:
            return np.array([], dtype=np.float64) if self.use_numpy else []
        if self.count < self.size:
            return self.values[:self.count].copy() if self.use_numpy else self.values[:self.count]
        if self.use_numpy:
            return np.concatenate((self.values[self.write_index:], self.values[:self.write_index]))
        else:
            return self.values[self.write_index:] + self.values[:self.write_index]

    def x_axis(self, count):
        if self.use_numpy:
            return np.linspace(-self.window_seconds, 0.0, count, endpoint=True, dtype=np.float64)
        else:
            if count <= 1: return [0.0]
            step = self.window_seconds / (count - 1)
            return [(-self.window_seconds + i * step) for i in range(count)]


class MultiSeriesBuffer:
    def __init__(self, sample_rate_hz, window_seconds):
        self.red = RingSeries(sample_rate_hz, window_seconds, True)
        self.ir = RingSeries(sample_rate_hz, window_seconds, True)
        self.green = RingSeries(sample_rate_hz, window_seconds, True)
        self.ax = RingSeries(sample_rate_hz, window_seconds, True)
        self.ay = RingSeries(sample_rate_hz, window_seconds, True)
        self.az = RingSeries(sample_rate_hz, window_seconds, True)
        self.gx = RingSeries(sample_rate_hz, window_seconds, True)
        self.gy = RingSeries(sample_rate_hz, window_seconds, True)
        self.gz = RingSeries(sample_rate_hz, window_seconds, True)
        self.mx = RingSeries(sample_rate_hz, window_seconds, True)
        self.my = RingSeries(sample_rate_hz, window_seconds, True)
        self.mz = RingSeries(sample_rate_hz, window_seconds, True)


# ===================== Sensor Component =====================
def sync_timestamps(ser, rounds=20):
    best_rtt = float("inf")
    best_offset = 0.0
    success_count = 0
    for _ in range(rounds):
        ser.reset_input_buffer()
        t1 = time.perf_counter()
        ser.write(b"t\n")
        ser.flush()
        response = ser.readline()
        t2 = time.perf_counter()
        if not response or not response.startswith(b"T"): continue
        try: arduino_ms = int(response[1:].strip())
        except ValueError: continue
        rtt = t2 - t1
        offset = (t1 + t2) / 2.0 - arduino_ms / 1000.0
        if rtt < best_rtt:
            best_rtt = rtt
            best_offset = offset
        success_count += 1
    return best_offset

class SerialPacketReader:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.packet_queue = queue.SimpleQueue()
        self._buffer = bytearray()
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        try:
            self.serial_port = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.05)
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            return True
        except Exception as e:
            print(f"Serial conn err: {e}")
            return False

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def send_command(self, command_text):
        if not self.serial_port or not self.serial_port.is_open: return
        with self._write_lock:
            self.serial_port.write(command_text.encode("ascii"))
            self.serial_port.flush()

    def _read_loop(self):
        while not self._stop_event.is_set():
            if not self.serial_port.is_open: break
            try:
                incoming = self.serial_port.read(self.serial_port.in_waiting or 1)
                if incoming:
                    self._buffer.extend(incoming)
                    self._consume_packets()
            except Exception:
                break

    def _consume_packets(self):
        while True:
            start_index = self._buffer.find(SYNC_MARKER)
            if start_index < 0:
                if self._buffer and self._buffer[-1] == SYNC_MARKER[0]:
                    self._buffer[:] = self._buffer[-1:]
                else: self._buffer.clear()
                return
            if start_index > 0: del self._buffer[:start_index]
            if len(self._buffer) < PACKET_SIZE: return
            payload = bytes(self._buffer[SYNC_LEN:PACKET_SIZE])
            del self._buffer[:PACKET_SIZE]
            try: values = struct.unpack('<14f', payload[:NUM_FLOATS * 4])
            except struct.error: continue

            if sum(1 for v in values if v != v) > 3: continue
            timestamp = values[13]
            if timestamp == timestamp and (timestamp < 0 or timestamp > 4.3e9): continue

            self.packet_queue.put({
                "timestamp": time.perf_counter(), "red": values[0], "ir": values[1], "green": values[2],
                "ax": values[3], "ay": values[4], "az": values[5], "gx": values[6], "gy": values[7],
                "gz": values[8], "mx": values[9], "my": values[10], "mz": values[11], "temp": values[12],
                "timestamp_ms": values[13],
            })


# ===================== CSV Writers =====================
class SensorCSVWriterThread:
    def __init__(self, output_file):
        self.output_file = output_file
        self.queue = queue.SimpleQueue()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self.packets_written = 0
    def start(self): self._thread.start()
    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)
    def push(self, values): self.queue.put(values)
    def _write_loop(self):
        with open(self.output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(SENSOR_COLUMN_NAMES)
            while not self._stop_event.is_set() or not self.queue.empty():
                try: values = self.queue.get(timeout=0.1)
                except queue.Empty: continue
                row = []
                for i, v in enumerate(values):
                    if i in (0, 1, 2, 13): row.append(str(int(v)))
                    else: row.append(f'{v:.6f}')
                writer.writerow(row)
                self.packets_written += 1

class EyeCSVWriterWorker(QtCore.QObject):
    def __init__(self, queue_obj, output_file):
        super().__init__()
        self.queue = queue_obj
        self.output_file = output_file
        self._stop = False
    def stop(self): self._stop = True
    def run(self):
        with open(self.output_file, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(EYE_COLUMN_NAMES)
            while not self._stop or not self.queue.empty():
                try: sample = self.queue.get(timeout=0.1)
                except queue.Empty: continue
                row = [
                    f"{sample['pc_timestamp']:.6f}", str(int(sample["device_timestamp"])), f"{sample['gaze_x']:.6f}", f"{sample['gaze_y']:.6f}", f"{sample['gaze_z']:.6f}",
                    f"{sample['left_pupil_x']:.6f}", f"{sample['left_pupil_y']:.6f}", f"{sample['right_pupil_x']:.6f}", f"{sample['right_pupil_y']:.6f}",
                    f"{sample['left_pupil_diameter_mm']:.6f}", f"{sample['right_pupil_diameter_mm']:.6f}", f"{sample['left_openness']:.6f}", f"{sample['right_openness']:.6f}",
                    str(int(sample["left_blink"])), str(int(sample["right_blink"])),
                ]
                writer.writerow(row)

class EyeCSVWriterThread:
    def __init__(self, output_file):
        self.output_file = output_file
        self.queue = queue.Queue()
        self._thread = QtCore.QThread()
        self._worker = EyeCSVWriterWorker(self.queue, self.output_file)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
    def start(self): self._thread.start()
    def stop(self):
        self._worker.stop()
        self._thread.quit()
        self._thread.wait(2000)
    def push(self, sample): self.queue.put(sample)


# ===================== UI Integrated Window =====================
class IntegratedMonitorWindow(QtWidgets.QMainWindow):
    # Eye Tracker SDK Signals
    set_sdk_running_signal = QtCore.pyqtSignal(bool)
    set_pupil_center_signal = QtCore.pyqtSignal(float, float, float, float)
    set_gaze_signal = QtCore.pyqtSignal(float, float)
    set_scene_image_signal = QtCore.pyqtSignal(QPixmap)
    set_left_eye_image_signal = QtCore.pyqtSignal(QPixmap)
    set_right_eye_image_signal = QtCore.pyqtSignal(QPixmap)
    set_calibration_finish_signal = QtCore.pyqtSignal(int, int, int)

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.theme = THEME_DARK if detect_dark_theme() else THEME_LIGHT

        # Time syncing
        self.ts_offset = 0.0

        # Data & Buffers
        self.sensor_rate_tracker = PacketRateTracker()
        self.eye_rate_tracker = PacketRateTracker()
        
        self.sensor_buffers = MultiSeriesBuffer(args.sensor_sample_rate, args.window_seconds)
        
        self.eye_gaze_x_series = RingSeries(args.eye_sample_rate, args.window_seconds, True)
        self.eye_gaze_y_series = RingSeries(args.eye_sample_rate, args.window_seconds, True)
        self.eye_left_pupil_series = RingSeries(args.eye_sample_rate, args.window_seconds, True)
        self.eye_right_pupil_series = RingSeries(args.eye_sample_rate, args.window_seconds, True)

        # Plot loop states
        self.last_plot_update_ms = 0.0
        self.last_metric_update_ms = 0.0
        self.last_range_update_ms = 0.0
        
        self.sensor_invalid_count = 0
        self.sensor_latest_temp = None

        # Writers & Reader components
        self.sensor_csv_writer = None
        self.eye_csv_writer = None
        self.sensor_reader = None

        # Eye tracker Component
        self.eye_sdk = wrapper()
        self.eye_sdk_config_path = os.path.join(args.sdk_root, "bin", "config")
        self.eye_sdk.load_library(self.eye_sdk_config_path)
        self.eye_sdk.set_ui_handle(self)
        
        self.scene_width, self.scene_height = 1280, 720
        self.eye_sdk_running = False
        self.cur_gaze_x, self.cur_gaze_y = 0, 0
        self.eye_packet_count = 0

        self.calibration_is_running = False
        self.current_points = 0
        self.finish_points = [[1, 1], [1, 1], [1, 1], [1, 1], [1, 1], [1, 1], [1, 1], [1, 1], [1, 1]]

        self.system_running = False

        self._build_ui()
        self._connect_signals()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_timer)

    def _build_ui(self):
        self.setWindowTitle("Integrated Monitor Platform (Eye Tracker + Sensor)")
        self.resize(1600, 1000)
        
        t = self.theme
        pg.setConfigOptions(antialias=False, useOpenGL=False, background=t['bg'], foreground=t['fg'])
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # Top Bar
        metrics_bar = QtWidgets.QHBoxLayout()
        metrics_bar.setSpacing(20)
        label_style = f"font-size: 13px; color: {t['label_color']};"
        
        # Sensor status
        self.lbl_sensor_rate = QtWidgets.QLabel("Sen Rate: -- Hz")
        self.lbl_sensor_packets = QtWidgets.QLabel("Sen Pkts: 0")
        self.lbl_sensor_temp = QtWidgets.QLabel("Sen Temp: --")
        # Eye status
        self.lbl_eye_rate = QtWidgets.QLabel("Eye Rate: -- Hz")
        self.lbl_eye_packets = QtWidgets.QLabel("Eye Pkts: 0")

        for lbl in (self.lbl_sensor_rate, self.lbl_sensor_packets, self.lbl_sensor_temp,
                    self.lbl_eye_rate, self.lbl_eye_packets):
            lbl.setStyleSheet(label_style)
            metrics_bar.addWidget(lbl)
            
        metrics_bar.addStretch()
        root.addLayout(metrics_bar)

        # Main Layout (Left: Sidebar, Right: TabWidget)
        main_layout = QtWidgets.QHBoxLayout()
        root.addLayout(main_layout, stretch=1)

        # ----- Sidebar -----
        sidebar = QtWidgets.QVBoxLayout()
        sidebar.setContentsMargins(0, 0, 0, 0)
        sidebar.setSpacing(10)
        main_layout.addLayout(sidebar)

        group_ss = f"QGroupBox {{ border: 1px solid {t['group_border']}; border-radius: 6px; margin-top: 10px; padding-top: 14px; font-weight: bold; color: {t['group_title']}; }} QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; }}"
        btn_ss = f"QPushButton {{ background: {t['btn_bg']}; color: {t['btn_text']}; border: 1px solid {t['group_border']}; border-radius: 5px; padding: 6px 12px; font-size: 13px; }} QPushButton:hover {{ background: {t['btn_hover']}; }} QPushButton:pressed {{ background: {t['group_border']}; }} QPushButton:disabled {{ background: {t['btn_disabled_bg']}; color: {t['btn_disabled_text']}; }}"
        combo_ss = f"QComboBox {{ background: {t['value_bg']}; color: {t['fg']}; border: 1px solid {t['group_border']}; border-radius: 4px; padding: 3px 8px; font-size: 12px; }} QComboBox::drop-down {{ border: none; }} QComboBox QAbstractItemView {{ background: {t['value_bg']}; color: {t['fg']}; selection-background-color: {t['btn_hover']}; }}"
        val_ss = f"QLabel {{ background: {t['value_bg']}; color: {t['value_fg']}; border: 1px solid {t['group_border']}; border-radius: 4px; padding: 2px 6px; font-family: 'Consolas', 'Monaco', monospace; font-size: 12px; min-width: 70px; }}"

        # Global Control
        self.btn_start = QtWidgets.QPushButton("▶ Start All")
        self.btn_start.setStyleSheet(btn_ss)
        self.btn_stop = QtWidgets.QPushButton("■ Stop All")
        self.btn_stop.setStyleSheet(btn_ss)
        self.btn_stop.setEnabled(False)
        sidebar.addWidget(self.btn_start)
        sidebar.addWidget(self.btn_stop)
        
        # Sensor Settings
        grp_sensor = QtWidgets.QGroupBox("Sensor Settings")
        grp_sensor.setStyleSheet(group_ss)
        lo_sensor = QtWidgets.QVBoxLayout(grp_sensor)
        self.combo_port = QtWidgets.QComboBox()
        self.combo_port.setStyleSheet(combo_ss)
        self.combo_port.addItems(["COM1", "COM2", "COM3", "COM4", "COM5", "COM6"])
        self.combo_port.setCurrentText(self.args.port)
        self.combo_baud = QtWidgets.QComboBox()
        self.combo_baud.setStyleSheet(combo_ss)
        self.combo_baud.addItems(["9600", "115200", "1000000"])
        self.combo_baud.setCurrentText(str(self.args.baud))
        lo_sensor.addWidget(QtWidgets.QLabel("Port:")); lo_sensor.addWidget(self.combo_port)
        lo_sensor.addWidget(QtWidgets.QLabel("Baudrate:")); lo_sensor.addWidget(self.combo_baud)
        sidebar.addWidget(grp_sensor)

        # Eye tracker Settings
        grp_eye = QtWidgets.QGroupBox("Eye Tracker Setup")
        grp_eye.setStyleSheet(group_ss)
        lo_eye = QtWidgets.QVBoxLayout(grp_eye)
        self.combo_env = QtWidgets.QComboBox()
        self.combo_env.setStyleSheet(combo_ss)
        self.combo_env.addItem("indoor", 301); self.combo_env.addItem("outdoor", 302); self.combo_env.addItem("darkness", 303)
        self.combo_res = QtWidgets.QComboBox()
        self.combo_res.setStyleSheet(combo_ss)
        self.combo_res.addItem("1280 * 960", 201); self.combo_res.addItem("1280 * 720", 202); self.combo_res.addItem("800 * 600", 203); self.combo_res.addItem("1920 * 1080", 204)
        self.combo_res.setCurrentIndex(1)
        lo_eye.addWidget(QtWidgets.QLabel("Env:")); lo_eye.addWidget(self.combo_env)
        lo_eye.addWidget(QtWidgets.QLabel("Res:")); lo_eye.addWidget(self.combo_res)
        sidebar.addWidget(grp_eye)

        # Calibration
        grp_cal = QtWidgets.QGroupBox("Eye Calibration")
        grp_cal.setStyleSheet(group_ss)
        lo_cal = QtWidgets.QVBoxLayout(grp_cal)
        self.combo_points = QtWidgets.QComboBox()
        self.combo_points.setStyleSheet(combo_ss)
        self.combo_points.addItems(["1", "3", "5", "9"])
        self.combo_points.setCurrentIndex(1)
        self.btn_cal_start = QtWidgets.QPushButton("Start Calibration")
        self.btn_cal_start.setStyleSheet(btn_ss)
        self.btn_cal_start.setEnabled(False)
        self.btn_cal_stop = QtWidgets.QPushButton("Stop Calibration")
        self.btn_cal_stop.setStyleSheet(btn_ss)
        lo_cal.addWidget(QtWidgets.QLabel("Points:")); lo_cal.addWidget(self.combo_points)
        lo_cal.addWidget(self.btn_cal_start); lo_cal.addWidget(self.btn_cal_stop)
        sidebar.addWidget(grp_cal)
        
        # Realtime Values
        grp_vals = QtWidgets.QGroupBox("Eye Tracker Live Data")
        grp_vals.setStyleSheet(group_ss)
        lo_vals = QtWidgets.QGridLayout(grp_vals)
        self.lbl_gaze_x, self.lbl_gaze_y = QtWidgets.QLabel(""), QtWidgets.QLabel("")
        self.lbl_pupil_x, self.lbl_pupil_y = QtWidgets.QLabel(""), QtWidgets.QLabel("")
        for l in (self.lbl_gaze_x, self.lbl_gaze_y, self.lbl_pupil_x, self.lbl_pupil_y): l.setStyleSheet(val_ss)
        lo_vals.addWidget(QtWidgets.QLabel("Gaze X"), 0, 0); lo_vals.addWidget(self.lbl_gaze_x, 0, 1)
        lo_vals.addWidget(QtWidgets.QLabel("Gaze Y"), 1, 0); lo_vals.addWidget(self.lbl_gaze_y, 1, 1)
        lo_vals.addWidget(QtWidgets.QLabel("Pupil L"), 2, 0); lo_vals.addWidget(self.lbl_pupil_x, 2, 1)
        lo_vals.addWidget(QtWidgets.QLabel("Pupil R"), 3, 0); lo_vals.addWidget(self.lbl_pupil_y, 3, 1)
        sidebar.addWidget(grp_vals)
        
        sidebar.addStretch()

        # ----- Right Content (One Unified View) -----
        content_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(content_layout, stretch=1)

        # -- Top Content: Eye Tracker Views (Largest Element) --
        scene_eye_row = QtWidgets.QHBoxLayout()
        
        self.labelSceneImage = SceneImageLabel()
        self.labelSceneImage.connect_customized_slot(self._on_scene_clicked)
        self.labelSceneImage.setStyleSheet(f"QLabel {{ background: {t['value_bg']}; border: 1px solid {t['group_border']}; border-radius: 6px; }}")
        self.labelSceneImage.setAlignment(Qt.AlignCenter)
        self.labelSceneImage.setMinimumSize(640, 360)
        scene_eye_row.addWidget(self.labelSceneImage, stretch=3)
        
        eyes_col = QtWidgets.QVBoxLayout()
        self.labelLeftEye = QtWidgets.QLabel()
        self.labelLeftEye.setFixedSize(200, 150)
        self.labelLeftEye.setStyleSheet(f"background:{t['value_bg']}; border: 1px solid {t['group_border']}; border-radius: 4px;")
        self.labelLeftEye.setAlignment(Qt.AlignCenter)
        self.labelRightEye = QtWidgets.QLabel()
        self.labelRightEye.setFixedSize(200, 150)
        self.labelRightEye.setStyleSheet(f"background:{t['value_bg']}; border: 1px solid {t['group_border']}; border-radius: 4px;")
        self.labelRightEye.setAlignment(Qt.AlignCenter)
        
        lyt_left_eye = QtWidgets.QVBoxLayout(); lyt_left_eye.addWidget(QtWidgets.QLabel("Left Eye"), alignment=Qt.AlignCenter); lyt_left_eye.addWidget(self.labelLeftEye)
        lyt_right_eye = QtWidgets.QVBoxLayout(); lyt_right_eye.addWidget(QtWidgets.QLabel("Right Eye"), alignment=Qt.AlignCenter); lyt_right_eye.addWidget(self.labelRightEye)
        
        eyes_col.addLayout(lyt_left_eye)
        eyes_col.addLayout(lyt_right_eye)
        eyes_col.addStretch()
        scene_eye_row.addLayout(eyes_col, stretch=1)
        
        content_layout.addLayout(scene_eye_row, stretch=2)

        # -- Bottom Content: Plots Grid --
        lo_plots = QtWidgets.QGridLayout()
        lo_plots.setHorizontalSpacing(8)
        lo_plots.setVerticalSpacing(8)
        
        # PPG Widget
        self.ppg_widget = pg.GraphicsLayoutWidget()
        self.ppg_widget.setBackground(t['bg'])
        self.ppg_red_plot = self.ppg_widget.addPlot(row=0, col=0, title="PPG Red")
        self.ppg_ir_plot = self.ppg_widget.addPlot(row=1, col=0, title="PPG IR")
        self.ppg_green_plot = self.ppg_widget.addPlot(row=2, col=0, title="PPG Green")
        
        # Link PPG axes together to align them natively inside the GraphicsLayout
        self.ppg_ir_plot.setXLink(self.ppg_red_plot)
        self.ppg_green_plot.setXLink(self.ppg_red_plot)
        self.ppg_red_plot.hideAxis('bottom')
        self.ppg_ir_plot.hideAxis('bottom')
        self.ppg_green_plot.setLabel("bottom", "Time", units="s")

        self.ppg_curves = []
        for plt_ref, col in [(self.ppg_red_plot, "#FF6B6B"), (self.ppg_ir_plot, "#2EC4B6"), (self.ppg_green_plot, "#90BE6D")]:
            plt_ref.showGrid(x=True, y=True, alpha=t['grid_alpha'])
            plt_ref.setXRange(-self.args.window_seconds, 0.0, padding=0.0)
            self.ppg_curves.append(plt_ref.plot(pen=pg.mkPen(color=col, width=1.5)))
        
        self.accel_plot = self._create_pg_plot("Accelerometer", "g", ["#4CC9F0", "#FFD166", "#06D6A0"], ["X", "Y", "Z"], self.ppg_red_plot)
        self.gyro_plot = self._create_pg_plot("Gyroscope", "dps", ["#F72585", "#B5179E", "#7209B7"], ["X", "Y", "Z"], self.ppg_red_plot)
        self.mag_plot = self._create_pg_plot("Magnetometer", "µT", ["#FF9F1C", "#E71D36", "#2EC4B6"], ["X", "Y", "Z"], self.ppg_red_plot)

        # Eye tracker plots
        self.gaze_plot_widget = pg.PlotWidget()
        gaze_it = self.gaze_plot_widget.getPlotItem()
        gaze_it.setTitle("Gaze X / Y"); gaze_it.showGrid(x=True, y=True, alpha=t['grid_alpha'])
        gaze_it.setXRange(-self.args.window_seconds, 0.0, padding=0.0)
        gaze_it.setXLink(self.ppg_red_plot)
        gaze_it.addLegend(offset=(6,6))
        self.gaze_x_curve = self.gaze_plot_widget.plot(pen=pg.mkPen("#4ED1A6", width=2), name="Gaze X")
        self.gaze_y_curve = self.gaze_plot_widget.plot(pen=pg.mkPen("#FF7F50", width=2), name="Gaze Y")
        
        self.pupil_plot_widget = pg.PlotWidget()
        pupil_it = self.pupil_plot_widget.getPlotItem()
        pupil_it.setTitle("Pupil Diameter (mm)"); pupil_it.showGrid(x=True, y=True, alpha=t['grid_alpha'])
        pupil_it.setXRange(-self.args.window_seconds, 0.0, padding=0.0)
        pupil_it.setXLink(self.ppg_red_plot)
        pupil_it.addLegend(offset=(6,6))
        self.pupil_l_curve = self.pupil_plot_widget.plot(pen=pg.mkPen("#9AD1FF", width=2), name="Left")
        self.pupil_r_curve = self.pupil_plot_widget.plot(pen=pg.mkPen("#FFD166", width=2), name="Right")

        lo_plots.addWidget(self.ppg_widget, 0, 0)
        lo_plots.addWidget(self.accel_plot[0], 0, 1)
        lo_plots.addWidget(self.gyro_plot[0], 0, 2)
        lo_plots.addWidget(self.mag_plot[0], 1, 0)
        lo_plots.addWidget(self.gaze_plot_widget, 1, 1)
        lo_plots.addWidget(self.pupil_plot_widget, 1, 2)

        content_layout.addLayout(lo_plots, stretch=1)

    def _create_pg_plot(self, title, ylabel, colors, legend, xlink_target=None):
        w = pg.PlotWidget()
        item = w.getPlotItem()
        item.setTitle(title)
        item.setLabel("left", ylabel)
        item.setLabel("bottom", "Time", units="s")
        item.showGrid(x=True, y=True, alpha=self.theme['grid_alpha'])
        item.setMenuEnabled(False)
        item.setMouseEnabled(x=False, y=False)
        item.setXRange(-self.args.window_seconds, 0.0, padding=0.0)
        if xlink_target:
            item.setXLink(xlink_target)
        item.addLegend(offset=(6, 6))
        curves = [item.plot(pen=pg.mkPen(color=col, width=1.5), name=name) for col, name in zip(colors, legend)]
        return w, curves

    def _connect_signals(self):
        self.btn_start.clicked.connect(self._on_start_all)
        self.btn_stop.clicked.connect(self._on_stop_all)
        self.btn_cal_start.clicked.connect(self._on_start_calibration)
        self.btn_cal_stop.clicked.connect(self._on_stop_calibration)

        self.set_sdk_running_signal.connect(self._on_set_sdk_running)
        self.set_pupil_center_signal.connect(self._display_pupil_data)
        self.set_gaze_signal.connect(self._display_gaze_data)
        self.set_scene_image_signal.connect(self._display_scene_image)
        self.set_left_eye_image_signal.connect(self._display_left_eye_image)
        self.set_right_eye_image_signal.connect(self._display_right_eye_image)
        self.set_calibration_finish_signal.connect(self._on_set_calibration_finish)

    # ---- Eye Tracker Callbacks ----
    def _display_scene_image(self, image):
        painter = QPainter(image)
        painter.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        c = QColor(); c.setGreen(255)
        painter.setBrush(QBrush(c))
        rect = QRectF(self.cur_gaze_x - 15, self.cur_gaze_y - 15, 30, 30)
        painter.drawEllipse(rect)
        self.labelSceneImage.setPixmap(image)
    
    def _display_left_eye_image(self, image): self.labelLeftEye.setPixmap(image)
    def _display_right_eye_image(self, image): self.labelRightEye.setPixmap(image)

    def _display_pupil_data(self, lx, ly, rx, ry):
        self.lbl_pupil_x.setText(str(lx)); self.lbl_pupil_y.setText(str(ly))

    def _display_gaze_data(self, x, y):
        self.lbl_gaze_x.setText(str(x)); self.lbl_gaze_y.setText(str(y))
        self.cur_gaze_x = x + self.scene_width / 2
        self.cur_gaze_y = y + self.scene_height / 2

    def _on_set_sdk_running(self, enabled):
        self.eye_sdk_running = enabled
        if not enabled:
            self.labelSceneImage.setPixmap(QPixmap())
            self.labelLeftEye.setPixmap(QPixmap())
            self.labelRightEye.setPixmap(QPixmap())

    def _on_set_calibration_finish(self, eye, index, error):
        self.finish_points[index - 1][eye] = error
        n = self.current_points
        # (Simplified calibration end check logic)
        err = False
        if n == 1: err = (1 != self.finish_points[0][0] and 1 != self.finish_points[0][1])
        if err: self._on_stop_calibration()

    def _on_scene_clicked(self, x, y):
        if not self.eye_sdk_running: return
        px = float(x) - float(self.scene_width / 2)
        py = float(y) - float(self.scene_height / 2)
        self.eye_sdk.set_current_point(px, py)

    def _on_start_calibration(self):
        self.current_points = int(self.combo_points.currentText())
        for i in range(len(self.finish_points)): self.finish_points[i] = [1, 1]
        self.calibration_is_running = True
        self.eye_sdk.start_calibration(self.current_points)
        self.btn_cal_start.setEnabled(False)

    def _on_stop_calibration(self):
        self.eye_sdk.stop_calibration()
        self.calibration_is_running = False
        self.btn_cal_start.setEnabled(True)

    # ---- Actions ----
    def _read_pwd(self):
        cf = configparser.ConfigParser()
        cf.read(os.path.join(self.eye_sdk_config_path, "config.ini"))
        return cf.get("softdog", "pwd", fallback="").encode("utf-8")

    def _on_start_all(self):
        self.system_running = True
        self.btn_start.setEnabled(False)
        
        # Prepare Logging Dirs
        log_dir = os.path.join(os.path.dirname(os.getcwd()), "log")
        os.makedirs(log_dir, exist_ok=True)
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Start Sensor
        self.sensor_reader = SerialPacketReader(self.combo_port.currentText(), int(self.combo_baud.currentText()))
        if self.sensor_reader.start():
            # TS offset logic requires reader serial to be opened
            time.sleep(SERIAL_STARTUP_SETTLE_SECONDS)
            self.sensor_reader.serial_port.reset_input_buffer()
            self.ts_offset = sync_timestamps(self.sensor_reader.serial_port)
            self.sensor_csv_writer = SensorCSVWriterThread(os.path.join(log_dir, f"sensor_{ts_str}.csv"))
            self.sensor_csv_writer.start()
            self.sensor_reader.send_command("s\n")
        else: QtWidgets.QMessageBox.warning(self, "Warning", "Sensor failed to connect.")

        # Start Eye Tracker
        pwd = self._read_pwd()
        if self.eye_sdk.connect_softdog(pwd) == 0:
            env = self.combo_env.currentData()
            res = self.combo_res.currentData()
            self.scene_width, self.scene_height = (1280, 960) if res == 201 else (1280, 720) if res == 202 else (800, 600) if res == 203 else (1920, 1080)
            
            if self.eye_sdk.start(env, res, self.scene_width, self.scene_height) == 0:
                self.eye_csv_writer = EyeCSVWriterThread(os.path.join(log_dir, f"eye_{ts_str}.csv"))
                self.eye_csv_writer.start()
                self.btn_cal_start.setEnabled(True)
            else: QtWidgets.QMessageBox.warning(self, "Warning", "Eye tracker start failed.")
        else: QtWidgets.QMessageBox.warning(self, "Warning", "Eye tracker dog connect failed.")

        self.btn_stop.setEnabled(True)
        self.timer.start(PLOT_UPDATE_INTERVAL_MS)

    def _on_stop_all(self):
        self.system_running = False
        self.timer.stop()
        self.btn_stop.setEnabled(False)
        self.btn_start.setEnabled(True)
        self.btn_cal_start.setEnabled(False)

        # Stop Sensor
        if self.sensor_reader:
            try: self.sensor_reader.send_command("e\n")
            except: pass
            self.sensor_reader.stop()
        if self.sensor_csv_writer: self.sensor_csv_writer.stop()
        
        # Stop Eye
        if self.calibration_is_running: self._on_stop_calibration()
        self.eye_sdk.stop()
        if self.eye_csv_writer: self.eye_csv_writer.stop()

    # ---- Timer Update ----
    def _on_timer(self):
        now_ms = time.perf_counter() * 1000

        # Drain Sensor
        if self.sensor_reader:
            red_b, ir_b, green_b, ax_b, ay_b, az_b, gx_b, gy_b, gz_b, mx_b, my_b, mz_b = ([] for _ in range(12))
            while True:
                try: p = self.sensor_reader.packet_queue.get_nowait()
                except queue.Empty: break
                
                self.sensor_rate_tracker.push(p["timestamp"])
                red_b.append(p["red"]); ir_b.append(p["ir"]); green_b.append(p["green"])
                ax_b.append(p["ax"]); ay_b.append(p["ay"]); az_b.append(p["az"])
                gx_b.append(p["gx"]); gy_b.append(p["gy"]); gz_b.append(p["gz"])
                mx_b.append(p["mx"]); my_b.append(p["my"]); mz_b.append(p["mz"])
                self.sensor_latest_temp = p["temp"]
                
                if self.sensor_csv_writer:
                    pc_ts = p["timestamp_ms"]/1000.0 + self.ts_offset
                    self.sensor_csv_writer.push([p["red"], p["ir"], p["green"], p["ax"], p["ay"], p["az"],
                                                 p["gx"], p["gy"], p["gz"], p["mx"], p["my"], p["mz"], p["temp"], p["timestamp_ms"], pc_ts])
            if red_b:
                self.sensor_buffers.red.append(red_b); self.sensor_buffers.ir.append(ir_b); self.sensor_buffers.green.append(green_b)
                self.sensor_buffers.ax.append(ax_b); self.sensor_buffers.ay.append(ay_b); self.sensor_buffers.az.append(az_b)
                self.sensor_buffers.gx.append(gx_b); self.sensor_buffers.gy.append(gy_b); self.sensor_buffers.gz.append(gz_b)
                self.sensor_buffers.mx.append(mx_b); self.sensor_buffers.my.append(my_b); self.sensor_buffers.mz.append(mz_b)

        # Drain Eye
        if self.eye_sdk_running:
            while True:
                try: s = self.eye_sdk.data_queue.get_nowait()
                except queue.Empty: break
                self.eye_rate_tracker.push(s["perf_timestamp"])
                self.eye_packet_count += 1
                self.eye_gaze_x_series.append([s["gaze_x"]])
                self.eye_gaze_y_series.append([s["gaze_y"]])
                self.eye_left_pupil_series.append([s["left_pupil_diameter_mm"]])
                self.eye_right_pupil_series.append([s["right_pupil_diameter_mm"]])
                if self.eye_csv_writer: self.eye_csv_writer.push(s)

        # Plot update
        if now_ms - self.last_plot_update_ms >= PLOT_UPDATE_INTERVAL_MS:
            self.last_plot_update_ms = now_ms
            self._update_plots()

        # Ranges
        if now_ms - self.last_range_update_ms >= Y_LIMIT_UPDATE_INTERVAL_MS:
            self.last_range_update_ms = now_ms
            self._update_ranges()

        # Metrics
        if now_ms - self.last_metric_update_ms >= METRIC_UPDATE_INTERVAL_MS:
            self.last_metric_update_ms = now_ms
            self._update_metrics()

    def _update_plots(self):
        # Update Sensor Plots
        red = self.sensor_buffers.red.ordered()
        if len(red) > 0:
            x_sen = self.sensor_buffers.red.x_axis(len(red))
            self.ppg_curves[0].setData(x_sen, red)
            self.ppg_curves[1].setData(x_sen, self.sensor_buffers.ir.ordered())
            self.ppg_curves[2].setData(x_sen, self.sensor_buffers.green.ordered())
            self.accel_plot[1][0].setData(x_sen, self.sensor_buffers.ax.ordered()); self.accel_plot[1][1].setData(x_sen, self.sensor_buffers.ay.ordered()); self.accel_plot[1][2].setData(x_sen, self.sensor_buffers.az.ordered())
            self.gyro_plot[1][0].setData(x_sen, self.sensor_buffers.gx.ordered()); self.gyro_plot[1][1].setData(x_sen, self.sensor_buffers.gy.ordered()); self.gyro_plot[1][2].setData(x_sen, self.sensor_buffers.gz.ordered())
            self.mag_plot[1][0].setData(x_sen, self.sensor_buffers.mx.ordered()); self.mag_plot[1][1].setData(x_sen, self.sensor_buffers.my.ordered()); self.mag_plot[1][2].setData(x_sen, self.sensor_buffers.mz.ordered())

        # Update Eye Plots
        gaze_x = self.eye_gaze_x_series.ordered()
        if len(gaze_x) > 0:
            x_eye = self.eye_gaze_x_series.x_axis(len(gaze_x))
            self.gaze_x_curve.setData(x_eye, gaze_x)
            self.gaze_y_curve.setData(x_eye, self.eye_gaze_y_series.ordered())
            
            p_l = self.eye_left_pupil_series.ordered()
            if len(p_l) > 0:
                p_x = self.eye_left_pupil_series.x_axis(len(p_l))
                self.pupil_l_curve.setData(p_x, p_l)
                self.pupil_r_curve.setData(p_x, self.eye_right_pupil_series.ordered())

    def _auto_range(self, plot_item, values):
        if len(values) < 8: return
        lower = float(np.percentile(values, 1.0)); upper = float(np.percentile(values, 99.0))
        if not np.isfinite(lower) or not np.isfinite(upper): return
        if upper <= lower:
            center = float(np.mean(values)); span = max(1.0, float(np.std(values)) * 4.0)
            lower = center - span * 0.5; upper = center + span * 0.5
        padding = max(1.0, (upper - lower) * 0.1)
        plot_item.setYRange(lower - padding, upper + padding, padding=0.0)

    def _update_ranges(self):
        self._auto_range(self.ppg_red_plot, self.sensor_buffers.red.ordered())
        self._auto_range(self.ppg_ir_plot, self.sensor_buffers.ir.ordered())
        self._auto_range(self.ppg_green_plot, self.sensor_buffers.green.ordered())
        acc_s = np.vstack((self.sensor_buffers.ax.ordered(), self.sensor_buffers.ay.ordered(), self.sensor_buffers.az.ordered())).flatten()
        gyr_s = np.vstack((self.sensor_buffers.gx.ordered(), self.sensor_buffers.gy.ordered(), self.sensor_buffers.gz.ordered())).flatten()
        mag_s = np.vstack((self.sensor_buffers.mx.ordered(), self.sensor_buffers.my.ordered(), self.sensor_buffers.mz.ordered())).flatten()
        if len(acc_s) > 0: self._auto_range(self.accel_plot[0].getPlotItem(), acc_s)
        if len(gyr_s) > 0: self._auto_range(self.gyro_plot[0].getPlotItem(), gyr_s)
        if len(mag_s) > 0: self._auto_range(self.mag_plot[0].getPlotItem(), mag_s)

    def _update_metrics(self):
        self.lbl_sensor_rate.setText(f"Sen Rate: {self.sensor_rate_tracker.current_rate():.1f} Hz")
        self.lbl_sensor_packets.setText(f"Sen Pkts: {self.sensor_rate_tracker.total_packets}")
        tval = self.sensor_latest_temp
        self.lbl_sensor_temp.setText(f"Sen Temp: {tval:.2f}" if tval and tval == tval else "Sen Temp: --")
        self.lbl_eye_rate.setText(f"Eye Rate: {self.eye_rate_tracker.current_rate():.1f} Hz")
        self.lbl_eye_packets.setText(f"Eye Pkts: {self.eye_packet_count}")

    def closeEvent(self, event):
        if self.system_running: self._on_stop_all()
        super().closeEvent(event)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-root", default="E:/7invensun/aSeeGlassesPlusUserSDK")
    parser.add_argument("--port", type=str, default="COM5")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--sensor-sample-rate", type=float, default=250.0)
    parser.add_argument("--eye-sample-rate", type=float, default=120.0)
    parser.add_argument("--window-seconds", type=float, default=DISPLAY_WINDOW_SECONDS)
    return parser.parse_args()

def main():
    if hasattr(QtCore.Qt, 'AA_EnableHighDpiScaling'):
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    if hasattr(QtCore.Qt, 'AA_UseHighDpiPixmaps'):
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
        
    args = parse_args()
    app = QtWidgets.QApplication(sys.argv)
    window = IntegratedMonitorWindow(args)
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()