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
from serial.tools import list_ports

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen, QPixmap, QFont
from PyQt5.QtCore import Qt, QRectF
import pyqtgraph as pg

# ---- Eye Tracker SDK Imports ----
from sdk_types import PY_7I_ENVIRONMENT, PY_7I_RESOLUTION
from sdk_wrapper import wrapper


# ===================== Constants =====================
# Sensor
PACKET_SIZE = 64
SYNC_PACKET_SIZE = 80
NUM_FLOATS = 14
SYNC_MARKER = bytes([0x55, 0xAA, 0x55, 0xAA])
SYNC_LEN = 4
SYNC_PACKET_FORMAT = '<14fIIHH8x'

DISPLAY_WINDOW_SECONDS = 6.0
PLOT_UPDATE_INTERVAL_MS = 33
METRIC_UPDATE_INTERVAL_MS = 250
Y_LIMIT_UPDATE_INTERVAL_MS = 500
SERIAL_STARTUP_SETTLE_SECONDS = 2.0
RATE_WINDOW_SECONDS = 3.0
TARGET_SERIAL_DESCRIPTION = "Silicon Labs CP210x USB to UART Bridge"
CALIBRATION_PROFILE_DIR = "calibration_profiles"
SYNC_EVENT_RETENTION_SECONDS = 5.0

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
class AspectRatioPixmapLabel(QtWidgets.QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._source_pixmap = QPixmap()

    def setPixmap(self, pixmap):
        self._source_pixmap = QPixmap(pixmap)
        self._update_scaled_pixmap()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_scaled_pixmap()

    def displayed_pixmap_rect(self):
        pixmap = super().pixmap()
        if pixmap is None or pixmap.isNull():
            return self.contentsRect()

        contents = self.contentsRect()
        x = contents.x() + (contents.width() - pixmap.width()) // 2
        y = contents.y() + (contents.height() - pixmap.height()) // 2
        return QtCore.QRect(x, y, pixmap.width(), pixmap.height())

    def _update_scaled_pixmap(self):
        if self._source_pixmap.isNull():
            super().setPixmap(QPixmap())
            return

        contents = self.contentsRect()
        if contents.width() <= 0 or contents.height() <= 0:
            return

        scaled = self._source_pixmap.scaled(
            contents.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        super().setPixmap(scaled)


class SceneImageLabel(AspectRatioPixmapLabel):
    button_clicked_signal = QtCore.pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)

    def mousePressEvent(self, ev):
        if ev.buttons() == Qt.LeftButton:
            target_rect = self.displayed_pixmap_rect()
            if target_rect.width() > 0 and target_rect.height() > 0 and target_rect.contains(ev.pos()):
                norm_x = (ev.x() - target_rect.x()) / target_rect.width()
                norm_y = (ev.y() - target_rect.y()) / target_rect.height()
                self.button_clicked_signal.emit(norm_x, norm_y)

    def connect_customized_slot(self, slot_func):
        self.button_clicked_signal.connect(slot_func)


class SensorPortComboBox(QtWidgets.QComboBox):
    popup_requested = QtCore.pyqtSignal()

    def showPopup(self):
        self.popup_requested.emit()
        super().showPopup()


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


class AdaptiveRisingEdgeDetector:
    def __init__(self, history_size=180, min_samples=30, refractory_seconds=0.8):
        self.history_size = history_size
        self.min_samples = min_samples
        self.refractory_seconds = refractory_seconds
        self.values = deque(maxlen=history_size)
        self.deltas = deque(maxlen=history_size)
        self.prev_value = None
        self.last_event_time = None

    def reset(self):
        self.values.clear()
        self.deltas.clear()
        self.prev_value = None
        self.last_event_time = None

    def push(self, timestamp, value):
        value = float(value)
        if self.prev_value is None:
            self.prev_value = value
            self.values.append(value)
            return None

        delta = value - self.prev_value
        detected = None
        if len(self.values) >= self.min_samples and len(self.deltas) >= self.min_samples:
            value_arr = np.asarray(self.values, dtype=np.float64)
            delta_arr = np.asarray(self.deltas, dtype=np.float64)
            value_med = float(np.median(value_arr))
            delta_med = float(np.median(delta_arr))
            value_mad = self._mad(value_arr, value_med)
            delta_mad = self._mad(delta_arr, delta_med)
            slope_score = (delta - delta_med) / delta_mad
            level_score = (value - value_med) / value_mad
            refractory_ok = (
                self.last_event_time is None
                or timestamp - self.last_event_time >= self.refractory_seconds
            )
            if refractory_ok and slope_score >= 8.0 and level_score >= 3.0:
                self.last_event_time = timestamp
                detected = {
                    "time": float(timestamp),
                    "value": value,
                    "slope_score": float(slope_score),
                    "level_score": float(level_score),
                }

        self.values.append(value)
        self.deltas.append(delta)
        self.prev_value = value
        return detected

    @staticmethod
    def _mad(values, median):
        mad = float(np.median(np.abs(values - median)))
        return max(mad * 1.4826, 1e-9)


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
    def __init__(self, port, baudrate, packet_mode="normal"):
        self.port = port
        self.baudrate = baudrate
        self.packet_mode = packet_mode
        self.packet_size = SYNC_PACKET_SIZE if packet_mode == "sync" else PACKET_SIZE
        self.serial_port = None
        self.packet_queue = queue.SimpleQueue()
        self.ack_queue = queue.SimpleQueue()
        self._buffer = bytearray()
        self._text_buffer = bytearray()
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._thread_started = False

    def open(self):
        try:
            self.serial_port = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.05)
            self._stop_event.clear()
            return True
        except Exception as e:
            print(f"Serial conn err: {e}")
            self.serial_port = None
            return False

    def start(self):
        if not self.serial_port and not self.open():
            return False
        if self._thread_started:
            return True
        try:
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self._thread_started = True
            return True
        except Exception as e:
            print(f"Serial reader err: {e}")
            return False

    def stop(self):
        self._stop_event.set()
        if self._thread_started and self._thread:
            self._thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self._thread_started = False
        self._thread = None

    def send_command(self, command_text):
        if not self.serial_port or not self.serial_port.is_open: return
        with self._write_lock:
            self.serial_port.write(command_text.encode("ascii"))
            self.serial_port.flush()

    def send_sync_start(self, sync_id):
        if not self.serial_port or not self.serial_port.is_open:
            return None
        with self._write_lock:
            self.serial_port.write(f"SYNC_START {int(sync_id)}\n".encode("ascii"))
            self.serial_port.flush()
            return time.perf_counter()

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
                preserve = self._marker_prefix_len()
                if preserve:
                    self._consume_text_preamble(bytes(self._buffer[:-preserve]))
                    self._buffer[:] = self._buffer[-preserve:]
                else:
                    self._consume_text_preamble(bytes(self._buffer))
                    self._buffer.clear()
                return
            if start_index > 0:
                self._consume_text_preamble(bytes(self._buffer[:start_index]))
                del self._buffer[:start_index]
            if len(self._buffer) < self.packet_size: return
            payload = bytes(self._buffer[SYNC_LEN:self.packet_size])
            del self._buffer[:self.packet_size]
            try:
                if self.packet_mode == "sync":
                    unpacked = struct.unpack(SYNC_PACKET_FORMAT, payload)
                    values = unpacked[:NUM_FLOATS]
                    sync_trigger_us = int(unpacked[NUM_FLOATS])
                    sample_us = int(unpacked[NUM_FLOATS + 1])
                    sync_id = int(unpacked[NUM_FLOATS + 2])
                    sync_flags = int(unpacked[NUM_FLOATS + 3])
                else:
                    values = struct.unpack('<14f', payload[:NUM_FLOATS * 4])
                    sync_trigger_us = None
                    sample_us = None
                    sync_id = None
                    sync_flags = None
            except struct.error: continue

            if sum(1 for v in values if v != v) > 3: continue
            timestamp = values[13]
            if timestamp == timestamp and (timestamp < 0 or timestamp > 4.3e9): continue

            packet = {
                "timestamp": time.perf_counter(), "red": values[0], "ir": values[1], "green": values[2],
                "ax": values[3], "ay": values[4], "az": values[5], "gx": values[6], "gy": values[7],
                "gz": values[8], "mx": values[9], "my": values[10], "mz": values[11], "temp": values[12],
                "timestamp_ms": values[13],
            }
            if self.packet_mode == "sync":
                packet.update({
                    "sync_id": sync_id,
                    "sync_trigger_us": sync_trigger_us,
                    "sync_sample_us": sample_us,
                    "sync_flags": sync_flags,
                    "timestamp_ms": sample_us / 1000.0,
                })
            self.packet_queue.put(packet)

    def _marker_prefix_len(self):
        max_len = min(len(self._buffer), len(SYNC_MARKER) - 1)
        for size in range(max_len, 0, -1):
            if self._buffer[-size:] == SYNC_MARKER[:size]:
                return size
        return 0

    def _consume_text_preamble(self, data):
        if not data:
            return
        self._text_buffer.extend(data)
        while b'\n' in self._text_buffer:
            line, _, rest = self._text_buffer.partition(b'\n')
            self._text_buffer = bytearray(rest)
            self._handle_text_line(line.decode("ascii", errors="ignore").strip())

    def _handle_text_line(self, line):
        if not line:
            return
        parts = line.split()
        if len(parts) >= 3 and parts[0] == "SYNC_ACK":
            try:
                self.ack_queue.put({
                    "sync_id": int(parts[1]),
                    "trigger_us": int(parts[2]),
                    "ack_pc": time.perf_counter(),
                })
            except ValueError:
                pass


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
        self.project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.log_dir = os.path.join(self.project_root, "log")
        self.calibration_root = os.path.join(self.project_root, CALIBRATION_PROFILE_DIR)
        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.calibration_root, exist_ok=True)

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
        self.sync_running = False
        self.sync_target_count = 3
        self.sync_results = []
        self.sync_output_file = None
        self.sync_ppg_events = deque()
        self.sync_left_events = deque()
        self.sync_right_events = deque()
        self.sync_next_id = 1
        self.sync_pending_triggers = {}
        self.sync_current_trigger = None
        self.sync_confirmation_pending = False
        self.sync_ppg_detectors = {
            "Red": AdaptiveRisingEdgeDetector(history_size=250, min_samples=40),
            "IR": AdaptiveRisingEdgeDetector(history_size=250, min_samples=40),
            "Green": AdaptiveRisingEdgeDetector(history_size=250, min_samples=40),
        }
        self.sync_left_detector = AdaptiveRisingEdgeDetector(history_size=90, min_samples=12)
        self.sync_right_detector = AdaptiveRisingEdgeDetector(history_size=90, min_samples=12)

        self._build_ui()
        self._connect_signals()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_timer)

    def _build_ui(self):
        self.setWindowTitle("Integrated Monitor Platform (Eye Tracker + Sensor)")
        self.resize(1600, 1000)
        
        t = self.theme
        mono_font = QFont("Consolas", 12)
        mono_font.setStyleHint(QFont.TypeWriter)
        value_width = QtGui.QFontMetrics(mono_font).horizontalAdvance("-0000.00000") + 16
        sidebar_width = 240
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
        self.btn_start.setFixedWidth(sidebar_width)
        self.btn_stop = QtWidgets.QPushButton("■ Stop All")
        self.btn_stop.setStyleSheet(btn_ss)
        self.btn_stop.setEnabled(False)
        self.btn_stop.setFixedWidth(sidebar_width)
        sidebar.addWidget(self.btn_start)
        sidebar.addWidget(self.btn_stop)

        # Sync Mode
        grp_sync = QtWidgets.QGroupBox("Sync Mode")
        grp_sync.setStyleSheet(group_ss)
        grp_sync.setFixedWidth(sidebar_width)
        lo_sync = QtWidgets.QVBoxLayout(grp_sync)
        self.spin_sync_count = QtWidgets.QSpinBox()
        self.spin_sync_count.setRange(3, 30)
        self.spin_sync_count.setValue(3)
        self.spin_sync_count.setStyleSheet(combo_ss)
        self.btn_sync_start = QtWidgets.QPushButton("Start Sync")
        self.btn_sync_start.setStyleSheet(btn_ss)
        self.btn_sync_stop = QtWidgets.QPushButton("Stop Sync")
        self.btn_sync_stop.setStyleSheet(btn_ss)
        self.btn_sync_stop.setEnabled(False)
        self.lbl_sync_status = QtWidgets.QLabel("Sync: idle")
        self.lbl_sync_status.setStyleSheet(f"font-size: 12px; color: {t['label_color']};")
        lo_sync.addWidget(QtWidgets.QLabel("Flash count:"))
        lo_sync.addWidget(self.spin_sync_count)
        lo_sync.addWidget(self.btn_sync_start)
        lo_sync.addWidget(self.btn_sync_stop)
        lo_sync.addWidget(self.lbl_sync_status)
        sidebar.addWidget(grp_sync)
        
        # Sensor Settings
        grp_sensor = QtWidgets.QGroupBox("Sensor Settings")
        grp_sensor.setStyleSheet(group_ss)
        grp_sensor.setFixedWidth(sidebar_width)
        lo_sensor = QtWidgets.QVBoxLayout(grp_sensor)
        self.combo_port = SensorPortComboBox()
        self.combo_port.setStyleSheet(combo_ss)
        self.combo_port.popup_requested.connect(self._populate_sensor_ports)
        self._populate_sensor_ports()
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
        grp_eye.setFixedWidth(sidebar_width)
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
        grp_cal.setFixedWidth(sidebar_width)
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
        self.btn_cal_stop.setEnabled(False)
        lo_cal.addWidget(QtWidgets.QLabel("Points:")); lo_cal.addWidget(self.combo_points)
        lo_cal.addWidget(self.btn_cal_start); lo_cal.addWidget(self.btn_cal_stop)
        sidebar.addWidget(grp_cal)

        grp_profiles = QtWidgets.QGroupBox("Calibration Profile")
        grp_profiles.setStyleSheet(group_ss)
        grp_profiles.setFixedWidth(sidebar_width)
        lo_profiles = QtWidgets.QVBoxLayout(grp_profiles)
        self.combo_calibration_profile = QtWidgets.QComboBox()
        self.combo_calibration_profile.setStyleSheet(combo_ss)
        self.combo_calibration_profile.setEditable(True)
        self.btn_refresh_profiles = QtWidgets.QPushButton("Refresh Profiles")
        self.btn_refresh_profiles.setStyleSheet(btn_ss)
        self.btn_load_profile = QtWidgets.QPushButton("Load Selected")
        self.btn_load_profile.setStyleSheet(btn_ss)
        self.btn_save_profile = QtWidgets.QPushButton("Save Current")
        self.btn_save_profile.setStyleSheet(btn_ss)
        lo_profiles.addWidget(self.combo_calibration_profile)
        lo_profiles.addWidget(self.btn_refresh_profiles)
        lo_profiles.addWidget(self.btn_load_profile)
        lo_profiles.addWidget(self.btn_save_profile)
        sidebar.addWidget(grp_profiles)
        self._refresh_calibration_profiles()
        
        # Realtime Values
        grp_vals = QtWidgets.QGroupBox("Eye Tracker Live Data")
        grp_vals.setStyleSheet(group_ss)
        grp_vals.setFixedWidth(sidebar_width)
        lo_vals = QtWidgets.QGridLayout(grp_vals)
        lo_vals.setHorizontalSpacing(6)
        lo_vals.setVerticalSpacing(4)
        lo_vals.setColumnMinimumWidth(0, 48)
        lo_vals.setColumnMinimumWidth(1, value_width)
        lo_vals.setColumnStretch(1, 1)
        self.lbl_gaze_x, self.lbl_gaze_y = QtWidgets.QLabel(""), QtWidgets.QLabel("")
        self.lbl_pupil_x, self.lbl_pupil_y = QtWidgets.QLabel(""), QtWidgets.QLabel("")
        for l in (self.lbl_gaze_x, self.lbl_gaze_y, self.lbl_pupil_x, self.lbl_pupil_y):
            l.setStyleSheet(val_ss)
            l.setFont(mono_font)
            l.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            l.setFixedWidth(value_width)
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
        self.labelSceneImage.setText("Scene Image")
        self.labelSceneImage.setMinimumSize(640, 360)
        scene_eye_row.addWidget(self.labelSceneImage, stretch=3)
        
        eyes_col = QtWidgets.QVBoxLayout()
        self.labelLeftEye = AspectRatioPixmapLabel()
        self.labelLeftEye.setText("left eye")
        self.labelLeftEye.setMinimumSize(200, 150)
        self.labelLeftEye.setMaximumWidth(220)
        self.labelLeftEye.setStyleSheet(f"background:{t['value_bg']}; border: 1px solid {t['group_border']}; border-radius: 4px;")
        self.labelLeftEye.setAlignment(Qt.AlignCenter)
        self.labelRightEye = AspectRatioPixmapLabel()
        self.labelRightEye.setText("right eye")
        self.labelRightEye.setMinimumSize(200, 150)
        self.labelRightEye.setMaximumWidth(220)
        self.labelRightEye.setStyleSheet(f"background:{t['value_bg']}; border: 1px solid {t['group_border']}; border-radius: 4px;")
        self.labelRightEye.setAlignment(Qt.AlignCenter)
        
        lyt_left_eye = QtWidgets.QVBoxLayout(); lyt_left_eye.addWidget(QtWidgets.QLabel("Left Eye"), alignment=Qt.AlignCenter); lyt_left_eye.addWidget(self.labelLeftEye)
        lyt_right_eye = QtWidgets.QVBoxLayout(); lyt_right_eye.addWidget(QtWidgets.QLabel("Right Eye"), alignment=Qt.AlignCenter); lyt_right_eye.addWidget(self.labelRightEye)
        
        eyes_col.addLayout(lyt_left_eye)
        eyes_col.addLayout(lyt_right_eye)
        eyes_col.addStretch()
        scene_eye_row.addLayout(eyes_col, stretch=1)
        
        content_layout.addLayout(scene_eye_row, stretch=3)

        # -- Bottom Content: Plot Columns --
        plots_row = QtWidgets.QHBoxLayout()
        plots_row.setSpacing(8)
        
        # PPG Widget
        self.ppg_widget = pg.GraphicsLayoutWidget()
        self.ppg_widget.setBackground(t['bg'])
        self.ppg_red_plot = self.ppg_widget.addPlot(row=0, col=0, title="PPG Red")
        self.ppg_ir_plot = self.ppg_widget.addPlot(row=1, col=0, title="PPG IR")
        self.ppg_green_plot = self.ppg_widget.addPlot(row=2, col=0, title="PPG Green")
        
        # Link PPG axes together to align them natively inside the GraphicsLayout
        self.ppg_ir_plot.setXLink(self.ppg_red_plot)
        self.ppg_green_plot.setXLink(self.ppg_red_plot)
        self.ppg_red_plot.getAxis('bottom').setStyle(showValues=False)
        self.ppg_ir_plot.getAxis('bottom').setStyle(showValues=False)
        self.ppg_green_plot.setLabel("bottom", "Time", units="s")
        self.ppg_widget.ci.layout.setRowStretchFactor(0, 1)
        self.ppg_widget.ci.layout.setRowStretchFactor(1, 1)
        self.ppg_widget.ci.layout.setRowStretchFactor(2, 1)

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

        imu_column = QtWidgets.QVBoxLayout()
        imu_column.setSpacing(8)
        imu_column.addWidget(self.accel_plot[0])
        imu_column.addWidget(self.gyro_plot[0])
        imu_column.addWidget(self.mag_plot[0])

        eye_wave_column = QtWidgets.QVBoxLayout()
        eye_wave_column.setSpacing(8)
        eye_wave_column.addWidget(self.gaze_plot_widget)
        eye_wave_column.addWidget(self.pupil_plot_widget)

        plots_row.addWidget(self.ppg_widget, stretch=2)
        plots_row.addLayout(imu_column, stretch=1)
        plots_row.addLayout(eye_wave_column, stretch=1)

        content_layout.addLayout(plots_row, stretch=2)

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

    def _populate_sensor_ports(self):
        current_port = self.combo_port.currentData() or self.args.port
        self.combo_port.clear()
        matched_ports = [
            port for port in list_ports.comports()
            if self._is_target_sensor_port(port)
        ]
        matched_ports.sort(key=lambda port: port.device)

        for port in matched_ports:
            self.combo_port.addItem(f"{port.device} - {port.description}", port.device)

        preferred_index = self.combo_port.findData(current_port)
        if preferred_index >= 0:
            self.combo_port.setCurrentIndex(preferred_index)
        elif self.combo_port.count() > 0:
            self.combo_port.setCurrentIndex(0)

    def _is_target_sensor_port(self, port):
        target = TARGET_SERIAL_DESCRIPTION.casefold()
        fields = (
            port.description,
            port.manufacturer,
            port.product,
            port.interface,
        )
        return any(target in (field or "").casefold() for field in fields)

    def _sanitize_profile_name(self, value):
        cleaned = "".join(ch if ch.isalnum() or ch in ("-", "_", " ") else "_" for ch in value).strip()
        return cleaned or datetime.now().strftime("profile_%Y%m%d_%H%M%S")

    def _profile_dir_from_name(self, profile_name):
        return os.path.join(self.calibration_root, profile_name)

    def _refresh_calibration_profiles(self):
        current_text = self.combo_calibration_profile.currentText() if hasattr(self, "combo_calibration_profile") else ""
        profile_names = []
        if os.path.isdir(self.calibration_root):
            for name in sorted(os.listdir(self.calibration_root)):
                profile_dir = self._profile_dir_from_name(name)
                if os.path.isdir(profile_dir):
                    left_path = os.path.join(profile_dir, "left_coe.dat")
                    right_path = os.path.join(profile_dir, "right_coe.dat")
                    if os.path.isfile(left_path) and os.path.isfile(right_path):
                        profile_names.append(name)

        self.combo_calibration_profile.blockSignals(True)
        self.combo_calibration_profile.clear()
        self.combo_calibration_profile.addItems(profile_names)
        if current_text:
            self.combo_calibration_profile.setCurrentText(current_text)
        elif profile_names:
            self.combo_calibration_profile.setCurrentIndex(0)
        self.combo_calibration_profile.blockSignals(False)
        self._update_calibration_profile_controls()

    def _update_calibration_profile_controls(self):
        profile_name = self.combo_calibration_profile.currentText().strip()
        has_profile = False
        if profile_name:
            profile_dir = self._profile_dir_from_name(profile_name)
            has_profile = (
                os.path.isfile(os.path.join(profile_dir, "left_coe.dat"))
                and os.path.isfile(os.path.join(profile_dir, "right_coe.dat"))
            )
        self.btn_load_profile.setEnabled(self.eye_sdk_running and has_profile)
        self.btn_save_profile.setEnabled(self.eye_sdk_running)

    def _load_selected_calibration_profile(self, show_message=True):
        profile_name = self.combo_calibration_profile.currentText().strip()
        if not profile_name:
            if show_message:
                QtWidgets.QMessageBox.warning(self, "Warning", "Please select a calibration profile.")
            return False

        profile_dir = self._profile_dir_from_name(profile_name)
        try:
            self.eye_sdk.load_calibration_profile(profile_dir)
        except Exception as exc:
            if show_message:
                QtWidgets.QMessageBox.warning(self, "Warning", f"Failed to load calibration profile '{profile_name}': {exc}")
            return False
        return True

    def _save_current_calibration_profile(self):
        if not self.eye_sdk_running:
            QtWidgets.QMessageBox.warning(self, "Warning", "Start the eye tracker before saving calibration data.")
            return

        profile_name = self._sanitize_profile_name(self.combo_calibration_profile.currentText())
        profile_dir = self._profile_dir_from_name(profile_name)
        try:
            self.eye_sdk.save_calibration_profile(profile_dir)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Warning", f"Failed to save calibration profile '{profile_name}': {exc}")
            return

        self.combo_calibration_profile.setCurrentText(profile_name)
        self._refresh_calibration_profiles()

    def _load_profile_from_button(self):
        if not self.eye_sdk_running:
            QtWidgets.QMessageBox.warning(self, "Warning", "Start the eye tracker before loading calibration data.")
            return
        self._load_selected_calibration_profile(show_message=True)

    def _reset_sync_state(self):
        self.sync_results = []
        self.sync_output_file = None
        self.sync_ppg_events.clear()
        self.sync_left_events.clear()
        self.sync_right_events.clear()
        self.sync_next_id = 1
        self.sync_pending_triggers.clear()
        self.sync_current_trigger = None
        self.sync_confirmation_pending = False
        for detector in self.sync_ppg_detectors.values():
            detector.reset()
        self.sync_left_detector.reset()
        self.sync_right_detector.reset()

    def _set_sync_controls_running(self, running):
        self.btn_sync_start.setEnabled(not running and not self.system_running)
        self.btn_sync_stop.setEnabled(running)
        self.spin_sync_count.setEnabled(not running)
        self.btn_start.setEnabled(not running and not self.system_running)
        self.btn_stop.setEnabled(self.system_running)

    def _on_start_sync(self):
        if self.system_running:
            QtWidgets.QMessageBox.warning(self, "Warning", "Stop normal logging before starting sync mode.")
            return
        if self.sync_running:
            return

        self._populate_sensor_ports()
        selected_port = self.combo_port.currentData()
        if not selected_port:
            QtWidgets.QMessageBox.warning(self, "Warning", f"No '{TARGET_SERIAL_DESCRIPTION}' serial port found.")
            return

        self.sync_target_count = int(self.spin_sync_count.value())
        self._reset_sync_state()
        self.sync_output_file = os.path.join(
            self.log_dir,
            f"sync_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )
        self.lbl_sync_status.setText(f"Sync: 0/{self.sync_target_count}")

        self.sensor_reader = SerialPacketReader(selected_port, int(self.combo_baud.currentText()), packet_mode="sync")
        if not self.sensor_reader.open():
            self.sensor_reader = None
            QtWidgets.QMessageBox.warning(self, "Warning", "Sensor failed to connect.")
            return

        time.sleep(SERIAL_STARTUP_SETTLE_SECONDS)
        self.sensor_reader.serial_port.reset_input_buffer()
        self.sensor_reader.serial_port.reset_output_buffer()
        if not self.sensor_reader.start():
            self.sensor_reader.stop()
            self.sensor_reader = None
            QtWidgets.QMessageBox.warning(self, "Warning", "Sensor reader thread failed to start.")
            return

        pwd = self._read_pwd()
        if self.eye_sdk.connect_softdog(pwd) != 0:
            self._stop_sync_devices()
            QtWidgets.QMessageBox.warning(self, "Warning", "Eye tracker dog connect failed.")
            return

        env = self.combo_env.currentData()
        res = self.combo_res.currentData()
        self.scene_width, self.scene_height = (1280, 960) if res == 201 else (1280, 720) if res == 202 else (800, 600) if res == 203 else (1920, 1080)
        if self.eye_sdk.start(env, res, self.scene_width, self.scene_height) != 0:
            self._stop_sync_devices()
            QtWidgets.QMessageBox.warning(self, "Warning", "Eye tracker start failed.")
            return

        self._on_set_sdk_running(True)
        self._load_selected_calibration_profile(show_message=False)
        self.sync_running = True
        self._set_sync_controls_running(True)
        self.lbl_sync_status.setText("Sync: settling camera baseline")
        self.timer.start(PLOT_UPDATE_INTERVAL_MS)
        QtCore.QTimer.singleShot(1000, self._send_next_sync_trigger)

    def _on_stop_sync(self):
        if not self.sync_running:
            return
        self.sync_running = False
        self.timer.stop()
        self._stop_sync_devices()
        self._save_sync_results()
        self._set_sync_controls_running(False)
        self._update_sync_status(final=True)

    def _stop_sync_devices(self):
        if self.sensor_reader:
            try:
                self.sensor_reader.send_command("e\n")
                time.sleep(0.3)
            except Exception:
                pass
            self.sensor_reader.stop()
            self.sensor_reader = None

        if self.eye_sdk_running:
            if self.calibration_is_running:
                self._on_stop_calibration()
            self.eye_sdk.stop()
            self._on_set_sdk_running(False)

    def _send_next_sync_trigger(self):
        if not self.sync_running or self.sync_confirmation_pending:
            return
        if len(self.sync_results) >= self.sync_target_count:
            return
        if not self.sensor_reader:
            return

        sync_id = self.sync_next_id
        send_pc = self.sensor_reader.send_sync_start(sync_id)
        if send_pc is None:
            QtWidgets.QMessageBox.warning(self, "Warning", "Failed to send SYNC_START to sensor.")
            self._on_stop_sync()
            return

        self.sync_pending_triggers[sync_id] = {"sync_id": sync_id, "send_pc": send_pc}
        self.sync_next_id += 1
        self.lbl_sync_status.setText(
            f"Sync: {len(self.sync_results)}/{self.sync_target_count}, trigger {sync_id} sent"
        )

    def _process_sync_acks(self):
        if not self.sensor_reader:
            return
        while True:
            try:
                ack = self.sensor_reader.ack_queue.get_nowait()
            except queue.Empty:
                break

            pending = self.sync_pending_triggers.pop(ack["sync_id"], None)
            if pending is None:
                continue
            send_pc = pending["send_pc"]
            trigger_pc = (send_pc + ack["ack_pc"]) / 2.0
            self.sync_current_trigger = {
                "sync_id": ack["sync_id"],
                "send_pc": send_pc,
                "ack_pc": ack["ack_pc"],
                "trigger_pc": trigger_pc,
                "trigger_us": ack["trigger_us"],
                "first_left_frame_pc": None,
                "first_right_frame_pc": None,
                "eye_event": None,
            }

    def _check_sync_trigger_timeout(self):
        if self.sync_confirmation_pending or self.sync_current_trigger:
            return
        now = time.perf_counter()
        expired = [
            sync_id for sync_id, trigger in self.sync_pending_triggers.items()
            if now - trigger["send_pc"] > SYNC_EVENT_RETENTION_SECONDS
        ]
        for sync_id in expired:
            self.sync_pending_triggers.pop(sync_id, None)
            self.lbl_sync_status.setText(
                f"Sync: {len(self.sync_results)}/{self.sync_target_count}, trigger {sync_id} no ACK"
            )
            QtCore.QTimer.singleShot(500, self._send_next_sync_trigger)
            break

    def _pixmap_mean_brightness(self, pixmap):
        image = pixmap.toImage().convertToFormat(QtGui.QImage.Format_Grayscale8)
        width = image.width()
        height = image.height()
        if width <= 0 or height <= 0:
            return None
        ptr = image.bits()
        ptr.setsize(image.byteCount())
        arr = np.frombuffer(ptr, dtype=np.uint8).reshape((height, image.bytesPerLine()))
        return float(arr[:, :width].mean())

    def _process_sync_eye_sample(self, eye_name, timestamp, image):
        brightness = self._pixmap_mean_brightness(image)
        if brightness is None:
            return
        if self.sync_current_trigger and timestamp >= self.sync_current_trigger["send_pc"]:
            frame_key = f"first_{eye_name}_frame_pc"
            if self.sync_current_trigger.get(frame_key) is None:
                self.sync_current_trigger[frame_key] = timestamp
        if eye_name == "left":
            event = self.sync_left_detector.push(timestamp, brightness)
            if event:
                event["source"] = "left eye"
                if self.sync_current_trigger and timestamp >= self.sync_current_trigger["send_pc"]:
                    event["sync_id"] = self.sync_current_trigger["sync_id"]
                    self.sync_left_events.append(event)
        else:
            event = self.sync_right_detector.push(timestamp, brightness)
            if event:
                event["source"] = "right eye"
                if self.sync_current_trigger and timestamp >= self.sync_current_trigger["send_pc"]:
                    event["sync_id"] = self.sync_current_trigger["sync_id"]
                    self.sync_right_events.append(event)
        self._try_confirm_sync_event(timestamp)

    def _process_sync_ppg_sample(self, sample):
        if not self.sync_current_trigger:
            return
        if sample.get("sync_id") != self.sync_current_trigger["sync_id"]:
            return
        self.sync_current_trigger["last_ppg_sample_us"] = sample.get("sync_sample_us")

    def _try_confirm_sync_event(self, now):
        if self.sync_confirmation_pending or not self.sync_running:
            return
        if not self.sync_current_trigger:
            return
        sync_id = self.sync_current_trigger["sync_id"]
        eye_events = [
            event for event in list(self.sync_left_events) + list(self.sync_right_events)
            if event.get("sync_id") == sync_id
        ]
        if not eye_events:
            if now - self.sync_current_trigger["send_pc"] > SYNC_EVENT_RETENTION_SECONDS:
                self.lbl_sync_status.setText(
                    f"Sync: {len(self.sync_results)}/{self.sync_target_count}, trigger {sync_id} timed out"
                )
                self.sync_current_trigger = None
                QtCore.QTimer.singleShot(500, self._send_next_sync_trigger)
            return
        eye_event = max(eye_events, key=lambda item: item["slope_score"])
        if eye_event["time"] - self.sync_current_trigger["send_pc"] > SYNC_EVENT_RETENTION_SECONDS:
            self.lbl_sync_status.setText(
                f"Sync: {len(self.sync_results)}/{self.sync_target_count}, trigger {sync_id} timed out"
            )
            self.sync_current_trigger = None
            QtCore.QTimer.singleShot(500, self._send_next_sync_trigger)
            return

        self._discard_sync_events_for_id(sync_id)
        self.sync_confirmation_pending = True

        trigger_pc = self.sync_current_trigger["trigger_pc"]
        diff_ms = (trigger_pc - eye_event["time"]) * 1000.0
        first_left = self.sync_current_trigger.get("first_left_frame_pc")
        first_right = self.sync_current_trigger.get("first_right_frame_pc")
        message = (
            f"Detected flash candidate #{len(self.sync_results) + 1}/{self.sync_target_count}.\n\n"
            f"SYNC_START id: {sync_id}\n"
            f"Arduino trigger micros: {self.sync_current_trigger['trigger_us']}\n"
            f"Camera edge: {eye_event['source']} score {eye_event['slope_score']:.1f}\n"
            f"First left frame after trigger: {self._format_relative_ms(first_left, trigger_pc)}\n"
            f"First right frame after trigger: {self._format_relative_ms(first_right, trigger_pc)}\n"
            f"Sensor trigger - camera edge: {diff_ms:.2f} ms\n\n"
            "Approve this sync point?"
        )
        reply = QtWidgets.QMessageBox.question(
            self,
            "Confirm Sync Flash",
            message,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.Yes,
        )

        if reply == QtWidgets.QMessageBox.Yes:
            self.sync_results.append({
                "sync_id": sync_id,
                "trigger_us": self.sync_current_trigger["trigger_us"],
                "trigger_pc": trigger_pc,
                "eye_source": eye_event["source"],
                "eye_time": eye_event["time"],
                "diff_ms": diff_ms,
                "first_left_frame_delta_ms": None if first_left is None else (first_left - trigger_pc) * 1000.0,
                "first_right_frame_delta_ms": None if first_right is None else (first_right - trigger_pc) * 1000.0,
            })
            self._update_sync_status()
            if len(self.sync_results) >= self.sync_target_count:
                avg_ms = self._sync_average_ms()
                QtWidgets.QMessageBox.information(
                    self,
                    "Sync Complete",
                    f"Sync complete.\nAverage PPG - eye timestamp difference: {avg_ms:.2f} ms",
                )
                self._on_stop_sync()
            else:
                self.sync_current_trigger = None
                QtCore.QTimer.singleShot(500, self._send_next_sync_trigger)
        else:
            self._update_sync_status(rejected=True)
            self.sync_current_trigger = None
            QtCore.QTimer.singleShot(500, self._send_next_sync_trigger)

        self.sync_confirmation_pending = False

    def _update_sync_status(self, final=False, rejected=False):
        count = len(self.sync_results)
        if count:
            avg_ms = self._sync_average_ms()
            suffix = f", avg {avg_ms:.2f} ms"
        else:
            suffix = ""
        if final:
            prefix = "Sync: stopped"
        elif rejected:
            prefix = f"Sync: {count}/{self.sync_target_count}, rejected"
        else:
            prefix = f"Sync: {count}/{self.sync_target_count}"
        self.lbl_sync_status.setText(prefix + suffix)

    def _save_sync_results(self):
        if not self.sync_output_file or not self.sync_results:
            return
        avg_ms = self._sync_average_ms()
        with open(self.sync_output_file, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                "index",
                "sync_id",
                "arduino_trigger_us",
                "sensor_trigger_minus_camera_edge_ms",
                "camera_source",
                "first_left_frame_delta_ms",
                "first_right_frame_delta_ms",
            ])
            for index, result in enumerate(self.sync_results, start=1):
                writer.writerow([
                    index,
                    result["sync_id"],
                    result["trigger_us"],
                    f"{result['diff_ms']:.6f}",
                    result["eye_source"],
                    self._format_optional_float(result["first_left_frame_delta_ms"]),
                    self._format_optional_float(result["first_right_frame_delta_ms"]),
                ])
            writer.writerow(["average", f"{avg_ms:.6f}"])

    def _sync_average_ms(self):
        return float(np.mean([result["diff_ms"] for result in self.sync_results]))

    @staticmethod
    def _format_relative_ms(timestamp, reference):
        if timestamp is None:
            return "--"
        return f"{(timestamp - reference) * 1000.0:.2f} ms"

    @staticmethod
    def _format_optional_float(value):
        return "" if value is None else f"{value:.6f}"

    def _trim_sync_events(self, now):
        cutoff = now - SYNC_EVENT_RETENTION_SECONDS
        for events in (self.sync_ppg_events, self.sync_left_events, self.sync_right_events):
            while events and events[0]["time"] < cutoff:
                events.popleft()

    @staticmethod
    def _nearest_event(events, target_time):
        if not events:
            return None
        return min(events, key=lambda item: abs(item["time"] - target_time))

    @staticmethod
    def _discard_sync_event(events, target_event):
        try:
            events.remove(target_event)
        except ValueError:
            pass

    def _discard_sync_events_for_id(self, sync_id):
        self.sync_left_events = deque(event for event in self.sync_left_events if event.get("sync_id") != sync_id)
        self.sync_right_events = deque(event for event in self.sync_right_events if event.get("sync_id") != sync_id)

    def _connect_signals(self):
        self.btn_start.clicked.connect(self._on_start_all)
        self.btn_stop.clicked.connect(self._on_stop_all)
        self.btn_sync_start.clicked.connect(self._on_start_sync)
        self.btn_sync_stop.clicked.connect(self._on_stop_sync)
        self.btn_cal_start.clicked.connect(self._on_start_calibration)
        self.btn_cal_stop.clicked.connect(self._on_stop_calibration)
        self.btn_refresh_profiles.clicked.connect(self._refresh_calibration_profiles)
        self.btn_load_profile.clicked.connect(self._load_profile_from_button)
        self.btn_save_profile.clicked.connect(self._save_current_calibration_profile)
        self.combo_calibration_profile.editTextChanged.connect(self._update_calibration_profile_controls)

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
    
    def _display_left_eye_image(self, image):
        self.labelLeftEye.setPixmap(image)
        if self.sync_running:
            self._process_sync_eye_sample("left", time.perf_counter(), image)

    def _display_right_eye_image(self, image):
        self.labelRightEye.setPixmap(image)
        if self.sync_running:
            self._process_sync_eye_sample("right", time.perf_counter(), image)

    def _display_pupil_data(self, lx, ly, rx, ry):
        self.lbl_pupil_x.setText(f"{lx:.5f}"); self.lbl_pupil_y.setText(f"{rx:.5f}")

    def _display_gaze_data(self, x, y):
        self.lbl_gaze_x.setText(f"{x:.5f}"); self.lbl_gaze_y.setText(f"{y:.5f}")
        self.cur_gaze_x = x + self.scene_width / 2
        self.cur_gaze_y = y + self.scene_height / 2

    def _on_set_sdk_running(self, enabled):
        self.eye_sdk_running = enabled
        self.btn_cal_start.setEnabled(enabled and not self.calibration_is_running)
        self.btn_cal_stop.setEnabled(enabled and self.calibration_is_running)
        self._update_calibration_profile_controls()
        if not enabled:
            self.calibration_is_running = False
            self.labelSceneImage.setPixmap(QPixmap())
            self.labelLeftEye.setPixmap(QPixmap())
            self.labelRightEye.setPixmap(QPixmap())

    def _on_set_calibration_finish(self, eye, index, error):
        if not self.calibration_is_running:
            return
        if index < 1 or index > len(self.finish_points):
            return
        if eye < 0 or eye >= len(self.finish_points[index - 1]):
            return
        self.finish_points[index - 1][eye] = error
        n = self.current_points
        if n == 1:
            if 1 != self.finish_points[0][0] and 1 != self.finish_points[0][1]:
                self._on_stop_calibration()
        elif n == 3:
            if (1 != self.finish_points[0][0] and 1 != self.finish_points[0][1] and
                1 != self.finish_points[1][0] and 1 != self.finish_points[1][1] and
                1 != self.finish_points[2][0] and 1 != self.finish_points[2][1]):
                self._on_stop_calibration()
        elif n == 5:
            if all(1 != self.finish_points[i][e] for i in range(5) for e in range(2)):
                self._on_stop_calibration()
        elif n == 9:
            if all(1 != self.finish_points[i][e] for i in range(9) for e in range(2)):
                self._on_stop_calibration()

    def _on_scene_clicked(self, norm_x, norm_y):
        if not self.eye_sdk_running or not self.calibration_is_running:
            return
        px = norm_x * self.scene_width - (self.scene_width / 2)
        py = norm_y * self.scene_height - (self.scene_height / 2)
        self.eye_sdk.set_current_point(px, py)

    def _on_start_calibration(self):
        if not self.eye_sdk_running or self.calibration_is_running:
            return
        self.current_points = int(self.combo_points.currentText())
        for i in range(len(self.finish_points)): self.finish_points[i] = [1, 1]
        self.calibration_is_running = True
        self.eye_sdk.start_calibration(self.current_points)
        self.btn_cal_start.setEnabled(False)
        self.btn_cal_stop.setEnabled(True)

    def _on_stop_calibration(self):
        if not self.calibration_is_running:
            return
        self.eye_sdk.stop_calibration()
        self.calibration_is_running = False
        self.btn_cal_start.setEnabled(self.eye_sdk_running)
        self.btn_cal_stop.setEnabled(False)

    # ---- Actions ----
    def _read_pwd(self):
        cf = configparser.ConfigParser()
        cf.read(os.path.join(self.eye_sdk_config_path, "config.ini"))
        return cf.get("softdog", "pwd", fallback="").encode("utf-8")

    def _on_start_all(self):
        if self.sync_running:
            QtWidgets.QMessageBox.warning(self, "Warning", "Stop sync mode before starting normal logging.")
            return
        self.system_running = True
        self.btn_start.setEnabled(False)
        self._set_sync_controls_running(False)
        self._populate_sensor_ports()
        self.sensor_reader = None
        self.eye_sdk_running = False
        
        # Prepare Logging Dirs
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Start Sensor
        selected_port = self.combo_port.currentData()
        if not selected_port:
            QtWidgets.QMessageBox.warning(self, "Warning", f"No '{TARGET_SERIAL_DESCRIPTION}' serial port found.")
        else:
            self.sensor_reader = SerialPacketReader(selected_port, int(self.combo_baud.currentText()))
        if self.sensor_reader and self.sensor_reader.open():
            time.sleep(SERIAL_STARTUP_SETTLE_SECONDS)
            self.sensor_reader.serial_port.reset_input_buffer()
            self.sensor_reader.serial_port.reset_output_buffer()
            self.ts_offset = sync_timestamps(self.sensor_reader.serial_port, rounds=20)
            self.sensor_csv_writer = SensorCSVWriterThread(os.path.join(self.log_dir, f"sensor_{ts_str}.csv"))
            self.sensor_csv_writer.start()
            if self.sensor_reader.start():
                time.sleep(0.3)
                self.sensor_reader.send_command("s\n")
            else:
                self.sensor_csv_writer.stop()
                self.sensor_csv_writer = None
                self.sensor_reader.stop()
                self.sensor_reader = None
                QtWidgets.QMessageBox.warning(self, "Warning", "Sensor reader thread failed to start.")
        elif selected_port:
            QtWidgets.QMessageBox.warning(self, "Warning", "Sensor failed to connect.")

        # Start Eye Tracker
        pwd = self._read_pwd()
        if self.eye_sdk.connect_softdog(pwd) == 0:
            env = self.combo_env.currentData()
            res = self.combo_res.currentData()
            self.scene_width, self.scene_height = (1280, 960) if res == 201 else (1280, 720) if res == 202 else (800, 600) if res == 203 else (1920, 1080)
            
            if self.eye_sdk.start(env, res, self.scene_width, self.scene_height) == 0:
                self._on_set_sdk_running(True)
                self._load_selected_calibration_profile(show_message=False)
                self.eye_csv_writer = EyeCSVWriterThread(os.path.join(self.log_dir, f"eye_{ts_str}.csv"))
                self.eye_csv_writer.start()
            else:
                self._on_set_sdk_running(False)
                QtWidgets.QMessageBox.warning(self, "Warning", "Eye tracker start failed.")
        else:
            self._on_set_sdk_running(False)
            QtWidgets.QMessageBox.warning(self, "Warning", "Eye tracker dog connect failed.")

        self.btn_stop.setEnabled(True)
        self.timer.start(PLOT_UPDATE_INTERVAL_MS)

    def _on_stop_all(self):
        self.system_running = False
        self.timer.stop()
        self.btn_stop.setEnabled(False)
        self.btn_start.setEnabled(True)
        self._set_sync_controls_running(False)
        self.btn_cal_start.setEnabled(False)
        self.btn_cal_stop.setEnabled(False)
        self._update_calibration_profile_controls()

        # Stop Sensor
        if self.sensor_reader:
            try:
                self.sensor_reader.send_command("e\n")
                time.sleep(0.3)
            except Exception:
                pass
            self.sensor_reader.stop()
            self.sensor_reader = None
        if self.sensor_csv_writer:
            self.sensor_csv_writer.stop()
            self.sensor_csv_writer = None
        
        # Stop Eye
        if self.calibration_is_running: self._on_stop_calibration()
        self.eye_sdk.stop()
        if self.eye_csv_writer: self.eye_csv_writer.stop()
        self.eye_sdk_running = False
        self.eye_csv_writer = None
        
        print("\n===== Logging Session Summary =====")
        print(f"Sensor Total Packets:    {self.sensor_rate_tracker.total_packets}")
        if self.sensor_csv_writer:
            print(f"Sensor Log Output:       {self.sensor_csv_writer.output_file}")
        print(f"Eye Tracker Data Frames: {self.eye_packet_count}")
        if self.eye_csv_writer:
            print(f"Eye Tracker Log Output:  {self.eye_csv_writer.output_file}")
        print("===================================\n")

    # ---- Timer Update ----
    def _on_timer(self):
        now_ms = time.perf_counter() * 1000
        if self.sync_running:
            self._process_sync_acks()
            self._check_sync_trigger_timeout()

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
                if self.sync_running:
                    self._process_sync_ppg_sample(p)
                
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
        if self.sync_running: self._on_stop_sync()
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
