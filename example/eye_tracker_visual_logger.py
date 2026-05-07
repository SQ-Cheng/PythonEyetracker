#!/usr/bin/env python3
"""
Eye tracker visualization and CSV logger.

Adapted from the official aSeeGlassesPlus SDK Python3 sample.
Uses PyQt5 with direct signal emission from SDK callbacks for reliable
scene camera display, eye image display, and gaze overlay.
"""

import argparse
import configparser
import csv
import os
import queue
import sys
import time
from collections import deque
from datetime import datetime

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen, QPixmap, QFont
from PyQt5.QtCore import Qt, QRectF

from sdk_types import PY_7I_ENVIRONMENT, PY_7I_RESOLUTION
from sdk_wrapper import wrapper
import pyqtgraph as pg


DISPLAY_WINDOW_SECONDS = 10.0
PLOT_UPDATE_INTERVAL_MS = 33
METRIC_UPDATE_INTERVAL_MS = 250
RATE_WINDOW_SECONDS = 3.0

COLUMN_NAMES = [
    "pc_timestamp",
    "device_timestamp",
    "gaze_x",
    "gaze_y",
    "gaze_z",
    "left_pupil_x",
    "left_pupil_y",
    "right_pupil_x",
    "right_pupil_y",
    "left_pupil_diameter_mm",
    "right_pupil_diameter_mm",
    "left_openness",
    "right_openness",
    "left_blink",
    "right_blink",
]


class SceneImageLabel(QtWidgets.QLabel):
    """Clickable QLabel for scene camera image (matching official sample)."""

    button_clicked_signal = QtCore.pyqtSignal(int, int)

    def __init__(self, parent=None):
        super(SceneImageLabel, self).__init__(parent)

    def mousePressEvent(self, ev):
        if ev.buttons() == Qt.LeftButton:
            self.button_clicked_signal.emit(ev.x(), ev.y())

    def connect_customized_slot(self, slot_func):
        self.button_clicked_signal.connect(slot_func)


class PacketRateTracker:
    def __init__(self, window_seconds=RATE_WINDOW_SECONDS):
        self.window_seconds = window_seconds
        self.timestamps = deque()

    def push(self, timestamp):
        self.timestamps.append(timestamp)
        cutoff = timestamp - self.window_seconds
        while self.timestamps and self.timestamps[0] < cutoff:
            self.timestamps.popleft()

    def current_rate(self):
        if len(self.timestamps) < 2:
            return 0.0
        duration = self.timestamps[-1] - self.timestamps[0]
        if duration <= 0:
            return 0.0
        return (len(self.timestamps) - 1) / duration


class RingSeries:
    def __init__(self, sample_rate_hz, window_seconds):
        self.sample_rate_hz = sample_rate_hz
        self.window_seconds = window_seconds
        self.size = max(128, int(round(sample_rate_hz * window_seconds)))
        self.values = [0.0] * self.size
        self.write_index = 0
        self.count = 0

    def append(self, values):
        if not values:
            return
        if len(values) >= self.size:
            values = values[-self.size:]
        for value in values:
            self.values[self.write_index] = float(value)
            self.write_index = (self.write_index + 1) % self.size
            self.count = min(self.size, self.count + 1)

    def ordered(self):
        if self.count == 0:
            return []
        if self.count < self.size:
            return self.values[:self.count]
        return self.values[self.write_index:] + self.values[:self.write_index]

    def x_axis(self, count):
        if count <= 1:
            return [0.0]
        step = self.window_seconds / (count - 1)
        return [(-self.window_seconds + i * step) for i in range(count)]


class CSVWriterThread:
    def __init__(self, output_file):
        self.output_file = output_file
        self.queue = queue.Queue()
        self._thread = QtCore.QThread()
        self._worker = _CSVWriterWorker(self.queue, self.output_file)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)

    def start(self):
        self._thread.start()

    def stop(self):
        self._worker.stop()
        self._thread.quit()
        self._thread.wait(2000)

    def push(self, sample):
        self.queue.put(sample)


class _CSVWriterWorker(QtCore.QObject):
    def __init__(self, queue_obj, output_file):
        super().__init__()
        self.queue = queue_obj
        self.output_file = output_file
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        with open(self.output_file, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(COLUMN_NAMES)
            while not self._stop or not self.queue.empty():
                try:
                    sample = self.queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                row = [
                    f"{sample['pc_timestamp']:.6f}",
                    str(int(sample["device_timestamp"])),
                    f"{sample['gaze_x']:.6f}",
                    f"{sample['gaze_y']:.6f}",
                    f"{sample['gaze_z']:.6f}",
                    f"{sample['left_pupil_x']:.6f}",
                    f"{sample['left_pupil_y']:.6f}",
                    f"{sample['right_pupil_x']:.6f}",
                    f"{sample['right_pupil_y']:.6f}",
                    f"{sample['left_pupil_diameter_mm']:.6f}",
                    f"{sample['right_pupil_diameter_mm']:.6f}",
                    f"{sample['left_openness']:.6f}",
                    f"{sample['right_openness']:.6f}",
                    str(int(sample["left_blink"])),
                    str(int(sample["right_blink"])),
                ]
                writer.writerow(row)


class EyeTrackerMonitorWindow(QtWidgets.QMainWindow):
    # Signals matching official sample pattern
    set_sdk_running_signal = QtCore.pyqtSignal(bool)
    set_pupil_center_signal = QtCore.pyqtSignal(float, float, float, float)
    set_gaze_signal = QtCore.pyqtSignal(float, float)
    set_scene_image_signal = QtCore.pyqtSignal(QPixmap)
    set_left_eye_image_signal = QtCore.pyqtSignal(QPixmap)
    set_right_eye_image_signal = QtCore.pyqtSignal(QPixmap)
    set_calibration_finish_signal = QtCore.pyqtSignal(int, int, int)

    def __init__(self, sdk_root, sample_rate_hz, window_seconds):
        super().__init__()
        self.sdk_root = sdk_root
        self.sample_rate_hz = sample_rate_hz
        self.window_seconds = window_seconds

        self.sdk = wrapper()
        self.sdk_config_path = os.path.join(self.sdk_root, "bin", "config")
        self.sdk.load_library(self.sdk_config_path)
        self.sdk.set_ui_handle(self)

        self.rate_tracker = PacketRateTracker()
        self.gaze_x_series = RingSeries(sample_rate_hz, window_seconds)
        self.gaze_y_series = RingSeries(sample_rate_hz, window_seconds)
        self.left_pupil_series = RingSeries(sample_rate_hz, window_seconds)
        self.right_pupil_series = RingSeries(sample_rate_hz, window_seconds)

        self.packet_count = 0
        self.last_plot_update = 0.0
        self.last_metric_update = 0.0
        self.latest_sample = None

        self.scene_width = 1280
        self.scene_height = 720
        self.sdk_running = False

        # Gaze position for overlay (in scene image coordinates)
        self.cur_gaze_x = 0
        self.cur_gaze_y = 0

        # Calibration state (matching official sample pattern)
        self.calibration_is_running = False
        self.current_points = 0
        self.finish_points = [[1, 1], [1, 1], [1, 1], [1, 1], [1, 1],
                              [1, 1], [1, 1], [1, 1], [1, 1]]

        self._build_ui()
        self._connect_signals()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer.setInterval(10)

        self.csv_writer = None

    def _build_ui(self):
        self.setWindowTitle("Eye Tracker Monitor")
        self.resize(1450, 734)

        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)

        # === Left sidebar (narrow, matching official sample layout) ===
        # Environment group
        self.groupBox_env = QtWidgets.QGroupBox("Environment")
        self.groupBox_env.setGeometry(3, -1, 120, 41)
        self.comboBoxEnvironment = QtWidgets.QComboBox(self.groupBox_env)
        self.comboBoxEnvironment.setGeometry(10, 13, 100, 22)
        self.comboBoxEnvironment.addItem("indoor", 301)
        self.comboBoxEnvironment.addItem("outdoor", 302)
        self.comboBoxEnvironment.addItem("darkness", 303)
        self.comboBoxEnvironment.setCurrentIndex(0)

        # Resolution group
        self.groupBox_res = QtWidgets.QGroupBox("Resolution")
        self.groupBox_res.setGeometry(3, 40, 121, 41)
        self.comboBoxResolution = QtWidgets.QComboBox(self.groupBox_res)
        self.comboBoxResolution.setGeometry(10, 12, 100, 22)
        self.comboBoxResolution.addItem("1280 * 960", 201)
        self.comboBoxResolution.addItem("1280 * 720", 202)
        self.comboBoxResolution.addItem(" 800 * 600", 203)
        self.comboBoxResolution.addItem("1920 * 1080", 204)
        self.comboBoxResolution.setCurrentIndex(1)

        # Start / Stop buttons
        self.pushButtonStart = QtWidgets.QPushButton("Start")
        self.pushButtonStop = QtWidgets.QPushButton("Stop")
        self.pushButtonStop.setEnabled(False)

        # Calibration controls
        self.label_points = QtWidgets.QLabel("Points")
        self.comboBoxPoints = QtWidgets.QComboBox()
        self.comboBoxPoints.addItem("1", 1)
        self.comboBoxPoints.addItem("3", 3)
        self.comboBoxPoints.addItem("5", 5)
        self.comboBoxPoints.addItem("9", 9)
        self.comboBoxPoints.setCurrentIndex(1)

        self.pushButtonStartCalibration = QtWidgets.QPushButton("Start Calibration")
        self.pushButtonStartCalibration.setEnabled(False)
        self.pushButtonCancelCalibration = QtWidgets.QPushButton("Stop Calibration")

        # Left Pupil group
        self.groupBox_left_pupil = QtWidgets.QGroupBox("Left Pupil")
        left_pupil_layout = QtWidgets.QGridLayout(self.groupBox_left_pupil)
        self.label_lp_x_title = QtWidgets.QLabel("X")
        self.label_lp_y_title = QtWidgets.QLabel("Y")
        self.labelLeftPupilCenterX = QtWidgets.QLabel("")
        self.labelLeftPupilCenterX.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.labelLeftPupilCenterX.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.labelLeftPupilCenterY = QtWidgets.QLabel("")
        self.labelLeftPupilCenterY.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.labelLeftPupilCenterY.setFrameShadow(QtWidgets.QFrame.Sunken)
        left_pupil_layout.addWidget(self.label_lp_x_title, 0, 0)
        left_pupil_layout.addWidget(self.labelLeftPupilCenterX, 0, 1)
        left_pupil_layout.addWidget(self.label_lp_y_title, 1, 0)
        left_pupil_layout.addWidget(self.labelLeftPupilCenterY, 1, 1)

        # Right Pupil group
        self.groupBox_right_pupil = QtWidgets.QGroupBox("Right Pupil")
        right_pupil_layout = QtWidgets.QGridLayout(self.groupBox_right_pupil)
        self.label_rp_x_title = QtWidgets.QLabel("X")
        self.label_rp_y_title = QtWidgets.QLabel("Y")
        self.labelRightPupilCenterX = QtWidgets.QLabel("")
        self.labelRightPupilCenterX.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.labelRightPupilCenterX.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.labelRightPupilCenterY = QtWidgets.QLabel("")
        self.labelRightPupilCenterY.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.labelRightPupilCenterY.setFrameShadow(QtWidgets.QFrame.Sunken)
        right_pupil_layout.addWidget(self.label_rp_x_title, 0, 0)
        right_pupil_layout.addWidget(self.labelRightPupilCenterX, 0, 1)
        right_pupil_layout.addWidget(self.label_rp_y_title, 1, 0)
        right_pupil_layout.addWidget(self.labelRightPupilCenterY, 1, 1)

        # Recommend Gaze group
        self.groupBox_gaze = QtWidgets.QGroupBox("Recommend Gaze")
        gaze_layout = QtWidgets.QGridLayout(self.groupBox_gaze)
        self.label_gaze_x_title = QtWidgets.QLabel("X")
        self.label_gaze_y_title = QtWidgets.QLabel("Y")
        self.labelGazeX = QtWidgets.QLabel("")
        self.labelGazeX.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.labelGazeX.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.labelGazeY = QtWidgets.QLabel("")
        self.labelGazeY.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.labelGazeY.setFrameShadow(QtWidgets.QFrame.Sunken)
        gaze_layout.addWidget(self.label_gaze_x_title, 0, 0)
        gaze_layout.addWidget(self.labelGazeX, 0, 1)
        gaze_layout.addWidget(self.label_gaze_y_title, 1, 0)
        gaze_layout.addWidget(self.labelGazeY, 1, 1)

        # Metrics (rate, count, log)
        self.rate_label = QtWidgets.QLabel("Rate: -- Hz")
        self.count_label = QtWidgets.QLabel("Samples: 0")
        self.log_label = QtWidgets.QLabel("Log: --")

        # === Main content area ===
        # Scene image label (clickable, matching official sample)
        self.labelSceneImage = SceneImageLabel()
        self.labelSceneImage.connect_customized_slot(self._on_scene_image_area_clicked)
        self.labelSceneImage.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.labelSceneImage.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.labelSceneImage.setText("scene image area")
        self.labelSceneImage.setMinimumSize(640, 360)

        # Eye images
        self.labelLeftEyeImage = QtWidgets.QLabel("left eye image")
        self.labelLeftEyeImage.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.labelLeftEyeImage.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.labelLeftEyeImage.setFixedSize(160, 120)

        self.labelRightEyeImage = QtWidgets.QLabel("right eye image")
        self.labelRightEyeImage.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.labelRightEyeImage.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.labelRightEyeImage.setFixedSize(160, 120)

        # pyqtgraph plots (secondary, below scene image)
        pg.setConfigOptions(antialias=False, useOpenGL=False)
        self.gaze_plot = pg.PlotWidget(title="Gaze X / Y")
        self.gaze_plot.showGrid(x=True, y=True, alpha=0.2)
        self.gaze_plot.setLabel("bottom", "Time (s)")
        self.gaze_plot.setLabel("left", "Pixels")
        self.gaze_x_curve = self.gaze_plot.plot(pen=pg.mkPen("#4ED1A6", width=2), name="Gaze X")
        self.gaze_y_curve = self.gaze_plot.plot(pen=pg.mkPen("#FF7F50", width=2), name="Gaze Y")

        self.pupil_plot = pg.PlotWidget(title="Pupil Diameter (mm)")
        self.pupil_plot.showGrid(x=True, y=True, alpha=0.2)
        self.pupil_plot.setLabel("bottom", "Time (s)")
        self.pupil_plot.setLabel("left", "Diameter (mm)")
        self.left_pupil_curve = self.pupil_plot.plot(pen=pg.mkPen("#9AD1FF", width=2), name="Left")
        self.right_pupil_curve = self.pupil_plot.plot(pen=pg.mkPen("#FFD166", width=2), name="Right")

        # === Layout assembly ===
        # Use a horizontal layout: left sidebar | right content
        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        # Left sidebar
        sidebar = QtWidgets.QVBoxLayout()
        sidebar.setSpacing(4)
        sidebar.addWidget(self.groupBox_env)
        sidebar.addWidget(self.groupBox_res)
        sidebar.addWidget(self.pushButtonStart)
        sidebar.addWidget(self.pushButtonStop)

        points_row = QtWidgets.QHBoxLayout()
        points_row.addWidget(self.label_points)
        points_row.addWidget(self.comboBoxPoints)
        sidebar.addLayout(points_row)

        sidebar.addWidget(self.pushButtonStartCalibration)
        sidebar.addWidget(self.pushButtonCancelCalibration)
        sidebar.addWidget(self.groupBox_left_pupil)
        sidebar.addWidget(self.groupBox_right_pupil)
        sidebar.addWidget(self.groupBox_gaze)

        sidebar.addWidget(self.rate_label)
        sidebar.addWidget(self.count_label)
        sidebar.addWidget(self.log_label)

        sidebar.addStretch()
        main_layout.addLayout(sidebar)

        # Right content: scene image + eye images on top, plots on bottom
        right_layout = QtWidgets.QVBoxLayout()
        right_layout.setSpacing(4)

        # Top row: scene image with eye images below-left
        scene_area = QtWidgets.QVBoxLayout()
        scene_area.setSpacing(2)
        scene_area.addWidget(self.labelSceneImage, stretch=1)

        eye_row = QtWidgets.QHBoxLayout()
        eye_row.setSpacing(2)
        eye_row.addWidget(self.labelLeftEyeImage)
        eye_row.addWidget(self.labelRightEyeImage)
        eye_row.addStretch()
        scene_area.addLayout(eye_row)

        right_layout.addLayout(scene_area, stretch=3)

        # Bottom: plots side by side
        plots_row = QtWidgets.QHBoxLayout()
        plots_row.setSpacing(4)
        plots_row.addWidget(self.gaze_plot)
        plots_row.addWidget(self.pupil_plot)
        right_layout.addLayout(plots_row, stretch=1)

        main_layout.addLayout(right_layout, stretch=1)

    def _connect_signals(self):
        # Button signals
        self.pushButtonStart.clicked.connect(self._on_start)
        self.pushButtonStop.clicked.connect(self._on_stop)
        self.pushButtonStartCalibration.clicked.connect(self._on_start_calibration)
        self.pushButtonCancelCalibration.clicked.connect(self._on_stop_calibration)

        # SDK callback signals (matching official sample pattern)
        self.set_sdk_running_signal.connect(self._on_set_sdk_running)
        self.set_pupil_center_signal.connect(self._display_pupil_data)
        self.set_gaze_signal.connect(self._display_gaze_data)
        self.set_scene_image_signal.connect(self._display_scene_image)
        self.set_left_eye_image_signal.connect(self._display_left_eye_image)
        self.set_right_eye_image_signal.connect(self._display_right_eye_image)
        self.set_calibration_finish_signal.connect(self._on_set_calibration_finish)

    # ---- Signal slots (matching official sample) ----

    def _display_scene_image(self, image):
        """Draw gaze overlay on scene image, then display (matching official sample)."""
        painter = QPainter(image)
        painter.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        color = QColor()
        color.setGreen(255)
        painter.setBrush(QBrush(color))
        diameter = 30
        rect = QRectF(self.cur_gaze_x - diameter / 2, self.cur_gaze_y - diameter / 2, diameter, diameter)
        painter.drawEllipse(rect)
        self.labelSceneImage.setPixmap(image)

    def _display_left_eye_image(self, image):
        self.labelLeftEyeImage.setPixmap(image)

    def _display_right_eye_image(self, image):
        self.labelRightEyeImage.setPixmap(image)

    def _display_pupil_data(self, left_x, left_y, right_x, right_y):
        self.labelLeftPupilCenterX.setText(str(left_x))
        self.labelLeftPupilCenterY.setText(str(left_y))
        self.labelRightPupilCenterX.setText(str(right_x))
        self.labelRightPupilCenterY.setText(str(right_y))

    def _display_gaze_data(self, x, y):
        self.labelGazeX.setText(str(x))
        self.labelGazeY.setText(str(y))
        # Convert from center-origin to top-left-origin for drawing (matching official sample)
        self.cur_gaze_x = (x + self.scene_width / 2)
        self.cur_gaze_y = (y + self.scene_height / 2)

    def _on_set_sdk_running(self, enabled):
        self.sdk_running = enabled
        self.pushButtonStart.setEnabled(not enabled)
        self.pushButtonStop.setEnabled(enabled)
        self.pushButtonStartCalibration.setEnabled(enabled)
        if not enabled:
            self.labelSceneImage.setPixmap(QPixmap())
            self.labelLeftEyeImage.setPixmap(QPixmap())
            self.labelRightEyeImage.setPixmap(QPixmap())

    def _on_set_calibration_finish(self, eye, index, error):
        """Handle calibration point finish (matching official sample logic)."""
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
            if all(1 != self.finish_points[i][e]
                   for i in range(5) for e in range(2)):
                self._on_stop_calibration()
        elif n == 9:
            if all(1 != self.finish_points[i][e]
                   for i in range(9) for e in range(2)):
                self._on_stop_calibration()

    # ---- Scene image click (calibration point selection) ----

    def _on_scene_image_area_clicked(self, x, y):
        """Convert click position to SDK center-origin coordinates (matching official sample)."""
        point_x = float(x) - float(self.scene_width / 2)
        point_y = float(y) - float(self.scene_height / 2)
        print("point:%f %f" % (point_x, point_y))
        self.sdk.set_current_point(point_x, point_y)

    # ---- SDK start / stop ----

    def _on_start(self):
        pwd = self._read_pwd()
        if not pwd:
            QtWidgets.QMessageBox.warning(self, "warning", "Please check that 'pwd' is correct in config.ini file!")
            return

        ret = self.sdk.connect_softdog(pwd)
        if ret != 0:
            QtWidgets.QMessageBox.warning(self, "warning", "Please check that 'pwd' is correct in config.ini file!")
            return

        environment = self.comboBoxEnvironment.currentData()
        resolution = self.comboBoxResolution.currentData()
        if PY_7I_RESOLUTION.P1280_960.value == resolution:
            self.scene_width, self.scene_height = 1280, 960
        elif PY_7I_RESOLUTION.P1280_720.value == resolution:
            self.scene_width, self.scene_height = 1280, 720
        elif PY_7I_RESOLUTION.P800_600.value == resolution:
            self.scene_width, self.scene_height = 800, 600
        elif PY_7I_RESOLUTION.P1920_1080.value == resolution:
            self.scene_width, self.scene_height = 1920, 1080

        ret = self.sdk.start(environment, resolution, self.scene_width, self.scene_height)
        if ret == 0:
            self.sdk_running = True
            self.pushButtonStop.setEnabled(True)
            self.pushButtonStart.setEnabled(False)
            self.pushButtonStartCalibration.setEnabled(True)

            # Start CSV logging
            log_dir = os.path.join(os.path.dirname(os.getcwd()), "log")
            os.makedirs(log_dir, exist_ok=True)
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_file = os.path.join(log_dir, f"eye_tracker_{timestamp_str}.csv")
            self.csv_writer = CSVWriterThread(output_file)
            self.csv_writer.start()
            self.log_label.setText(f"Log: {output_file}")

            self.timer.start()
        else:
            self.sdk_running = False
            self.pushButtonStartCalibration.setEnabled(False)

    def _on_stop(self):
        self.sdk.stop()
        self.pushButtonStop.setEnabled(False)
        self.pushButtonStart.setEnabled(True)
        self.pushButtonStartCalibration.setEnabled(False)
        self.labelSceneImage.setPixmap(QPixmap())
        self.labelLeftEyeImage.setPixmap(QPixmap())
        self.labelRightEyeImage.setPixmap(QPixmap())
        self.sdk_running = False
        self.timer.stop()
        if self.csv_writer:
            self.csv_writer.stop()
            self.csv_writer = None

    # ---- Calibration (click-on-scene pattern, matching official sample) ----

    def _on_start_calibration(self):
        self.current_points = self.comboBoxPoints.currentData()
        self._init_finish_points()
        self.calibration_is_running = True
        self.sdk.start_calibration(self.current_points)
        self.pushButtonStartCalibration.setEnabled(False)

    def _on_stop_calibration(self):
        self.sdk.stop_calibration()
        self.calibration_is_running = False
        self.pushButtonStartCalibration.setEnabled(self.sdk_running)

    def _init_finish_points(self):
        for i in range(len(self.finish_points)):
            self.finish_points[i][0] = 1
            self.finish_points[i][1] = 1

    # ---- Timer / data for plots and CSV ----

    def _on_timer(self):
        now = time.perf_counter()
        drained = 0

        while True:
            try:
                sample = self.sdk.data_queue.get_nowait()
            except queue.Empty:
                break
            drained += 1
            self.packet_count += 1
            self.latest_sample = sample
            self.rate_tracker.push(sample["perf_timestamp"])

            self.gaze_x_series.append([sample["gaze_x"]])
            self.gaze_y_series.append([sample["gaze_y"]])
            self.left_pupil_series.append([sample["left_pupil_diameter_mm"]])
            self.right_pupil_series.append([sample["right_pupil_diameter_mm"]])

            if self.csv_writer:
                self.csv_writer.push(sample)

        if drained == 0:
            return

        if (now - self.last_plot_update) * 1000 >= PLOT_UPDATE_INTERVAL_MS:
            self._update_plots()
            self.last_plot_update = now

        if (now - self.last_metric_update) * 1000 >= METRIC_UPDATE_INTERVAL_MS:
            self._update_metrics()
            self.last_metric_update = now

    def _update_plots(self):
        gaze_x = self.gaze_x_series.ordered()
        gaze_y = self.gaze_y_series.ordered()
        x_axis = self.gaze_x_series.x_axis(len(gaze_x))
        self.gaze_x_curve.setData(x_axis, gaze_x)
        self.gaze_y_curve.setData(x_axis, gaze_y)

        left_pupil = self.left_pupil_series.ordered()
        right_pupil = self.right_pupil_series.ordered()
        x_axis_pupil = self.left_pupil_series.x_axis(len(left_pupil))
        self.left_pupil_curve.setData(x_axis_pupil, left_pupil)
        self.right_pupil_curve.setData(x_axis_pupil, right_pupil)

    def _update_metrics(self):
        rate = self.rate_tracker.current_rate()
        self.rate_label.setText(f"Rate: {rate:.1f} Hz")
        self.count_label.setText(f"Samples: {self.packet_count}")

    def _read_pwd(self):
        config_path = os.path.join(self.sdk_config_path, "config.ini")
        cf = configparser.ConfigParser()
        cf.read(config_path)
        pwd = cf.get("softdog", "pwd", fallback="")
        return pwd.encode("utf-8")

    def closeEvent(self, event):
        reply = QtWidgets.QMessageBox.question(
            self, 'warning', "Are you sure exit?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if reply == QtWidgets.QMessageBox.Yes:
            if self.calibration_is_running:
                self.sdk.stop_calibration()
            if self.sdk_running:
                self.sdk.stop()
                time.sleep(3)
            if self.csv_writer:
                self.csv_writer.stop()
                self.csv_writer = None
            event.accept()
            self.close()
        else:
            event.ignore()


def parse_args():
    parser = argparse.ArgumentParser(description="Eye tracker visual logger")
    parser.add_argument("--sdk-root", default="E:/7invensun/aSeeGlassesPlusUserSDK")
    parser.add_argument("--sample-rate", type=float, default=120.0)
    parser.add_argument("--window-seconds", type=float, default=DISPLAY_WINDOW_SECONDS)
    return parser.parse_args()


def main():
    args = parse_args()
    app = QtWidgets.QApplication(sys.argv)
    window = EyeTrackerMonitorWindow(args.sdk_root, args.sample_rate, args.window_seconds)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()