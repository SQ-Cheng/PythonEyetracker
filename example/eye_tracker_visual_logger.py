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


# ===================== Theme =====================

THEME_DARK = {
    'bg': '#12161C',
    'fg': '#EAF0F7',
    'grid_alpha': 0.2,
    'label_color': '#EAF0F7',
    'group_border': '#3A3F4B',
    'group_title': '#EAF0F7',
    'value_bg': '#1E2330',
    'value_fg': '#4ED1A6',
    'btn_bg': '#2A3040',
    'btn_hover': '#3A4050',
    'btn_text': '#EAF0F7',
    'btn_disabled_bg': '#1A1F2A',
    'btn_disabled_text': '#5A6070',
    'separator': '#3A3F4B',
}

THEME_LIGHT = {
    'bg': '#FFFFFF',
    'fg': '#1A1A2E',
    'grid_alpha': 0.3,
    'label_color': '#1A1A2E',
    'group_border': '#D0D5DD',
    'group_title': '#1A1A2E',
    'value_bg': '#F5F7FA',
    'value_fg': '#0D7C66',
    'btn_bg': '#E8ECF0',
    'btn_hover': '#D0D5DD',
    'btn_text': '#1A1A2E',
    'btn_disabled_bg': '#F0F2F5',
    'btn_disabled_text': '#A0A8B4',
    'separator': '#D0D5DD',
}


def detect_dark_theme():
    """Detect if the system is using a dark theme by checking palette luminance."""
    palette = QtWidgets.QApplication.instance().palette()
    window_color = palette.color(QtGui.QPalette.Window)
    luminance = (0.299 * window_color.redF()
                 + 0.587 * window_color.greenF()
                 + 0.114 * window_color.blueF())
    return luminance < 0.5


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


class AspectRatioPixmapLabel(QtWidgets.QLabel):
    """QLabel that scales pixmaps to fit while preserving aspect ratio."""

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
    """Clickable QLabel for scene camera image (matching official sample)."""

    button_clicked_signal = QtCore.pyqtSignal(float, float)

    def __init__(self, parent=None):
        super(SceneImageLabel, self).__init__(parent)

    def mousePressEvent(self, ev):
        if ev.buttons() == Qt.LeftButton:
            target_rect = self.displayed_pixmap_rect()
            if target_rect.width() > 0 and target_rect.height() > 0 and target_rect.contains(ev.pos()):
                norm_x = (ev.x() - target_rect.x()) / target_rect.width()
                norm_y = (ev.y() - target_rect.y()) / target_rect.height()
                self.button_clicked_signal.emit(norm_x, norm_y)

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
        # Theme
        self.theme = THEME_DARK if detect_dark_theme() else THEME_LIGHT
        t = self.theme
        mono_font = QFont("Consolas", 12)
        mono_font.setStyleHint(QFont.TypeWriter)
        value_width = QtGui.QFontMetrics(mono_font).horizontalAdvance("-0000.0000") + 16

        self.setWindowTitle("Eye Tracker Monitor")
        self.resize(1500, 900)

        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)

        # ---- Stylesheet helpers ----
        group_ss = (
            f"QGroupBox {{"
            f"  border: 1px solid {t['group_border']};"
            f"  border-radius: 6px;"
            f"  margin-top: 10px;"
            f"  padding-top: 14px;"
            f"  font-weight: bold;"
            f"  color: {t['group_title']};"
            f"}}"
            f"QGroupBox::title {{"
            f"  subcontrol-origin: margin;"
            f"  left: 10px;"
            f"  padding: 0 4px;"
            f"}}"
        )
        value_ss = (
            f"QLabel {{"
            f"  background: {t['value_bg']};"
            f"  color: {t['value_fg']};"
            f"  border: 1px solid {t['group_border']};"
            f"  border-radius: 4px;"
            f"  padding: 2px 6px;"
            f"  font-family: 'Consolas', 'Monaco', monospace;"
            f"  font-size: 12px;"
            f"  min-width: 70px;"
            f"}}"
        )
        btn_ss = (
            f"QPushButton {{"
            f"  background: {t['btn_bg']};"
            f"  color: {t['btn_text']};"
            f"  border: 1px solid {t['group_border']};"
            f"  border-radius: 5px;"
            f"  padding: 6px 12px;"
            f"  font-size: 13px;"
            f"}}"
            f"QPushButton:hover {{ background: {t['btn_hover']}; }}"
            f"QPushButton:pressed {{ background: {t['group_border']}; }}"
            f"QPushButton:disabled {{"
            f"  background: {t['btn_disabled_bg']};"
            f"  color: {t['btn_disabled_text']};"
            f"}}"
        )
        combo_ss = (
            f"QComboBox {{"
            f"  background: {t['value_bg']};"
            f"  color: {t['fg']};"
            f"  border: 1px solid {t['group_border']};"
            f"  border-radius: 4px;"
            f"  padding: 3px 8px;"
            f"  font-size: 12px;"
            f"}}"
            f"QComboBox::drop-down {{ border: none; }}"
            f"QComboBox QAbstractItemView {{"
            f"  background: {t['value_bg']};"
            f"  color: {t['fg']};"
            f"  selection-background-color: {t['btn_hover']};"
            f"}}"
        )
        metric_ss = f"font-size: 13px; color: {t['label_color']};"
        separator_ss = (
            f"QFrame {{ background: {t['separator']}; max-height: 1px; }}"
        )

        # ---- Root layout ----
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # ===== Top metrics bar =====
        metrics_bar = QtWidgets.QHBoxLayout()
        metrics_bar.setSpacing(24)
        root.addLayout(metrics_bar)

        self.rate_label = QtWidgets.QLabel("Rate: -- Hz")
        self.count_label = QtWidgets.QLabel("Samples: 0")
        self.log_label = QtWidgets.QLabel("Log: --")
        for lbl in (self.rate_label, self.count_label, self.log_label):
            lbl.setStyleSheet(metric_ss)
            metrics_bar.addWidget(lbl)
        metrics_bar.addStretch()

        # ===== Main body: sidebar | content =====
        body = QtWidgets.QHBoxLayout()
        body.setSpacing(10)
        root.addLayout(body, stretch=1)

        # ---------- Left sidebar ----------
        sidebar = QtWidgets.QVBoxLayout()
        sidebar.setSpacing(6)
        sidebar.setContentsMargins(0, 0, 0, 0)
        sidebar_width = 240

        # -- Device section --
        self.groupBox_env = QtWidgets.QGroupBox("Environment")
        self.groupBox_env.setStyleSheet(group_ss)
        self.groupBox_env.setFixedWidth(sidebar_width)
        env_layout = QtWidgets.QVBoxLayout(self.groupBox_env)
        env_layout.setContentsMargins(8, 4, 8, 6)
        env_layout.setSpacing(4)
        self.comboBoxEnvironment = QtWidgets.QComboBox()
        self.comboBoxEnvironment.setStyleSheet(combo_ss)
        self.comboBoxEnvironment.addItem("indoor", 301)
        self.comboBoxEnvironment.addItem("outdoor", 302)
        self.comboBoxEnvironment.addItem("darkness", 303)
        self.comboBoxEnvironment.setCurrentIndex(0)
        env_layout.addWidget(self.comboBoxEnvironment)

        self.groupBox_res = QtWidgets.QGroupBox("Resolution")
        self.groupBox_res.setStyleSheet(group_ss)
        self.groupBox_res.setFixedWidth(sidebar_width)
        res_layout = QtWidgets.QVBoxLayout(self.groupBox_res)
        res_layout.setContentsMargins(8, 4, 8, 6)
        res_layout.setSpacing(4)
        self.comboBoxResolution = QtWidgets.QComboBox()
        self.comboBoxResolution.setStyleSheet(combo_ss)
        self.comboBoxResolution.addItem("1280 * 960", 201)
        self.comboBoxResolution.addItem("1280 * 720", 202)
        self.comboBoxResolution.addItem("800 * 600", 203)
        self.comboBoxResolution.addItem("1920 * 1080", 204)
        self.comboBoxResolution.setCurrentIndex(1)
        res_layout.addWidget(self.comboBoxResolution)

        sidebar.addWidget(self.groupBox_env)
        sidebar.addWidget(self.groupBox_res)

        # -- Control buttons --
        self.pushButtonStart = QtWidgets.QPushButton("▶  Start")
        self.pushButtonStart.setStyleSheet(btn_ss)
        self.pushButtonStart.setFixedWidth(sidebar_width)
        self.pushButtonStop = QtWidgets.QPushButton("■  Stop")
        self.pushButtonStop.setStyleSheet(btn_ss)
        self.pushButtonStop.setEnabled(False)
        self.pushButtonStop.setFixedWidth(sidebar_width)
        sidebar.addWidget(self.pushButtonStart)
        sidebar.addWidget(self.pushButtonStop)

        # Separator
        sep1 = QtWidgets.QFrame()
        sep1.setFrameShape(QtWidgets.QFrame.HLine)
        sep1.setStyleSheet(separator_ss)
        sidebar.addWidget(sep1)

        # -- Calibration section --
        cal_group = QtWidgets.QGroupBox("Calibration")
        cal_group.setStyleSheet(group_ss)
        cal_group.setFixedWidth(sidebar_width)
        cal_layout = QtWidgets.QVBoxLayout(cal_group)
        cal_layout.setContentsMargins(8, 4, 8, 6)
        cal_layout.setSpacing(4)

        points_row = QtWidgets.QHBoxLayout()
        points_row.setSpacing(6)
        self.label_points = QtWidgets.QLabel("Points")
        self.label_points.setStyleSheet(f"font-size: 12px; color: {t['label_color']};")
        self.comboBoxPoints = QtWidgets.QComboBox()
        self.comboBoxPoints.setStyleSheet(combo_ss)
        self.comboBoxPoints.addItem("1", 1)
        self.comboBoxPoints.addItem("3", 3)
        self.comboBoxPoints.addItem("5", 5)
        self.comboBoxPoints.addItem("9", 9)
        self.comboBoxPoints.setCurrentIndex(1)
        points_row.addWidget(self.label_points)
        points_row.addWidget(self.comboBoxPoints, stretch=1)
        cal_layout.addLayout(points_row)

        self.pushButtonStartCalibration = QtWidgets.QPushButton("Start Calibration")
        self.pushButtonStartCalibration.setStyleSheet(btn_ss)
        self.pushButtonStartCalibration.setEnabled(False)
        self.pushButtonCancelCalibration = QtWidgets.QPushButton("Stop Calibration")
        self.pushButtonCancelCalibration.setStyleSheet(btn_ss)
        cal_layout.addWidget(self.pushButtonStartCalibration)
        cal_layout.addWidget(self.pushButtonCancelCalibration)
        sidebar.addWidget(cal_group)

        # Separator
        sep2 = QtWidgets.QFrame()
        sep2.setFrameShape(QtWidgets.QFrame.HLine)
        sep2.setStyleSheet(separator_ss)
        sidebar.addWidget(sep2)

        # -- Data readouts --
        title_ss = f"font-size: 11px; color: {t['label_color']}; font-weight: bold;"

        self.groupBox_gaze = QtWidgets.QGroupBox("Recommend Gaze")
        self.groupBox_gaze.setStyleSheet(group_ss)
        self.groupBox_gaze.setFixedWidth(sidebar_width)
        gaze_layout = QtWidgets.QGridLayout(self.groupBox_gaze)
        gaze_layout.setContentsMargins(8, 4, 8, 6)
        gaze_layout.setHorizontalSpacing(6)
        gaze_layout.setVerticalSpacing(4)
        gaze_layout.setColumnMinimumWidth(0, 16)
        gaze_layout.setColumnMinimumWidth(1, value_width)
        gaze_layout.setColumnStretch(1, 1)
        self.label_gaze_x_title = QtWidgets.QLabel("X")
        self.label_gaze_x_title.setStyleSheet(title_ss)
        self.label_gaze_y_title = QtWidgets.QLabel("Y")
        self.label_gaze_y_title.setStyleSheet(title_ss)
        self.labelGazeX = QtWidgets.QLabel("")
        self.labelGazeX.setStyleSheet(value_ss)
        self.labelGazeX.setFont(mono_font)
        self.labelGazeX.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.labelGazeX.setFixedWidth(value_width)
        self.labelGazeY = QtWidgets.QLabel("")
        self.labelGazeY.setStyleSheet(value_ss)
        self.labelGazeY.setFont(mono_font)
        self.labelGazeY.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.labelGazeY.setFixedWidth(value_width)
        gaze_layout.addWidget(self.label_gaze_x_title, 0, 0)
        gaze_layout.addWidget(self.labelGazeX, 0, 1)
        gaze_layout.addWidget(self.label_gaze_y_title, 1, 0)
        gaze_layout.addWidget(self.labelGazeY, 1, 1)

        self.groupBox_left_pupil = QtWidgets.QGroupBox("Left Pupil")
        self.groupBox_left_pupil.setStyleSheet(group_ss)
        self.groupBox_left_pupil.setFixedWidth(sidebar_width)
        left_pupil_layout = QtWidgets.QGridLayout(self.groupBox_left_pupil)
        left_pupil_layout.setContentsMargins(8, 4, 8, 6)
        left_pupil_layout.setHorizontalSpacing(6)
        left_pupil_layout.setVerticalSpacing(4)
        left_pupil_layout.setColumnMinimumWidth(0, 16)
        left_pupil_layout.setColumnMinimumWidth(1, value_width)
        left_pupil_layout.setColumnStretch(1, 1)
        self.label_lp_x_title = QtWidgets.QLabel("X")
        self.label_lp_x_title.setStyleSheet(title_ss)
        self.label_lp_y_title = QtWidgets.QLabel("Y")
        self.label_lp_y_title.setStyleSheet(title_ss)
        self.labelLeftPupilCenterX = QtWidgets.QLabel("")
        self.labelLeftPupilCenterX.setStyleSheet(value_ss)
        self.labelLeftPupilCenterX.setFont(mono_font)
        self.labelLeftPupilCenterX.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.labelLeftPupilCenterX.setFixedWidth(value_width)
        self.labelLeftPupilCenterY = QtWidgets.QLabel("")
        self.labelLeftPupilCenterY.setStyleSheet(value_ss)
        self.labelLeftPupilCenterY.setFont(mono_font)
        self.labelLeftPupilCenterY.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.labelLeftPupilCenterY.setFixedWidth(value_width)
        left_pupil_layout.addWidget(self.label_lp_x_title, 0, 0)
        left_pupil_layout.addWidget(self.labelLeftPupilCenterX, 0, 1)
        left_pupil_layout.addWidget(self.label_lp_y_title, 1, 0)
        left_pupil_layout.addWidget(self.labelLeftPupilCenterY, 1, 1)

        self.groupBox_right_pupil = QtWidgets.QGroupBox("Right Pupil")
        self.groupBox_right_pupil.setStyleSheet(group_ss)
        self.groupBox_right_pupil.setFixedWidth(sidebar_width)
        right_pupil_layout = QtWidgets.QGridLayout(self.groupBox_right_pupil)
        right_pupil_layout.setContentsMargins(8, 4, 8, 6)
        right_pupil_layout.setHorizontalSpacing(6)
        right_pupil_layout.setVerticalSpacing(4)
        right_pupil_layout.setColumnMinimumWidth(0, 16)
        right_pupil_layout.setColumnMinimumWidth(1, value_width)
        right_pupil_layout.setColumnStretch(1, 1)
        self.label_rp_x_title = QtWidgets.QLabel("X")
        self.label_rp_x_title.setStyleSheet(title_ss)
        self.label_rp_y_title = QtWidgets.QLabel("Y")
        self.label_rp_y_title.setStyleSheet(title_ss)
        self.labelRightPupilCenterX = QtWidgets.QLabel("")
        self.labelRightPupilCenterX.setStyleSheet(value_ss)
        self.labelRightPupilCenterX.setFont(mono_font)
        self.labelRightPupilCenterX.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.labelRightPupilCenterX.setFixedWidth(value_width)
        self.labelRightPupilCenterY = QtWidgets.QLabel("")
        self.labelRightPupilCenterY.setStyleSheet(value_ss)
        self.labelRightPupilCenterY.setFont(mono_font)
        self.labelRightPupilCenterY.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.labelRightPupilCenterY.setFixedWidth(value_width)
        right_pupil_layout.addWidget(self.label_rp_x_title, 0, 0)
        right_pupil_layout.addWidget(self.labelRightPupilCenterX, 0, 1)
        right_pupil_layout.addWidget(self.label_rp_y_title, 1, 0)
        right_pupil_layout.addWidget(self.labelRightPupilCenterY, 1, 1)

        sidebar.addWidget(self.groupBox_gaze)
        sidebar.addWidget(self.groupBox_left_pupil)
        sidebar.addWidget(self.groupBox_right_pupil)

        sidebar.addStretch()
        body.addLayout(sidebar)

        # ---------- Right content area ----------
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(8)
        right.setContentsMargins(0, 0, 0, 0)

        # -- Top visual area --
        visual_row = QtWidgets.QHBoxLayout()
        visual_row.setSpacing(8)

        self.labelSceneImage = SceneImageLabel()
        self.labelSceneImage.connect_customized_slot(self._on_scene_image_area_clicked)
        self.labelSceneImage.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.labelSceneImage.setStyleSheet(
            f"QLabel {{ background: {t['value_bg']}; border: 1px solid {t['group_border']}; "
            f"border-radius: 6px; }}"
        )
        self.labelSceneImage.setText("Scene Image")
        self.labelSceneImage.setAlignment(Qt.AlignCenter)
        self.labelSceneImage.setMinimumSize(640, 360)
        visual_row.addWidget(self.labelSceneImage, stretch=1)

        eye_column = QtWidgets.QVBoxLayout()
        eye_column.setSpacing(8)
        left_eye_container = QtWidgets.QVBoxLayout()
        left_eye_container.setSpacing(2)
        left_eye_title = QtWidgets.QLabel("Left Eye")
        left_eye_title.setStyleSheet(f"font-size: 11px; color: {t['label_color']}; font-weight: bold;")
        left_eye_title.setAlignment(Qt.AlignCenter)
        self.labelLeftEyeImage = AspectRatioPixmapLabel()
        self.labelLeftEyeImage.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.labelLeftEyeImage.setStyleSheet(
            f"QLabel {{ background: {t['value_bg']}; border: 1px solid {t['group_border']}; "
            f"border-radius: 4px; }}"
        )
        self.labelLeftEyeImage.setText("left eye")
        self.labelLeftEyeImage.setAlignment(Qt.AlignCenter)
        self.labelLeftEyeImage.setMinimumSize(220, 165)
        self.labelLeftEyeImage.setMaximumWidth(240)
        left_eye_container.addWidget(left_eye_title)
        left_eye_container.addWidget(self.labelLeftEyeImage)

        right_eye_container = QtWidgets.QVBoxLayout()
        right_eye_container.setSpacing(2)
        right_eye_title = QtWidgets.QLabel("Right Eye")
        right_eye_title.setStyleSheet(f"font-size: 11px; color: {t['label_color']}; font-weight: bold;")
        right_eye_title.setAlignment(Qt.AlignCenter)
        self.labelRightEyeImage = AspectRatioPixmapLabel()
        self.labelRightEyeImage.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.labelRightEyeImage.setStyleSheet(
            f"QLabel {{ background: {t['value_bg']}; border: 1px solid {t['group_border']}; "
            f"border-radius: 4px; }}"
        )
        self.labelRightEyeImage.setText("right eye")
        self.labelRightEyeImage.setAlignment(Qt.AlignCenter)
        self.labelRightEyeImage.setMinimumSize(220, 165)
        self.labelRightEyeImage.setMaximumWidth(240)
        right_eye_container.addWidget(right_eye_title)
        right_eye_container.addWidget(self.labelRightEyeImage)

        eye_column.addLayout(left_eye_container)
        eye_column.addLayout(right_eye_container)
        eye_column.addStretch()
        visual_row.addLayout(eye_column)
        right.addLayout(visual_row, stretch=3)

        # -- Plots --
        pg.setConfigOptions(
            antialias=False, useOpenGL=False,
            background=t['bg'], foreground=t['fg'],
        )

        self.gaze_plot = pg.PlotWidget()
        gaze_item = self.gaze_plot.getPlotItem()
        gaze_item.setTitle("Gaze X / Y")
        gaze_item.setLabel("bottom", "Time", units="s")
        gaze_item.setLabel("left", "Pixels")
        gaze_item.showGrid(x=True, y=True, alpha=t['grid_alpha'])
        gaze_item.setMenuEnabled(False)
        gaze_item.setMouseEnabled(x=False, y=False)
        gaze_item.setXRange(-self.window_seconds, 0.0, padding=0.0)
        gaze_item.addLegend(offset=(6, 6))
        self.gaze_x_curve = self.gaze_plot.plot(pen=pg.mkPen("#4ED1A6", width=2), name="Gaze X")
        self.gaze_y_curve = self.gaze_plot.plot(pen=pg.mkPen("#FF7F50", width=2), name="Gaze Y")

        self.pupil_plot = pg.PlotWidget()
        pupil_item = self.pupil_plot.getPlotItem()
        pupil_item.setTitle("Pupil Diameter (mm)")
        pupil_item.setLabel("bottom", "Time", units="s")
        pupil_item.setLabel("left", "Diameter", units="mm")
        pupil_item.showGrid(x=True, y=True, alpha=t['grid_alpha'])
        pupil_item.setMenuEnabled(False)
        pupil_item.setMouseEnabled(x=False, y=False)
        pupil_item.setXRange(-self.window_seconds, 0.0, padding=0.0)
        pupil_item.addLegend(offset=(6, 6))
        self.left_pupil_curve = self.pupil_plot.plot(pen=pg.mkPen("#9AD1FF", width=2), name="Left")
        self.right_pupil_curve = self.pupil_plot.plot(pen=pg.mkPen("#FFD166", width=2), name="Right")

        plots_row = QtWidgets.QHBoxLayout()
        plots_row.setSpacing(8)
        plots_row.addWidget(self.gaze_plot)
        plots_row.addWidget(self.pupil_plot)
        right.addLayout(plots_row, stretch=2)

        body.addLayout(right, stretch=1)

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
        self.labelLeftPupilCenterX.setText(f"{left_x:.5f}")
        self.labelLeftPupilCenterY.setText(f"{left_y:.5f}")
        self.labelRightPupilCenterX.setText(f"{right_x:.5f}")
        self.labelRightPupilCenterY.setText(f"{right_y:.5f}")

    def _display_gaze_data(self, x, y):
        self.labelGazeX.setText(f"{x:.5f}")
        self.labelGazeY.setText(f"{y:.5f}")
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

    def _on_scene_image_area_clicked(self, norm_x, norm_y):
        """Convert click position to SDK center-origin coordinates (matching official sample)."""
        point_x = norm_x * self.scene_width - (self.scene_width / 2)
        point_y = norm_y * self.scene_height - (self.scene_height / 2)
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
        if self.calibration_is_running:
            self._on_stop_calibration()
        self.sdk.stop()
        self.pushButtonStop.setEnabled(False)
        self.pushButtonStart.setEnabled(True)
        self.pushButtonStartCalibration.setEnabled(False)
        self.labelSceneImage.setPixmap(QPixmap())
        self.labelLeftEyeImage.setPixmap(QPixmap())
        self.labelRightEyeImage.setPixmap(QPixmap())
        self.sdk_running = False
        self.timer.stop()
        
        print("\n===== Logging Session Summary =====")
        print(f"Eye Tracker Data Frames: {self.packet_count}")
        
        if self.csv_writer:
            print(f"Eye Tracker Log Output:  {self.csv_writer.output_file}")
            self.csv_writer.stop()
            self.csv_writer = None
            
        print("===================================\n")

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
    if hasattr(QtCore.Qt, 'AA_EnableHighDpiScaling'):
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    if hasattr(QtCore.Qt, 'AA_UseHighDpiPixmaps'):
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    args = parse_args()
    app = QtWidgets.QApplication(sys.argv)
    window = EyeTrackerMonitorWindow(args.sdk_root, args.sample_rate, args.window_seconds)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
