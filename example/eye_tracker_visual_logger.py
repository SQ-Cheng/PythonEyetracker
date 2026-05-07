#!/usr/bin/env python3
"""
Eye tracker visualization and CSV logger.

This script provides a GUI to start/stop the SDK, visualize gaze and pupil
signals, log samples to CSV, and perform calibration by clicking on the
calibration canvas.
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

import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from sdk_types import PY_7I_ENVIRONMENT, PY_7I_RESOLUTION
from sdk_wrapper import wrapper


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

THEME_DARK = {
    "bg": "#12161C",
    "fg": "#EAF0F7",
    "grid_alpha": 0.2,
    "label_color": "#EAF0F7",
    "accent": "#4ED1A6",
}

THEME_LIGHT = {
    "bg": "#FFFFFF",
    "fg": "#1A1A2E",
    "grid_alpha": 0.3,
    "label_color": "#1A1A2E",
    "accent": "#2F6BFF",
}


def detect_dark_theme():
    palette = QtWidgets.QApplication.instance().palette()
    window_color = palette.color(QtGui.QPalette.ColorRole.Window)
    luminance = 0.299 * window_color.redF() + 0.587 * window_color.greenF() + 0.114 * window_color.blueF()
    return luminance < 0.5


def get_theme_colors():
    return THEME_DARK if detect_dark_theme() else THEME_LIGHT


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
            values = values[-self.size :]
        for value in values:
            self.values[self.write_index] = float(value)
            self.write_index = (self.write_index + 1) % self.size
            self.count = min(self.size, self.count + 1)

    def ordered(self):
        if self.count == 0:
            return []
        if self.count < self.size:
            return self.values[: self.count]
        return self.values[self.write_index :] + self.values[: self.write_index]

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


class CalibrationCanvas(QtWidgets.QWidget):
    point_clicked = QtCore.Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 360)
        self._target_pos = None
        self._gaze_pos = None
        self._theme = get_theme_colors()

    def set_theme(self, theme):
        self._theme = theme
        self.update()

    def set_target(self, x, y):
        self._target_pos = (x, y)
        self.update()

    def set_gaze(self, x, y):
        self._gaze_pos = (x, y)
        self.update()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            pos = event.position() if hasattr(event, "position") else event.localPos()
            self._target_pos = (pos.x(), pos.y())
            self.point_clicked.emit(pos.x(), pos.y())
            self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor(self._theme["bg"]))

        if self._target_pos:
            self._draw_target(painter, self._target_pos, QtGui.QColor(self._theme["accent"]))

        if self._gaze_pos:
            self._draw_target(painter, self._gaze_pos, QtGui.QColor("#FFB347"), radius=6)

    def _draw_target(self, painter, pos, color, radius=10):
        painter.save()
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.setPen(QtGui.QPen(color, 2))
        painter.setBrush(QtGui.QBrush(QtGui.QColor(color.red(), color.green(), color.blue(), 120)))
        painter.drawEllipse(QtCore.QPointF(pos[0], pos[1]), radius, radius)
        painter.restore()


class EyeTrackerMonitorWindow(QtWidgets.QMainWindow):
    set_calibration_finish_signal = QtCore.Signal(int, int, int)

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
        self.current_points = 0
        self.finish_points = [[1, 1], [1, 1], [1, 1]]

        self.scene_width = 1280
        self.scene_height = 720
        self.sdk_running = False

        self.theme = get_theme_colors()
        self._build_ui()

        self.set_calibration_finish_signal.connect(self._on_calibration_finish)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer.setInterval(10)

        self.csv_writer = None

    def _build_ui(self):
        pg.setConfigOptions(antialias=False, useOpenGL=False,
                            background=self.theme["bg"], foreground=self.theme["fg"])

        self.setWindowTitle("Eye Tracker Visual Logger")
        self.resize(1400, 900)

        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        metrics_layout = QtWidgets.QHBoxLayout()
        metrics_layout.setSpacing(20)
        root.addLayout(metrics_layout)

        self.rate_label = QtWidgets.QLabel("Rate: -- Hz")
        self.count_label = QtWidgets.QLabel("Samples: 0")
        self.log_label = QtWidgets.QLabel("Log: --")
        metrics_layout.addWidget(self.rate_label)
        metrics_layout.addWidget(self.count_label)
        metrics_layout.addWidget(self.log_label)
        metrics_layout.addStretch()

        content_layout = QtWidgets.QHBoxLayout()
        content_layout.setSpacing(12)
        root.addLayout(content_layout, stretch=1)

        left_panel = QtWidgets.QVBoxLayout()
        left_panel.setSpacing(10)
        content_layout.addLayout(left_panel, stretch=0)

        connection_group = QtWidgets.QGroupBox("Connection")
        connection_layout = QtWidgets.QVBoxLayout(connection_group)
        left_panel.addWidget(connection_group)

        self.environment_combo = QtWidgets.QComboBox()
        self.environment_combo.addItem("Indoor", PY_7I_ENVIRONMENT.INDOOR.value)
        self.environment_combo.addItem("Outdoor", PY_7I_ENVIRONMENT.OUTDOOR.value)
        self.environment_combo.addItem("Darkness", PY_7I_ENVIRONMENT.DARKNESS.value)

        self.resolution_combo = QtWidgets.QComboBox()
        self.resolution_combo.addItem("1280 x 720", PY_7I_RESOLUTION.P1280_720.value)
        self.resolution_combo.addItem("1280 x 960", PY_7I_RESOLUTION.P1280_960.value)
        self.resolution_combo.addItem("800 x 600", PY_7I_RESOLUTION.P800_600.value)
        self.resolution_combo.addItem("1920 x 1080", PY_7I_RESOLUTION.P1920_1080.value)

        connection_layout.addWidget(QtWidgets.QLabel("Environment"))
        connection_layout.addWidget(self.environment_combo)
        connection_layout.addWidget(QtWidgets.QLabel("Resolution"))
        connection_layout.addWidget(self.resolution_combo)

        self.start_button = QtWidgets.QPushButton("Start")
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.setEnabled(False)
        connection_layout.addWidget(self.start_button)
        connection_layout.addWidget(self.stop_button)

        calibration_group = QtWidgets.QGroupBox("Calibration")
        calibration_layout = QtWidgets.QVBoxLayout(calibration_group)
        left_panel.addWidget(calibration_group)

        self.points_combo = QtWidgets.QComboBox()
        self.points_combo.addItem("1 point", 1)
        self.points_combo.addItem("3 points", 3)
        calibration_layout.addWidget(QtWidgets.QLabel("Points"))
        calibration_layout.addWidget(self.points_combo)

        self.calibrate_button = QtWidgets.QPushButton("Start Calibration")
        self.stop_calibration_button = QtWidgets.QPushButton("Stop Calibration")
        self.calibrate_button.setEnabled(False)
        self.stop_calibration_button.setEnabled(False)
        calibration_layout.addWidget(self.calibrate_button)
        calibration_layout.addWidget(self.stop_calibration_button)

        self.calibration_hint = QtWidgets.QLabel("Click on the canvas to place the target")
        self.calibration_hint.setWordWrap(True)
        calibration_layout.addWidget(self.calibration_hint)

        live_group = QtWidgets.QGroupBox("Live Values")
        live_layout = QtWidgets.QGridLayout(live_group)
        left_panel.addWidget(live_group)

        self.gaze_value = QtWidgets.QLabel("--")
        self.left_pupil_value = QtWidgets.QLabel("--")
        self.right_pupil_value = QtWidgets.QLabel("--")
        self.openness_value = QtWidgets.QLabel("--")
        self.blink_value = QtWidgets.QLabel("--")

        live_layout.addWidget(QtWidgets.QLabel("Gaze (x, y)"), 0, 0)
        live_layout.addWidget(self.gaze_value, 0, 1)
        live_layout.addWidget(QtWidgets.QLabel("Left pupil (x, y)"), 1, 0)
        live_layout.addWidget(self.left_pupil_value, 1, 1)
        live_layout.addWidget(QtWidgets.QLabel("Right pupil (x, y)"), 2, 0)
        live_layout.addWidget(self.right_pupil_value, 2, 1)
        live_layout.addWidget(QtWidgets.QLabel("Openness (L/R)"), 3, 0)
        live_layout.addWidget(self.openness_value, 3, 1)
        live_layout.addWidget(QtWidgets.QLabel("Blink (L/R)"), 4, 0)
        live_layout.addWidget(self.blink_value, 4, 1)

        left_panel.addStretch(1)

        right_panel = QtWidgets.QVBoxLayout()
        right_panel.setSpacing(10)
        content_layout.addLayout(right_panel, stretch=1)

        self.gaze_plot = pg.PlotWidget(title="Gaze X / Y")
        self.gaze_plot.showGrid(x=True, y=True, alpha=self.theme["grid_alpha"])
        self.gaze_plot.setLabel("bottom", "Time (s)")
        self.gaze_plot.setLabel("left", "Pixels")
        self.gaze_x_curve = self.gaze_plot.plot(pen=pg.mkPen(self.theme["accent"], width=2), name="Gaze X")
        self.gaze_y_curve = self.gaze_plot.plot(pen=pg.mkPen("#FF7F50", width=2), name="Gaze Y")

        self.pupil_plot = pg.PlotWidget(title="Pupil Diameter (mm)")
        self.pupil_plot.showGrid(x=True, y=True, alpha=self.theme["grid_alpha"])
        self.pupil_plot.setLabel("bottom", "Time (s)")
        self.pupil_plot.setLabel("left", "Diameter (mm)")
        self.left_pupil_curve = self.pupil_plot.plot(pen=pg.mkPen("#9AD1FF", width=2), name="Left")
        self.right_pupil_curve = self.pupil_plot.plot(pen=pg.mkPen("#FFD166", width=2), name="Right")

        right_panel.addWidget(self.gaze_plot, stretch=1)
        right_panel.addWidget(self.pupil_plot, stretch=1)

        canvas_group = QtWidgets.QGroupBox("Calibration Canvas")
        canvas_layout = QtWidgets.QVBoxLayout(canvas_group)
        self.canvas = CalibrationCanvas()
        self.canvas.set_theme(self.theme)
        canvas_layout.addWidget(self.canvas)
        right_panel.addWidget(canvas_group, stretch=1)

        self.start_button.clicked.connect(self._on_start)
        self.stop_button.clicked.connect(self._on_stop)
        self.calibrate_button.clicked.connect(self._on_start_calibration)
        self.stop_calibration_button.clicked.connect(self._on_stop_calibration)
        self.canvas.point_clicked.connect(self._on_canvas_clicked)

    def _on_start(self):
        pwd = self._read_pwd()
        if not pwd:
            QtWidgets.QMessageBox.warning(self, "Warning", "Password not found in config.ini")
            return

        ret = self.sdk.connect_softdog(pwd)
        if ret != 0:
            QtWidgets.QMessageBox.warning(self, "Warning", "Softdog connection failed")
            return

        environment = self.environment_combo.currentData()
        resolution = self.resolution_combo.currentData()
        if resolution == PY_7I_RESOLUTION.P1280_960.value:
            self.scene_width, self.scene_height = 1280, 960
        elif resolution == PY_7I_RESOLUTION.P1280_720.value:
            self.scene_width, self.scene_height = 1280, 720
        elif resolution == PY_7I_RESOLUTION.P800_600.value:
            self.scene_width, self.scene_height = 800, 600
        elif resolution == PY_7I_RESOLUTION.P1920_1080.value:
            self.scene_width, self.scene_height = 1920, 1080

        ret = self.sdk.start(environment, resolution, self.scene_width, self.scene_height)
        if ret != 0:
            QtWidgets.QMessageBox.warning(self, "Warning", "SDK start failed")
            return

        log_dir = os.path.join(os.path.dirname(os.getcwd()), "log")
        os.makedirs(log_dir, exist_ok=True)
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = os.path.join(log_dir, f"eye_tracker_{timestamp_str}.csv")
        self.csv_writer = CSVWriterThread(output_file)
        self.csv_writer.start()
        self.log_label.setText(f"Log: {output_file}")

        self.sdk_running = True
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.calibrate_button.setEnabled(True)
        self.stop_calibration_button.setEnabled(False)
        self.timer.start()

    def _on_stop(self):
        if self.sdk_running:
            self.sdk.stop()
        self.sdk_running = False
        self.timer.stop()
        if self.csv_writer:
            self.csv_writer.stop()
            self.csv_writer = None

        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.calibrate_button.setEnabled(False)
        self.stop_calibration_button.setEnabled(False)

    def _on_start_calibration(self):
        if not self.sdk_running:
            return
        self.current_points = self.points_combo.currentData()
        self._reset_finish_points()
        self.sdk.start_calibration(self.current_points)
        self.calibrate_button.setEnabled(False)
        self.stop_calibration_button.setEnabled(True)

    def _on_stop_calibration(self):
        self.sdk.stop_calibration()
        self.calibrate_button.setEnabled(True)
        self.stop_calibration_button.setEnabled(False)

    def _on_canvas_clicked(self, x, y):
        if not self.sdk_running:
            return
        canvas_w = max(1, self.canvas.width())
        canvas_h = max(1, self.canvas.height())
        scale_x = self.scene_width / canvas_w
        scale_y = self.scene_height / canvas_h

        # Invert Y logic: SDK treats upward as positive Y, but UI treats downward as positive
        point_x = float(x * scale_x) - float(self.scene_width / 2)
        point_y = float(self.scene_height / 2) - float(y * scale_y) 
        self.sdk.set_current_point(point_x, point_y)

    def _on_calibration_finish(self, eye, index, error):
        self.finish_points[index - 1][eye] = error
        if self.current_points == 1:
            if self.finish_points[0][0] != 1 and self.finish_points[0][1] != 1:
                self._on_stop_calibration()
        elif self.current_points == 3:
            done = all(self.finish_points[i][0] != 1 and self.finish_points[i][1] != 1 for i in range(3))
            if done:
                self._on_stop_calibration()

    def _reset_finish_points(self):
        for i in range(3):
            self.finish_points[i][0] = 1
            self.finish_points[i][1] = 1

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

            self._update_canvas_gaze(sample)

        if drained == 0:
            return

        if (now - self.last_plot_update) * 1000 >= PLOT_UPDATE_INTERVAL_MS:
            self._update_plots()
            self.last_plot_update = now

        if (now - self.last_metric_update) * 1000 >= METRIC_UPDATE_INTERVAL_MS:
            self._update_metrics()
            self.last_metric_update = now

    def _update_canvas_gaze(self, sample):
        canvas_w = max(1, self.canvas.width())
        canvas_h = max(1, self.canvas.height())
        
        # Invert Y logic back to UI coordinates (upward gaze yields positive SDK Y)
        x = (sample["gaze_x"] + self.scene_width / 2) / self.scene_width * canvas_w
        y = (self.scene_height / 2 - sample["gaze_y"]) / self.scene_height * canvas_h
        self.canvas.set_gaze(x, y)

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
        if self.latest_sample:
            self.gaze_value.setText(f"{self.latest_sample['gaze_x']:.1f}, {self.latest_sample['gaze_y']:.1f}")
            self.left_pupil_value.setText(
                f"{self.latest_sample['left_pupil_x']:.1f}, {self.latest_sample['left_pupil_y']:.1f}"
            )
            self.right_pupil_value.setText(
                f"{self.latest_sample['right_pupil_x']:.1f}, {self.latest_sample['right_pupil_y']:.1f}"
            )
            self.openness_value.setText(
                f"{self.latest_sample['left_openness']:.2f}, {self.latest_sample['right_openness']:.2f}"
            )
            self.blink_value.setText(
                f"{self.latest_sample['left_blink']}, {self.latest_sample['right_blink']}"
            )

    def _read_pwd(self):
        config_path = os.path.join(self.sdk_config_path, "config.ini")
        cf = configparser.ConfigParser()
        cf.read(config_path)
        pwd = cf.get("softdog", "pwd", fallback="")
        return pwd.encode("utf-8")

    def closeEvent(self, event):
        if self.sdk_running:
            self._on_stop()
            time.sleep(1)
        event.accept()


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
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
