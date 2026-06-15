#!/usr/bin/env python
"""
sensor_visual_logger.py

Real-time visualization + CSV logging of sensor data from Arduino.

Packet format (64 bytes, little-endian):
  Bytes  0-3:   Sync marker (0x55 0xAA 0x55 0xAA)
  Bytes  4-7:   PPG Red
  Bytes  8-11:  PPG IR
  Bytes 12-15:  PPG Green
  Bytes 16-19:  accX (scaled by 0.01)
  Bytes 20-23:  accY (scaled by 0.01)
  Bytes 24-27:  accZ (scaled by 0.01)
  Bytes 28-31:  gyrX
  Bytes 32-35:  gyrY
  Bytes 36-39:  gyrZ
  Bytes 40-43:  magX
  Bytes 44-47:  magY
  Bytes 48-51:  magZ
  Bytes 52-55:  temperature
  Bytes 56-59:  timestamp (millis() as float)
  Bytes 60-63:  padding (zeros)

Usage:
  python sensor_visual_logger.py [--port COM_PORT] [--baud BAUD_RATE]
                                 [--window-seconds N] [--sample-rate Hz]
"""

import argparse
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
import pyqtgraph as pg
import serial
from PySide6 import QtCore, QtGui, QtWidgets

# ===================== Constants =====================
PACKET_SIZE = 64
NUM_FLOATS = 14
SYNC_MARKER = bytes([0x55, 0xAA, 0x55, 0xAA])
SYNC_LEN = 4

DISPLAY_WINDOW_SECONDS = 10.0
PLOT_UPDATE_INTERVAL_MS = 30
METRIC_UPDATE_INTERVAL_MS = 250
Y_LIMIT_UPDATE_INTERVAL_MS = 500
SERIAL_STARTUP_SETTLE_SECONDS = 2.0
RATE_WINDOW_SECONDS = 3.0

COLUMN_NAMES = [
    'Red', 'IR', 'Green',
    'accX', 'accY', 'accZ',
    'gyrX', 'gyrY', 'gyrZ',
    'magX', 'magY', 'magZ',
    'temp', 'timestamp', 'pc_timestamp'
]

# Theme color palettes
THEME_DARK = {
    'bg': '#12161C',
    'fg': '#EAF0F7',
    'grid_alpha': 0.2,
    'label_color': '#EAF0F7',
}
THEME_LIGHT = {
    'bg': '#FFFFFF',
    'fg': '#1A1A2E',
    'grid_alpha': 0.3,
    'label_color': '#1A1A2E',
}


# ===================== Theme Detection =====================

def detect_dark_theme():
    """Detect if the system is using a dark theme by checking palette luminance."""
    palette = QtWidgets.QApplication.instance().palette()
    window_color = palette.color(QtGui.QPalette.ColorRole.Window)
    luminance = 0.299 * window_color.redF() + 0.587 * window_color.greenF() + 0.114 * window_color.blueF()
    return luminance < 0.5


def get_theme_colors():
    """Return the appropriate color palette for the current system theme."""
    return THEME_DARK if detect_dark_theme() else THEME_LIGHT


# ===================== Packet Rate Tracker =====================

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
        if len(self.timestamps) < 2:
            return 0.0
        duration = self.timestamps[-1] - self.timestamps[0]
        if duration <= 0:
            return 0.0
        return (len(self.timestamps) - 1) / duration


# ===================== Ring Series Buffer =====================

class RingSeries:
    def __init__(self, sample_rate_hz, window_seconds):
        self.sample_rate_hz = sample_rate_hz
        self.window_seconds = window_seconds
        self.size = max(128, int(round(sample_rate_hz * window_seconds)))
        self.values = np.zeros(self.size, dtype=np.float64)
        self.write_index = 0
        self.count = 0

    def append(self, batch):
        if not batch:
            return
        values = np.asarray(batch, dtype=np.float64)
        if len(values) >= self.size:
            values = values[-self.size:]
        n = len(values)
        first = min(n, self.size - self.write_index)
        second = n - first
        self.values[self.write_index:self.write_index + first] = values[:first]
        if second > 0:
            self.values[:second] = values[first:]
        self.write_index = (self.write_index + n) % self.size
        self.count = min(self.size, self.count + n)

    def ordered(self):
        if self.count == 0:
            return np.array([], dtype=np.float64)
        if self.count < self.size:
            return self.values[:self.count].copy()
        return np.concatenate((self.values[self.write_index:], self.values[:self.write_index]))

    def x_axis(self, count):
        return np.linspace(-self.window_seconds, 0.0, count, endpoint=True, dtype=np.float64)


class MultiSeriesBuffer:
    def __init__(self, sample_rate_hz, window_seconds):
        self.red = RingSeries(sample_rate_hz, window_seconds)
        self.ir = RingSeries(sample_rate_hz, window_seconds)
        self.green = RingSeries(sample_rate_hz, window_seconds)
        self.ax = RingSeries(sample_rate_hz, window_seconds)
        self.ay = RingSeries(sample_rate_hz, window_seconds)
        self.az = RingSeries(sample_rate_hz, window_seconds)
        self.gx = RingSeries(sample_rate_hz, window_seconds)
        self.gy = RingSeries(sample_rate_hz, window_seconds)
        self.gz = RingSeries(sample_rate_hz, window_seconds)
        self.mx = RingSeries(sample_rate_hz, window_seconds)
        self.my = RingSeries(sample_rate_hz, window_seconds)
        self.mz = RingSeries(sample_rate_hz, window_seconds)


# ===================== Timestamp Synchronization =====================

def sync_timestamps(ser, rounds=20):
    """Synchronize Arduino millis() with PC wall-clock time via round-trip measurement.

    Sends multiple 't' commands and selects the round with minimum RTT to minimize
    asymmetric delay error. Returns the offset in seconds:
        pc_time = arduino_millis / 1000.0 + offset

    Expected accuracy: ≤1 ms over USB serial (typical min RTT ~1-2 ms).
    """
    best_rtt = float('inf')
    best_offset = 0.0
    success_count = 0

    for _ in range(rounds):
        ser.reset_input_buffer()
        t1 = time.perf_counter()
        ser.write(b't\n')
        ser.flush()
        response = ser.readline()
        t2 = time.perf_counter()

        if not response or not response.startswith(b'T'):
            continue

        try:
            arduino_ms = int(response[1:].strip())
        except ValueError:
            continue

        rtt = t2 - t1
        # Offset = PC midpoint time - Arduino time
        offset = (t1 + t2) / 2.0 - arduino_ms / 1000.0

        if rtt < best_rtt:
            best_rtt = rtt
            best_offset = offset
        success_count += 1

    if success_count == 0:
        print("Warning: Timestamp sync failed, using offset=0")
        return 0.0

    print(f"Timestamp sync: offset={best_offset * 1000:.3f} ms "
          f"(min RTT={best_rtt * 1000:.3f} ms, {success_count}/{rounds} rounds)")
    return best_offset


# ===================== Serial Reader =====================

class SerialPacketReader:
    def __init__(self, port, baudrate):
        self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=0.05)
        self.packet_queue = queue.SimpleQueue()
        self._buffer = bytearray()
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread_started = False

    def start(self):
        self._thread.start()
        self._thread_started = True

    def stop(self):
        self._stop_event.set()
        if self._thread_started:
            self._thread.join(timeout=1.0)
        if self.serial_port.is_open:
            self.serial_port.close()

    def send_command(self, command_text):
        with self._write_lock:
            self.serial_port.write(command_text.encode("ascii"))
            self.serial_port.flush()

    def _read_loop(self):
        while not self._stop_event.is_set():
            incoming = self.serial_port.read(self.serial_port.in_waiting or 1)
            if incoming:
                self._buffer.extend(incoming)
                self._consume_packets()

    def _consume_packets(self):
        while True:
            start_index = self._buffer.find(SYNC_MARKER)
            if start_index < 0:
                if self._buffer and self._buffer[-1] == SYNC_MARKER[0]:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                return

            if start_index > 0:
                del self._buffer[:start_index]

            if len(self._buffer) < PACKET_SIZE:
                return

            payload = bytes(self._buffer[SYNC_LEN:PACKET_SIZE])
            del self._buffer[:PACKET_SIZE]

            try:
                values = struct.unpack('<14f', payload[:NUM_FLOATS * 4])
            except struct.error:
                continue

            nan_count = sum(1 for v in values if v != v)
            if nan_count > 3:
                continue
            timestamp = values[13]
            if timestamp == timestamp and (timestamp < 0 or timestamp > 4.3e9):
                continue

            self.packet_queue.put({
                "timestamp": time.perf_counter(),
                "red": values[0],
                "ir": values[1],
                "green": values[2],
                "ax": values[3],
                "ay": values[4],
                "az": values[5],
                "gx": values[6],
                "gy": values[7],
                "gz": values[8],
                "mx": values[9],
                "my": values[10],
                "mz": values[11],
                "temp": values[12],
                "timestamp_ms": values[13],
            })


# ===================== CSV Writer Thread =====================

class CSVWriterThread:
    def __init__(self, output_file):
        self.output_file = output_file
        self.queue = queue.SimpleQueue()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self.packets_written = 0

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)

    def push(self, values):
        self.queue.put(values)

    def _write_loop(self):
        with open(self.output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(COLUMN_NAMES)
            while not self._stop_event.is_set() or not self.queue.empty():
                try:
                    values = self.queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                row = []
                for i, v in enumerate(values):
                    if i in (0, 1, 2, 13):
                        row.append(str(int(v)))
                    else:
                        row.append(f'{v:.6f}')
                writer.writerow(row)
                self.packets_written += 1


# ===================== Main Window =====================

class SensorMonitorWindow(QtWidgets.QMainWindow):
    def __init__(self, port, baudrate, sample_rate_hz, window_seconds):
        super().__init__()
        self.reader = SerialPacketReader(port, baudrate)
        self.rate_tracker = PacketRateTracker()
        self.buffers = MultiSeriesBuffer(sample_rate_hz, window_seconds)
        self.window_seconds = window_seconds
        self.last_plot_update_ms = 0.0
        self.last_metric_update_ms = 0.0
        self.last_range_update_ms = 0.0
        self.invalid_count = 0
        self.latest_temp = None

        # Theme colors
        self.theme = get_theme_colors()

        # CSV output
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.join(os.path.dirname(os.getcwd()), 'log')
        os.makedirs(log_dir, exist_ok=True)
        self.output_file = os.path.join(log_dir, f'sensor_data_{timestamp_str}.csv')
        self.csv_writer = CSVWriterThread(self.output_file)

        self.setWindowTitle(f"Sensor Monitor - {port}")
        self.resize(1400, 900)
        self._build_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_timer)

    def _build_ui(self):
        bg = self.theme['bg']
        fg = self.theme['fg']
        grid_alpha = self.theme['grid_alpha']

        pg.setConfigOptions(antialias=False, useOpenGL=False,
                            background=bg, foreground=fg)

        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # Metrics bar
        metrics = QtWidgets.QHBoxLayout()
        metrics.setSpacing(20)
        root.addLayout(metrics)

        self.data_rate_label = QtWidgets.QLabel("Rate: -- Hz")
        self.packet_count_label = QtWidgets.QLabel("Packets: 0")
        self.invalid_label = QtWidgets.QLabel("Invalid: 0")
        self.temp_label = QtWidgets.QLabel("Temp: --")
        self.file_label = QtWidgets.QLabel(f"CSV: {os.path.basename(self.output_file)}")

        label_style = f"font-size: 13px; color: {self.theme['label_color']};"
        for label in [self.data_rate_label, self.packet_count_label,
                       self.invalid_label, self.temp_label, self.file_label]:
            label.setStyleSheet(label_style)
            metrics.addWidget(label)

        metrics.addStretch()

        # PPG: 3 stacked sub-plots in a GraphicsLayoutWidget
        self.ppg_widget = pg.GraphicsLayoutWidget()
        self.ppg_widget.setBackground(bg)

        self.ppg_red_plot = self.ppg_widget.addPlot(row=0, col=0, title="PPG Red")
        self.ppg_ir_plot = self.ppg_widget.addPlot(row=1, col=0, title="PPG IR")
        self.ppg_green_plot = self.ppg_widget.addPlot(row=2, col=0, title="PPG Green")

        # Configure each PPG sub-plot
        ppg_configs = [
            (self.ppg_red_plot, "counts", "#FF6B6B"),
            (self.ppg_ir_plot, "counts", "#2EC4B6"),
            (self.ppg_green_plot, "counts", "#90BE6D"),
        ]
        self.ppg_curves = []
        for plot_item, ylabel, color in ppg_configs:
            plot_item.setLabel("left", ylabel)
            plot_item.showGrid(x=True, y=True, alpha=grid_alpha)
            plot_item.setMenuEnabled(False)
            plot_item.setMouseEnabled(x=False, y=False)
            plot_item.setXRange(-self.window_seconds, 0.0, padding=0.0)
            curve = plot_item.plot(pen=pg.mkPen(color=color, width=1.5))
            self.ppg_curves.append(curve)

        # Link X-axes of all 3 PPG sub-plots
        self.ppg_ir_plot.setXLink(self.ppg_red_plot)
        self.ppg_green_plot.setXLink(self.ppg_red_plot)

        # Hide X-axis labels on top two, show only on bottom
        self.ppg_red_plot.hideAxis('bottom')
        self.ppg_ir_plot.hideAxis('bottom')
        self.ppg_green_plot.setLabel("bottom", "Time", units="s")

        # Adjust row heights (top plots get less space)
        self.ppg_widget.ci.layout.setRowStretchFactor(0, 1)
        self.ppg_widget.ci.layout.setRowStretchFactor(1, 1)
        self.ppg_widget.ci.layout.setRowStretchFactor(2, 1)

        # Other plots: 2x2 grid (PPG widget takes top-left)
        self.accel_plot = self._create_plot(
            "Accelerometer", "g",
            ["#4CC9F0", "#FFD166", "#06D6A0"],
            legend=["X", "Y", "Z"])

        self.gyro_plot = self._create_plot(
            "Gyroscope", "dps",
            ["#F72585", "#B5179E", "#7209B7"],
            legend=["X", "Y", "Z"])

        self.mag_plot = self._create_plot(
            "Magnetometer", "µT",
            ["#FF9F1C", "#E71D36", "#2EC4B6"],
            legend=["X", "Y", "Z"])

        plots = QtWidgets.QGridLayout()
        plots.setHorizontalSpacing(8)
        plots.setVerticalSpacing(8)
        plots.addWidget(self.ppg_widget, 0, 0)
        plots.addWidget(self.accel_plot[0], 0, 1)
        plots.addWidget(self.gyro_plot[0], 1, 0)
        plots.addWidget(self.mag_plot[0], 1, 1)
        root.addLayout(plots, stretch=1)

    def _create_plot(self, title, ylabel, colors, legend=None):
        bg = self.theme['bg']
        grid_alpha = self.theme['grid_alpha']

        widget = pg.PlotWidget()
        item = widget.getPlotItem()
        widget.setBackground(bg)
        item.setTitle(title)
        item.setLabel("left", ylabel)
        item.setLabel("bottom", "Time", units="s")
        item.showGrid(x=True, y=True, alpha=grid_alpha)
        item.setMenuEnabled(False)
        item.setMouseEnabled(x=False, y=False)
        item.setXRange(-self.window_seconds, 0.0, padding=0.0)

        if legend is not None:
            item.addLegend(offset=(6, 6))

        curves = []
        for index, color in enumerate(colors):
            name = None if legend is None else legend[index]
            curves.append(item.plot(pen=pg.mkPen(color=color, width=1.5), name=name))

        return widget, curves

    def start(self):
        # Wait for Arduino to reset after serial connection
        time.sleep(SERIAL_STARTUP_SETTLE_SECONDS)
        self.reader.serial_port.reset_input_buffer()
        self.reader.serial_port.reset_output_buffer()

        # Synchronize Arduino millis() with PC wall-clock time (before reader thread)
        self.ts_offset = sync_timestamps(self.reader.serial_port, rounds=20)

        # Start CSV writer and reader thread
        self.csv_writer.start()
        self.reader.start()

        time.sleep(0.3)
        self.reader.send_command("s\n")
        print(f"Sent 's' command - data collection started")

        self.timer.start(PLOT_UPDATE_INTERVAL_MS)
        self.show()

    def closeEvent(self, event):
        self.timer.stop()

        try:
            self.reader.send_command("e\n")
            time.sleep(0.3)
        except Exception:
            pass

        self.reader.stop()
        self.csv_writer.stop()

        print(f"\n===== Summary =====")
        print(f"Total packets: {self.rate_tracker.total_packets}")
        print(f"Invalid packets: {self.invalid_count}")
        print(f"CSV rows written: {self.csv_writer.packets_written}")
        print(f"Data saved to: {self.output_file}")
        print(f"====================")

        super().closeEvent(event)

    def _on_timer(self):
        self._drain_queue()

        now_ms = time.perf_counter() * 1000.0
        if self.buffers.red.count > 0 and now_ms - self.last_plot_update_ms >= PLOT_UPDATE_INTERVAL_MS:
            self.last_plot_update_ms = now_ms
            self._update_plots()

        if now_ms - self.last_metric_update_ms >= METRIC_UPDATE_INTERVAL_MS:
            self.last_metric_update_ms = now_ms
            self._update_metrics()

    def _drain_queue(self):
        red_batch = []
        ir_batch = []
        green_batch = []
        ax_batch = []
        ay_batch = []
        az_batch = []
        gx_batch = []
        gy_batch = []
        gz_batch = []
        mx_batch = []
        my_batch = []
        mz_batch = []
        drained = 0

        while True:
            try:
                packet = self.reader.packet_queue.get_nowait()
            except queue.Empty:
                break

            drained += 1
            self.rate_tracker.push(packet["timestamp"])

            red_batch.append(packet["red"])
            ir_batch.append(packet["ir"])
            green_batch.append(packet["green"])
            ax_batch.append(packet["ax"])
            ay_batch.append(packet["ay"])
            az_batch.append(packet["az"])
            gx_batch.append(packet["gx"])
            gy_batch.append(packet["gy"])
            gz_batch.append(packet["gz"])
            mx_batch.append(packet["mx"])
            my_batch.append(packet["my"])
            mz_batch.append(packet["mz"])

            self.latest_temp = packet["temp"]

            pc_ts = packet["timestamp_ms"] / 1000.0 + self.ts_offset
            self.csv_writer.push([
                packet["red"], packet["ir"], packet["green"],
                packet["ax"], packet["ay"], packet["az"],
                packet["gx"], packet["gy"], packet["gz"],
                packet["mx"], packet["my"], packet["mz"],
                packet["temp"], packet["timestamp_ms"],
                pc_ts,
            ])

        if drained:
            self.buffers.red.append(red_batch)
            self.buffers.ir.append(ir_batch)
            self.buffers.green.append(green_batch)
            self.buffers.ax.append(ax_batch)
            self.buffers.ay.append(ay_batch)
            self.buffers.az.append(az_batch)
            self.buffers.gx.append(gx_batch)
            self.buffers.gy.append(gy_batch)
            self.buffers.gz.append(gz_batch)
            self.buffers.mx.append(mx_batch)
            self.buffers.my.append(my_batch)
            self.buffers.mz.append(mz_batch)

    def _update_plots(self):
        red = self.buffers.red.ordered()
        if len(red) == 0:
            return

        x = self.buffers.red.x_axis(len(red))

        # PPG: each channel in its own sub-plot
        self.ppg_curves[0].setData(x, red)
        self.ppg_curves[1].setData(x, self.buffers.ir.ordered())
        self.ppg_curves[2].setData(x, self.buffers.green.ordered())

        # IMU plots
        self.accel_plot[1][0].setData(x, self.buffers.ax.ordered())
        self.accel_plot[1][1].setData(x, self.buffers.ay.ordered())
        self.accel_plot[1][2].setData(x, self.buffers.az.ordered())

        self.gyro_plot[1][0].setData(x, self.buffers.gx.ordered())
        self.gyro_plot[1][1].setData(x, self.buffers.gy.ordered())
        self.gyro_plot[1][2].setData(x, self.buffers.gz.ordered())

        self.mag_plot[1][0].setData(x, self.buffers.mx.ordered())
        self.mag_plot[1][1].setData(x, self.buffers.my.ordered())
        self.mag_plot[1][2].setData(x, self.buffers.mz.ordered())

        now_ms = time.perf_counter() * 1000.0
        if now_ms - self.last_range_update_ms >= Y_LIMIT_UPDATE_INTERVAL_MS:
            self.last_range_update_ms = now_ms
            self._update_ranges()

    def _update_ranges(self):
        # PPG: each channel auto-ranges independently
        self._auto_range(self.ppg_red_plot, self.buffers.red.ordered())
        self._auto_range(self.ppg_ir_plot, self.buffers.ir.ordered())
        self._auto_range(self.ppg_green_plot, self.buffers.green.ordered())

        # IMU: all channels share one Y-axis per plot
        accel_stack = np.vstack((
            self.buffers.ax.ordered(),
            self.buffers.ay.ordered(),
            self.buffers.az.ordered(),
        ))
        gyro_stack = np.vstack((
            self.buffers.gx.ordered(),
            self.buffers.gy.ordered(),
            self.buffers.gz.ordered(),
        ))
        mag_stack = np.vstack((
            self.buffers.mx.ordered(),
            self.buffers.my.ordered(),
            self.buffers.mz.ordered(),
        ))

        self._auto_range(self.accel_plot[0].getPlotItem(), accel_stack.flatten())
        self._auto_range(self.gyro_plot[0].getPlotItem(), gyro_stack.flatten())
        self._auto_range(self.mag_plot[0].getPlotItem(), mag_stack.flatten())

    @staticmethod
    def _auto_range(plot_item, values):
        if len(values) < 8:
            return
        lower = float(np.percentile(values, 1.0))
        upper = float(np.percentile(values, 99.0))
        if not np.isfinite(lower) or not np.isfinite(upper):
            return
        if upper <= lower:
            center = float(np.mean(values))
            span = max(1.0, float(np.std(values)) * 4.0)
            lower = center - span * 0.5
            upper = center + span * 0.5
        padding = max(1.0, (upper - lower) * 0.1)
        plot_item.setYRange(lower - padding, upper + padding, padding=0.0)

    def _update_metrics(self):
        rate = self.rate_tracker.current_rate()
        self.data_rate_label.setText(f"Rate: {rate:.1f} Hz")
        self.packet_count_label.setText(f"Packets: {self.rate_tracker.total_packets}")
        self.invalid_label.setText(f"Invalid: {self.invalid_count}")
        if self.latest_temp is not None and self.latest_temp == self.latest_temp:
            self.temp_label.setText(f"Temp: {self.latest_temp:.2f}")
        else:
            self.temp_label.setText("Temp: --")


# ===================== Entry Point =====================

def parse_args():
    parser = argparse.ArgumentParser(
        description="Real-time sensor visualization + CSV logging")
    parser.add_argument("--port", type=str, default="COM5",
                        help="Serial port (default: COM5)")
    parser.add_argument("--baud", type=int, default=1000000,
                        help="Baud rate (default: 1000000)")
    parser.add_argument("--sample-rate", type=float, default=250.0,
                        help="Expected packet rate for plot window sizing (default: 250)")
    parser.add_argument("--window-seconds", type=float, default=DISPLAY_WINDOW_SECONDS,
                        help="Visible waveform window in seconds (default: 10)")
    return parser.parse_args()


def main():
    args = parse_args()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = SensorMonitorWindow(
        args.port,
        args.baud,
        args.sample_rate,
        args.window_seconds,
    )
    window.start()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()