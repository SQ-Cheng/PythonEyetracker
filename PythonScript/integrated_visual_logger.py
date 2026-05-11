#!/usr/bin/env python3
"""
Integrated Visual Logger

Merges functionalities of sensor_visual_logger.py and eye_tracker_visual_logger.py.
Uses PyQt5. Start/Stop manages both devices simultaneously. Data is logged to 2 separate CSV files.
"""

import argparse
import configparser
import csv
import json
import os
import queue
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Optional
import wave

import numpy as np
import serial
from serial.tools import list_ports

try:
    import sounddevice as sd
    SOUNDDEVICE_IMPORT_ERROR = None
except Exception as exc:
    sd = None
    SOUNDDEVICE_IMPORT_ERROR = exc

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPainter, QColor, QBrush, QPixmap, QFont, QImage
from PyQt5.QtCore import Qt, QRectF
import pyqtgraph as pg

# ---- Eye Tracker SDK Imports ----
from example_paths import (
    CALIBRATION_PROFILE_DIR as SHARED_CALIBRATION_PROFILE_DIR,
    LOG_DIR,
    add_sdk_root_argument,
    sdk_config_dir,
)
from sdk_wrapper import wrapper


# ===================== Constants =====================
# Sensor
PACKET_SIZE = 64
NUM_FLOATS = 14
SYNC_MARKER = bytes([0x55, 0xAA, 0x55, 0xAA])
SYNC_LEN = 4

DISPLAY_WINDOW_SECONDS = 6.0
PLOT_UPDATE_INTERVAL_MS = 33
METRIC_UPDATE_INTERVAL_MS = 250
Y_LIMIT_UPDATE_INTERVAL_MS = 500
SERIAL_STARTUP_SETTLE_SECONDS = 2.0
RATE_WINDOW_SECONDS = 3.0
TARGET_SERIAL_DESCRIPTION = "Silicon Labs CP210x USB to UART Bridge"
SENSOR_COLUMN_NAMES = [
    'Red', 'IR', 'Green',
    'accX', 'accY', 'accZ',
    'gyrX', 'gyrY', 'gyrZ',
    'magX', 'magY', 'magZ',
    'temp', 'device_timestamp_ms', 'pc_arrival_timestamp'
]

EYE_COLUMN_NAMES = [
    "device_timestamp", "pc_arrival_timestamp", "gaze_x", "gaze_y", "gaze_z",
    "left_pupil_x", "left_pupil_y", "right_pupil_x", "right_pupil_y",
    "left_pupil_diameter_mm", "right_pupil_diameter_mm",
    "left_openness", "right_openness", "left_blink", "right_blink",
    "gyro_timestamp", "gyro_x", "gyro_y", "gyro_z",
]

AUDIO_CHANNELS = 1
AUDIO_DTYPE = "int16"
AUDIO_SAMPLE_WIDTH_BYTES = 2
AUDIO_ANCHOR_INTERVAL_SECONDS = 1.0
AUDIO_SAMPLE_RATE_OPTIONS = (8000, 16000, 22050, 32000, 44100, 48000, 96000)
AUDIO_TIMESTAMP_COLUMN_NAMES = [
    "sample_index",
    "sample_offset_seconds",
    "pc_perf_timestamp",
    "callback_pc_perf_timestamp",
    "pa_input_buffer_adc_time",
    "pa_current_time",
    "pc_clock_offset",
    "block_start_sample_index",
    "frames_in_block",
    "status_flags",
    "clock_source",
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


class AudioDeviceComboBox(QtWidgets.QComboBox):
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

    def clear(self):
        self.write_index = 0
        self.count = 0
        if self.use_numpy:
            self.values.fill(0.0)
        else:
            self.values = [0.0] * self.size

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
class SerialPacketReader:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.packet_size = PACKET_SIZE
        self.serial_port = None
        self.packet_queue = queue.SimpleQueue()
        self._buffer = bytearray()
        self._text_buffer = bytearray()
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._thread_started = False

    def open(self):
        try:
            self.serial_port = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.01)
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
            pc_arrival_timestamp = time.perf_counter()
            try:
                values = struct.unpack('<14f', payload[:NUM_FLOATS * 4])
            except struct.error: continue

            if sum(1 for v in values if v != v) > 3: continue
            device_timestamp_ms = values[13]
            if device_timestamp_ms == device_timestamp_ms and (device_timestamp_ms < 0 or device_timestamp_ms > 4.3e9): continue

            packet = {
                "pc_arrival_timestamp": pc_arrival_timestamp, "red": values[0], "ir": values[1], "green": values[2],
                "ax": values[3], "ay": values[4], "az": values[5], "gx": values[6], "gy": values[7],
                "gz": values[8], "mx": values[9], "my": values[10], "mz": values[11], "temp": values[12],
                "device_timestamp_ms": device_timestamp_ms,
            }
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
        return


# ===================== Audio Component =====================
@dataclass
class AudioBlock:
    audio_bytes: bytes
    frames: int
    sample_index: int
    pc_perf_timestamp: float
    callback_pc_perf_timestamp: float
    pa_input_buffer_adc_time: Optional[float]
    pa_current_time: Optional[float]
    pc_clock_offset: Optional[float]
    status_flags: str
    clock_source: str


class AudioRecorder:
    def __init__(self, device_index, device_name, sample_rate, wav_file, timestamp_file, metadata_file):
        if sd is None:
            raise RuntimeError(f"sounddevice is not available: {SOUNDDEVICE_IMPORT_ERROR}")

        self.device_index = int(device_index)
        self.device_name = str(device_name)
        self.sample_rate = int(sample_rate)
        self.wav_file = wav_file
        self.timestamp_file = timestamp_file
        self.metadata_file = metadata_file

        self.queue = queue.SimpleQueue()
        self._stop_event = threading.Event()
        self._writer_thread = threading.Thread(target=self._write_loop, daemon=True)
        self._stream = None
        self._sample_lock = threading.Lock()
        self._loudness_lock = threading.Lock()
        self._next_sample_index = 0
        self._next_anchor_sample_index = 0
        self._anchor_interval_frames = max(1, int(round(self.sample_rate * AUDIO_ANCHOR_INTERVAL_SECONDS)))

        self.frames_captured = 0
        self.frames_written = 0
        self.blocks_captured = 0
        self.anchors_written = 0
        self.started_pc_perf_timestamp = None
        self.stopped_pc_perf_timestamp = None
        self.latest_loudness_percent = 0
        self.latest_loudness_dbfs = None
        self.latest_loudness_timestamp = None
        self.error = None

    def start(self):
        sd.check_input_settings(
            device=self.device_index,
            channels=AUDIO_CHANNELS,
            samplerate=self.sample_rate,
            dtype=AUDIO_DTYPE,
        )
        self.started_pc_perf_timestamp = time.perf_counter()
        self._writer_thread.start()
        try:
            self._stream = sd.InputStream(
                device=self.device_index,
                channels=AUDIO_CHANNELS,
                samplerate=self.sample_rate,
                dtype=AUDIO_DTYPE,
                callback=self._callback,
            )
            self._stream.start()
        except Exception:
            if self._stream is not None:
                try:
                    self._stream.close()
                except Exception:
                    pass
                self._stream = None
            self._stop_event.set()
            self._writer_thread.join(timeout=2.0)
            raise

    def stop(self):
        if self._stream is not None:
            try:
                self._stream.stop()
            finally:
                self._stream.close()
                self._stream = None

        self.stopped_pc_perf_timestamp = time.perf_counter()
        self._stop_event.set()
        self._writer_thread.join(timeout=5.0)
        if self._writer_thread.is_alive():
            self.error = self.error or "audio writer did not finish flushing"
        self._write_metadata()
        if self.error:
            raise RuntimeError(f"Audio writer failed: {self.error}")

    def _callback(self, indata, frames, time_info, status):
        callback_pc = time.perf_counter()
        pa_adc = self._time_attr(time_info, "inputBufferAdcTime")
        pa_current = self._time_attr(time_info, "currentTime")

        if pa_adc is not None and pa_current is not None:
            pc_clock_offset = callback_pc - pa_current
            block_pc_timestamp = pa_adc + pc_clock_offset
            clock_source = "portaudio_input_adc"
        else:
            pc_clock_offset = None
            block_pc_timestamp = callback_pc - (float(frames) / float(self.sample_rate))
            clock_source = "callback_fallback"

        with self._sample_lock:
            sample_index = self._next_sample_index
            self._next_sample_index += int(frames)
            self.frames_captured += int(frames)
            self.blocks_captured += 1

        audio_bytes = np.asarray(indata, dtype=np.int16).copy(order="C").tobytes()
        status_flags = str(status).strip() if status else ""
        self.queue.put(AudioBlock(
            audio_bytes=audio_bytes,
            frames=int(frames),
            sample_index=sample_index,
            pc_perf_timestamp=float(block_pc_timestamp),
            callback_pc_perf_timestamp=float(callback_pc),
            pa_input_buffer_adc_time=pa_adc,
            pa_current_time=pa_current,
            pc_clock_offset=pc_clock_offset,
            status_flags=status_flags,
            clock_source=clock_source,
        ))

    def _write_loop(self):
        try:
            with wave.open(self.wav_file, "wb") as wav_file, open(self.timestamp_file, "w", newline="") as csvfile:
                wav_file.setnchannels(AUDIO_CHANNELS)
                wav_file.setsampwidth(AUDIO_SAMPLE_WIDTH_BYTES)
                wav_file.setframerate(self.sample_rate)
                writer = csv.writer(csvfile)
                writer.writerow(AUDIO_TIMESTAMP_COLUMN_NAMES)

                while not self._stop_event.is_set() or not self.queue.empty():
                    try:
                        block = self.queue.get(timeout=0.1)
                    except queue.Empty:
                        continue

                    wav_file.writeframes(block.audio_bytes)
                    self.frames_written += block.frames
                    self._update_loudness(block.audio_bytes, block.callback_pc_perf_timestamp)
                    self._write_timestamp_anchors(writer, block)
        except Exception as exc:
            self.error = exc

    def latest_loudness(self):
        with self._loudness_lock:
            return self.latest_loudness_percent, self.latest_loudness_dbfs, self.latest_loudness_timestamp

    def _update_loudness(self, audio_bytes, timestamp):
        samples = np.frombuffer(audio_bytes, dtype=np.int16)
        if samples.size == 0:
            return

        values = samples.astype(np.float32, copy=False)
        rms = float(np.sqrt(np.mean(values * values)))
        if rms <= 0.0:
            dbfs = -90.0
        else:
            dbfs = 20.0 * np.log10(rms / 32768.0)
        dbfs = max(-90.0, min(0.0, float(dbfs)))
        percent = int(round(max(0.0, min(1.0, (dbfs + 60.0) / 60.0)) * 100.0))

        with self._loudness_lock:
            self.latest_loudness_percent = percent
            self.latest_loudness_dbfs = dbfs
            self.latest_loudness_timestamp = float(timestamp)

    def _write_timestamp_anchors(self, writer, block):
        block_end = block.sample_index + block.frames
        while self._next_anchor_sample_index < block_end:
            anchor_sample_index = self._next_anchor_sample_index
            if anchor_sample_index >= block.sample_index:
                sample_delta = anchor_sample_index - block.sample_index
                anchor_pc_timestamp = block.pc_perf_timestamp + (float(sample_delta) / float(self.sample_rate))
                writer.writerow([
                    str(anchor_sample_index),
                    f"{anchor_sample_index / float(self.sample_rate):.9f}",
                    f"{anchor_pc_timestamp:.9f}",
                    f"{block.callback_pc_perf_timestamp:.9f}",
                    self._format_optional_float(block.pa_input_buffer_adc_time),
                    self._format_optional_float(block.pa_current_time),
                    self._format_optional_float(block.pc_clock_offset),
                    str(block.sample_index),
                    str(block.frames),
                    block.status_flags,
                    block.clock_source,
                ])
                self.anchors_written += 1
            self._next_anchor_sample_index += self._anchor_interval_frames

    def _write_metadata(self):
        metadata = {
            "device_index": self.device_index,
            "device_name": self.device_name,
            "sample_rate": self.sample_rate,
            "channels": AUDIO_CHANNELS,
            "dtype": AUDIO_DTYPE,
            "sample_width_bytes": AUDIO_SAMPLE_WIDTH_BYTES,
            "wav_file": self.wav_file,
            "timestamp_file": self.timestamp_file,
            "frames_captured": self.frames_captured,
            "frames_written": self.frames_written,
            "blocks_captured": self.blocks_captured,
            "timestamp_anchors_written": self.anchors_written,
            "anchor_interval_seconds": AUDIO_ANCHOR_INTERVAL_SECONDS,
            "timing_method": "First and per-second sample-index anchors using time.perf_counter() / pc_perf_timestamp.",
            "started_pc_perf_timestamp": self.started_pc_perf_timestamp,
            "stopped_pc_perf_timestamp": self.stopped_pc_perf_timestamp,
            "writer_error": str(self.error) if self.error else "",
        }
        try:
            with open(self.metadata_file, "w") as fp:
                json.dump(metadata, fp, indent=2)
        except Exception as exc:
            print(f"Audio metadata write failed: {exc}")

    @staticmethod
    def _time_attr(time_info, attr):
        try:
            value = getattr(time_info, attr)
        except Exception:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _format_optional_float(value):
        return "" if value is None else f"{float(value):.9f}"


# ===================== CSV Writers =====================
class CSVWriterThread:
    def __init__(self, output_file, column_names, row_formatter):
        self.output_file = output_file
        self.column_names = column_names
        self.row_formatter = row_formatter
        self.queue = queue.SimpleQueue()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self.rows_written = 0
        self.error = None

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            self.error = self.error or "CSV writer did not finish flushing"
        if self.error:
            raise RuntimeError(self.error)

    def push(self, sample):
        self.queue.put(sample)

    def _write_loop(self):
        try:
            with open(self.output_file, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(self.column_names)
                while not self._stop_event.is_set() or not self.queue.empty():
                    try:
                        sample = self.queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    writer.writerow(self.row_formatter(sample))
                    self.rows_written += 1
        except Exception as exc:
            self.error = exc


def format_sensor_csv_row(sample):
    return [
        str(int(sample["red"])),
        str(int(sample["ir"])),
        str(int(sample["green"])),
        f"{sample['ax']:.6f}",
        f"{sample['ay']:.6f}",
        f"{sample['az']:.6f}",
        f"{sample['gx']:.6f}",
        f"{sample['gy']:.6f}",
        f"{sample['gz']:.6f}",
        f"{sample['mx']:.6f}",
        f"{sample['my']:.6f}",
        f"{sample['mz']:.6f}",
        f"{sample['temp']:.6f}",
        f"{sample['device_timestamp_ms']:.3f}",
        f"{sample['pc_arrival_timestamp']:.9f}",
    ]


def format_eye_csv_row(sample):
    return [
        str(int(sample["device_timestamp"])),
        f"{sample['pc_arrival_timestamp']:.9f}",
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
        str(int(sample.get("gyro_timestamp", 0))),
        f"{sample.get('gyro_x', 0.0):.6f}",
        f"{sample.get('gyro_y', 0.0):.6f}",
        f"{sample.get('gyro_z', 0.0):.6f}",
    ]


# ===================== UI Integrated Window =====================
class IntegratedMonitorWindow(QtWidgets.QMainWindow):
    # Eye Tracker SDK Signals
    set_sdk_running_signal = QtCore.pyqtSignal(bool)
    set_scene_image_signal = QtCore.pyqtSignal(QPixmap)
    set_calibration_finish_signal = QtCore.pyqtSignal(int, int, int)

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.theme = THEME_DARK if detect_dark_theme() else THEME_LIGHT
        self.log_dir = os.fspath(LOG_DIR)
        self.calibration_root = os.fspath(SHARED_CALIBRATION_PROFILE_DIR)
        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.calibration_root, exist_ok=True)

        # Data & Buffers
        self.sensor_rate_tracker = PacketRateTracker()
        self.eye_rate_tracker = PacketRateTracker()
        self.eye_gyro_rate_tracker = PacketRateTracker()
        self._last_eye_gyro_timestamp = None
        
        self.sensor_buffers = MultiSeriesBuffer(args.sensor_sample_rate, args.window_seconds)
        
        self.eye_gaze_x_series = RingSeries(args.eye_sample_rate, args.window_seconds, True)
        self.eye_gaze_y_series = RingSeries(args.eye_sample_rate, args.window_seconds, True)
        self.eye_left_pupil_series = RingSeries(args.eye_sample_rate, args.window_seconds, True)
        self.eye_right_pupil_series = RingSeries(args.eye_sample_rate, args.window_seconds, True)
        self.eye_gyro_x_series = RingSeries(args.eye_sample_rate, args.window_seconds, True)
        self.eye_gyro_y_series = RingSeries(args.eye_sample_rate, args.window_seconds, True)
        self.eye_gyro_z_series = RingSeries(args.eye_sample_rate, args.window_seconds, True)

        # Plot loop states
        self.last_plot_update_ms = 0.0
        self.last_metric_update_ms = 0.0
        self.last_range_update_ms = 0.0
        
        self.sensor_latest_temp = None

        # Writers & Reader components
        self.sensor_csv_writer = None
        self.eye_csv_writer = None
        self.audio_recorder = None
        self.sensor_reader = None
        self._preview_lock = threading.Lock()
        self._pending_left_eye_frame = None
        self._pending_right_eye_frame = None

        # Eye tracker Component
        self.eye_sdk = wrapper()
        self.eye_sdk_config_path = os.fspath(sdk_config_dir(args.sdk_root))
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
        self.lbl_eye_gyro_rate = QtWidgets.QLabel("Gyro Rate: -- Hz")

        for lbl in (self.lbl_sensor_rate, self.lbl_sensor_packets, self.lbl_sensor_temp,
                    self.lbl_eye_rate, self.lbl_eye_packets, self.lbl_eye_gyro_rate):
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

        # Audio Settings
        grp_audio = QtWidgets.QGroupBox("Audio Recording")
        grp_audio.setStyleSheet(group_ss)
        grp_audio.setFixedWidth(sidebar_width)
        lo_audio = QtWidgets.QVBoxLayout(grp_audio)
        self.combo_audio_device = AudioDeviceComboBox()
        self.combo_audio_device.setStyleSheet(combo_ss)
        self.combo_audio_rate = QtWidgets.QComboBox()
        self.combo_audio_rate.setStyleSheet(combo_ss)
        self.combo_audio_device.popup_requested.connect(self._populate_audio_devices)
        self.combo_audio_device.currentIndexChanged.connect(self._populate_audio_sample_rates)
        lo_audio.addWidget(QtWidgets.QLabel("Microphone:")); lo_audio.addWidget(self.combo_audio_device)
        lo_audio.addWidget(QtWidgets.QLabel("Sample Rate:")); lo_audio.addWidget(self.combo_audio_rate)
        sidebar.addWidget(grp_audio)
        self._populate_audio_devices()

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
        self.audio_loudness_bar = QtWidgets.QProgressBar()
        self.audio_loudness_bar.setRange(0, 100)
        self.audio_loudness_bar.setValue(0)
        self.audio_loudness_bar.setTextVisible(True)
        self.audio_loudness_bar.setFormat("Mic: -- dBFS")
        self.audio_loudness_bar.setFixedHeight(18)
        self.audio_loudness_bar.setMaximumWidth(220)
        self.audio_loudness_bar.setStyleSheet(
            f"QProgressBar {{ background: {t['value_bg']}; color: {t['fg']}; border: 1px solid {t['group_border']}; border-radius: 4px; text-align: center; font-size: 11px; }}"
            f"QProgressBar::chunk {{ background: {t['value_fg']}; border-radius: 3px; }}"
        )
        
        lyt_left_eye = QtWidgets.QVBoxLayout(); lyt_left_eye.addWidget(QtWidgets.QLabel("Left Eye"), alignment=Qt.AlignCenter); lyt_left_eye.addWidget(self.labelLeftEye)
        lyt_right_eye = QtWidgets.QVBoxLayout(); lyt_right_eye.addWidget(QtWidgets.QLabel("Right Eye"), alignment=Qt.AlignCenter); lyt_right_eye.addWidget(self.labelRightEye)
        lyt_right_eye.addWidget(self.audio_loudness_bar)
        
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

        self.eye_gyro_plot_widget = pg.PlotWidget()
        eye_gyro_it = self.eye_gyro_plot_widget.getPlotItem()
        eye_gyro_it.setTitle("Eye Tracker Gyroscope")
        eye_gyro_it.setLabel("left", "dps")
        eye_gyro_it.setXRange(-self.args.window_seconds, 0.0, padding=0.0)
        eye_gyro_it.setXLink(self.ppg_red_plot)
        eye_gyro_it.showGrid(x=True, y=True, alpha=t['grid_alpha'])
        eye_gyro_it.addLegend(offset=(6,6))
        self.eye_gyro_x_curve = self.eye_gyro_plot_widget.plot(pen=pg.mkPen("#E63946", width=1.5), name="X")
        self.eye_gyro_y_curve = self.eye_gyro_plot_widget.plot(pen=pg.mkPen("#457B9D", width=1.5), name="Y")
        self.eye_gyro_z_curve = self.eye_gyro_plot_widget.plot(pen=pg.mkPen("#2A9D8F", width=1.5), name="Z")

        imu_column = QtWidgets.QVBoxLayout()
        imu_column.setSpacing(8)
        imu_column.addWidget(self.accel_plot[0])
        imu_column.addWidget(self.gyro_plot[0])
        imu_column.addWidget(self.mag_plot[0])

        eye_wave_column = QtWidgets.QVBoxLayout()
        eye_wave_column.setSpacing(8)
        eye_wave_column.addWidget(self.gaze_plot_widget)
        eye_wave_column.addWidget(self.pupil_plot_widget)
        eye_wave_column.addWidget(self.eye_gyro_plot_widget)

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

    def _populate_audio_devices(self):
        current_device = self.combo_audio_device.currentData()
        self.combo_audio_device.blockSignals(True)
        self.combo_audio_device.clear()

        if sd is None:
            self.combo_audio_device.addItem("sounddevice not installed", None)
            self.combo_audio_device.blockSignals(False)
            self._populate_audio_sample_rates()
            self._set_audio_controls_enabled(False)
            return

        try:
            devices = sd.query_devices()
            hostapis = sd.query_hostapis()
        except Exception as exc:
            self.combo_audio_device.addItem(f"Audio query failed: {exc}", None)
            self.combo_audio_device.blockSignals(False)
            self._populate_audio_sample_rates()
            self._set_audio_controls_enabled(False)
            return

        input_devices = self._filtered_audio_input_devices(devices, hostapis)
        for item in input_devices:
            self.combo_audio_device.addItem(item["label"], item["index"])

        if not input_devices:
            self.combo_audio_device.addItem("No input devices found", None)
        else:
            preferred_device = current_device
            if preferred_device is None:
                preferred_device = self._default_audio_input_device()
            preferred_index = self.combo_audio_device.findData(preferred_device)
            if preferred_index >= 0:
                self.combo_audio_device.setCurrentIndex(preferred_index)
            else:
                self.combo_audio_device.setCurrentIndex(0)

        self.combo_audio_device.blockSignals(False)
        self._populate_audio_sample_rates()
        self._set_audio_controls_enabled(not self.system_running)

    def _filtered_audio_input_devices(self, devices, hostapis):
        rows = []
        default_device = self._default_audio_input_device()
        default_key = None
        if default_device is not None and 0 <= default_device < len(devices):
            default_key = self._audio_device_dedupe_key(
                self._audio_display_name(str(devices[default_device].get("name", "")))
            )

        for index, device in enumerate(devices):
            try:
                input_channels = int(device.get("max_input_channels", 0))
            except Exception:
                input_channels = 0
            if input_channels <= 0:
                continue

            raw_name = str(device.get("name", "Unknown microphone")).strip()
            hostapi_name = self._audio_hostapi_name(device, hostapis)
            raw_name_lower = raw_name.casefold()
            if "loopback" in raw_name_lower:
                continue

            rows.append({
                "index": int(index),
                "raw_name": raw_name,
                "display_name": self._audio_display_name(raw_name),
                "hostapi_name": hostapi_name,
                "input_channels": input_channels,
                "default_samplerate": float(device.get("default_samplerate", 0.0) or 0.0),
                "is_default": default_device == int(index),
                "is_wasapi": "wasapi" in hostapi_name.casefold(),
                "is_windows_api": any(token in hostapi_name.casefold() for token in ("wasapi", "mme", "directsound")),
            })
            if default_key and self._audio_device_dedupe_key(rows[-1]["display_name"]) == default_key:
                rows[-1]["is_default"] = True

        windows_rows = [row for row in rows if row["is_windows_api"]]
        if windows_rows:
            rows = windows_rows

        wasapi_rows = [row for row in rows if row["is_wasapi"]]
        if wasapi_rows:
            rows = wasapi_rows

        deduped = {}
        for row in rows:
            key = self._audio_device_dedupe_key(row["display_name"])
            existing = deduped.get(key)
            if existing is None or self._audio_device_sort_key(row) < self._audio_device_sort_key(existing):
                deduped[key] = row

        result = sorted(deduped.values(), key=lambda row: (not row["is_default"], row["display_name"].casefold()))
        for row in result:
            label = row["display_name"]
            if row["is_default"]:
                label += " (Default)"
            row["label"] = label
        return result

    def _audio_hostapi_name(self, device, hostapis):
        try:
            return str(hostapis[int(device.get("hostapi", -1))].get("name", ""))
        except Exception:
            return ""

    def _audio_display_name(self, raw_name):
        name = raw_name.strip()
        for suffix in (" (WASAPI)", " (Windows WASAPI)", " (DirectSound)", " (MME)"):
            if name.endswith(suffix):
                name = name[:-len(suffix)].strip()
        if name.casefold().startswith("microphone array ("):
            return "Microphone Array " + name[len("microphone array "):]
        return name

    def _audio_device_dedupe_key(self, display_name):
        key = display_name.casefold()
        for token in ("default - ", "communications - "):
            if key.startswith(token):
                key = key[len(token):]
        return " ".join(key.replace("_", " ").split())

    def _audio_device_sort_key(self, row):
        return (
            not row["is_default"],
            not row["is_wasapi"],
            -row["input_channels"],
            row["display_name"].casefold(),
            row["index"],
        )

    def _populate_audio_sample_rates(self, _index=None):
        if not hasattr(self, "combo_audio_rate"):
            return

        current_rate = self.combo_audio_rate.currentData()
        selected_device = self.combo_audio_device.currentData() if hasattr(self, "combo_audio_device") else None
        self.combo_audio_rate.blockSignals(True)
        self.combo_audio_rate.clear()

        if sd is None:
            self.combo_audio_rate.addItem("Unavailable", None)
            self.combo_audio_rate.blockSignals(False)
            self._set_audio_controls_enabled(False)
            return

        if selected_device is None:
            self.combo_audio_rate.addItem("No microphone", None)
            self.combo_audio_rate.blockSignals(False)
            self._set_audio_controls_enabled(False)
            return

        supported_rates = self._supported_audio_sample_rates(int(selected_device))
        if not supported_rates:
            self.combo_audio_rate.addItem("No supported rates", None)
            self.combo_audio_rate.blockSignals(False)
            self._set_audio_controls_enabled(False)
            return

        for rate in supported_rates:
            self.combo_audio_rate.addItem(f"{rate} Hz", int(rate))

        preferred_rate = current_rate or int(self.args.audio_sample_rate)
        if self.combo_audio_rate.findData(preferred_rate) < 0:
            preferred_rate = int(self.args.audio_sample_rate)
        if self.combo_audio_rate.findData(preferred_rate) < 0:
            preferred_rate = 48000
        preferred_index = self.combo_audio_rate.findData(preferred_rate)
        if preferred_index < 0:
            preferred_index = 0
        self.combo_audio_rate.setCurrentIndex(preferred_index)
        self.combo_audio_rate.blockSignals(False)
        self._set_audio_controls_enabled(not self.system_running)

    def _supported_audio_sample_rates(self, device_index):
        try:
            device_info = sd.query_devices(device_index)
            default_rate = int(round(float(device_info.get("default_samplerate", 0))))
        except Exception:
            default_rate = 0

        candidate_rates = set(AUDIO_SAMPLE_RATE_OPTIONS)
        if default_rate > 0:
            candidate_rates.add(default_rate)

        supported_rates = []
        for rate in sorted(candidate_rates):
            try:
                sd.check_input_settings(
                    device=device_index,
                    channels=AUDIO_CHANNELS,
                    samplerate=int(rate),
                    dtype=AUDIO_DTYPE,
                )
            except Exception:
                continue
            supported_rates.append(int(rate))
        return supported_rates

    def _default_audio_input_device(self):
        try:
            default_device = sd.default.device
            if isinstance(default_device, (tuple, list)):
                default_input = int(default_device[0])
            else:
                default_input = int(default_device)
            return default_input if default_input >= 0 else None
        except Exception:
            return None

    def _set_audio_controls_enabled(self, enabled):
        dependency_ok = sd is not None
        device_ok = hasattr(self, "combo_audio_device") and self.combo_audio_device.currentData() is not None
        rate_ok = hasattr(self, "combo_audio_rate") and self.combo_audio_rate.currentData() is not None
        if hasattr(self, "combo_audio_device"):
            self.combo_audio_device.setEnabled(enabled and dependency_ok and device_ok)
        if hasattr(self, "combo_audio_rate"):
            self.combo_audio_rate.setEnabled(enabled and dependency_ok and device_ok and rate_ok)

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

    def _connect_signals(self):
        self.btn_start.clicked.connect(self._on_start_all)
        self.btn_stop.clicked.connect(self._on_stop_all)
        self.btn_cal_start.clicked.connect(self._on_start_calibration)
        self.btn_cal_stop.clicked.connect(self._on_stop_calibration)
        self.btn_refresh_profiles.clicked.connect(self._refresh_calibration_profiles)
        self.btn_load_profile.clicked.connect(self._load_profile_from_button)
        self.btn_save_profile.clicked.connect(self._save_current_calibration_profile)
        self.combo_calibration_profile.editTextChanged.connect(self._update_calibration_profile_controls)

        self.set_sdk_running_signal.connect(self._on_set_sdk_running)
        self.set_scene_image_signal.connect(self._display_scene_image)
        self.set_calibration_finish_signal.connect(self._on_set_calibration_finish)

    # ---- Eye Tracker Callbacks ----
    def wants_preview_frame(self, eye):
        with self._preview_lock:
            if eye == 1:
                return self._pending_left_eye_frame is None
            if eye == 2:
                return self._pending_right_eye_frame is None
        return False

    def handle_eye_preview_frame(self, eye, frame_bytes, width, height, device_timestamp, pc_arrival_timestamp):
        frame = (
            bytes(frame_bytes),
            int(width),
            int(height),
            int(device_timestamp),
            float(pc_arrival_timestamp),
        )
        with self._preview_lock:
            if eye == 1:
                self._pending_left_eye_frame = frame
            elif eye == 2:
                self._pending_right_eye_frame = frame

    def _process_preview_frames(self):
        with self._preview_lock:
            left_frame = self._pending_left_eye_frame
            right_frame = self._pending_right_eye_frame
            self._pending_left_eye_frame = None
            self._pending_right_eye_frame = None

        if left_frame:
            self._display_eye_preview_frame(self.labelLeftEye, left_frame)
        if right_frame:
            self._display_eye_preview_frame(self.labelRightEye, right_frame)

    def _display_eye_preview_frame(self, label, frame):
        frame_bytes, width, height, _device_timestamp, _pc_arrival_timestamp = frame
        try:
            image = QImage(frame_bytes, int(width), int(height), QImage.Format_Indexed8)
            pixmap = QPixmap.fromImage(image.scaled(160, 120, Qt.IgnoreAspectRatio, Qt.SmoothTransformation))
            label.setPixmap(pixmap)
        except Exception as exc:
            print(f"Eye preview update failed: {exc}")

    def _display_scene_image(self, image):
        painter = QPainter(image)
        painter.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        c = QColor(); c.setGreen(255)
        painter.setBrush(QBrush(c))
        rect = QRectF(self.cur_gaze_x - 15, self.cur_gaze_y - 15, 30, 30)
        painter.drawEllipse(rect)
        self.labelSceneImage.setPixmap(image)
    
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
            with self._preview_lock:
                self._pending_left_eye_frame = None
                self._pending_right_eye_frame = None
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

    def _start_audio_recording(self, ts_str):
        if sd is None:
            QtWidgets.QMessageBox.warning(
                self,
                "Warning",
                f"Audio recording requires the 'sounddevice' package.\nImport error: {SOUNDDEVICE_IMPORT_ERROR}",
            )
            return False

        device_index = self.combo_audio_device.currentData()
        sample_rate = self.combo_audio_rate.currentData()
        if device_index is None:
            QtWidgets.QMessageBox.warning(self, "Warning", "No microphone input device is selected.")
            return False
        if sample_rate is None:
            QtWidgets.QMessageBox.warning(self, "Warning", "No supported audio sample rate is selected.")
            return False

        wav_file = os.path.join(self.log_dir, f"audio_{ts_str}.wav")
        timestamp_file = os.path.join(self.log_dir, f"audio_{ts_str}_timestamps.csv")
        metadata_file = os.path.join(self.log_dir, f"audio_{ts_str}_metadata.json")
        recorder = AudioRecorder(
            device_index=device_index,
            device_name=self.combo_audio_device.currentText(),
            sample_rate=int(sample_rate),
            wav_file=wav_file,
            timestamp_file=timestamp_file,
            metadata_file=metadata_file,
        )

        try:
            recorder.start()
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Warning", f"Audio recording failed to start:\n{exc}")
            return False

        self.audio_recorder = recorder
        self._set_audio_controls_enabled(False)
        return True

    def _stop_audio_recording(self):
        recorder = self.audio_recorder
        self.audio_recorder = None
        if recorder is None:
            self._set_audio_controls_enabled(True)
            return None

        try:
            recorder.stop()
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Warning", f"Audio recording did not stop cleanly:\n{exc}")
        finally:
            self._set_audio_controls_enabled(True)
        return recorder

    def _on_start_all(self):
        if self.system_running:
            return
        self.btn_start.setEnabled(False)
        self._populate_sensor_ports()
        self._populate_audio_devices()
        self.sensor_reader = None
        self.eye_sdk_running = False
        self.audio_recorder = None

        # Prepare Logging Dirs
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")

        try:
            # Prepare sensor before active recording begins.
            selected_port = self.combo_port.currentData()
            if not selected_port:
                QtWidgets.QMessageBox.warning(self, "Warning", f"No '{TARGET_SERIAL_DESCRIPTION}' serial port found.")
            else:
                self.sensor_reader = SerialPacketReader(selected_port, int(self.combo_baud.currentText()))
            if self.sensor_reader and self.sensor_reader.open():
                time.sleep(SERIAL_STARTUP_SETTLE_SECONDS)
                self.sensor_reader.serial_port.reset_input_buffer()
                self.sensor_reader.serial_port.reset_output_buffer()
                self.sensor_csv_writer = CSVWriterThread(
                    os.path.join(self.log_dir, f"sensor_{ts_str}.csv"),
                    SENSOR_COLUMN_NAMES,
                    format_sensor_csv_row,
                )
                self.sensor_csv_writer.start()
                if self.sensor_reader.start():
                    pass
                else:
                    self.sensor_csv_writer.stop()
                    self.sensor_csv_writer = None
                    self.sensor_reader.stop()
                    self.sensor_reader = None
                    QtWidgets.QMessageBox.warning(self, "Warning", "Sensor reader thread failed to start.")
            elif selected_port:
                QtWidgets.QMessageBox.warning(self, "Warning", "Sensor failed to connect.")

            if not self._start_audio_recording(ts_str):
                self._cleanup_start_failure()
                return

            self.system_running = True
            if self.sensor_reader:
                self.sensor_reader.send_command("s\n")

            # Start Eye Tracker
            pwd = self._read_pwd()
            if self.eye_sdk.connect_softdog(pwd) == 0:
                env = self.combo_env.currentData()
                res = self.combo_res.currentData()
                self.scene_width, self.scene_height = (1280, 960) if res == 201 else (1280, 720) if res == 202 else (800, 600) if res == 203 else (1920, 1080)

                if self.eye_sdk.start(env, res, self.scene_width, self.scene_height) == 0:
                    self._on_set_sdk_running(True)
                    self._load_selected_calibration_profile(show_message=False)
                    self.eye_csv_writer = CSVWriterThread(
                        os.path.join(self.log_dir, f"eye_{ts_str}.csv"),
                        EYE_COLUMN_NAMES,
                        format_eye_csv_row,
                    )
                    self.eye_csv_writer.start()
                else:
                    self._on_set_sdk_running(False)
                    QtWidgets.QMessageBox.warning(self, "Warning", "Eye tracker start failed.")
            else:
                self._on_set_sdk_running(False)
                QtWidgets.QMessageBox.warning(self, "Warning", "Eye tracker dog connect failed.")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to start logging session:\n{exc}")
            self._on_stop_all()
            return

        self.btn_stop.setEnabled(True)
        self.timer.start(PLOT_UPDATE_INTERVAL_MS)

    def _cleanup_start_failure(self):
        if self.sensor_reader:
            try:
                self.sensor_reader.stop()
            except Exception as exc:
                print(f"Sensor reader cleanup failed: {exc}")
            self.sensor_reader = None
        if self.sensor_csv_writer:
            try:
                self.sensor_csv_writer.stop()
            except Exception as exc:
                print(f"Sensor CSV writer cleanup failed: {exc}")
            self.sensor_csv_writer = None
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.system_running = False
        self._set_audio_controls_enabled(True)

    def _on_stop_all(self):
        self.system_running = False
        self.timer.stop()
        self.btn_stop.setEnabled(False)
        self.btn_start.setEnabled(True)
        self.btn_cal_start.setEnabled(False)
        self.btn_cal_stop.setEnabled(False)
        self._update_calibration_profile_controls()

        sensor_writer = self.sensor_csv_writer
        eye_writer = self.eye_csv_writer

        # Stop Sensor
        if self.sensor_reader:
            try:
                self.sensor_reader.send_command("e\n")
            except Exception:
                pass
            try:
                self.sensor_reader.stop()
            except Exception as exc:
                print(f"Sensor reader stop failed: {exc}")
            self.sensor_reader = None
        if self.sensor_csv_writer:
            try:
                self.sensor_csv_writer.stop()
            except Exception as exc:
                print(f"Sensor CSV writer stop failed: {exc}")
            self.sensor_csv_writer = None
        
        # Stop Eye
        if self.calibration_is_running:
            try:
                self._on_stop_calibration()
            except Exception as exc:
                print(f"Calibration stop failed: {exc}")
        try:
            self.eye_sdk.stop()
        except Exception as exc:
            print(f"Eye tracker stop failed: {exc}")
        if self.eye_csv_writer:
            try:
                self.eye_csv_writer.stop()
            except Exception as exc:
                print(f"Eye CSV writer stop failed: {exc}")
        self.eye_sdk_running = False
        self.eye_csv_writer = None

        # Stop Audio last so the WAV covers the full shutdown boundary.
        audio_recorder = self._stop_audio_recording()
        
        print("\n===== Logging Session Summary =====")
        print(f"Sensor Total Packets:    {self.sensor_rate_tracker.total_packets}")
        if sensor_writer:
            print(f"Sensor Log Output:       {sensor_writer.output_file}")
        print(f"Eye Tracker Data Frames: {self.eye_packet_count}")
        if eye_writer:
            print(f"Eye Tracker Log Output:  {eye_writer.output_file}")
        if audio_recorder:
            print(f"Audio Frames Written:    {audio_recorder.frames_written}")
            print(f"Audio WAV Output:        {audio_recorder.wav_file}")
            print(f"Audio Timestamp Output:  {audio_recorder.timestamp_file}")
            print(f"Audio Metadata Output:   {audio_recorder.metadata_file}")
        print("===================================\n")

    # ---- Timer Update ----
    def _on_timer(self):
        now_ms = time.perf_counter() * 1000
        self._process_preview_frames()

        # Drain Sensor
        if self.sensor_reader:
            red_b, ir_b, green_b, ax_b, ay_b, az_b, gx_b, gy_b, gz_b, mx_b, my_b, mz_b = ([] for _ in range(12))
            while True:
                try: p = self.sensor_reader.packet_queue.get_nowait()
                except queue.Empty: break

                self.sensor_rate_tracker.push(p["pc_arrival_timestamp"])
                red_b.append(p["red"]); ir_b.append(p["ir"]); green_b.append(p["green"])
                ax_b.append(p["ax"]); ay_b.append(p["ay"]); az_b.append(p["az"])
                gx_b.append(p["gx"]); gy_b.append(p["gy"]); gz_b.append(p["gz"])
                mx_b.append(p["mx"]); my_b.append(p["my"]); mz_b.append(p["mz"])
                self.sensor_latest_temp = p["temp"]
                if self.sensor_csv_writer:
                    self.sensor_csv_writer.push(p)
            if red_b:
                self.sensor_buffers.red.append(red_b); self.sensor_buffers.ir.append(ir_b); self.sensor_buffers.green.append(green_b)
                self.sensor_buffers.ax.append(ax_b); self.sensor_buffers.ay.append(ay_b); self.sensor_buffers.az.append(az_b)
                self.sensor_buffers.gx.append(gx_b); self.sensor_buffers.gy.append(gy_b); self.sensor_buffers.gz.append(gz_b)
                self.sensor_buffers.mx.append(mx_b); self.sensor_buffers.my.append(my_b); self.sensor_buffers.mz.append(mz_b)

        # Drain Eye
        if self.eye_sdk_running:
            last_eye_sample = None
            while True:
                try: s = self.eye_sdk.data_queue.get_nowait()
                except queue.Empty: break
                self.eye_rate_tracker.push(s["pc_arrival_timestamp"])
                self.eye_packet_count += 1
                last_eye_sample = s
                self.eye_gaze_x_series.append([s["gaze_x"]])
                self.eye_gaze_y_series.append([s["gaze_y"]])
                self.eye_left_pupil_series.append([s["left_pupil_diameter_mm"]])
                self.eye_right_pupil_series.append([s["right_pupil_diameter_mm"]])
                self.eye_gyro_x_series.append([s.get("gyro_x", 0.0)])
                self.eye_gyro_y_series.append([s.get("gyro_y", 0.0)])
                self.eye_gyro_z_series.append([s.get("gyro_z", 0.0)])
                gyro_timestamp = s.get("gyro_timestamp")
                if gyro_timestamp and gyro_timestamp != self._last_eye_gyro_timestamp:
                    self._last_eye_gyro_timestamp = gyro_timestamp
                    self.eye_gyro_rate_tracker.push(s["pc_arrival_timestamp"])
                if self.eye_csv_writer: self.eye_csv_writer.push(s)
            if last_eye_sample:
                self._display_gaze_data(last_eye_sample["gaze_x"], last_eye_sample["gaze_y"])
                self._display_pupil_data(
                    last_eye_sample["left_pupil_x"],
                    last_eye_sample["left_pupil_y"],
                    last_eye_sample["right_pupil_x"],
                    last_eye_sample["right_pupil_y"],
                )

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

            gyro_x = self.eye_gyro_x_series.ordered()
            if len(gyro_x) > 0:
                gyro_t = self.eye_gyro_x_series.x_axis(len(gyro_x))
                self.eye_gyro_x_curve.setData(gyro_t, gyro_x)
                self.eye_gyro_y_curve.setData(gyro_t, self.eye_gyro_y_series.ordered())
                self.eye_gyro_z_curve.setData(gyro_t, self.eye_gyro_z_series.ordered())

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
        eye_gyr_s = np.vstack((self.eye_gyro_x_series.ordered(), self.eye_gyro_y_series.ordered(), self.eye_gyro_z_series.ordered())).flatten()
        if len(eye_gyr_s) > 0: self._auto_range(self.eye_gyro_plot_widget.getPlotItem(), eye_gyr_s)

    def _update_metrics(self):
        self.lbl_sensor_rate.setText(f"Sen Rate: {self.sensor_rate_tracker.current_rate():.1f} Hz")
        self.lbl_sensor_packets.setText(f"Sen Pkts: {self.sensor_rate_tracker.total_packets}")
        tval = self.sensor_latest_temp
        self.lbl_sensor_temp.setText(f"Sen Temp: {tval:.2f}" if tval and tval == tval else "Sen Temp: --")
        self.lbl_eye_rate.setText(f"Eye Rate: {self.eye_rate_tracker.current_rate():.1f} Hz")
        self.lbl_eye_packets.setText(f"Eye Pkts: {self.eye_packet_count}")
        self.lbl_eye_gyro_rate.setText(f"Gyro Rate: {self.eye_gyro_rate_tracker.current_rate():.1f} Hz")
        self._update_audio_loudness()

    def _update_audio_loudness(self):
        if not hasattr(self, "audio_loudness_bar"):
            return

        if not self.audio_recorder:
            self.audio_loudness_bar.setValue(0)
            self.audio_loudness_bar.setFormat("Mic: -- dBFS")
            return

        level, dbfs, timestamp = self.audio_recorder.latest_loudness()
        if dbfs is None or timestamp is None or time.perf_counter() - timestamp > 1.0:
            self.audio_loudness_bar.setValue(0)
            self.audio_loudness_bar.setFormat("Mic: -- dBFS")
            return

        self.audio_loudness_bar.setValue(level)
        self.audio_loudness_bar.setFormat(f"Mic: {dbfs:.1f} dBFS")

    def closeEvent(self, event):
        if self.system_running: self._on_stop_all()
        super().closeEvent(event)


def parse_args():
    parser = argparse.ArgumentParser()
    add_sdk_root_argument(parser)
    parser.add_argument("--port", type=str, default="COM5")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--sensor-sample-rate", type=float, default=250.0)
    parser.add_argument("--eye-sample-rate", type=float, default=120.0)
    parser.add_argument("--audio-sample-rate", type=int, default=48000)
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
