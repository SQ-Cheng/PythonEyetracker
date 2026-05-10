#!/usr/bin/env python3
"""
Terminal-only sync capture.

The capture phase intentionally does no clock alignment. Each device writes only
its native device timestamp and the PC perf-counter timestamp observed when the
frame or packet reached Python.
"""

import argparse
import configparser
import csv
import ctypes
import os
import queue
import signal
import struct
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime

import numpy as np
import serial
from serial.tools import list_ports

from example_paths import LOG_DIR, add_sdk_root_argument, sdk_config_dir
from sdk_types import PY_7I_RESOLUTION
from sdk_wrapper import wrapper


PACKET_SIZE = 64
SYNC_MARKER = bytes([0x55, 0xAA, 0x55, 0xAA])
SYNC_LEN = len(SYNC_MARKER)
PPG_FLOAT_COUNT = 14

DEFAULT_CAPTURE_SECONDS = 40.0
DEFAULT_BAUD = 1000000
DEFAULT_ENVIRONMENT = 301
DEFAULT_RESOLUTION = 202
SERIAL_STARTUP_SECONDS = 2.0
TARGET_SERIAL_DESCRIPTION = "Silicon Labs CP210x USB to UART Bridge"

SENSOR_COLUMNS = [
    "sensor_device_timestamp_ms",
    "sensor_pc_timestamp",
    "red",
    "ir",
    "green",
]

EYE_COLUMNS = [
    "eye_device_timestamp",
    "eye_pc_timestamp",
    "mean_brightness",
]


@dataclass(frozen=True)
class SensorSample:
    device_timestamp_ms: float
    pc_timestamp: float
    red: float
    ir: float
    green: float


@dataclass(frozen=True)
class EyeFrame:
    frame_bytes: bytes
    width: int
    height: int
    device_timestamp: int
    pc_timestamp: float


def read_pwd(config_dir):
    config = configparser.ConfigParser()
    config.read(os.path.join(config_dir, "config.ini"))
    return config.get("softdog", "pwd", fallback="").encode("utf-8")


def resolve_scene_dimensions(resolution_code):
    if resolution_code == PY_7I_RESOLUTION.P1280_960.value:
        return 1280, 960
    if resolution_code == PY_7I_RESOLUTION.P1280_720.value:
        return 1280, 720
    if resolution_code == PY_7I_RESOLUTION.P800_600.value:
        return 800, 600
    if resolution_code == PY_7I_RESOLUTION.P1920_1080.value:
        return 1920, 1080
    return 1280, 720


def autodetect_sensor_port():
    for port in list_ports.comports():
        if TARGET_SERIAL_DESCRIPTION in (port.description or ""):
            return port.device
    return None


class CsvWriter:
    def __init__(self, output_file, columns, format_row):
        self.output_file = output_file
        self.columns = columns
        self.format_row = format_row
        self.queue = queue.SimpleQueue()
        self.rows_written = 0
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._write_loop, daemon=True)

    def start(self):
        self._thread.start()

    def write(self, row):
        self.queue.put(row)

    def stop(self):
        self._stop_event.set()
        self._thread.join()

    def _write_loop(self):
        with open(self.output_file, "w", newline="") as fp:
            writer = csv.writer(fp)
            writer.writerow(self.columns)
            while not self._stop_event.is_set() or not self.queue.empty():
                try:
                    row = self.queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                writer.writerow(self.format_row(row))
                self.rows_written += 1


class SensorReader:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.samples = queue.SimpleQueue()
        self.serial_port = None
        self._buffer = bytearray()
        self._text_buffer = bytearray()
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        self.serial_port = serial.Serial(self.port, self.baud, timeout=0.05)
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self._thread = None

    def send(self, command):
        if not self.serial_port or not self.serial_port.is_open:
            return
        with self._write_lock:
            self.serial_port.write(command.encode("ascii"))
            self.serial_port.flush()

    def reset_buffers(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

    def _read_loop(self):
        while not self._stop_event.is_set():
            try:
                incoming = self.serial_port.read(self.serial_port.in_waiting or 1)
            except Exception:
                break
            if not incoming:
                continue
            receive_time = time.perf_counter()
            self._buffer.extend(incoming)
            self._consume_packets(receive_time)

    def _consume_packets(self, receive_time):
        while True:
            marker_index = self._buffer.find(SYNC_MARKER)
            if marker_index < 0:
                self._consume_non_packet_bytes()
                return
            if marker_index > 0:
                self._consume_text(bytes(self._buffer[:marker_index]))
                del self._buffer[:marker_index]
            if len(self._buffer) < PACKET_SIZE:
                return

            payload = bytes(self._buffer[SYNC_LEN:PACKET_SIZE])
            del self._buffer[:PACKET_SIZE]
            try:
                values = struct.unpack("<14f", payload[:PPG_FLOAT_COUNT * 4])
            except struct.error:
                continue

            device_timestamp_ms = values[13]
            if not np.isfinite(device_timestamp_ms) or device_timestamp_ms < 0.0:
                continue

            self.samples.put(SensorSample(
                device_timestamp_ms=device_timestamp_ms,
                pc_timestamp=receive_time,
                red=values[0],
                ir=values[1],
                green=values[2],
            ))

    def _consume_non_packet_bytes(self):
        keep = self._marker_prefix_len()
        if keep:
            self._consume_text(bytes(self._buffer[:-keep]))
            self._buffer[:] = self._buffer[-keep:]
        else:
            self._consume_text(bytes(self._buffer))
            self._buffer.clear()

    def _marker_prefix_len(self):
        max_len = min(len(self._buffer), SYNC_LEN - 1)
        for size in range(max_len, 0, -1):
            if self._buffer[-size:] == SYNC_MARKER[:size]:
                return size
        return 0

    def _consume_text(self, data):
        if not data:
            return
        self._text_buffer.extend(data)
        while b"\n" in self._text_buffer:
            line, _, rest = self._text_buffer.partition(b"\n")
            self._text_buffer = bytearray(rest)
            line_text = line.decode("ascii", errors="ignore").strip()
            if line_text:
                print(f"[sensor] {line_text}")


class RightEyeBrightnessSink:
    def __init__(self, writer):
        self.writer = writer
        self._frames = queue.Queue()
        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._process_frames, daemon=True)
        self._worker.start()

    def handle_right_eye_frame(self, image, width, height, eye_device_timestamp, eye_pc_timestamp):
        width = int(width)
        height = int(height)
        byte_count = width * height
        if byte_count <= 0:
            return

        if isinstance(image, (bytes, bytearray)):
            frame_bytes = bytes(image[:byte_count])
        else:
            frame_bytes = ctypes.string_at(image, byte_count)
        if len(frame_bytes) != byte_count:
            return

        self._frames.put(EyeFrame(
            frame_bytes=frame_bytes,
            width=width,
            height=height,
            device_timestamp=int(eye_device_timestamp),
            pc_timestamp=float(eye_pc_timestamp),
        ))

    def stop(self):
        self._stop_event.set()
        self._worker.join()

    def _process_frames(self):
        while not self._stop_event.is_set() or not self._frames.empty():
            try:
                frame = self._frames.get(timeout=0.1)
            except queue.Empty:
                continue
            pixels = np.frombuffer(frame.frame_bytes, dtype=np.uint8)
            if pixels.size != frame.width * frame.height:
                continue
            self.writer.write({
                "device_timestamp": frame.device_timestamp,
                "pc_timestamp": frame.pc_timestamp,
                "mean_brightness": float(pixels.mean()),
            })


class SyncCaptureSession:
    def __init__(self, args):
        self.args = args
        self.sdk = wrapper()
        self.sensor = None
        self.sensor_writer = None
        self.eye_writer = None
        self.eye_sink = None
        self.output_dir = ""
        self.stop_requested = False
        self.sensor_streaming = False
        self.sensor_syncing = False
        self.eye_started = False

    def request_stop(self):
        self.stop_requested = True

    def run(self):
        self._prepare_output_dir()
        self._start_writers()
        try:
            self._start_sensor_stream()
            self._start_eye_tracker()
            self._start_sensor_sync()
            self._capture_loop()
        finally:
            self._cleanup()
        self._print_summary()

    def _prepare_output_dir(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = os.path.join(os.fspath(LOG_DIR), f"sync_capture_{timestamp}")
        os.makedirs(self.output_dir, exist_ok=True)

    def _start_writers(self):
        self.sensor_writer = CsvWriter(
            os.path.join(self.output_dir, "sensor_ppg.csv"),
            SENSOR_COLUMNS,
            self._format_sensor_row,
        )
        self.eye_writer = CsvWriter(
            os.path.join(self.output_dir, "right_eye_brightness.csv"),
            EYE_COLUMNS,
            self._format_eye_row,
        )
        self.sensor_writer.start()
        self.eye_writer.start()
        self.eye_sink = RightEyeBrightnessSink(self.eye_writer)

    def _start_sensor_stream(self):
        port = self.args.port or autodetect_sensor_port()
        if not port:
            raise RuntimeError(f"No '{TARGET_SERIAL_DESCRIPTION}' serial port found.")

        self.sensor = SensorReader(port, self.args.baud)
        self.sensor.start()
        print(f"Sensor port: {port}")
        time.sleep(SERIAL_STARTUP_SECONDS)
        self.sensor.reset_buffers()
        self.sensor.send("s\n")
        self.sensor_streaming = True
        print("Sensor stream started.")

    def _start_eye_tracker(self):
        sdk_config_path = os.fspath(sdk_config_dir(self.args.sdk_root))
        pwd = read_pwd(sdk_config_path)
        if not pwd:
            raise RuntimeError("Missing softdog password in SDK config.ini.")

        self.sdk.load_library(sdk_config_path)
        self.sdk.set_ui_handle(self.eye_sink)
        if self.sdk.connect_softdog(pwd) != 0:
            raise RuntimeError("Eye tracker softdog connect failed.")

        scene_width, scene_height = resolve_scene_dimensions(self.args.resolution)
        if self.sdk.start(self.args.environment, self.args.resolution, scene_width, scene_height) != 0:
            raise RuntimeError("Eye tracker start failed.")
        self.eye_started = True
        print("Eye tracker started.")

    def _start_sensor_sync(self):
        self.sensor.send("SYNC_START\n")
        self.sensor_syncing = True
        print("Sensor sync mode started.")

    def _capture_loop(self):
        start_time = time.perf_counter()
        next_print = start_time
        print(f"Capturing for {self.args.duration:.1f} seconds. Press Ctrl+C to stop early.")
        while not self.stop_requested:
            now = time.perf_counter()
            if now - start_time >= self.args.duration:
                break
            self._drain_sensor_samples()
            if now >= next_print:
                next_print = now + 1.0
                elapsed = now - start_time
                print(
                    f"\rElapsed {elapsed:5.1f}s | "
                    f"PPG rows {self.sensor_writer.rows_written} | "
                    f"Eye rows {self.eye_writer.rows_written}",
                    end="",
                    flush=True,
                )
            time.sleep(0.005)
        self._drain_sensor_samples()
        print()

    def _drain_sensor_samples(self):
        while True:
            try:
                sample = self.sensor.samples.get_nowait()
            except queue.Empty:
                break
            self.sensor_writer.write(sample)

    def _cleanup(self):
        if self.sensor:
            try:
                if self.sensor_syncing:
                    self.sensor.send("SYNC_STOP\n")
                    time.sleep(0.1)
                if self.sensor_streaming:
                    self.sensor.send("e\n")
                    time.sleep(0.1)
            except Exception:
                pass
        if self.eye_started:
            try:
                self.sdk.stop()
            except Exception:
                pass
            self.eye_started = False
        if self.eye_sink:
            self.eye_sink.stop()
            self.eye_sink = None
        if self.sensor:
            self.sensor.stop()
            self.sensor = None
        if self.sensor_writer:
            self.sensor_writer.stop()
        if self.eye_writer:
            self.eye_writer.stop()

    @staticmethod
    def _format_sensor_row(sample):
        return [
            f"{sample.device_timestamp_ms:.3f}",
            f"{sample.pc_timestamp:.9f}",
            str(int(sample.red)),
            str(int(sample.ir)),
            str(int(sample.green)),
        ]

    @staticmethod
    def _format_eye_row(row):
        return [
            str(int(row["device_timestamp"])),
            f"{row['pc_timestamp']:.9f}",
            f"{row['mean_brightness']:.6f}",
        ]

    def _print_summary(self):
        print(f"Capture saved to: {self.output_dir}")
        print(f"Sensor rows: {self.sensor_writer.rows_written}")
        print(f"Right-eye rows: {self.eye_writer.rows_written}")
        print("Review with: python sync_capture_review.py")


def parse_args():
    parser = argparse.ArgumentParser(description="Capture sync PPG and right-eye brightness traces.")
    add_sdk_root_argument(parser)
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--duration", type=float, default=DEFAULT_CAPTURE_SECONDS)
    parser.add_argument("--environment", type=int, default=DEFAULT_ENVIRONMENT)
    parser.add_argument("--resolution", type=int, default=DEFAULT_RESOLUTION)
    return parser.parse_args()


def main():
    args = parse_args()
    session = SyncCaptureSession(args)

    def handle_sigint(_signum, _frame):
        session.request_stop()

    signal.signal(signal.SIGINT, handle_sigint)
    try:
        session.run()
    except KeyboardInterrupt:
        session.request_stop()
        session._cleanup()
        session._print_summary()
    except Exception as exc:
        session._cleanup()
        print(f"Capture failed: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
