#!/usr/bin/env python3
"""
Terminal-based sync capture tool.

This script only captures and logs raw data. It does not open any review UI.
Use `sync_capture_review.py` afterwards to visualize the recorded traces and
estimate the offset between the camera brightness edge and the PPG edge.
"""

import argparse
import configparser
import csv
import ctypes
import json
import os
import queue
import signal
import struct
import sys
import threading
import time
from datetime import datetime

import numpy as np
import serial
from serial.tools import list_ports

from sdk_types import PY_7I_RESOLUTION
from sdk_wrapper import wrapper

try:
    import cv2
except ImportError:
    cv2 = None


PACKET_SIZE = 64
NUM_FLOATS = 14
SYNC_MARKER = bytes([0x55, 0xAA, 0x55, 0xAA])
SYNC_LEN = 4
SERIAL_STARTUP_SETTLE_SECONDS = 2.0
TARGET_SERIAL_DESCRIPTION = "Silicon Labs CP210x USB to UART Bridge"
DEFAULT_DURATION_SECONDS = 30.0
DEFAULT_SENSOR_SAMPLE_RATE = 250.0
DEFAULT_RESOLUTION = 202
DEFAULT_ENVIRONMENT = 301

SENSOR_COLUMNS = [
    "pc_perf_timestamp",
    "sensor_timestamp_ms",
    "sensor_pc_timestamp",
    "red",
    "ir",
    "green",
    "acc_x",
    "acc_y",
    "acc_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "mag_x",
    "mag_y",
    "mag_z",
    "temp",
]

RIGHT_EYE_COLUMNS = [
    "frame_index",
    "pc_perf_timestamp",
    "eye_device_timestamp",
    "eye_device_time_seconds",
    "eye_device_pc_timestamp",
    "eye_clock_offset_seconds",
    "eye_clock_calibrated",
    "mean_brightness",
    "width",
    "height",
    "queue_latency_ms",
]


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
    matches = []
    for port in list_ports.comports():
        description = port.description or ""
        if TARGET_SERIAL_DESCRIPTION in description:
            matches.append(port.device)
    return matches[0] if matches else None


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
        if not response or not response.startswith(b"T"):
            continue
        try:
            arduino_ms = int(response[1:].strip())
        except ValueError:
            continue
        rtt = t2 - t1
        offset = (t1 + t2) * 0.5 - (arduino_ms / 1000.0)
        if rtt < best_rtt:
            best_rtt = rtt
            best_offset = offset
        success_count += 1
    if success_count == 0:
        raise RuntimeError("Failed to synchronize timestamps with the sensor.")
    return best_offset


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
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.05,
            )
            self._stop_event.clear()
            return True
        except Exception as exc:
            print(f"Sensor serial open failed: {exc}")
            self.serial_port = None
            return False

    def start(self):
        if not self.serial_port and not self.open():
            return False
        if self._thread_started:
            return True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self._thread_started = True
        return True

    def stop(self):
        self._stop_event.set()
        if self._thread_started and self._thread:
            self._thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self._thread = None
        self._thread_started = False

    def send_command(self, command_text):
        if not self.serial_port or not self.serial_port.is_open:
            return
        with self._write_lock:
            self.serial_port.write(command_text.encode("ascii"))
            self.serial_port.flush()

    def _read_loop(self):
        while not self._stop_event.is_set():
            if not self.serial_port or not self.serial_port.is_open:
                break
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
            if len(self._buffer) < self.packet_size:
                return
            payload = bytes(self._buffer[SYNC_LEN:self.packet_size])
            del self._buffer[:self.packet_size]
            try:
                values = struct.unpack("<14f", payload[:NUM_FLOATS * 4])
            except struct.error:
                continue

            timestamp_ms = values[13]
            if timestamp_ms == timestamp_ms and (timestamp_ms < 0 or timestamp_ms > 4.3e9):
                continue

            packet = {
                "pc_perf_timestamp": time.perf_counter(),
                "red": values[0],
                "ir": values[1],
                "green": values[2],
                "acc_x": values[3],
                "acc_y": values[4],
                "acc_z": values[5],
                "gyro_x": values[6],
                "gyro_y": values[7],
                "gyro_z": values[8],
                "mag_x": values[9],
                "mag_y": values[10],
                "mag_z": values[11],
                "temp": values[12],
                "sensor_timestamp_ms": values[13],
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
        while b"\n" in self._text_buffer:
            line, _, rest = self._text_buffer.partition(b"\n")
            self._text_buffer = bytearray(rest)
            self._handle_text_line(line.decode("ascii", errors="ignore").strip())

    def _handle_text_line(self, line):
        if line:
            print(f"[sensor] {line}")


class CsvWriterThread:
    def __init__(self, output_file, columns, format_row):
        self.output_file = output_file
        self.columns = columns
        self.format_row = format_row
        self.queue = queue.SimpleQueue()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self.rows_written = 0

    def start(self):
        self._thread.start()

    def push(self, row):
        self.queue.put(row)

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=5.0)

    def _write_loop(self):
        with open(self.output_file, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.columns)
            while not self._stop_event.is_set() or not self.queue.empty():
                try:
                    row = self.queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                writer.writerow(self.format_row(row))
                self.rows_written += 1


class EyeDeviceClockMapper:
    def __init__(self, calibration_frames=32):
        self.calibration_frames = calibration_frames
        self.samples = []
        self.scale_seconds_per_tick = None
        self.offset_seconds = 0.0
        self.calibrated = False
        self.lock = threading.Lock()

    def update_and_map(self, device_timestamp, callback_perf_timestamp):
        self.update(device_timestamp, callback_perf_timestamp)
        return self.map_device_timestamp(device_timestamp)

    def update(self, device_timestamp, callback_perf_timestamp):
        device_tick = float(device_timestamp)
        callback_perf_timestamp = float(callback_perf_timestamp)
        with self.lock:
            if self.calibrated:
                return
            self.samples.append((device_tick, callback_perf_timestamp))
            self._fit_locked()
            if len(self.samples) >= self.calibration_frames and not self.calibrated:
                self._fallback_fit_locked()

    def map_device_timestamp(self, device_timestamp):
        device_tick = float(device_timestamp)
        with self.lock:
            scale = self.scale_seconds_per_tick
            if scale is None:
                scale = 0.001
                if self.samples:
                    offsets = [pc - dev * scale for dev, pc in self.samples]
                    offset = float(np.median(offsets))
                else:
                    offset = 0.0
            else:
                offset = self.offset_seconds

            device_time_seconds = device_tick * scale
            mapped_perf_timestamp = device_time_seconds + offset
            calibrated = self.calibrated

        return {
            "device_time_seconds": device_time_seconds,
            "mapped_perf_timestamp": mapped_perf_timestamp,
            "offset_seconds": offset,
            "calibrated": calibrated,
        }

    def is_calibrated(self):
        with self.lock:
            return self.calibrated

    def snapshot(self):
        with self.lock:
            return {
                "scale_seconds_per_tick": self.scale_seconds_per_tick,
                "offset_seconds": self.offset_seconds,
                "calibrated": self.calibrated,
                "calibration_samples": len(self.samples),
                "calibration_target_samples": self.calibration_frames,
            }

    def _fit_locked(self):
        if len(self.samples) < 2:
            return

        device = np.array([item[0] for item in self.samples], dtype=np.float64)
        pc = np.array([item[1] for item in self.samples], dtype=np.float64)
        device_rel = device - device[0]
        pc_rel = pc - pc[0]
        valid = (device_rel > 0.0) & (pc_rel > 0.0)
        if np.count_nonzero(valid) < 2:
            return

        device_fit = device_rel[valid]
        pc_fit = pc_rel[valid]
        denom = float(np.dot(device_fit, device_fit))
        if denom <= 0.0:
            return

        scale = float(np.dot(device_fit, pc_fit) / denom)
        if not np.isfinite(scale) or scale <= 0.0:
            return

        offsets = pc - device * scale
        offset = float(np.median(offsets))
        if not np.isfinite(offset):
            return

        self.scale_seconds_per_tick = scale
        self.offset_seconds = offset
        self.calibrated = len(self.samples) >= self.calibration_frames

    def _fallback_fit_locked(self):
        device = np.array([item[0] for item in self.samples], dtype=np.float64)
        pc = np.array([item[1] for item in self.samples], dtype=np.float64)
        if len(device) >= 2:
            device_delta = np.diff(device)
            pc_delta = np.diff(pc)
            valid = (device_delta > 0.0) & (pc_delta > 0.0)
            if np.any(valid):
                scale = float(np.median(pc_delta[valid] / device_delta[valid]))
            else:
                scale = 0.001
        else:
            scale = 0.001
        if not np.isfinite(scale) or scale <= 0.0:
            scale = 0.001

        offsets = pc - device * scale
        offset = float(np.median(offsets)) if len(offsets) else 0.0
        self.scale_seconds_per_tick = scale
        self.offset_seconds = offset
        self.calibrated = True


class RightEyeCaptureSink:
    def __init__(self, brightness_writer, video_writer=None):
        self.brightness_writer = brightness_writer
        self.video_writer = video_writer
        self.frame_count = 0
        self.lock = threading.Lock()
        self.clock_mapper = EyeDeviceClockMapper()
        self.frame_queue = queue.Queue()
        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def handle_right_eye_frame(self, frame_bytes, width, height, eye_device_timestamp, pc_perf_timestamp):
        byte_count = int(width) * int(height)
        if isinstance(frame_bytes, (bytes, bytearray)):
            copied_frame = bytes(frame_bytes[:byte_count])
        else:
            copied_frame = ctypes.string_at(frame_bytes, byte_count)
        if len(copied_frame) != byte_count:
            return

        with self.lock:
            frame_index = self.frame_count
            self.frame_count += 1

        self.frame_queue.put({
            "frame_index": frame_index,
            "frame_bytes": copied_frame,
            "pc_perf_timestamp": pc_perf_timestamp,
            "eye_device_timestamp": int(eye_device_timestamp),
            "width": width,
            "height": height,
            "queued_perf_timestamp": time.perf_counter(),
        })

    def stop(self):
        self._stop_event.set()
        self._worker.join()

    def clock_snapshot(self):
        return self.clock_mapper.snapshot()

    def _worker_loop(self):
        pending_items = []
        while not self._stop_event.is_set() or not self.frame_queue.empty():
            try:
                item = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                if self._stop_event.is_set():
                    self._flush_pending_items(pending_items)
                continue

            self.clock_mapper.update(item["eye_device_timestamp"], item["pc_perf_timestamp"])
            pending_items.append(item)
            if not self.clock_mapper.is_calibrated() and not self._stop_event.is_set():
                continue

            self._flush_pending_items(pending_items)

        self._flush_pending_items(pending_items)

    def _flush_pending_items(self, pending_items):
        while pending_items:
            self._process_frame_item(pending_items.pop(0))

    def _process_frame_item(self, item):
        frame = np.frombuffer(item["frame_bytes"], dtype=np.uint8)
        width = int(item["width"])
        height = int(item["height"])
        if frame.size != width * height:
            return

        mapped_time = self.clock_mapper.map_device_timestamp(item["eye_device_timestamp"])
        mean_brightness = float(frame.mean())
        queue_latency_ms = (time.perf_counter() - item["queued_perf_timestamp"]) * 1000.0
        self.brightness_writer.push({
            "frame_index": item["frame_index"],
            "pc_perf_timestamp": item["pc_perf_timestamp"],
            "eye_device_timestamp": item["eye_device_timestamp"],
            "eye_device_time_seconds": mapped_time["device_time_seconds"],
            "eye_device_pc_timestamp": mapped_time["mapped_perf_timestamp"],
            "eye_clock_offset_seconds": mapped_time["offset_seconds"],
            "eye_clock_calibrated": mapped_time["calibrated"],
            "mean_brightness": mean_brightness,
            "width": width,
            "height": height,
            "queue_latency_ms": queue_latency_ms,
        })

        if self.video_writer is not None:
            frame_2d = frame.reshape((height, width))
            self.video_writer.write(frame_2d)


class VideoWriterWrapper:
    def __init__(self, output_file, fps):
        self.output_file = output_file
        self.fps = fps
        self.writer = None
        self.frame_size = None

    def write(self, gray_frame):
        if cv2 is None:
            return
        height, width = gray_frame.shape[:2]
        if self.writer is None:
            fourcc = cv2.VideoWriter_fourcc(*"XVID")
            self.frame_size = (width, height)
            self.writer = cv2.VideoWriter(self.output_file, fourcc, self.fps, self.frame_size, True)
        if self.writer is None or not self.writer.isOpened():
            return
        bgr_frame = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)
        self.writer.write(bgr_frame)

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None


class SyncCaptureSession:
    def __init__(self, args):
        self.args = args
        self.sdk = wrapper()
        self.sensor_reader = None
        self.sensor_writer = None
        self.right_eye_writer = None
        self.video_writer = None
        self.eye_sink = None
        self.sensor_clock_offset = 0.0
        self.sensor_port = None
        self.output_dir = ""
        self.capture_start_perf = 0.0
        self.capture_end_perf = 0.0
        self.interrupted = False
        self.sensor_stream_started = False
        self.sensor_sync_started = False
        self.eye_started = False
        self.stop_requested = False

    def request_stop(self):
        self.stop_requested = True

    def run(self):
        self._prepare_output_dir()
        self._start_logging()
        try:
            self._start_sensor()
            self._start_eye_tracker()
            self._capture_loop()
        finally:
            self._cleanup()
        self._write_metadata()
        self._print_summary()

    def _prepare_output_dir(self):
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        log_root = os.path.join(project_root, "log")
        os.makedirs(log_root, exist_ok=True)
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = os.path.join(log_root, f"sync_capture_{ts_str}")
        os.makedirs(self.output_dir, exist_ok=True)

    def _start_logging(self):
        sensor_csv = os.path.join(self.output_dir, "sensor_ppg.csv")
        eye_csv = os.path.join(self.output_dir, "right_eye_brightness.csv")
        self.sensor_writer = CsvWriterThread(sensor_csv, SENSOR_COLUMNS, self._format_sensor_row)
        self.right_eye_writer = CsvWriterThread(eye_csv, RIGHT_EYE_COLUMNS, self._format_eye_row)
        self.sensor_writer.start()
        self.right_eye_writer.start()
        if cv2 is not None:
            video_file = os.path.join(self.output_dir, "right_eye_video.avi")
            self.video_writer = VideoWriterWrapper(video_file, fps=max(1.0, self.args.eye_sample_rate))
        self.eye_sink = RightEyeCaptureSink(self.right_eye_writer, self.video_writer)

    def _format_sensor_row(self, row):
        return [
            f"{row['pc_perf_timestamp']:.9f}",
            f"{row['sensor_timestamp_ms']:.3f}",
            f"{row['sensor_pc_timestamp']:.9f}",
            str(int(row["red"])),
            str(int(row["ir"])),
            str(int(row["green"])),
            f"{row['acc_x']:.6f}",
            f"{row['acc_y']:.6f}",
            f"{row['acc_z']:.6f}",
            f"{row['gyro_x']:.6f}",
            f"{row['gyro_y']:.6f}",
            f"{row['gyro_z']:.6f}",
            f"{row['mag_x']:.6f}",
            f"{row['mag_y']:.6f}",
            f"{row['mag_z']:.6f}",
            f"{row['temp']:.6f}",
        ]

    def _format_eye_row(self, row):
        return [
            str(int(row["frame_index"])),
            f"{row['pc_perf_timestamp']:.9f}",
            str(int(row["eye_device_timestamp"])),
            f"{row['eye_device_time_seconds']:.9f}",
            f"{row['eye_device_pc_timestamp']:.9f}",
            f"{row['eye_clock_offset_seconds']:.9f}",
            str(int(bool(row["eye_clock_calibrated"]))),
            f"{row['mean_brightness']:.6f}",
            str(int(row["width"])),
            str(int(row["height"])),
            f"{row['queue_latency_ms']:.3f}",
        ]

    def _start_sensor(self):
        self.sensor_port = self.args.port or autodetect_sensor_port()
        if not self.sensor_port:
            raise RuntimeError(f"No '{TARGET_SERIAL_DESCRIPTION}' serial port found.")

        self.sensor_reader = SerialPacketReader(self.sensor_port, self.args.baud)
        if not self.sensor_reader.open():
            raise RuntimeError(f"Failed to open sensor port {self.sensor_port}.")

        print(f"Sensor port: {self.sensor_port}")
        time.sleep(SERIAL_STARTUP_SETTLE_SECONDS)
        self.sensor_reader.serial_port.reset_input_buffer()
        self.sensor_reader.serial_port.reset_output_buffer()
        self.sensor_clock_offset = sync_timestamps(self.sensor_reader.serial_port, rounds=20)
        print(f"Sensor clock offset: {self.sensor_clock_offset * 1000.0:.3f} ms")

        if not self.sensor_reader.start():
            raise RuntimeError("Failed to start sensor reader thread.")

        time.sleep(0.3)
        self.sensor_reader.send_command("s\n")
        self.sensor_stream_started = True
        time.sleep(0.3)
        self.sensor_reader.send_command("SYNC_START\n")
        self.sensor_sync_started = True
        print("Sensor sync mode started.")

    def _start_eye_tracker(self):
        sdk_config_path = os.path.join(self.args.sdk_root, "bin", "config")
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

    def _capture_loop(self):
        self.capture_start_perf = time.perf_counter()
        last_progress_print = 0.0
        print(f"Capturing for {self.args.duration:.1f} seconds. Press Ctrl+C to stop early.")
        while not self.stop_requested:
            now = time.perf_counter()
            elapsed = now - self.capture_start_perf
            if elapsed >= self.args.duration:
                break
            self._drain_sensor_packets()
            if now - last_progress_print >= 1.0:
                last_progress_print = now
                print(
                    f"\rElapsed {elapsed:5.1f}s | "
                    f"PPG rows {self.sensor_writer.rows_written} | "
                    f"Eye rows {self.right_eye_writer.rows_written}",
                    end="",
                    flush=True,
                )
            time.sleep(0.005)
        self.capture_end_perf = time.perf_counter()
        self._drain_sensor_packets()
        print()

    def _drain_sensor_packets(self):
        while True:
            try:
                packet = self.sensor_reader.packet_queue.get_nowait()
            except queue.Empty:
                break
            packet["sensor_pc_timestamp"] = (packet["sensor_timestamp_ms"] / 1000.0) + self.sensor_clock_offset
            self.sensor_writer.push(packet)

    def _cleanup(self):
        if self.sensor_reader:
            try:
                if self.sensor_sync_started:
                    self.sensor_reader.send_command("SYNC_STOP\n")
                    time.sleep(0.1)
                if self.sensor_stream_started:
                    self.sensor_reader.send_command("e\n")
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
        if self.sensor_reader:
            self.sensor_reader.stop()
            self.sensor_reader = None
        if self.sensor_writer:
            self.sensor_writer.stop()
        if self.right_eye_writer:
            self.right_eye_writer.stop()
        if self.video_writer:
            self.video_writer.close()

    def _write_metadata(self):
        metadata = {
            "capture_started_at": datetime.now().isoformat(),
            "duration_requested_seconds": self.args.duration,
            "capture_start_perf": self.capture_start_perf,
            "capture_end_perf": self.capture_end_perf,
            "sensor_clock_offset_seconds": self.sensor_clock_offset,
            "sensor_sample_rate_hz": self.args.sensor_sample_rate,
            "eye_sample_rate_hz": self.args.eye_sample_rate,
            "sensor_port": self.sensor_port,
            "eye_clock_mapping": self.eye_sink.clock_snapshot() if self.eye_sink else None,
            "baud": self.args.baud,
            "environment": self.args.environment,
            "resolution": self.args.resolution,
            "interrupted": self.interrupted,
            "files": {
                "sensor_ppg_csv": "sensor_ppg.csv",
                "right_eye_brightness_csv": "right_eye_brightness.csv",
                "right_eye_video": "right_eye_video.avi" if cv2 is not None else None,
            },
        }
        with open(os.path.join(self.output_dir, "capture_metadata.json"), "w", encoding="utf-8") as fp:
            json.dump(metadata, fp, indent=2)

    def _print_summary(self):
        print(f"Capture saved to: {self.output_dir}")
        print(f"Sensor rows: {self.sensor_writer.rows_written}")
        print(f"Right-eye rows: {self.right_eye_writer.rows_written}")
        if self.video_writer is not None:
            print(f"Saved right-eye video: {self.video_writer.output_file}")
        print("Use sync_capture_review.py to visualize and calculate the offset.")


def parse_args():
    parser = argparse.ArgumentParser(description="Capture raw sync data for offline review.")
    parser.add_argument("--sdk-root", default="E:/7invensun/aSeeGlassesPlusUserSDK")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_SECONDS)
    parser.add_argument("--sensor-sample-rate", type=float, default=DEFAULT_SENSOR_SAMPLE_RATE)
    parser.add_argument("--eye-sample-rate", type=float, default=120.0)
    parser.add_argument("--environment", type=int, default=DEFAULT_ENVIRONMENT)
    parser.add_argument("--resolution", type=int, default=DEFAULT_RESOLUTION)
    return parser.parse_args()


def main():
    args = parse_args()
    session = SyncCaptureSession(args)

    def _handle_sigint(_signum, _frame):
        session.interrupted = True
        session.request_stop()

    signal.signal(signal.SIGINT, _handle_sigint)
    try:
        session.run()
    except KeyboardInterrupt:
        session.interrupted = True
        session.request_stop()
        session._cleanup()
        session._write_metadata()
        session._print_summary()
    except Exception as exc:
        session._cleanup()
        print(f"Capture failed: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
