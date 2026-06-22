#!/usr/bin/env python
"""
VR IMU / eye-flash synchronization test.

This terminal script reuses the VR visual logger data path:
  - VrUserSdkWrapper.data_queue provides gaze/pupil/head-IMU samples.
  - VrUserSdkWrapper.image_queue provides eye images used for brightness traces.
  - Optional CP210x ring/watch serial sensors are logged with the visual logger
    packet format when present.

The capture phase logs raw streams for a fixed duration. The analysis phase
opens a matplotlib window, saves the plotted head-IMU / eye-brightness traces,
detects one peak in each, and reports eye-minus-IMU timing on the shared PC
perf-counter timeline.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import queue
import signal
import struct
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
from serial.tools import list_ports

from example_paths import LOG_DIR, VR_SDK_ROOT_ENV_VAR, resolve_vr_sdk_root, vr_sdk_bin_dir
from sdk_vr_usersdk_wrapper import EyeImageFrame, VrUserSdkWrapper


PACKET_SIZE = 64
NUM_FLOATS = 14
SYNC_MARKER = bytes([0x55, 0xAA, 0x55, 0xAA])
SYNC_LEN = len(SYNC_MARKER)
TARGET_SERIAL_DESCRIPTION = "Silicon Labs CP210x USB to UART Bridge"
SENSOR_IDENTITY_TIMEOUT_SECONDS = 2.0
SENSOR_ROLES = ("ring", "watch")

DEFAULT_DURATION_SECONDS = 10.0
DEFAULT_BAUD = 1_000_000
DEFAULT_THRESHOLD_MULTIPLIER = 4.0
DEFAULT_OUTPUT_ROOT = LOG_DIR / "sync_vr_imu_flash"

EYE_COLUMNS = [
    "device_timestamp",
    "pc_arrival_timestamp",
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
    "gyro_timestamp",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "mag_x",
    "mag_y",
    "mag_z",
]

BRIGHTNESS_COLUMNS = [
    "eye",
    "eye_name",
    "eye_device_timestamp",
    "eye_pc_arrival_timestamp",
    "width",
    "height",
    "mean_brightness",
    "max_brightness",
    "p99_brightness",
]

SENSOR_COLUMNS = [
    "Red",
    "IR",
    "Green",
    "accX",
    "accY",
    "accZ",
    "gyrX",
    "gyrY",
    "gyrZ",
    "magX",
    "magY",
    "magZ",
    "temp",
    "device_timestamp_ms",
    "pc_arrival_timestamp",
]


@dataclass
class SensorReader:
    role: str
    port: str
    baudrate: int
    serial_port: serial.Serial | None = None

    def __post_init__(self):
        self.packet_queue: queue.SimpleQueue[dict] = queue.SimpleQueue()
        self.identity: str | None = None
        self._buffer = bytearray()
        self._text_buffer = bytearray()
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def open(self) -> bool:
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.01)
            self._stop_event.clear()
            return True
        except Exception as exc:
            print(f"[sensor] {self.port} open failed: {exc}")
            self.serial_port = None
            return False

    def start(self) -> bool:
        if not self.serial_port and not self.open():
            return False
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self._thread = None

    def send_command(self, command_text: str):
        if not self.serial_port or not self.serial_port.is_open:
            return
        with self._write_lock:
            self.serial_port.write(command_text.encode("ascii"))
            self.serial_port.flush()

    def detect_identity(self, timeout_seconds: float = SENSOR_IDENTITY_TIMEOUT_SECONDS) -> str | None:
        if not self.serial_port and not self.open():
            return None
        deadline = time.perf_counter() + timeout_seconds
        while time.perf_counter() < deadline:
            if self.identity:
                return self.identity
            try:
                incoming = self.serial_port.read(self.serial_port.in_waiting or 1)
            except Exception:
                break
            if incoming:
                self._buffer.extend(incoming)
                self._consume_packets()
        return self.identity

    def _read_loop(self):
        while not self._stop_event.is_set():
            if not self.serial_port or not self.serial_port.is_open:
                break
            try:
                incoming = self.serial_port.read(self.serial_port.in_waiting or 1)
            except Exception:
                break
            if incoming:
                self._buffer.extend(incoming)
                self._consume_packets()

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
            if len(self._buffer) < PACKET_SIZE:
                return

            payload = bytes(self._buffer[SYNC_LEN:PACKET_SIZE])
            del self._buffer[:PACKET_SIZE]
            pc_arrival_timestamp = time.perf_counter()
            try:
                values = struct.unpack("<14f", payload[: NUM_FLOATS * 4])
            except struct.error:
                continue
            if sum(1 for value in values if value != value) > 3:
                continue
            device_timestamp_ms = values[13]
            if device_timestamp_ms == device_timestamp_ms and (
                device_timestamp_ms < 0 or device_timestamp_ms > 4.3e9
            ):
                continue

            self.packet_queue.put(
                {
                    "pc_arrival_timestamp": pc_arrival_timestamp,
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
                    "device_timestamp_ms": device_timestamp_ms,
                }
            )

    def _marker_prefix_len(self) -> int:
        max_len = min(len(self._buffer), SYNC_LEN - 1)
        for size in range(max_len, 0, -1):
            if self._buffer[-size:] == SYNC_MARKER[:size]:
                return size
        return 0

    def _consume_text_preamble(self, data: bytes):
        if not data:
            return
        self._text_buffer.extend(data)
        while b"\n" in self._text_buffer:
            line, _, rest = self._text_buffer.partition(b"\n")
            self._text_buffer = bytearray(rest)
            marker = line.decode("ascii", errors="ignore").strip().casefold()
            if marker in SENSOR_ROLES:
                self.identity = marker


class CsvLog:
    def __init__(self, path: Path, columns: list[str]):
        self.path = path
        self.fp = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.fp)
        self.writer.writerow(columns)
        self.rows = 0

    def write(self, row: list):
        self.writer.writerow(row)
        self.rows += 1

    def close(self):
        self.fp.close()


def candidate_sensor_ports():
    target = TARGET_SERIAL_DESCRIPTION.casefold()
    ports = []
    for port in list_ports.comports():
        fields = (port.description, port.manufacturer, port.product, port.interface)
        if any(target in (field or "").casefold() for field in fields):
            ports.append(port)
    ports.sort(key=lambda p: p.device)
    return ports


def detect_sensor_readers(baudrate: int) -> dict[str, SensorReader]:
    detected: dict[str, SensorReader] = {}
    for port in candidate_sensor_ports():
        reader = SensorReader(role="", port=port.device, baudrate=baudrate)
        if not reader.open():
            continue
        identity = reader.detect_identity()
        if identity not in SENSOR_ROLES or identity in detected:
            reader.stop()
            continue
        reader.role = identity
        detected[identity] = reader
    return detected


def eye_csv_row(sample: dict) -> list[str]:
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
        f"{sample.get('accel_x', 0.0):.6f}",
        f"{sample.get('accel_y', 0.0):.6f}",
        f"{sample.get('accel_z', 0.0):.6f}",
        f"{sample.get('gyro_x', 0.0):.6f}",
        f"{sample.get('gyro_y', 0.0):.6f}",
        f"{sample.get('gyro_z', 0.0):.6f}",
        f"{sample.get('mag_x', 0.0):.6f}",
        f"{sample.get('mag_y', 0.0):.6f}",
        f"{sample.get('mag_z', 0.0):.6f}",
    ]


def sensor_csv_row(sample: dict) -> list[str]:
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


def brightness_row(frame: EyeImageFrame) -> tuple[list[str], dict] | None:
    pixels = np.frombuffer(frame.data, dtype=np.uint8)
    expected = int(frame.width) * int(frame.height)
    if pixels.size < expected or expected <= 0:
        return None
    pixels = pixels[:expected]
    mean = float(pixels.mean())
    row_dict = {
        "eye": int(frame.eye),
        "eye_name": "left" if int(frame.eye) == 1 else "right",
        "eye_device_timestamp": int(frame.device_timestamp),
        "eye_pc_arrival_timestamp": float(frame.pc_arrival_timestamp),
        "width": int(frame.width),
        "height": int(frame.height),
        "mean_brightness": mean,
        "max_brightness": int(pixels.max()),
        "p99_brightness": float(np.percentile(pixels, 99)),
    }
    return (
        [
            str(row_dict["eye"]),
            row_dict["eye_name"],
            str(row_dict["eye_device_timestamp"]),
            f"{row_dict['eye_pc_arrival_timestamp']:.9f}",
            str(row_dict["width"]),
            str(row_dict["height"]),
            f"{row_dict['mean_brightness']:.6f}",
            str(row_dict["max_brightness"]),
            f"{row_dict['p99_brightness']:.6f}",
        ],
        row_dict,
    )


def moving_average(values: np.ndarray, window_samples: int) -> np.ndarray:
    if values.size == 0:
        return values
    window_samples = max(3, int(window_samples))
    if window_samples % 2 == 0:
        window_samples += 1
    if window_samples >= values.size:
        return np.full_like(values, float(np.mean(values)), dtype=np.float64)
    pad = window_samples // 2
    kernel = np.ones(window_samples, dtype=np.float64) / float(window_samples)
    padded = np.pad(values, (pad, pad), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def robust_strength(times: np.ndarray, values: np.ndarray, baseline_seconds: float) -> dict:
    if times.size == 0 or values.size == 0:
        empty = np.array([], dtype=np.float64)
        return {"baseline": empty, "centered": empty, "strength": empty, "sigma": 1e-6}
    sample_dt = float(np.median(np.diff(times))) if times.size >= 2 else 0.01
    baseline_samples = max(5, int(round(baseline_seconds / max(sample_dt, 1e-6))))
    baseline = moving_average(values, baseline_samples)
    centered = values - baseline
    median = float(np.median(centered))
    mad = float(np.median(np.abs(centered - median)))
    sigma = max(1.4826 * mad, 1e-6)
    strength = np.maximum(0.0, centered / sigma)
    return {"baseline": baseline, "centered": centered, "strength": strength, "sigma": sigma}


def strongest_peak(times: np.ndarray, values: np.ndarray, strength: np.ndarray, threshold: float, ignore_initial: float):
    if times.size == 0:
        return None
    keep = np.ones(times.size, dtype=bool)
    if ignore_initial > 0:
        keep &= times >= (float(times[0]) + ignore_initial)
    if not np.any(keep):
        return None
    candidate = np.where(keep)[0]
    strong = candidate[strength[candidate] >= threshold]
    search = strong if strong.size else candidate
    idx = int(search[np.argmax(strength[search])])
    return {
        "index": idx,
        "time": float(times[idx]),
        "value": float(values[idx]),
        "strength": float(strength[idx]),
        "above_threshold": bool(strength[idx] >= threshold),
    }


def load_csv_dicts(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open("r", newline="", encoding="utf-8") as fp:
        return list(csv.DictReader(fp))


def analyze_capture(output_dir: Path, threshold_multiplier: float, ignore_initial: float, imu_signal: str) -> dict:
    eye_rows = load_csv_dicts(output_dir / "vr_eye_samples.csv")
    brightness_rows = load_csv_dicts(output_dir / "eye_brightness.csv")
    if not eye_rows:
        raise RuntimeError("No VR eye/IMU rows captured.")
    if not brightness_rows:
        raise RuntimeError("No eye brightness rows captured.")

    imu_times = np.array([float(row["pc_arrival_timestamp"]) for row in eye_rows], dtype=np.float64)
    accel = np.array(
        [
            [float(row["accel_x"]), float(row["accel_y"]), float(row["accel_z"])]
            for row in eye_rows
        ],
        dtype=np.float64,
    )
    gyro = np.array(
        [
            [float(row["gyro_x"]), float(row["gyro_y"]), float(row["gyro_z"])]
            for row in eye_rows
        ],
        dtype=np.float64,
    )
    accel_norm = np.linalg.norm(accel, axis=1)
    gyro_norm = np.linalg.norm(gyro, axis=1)

    accel_analysis = robust_strength(imu_times, accel_norm, baseline_seconds=0.35)
    gyro_analysis = robust_strength(imu_times, gyro_norm, baseline_seconds=0.35)
    if imu_signal == "accel":
        imu_values = accel_norm
        imu_strength = accel_analysis["strength"]
        imu_label = "Accel norm"
    elif imu_signal == "gyro":
        imu_values = gyro_norm
        imu_strength = gyro_analysis["strength"]
        imu_label = "Gyro norm"
    else:
        imu_values = np.maximum(accel_analysis["centered"], gyro_analysis["centered"])
        imu_strength = np.maximum(accel_analysis["strength"], gyro_analysis["strength"])
        imu_label = "Combined IMU strength"

    brightness_by_eye: dict[int, list[dict]] = {1: [], 2: []}
    for row in brightness_rows:
        try:
            brightness_by_eye[int(row["eye"])].append(row)
        except Exception:
            continue

    eye_candidates = []
    eye_traces = {}
    for eye, rows in brightness_by_eye.items():
        if not rows:
            continue
        times = np.array([float(row["eye_pc_arrival_timestamp"]) for row in rows], dtype=np.float64)
        values = np.array([float(row["mean_brightness"]) for row in rows], dtype=np.float64)
        analysis = robust_strength(times, values, baseline_seconds=0.25)
        peak = strongest_peak(times, values, analysis["strength"], threshold_multiplier, ignore_initial)
        eye_traces[eye] = {"times": times, "values": values, "analysis": analysis, "peak": peak}
        if peak:
            peak_with_eye = dict(peak)
            peak_with_eye["eye"] = eye
            eye_candidates.append(peak_with_eye)

    imu_peak = strongest_peak(imu_times, imu_values, imu_strength, threshold_multiplier, ignore_initial)
    if not imu_peak:
        raise RuntimeError("Could not detect an IMU impact peak.")
    if not eye_candidates:
        raise RuntimeError("Could not detect an eye brightness peak.")

    eye_peak = max(eye_candidates, key=lambda peak: peak["strength"])
    delta_ms = (eye_peak["time"] - imu_peak["time"]) * 1000.0
    base_time = min(float(imu_times[0]), min(float(trace["times"][0]) for trace in eye_traces.values()))

    plot_path = output_dir / "sync_imu_eye_flash.png"
    draw_analysis_plot(
        plot_path,
        base_time,
        imu_times,
        accel_norm,
        gyro_norm,
        imu_strength,
        imu_peak,
        eye_traces,
        eye_peak,
        delta_ms,
        threshold_multiplier,
        imu_label,
    )

    result = {
        "output_dir": str(output_dir),
        "imu_signal": imu_signal,
        "threshold_multiplier": threshold_multiplier,
        "ignore_initial_seconds": ignore_initial,
        "imu_peak_pc_time": imu_peak["time"],
        "imu_peak_time_since_start_seconds": imu_peak["time"] - base_time,
        "imu_peak_value": imu_peak["value"],
        "imu_peak_strength": imu_peak["strength"],
        "imu_peak_above_threshold": imu_peak["above_threshold"],
        "eye_peak_eye": eye_peak["eye"],
        "eye_peak_pc_time": eye_peak["time"],
        "eye_peak_time_since_start_seconds": eye_peak["time"] - base_time,
        "eye_peak_value": eye_peak["value"],
        "eye_peak_strength": eye_peak["strength"],
        "eye_peak_above_threshold": eye_peak["above_threshold"],
        "eye_minus_imu_ms": delta_ms,
        "plot": str(plot_path),
    }

    with (output_dir / "sync_summary.json").open("w", encoding="utf-8") as fp:
        json.dump(result, fp, indent=2)
    with (output_dir / "sync_report.txt").open("w", encoding="utf-8") as fp:
        fp.write(format_report(result))
    return result


def draw_analysis_plot(
    path: Path,
    base_time: float,
    imu_times: np.ndarray,
    accel_norm: np.ndarray,
    gyro_norm: np.ndarray,
    imu_strength: np.ndarray,
    imu_peak: dict,
    eye_traces: dict[int, dict],
    eye_peak: dict,
    delta_ms: float,
    threshold_multiplier: float,
    imu_label: str,
):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    fig.suptitle(f"VR IMU / eye flash sync: eye - IMU = {delta_ms:.3f} ms", fontsize=13)

    x_imu = imu_times - base_time
    axes[0].plot(x_imu, accel_norm, label="Accel norm", color="tab:blue", lw=1.0)
    axes[0].plot(x_imu, gyro_norm, label="Gyro norm", color="tab:orange", lw=1.0)
    axes[0].axvline(imu_peak["time"] - base_time, color="tab:red", lw=1.4, label="IMU peak")
    axes[0].set_ylabel("IMU norm")
    axes[0].set_title(f"Head IMU ({imu_label} used for detection)")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.25)

    colors = {1: "tab:green", 2: "black"}
    for eye, trace in sorted(eye_traces.items()):
        label = "Left brightness" if eye == 1 else "Right brightness"
        axes[1].plot(trace["times"] - base_time, trace["values"], color=colors.get(eye, "gray"), lw=1.0, label=label)
    axes[1].axvline(eye_peak["time"] - base_time, color="tab:purple", lw=1.4, label="Eye brightness peak")
    axes[1].set_ylabel("Brightness")
    axes[1].set_title("Eye image mean brightness")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(x_imu, imu_strength, color="tab:red", lw=1.0, label="IMU strength")
    for eye, trace in sorted(eye_traces.items()):
        label = "Left brightness strength" if eye == 1 else "Right brightness strength"
        axes[2].plot(
            trace["times"] - base_time,
            trace["analysis"]["strength"],
            color=colors.get(eye, "gray"),
            lw=1.0,
            alpha=0.8,
            label=label,
        )
    axes[2].axhline(threshold_multiplier, color="gray", ls="--", lw=1.0, label="threshold")
    axes[2].axvspan(
        min(imu_peak["time"], eye_peak["time"]) - base_time,
        max(imu_peak["time"], eye_peak["time"]) - base_time,
        color="tab:purple",
        alpha=0.12,
    )
    axes[2].set_xlabel("Time since first sample (s)")
    axes[2].set_ylabel("Noise x")
    axes[2].set_title("Robust peak strength")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.25)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(path, dpi=160)
    plt.show(block=True)
    plt.close(fig)


def format_report(result: dict) -> str:
    eye_name = "left" if result["eye_peak_eye"] == 1 else "right"
    return "\n".join(
        [
            "VR IMU / eye-flash synchronization result",
            f"Output: {result['output_dir']}",
            f"IMU signal: {result['imu_signal']}",
            f"IMU peak: {result['imu_peak_time_since_start_seconds']:.6f} s "
            f"(strength {result['imu_peak_strength']:.3f}x)",
            f"Eye peak: {result['eye_peak_time_since_start_seconds']:.6f} s "
            f"({eye_name}, strength {result['eye_peak_strength']:.3f}x)",
            f"Eye minus IMU: {result['eye_minus_imu_ms']:.3f} ms",
            f"Plot: {result['plot']}",
            "",
        ]
    )


def ensure_matplotlib_available():
    try:
        import matplotlib.pyplot  # noqa: F401
    except Exception as exc:
        raise RuntimeError(
            "matplotlib with an interactive window backend is required for this sync test. "
            "Install matplotlib in the Python environment used to run the script."
        ) from exc


class CaptureSession:
    def __init__(self, args):
        self.args = args
        self.output_dir = self._make_output_dir()
        self.sdk = VrUserSdkWrapper()
        self.sensor_readers: dict[str, SensorReader] = {}
        self.stop_requested = False
        self.logs: dict[str, CsvLog] = {}

    def request_stop(self):
        self.stop_requested = True

    def run(self) -> dict:
        try:
            self._start_logs()
            self._start_vr()
            self._start_sensors()
            self._capture_loop()
        finally:
            self._cleanup()
        return self._write_metadata()

    def _make_output_dir(self) -> Path:
        base = Path(self.args.output_root)
        base.mkdir(parents=True, exist_ok=True)
        output_dir = base / datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir.mkdir(parents=True, exist_ok=False)
        return output_dir

    def _start_logs(self):
        self.logs["eye"] = CsvLog(self.output_dir / "vr_eye_samples.csv", EYE_COLUMNS)
        self.logs["brightness"] = CsvLog(self.output_dir / "eye_brightness.csv", BRIGHTNESS_COLUMNS)

    def _start_vr(self):
        vr_bin = os.fspath(vr_sdk_bin_dir(self.args.vr_sdk_root))
        self.sdk.load_library(vr_bin)
        self.sdk.connect()
        self.sdk.start()
        print("VR eye tracker started.")

    def _start_sensors(self):
        if self.args.no_serial_sensors:
            return
        self.sensor_readers = detect_sensor_readers(self.args.baud)
        for role, reader in self.sensor_readers.items():
            self.logs[f"sensor_{role}"] = CsvLog(self.output_dir / f"sensor_{role}.csv", SENSOR_COLUMNS)
            if reader.serial_port:
                reader.serial_port.reset_input_buffer()
                reader.serial_port.reset_output_buffer()
            reader.start()
            reader.send_command("s\n")
            print(f"{role} sensor started on {reader.port}.")

    def _capture_loop(self):
        start = time.perf_counter()
        next_print = start
        print(f"Capturing for {self.args.duration:.1f} seconds. Trigger impact + flash during this window.")
        while not self.stop_requested:
            now = time.perf_counter()
            if now - start >= self.args.duration:
                break
            self._drain_all()
            if now >= next_print:
                next_print = now + 1.0
                print(
                    f"\rElapsed {now - start:5.1f}s | "
                    f"IMU rows {self.logs['eye'].rows} | "
                    f"brightness rows {self.logs['brightness'].rows}",
                    end="",
                    flush=True,
                )
            time.sleep(0.003)
        self._drain_all()
        print()

    def _drain_all(self):
        while True:
            try:
                sample = self.sdk.data_queue.get_nowait()
            except queue.Empty:
                break
            self.logs["eye"].write(eye_csv_row(sample))

        while True:
            try:
                frame = self.sdk.image_queue.get_nowait()
            except queue.Empty:
                break
            result = brightness_row(frame)
            if result:
                row, _row_dict = result
                self.logs["brightness"].write(row)

        for role, reader in self.sensor_readers.items():
            log = self.logs.get(f"sensor_{role}")
            while True:
                try:
                    sample = reader.packet_queue.get_nowait()
                except queue.Empty:
                    break
                if log:
                    log.write(sensor_csv_row(sample))

    def _cleanup(self):
        for reader in self.sensor_readers.values():
            try:
                reader.send_command("e\n")
            except Exception:
                pass
        for reader in self.sensor_readers.values():
            try:
                reader.stop()
            except Exception as exc:
                print(f"[sensor] stop failed: {exc}")
        try:
            self.sdk.disconnect()
        except Exception as exc:
            print(f"[vr] disconnect failed: {exc}")
        for log in self.logs.values():
            try:
                log.close()
            except Exception:
                pass

    def _write_metadata(self) -> dict:
        metadata = {
            "output_dir": str(self.output_dir),
            "output_root": str(Path(self.args.output_root)),
            "duration_seconds": self.args.duration,
            "vr_sdk_root": self.args.vr_sdk_root,
            "serial_sensors_enabled": not self.args.no_serial_sensors,
            "rows": {name: log.rows for name, log in self.logs.items()},
            "sensor_ports": {role: reader.port for role, reader in self.sensor_readers.items()},
        }
        with (self.output_dir / "capture_metadata.json").open("w", encoding="utf-8") as fp:
            json.dump(metadata, fp, indent=2)
        return metadata


def parse_args():
    parser = argparse.ArgumentParser(description="Capture 10s VR head-IMU and eye-flash data, then compute sync delta.")
    parser.add_argument(
        "--vr-sdk-root",
        default=str(resolve_vr_sdk_root()),
        help=(
            "VR vendor SDK root directory. Defaults to the value of "
            f"{VR_SDK_ROOT_ENV_VAR} or {resolve_vr_sdk_root()}."
        ),
    )
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_SECONDS)
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="Dedicated root for this sync-test data. Each run creates a timestamped child directory.",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--no-serial-sensors", action="store_true", help="Do not auto-log CP210x ring/watch sensors.")
    parser.add_argument(
        "--threshold",
        type=float,
        default=DEFAULT_THRESHOLD_MULTIPLIER,
        help="Robust noise multiplier used for peak confidence reporting.",
    )
    parser.add_argument(
        "--ignore-initial",
        type=float,
        default=0.25,
        help="Seconds from first sample ignored during peak search to avoid startup transients.",
    )
    parser.add_argument(
        "--imu-signal",
        choices=["combined", "accel", "gyro"],
        default="combined",
        help="IMU trace used to pick the impact peak.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        ensure_matplotlib_available()
    except Exception as exc:
        print(f"Sync test failed: {exc}", file=sys.stderr)
        sys.exit(1)

    session = CaptureSession(args)

    def handle_sigint(_signum, _frame):
        session.request_stop()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        metadata = session.run()
        print(f"Capture saved to: {metadata['output_dir']}")
        result = analyze_capture(
            Path(metadata["output_dir"]),
            threshold_multiplier=args.threshold,
            ignore_initial=args.ignore_initial,
            imu_signal=args.imu_signal,
        )
    except KeyboardInterrupt:
        session.request_stop()
        print("\nInterrupted.", file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print(f"Sync test failed: {exc}", file=sys.stderr)
        sys.exit(1)

    print(format_report(result))


if __name__ == "__main__":
    main()
