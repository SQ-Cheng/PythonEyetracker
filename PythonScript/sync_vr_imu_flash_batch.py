#!/usr/bin/env python
"""
VR IMU / eye-flash batch synchronization script.

Extends sync_vr_imu_flash_test.py with a 120 s long capture followed by
automatic extraction of every potential sync event (±4 s window, ≥5 s apart).
Each window is presented interactively: the user may accept, reject, or click
on a trace to adjust a channel's onset before accepting.

Accepted windows are written under log/sync_vr_imu_flash_batch/<run>/window_<N>/.
The original sync_vr_imu_flash/ tree is never touched.
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
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
from serial.tools import list_ports

try:
    import sounddevice as sd
    SOUNDDEVICE_IMPORT_ERROR = None
except Exception as exc:
    sd = None
    SOUNDDEVICE_IMPORT_ERROR = exc

from example_paths import LOG_DIR, VR_SDK_ROOT_ENV_VAR, resolve_vr_sdk_root, vr_sdk_bin_dir
from sdk_vr_usersdk_wrapper import EyeImageFrame, VrUserSdkWrapper

# ---------------------------------------------------------------------------
# Constants copied / adjusted from sync_vr_imu_flash_test.py
# ---------------------------------------------------------------------------

PACKET_SIZE = 64
NUM_FLOATS = 14
SYNC_MARKER = bytes([0x55, 0xAA, 0x55, 0xAA])
SYNC_LEN = len(SYNC_MARKER)
TARGET_SERIAL_DESCRIPTION = "Silicon Labs CP210x USB to UART Bridge"
SENSOR_IDENTITY_TIMEOUT_SECONDS = 2.0
SENSOR_ROLES = ("ring", "watch")

DEFAULT_DURATION_SECONDS = 120.0          # ← new default
DEFAULT_BAUD = 1_000_000
DEFAULT_AUDIO_SAMPLE_RATE = 48_000
DEFAULT_OUTPUT_ROOT = LOG_DIR / "sync_vr_imu_flash_batch"
PEAK_REFRACTORY_SECONDS = 0.35
PAIR_MAX_GAP_SECONDS = 0.75
ONSET_PEAK_WINDOW_SAMPLES = 10
AUDIO_CHANNELS = 1
AUDIO_DTYPE = "int16"
AUDIO_BLOCK_SECONDS = 0.010
AUDIO_PROBE_SECONDS = 0.60
AUDIO_MIN_VALID_PEAK_ABS = 2.0

WINDOW_HALF_WIDTH_SECONDS = 2.5
MIN_EVENT_SEPARATION_SECONDS = 3.0


# ---------------------------------------------------------------------------
# Utility helpers (verbatim from sync_vr_imu_flash_test.py)
# ---------------------------------------------------------------------------

def status_print(*args, **kwargs):
    kwargs.setdefault("file", sys.stderr)
    print(*args, **kwargs)


def _flush_c_stdout():
    if os.name != "nt":
        return
    try:
        import ctypes
        ctypes.CDLL("msvcrt").fflush(None)
    except Exception:
        pass


@contextmanager
def suppress_native_stdout(enabled: bool = True):
    if not enabled:
        yield
        return
    sys.stdout.flush()
    _flush_c_stdout()
    saved_fd = os.dup(1)
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    old_windows_stdout = None
    kernel32 = None
    try:
        os.dup2(devnull_fd, 1)
        if os.name == "nt":
            try:
                import ctypes
                import msvcrt
                kernel32 = ctypes.windll.kernel32
                old_windows_stdout = kernel32.GetStdHandle(-11)
                kernel32.SetStdHandle(-11, msvcrt.get_osfhandle(devnull_fd))
            except Exception:
                old_windows_stdout = None
        yield
    finally:
        sys.stdout.flush()
        _flush_c_stdout()
        if old_windows_stdout is not None and kernel32 is not None:
            try:
                kernel32.SetStdHandle(-11, old_windows_stdout)
            except Exception:
                pass
        os.dup2(saved_fd, 1)
        os.close(saved_fd)
        os.close(devnull_fd)


# ---------------------------------------------------------------------------
# CSV column definitions
# ---------------------------------------------------------------------------

EYE_COLUMNS = [
    "device_timestamp", "pc_arrival_timestamp",
    "gaze_x", "gaze_y", "gaze_z",
    "left_pupil_x", "left_pupil_y", "right_pupil_x", "right_pupil_y",
    "left_pupil_diameter_mm", "right_pupil_diameter_mm",
    "left_openness", "right_openness", "left_blink", "right_blink",
    "gyro_timestamp",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_z",
]

BRIGHTNESS_COLUMNS = [
    "eye", "eye_name", "eye_device_timestamp", "eye_pc_arrival_timestamp",
    "width", "height", "mean_brightness", "max_brightness", "p99_brightness",
]

MIC_COLUMNS = [
    "block_start_pc_timestamp", "callback_pc_timestamp",
    "sample_index", "frames", "rms", "dbfs", "peak_abs",
    "status_flags", "clock_source",
]

SENSOR_COLUMNS = [
    "Red", "IR", "Green",
    "accX", "accY", "accZ",
    "gyrX", "gyrY", "gyrZ",
    "magX", "magY", "magZ",
    "temp", "device_timestamp_ms", "pc_arrival_timestamp",
]


# ---------------------------------------------------------------------------
# SensorReader / CsvLog / MicLoudnessRecorder  (verbatim copies)
# ---------------------------------------------------------------------------

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


class SensorReader:
    def __init__(self, role: str, port: str, baudrate: int):
        self.role = role
        self.port = port
        self.baudrate = baudrate
        self.serial_port: serial.Serial | None = None
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
            status_print(f"[sensor] {self.port} open failed: {exc}")
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

            self.packet_queue.put({
                "pc_arrival_timestamp": pc_arrival_timestamp,
                "red": values[0], "ir": values[1], "green": values[2],
                "ax": values[3], "ay": values[4], "az": values[5],
                "gx": values[6], "gy": values[7], "gz": values[8],
                "mx": values[9], "my": values[10], "mz": values[11],
                "temp": values[12], "device_timestamp_ms": device_timestamp_ms,
            })

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


class MicLoudnessRecorder:
    def __init__(self, device_index, sample_rate: int):
        if sd is None:
            raise RuntimeError(f"sounddevice is not available: {SOUNDDEVICE_IMPORT_ERROR}")
        self.device_index = None if device_index is None else int(device_index)
        self.sample_rate = int(sample_rate)
        self.blocksize = max(1, int(round(self.sample_rate * AUDIO_BLOCK_SECONDS)))
        self.queue: queue.SimpleQueue[dict] = queue.SimpleQueue()
        self._stream = None
        self._sample_index = 0
        self._lock = threading.Lock()
        self.blocks = 0
        self.frames = 0

    def start(self):
        sd.check_input_settings(
            device=self.device_index,
            channels=AUDIO_CHANNELS,
            samplerate=self.sample_rate,
            dtype=AUDIO_DTYPE,
        )
        self._stream = sd.InputStream(
            device=self.device_index,
            channels=AUDIO_CHANNELS,
            samplerate=self.sample_rate,
            dtype=AUDIO_DTYPE,
            blocksize=self.blocksize,
            latency="low",
            callback=self._callback,
        )
        self._stream.start()

    def stop(self):
        if self._stream is None:
            return
        try:
            self._stream.stop()
        finally:
            self._stream.close()
            self._stream = None

    def _callback(self, indata, frames, time_info, status):
        callback_pc = time.perf_counter()
        block_start = callback_pc - (float(frames) / float(self.sample_rate))
        samples = np.asarray(indata, dtype=np.int16).reshape(-1)
        values = samples.astype(np.float64, copy=False)
        if values.size:
            rms = float(np.sqrt(np.mean(values * values)))
            peak_abs = float(np.max(np.abs(values)))
        else:
            rms = 0.0
            peak_abs = 0.0
        dbfs = -90.0 if rms <= 0.0 else max(-90.0, min(0.0, 20.0 * np.log10(rms / 32768.0)))
        status_flags = str(status).strip() if status else ""

        with self._lock:
            sample_index = self._sample_index
            self._sample_index += int(frames)
            self.blocks += 1
            self.frames += int(frames)

        self.queue.put({
            "block_start_pc_timestamp": float(block_start),
            "callback_pc_timestamp": float(callback_pc),
            "sample_index": int(sample_index),
            "frames": int(frames),
            "rms": rms, "dbfs": dbfs, "peak_abs": peak_abs,
            "status_flags": status_flags, "clock_source": "callback_pc_minus_block_duration",
        })

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


# ---------------------------------------------------------------------------
# Audio-device helpers (verbatim)
# ---------------------------------------------------------------------------

def input_audio_devices():
    if sd is None:
        return []
    devices = []
    try:
        hostapis = sd.query_hostapis()
    except Exception:
        hostapis = []
    try:
        default_device = sd.default.device
        default_input = int(default_device[0])
    except Exception:
        default_input = None
    try:
        for index, device in enumerate(sd.query_devices()):
            if int(device.get("max_input_channels", 0)) <= 0:
                continue
            name = str(device.get("name", ""))
            lowered = name.casefold()
            if (
                "mapper" in lowered or "映射器" in lowered
                or "主声音捕获" in lowered or "loopback" in lowered
                or "speaker" in lowered or "扬声器" in lowered
                or "电脑扬声器" in lowered or "stereo mix" in lowered
                or "立体声混音" in lowered
            ):
                continue
            hostapi_index = int(device.get("hostapi", -1))
            hostapi_name = ""
            if 0 <= hostapi_index < len(hostapis):
                try:
                    hostapi_name = str(hostapis[hostapi_index].get("name", ""))
                except Exception:
                    hostapi_name = ""
            devices.append({
                "index": index, "name": name,
                "hostapi": hostapi_index, "hostapi_name": hostapi_name,
                "default_samplerate": int(round(float(device.get("default_samplerate", 0) or 0))),
                "is_default": default_input == index,
            })
    except Exception:
        return devices
    devices.sort(key=audio_candidate_sort_key, reverse=True)
    return devices


def audio_candidate_sort_key(device: dict):
    name = str(device.get("name", "")).casefold()
    hostapi = str(device.get("hostapi_name", "")).casefold()
    hostapi_index = int(device.get("hostapi", -1))
    preference = 0
    if "vive" in name:
        preference += 1000
    if "麦克风阵列" in name or "microphone array" in name:
        preference += 450
    if "麦克风" in name or "microphone" in name:
        preference += 250
    if "wdm-ks" in hostapi or hostapi_index == 3:
        preference += 120
    if "wasapi" in hostapi or hostapi_index == 2:
        preference += 100
    if device.get("is_default"):
        preference += 40
    return preference, int(device.get("default_samplerate") or 0)


def probe_audio_device_health(device_index: int, sample_rate: int) -> dict:
    blocksize = max(1, int(round(float(sample_rate) * AUDIO_BLOCK_SECONDS)))
    callbacks = 0
    frames_total = 0
    max_rms = 0.0
    max_peak_abs = 0.0
    first_callback = None
    last_callback = None
    statuses = set()

    def callback(indata, frames, _time_info, status):
        nonlocal callbacks, frames_total, max_rms, max_peak_abs, first_callback, last_callback
        callback_pc = time.perf_counter()
        if first_callback is None:
            first_callback = callback_pc
        last_callback = callback_pc
        callbacks += 1
        frames_total += int(frames)
        samples = np.asarray(indata, dtype=np.int16).reshape(-1)
        values = samples.astype(np.float64, copy=False)
        if values.size:
            max_rms = max(max_rms, float(np.sqrt(np.mean(values * values))))
            max_peak_abs = max(max_peak_abs, float(np.max(np.abs(values))))
        if status:
            text = str(status).strip()
            if text:
                statuses.add(text)

    try:
        stream = sd.InputStream(
            device=int(device_index),
            channels=AUDIO_CHANNELS,
            samplerate=int(sample_rate),
            dtype=AUDIO_DTYPE,
            blocksize=blocksize,
            latency="low",
            callback=callback,
        )
        stream.start()
        try:
            time.sleep(AUDIO_PROBE_SECONDS)
        finally:
            stream.stop()
            stream.close()
    except Exception as exc:
        return {
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
            "callbacks": int(callbacks), "frames": int(frames_total),
            "pc_span_seconds": 0.0, "audio_seconds": 0.0,
            "max_rms": float(max_rms), "max_peak_abs": float(max_peak_abs),
            "statuses": sorted(statuses),
        }

    pc_span = float((last_callback - first_callback) if first_callback is not None and last_callback is not None else 0.0)
    audio_seconds = float(frames_total) / float(sample_rate) if sample_rate else 0.0
    ok = (
        callbacks >= max(5, int(AUDIO_PROBE_SECONDS / AUDIO_BLOCK_SECONDS * 0.25))
        and pc_span >= AUDIO_PROBE_SECONDS * 0.35
        and max_peak_abs >= AUDIO_MIN_VALID_PEAK_ABS
    )
    return {
        "ok": bool(ok), "callbacks": int(callbacks), "frames": int(frames_total),
        "pc_span_seconds": pc_span, "audio_seconds": audio_seconds,
        "max_rms": float(max_rms), "max_peak_abs": float(max_peak_abs),
        "statuses": sorted(statuses),
    }


def choose_audio_device(explicit_device, explicit_sample_rate):
    if sd is None:
        raise RuntimeError(f"sounddevice is not available: {SOUNDDEVICE_IMPORT_ERROR}")
    candidates = input_audio_devices()
    if explicit_device is not None:
        info = sd.query_devices(int(explicit_device))
        hostapis = sd.query_hostapis()
        hostapi_index = int(info.get("hostapi", -1))
        hostapi_name = str(hostapis[hostapi_index].get("name", "")) if 0 <= hostapi_index < len(hostapis) else ""
        sample_rate = int(explicit_sample_rate or round(float(info.get("default_samplerate", DEFAULT_AUDIO_SAMPLE_RATE))))
        return int(explicit_device), sample_rate, {
            "device": int(explicit_device), "name": str(info.get("name", "")),
            "hostapi": hostapi_index, "hostapi_name": hostapi_name,
            "sample_rate": sample_rate, "selection": "explicit",
        }
    if not candidates:
        raise RuntimeError("No microphone input device found.")
    vive_candidates = [device for device in candidates if "vive" in str(device.get("name", "")).casefold()]
    ordered_candidates = vive_candidates + [device for device in candidates if device not in vive_candidates]
    probe_results = []
    chosen = None
    chosen_probe = None
    for candidate in ordered_candidates:
        sample_rate = int(explicit_sample_rate or candidate["default_samplerate"] or DEFAULT_AUDIO_SAMPLE_RATE)
        probe = probe_audio_device_health(int(candidate["index"]), sample_rate)
        probe_results.append({
            "device": int(candidate["index"]), "name": candidate["name"],
            "hostapi_name": candidate["hostapi_name"], "sample_rate": sample_rate,
            **probe,
        })
        if probe["ok"]:
            chosen = candidate
            chosen_probe = probe
            break
    if chosen is None:
        details = "; ".join(
            f"{item['device']} {item['name']} ({item['hostapi_name']}): "
            f"callbacks={item['callbacks']}, span={item['pc_span_seconds']:.3f}s, "
            f"peak={item['max_peak_abs']:.1f}"
            for item in probe_results
        )
        raise RuntimeError(f"No usable microphone input produced a live nonzero signal. Probe results: {details}")
    sample_rate = int(explicit_sample_rate or chosen["default_samplerate"] or DEFAULT_AUDIO_SAMPLE_RATE)
    selection = "auto_vive_probe" if "vive" in str(chosen.get("name", "")).casefold() else "auto_probe_fallback"
    return int(chosen["index"]), sample_rate, {
        "device": int(chosen["index"]), "name": chosen["name"],
        "hostapi": chosen["hostapi"], "hostapi_name": chosen["hostapi_name"],
        "sample_rate": sample_rate, "selection": selection,
        "probe": chosen_probe, "probe_results": probe_results,
    }


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


# ---------------------------------------------------------------------------
# CSV row formatters (verbatim)
# ---------------------------------------------------------------------------

def eye_csv_row(sample: dict) -> list[str]:
    return [
        str(int(sample["device_timestamp"])),
        f"{sample['pc_arrival_timestamp']:.9f}",
        f"{sample['gaze_x']:.6f}", f"{sample['gaze_y']:.6f}", f"{sample['gaze_z']:.6f}",
        f"{sample['left_pupil_x']:.6f}", f"{sample['left_pupil_y']:.6f}",
        f"{sample['right_pupil_x']:.6f}", f"{sample['right_pupil_y']:.6f}",
        f"{sample['left_pupil_diameter_mm']:.6f}", f"{sample['right_pupil_diameter_mm']:.6f}",
        f"{sample['left_openness']:.6f}", f"{sample['right_openness']:.6f}",
        str(int(sample["left_blink"])), str(int(sample["right_blink"])),
        str(int(sample.get("gyro_timestamp", 0))),
        f"{sample.get('accel_x', 0.0):.6f}", f"{sample.get('accel_y', 0.0):.6f}",
        f"{sample.get('accel_z', 0.0):.6f}", f"{sample.get('gyro_x', 0.0):.6f}",
        f"{sample.get('gyro_y', 0.0):.6f}", f"{sample.get('gyro_z', 0.0):.6f}",
        f"{sample.get('mag_x', 0.0):.6f}", f"{sample.get('mag_y', 0.0):.6f}",
        f"{sample.get('mag_z', 0.0):.6f}",
    ]


def sensor_csv_row(sample: dict) -> list[str]:
    return [
        str(int(sample["red"])), str(int(sample["ir"])), str(int(sample["green"])),
        f"{sample['ax']:.6f}", f"{sample['ay']:.6f}", f"{sample['az']:.6f}",
        f"{sample['gx']:.6f}", f"{sample['gy']:.6f}", f"{sample['gz']:.6f}",
        f"{sample['mx']:.6f}", f"{sample['my']:.6f}", f"{sample['mz']:.6f}",
        f"{sample['temp']:.6f}",
        f"{sample['device_timestamp_ms']:.3f}",
        f"{sample['pc_arrival_timestamp']:.9f}",
    ]


def mic_csv_row(sample: dict) -> list[str]:
    return [
        f"{sample['block_start_pc_timestamp']:.9f}",
        f"{sample['callback_pc_timestamp']:.9f}",
        str(int(sample["sample_index"])), str(int(sample["frames"])),
        f"{sample['rms']:.6f}", f"{sample['dbfs']:.6f}", f"{sample['peak_abs']:.6f}",
        sample["status_flags"], sample["clock_source"],
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
        "width": int(frame.width), "height": int(frame.height),
        "mean_brightness": mean,
        "max_brightness": int(pixels.max()),
        "p99_brightness": float(np.percentile(pixels, 99)),
    }
    return (
        [
            str(row_dict["eye"]), row_dict["eye_name"],
            str(row_dict["eye_device_timestamp"]),
            f"{row_dict['eye_pc_arrival_timestamp']:.9f}",
            str(row_dict["width"]), str(row_dict["height"]),
            f"{row_dict['mean_brightness']:.6f}",
            str(row_dict["max_brightness"]),
            f"{row_dict['p99_brightness']:.6f}",
        ],
        row_dict,
    )


# ---------------------------------------------------------------------------
# Analysis helpers (verbatim copies)
# ---------------------------------------------------------------------------

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


def dynamic_strength_threshold(times: np.ndarray, strength: np.ndarray, ignore_initial: float) -> float:
    keep = np.isfinite(strength)
    if times.size == strength.size and ignore_initial > 0 and times.size:
        keep &= times >= (float(times[0]) + ignore_initial)
    values = np.asarray(strength[keep], dtype=np.float64)
    values = values[values > 0.0]
    if values.size == 0:
        raise RuntimeError("Not enough nonzero signal samples to generate a dynamic onset threshold.")
    if values.size == 1:
        return max(float(values[0]) * 0.5, 1e-6)
    if values.size == 2:
        return float(np.mean(values))
    ordered = np.sort(np.log1p(values))
    if ordered.size < 3:
        return float(np.expm1(np.median(ordered)))
    gaps = np.diff(ordered)
    if gaps.size == 0:
        return float(np.expm1(ordered[-1]))
    split_index = int(np.argmax(gaps))
    threshold_log = (ordered[split_index] + ordered[split_index + 1]) * 0.5
    return float(np.expm1(threshold_log))


def detect_upward_jump_events(
    times, values, strength, threshold, ignore_initial,
    refractory_seconds=PEAK_REFRACTORY_SECONDS,
    onset_window_samples=ONSET_PEAK_WINDOW_SAMPLES,
):
    if times.size == 0:
        return []
    keep = np.ones(times.size, dtype=bool)
    if ignore_initial > 0:
        keep &= times >= (float(times[0]) + ignore_initial)
    candidate = np.where(keep & (strength >= threshold))[0]
    if candidate.size == 0:
        return []
    events = []
    cursor = 0
    while cursor < candidate.size:
        group = [int(candidate[cursor])]
        cursor += 1
        while cursor < candidate.size and times[int(candidate[cursor])] <= times[group[-1]] + refractory_seconds:
            group.append(int(candidate[cursor]))
            cursor += 1
        peak_idx = max(group, key=lambda i: strength[i])
        onset_window_start = max(0, peak_idx - onset_window_samples)
        onset_window_end = min(len(times) - 1, peak_idx + onset_window_samples)
        local_candidates = [
            int(index)
            for index in candidate
            if onset_window_start <= int(index) <= onset_window_end
        ]
        onset_idx = min(local_candidates) if local_candidates else peak_idx
        events.append({
            "index": int(onset_idx), "time": float(times[onset_idx]),
            "value": float(values[onset_idx]), "strength": float(strength[onset_idx]),
            "peak_index": int(peak_idx), "peak_time": float(times[peak_idx]),
            "peak_value": float(values[peak_idx]), "peak_strength": float(strength[peak_idx]),
            "threshold": float(threshold), "above_threshold": True,
        })
    return events


def load_csv_dicts(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open("r", newline="", encoding="utf-8") as fp:
        return list(csv.DictReader(fp))


# ---------------------------------------------------------------------------
# Capture session (reused, duration extended)
# ---------------------------------------------------------------------------

class CaptureSession:
    def __init__(self, args):
        self.args = args
        self.output_dir = self._make_output_dir()
        self.sdk = VrUserSdkWrapper()
        self.sensor_readers: dict[str, SensorReader] = {}
        self.mic = None
        self.audio_device_index = None
        self.audio_sample_rate = None
        self.audio_device_info = None
        self.stop_requested = False
        self.logs: dict[str, CsvLog] = {}

    def request_stop(self):
        self.stop_requested = True

    def run(self) -> dict:
        with suppress_native_stdout(not self.args.show_sdk_output):
            try:
                self._start_logs()
                self._start_vr()
                self._start_sensors()
                self._start_mic()
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
        self.logs["mic"] = CsvLog(self.output_dir / "mic_loudness.csv", MIC_COLUMNS)

    def _start_mic(self):
        device_index, sample_rate, device_info = choose_audio_device(
            self.args.audio_device, self.args.audio_sample_rate)
        self.audio_device_index = int(device_index)
        self.audio_sample_rate = int(sample_rate)
        self.audio_device_info = device_info
        self.mic = MicLoudnessRecorder(device_index, sample_rate)
        self.mic.start()
        probe = device_info.get("probe") or {}
        probe_text = ""
        if probe:
            probe_text = (
                f", probe peak {probe.get('max_peak_abs', 0.0):.1f}, "
                f"callbacks {probe.get('callbacks', 0)}"
            )
        status_print(
            f"Microphone started on device {device_index} at {sample_rate} Hz "
            f"({device_info.get('name', 'unknown')}, {device_info.get('hostapi_name', '')}, "
            f"{device_info.get('selection', '')}{probe_text}).")

    def _start_vr(self):
        vr_bin = os.fspath(vr_sdk_bin_dir(self.args.vr_sdk_root))
        self.sdk.load_library(vr_bin)
        self.sdk.connect()
        self.sdk.start()
        status_print("VR eye tracker started.")

    def _start_sensors(self):
        self.sensor_readers = detect_sensor_readers(self.args.baud)
        if "watch" not in self.sensor_readers:
            found = ", ".join(sorted(self.sensor_readers)) or "none"
            raise RuntimeError(
                f"Head IMU sensor not found. Expected COM device identity 'watch', found: {found}.")
        for role, reader in self.sensor_readers.items():
            self.logs[f"sensor_{role}"] = CsvLog(
                self.output_dir / f"sensor_{role}.csv", SENSOR_COLUMNS)
            if reader.serial_port:
                reader.serial_port.reset_input_buffer()
                reader.serial_port.reset_output_buffer()
            reader.start()
            reader.send_command("s\n")
            status_print(f"{role} sensor started on {reader.port}.")

    def _capture_loop(self):
        start = time.perf_counter()
        next_print = start
        status_print(
            f"Capturing for {self.args.duration:.0f} s. "
            f"Trigger multiple impact + flash events during this window "
            f"(≥{MIN_EVENT_SEPARATION_SECONDS:.0f} s apart).")
        while not self.stop_requested:
            now = time.perf_counter()
            if now - start >= self.args.duration:
                break
            self._drain_all()
            if now >= next_print:
                next_print = now + 1.0
                watch_rows = self.logs.get("sensor_watch").rows if self.logs.get("sensor_watch") else 0
                status_print(
                    f"\rElapsed {now - start:6.1f}s | "
                    f"Head IMU rows {watch_rows} | "
                    f"brightness rows {self.logs['brightness'].rows} | "
                    f"mic rows {self.logs['mic'].rows}",
                    end="", flush=True)
            time.sleep(0.003)
        self._drain_all()
        status_print()

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
        if self.mic is not None:
            while True:
                try:
                    sample = self.mic.queue.get_nowait()
                except queue.Empty:
                    break
                self.logs["mic"].write(mic_csv_row(sample))

    def _cleanup(self):
        if self.mic is not None:
            try:
                self.mic.stop()
                self._drain_all()
            except Exception as exc:
                status_print(f"[mic] stop failed: {exc}")
            self.mic = None
        for reader in self.sensor_readers.values():
            try:
                reader.send_command("e\n")
            except Exception:
                pass
        for reader in self.sensor_readers.values():
            try:
                reader.stop()
            except Exception as exc:
                status_print(f"[sensor] stop failed: {exc}")
        try:
            self.sdk.disconnect()
        except Exception as exc:
            status_print(f"[vr] disconnect failed: {exc}")
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
            "serial_sensors_enabled": True,
            "rows": {name: log.rows for name, log in self.logs.items()},
            "sensor_ports": {role: reader.port for role, reader in self.sensor_readers.items()},
            "audio_device": self.audio_device_index,
            "audio_sample_rate": self.audio_sample_rate,
            "audio_device_info": self.audio_device_info,
        }
        with (self.output_dir / "capture_metadata.json").open("w", encoding="utf-8") as fp:
            json.dump(metadata, fp, indent=2)
        return metadata


# ===================================================================
# New: window extraction & interactive review
# ===================================================================

def extract_windows_from_full_capture(
    output_dir: Path,
    ignore_initial: float,
    imu_signal: str,
):
    """Detect all IMU events across the full capture and partition them into
    non-overlapping ±WINDOW_HALF_WIDTH_SECONDS windows separated by at least
    MIN_EVENT_SEPARATION_SECONDS."""

    watch_rows = load_csv_dicts(output_dir / "sensor_watch.csv")
    brightness_rows = load_csv_dicts(output_dir / "eye_brightness.csv")
    mic_rows = load_csv_dicts(output_dir / "mic_loudness.csv")
    eye_rows = load_csv_dicts(output_dir / "vr_eye_samples.csv")

    if not watch_rows:
        raise RuntimeError("No Head IMU rows captured.")
    if not brightness_rows:
        raise RuntimeError("No eye brightness rows captured.")
    if not mic_rows:
        raise RuntimeError("No microphone loudness rows captured.")

    # --- IMU strength --------------------------------------------------
    imu_times = np.array([float(r["pc_arrival_timestamp"]) for r in watch_rows], dtype=np.float64)
    accel = np.array([[float(r["accX"]), float(r["accY"]), float(r["accZ"])] for r in watch_rows], dtype=np.float64)
    gyro = np.array([[float(r["gyrX"]), float(r["gyrY"]), float(r["gyrZ"])] for r in watch_rows], dtype=np.float64)
    accel_norm = np.linalg.norm(accel, axis=1)
    gyro_norm = np.linalg.norm(gyro, axis=1)

    accel_analysis = robust_strength(imu_times, accel_norm, baseline_seconds=0.35)
    gyro_analysis = robust_strength(imu_times, gyro_norm, baseline_seconds=0.35)
    if imu_signal == "accel":
        imu_values = accel_norm
        imu_strength = accel_analysis["strength"]
    elif imu_signal == "gyro":
        imu_values = gyro_norm
        imu_strength = gyro_analysis["strength"]
    else:
        imu_values = np.maximum(accel_analysis["centered"], gyro_analysis["centered"])
        imu_strength = np.maximum(accel_analysis["strength"], gyro_analysis["strength"])

    imu_threshold = dynamic_strength_threshold(imu_times, imu_strength, ignore_initial)
    imu_events = detect_upward_jump_events(
        imu_times, imu_values, imu_strength, imu_threshold, ignore_initial)

    if len(imu_events) < 2:
        raise RuntimeError(
            f"Only {len(imu_events)} IMU event(s) detected in the 120 s capture. "
            f"Trigger the eye tracker + mic multiple times (≥5 s apart).")

    # --- Select anchor events spaced ≥ MIN_EVENT_SEPARATION_SECONDS ----
    anchor_events = [imu_events[0]]
    for event in imu_events[1:]:
        if event["time"] - anchor_events[-1]["time"] >= MIN_EVENT_SEPARATION_SECONDS:
            anchor_events.append(event)

    # --- Build window descriptors --------------------------------------
    window_half = WINDOW_HALF_WIDTH_SECONDS
    windows = []
    for idx, anchor in enumerate(anchor_events):
        t0 = anchor["time"] - window_half
        t1 = anchor["time"] + window_half
        windows.append({
            "window_index": idx,
            "imu_anchor": anchor,
            "t_min": t0,
            "t_max": t1,
        })
    return windows, {
        "imu_times": imu_times,
        "imu_values": imu_values,
        "accel_norm": accel_norm,
        "gyro_norm": gyro_norm,
        "imu_strength": imu_strength,
        "imu_threshold": imu_threshold,
        "watch_rows": watch_rows,
        "brightness_rows": brightness_rows,
        "mic_rows": mic_rows,
        "eye_rows": eye_rows,
    }


def trim_data_to_window(
    window: dict,
    data: dict,
    ignore_initial: float,
):
    """Slice all raw data to [t_min, t_max] and re-run onset detection on the slice."""
    t_min, t_max = window["t_min"], window["t_max"]
    imu_times_full = data["imu_times"]

    # --- slice watch (IMU + PPG) ---
    watch_rows = data["watch_rows"]
    watch_slice = [
        r for r in watch_rows
        if t_min <= float(r["pc_arrival_timestamp"]) <= t_max
    ]
    imu_times = np.array([float(r["pc_arrival_timestamp"]) for r in watch_slice], dtype=np.float64)
    accel = np.array(
        [[float(r["accX"]), float(r["accY"]), float(r["accZ"])] for r in watch_slice],
        dtype=np.float64)
    gyro = np.array(
        [[float(r["gyrX"]), float(r["gyrY"]), float(r["gyrZ"])] for r in watch_slice],
        dtype=np.float64)
    accel_norm = np.linalg.norm(accel, axis=1) if accel.size else np.array([], dtype=np.float64)
    gyro_norm = np.linalg.norm(gyro, axis=1) if gyro.size else np.array([], dtype=np.float64)
    ppg_red = np.array([float(r["Red"]) for r in watch_slice], dtype=np.float64)
    ppg_ir = np.array([float(r["IR"]) for r in watch_slice], dtype=np.float64)
    ppg_green = np.array([float(r["Green"]) for r in watch_slice], dtype=np.float64)

    # --- slice brightness ---
    brightness_rows = data["brightness_rows"]
    bright_slice = [
        r for r in brightness_rows
        if t_min <= float(r["eye_pc_arrival_timestamp"]) <= t_max
    ]
    brightness_by_eye: dict[int, list[dict]] = {1: [], 2: []}
    for r in bright_slice:
        try:
            brightness_by_eye[int(r["eye"])].append(r)
        except Exception:
            continue

    # --- slice mic ---
    mic_rows = data["mic_rows"]
    mic_slice = [
        r for r in mic_rows
        if t_min <= float(r["block_start_pc_timestamp"]) <= t_max
    ]
    mic_times = np.array([float(r["block_start_pc_timestamp"]) for r in mic_slice], dtype=np.float64)
    mic_rms = np.array([float(r["rms"]) for r in mic_slice], dtype=np.float64)

    # --- slice eye ---
    eye_rows = data["eye_rows"]
    eye_slice = [
        r for r in eye_rows
        if t_min <= float(r["pc_arrival_timestamp"]) <= t_max
    ]

    # === re-detect onsets on the windowed data ===

    # IMU
    accel_analysis = robust_strength(imu_times, accel_norm, baseline_seconds=0.35)
    gyro_analysis = robust_strength(imu_times, gyro_norm, baseline_seconds=0.35)
    imu_values = np.maximum(accel_analysis["centered"], gyro_analysis["centered"])
    imu_strength = np.maximum(accel_analysis["strength"], gyro_analysis["strength"])
    try:
        imu_threshold = dynamic_strength_threshold(imu_times, imu_strength, ignore_initial)
    except RuntimeError:
        imu_threshold = 1e-6
    imu_events = detect_upward_jump_events(
        imu_times, imu_values, imu_strength, imu_threshold, ignore_initial)

    # PPG
    ppg_red_analysis = robust_strength(imu_times, ppg_red, baseline_seconds=0.35)
    ppg_ir_analysis = robust_strength(imu_times, ppg_ir, baseline_seconds=0.35)
    ppg_green_analysis = robust_strength(imu_times, ppg_green, baseline_seconds=0.35)
    ppg_values = np.maximum.reduce([
        ppg_red_analysis["centered"],
        ppg_ir_analysis["centered"],
        ppg_green_analysis["centered"],
    ])
    ppg_strength = np.maximum.reduce([
        ppg_red_analysis["strength"],
        ppg_ir_analysis["strength"],
        ppg_green_analysis["strength"],
    ])
    try:
        ppg_threshold = dynamic_strength_threshold(imu_times, ppg_strength, ignore_initial)
    except RuntimeError:
        ppg_threshold = 1e-6
    ppg_events = detect_upward_jump_events(
        imu_times, ppg_values, ppg_strength, ppg_threshold, ignore_initial)

    # Mic
    mic_analysis = robust_strength(mic_times, mic_rms, baseline_seconds=0.20)
    try:
        mic_threshold = dynamic_strength_threshold(mic_times, mic_analysis["strength"], ignore_initial)
    except RuntimeError:
        mic_threshold = 1e-6
    mic_events = detect_upward_jump_events(
        mic_times, mic_rms, mic_analysis["strength"], mic_threshold, ignore_initial)

    # Eye brightness
    eye_traces = {}
    eye_events_all = []
    for eye, rows in brightness_by_eye.items():
        if not rows:
            continue
        times = np.array([float(r["eye_pc_arrival_timestamp"]) for r in rows], dtype=np.float64)
        values = np.array([float(r["mean_brightness"]) for r in rows], dtype=np.float64)
        analysis = robust_strength(times, values, baseline_seconds=0.25)
        try:
            threshold = dynamic_strength_threshold(times, analysis["strength"], ignore_initial)
        except RuntimeError:
            threshold = 1e-6
        events = detect_upward_jump_events(
            times, values, analysis["strength"], threshold, ignore_initial)
        eye_traces[eye] = {"times": times, "values": values, "analysis": analysis,
                           "threshold": threshold, "events": events}
        for evt in events:
            evt_with_eye = dict(evt)
            evt_with_eye["eye"] = eye
            eye_events_all.append(evt_with_eye)

    # --- pick best onset per channel --------------------------------
    def _best_event(events_list):
        if not events_list:
            return None
        return max(events_list, key=lambda e: e["peak_strength"])

    imu_onset = _best_event(imu_events)
    ppg_onset = _best_event(ppg_events)
    mic_onset = _best_event(mic_events)
    eye_onset = _best_event(eye_events_all)

    # Fallback: pick sample nearest to window centre
    centre = (t_min + t_max) / 2.0
    def _fallback_event(times_arr, values_arr, strength_arr, threshold_val, eye=None):
        if times_arr.size == 0:
            return None
        idx = int(np.argmin(np.abs(times_arr - centre)))
        event = {
            "index": int(idx), "time": float(times_arr[idx]),
            "value": float(values_arr[idx]), "strength": float(strength_arr[idx]),
            "peak_index": int(idx), "peak_time": float(times_arr[idx]),
            "peak_value": float(values_arr[idx]), "peak_strength": float(strength_arr[idx]),
            "threshold": float(threshold_val), "above_threshold": bool(strength_arr[idx] >= threshold_val),
        }
        if eye is not None:
            event["eye"] = int(eye)
        return event

    if imu_onset is None:
        imu_onset = _fallback_event(imu_times, imu_values, imu_strength, imu_threshold)
    if ppg_onset is None:
        ppg_onset = _fallback_event(imu_times, ppg_values, ppg_strength, ppg_threshold)
    if mic_onset is None:
        mic_onset = _fallback_event(mic_times, mic_rms, mic_analysis["strength"], mic_threshold)
    if eye_onset is None:
        # Try left first, then right
        for eye in (1, 2):
            trace = eye_traces.get(eye)
            if trace and trace["times"].size:
                eye_onset = _fallback_event(
                    trace["times"], trace["values"],
                    trace["analysis"]["strength"], trace["threshold"], eye=eye)
                break

    # --- final check ---
    if imu_onset is None:
        imu_onset = {"time": centre, "value": 0.0, "strength": 0.0, "peak_strength": 0.0,
                      "threshold": 1e-6, "above_threshold": False,
                      "index": 0, "peak_index": 0, "peak_time": centre, "peak_value": 0.0}
    if eye_onset is None:
        eye_onset = {"time": centre, "value": 0.0, "strength": 0.0, "peak_strength": 0.0,
                      "threshold": 1e-6, "above_threshold": False, "eye": 1,
                      "index": 0, "peak_index": 0, "peak_time": centre, "peak_value": 0.0}
    if mic_onset is None:
        mic_onset = {"time": centre, "value": 0.0, "strength": 0.0, "peak_strength": 0.0,
                      "threshold": 1e-6, "above_threshold": False,
                      "index": 0, "peak_index": 0, "peak_time": centre, "peak_value": 0.0}
    if ppg_onset is None:
        ppg_onset = {"time": centre, "value": 0.0, "strength": 0.0, "peak_strength": 0.0,
                      "threshold": 1e-6, "above_threshold": False,
                      "index": 0, "peak_index": 0, "peak_time": centre, "peak_value": 0.0}

    return {
        "window": window,
        "watch_slice": watch_slice,
        "bright_slice": bright_slice,
        "mic_slice": mic_slice,
        "eye_slice": eye_slice,
        "imu_times": imu_times,
        "imu_values": imu_values,
        "accel_norm": accel_norm,
        "gyro_norm": gyro_norm,
        "imu_strength": imu_strength,
        "imu_threshold": imu_threshold,
        "imu_events": imu_events,
        "eye_traces": eye_traces,
        "eye_onset": eye_onset,
        "mic_times": mic_times,
        "mic_rms": mic_rms,
        "mic_analysis": mic_analysis,
        "mic_threshold": mic_threshold,
        "mic_events": mic_events,
        "ppg_red": ppg_red,
        "ppg_ir": ppg_ir,
        "ppg_green": ppg_green,
        "ppg_values": ppg_values,
        "ppg_strength": ppg_strength,
        "ppg_threshold": ppg_threshold,
        "ppg_events": ppg_events,
        "imu_onset": imu_onset,
        "ppg_onset": ppg_onset,
        "mic_onset": mic_onset,
    }


class _ReviewFigure:
    """Single reusable figure for interactive sync-window review.

    The figure is created once and its content is replaced for each window
    via :meth:`review`.  Keyboard / mouse callbacks are connected once.
    """

    def __init__(self):
        import matplotlib.pyplot as plt

        plt.ion()
        self.fig, self.axes = plt.subplots(5, 1, figsize=(15, 11), sharex=True)
        self._hint = self.fig.text(
            0.5, 0.008,
            "Click a trace to adjust onset.  [a] Accept  [r] Reject  [q] Quit batch",
            ha="center", va="bottom", fontsize=9, color="#555555")
        self._title = self.fig.suptitle("", fontsize=12)

        # Per-window mutable state (overwritten by review())
        self._wd = None
        self._selection = None
        self._decision = None
        self._base_time = 0.0
        self._win_idx = 0
        self._total = 0

        # connect callbacks once
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.fig.show()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def review(self, window_data: dict, window_index: int, total_windows: int):
        """Update the figure with *window_data* and block until the user
        presses a / r / q or closes the window.

        Returns ``("accept", selection)``, ``("reject", None)``, or
        ``("quit", None)``.
        """
        self._wd = window_data
        self._win_idx = window_index
        self._total = total_windows
        self._decision = None
        self._base_time = (self._wd["imu_times"][0]
                           if self._wd["imu_times"].size else 0.0)

        self._selection = {
            "imu_onset": dict(self._wd["imu_onset"]),
            "eye_onset": dict(self._wd["eye_onset"]),
            "mic_onset": dict(self._wd["mic_onset"]),
            "ppg_onset": dict(self._wd["ppg_onset"]),
            "manual_adjusted": False,
        }

        self._redraw()
        self._wait_for_decision()

        if self._decision is None:
            self._decision = "reject"

        if self._decision == "accept":
            return "accept", self._selection
        elif self._decision == "quit":
            return "quit", None
        else:
            return "reject", None

    def close(self):
        import matplotlib.pyplot as plt

        plt.close(self.fig)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _event_from_sample(times, values, strength, index, threshold, eye=None):
        event = {
            "index": int(index), "time": float(times[index]),
            "value": float(values[index]), "strength": float(strength[index]),
            "peak_index": int(index), "peak_time": float(times[index]),
            "peak_value": float(values[index]),
            "peak_strength": float(strength[index]),
            "threshold": float(threshold),
            "above_threshold": bool(strength[index] >= threshold), "manual": True,
        }
        if eye is not None:
            event["eye"] = int(eye)
        return event

    def _redraw(self):
        """Clear all axes and redraw with current ``_wd`` and ``_selection``."""
        wd = self._wd
        sel = self._selection
        bt = self._base_time

        for ax in self.axes:
            ax.cla()

        imu_times = wd["imu_times"]
        imu_values = wd["imu_values"]
        accel_norm = wd["accel_norm"]
        gyro_norm = wd["gyro_norm"]
        imu_strength = wd["imu_strength"]
        imu_threshold = wd["imu_threshold"]
        eye_traces = wd["eye_traces"]
        mic_times = wd["mic_times"]
        mic_rms = wd["mic_rms"]
        mic_strength = wd["mic_analysis"]["strength"]
        mic_threshold = wd["mic_threshold"]
        ppg_red = wd["ppg_red"]
        ppg_ir = wd["ppg_ir"]
        ppg_green = wd["ppg_green"]
        ppg_values = wd["ppg_values"]
        ppg_strength = wd["ppg_strength"]
        ppg_threshold = wd["ppg_threshold"]

        x_imu = imu_times - bt if imu_times.size else np.array([])
        x_mic = mic_times - bt if mic_times.size else np.array([])
        colors = {1: "tab:green", 2: "black"}

        # Axes 0: IMU -------------------------------------------------
        ax0 = self.axes[0]
        if accel_norm.size:
            ax0.plot(x_imu, accel_norm, label="Accel norm", color="tab:blue", lw=1.0)
        if gyro_norm.size:
            ax0.plot(x_imu, gyro_norm, label="Gyro norm", color="tab:orange", lw=1.0)
        ax0.axvline(sel["imu_onset"]["time"] - bt, color="tab:red", lw=1.4, label="IMU onset")
        ax0.set_ylabel("IMU norm")
        ax0.set_title(f"Window {self._win_idx + 1}/{self._total}  —  Head IMU")
        ax0.legend(loc="upper right")
        ax0.grid(True, alpha=0.25)

        # Axes 1: Eye brightness --------------------------------------
        ax1 = self.axes[1]
        for eye, trace in sorted(eye_traces.items()):
            label = "Left brightness" if eye == 1 else "Right brightness"
            ax1.plot(trace["times"] - bt, trace["values"],
                     color=colors.get(eye, "gray"), lw=1.0, label=label)
        ax1.axvline(sel["eye_onset"]["time"] - bt, color="tab:purple", lw=1.4,
                    label="Eye brightness onset")
        ax1.set_ylabel("Brightness")
        ax1.set_title("Eye image mean brightness")
        ax1.legend(loc="upper right")
        ax1.grid(True, alpha=0.25)

        # Axes 2: Mic -------------------------------------------------
        ax2 = self.axes[2]
        ax2.plot(x_mic, mic_rms, label="Mic RMS", color="tab:brown", lw=1.0)
        ax2.axvline(sel["mic_onset"]["time"] - bt, color="tab:brown", lw=1.4,
                    label="Mic onset")
        ax2.set_ylabel("RMS")
        ax2.set_title("Microphone loudness")
        ax2.legend(loc="upper right")
        ax2.grid(True, alpha=0.25)

        # Axes 3: PPG -------------------------------------------------
        ax3 = self.axes[3]
        if ppg_red.size:
            ax3.plot(x_imu, ppg_red, label="Red", color="tab:red", lw=0.9)
        if ppg_ir.size:
            ax3.plot(x_imu, ppg_ir, label="IR", color="tab:orange", lw=0.9)
        if ppg_green.size:
            ax3.plot(x_imu, ppg_green, label="Green", color="tab:green", lw=0.9)
        ax3.axvline(sel["ppg_onset"]["time"] - bt, color="tab:cyan", lw=1.4,
                    label="Head PPG onset")
        ax3.set_ylabel("PPG")
        ax3.set_title("Head PPG from watch COM sensor")
        ax3.legend(loc="upper right")
        ax3.grid(True, alpha=0.25)

        # Axes 4: Strength overview -----------------------------------
        ax4 = self.axes[4]
        ax4.plot(x_imu, imu_strength, color="tab:red", lw=1.0, label="IMU strength")
        ax4.axhline(imu_threshold, color="tab:red", ls="--", lw=1.0, alpha=0.75,
                    label="IMU dynamic threshold")
        ax4.plot(x_mic, mic_strength, color="tab:brown", lw=1.0, label="Mic strength")
        ax4.axhline(mic_threshold, color="tab:brown", ls="--", lw=1.0, alpha=0.75,
                    label="Mic dynamic threshold")
        ax4.plot(x_imu, ppg_strength, color="tab:cyan", lw=1.0, label="Head PPG strength")
        ax4.axhline(ppg_threshold, color="tab:cyan", ls="--", lw=1.0, alpha=0.75,
                    label="Head PPG dynamic threshold")
        ax4.axvline(sel["imu_onset"]["time"] - bt, color="tab:red", lw=1.0, alpha=0.55)
        ax4.axvline(sel["eye_onset"]["time"] - bt, color="tab:purple", lw=1.0, alpha=0.55)
        ax4.axvline(sel["mic_onset"]["time"] - bt, color="tab:brown", lw=1.0, alpha=0.55)
        ax4.axvline(sel["ppg_onset"]["time"] - bt, color="tab:cyan", lw=1.0, alpha=0.55)
        for eye, trace in sorted(eye_traces.items()):
            label = "Left brightness strength" if eye == 1 else "Right brightness strength"
            ax4.plot(trace["times"] - bt, trace["analysis"]["strength"],
                     color=colors.get(eye, "gray"), lw=1.0, alpha=0.8, label=label)
            thr_label = "Left dynamic thr." if eye == 1 else "Right dynamic thr."
            ax4.axhline(trace["threshold"], color=colors.get(eye, "gray"),
                        ls=":", lw=1.0, alpha=0.75, label=thr_label)
        ax4.axvspan(
            min(sel["imu_onset"]["time"], sel["eye_onset"]["time"],
                sel["mic_onset"]["time"], sel["ppg_onset"]["time"]) - bt,
            max(sel["imu_onset"]["time"], sel["eye_onset"]["time"],
                sel["mic_onset"]["time"], sel["ppg_onset"]["time"]) - bt,
            color="tab:purple", alpha=0.12)
        ax4.set_xlabel("Time since window start (s)")
        ax4.set_ylabel("Noise ×")
        ax4.set_title("Dynamic onset strength")
        ax4.legend(loc="upper right", fontsize=7)
        ax4.grid(True, alpha=0.25)

        self._update_title()
        self.fig.tight_layout(rect=[0, 0.03, 1, 0.96])
        self.fig.canvas.draw_idle()

    def _update_title(self):
        sel = self._selection

        def _delta(ch):
            return (sel[ch]["time"] - sel["imu_onset"]["time"]) * 1000.0

        eye_name = "left" if sel["eye_onset"].get("eye") == 1 else "right"
        self._title.set_text(
            f"Window {self._win_idx + 1}/{self._total}  —  "
            f"eye={_delta('eye_onset'):.3f} ms  "
            f"mic={_delta('mic_onset'):.3f} ms  "
            f"ppg={_delta('ppg_onset'):.3f} ms  |  "
            f"[a] Accept  [r] Reject  [q] Quit")
        self.fig.canvas.draw_idle()

    def _wait_for_decision(self):
        """Poll GUI events until ``_decision`` is set or the figure is closed."""
        while self._decision is None:
            try:
                self.fig.canvas.flush_events()
            except Exception:
                break
            time.sleep(0.05)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _nearest_imu(self, xdata):
        wd = self._wd
        imu_times = wd["imu_times"]
        idx = int(np.argmin(np.abs(imu_times - (self._base_time + float(xdata)))))
        return self._event_from_sample(
            imu_times, wd["imu_values"], wd["imu_strength"],
            idx, wd["imu_threshold"])

    def _nearest_eye(self, mouse_event):
        best = None
        ax1 = self.axes[1]
        for eye, trace in self._wd["eye_traces"].items():
            times = trace["times"]
            if times.size == 0:
                continue
            target = self._base_time + float(mouse_event.xdata)
            center = int(np.argmin(np.abs(times - target)))
            lo = max(0, center - 3)
            hi = min(times.size, center + 4)
            for idx in range(lo, hi):
                x_disp, y_disp = ax1.transData.transform(
                    (times[idx] - self._base_time, trace["values"][idx]))
                d = (x_disp - mouse_event.x) ** 2 + (y_disp - mouse_event.y) ** 2
                if best is None or d < best[0]:
                    evt = self._event_from_sample(
                        times, trace["values"], trace["analysis"]["strength"],
                        idx, trace["threshold"], eye=eye)
                    best = (d, evt)
        return best[1] if best else None

    def _nearest_mic(self, xdata):
        wd = self._wd
        mic_times = wd["mic_times"]
        idx = int(np.argmin(np.abs(mic_times - (self._base_time + float(xdata)))))
        return self._event_from_sample(
            mic_times, wd["mic_rms"], wd["mic_analysis"]["strength"],
            idx, wd["mic_threshold"])

    def _nearest_ppg(self, xdata):
        wd = self._wd
        imu_times = wd["imu_times"]
        idx = int(np.argmin(np.abs(imu_times - (self._base_time + float(xdata)))))
        return self._event_from_sample(
            imu_times, wd["ppg_values"], wd["ppg_strength"],
            idx, wd["ppg_threshold"])

    def _on_click(self, event):
        if event.xdata is None or event.button != 1:
            return
        if event.inaxes is self.axes[0]:
            self._selection["imu_onset"] = self._nearest_imu(event.xdata)
        elif event.inaxes is self.axes[1]:
            eye_event = self._nearest_eye(event)
            if eye_event is None:
                return
            self._selection["eye_onset"] = eye_event
        elif event.inaxes is self.axes[2]:
            self._selection["mic_onset"] = self._nearest_mic(event.xdata)
        elif event.inaxes is self.axes[3]:
            self._selection["ppg_onset"] = self._nearest_ppg(event.xdata)
        else:
            return
        self._selection["manual_adjusted"] = True
        self._redraw()

    def _on_key(self, event):
        if event.key in ("a", "r", "q"):
            self._decision = {"a": "accept", "r": "reject", "q": "quit"}[event.key]

    def _on_close(self, event):
        if self._decision is None:
            self._decision = "reject"


def _sensor_csv_row_from_loaded(row: dict) -> list[str]:
    """Convert a CSV-loaded sensor dict to a row list (passthrough)."""
    return [
        row.get("Red", ""),
        row.get("IR", ""),
        row.get("Green", ""),
        row.get("accX", ""),
        row.get("accY", ""),
        row.get("accZ", ""),
        row.get("gyrX", ""),
        row.get("gyrY", ""),
        row.get("gyrZ", ""),
        row.get("magX", ""),
        row.get("magY", ""),
        row.get("magZ", ""),
        row.get("temp", ""),
        row.get("device_timestamp_ms", ""),
        row.get("pc_arrival_timestamp", ""),
    ]


def _eye_csv_row_from_loaded(row: dict) -> list[str]:
    """Convert a CSV-loaded eye dict to a row list (passthrough)."""
    return [
        row.get("device_timestamp", ""),
        row.get("pc_arrival_timestamp", ""),
        row.get("gaze_x", ""), row.get("gaze_y", ""), row.get("gaze_z", ""),
        row.get("left_pupil_x", ""), row.get("left_pupil_y", ""),
        row.get("right_pupil_x", ""), row.get("right_pupil_y", ""),
        row.get("left_pupil_diameter_mm", ""), row.get("right_pupil_diameter_mm", ""),
        row.get("left_openness", ""), row.get("right_openness", ""),
        row.get("left_blink", ""), row.get("right_blink", ""),
        row.get("gyro_timestamp", ""),
        row.get("accel_x", ""), row.get("accel_y", ""), row.get("accel_z", ""),
        row.get("gyro_x", ""), row.get("gyro_y", ""), row.get("gyro_z", ""),
        row.get("mag_x", ""), row.get("mag_y", ""), row.get("mag_z", ""),
    ]


def _mic_csv_row_from_loaded(row: dict) -> list[str]:
    """Convert a CSV-loaded mic dict to a row list (passthrough)."""
    return [
        row.get("block_start_pc_timestamp", ""),
        row.get("callback_pc_timestamp", ""),
        row.get("sample_index", ""),
        row.get("frames", ""),
        row.get("rms", ""),
        row.get("dbfs", ""),
        row.get("peak_abs", ""),
        row.get("status_flags", ""),
        row.get("clock_source", ""),
    ]


def _brightness_csv_row_from_loaded(row: dict) -> list[str]:
    """Convert a CSV-loaded brightness dict to a row list (passthrough)."""
    return [
        row.get("eye", ""),
        row.get("eye_name", ""),
        row.get("eye_device_timestamp", ""),
        row.get("eye_pc_arrival_timestamp", ""),
        row.get("width", ""),
        row.get("height", ""),
        row.get("mean_brightness", ""),
        row.get("max_brightness", ""),
        row.get("p99_brightness", ""),
    ]


def save_accepted_window(
    window_data: dict,
    selection: dict,
    parent_output_dir: Path,
    window_index: int,
    imu_signal: str,
):
    """Save the accepted window's data and sync summary into a subfolder."""
    window_dir = parent_output_dir / f"window_{window_index:04d}"
    window_dir.mkdir(parents=True, exist_ok=True)

    # Write trimmed CSVs --------------------------------------------------
    _write_csv(window_dir / "sensor_watch.csv", SENSOR_COLUMNS,
               [_sensor_csv_row_from_loaded(r) for r in window_data["watch_slice"]])
    _write_csv(window_dir / "eye_brightness.csv", BRIGHTNESS_COLUMNS,
               [_brightness_csv_row_from_loaded(r) for r in window_data["bright_slice"]])
    _write_csv(window_dir / "mic_loudness.csv", MIC_COLUMNS,
               [_mic_csv_row_from_loaded(r) for r in window_data["mic_slice"]])
    _write_csv(window_dir / "vr_eye_samples.csv", EYE_COLUMNS,
               [_eye_csv_row_from_loaded(r) for r in window_data["eye_slice"]])

    # Plot -----------------------------------------------------------------
    plot_path = window_dir / "sync_imu_eye_flash.png"
    _save_plot_for_window(window_data, selection, plot_path, window_index)

    # Summary JSON ---------------------------------------------------------
    imu_onset = selection["imu_onset"]
    eye_onset = selection["eye_onset"]
    mic_onset = selection["mic_onset"]
    ppg_onset = selection["ppg_onset"]
    base_time = window_data["imu_times"][0] if window_data["imu_times"].size else 0.0

    delta_ms = (eye_onset["time"] - imu_onset["time"]) * 1000.0
    mic_delta_ms = (mic_onset["time"] - imu_onset["time"]) * 1000.0
    ppg_delta_ms = (ppg_onset["time"] - imu_onset["time"]) * 1000.0

    result = {
        "window_index": window_index,
        "output_dir": str(window_dir),
        "imu_source": "sensor_watch.csv",
        "imu_signal": imu_signal,
        "onset_search": "first_dynamic_threshold_crossing_within_peak_plus_minus_samples",
        "onset_peak_window_samples": ONSET_PEAK_WINDOW_SAMPLES,
        "imu_dynamic_threshold_strength": imu_onset["threshold"],
        "imu_onset_pc_time": imu_onset["time"],
        "imu_onset_time_since_window_start_seconds": imu_onset["time"] - base_time,
        "imu_onset_value": imu_onset["value"],
        "imu_onset_strength": imu_onset["strength"],
        "imu_event_peak_strength": imu_onset["peak_strength"],
        "eye_onset_eye": eye_onset.get("eye", 1),
        "eye_onset_pc_time": eye_onset["time"],
        "eye_onset_time_since_window_start_seconds": eye_onset["time"] - base_time,
        "eye_onset_value": eye_onset["value"],
        "eye_onset_strength": eye_onset["strength"],
        "eye_event_peak_strength": eye_onset["peak_strength"],
        "eye_dynamic_threshold_strength": eye_onset["threshold"],
        "mic_onset_pc_time": mic_onset["time"],
        "mic_onset_time_since_window_start_seconds": mic_onset["time"] - base_time,
        "mic_onset_value": mic_onset["value"],
        "mic_onset_strength": mic_onset["strength"],
        "mic_event_peak_strength": mic_onset["peak_strength"],
        "mic_dynamic_threshold_strength": mic_onset["threshold"],
        "head_ppg_onset_pc_time": ppg_onset["time"],
        "head_ppg_onset_time_since_window_start_seconds": ppg_onset["time"] - base_time,
        "head_ppg_onset_value": ppg_onset["value"],
        "head_ppg_onset_strength": ppg_onset["strength"],
        "head_ppg_event_peak_strength": ppg_onset["peak_strength"],
        "head_ppg_dynamic_threshold_strength": ppg_onset["threshold"],
        "manual_adjusted": selection["manual_adjusted"],
        "eye_minus_imu_ms": delta_ms,
        "mic_minus_imu_ms": mic_delta_ms,
        "head_ppg_minus_imu_ms": ppg_delta_ms,
        "channel_offsets_ms": {
            "head_imu": 0.0,
            "eye_brightness": delta_ms,
            "microphone": mic_delta_ms,
            "head_ppg": ppg_delta_ms,
        },
        "plot": str(plot_path),
        "window_t_min": window_data["window"]["t_min"],
        "window_t_max": window_data["window"]["t_max"],
    }

    with (window_dir / "sync_summary.json").open("w", encoding="utf-8") as fp:
        json.dump(result, fp, indent=2)
    with (window_dir / "sync_report.txt").open("w", encoding="utf-8") as fp:
        fp.write(format_window_report(result))

    return result


def _write_csv(path: Path, columns: list[str], rows: list[list[str]]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.writer(fp)
        writer.writerow(columns)
        for row in rows:
            writer.writerow(row)


def _save_plot_for_window(
    window_data: dict,
    selection: dict,
    path: Path,
    window_index: int,
):
    """Generate and save the sync plot for an accepted window (non-interactive)."""
    import matplotlib.pyplot as plt

    wd = window_data
    imu_times = wd["imu_times"]
    accel_norm = wd["accel_norm"]
    gyro_norm = wd["gyro_norm"]
    eye_traces = wd["eye_traces"]
    mic_times = wd["mic_times"]
    mic_rms = wd["mic_rms"]
    ppg_red = wd["ppg_red"]
    ppg_ir = wd["ppg_ir"]
    ppg_green = wd["ppg_green"]

    imu_onset = selection["imu_onset"]
    eye_onset = selection["eye_onset"]
    mic_onset = selection["mic_onset"]
    ppg_onset = selection["ppg_onset"]

    base_time = imu_times[0] if imu_times.size else 0.0

    fig, axes = plt.subplots(4, 1, figsize=(14, 9), sharex=True)
    x_imu = imu_times - base_time if imu_times.size else np.array([])

    axes[0].plot(x_imu, accel_norm, label="Accel norm", color="tab:blue", lw=1.0)
    axes[0].plot(x_imu, gyro_norm, label="Gyro norm", color="tab:orange", lw=1.0)
    axes[0].axvline(imu_onset["time"] - base_time, color="tab:red", lw=1.4, label="IMU onset")
    axes[0].set_ylabel("IMU norm")
    axes[0].set_title(f"Window {window_index}  —  Head IMU")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.25)

    colors = {1: "tab:green", 2: "black"}
    for eye, trace in sorted(eye_traces.items()):
        label = "Left brightness" if eye == 1 else "Right brightness"
        axes[1].plot(trace["times"] - base_time, trace["values"],
                     color=colors.get(eye, "gray"), lw=1.0, label=label)
    axes[1].axvline(eye_onset["time"] - base_time, color="tab:purple", lw=1.4, label="Eye onset")
    axes[1].set_ylabel("Brightness")
    axes[1].set_title("Eye image mean brightness")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.25)

    x_mic = mic_times - base_time if mic_times.size else np.array([])
    axes[2].plot(x_mic, mic_rms, label="Mic RMS", color="tab:brown", lw=1.0)
    axes[2].axvline(mic_onset["time"] - base_time, color="tab:brown", lw=1.4, label="Mic onset")
    axes[2].set_ylabel("RMS")
    axes[2].set_title("Microphone loudness")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.25)

    if ppg_red.size:
        axes[3].plot(x_imu, ppg_red, label="Red", color="tab:red", lw=0.9)
    if ppg_ir.size:
        axes[3].plot(x_imu, ppg_ir, label="IR", color="tab:orange", lw=0.9)
    if ppg_green.size:
        axes[3].plot(x_imu, ppg_green, label="Green", color="tab:green", lw=0.9)
    axes[3].axvline(ppg_onset["time"] - base_time, color="tab:cyan", lw=1.4, label="Head PPG onset")
    axes[3].set_xlabel("Time since window start (s)")
    axes[3].set_ylabel("PPG")
    axes[3].set_title("Head PPG from watch COM sensor")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def format_window_report(result: dict) -> str:
    eye_name = "left" if result["eye_onset_eye"] == 1 else "right"
    return "\n".join([
        f"Window {result['window_index']}  —  Head IMU / eye-flash sync",
        f"Output: {result['output_dir']}",
        f"IMU onset: {result['imu_onset_time_since_window_start_seconds']:.6f} s "
        f"(strength {result['imu_onset_strength']:.3f}x, "
        f"dynamic threshold {result['imu_dynamic_threshold_strength']:.3f}x)",
        f"Eye onset: {result['eye_onset_time_since_window_start_seconds']:.6f} s "
        f"({eye_name}, strength {result['eye_onset_strength']:.3f}x, "
        f"dynamic threshold {result['eye_dynamic_threshold_strength']:.3f}x)",
        f"Mic onset: {result['mic_onset_time_since_window_start_seconds']:.6f} s "
        f"(strength {result['mic_onset_strength']:.3f}x, "
        f"dynamic threshold {result['mic_dynamic_threshold_strength']:.3f}x)",
        f"Head PPG onset: {result['head_ppg_onset_time_since_window_start_seconds']:.6f} s "
        f"(strength {result['head_ppg_onset_strength']:.3f}x, "
        f"dynamic threshold {result['head_ppg_dynamic_threshold_strength']:.3f}x)",
        f"Eye minus IMU: {result['eye_minus_imu_ms']:.3f} ms",
        f"Mic minus IMU: {result['mic_minus_imu_ms']:.3f} ms",
        f"Head PPG minus IMU: {result['head_ppg_minus_imu_ms']:.3f} ms",
        f"Manual adjusted: {result['manual_adjusted']}",
        f"Plot: {result['plot']}",
        "",
    ])


# ---------------------------------------------------------------------------
# Pairwise latency matrix summary
# ---------------------------------------------------------------------------

CHANNEL_NAMES = ["Head IMU", "Eye", "Mic", "Head PPG"]
CHANNEL_KEYS = ["head_imu", "eye_brightness", "microphone", "head_ppg"]


def _save_pairwise_latency_matrix(
    offsets_list: list[dict[str, float]],
    output_dir: Path,
):
    """Build and plot the pairwise inter-channel latency matrix.

    For each accepted window we have *offsets* relative to Head IMU
    (``channel_offsets_ms``).  The pairwise delay from channel *A* to
    channel *B* is ``offset_B - offset_A``.  We report the mean and
    standard deviation across all accepted windows.
    """
    n = len(CHANNEL_KEYS)
    offsets_arr = np.array([[d[k] for k in CHANNEL_KEYS] for d in offsets_list], dtype=np.float64)

    # Pairwise: delay(i→j) = offset_j - offset_i
    mean_matrix = np.zeros((n, n), dtype=np.float64)
    std_matrix = np.zeros((n, n), dtype=np.float64)
    for i in range(n):
        for j in range(n):
            deltas = offsets_arr[:, j] - offsets_arr[:, i]
            mean_matrix[i, j] = float(np.mean(deltas))
            std_matrix[i, j] = float(np.std(deltas, ddof=1)) if deltas.size > 1 else 0.0

    # --- Plot -----------------------------------------------------------
    import matplotlib.pyplot as plt

    was_interactive = plt.isinteractive()
    plt.ioff()  # prevent window flash — we only save to file here

    fig, ax = plt.subplots(figsize=(7, 6))
    im = ax.imshow(mean_matrix, cmap="RdBu_r", aspect="equal",
                   vmin=-abs(mean_matrix).max(), vmax=abs(mean_matrix).max())

    # Annotate cells with  mean ± std
    for i in range(n):
        for j in range(n):
            if i == j:
                text = "0"
            else:
                text = f"{mean_matrix[i, j]:+.2f}\n±{std_matrix[i, j]:.2f}"
            ax.text(j, i, text, ha="center", va="center", fontsize=10,
                    color="black" if abs(mean_matrix[i, j]) < abs(mean_matrix).max() * 0.5 else "white")

    ax.set_xticks(range(n))
    ax.set_yticks(range(n))
    ax.set_xticklabels(CHANNEL_NAMES, rotation=30, ha="right")
    ax.set_yticklabels(CHANNEL_NAMES)
    ax.set_title(f"Pairwise channel latency (ms)  —  mean ± std  "
                 f"(n = {len(offsets_list)} accepted windows)")
    fig.colorbar(im, ax=ax, shrink=0.82, label="mean delay (ms)")

    fig.tight_layout()
    plot_path = output_dir / "pairwise_latency_matrix.png"
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    if was_interactive:
        plt.ion()

    # --- Text report ----------------------------------------------------
    lines = ["Pairwise inter-channel latency summary (ms)",
             f"Accepted windows: {len(offsets_list)}",
             ""]
    for i in range(n):
        for j in range(i + 1, n):
            lines.append(
                f"  {CHANNEL_NAMES[i]:>10}  →  {CHANNEL_NAMES[j]:<10}  "
                f"{mean_matrix[i, j]:+8.3f}  ± {std_matrix[i, j]:.3f}")
    lines.append("")
    report_text = "\n".join(lines)
    print(report_text)

    report_path = output_dir / "pairwise_latency_summary.txt"
    report_path.write_text(report_text, encoding="utf-8")
    print(f"Pairwise latency matrix saved to: {plot_path}")


# ===================================================================
# Top-level runner
# ===================================================================

def ensure_matplotlib_available():
    try:
        import matplotlib.pyplot  # noqa: F401
    except Exception as exc:
        raise RuntimeError(
            "matplotlib with an interactive window backend is required. "
            "Install matplotlib in the Python environment used to run the script."
        ) from exc


def parse_args():
    parser = argparse.ArgumentParser(
        description="120 s capture → auto-detect sync windows → interactive review → save accepted windows.")
    parser.add_argument("--vr-sdk-root", default=str(resolve_vr_sdk_root()),
                        help=f"VR vendor SDK root directory. Defaults to the value of "
                             f"{VR_SDK_ROOT_ENV_VAR} or {resolve_vr_sdk_root()}.")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_SECONDS,
                        help=f"Capture duration in seconds (default: {DEFAULT_DURATION_SECONDS:.0f}).")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT),
                        help="Root for batch sync-test output. Each run creates a timestamped child directory.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--audio-device", type=int, default=None,
                        help="Microphone input device index.")
    parser.add_argument("--audio-sample-rate", type=int, default=None,
                        help="Microphone sample rate.")
    parser.add_argument("--ignore-initial", type=float, default=0.25,
                        help="Seconds from first sample ignored during onset search.")
    parser.add_argument("--imu-signal", choices=["combined", "accel", "gyro"], default="combined",
                        help="IMU trace used to detect the impact onset.")
    parser.add_argument("--window-half-width", type=float, default=WINDOW_HALF_WIDTH_SECONDS,
                        help=f"Half-width of each extracted window in seconds (default: {WINDOW_HALF_WIDTH_SECONDS}).")
    parser.add_argument("--min-separation", type=float, default=MIN_EVENT_SEPARATION_SECONDS,
                        help=f"Minimum time between consecutive sync events (default: {MIN_EVENT_SEPARATION_SECONDS}).")
    parser.add_argument("--show-sdk-output", action="store_true",
                        help="Do not suppress native VR SDK stdout during capture.")
    return parser.parse_args()


def main():
    args = parse_args()

    # Allow CLI override of these globals before extraction
    global WINDOW_HALF_WIDTH_SECONDS, MIN_EVENT_SEPARATION_SECONDS
    WINDOW_HALF_WIDTH_SECONDS = args.window_half_width
    MIN_EVENT_SEPARATION_SECONDS = args.min_separation

    try:
        ensure_matplotlib_available()
    except Exception as exc:
        print(f"Batch sync test failed: {exc}", file=sys.stderr)
        sys.exit(1)

    # ---- Phase 1: Capture ------------------------------------------------
    session = CaptureSession(args)

    def handle_sigint(_signum, _frame):
        session.request_stop()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        metadata = session.run()
        print(f"\nFull capture saved to: {metadata['output_dir']}")
    except KeyboardInterrupt:
        session.request_stop()
        print("\nInterrupted during capture.", file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print(f"Capture failed: {exc}", file=sys.stderr)
        sys.exit(1)

    capture_dir = Path(metadata["output_dir"])

    # ---- Phase 2: Extract windows ---------------------------------------
    print("Detecting sync windows …")
    try:
        windows, full_data = extract_windows_from_full_capture(
            capture_dir,
            ignore_initial=args.ignore_initial,
            imu_signal=args.imu_signal,
        )
    except Exception as exc:
        print(f"Window extraction failed: {exc}", file=sys.stderr)
        sys.exit(1)

    print(f"Found {len(windows)} candidate windows (≥{MIN_EVENT_SEPARATION_SECONDS:.0f} s apart).")

    if not windows:
        print("No windows to review.", file=sys.stderr)
        sys.exit(0)

    # ---- Phase 3: Interactive review ------------------------------------
    reviewer = _ReviewFigure()
    accepted_count = 0
    rejected_count = 0
    quit_early = False
    accepted_offsets: list[dict[str, float]] = []  # collect per-window channel_offsets_ms

    try:
        for idx, window in enumerate(windows):
            if quit_early:
                break

            print(f"\n--- Window {idx + 1}/{len(windows)} "
                  f"(IMU anchor @ {window['imu_anchor']['time'] - full_data['imu_times'][0]:.3f} s) ---")
            try:
                window_data = trim_data_to_window(window, full_data, args.ignore_initial)
            except Exception as exc:
                print(f"  Skipping window {idx + 1}: trim error ({exc})")
                rejected_count += 1
                continue

            decision, selection = reviewer.review(window_data, idx, len(windows))

            if decision == "quit":
                print(f"  User quit batch. {accepted_count} accepted, {rejected_count} rejected so far.")
                quit_early = True
                break
            elif decision == "accept":
                try:
                    result = save_accepted_window(window_data, selection, capture_dir,
                                                  accepted_count, args.imu_signal)
                except Exception as exc:
                    print(f"  Save failed for window {idx + 1}: {exc}")
                    rejected_count += 1
                    continue
                accepted_offsets.append(result["channel_offsets_ms"])
                delta = (selection["eye_onset"]["time"] - selection["imu_onset"]["time"]) * 1000.0
                adj = " (manual)" if selection.get("manual_adjusted") else ""
                print(f"  ✓ Accepted{adj}. eye-imu delta = {delta:.3f} ms")
                accepted_count += 1
            else:
                print(f"  ✗ Rejected.")
                rejected_count += 1
    finally:
        reviewer.close()

    # ---- Phase 4: Batch summary -----------------------------------------
    print(f"\n{'='*60}")
    print(f"Batch complete: {accepted_count} accepted, {rejected_count} rejected, "
          f"{len(windows) - accepted_count - rejected_count} skipped (quit early).")
    print(f"Output root: {capture_dir}")

    if accepted_offsets:
        _save_pairwise_latency_matrix(accepted_offsets, capture_dir)
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
