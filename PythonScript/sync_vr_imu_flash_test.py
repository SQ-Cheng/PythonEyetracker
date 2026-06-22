#!/usr/bin/env python
"""
VR IMU / eye-flash synchronization test.

This terminal script reuses the VR visual logger data path:
  - The CP210x serial device that identifies itself as "watch" provides Head IMU and Head PPG.
  - VrUserSdkWrapper.data_queue provides VR gaze/pupil samples.
  - VrUserSdkWrapper.image_queue provides eye images used for brightness traces.
  - A microphone input stream provides loudness/RMS events.
  - CP210x ring/watch serial sensors are logged with the visual logger packet
    format; the "watch" sensor is required for Head IMU analysis.

The capture phase logs raw streams for a fixed duration. The analysis phase
opens a matplotlib window, saves the plotted traces, detects the first large
upward-jump sample in each stream, and reports eye/mic/Head-PPG timing relative
to Head IMU on the shared PC perf-counter timeline.
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
from dataclasses import dataclass
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


PACKET_SIZE = 64
NUM_FLOATS = 14
SYNC_MARKER = bytes([0x55, 0xAA, 0x55, 0xAA])
SYNC_LEN = len(SYNC_MARKER)
TARGET_SERIAL_DESCRIPTION = "Silicon Labs CP210x USB to UART Bridge"
SENSOR_IDENTITY_TIMEOUT_SECONDS = 2.0
SENSOR_ROLES = ("ring", "watch")

DEFAULT_DURATION_SECONDS = 10.0
DEFAULT_BAUD = 1_000_000
DEFAULT_AUDIO_SAMPLE_RATE = 48_000
DEFAULT_OUTPUT_ROOT = LOG_DIR / "sync_vr_imu_flash"
PEAK_REFRACTORY_SECONDS = 0.35
PAIR_MAX_GAP_SECONDS = 0.75
ONSET_PEAK_WINDOW_SAMPLES = 10
AUDIO_CHANNELS = 1
AUDIO_DTYPE = "int16"
AUDIO_BLOCK_SECONDS = 0.010
AUDIO_PROBE_SECONDS = 0.60
AUDIO_MIN_VALID_PEAK_ABS = 2.0


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

MIC_COLUMNS = [
    "block_start_pc_timestamp",
    "callback_pc_timestamp",
    "sample_index",
    "frames",
    "rms",
    "dbfs",
    "peak_abs",
    "status_flags",
    "clock_source",
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
        clock_source = "callback_pc_minus_block_duration"

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

        self.queue.put(
            {
                "block_start_pc_timestamp": float(block_start),
                "callback_pc_timestamp": float(callback_pc),
                "sample_index": int(sample_index),
                "frames": int(frames),
                "rms": rms,
                "dbfs": dbfs,
                "peak_abs": peak_abs,
                "status_flags": status_flags,
                "clock_source": clock_source,
            }
        )

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
                "mapper" in lowered
                or "映射器" in lowered
                or "主声音捕获" in lowered
                or "loopback" in lowered
                or "speaker" in lowered
                or "扬声器" in lowered
                or "电脑扬声器" in lowered
                or "stereo mix" in lowered
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
            devices.append(
                {
                    "index": index,
                    "name": name,
                    "hostapi": hostapi_index,
                    "hostapi_name": hostapi_name,
                    "default_samplerate": int(round(float(device.get("default_samplerate", 0) or 0))),
                    "is_default": default_input == index,
                }
            )
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
            "callbacks": int(callbacks),
            "frames": int(frames_total),
            "pc_span_seconds": 0.0,
            "audio_seconds": 0.0,
            "max_rms": float(max_rms),
            "max_peak_abs": float(max_peak_abs),
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
        "ok": bool(ok),
        "callbacks": int(callbacks),
        "frames": int(frames_total),
        "pc_span_seconds": pc_span,
        "audio_seconds": audio_seconds,
        "max_rms": float(max_rms),
        "max_peak_abs": float(max_peak_abs),
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
            "device": int(explicit_device),
            "name": str(info.get("name", "")),
            "hostapi": hostapi_index,
            "hostapi_name": hostapi_name,
            "sample_rate": sample_rate,
            "selection": "explicit",
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
        probe_results.append(
            {
                "device": int(candidate["index"]),
                "name": candidate["name"],
                "hostapi_name": candidate["hostapi_name"],
                "sample_rate": sample_rate,
                **probe,
            }
        )
        if probe["ok"]:
            chosen = candidate
            chosen_probe = probe
            break

    if chosen is None:
        details = "; ".join(
            (
                f"{item['device']} {item['name']} ({item['hostapi_name']}): "
                f"callbacks={item['callbacks']}, span={item['pc_span_seconds']:.3f}s, "
                f"peak={item['max_peak_abs']:.1f}"
            )
            for item in probe_results
        )
        raise RuntimeError(f"No usable microphone input produced a live nonzero signal. Probe results: {details}")

    sample_rate = int(explicit_sample_rate or chosen["default_samplerate"] or DEFAULT_AUDIO_SAMPLE_RATE)
    selection = "auto_vive_probe" if "vive" in str(chosen.get("name", "")).casefold() else "auto_probe_fallback"
    return int(chosen["index"]), sample_rate, {
        "device": int(chosen["index"]),
        "name": chosen["name"],
        "hostapi": chosen["hostapi"],
        "hostapi_name": chosen["hostapi_name"],
        "sample_rate": sample_rate,
        "selection": selection,
        "probe": chosen_probe,
        "probe_results": probe_results,
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


def mic_csv_row(sample: dict) -> list[str]:
    return [
        f"{sample['block_start_pc_timestamp']:.9f}",
        f"{sample['callback_pc_timestamp']:.9f}",
        str(int(sample["sample_index"])),
        str(int(sample["frames"])),
        f"{sample['rms']:.6f}",
        f"{sample['dbfs']:.6f}",
        f"{sample['peak_abs']:.6f}",
        sample["status_flags"],
        sample["clock_source"],
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
    times: np.ndarray,
    values: np.ndarray,
    strength: np.ndarray,
    threshold: float,
    ignore_initial: float,
    refractory_seconds: float = PEAK_REFRACTORY_SECONDS,
    onset_window_samples: int = ONSET_PEAK_WINDOW_SAMPLES,
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
        if local_candidates:
            onset_idx = min(local_candidates)
        else:
            onset_idx = peak_idx
        events.append(
            {
                "index": int(onset_idx),
                "time": float(times[onset_idx]),
                "value": float(values[onset_idx]),
                "strength": float(strength[onset_idx]),
                "peak_index": int(peak_idx),
                "peak_time": float(times[peak_idx]),
                "peak_value": float(values[peak_idx]),
                "peak_strength": float(strength[peak_idx]),
                "threshold": float(threshold),
                "above_threshold": True,
            }
        )
    return events


def match_upward_jump_events(imu_events: list[dict], eye_events: list[dict], max_gap_seconds: float = PAIR_MAX_GAP_SECONDS):
    matches = []
    for eye_event in eye_events:
        candidates = [
            imu_event
            for imu_event in imu_events
            if abs(eye_event["time"] - imu_event["time"]) <= max_gap_seconds
        ]
        if not candidates:
            continue
        imu_event = min(candidates, key=lambda event: abs(eye_event["time"] - event["time"]))
        matches.append(
            {
                "imu": imu_event,
                "eye": eye_event,
                "gap_seconds": abs(eye_event["time"] - imu_event["time"]),
                "score": min(imu_event["peak_strength"], eye_event["peak_strength"]),
            }
        )
    return matches


def nearest_event_within(events: list[dict], timestamp: float, max_gap_seconds: float = PAIR_MAX_GAP_SECONDS):
    candidates = [
        event
        for event in events
        if abs(event["time"] - timestamp) <= max_gap_seconds
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda event: abs(event["time"] - timestamp))


def load_csv_dicts(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open("r", newline="", encoding="utf-8") as fp:
        return list(csv.DictReader(fp))


def analyze_capture(output_dir: Path, ignore_initial: float, imu_signal: str) -> dict:
    watch_rows = load_csv_dicts(output_dir / "sensor_watch.csv")
    brightness_rows = load_csv_dicts(output_dir / "eye_brightness.csv")
    mic_rows = load_csv_dicts(output_dir / "mic_loudness.csv")
    if not watch_rows:
        raise RuntimeError("No Head IMU rows captured. Expected sensor_watch.csv from the COM device identifying as 'watch'.")
    if not brightness_rows:
        raise RuntimeError("No eye brightness rows captured.")
    if not mic_rows:
        raise RuntimeError("No microphone loudness rows captured. Expected mic_loudness.csv.")

    imu_times = np.array([float(row["pc_arrival_timestamp"]) for row in watch_rows], dtype=np.float64)
    accel = np.array(
        [
            [float(row["accX"]), float(row["accY"]), float(row["accZ"])]
            for row in watch_rows
        ],
        dtype=np.float64,
    )
    gyro = np.array(
        [
            [float(row["gyrX"]), float(row["gyrY"]), float(row["gyrZ"])]
            for row in watch_rows
        ],
        dtype=np.float64,
    )
    accel_norm = np.linalg.norm(accel, axis=1)
    gyro_norm = np.linalg.norm(gyro, axis=1)
    ppg_red = np.array([float(row["Red"]) for row in watch_rows], dtype=np.float64)
    ppg_ir = np.array([float(row["IR"]) for row in watch_rows], dtype=np.float64)
    ppg_green = np.array([float(row["Green"]) for row in watch_rows], dtype=np.float64)

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
    imu_threshold = dynamic_strength_threshold(imu_times, imu_strength, ignore_initial)

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
    ppg_threshold = dynamic_strength_threshold(imu_times, ppg_strength, ignore_initial)
    ppg_events = detect_upward_jump_events(imu_times, ppg_values, ppg_strength, ppg_threshold, ignore_initial)
    if not ppg_events:
        raise RuntimeError("Could not detect a Head PPG upward-jump onset.")

    mic_times = np.array([float(row["block_start_pc_timestamp"]) for row in mic_rows], dtype=np.float64)
    mic_rms = np.array([float(row["rms"]) for row in mic_rows], dtype=np.float64)
    mic_span = float(mic_times[-1] - mic_times[0]) if mic_times.size >= 2 else 0.0
    if mic_times.size < 5 or mic_span < 1.0:
        raise RuntimeError(
            f"Microphone capture is too short for sync analysis: rows={mic_times.size}, "
            f"span={mic_span:.3f}s. The mic stream likely stopped before/during capture."
        )
    mic_analysis = robust_strength(mic_times, mic_rms, baseline_seconds=0.20)
    mic_threshold = dynamic_strength_threshold(mic_times, mic_analysis["strength"], ignore_initial)
    mic_events = detect_upward_jump_events(mic_times, mic_rms, mic_analysis["strength"], mic_threshold, ignore_initial)
    if not mic_events:
        raise RuntimeError("Could not detect a microphone upward-jump onset.")

    brightness_by_eye: dict[int, list[dict]] = {1: [], 2: []}
    for row in brightness_rows:
        try:
            brightness_by_eye[int(row["eye"])].append(row)
        except Exception:
            continue

    eye_events = []
    eye_traces = {}
    for eye, rows in brightness_by_eye.items():
        if not rows:
            continue
        times = np.array([float(row["eye_pc_arrival_timestamp"]) for row in rows], dtype=np.float64)
        values = np.array([float(row["mean_brightness"]) for row in rows], dtype=np.float64)
        analysis = robust_strength(times, values, baseline_seconds=0.25)
        threshold = dynamic_strength_threshold(times, analysis["strength"], ignore_initial)
        events = detect_upward_jump_events(times, values, analysis["strength"], threshold, ignore_initial)
        eye_traces[eye] = {
            "times": times,
            "values": values,
            "analysis": analysis,
            "threshold": threshold,
            "events": events,
        }
        for event in events:
            event_with_eye = dict(event)
            event_with_eye["eye"] = eye
            eye_events.append(event_with_eye)

    imu_events = detect_upward_jump_events(imu_times, imu_values, imu_strength, imu_threshold, ignore_initial)
    if not imu_events:
        raise RuntimeError("Could not detect an IMU upward-jump onset.")
    if not eye_events:
        raise RuntimeError("Could not detect an eye brightness upward-jump onset.")

    four_channel_matches = []
    for imu_event in imu_events:
        eye_event = nearest_event_within(eye_events, imu_event["time"])
        mic_event = nearest_event_within(mic_events, imu_event["time"])
        ppg_event = nearest_event_within(ppg_events, imu_event["time"])
        if not (eye_event and mic_event and ppg_event):
            continue
        gaps = [
            abs(eye_event["time"] - imu_event["time"]),
            abs(mic_event["time"] - imu_event["time"]),
            abs(ppg_event["time"] - imu_event["time"]),
        ]
        four_channel_matches.append(
            {
                "imu": imu_event,
                "eye": eye_event,
                "mic": mic_event,
                "ppg": ppg_event,
                "max_gap_seconds": max(gaps),
                "score": min(
                    imu_event["peak_strength"],
                    eye_event["peak_strength"],
                    mic_event["peak_strength"],
                    ppg_event["peak_strength"],
                ),
            }
        )

    if not four_channel_matches:
        raise RuntimeError(
            "No four-channel IMU/eye/mic/Head-PPG upward-jump onset was found within "
            f"{PAIR_MAX_GAP_SECONDS:.3f}s. Discard this capture or trigger all stimuli closer together."
        )

    selected_match = max(four_channel_matches, key=lambda match: (match["score"], -match["max_gap_seconds"]))
    imu_onset = selected_match["imu"]
    eye_onset = selected_match["eye"]
    mic_onset = selected_match["mic"]
    ppg_onset = selected_match["ppg"]
    base_time = min(
        float(imu_times[0]),
        float(mic_times[0]),
        min(float(trace["times"][0]) for trace in eye_traces.values()),
    )

    plot_path = output_dir / "sync_imu_eye_flash.png"
    plot_selection = draw_analysis_plot(
        plot_path,
        base_time,
        imu_times,
        imu_values,
        accel_norm,
        gyro_norm,
        imu_strength,
        imu_onset,
        eye_traces,
        eye_onset,
        mic_times,
        mic_rms,
        mic_analysis["strength"],
        mic_threshold,
        mic_onset,
        ppg_red,
        ppg_ir,
        ppg_green,
        ppg_values,
        ppg_strength,
        ppg_threshold,
        ppg_onset,
        imu_threshold,
        imu_label,
    )
    imu_onset = plot_selection["imu_onset"]
    eye_onset = plot_selection["eye_onset"]
    mic_onset = plot_selection["mic_onset"]
    ppg_onset = plot_selection["ppg_onset"]
    delta_ms = (eye_onset["time"] - imu_onset["time"]) * 1000.0
    mic_delta_ms = (mic_onset["time"] - imu_onset["time"]) * 1000.0
    ppg_delta_ms = (ppg_onset["time"] - imu_onset["time"]) * 1000.0

    result = {
        "output_dir": str(output_dir),
        "imu_source": "sensor_watch.csv",
        "time_basis": "first_threshold_crossing_after_large_upward_jump",
        "imu_signal": imu_signal,
        "threshold_generation": "largest_log_strength_gap_per_trace",
        "onset_search": "first_dynamic_threshold_crossing_within_peak_plus_minus_samples",
        "onset_peak_window_samples": ONSET_PEAK_WINDOW_SAMPLES,
        "imu_dynamic_threshold_strength": imu_threshold,
        "ignore_initial_seconds": ignore_initial,
        "imu_onset_pc_time": imu_onset["time"],
        "imu_onset_time_since_start_seconds": imu_onset["time"] - base_time,
        "imu_onset_value": imu_onset["value"],
        "imu_onset_strength": imu_onset["strength"],
        "imu_event_peak_strength": imu_onset["peak_strength"],
        "imu_onset_above_threshold": imu_onset["above_threshold"],
        "imu_events_detected": len(imu_events),
        "eye_onset_eye": eye_onset["eye"],
        "eye_onset_pc_time": eye_onset["time"],
        "eye_onset_time_since_start_seconds": eye_onset["time"] - base_time,
        "eye_onset_value": eye_onset["value"],
        "eye_onset_strength": eye_onset["strength"],
        "eye_event_peak_strength": eye_onset["peak_strength"],
        "eye_onset_above_threshold": eye_onset["above_threshold"],
        "eye_dynamic_threshold_strength": eye_onset["threshold"],
        "eye_events_detected": len(eye_events),
        "mic_onset_pc_time": mic_onset["time"],
        "mic_onset_time_since_start_seconds": mic_onset["time"] - base_time,
        "mic_onset_value": mic_onset["value"],
        "mic_onset_strength": mic_onset["strength"],
        "mic_event_peak_strength": mic_onset["peak_strength"],
        "mic_dynamic_threshold_strength": mic_onset["threshold"],
        "mic_events_detected": len(mic_events),
        "head_ppg_onset_pc_time": ppg_onset["time"],
        "head_ppg_onset_time_since_start_seconds": ppg_onset["time"] - base_time,
        "head_ppg_onset_value": ppg_onset["value"],
        "head_ppg_onset_strength": ppg_onset["strength"],
        "head_ppg_event_peak_strength": ppg_onset["peak_strength"],
        "head_ppg_dynamic_threshold_strength": ppg_onset["threshold"],
        "head_ppg_events_detected": len(ppg_events),
        "matched_events": len(four_channel_matches),
        "pair_max_gap_seconds": PAIR_MAX_GAP_SECONDS,
        "manual_adjusted": plot_selection["manual_adjusted"],
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
    imu_values: np.ndarray,
    accel_norm: np.ndarray,
    gyro_norm: np.ndarray,
    imu_strength: np.ndarray,
    imu_onset: dict,
    eye_traces: dict[int, dict],
    eye_onset: dict,
    mic_times: np.ndarray,
    mic_rms: np.ndarray,
    mic_strength: np.ndarray,
    mic_threshold: float,
    mic_onset: dict,
    ppg_red: np.ndarray,
    ppg_ir: np.ndarray,
    ppg_green: np.ndarray,
    ppg_values: np.ndarray,
    ppg_strength: np.ndarray,
    ppg_threshold: float,
    ppg_onset: dict,
    imu_threshold: float,
    imu_label: str,
):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(5, 1, figsize=(15, 11), sharex=True)
    selection = {
        "imu_onset": dict(imu_onset),
        "eye_onset": dict(eye_onset),
        "mic_onset": dict(mic_onset),
        "ppg_onset": dict(ppg_onset),
        "manual_adjusted": False,
    }

    def delta_ms(channel):
        return (selection[channel]["time"] - selection["imu_onset"]["time"]) * 1000.0

    title = fig.suptitle("", fontsize=13)
    hint = fig.text(
        0.5,
        0.012,
        "Manual adjust: click a signal plot to set that channel's onset; close the window to save.",
        ha="center",
        va="bottom",
        fontsize=9,
        color="#555555",
    )
    del hint

    x_imu = imu_times - base_time
    axes[0].plot(x_imu, accel_norm, label="Accel norm", color="tab:blue", lw=1.0)
    axes[0].plot(x_imu, gyro_norm, label="Gyro norm", color="tab:orange", lw=1.0)
    imu_line = axes[0].axvline(imu_onset["time"] - base_time, color="tab:red", lw=1.4, label="IMU onset")
    axes[0].set_ylabel("IMU norm")
    axes[0].set_title(f"Head IMU from watch COM sensor ({imu_label} used for detection)")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.25)

    colors = {1: "tab:green", 2: "black"}
    for eye, trace in sorted(eye_traces.items()):
        label = "Left brightness" if eye == 1 else "Right brightness"
        axes[1].plot(trace["times"] - base_time, trace["values"], color=colors.get(eye, "gray"), lw=1.0, label=label)
    eye_line = axes[1].axvline(eye_onset["time"] - base_time, color="tab:purple", lw=1.4, label="Eye brightness onset")
    axes[1].set_ylabel("Brightness")
    axes[1].set_title("Eye image mean brightness")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(mic_times - base_time, mic_rms, label="Mic RMS", color="tab:brown", lw=1.0)
    mic_line = axes[2].axvline(mic_onset["time"] - base_time, color="tab:brown", lw=1.4, label="Mic onset")
    axes[2].set_ylabel("RMS")
    axes[2].set_title("Microphone loudness")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.25)

    axes[3].plot(x_imu, ppg_red, label="Red", color="tab:red", lw=0.9)
    axes[3].plot(x_imu, ppg_ir, label="IR", color="tab:orange", lw=0.9)
    axes[3].plot(x_imu, ppg_green, label="Green", color="tab:green", lw=0.9)
    ppg_line = axes[3].axvline(ppg_onset["time"] - base_time, color="tab:cyan", lw=1.4, label="Head PPG onset")
    axes[3].set_ylabel("PPG")
    axes[3].set_title("Head PPG from watch COM sensor")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.25)

    axes[4].plot(x_imu, imu_strength, color="tab:red", lw=1.0, label="IMU strength")
    axes[4].axhline(imu_threshold, color="tab:red", ls="--", lw=1.0, alpha=0.75, label="IMU dynamic threshold")
    axes[4].plot(mic_times - base_time, mic_strength, color="tab:brown", lw=1.0, label="Mic strength")
    axes[4].axhline(mic_threshold, color="tab:brown", ls="--", lw=1.0, alpha=0.75, label="Mic dynamic threshold")
    axes[4].plot(x_imu, ppg_strength, color="tab:cyan", lw=1.0, label="Head PPG strength")
    axes[4].axhline(ppg_threshold, color="tab:cyan", ls="--", lw=1.0, alpha=0.75, label="Head PPG dynamic threshold")
    imu_strength_line = axes[4].axvline(imu_onset["time"] - base_time, color="tab:red", lw=1.0, alpha=0.55)
    eye_strength_line = axes[4].axvline(eye_onset["time"] - base_time, color="tab:purple", lw=1.0, alpha=0.55)
    mic_strength_line = axes[4].axvline(mic_onset["time"] - base_time, color="tab:brown", lw=1.0, alpha=0.55)
    ppg_strength_line = axes[4].axvline(ppg_onset["time"] - base_time, color="tab:cyan", lw=1.0, alpha=0.55)
    for eye, trace in sorted(eye_traces.items()):
        label = "Left brightness strength" if eye == 1 else "Right brightness strength"
        axes[4].plot(
            trace["times"] - base_time,
            trace["analysis"]["strength"],
            color=colors.get(eye, "gray"),
            lw=1.0,
            alpha=0.8,
            label=label,
        )
        threshold_label = "Left dynamic threshold" if eye == 1 else "Right dynamic threshold"
        axes[4].axhline(
            trace["threshold"],
            color=colors.get(eye, "gray"),
            ls=":",
            lw=1.0,
            alpha=0.75,
            label=threshold_label,
        )
    span = axes[4].axvspan(
        min(imu_onset["time"], eye_onset["time"], mic_onset["time"], ppg_onset["time"]) - base_time,
        max(imu_onset["time"], eye_onset["time"], mic_onset["time"], ppg_onset["time"]) - base_time,
        color="tab:purple",
        alpha=0.12,
    )
    axes[4].set_xlabel("Time since first sample (s)")
    axes[4].set_ylabel("Noise x")
    axes[4].set_title("Dynamic onset strength")
    axes[4].legend(loc="upper right", fontsize=8)
    axes[4].grid(True, alpha=0.25)

    def event_from_sample(times, values, strength, index, threshold, eye=None):
        event = {
            "index": int(index),
            "time": float(times[index]),
            "value": float(values[index]),
            "strength": float(strength[index]),
            "peak_index": int(index),
            "peak_time": float(times[index]),
            "peak_value": float(values[index]),
            "peak_strength": float(strength[index]),
            "threshold": float(threshold),
            "above_threshold": bool(strength[index] >= threshold),
            "manual": True,
        }
        if eye is not None:
            event["eye"] = int(eye)
        return event

    def nearest_imu_event(xdata):
        target_time = base_time + float(xdata)
        index = int(np.argmin(np.abs(imu_times - target_time)))
        return event_from_sample(imu_times, imu_values, imu_strength, index, imu_threshold)

    def nearest_eye_event(mouse_event):
        best = None
        for eye, trace in eye_traces.items():
            times = trace["times"]
            if times.size == 0:
                continue
            target_time = base_time + float(mouse_event.xdata)
            center = int(np.argmin(np.abs(times - target_time)))
            lo = max(0, center - 3)
            hi = min(times.size, center + 4)
            for index in range(lo, hi):
                x_display, y_display = axes[1].transData.transform((times[index] - base_time, trace["values"][index]))
                distance = (x_display - mouse_event.x) ** 2 + (y_display - mouse_event.y) ** 2
                if best is None or distance < best[0]:
                    event = event_from_sample(
                        times,
                        trace["values"],
                        trace["analysis"]["strength"],
                        index,
                        trace["threshold"],
                        eye=eye,
                    )
                    best = (distance, event)
        return best[1] if best else None

    def nearest_mic_event(xdata):
        target_time = base_time + float(xdata)
        index = int(np.argmin(np.abs(mic_times - target_time)))
        return event_from_sample(mic_times, mic_rms, mic_strength, index, mic_threshold)

    def nearest_ppg_event(xdata):
        target_time = base_time + float(xdata)
        index = int(np.argmin(np.abs(imu_times - target_time)))
        return event_from_sample(imu_times, ppg_values, ppg_strength, index, ppg_threshold)

    def refresh_selection():
        imu_x = selection["imu_onset"]["time"] - base_time
        eye_x = selection["eye_onset"]["time"] - base_time
        mic_x = selection["mic_onset"]["time"] - base_time
        ppg_x = selection["ppg_onset"]["time"] - base_time
        imu_line.set_xdata([imu_x, imu_x])
        eye_line.set_xdata([eye_x, eye_x])
        mic_line.set_xdata([mic_x, mic_x])
        ppg_line.set_xdata([ppg_x, ppg_x])
        imu_strength_line.set_xdata([imu_x, imu_x])
        eye_strength_line.set_xdata([eye_x, eye_x])
        mic_strength_line.set_xdata([mic_x, mic_x])
        ppg_strength_line.set_xdata([ppg_x, ppg_x])
        nonlocal span
        span.remove()
        span = axes[4].axvspan(min(imu_x, eye_x, mic_x, ppg_x), max(imu_x, eye_x, mic_x, ppg_x), color="tab:purple", alpha=0.12)
        title.set_text(
            "Four-channel sync relative to Head IMU: "
            f"eye={delta_ms('eye_onset'):.3f} ms, "
            f"mic={delta_ms('mic_onset'):.3f} ms, "
            f"ppg={delta_ms('ppg_onset'):.3f} ms"
        )
        fig.canvas.draw_idle()

    def on_click(event):
        if event.xdata is None or event.button != 1:
            return
        if event.inaxes is axes[0]:
            selection["imu_onset"] = nearest_imu_event(event.xdata)
        elif event.inaxes is axes[1]:
            eye_event = nearest_eye_event(event)
            if eye_event is None:
                return
            selection["eye_onset"] = eye_event
        elif event.inaxes is axes[2]:
            selection["mic_onset"] = nearest_mic_event(event.xdata)
        elif event.inaxes is axes[3]:
            selection["ppg_onset"] = nearest_ppg_event(event.xdata)
        else:
            return
        selection["manual_adjusted"] = True
        refresh_selection()

    refresh_selection()
    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show(block=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return selection


def format_report(result: dict) -> str:
    eye_name = "left" if result["eye_onset_eye"] == 1 else "right"
    return "\n".join(
        [
            "Head IMU / eye-flash synchronization result",
            f"Output: {result['output_dir']}",
            f"IMU source: {result['imu_source']}",
            f"Time basis: first dynamic-threshold upward crossing",
            f"Threshold generation: {result['threshold_generation']}",
            f"Onset search: peak +/- {result['onset_peak_window_samples']} samples",
            f"IMU signal: {result['imu_signal']}",
            f"IMU onset: {result['imu_onset_time_since_start_seconds']:.6f} s "
            f"(onset strength {result['imu_onset_strength']:.3f}x, "
            f"dynamic threshold {result['imu_dynamic_threshold_strength']:.3f}x)",
            f"Eye onset: {result['eye_onset_time_since_start_seconds']:.6f} s "
            f"({eye_name}, onset strength {result['eye_onset_strength']:.3f}x, "
            f"dynamic threshold {result['eye_dynamic_threshold_strength']:.3f}x)",
            f"Mic onset: {result['mic_onset_time_since_start_seconds']:.6f} s "
            f"(onset strength {result['mic_onset_strength']:.3f}x, "
            f"dynamic threshold {result['mic_dynamic_threshold_strength']:.3f}x)",
            f"Head PPG onset: {result['head_ppg_onset_time_since_start_seconds']:.6f} s "
            f"(onset strength {result['head_ppg_onset_strength']:.3f}x, "
            f"dynamic threshold {result['head_ppg_dynamic_threshold_strength']:.3f}x)",
            f"Eye minus IMU: {result['eye_minus_imu_ms']:.3f} ms",
            f"Mic minus IMU: {result['mic_minus_imu_ms']:.3f} ms",
            f"Head PPG minus IMU: {result['head_ppg_minus_imu_ms']:.3f} ms",
            f"Manual adjusted: {result['manual_adjusted']}",
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
        device_index, sample_rate, device_info = choose_audio_device(self.args.audio_device, self.args.audio_sample_rate)
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
            f"{device_info.get('selection', '')}{probe_text})."
        )

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
            raise RuntimeError(f"Head IMU sensor not found. Expected COM device identity 'watch', found: {found}.")
        for role, reader in self.sensor_readers.items():
            self.logs[f"sensor_{role}"] = CsvLog(self.output_dir / f"sensor_{role}.csv", SENSOR_COLUMNS)
            if reader.serial_port:
                reader.serial_port.reset_input_buffer()
                reader.serial_port.reset_output_buffer()
            reader.start()
            reader.send_command("s\n")
            status_print(f"{role} sensor started on {reader.port}.")

    def _capture_loop(self):
        start = time.perf_counter()
        next_print = start
        status_print(f"Capturing for {self.args.duration:.1f} seconds. Trigger impact + flash during this window.")
        while not self.stop_requested:
            now = time.perf_counter()
            if now - start >= self.args.duration:
                break
            self._drain_all()
            if now >= next_print:
                next_print = now + 1.0
                status_print(
                    f"\rElapsed {now - start:5.1f}s | "
                    f"Head IMU rows {self.logs.get('sensor_watch').rows if self.logs.get('sensor_watch') else 0} | "
                    f"brightness rows {self.logs['brightness'].rows} | "
                    f"mic rows {self.logs['mic'].rows}",
                    end="",
                    flush=True,
                )
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


def parse_args():
    parser = argparse.ArgumentParser(description="Capture 10s VR Head IMU, eye brightness, microphone, and Head PPG data, then compute sync deltas.")
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
    parser.add_argument("--audio-device", type=int, default=None, help="Microphone input device index. Defaults to system default input.")
    parser.add_argument(
        "--audio-sample-rate",
        type=int,
        default=None,
        help="Microphone sample rate. Defaults to the selected device's default rate.",
    )
    parser.add_argument(
        "--ignore-initial",
        type=float,
        default=0.25,
        help="Seconds from first sample ignored during dynamic onset search to avoid startup transients.",
    )
    parser.add_argument(
        "--imu-signal",
        choices=["combined", "accel", "gyro"],
        default="combined",
        help="IMU trace used to detect the impact onset.",
    )
    parser.add_argument(
        "--show-sdk-output",
        action="store_true",
        help="Do not suppress native VR SDK stdout during capture. Useful only when debugging SDK internals.",
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
