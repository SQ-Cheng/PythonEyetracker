"""Direct aSeeVR USERSDK wrapper for synchronized gaze and eye images.

This wrapper uses the current runtime/USERSDK.dll path directly instead of the
public aSeeVRClient streaming API. The public API is still used briefly to
refresh the 2048-byte calibration coefficient, then Runtime.exe is stopped so
USERSDK.dll can open the eye cameras directly.
"""

from __future__ import annotations

import ctypes as C
import os
import queue
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from sdk_types import py_7i_eye_data_ex_t


DEFAULT_RUNTIME_PIDS = (17178, 17156)
DEFAULT_PORT = 5777
SUCCESS = 0


class UserSdkCoefficientWithSize(C.Structure):
    _fields_ = [
        ("buf", C.c_ubyte * 1024),
        ("size", C.c_int),
    ]


class ASeeVRInitParam(C.Structure):
    _fields_ = [
        ("mode", C.c_int32),
        ("ports", C.c_int16 * 10),
    ]


class ASeeVRCoefficient(C.Structure):
    _fields_ = [("buf", C.c_ubyte * 2048)]


class ASeeVRState(C.Structure):
    _fields_ = [
        ("code", C.c_int32),
        ("error", C.c_int32),
    ]


CoefficientCallback = C.WINFUNCTYPE(None, C.POINTER(ASeeVRCoefficient), C.c_void_p)
StateCallback = C.WINFUNCTYPE(None, C.POINTER(ASeeVRState), C.c_void_p)
ImageCallback = C.WINFUNCTYPE(
    None,
    C.c_int,
    C.POINTER(C.c_uint8),
    C.c_int,
    C.c_int,
    C.c_int,
    C.c_longlong,
    C.c_void_p,
)
GazeCallback = C.WINFUNCTYPE(None, C.POINTER(py_7i_eye_data_ex_t), C.c_void_p)


@dataclass
class EyeImageFrame:
    eye: int
    data: bytes
    width: int
    height: int
    device_timestamp: int
    pc_arrival_timestamp: float


class VrUserSdkWrapper:
    """Low-level VR wrapper with synchronized gaze/pupil and eye images."""

    def __init__(self):
        self._sdk_root = Path()
        self._client_dir = Path()
        self._runtime_dir = Path()
        self._sdk_config = Path()
        self._runtime_proc: subprocess.Popen | None = None
        self._runtime_owned = False
        self._dll_dirs = []
        self._dll: C.WinDLL | None = None
        self._client_dll: C.WinDLL | None = None
        self._coefficients: list[UserSdkCoefficientWithSize] = []
        self._coefficient_blob = bytes()

        self._image_cb = ImageCallback(self._on_image)
        self._gaze_cb = GazeCallback(self._on_gaze)
        self._state_cb = StateCallback(self._on_state)
        self._coefficient_cb = CoefficientCallback(self._on_coefficient)

        self._started_camera = False
        self._started_gaze = False
        self._started_tracking = False
        self._released = True
        self._first_callback_event = threading.Event()
        self._coefficient_event = threading.Event()

        self.data_queue: queue.Queue = queue.Queue(maxsize=4096)
        self.image_queue: queue.Queue = queue.Queue(maxsize=128)
        self.error: str | None = None
        self.eye_packet_count = 0
        self.image_packet_count = 0

    def load_library(self, sdk_bin_dir: str):
        """Load runtime/USERSDK.dll.

        sdk_bin_dir is the existing aSeeVRClient/bin directory used by the old
        VR wrapper. The runtime directory is derived from the SDK root.
        """
        self._client_dir = Path(sdk_bin_dir).resolve()
        if self._client_dir.name.lower() == "bin":
            self._sdk_root = self._client_dir.parent.parent
        else:
            self._sdk_root = self._client_dir

        self._runtime_dir = self._sdk_root / "runtime"
        self._sdk_config = self._runtime_dir / "devices" / "Droolon" / "config"
        usersdk_path = self._runtime_dir / "USERSDK.dll"
        client_path = self._sdk_root / "aSeeVRClient" / "bin" / "aSeeVRClient.dll"

        if not usersdk_path.is_file():
            raise FileNotFoundError(f"USERSDK.dll not found: {usersdk_path}")
        if not client_path.is_file():
            raise FileNotFoundError(f"aSeeVRClient.dll not found: {client_path}")
        if not self._sdk_config.is_dir():
            raise FileNotFoundError(f"VR SDK config directory not found: {self._sdk_config}")

        self._add_dll_dir(self._runtime_dir)
        self._add_dll_dir(self._sdk_config)
        self._add_dll_dir(self._sdk_config / "armeabi")
        self._add_dll_dir(client_path.parent)

        self._dll = C.WinDLL(str(usersdk_path))
        self._declare_usersdk(self._dll)
        self._client_dll = C.WinDLL(str(client_path))
        self._declare_client_sdk(self._client_dll)

    def connect(self, port: int = DEFAULT_PORT):
        if self._dll is None or self._client_dll is None:
            raise RuntimeError("VR USERSDK library not loaded")
        self._coefficient_blob = self._refresh_coefficient(port)
        self._stop_runtime_processes()

    def start(self, enable_iris: bool = False, eye_mode: int = 3):
        del enable_iris, eye_mode
        if self._dll is None:
            raise RuntimeError("VR USERSDK library not loaded")

        self._clear_queues()
        self.error = None
        self.eye_packet_count = 0
        self.image_packet_count = 0
        self._first_callback_event.clear()
        self._coefficients = self._split_coefficient(self._coefficient_blob)
        self._released = False

        config_bytes = str(self._sdk_config).encode("mbcs")
        ret = self._dll._7_CALL_init_sdk(config_bytes, 3, 0)
        if ret != SUCCESS:
            raise RuntimeError(f"_7_CALL_init_sdk failed: {ret}")

        ret = self._dll._7_CALL_set_camera_image_callback(C.cast(self._image_cb, C.c_void_p), None)
        if ret != SUCCESS:
            self.disconnect()
            raise RuntimeError(f"_7_CALL_set_camera_image_callback failed: {ret}")

        pid1, pid2 = DEFAULT_RUNTIME_PIDS
        ret = self._dll._7_CALL_start_camera(pid1, pid2)
        if ret != SUCCESS:
            self.disconnect()
            raise RuntimeError(f"_7_CALL_start_camera({pid1}, {pid2}) failed: {ret}")
        self._started_camera = True

        ret = self._dll._7_CALL_start_eye_gaze_callback_ex(C.cast(self._gaze_cb, C.c_void_p), None)
        if ret != SUCCESS:
            self.disconnect()
            raise RuntimeError(f"_7_CALL_start_eye_gaze_callback_ex failed: {ret}")
        self._started_gaze = True

        for eye, coeff in ((1, self._coefficients[0]), (2, self._coefficients[1])):
            ret = self._dll._7_CALL_start_tracking(eye, C.byref(coeff))
            if ret != SUCCESS:
                self.disconnect()
                raise RuntimeError(f"_7_CALL_start_tracking({eye}) failed: {ret}")
        self._started_tracking = True

        if not self._first_callback_event.wait(timeout=5.0):
            self.disconnect()
            raise RuntimeError(
                "USERSDK started but no gaze/image callbacks arrived within 5s. "
                "Stop competing VR/Tobii/VIVE runtimes or replug/restart the eye tracker."
            )

    def disconnect(self):
        if self._dll is None or self._released:
            return

        if self._started_tracking:
            try:
                self._dll._7_CALL_stop_tracking()
            except Exception:
                pass
        self._started_tracking = False

        if self._started_gaze:
            try:
                self._dll._7_CALL_stop_eye_callback()
            except Exception:
                pass
        self._started_gaze = False

        if self._started_camera:
            try:
                self._dll._7_CALL_stop_camera()
            except Exception:
                pass
        self._started_camera = False

        try:
            self._dll._7_CALL_release()
        except Exception:
            pass
        self._released = True

        self._start_runtime_if_needed(DEFAULT_PORT)

    def stop(self):
        self.disconnect()

    def _declare_usersdk(self, dll: C.WinDLL):
        dll._7_CALL_init_sdk.argtypes = [C.c_char_p, C.c_int, C.c_int]
        dll._7_CALL_init_sdk.restype = C.c_int
        dll._7_CALL_release.argtypes = []
        dll._7_CALL_release.restype = C.c_int
        dll._7_CALL_set_camera_image_callback.argtypes = [C.c_void_p, C.c_void_p]
        dll._7_CALL_set_camera_image_callback.restype = C.c_int
        dll._7_CALL_start_camera.argtypes = [C.c_int, C.c_int]
        dll._7_CALL_start_camera.restype = C.c_int
        dll._7_CALL_stop_camera.argtypes = []
        dll._7_CALL_stop_camera.restype = C.c_int
        dll._7_CALL_start_eye_gaze_callback_ex.argtypes = [C.c_void_p, C.c_void_p]
        dll._7_CALL_start_eye_gaze_callback_ex.restype = C.c_int
        dll._7_CALL_stop_eye_callback.argtypes = []
        dll._7_CALL_stop_eye_callback.restype = C.c_int
        dll._7_CALL_start_tracking.argtypes = [C.c_int, C.c_void_p]
        dll._7_CALL_start_tracking.restype = C.c_int
        dll._7_CALL_stop_tracking.argtypes = []
        dll._7_CALL_stop_tracking.restype = C.c_int

    def _declare_client_sdk(self, dll: C.WinDLL):
        dll.aSeeVR_connect_server.argtypes = [C.POINTER(ASeeVRInitParam)]
        dll.aSeeVR_connect_server.restype = C.c_int
        dll.aSeeVR_register_callback.argtypes = [C.c_int, C.c_void_p, C.c_void_p]
        dll.aSeeVR_register_callback.restype = C.c_int
        dll.aSeeVR_get_coefficient.argtypes = []
        dll.aSeeVR_get_coefficient.restype = C.c_int
        dll.aSeeVR_disconnect_server.argtypes = []
        dll.aSeeVR_disconnect_server.restype = C.c_int

    def _refresh_coefficient(self, port: int) -> bytes:
        coefficient = ASeeVRCoefficient()
        self._coefficient_event.clear()
        self._pending_coefficient = coefficient

        self._start_runtime_if_needed(port)
        param = ASeeVRInitParam()
        param.mode = 1
        param.ports[0] = port

        ret = self._client_dll.aSeeVR_connect_server(C.byref(param))
        if ret != SUCCESS:
            fallback = self._runtime_dir / "user_data.dat"
            if fallback.is_file() and fallback.stat().st_size >= 2048:
                return fallback.read_bytes()[:2048]
            raise RuntimeError(f"aSeeVR_connect_server failed while refreshing coefficient: {ret}")

        try:
            self._client_dll.aSeeVR_register_callback(0, C.cast(self._state_cb, C.c_void_p), None)
            self._client_dll.aSeeVR_register_callback(3, C.cast(self._coefficient_cb, C.c_void_p), None)
            ret = self._client_dll.aSeeVR_get_coefficient()
            if ret != SUCCESS:
                raise RuntimeError(f"aSeeVR_get_coefficient failed: {ret}")
            if not self._coefficient_event.wait(timeout=3.0):
                fallback = self._runtime_dir / "user_data.dat"
                if fallback.is_file() and fallback.stat().st_size >= 2048:
                    return fallback.read_bytes()[:2048]
                raise RuntimeError("Timed out waiting for aSeeVR coefficient callback")
            return bytes(coefficient.buf)
        finally:
            try:
                self._client_dll.aSeeVR_disconnect_server()
            except Exception:
                pass

    def _split_coefficient(self, blob: bytes) -> list[UserSdkCoefficientWithSize]:
        if len(blob) < 2048:
            raise ValueError(f"Expected at least 2048 coefficient bytes, got {len(blob)}")
        result = []
        for offset in (0, 1024):
            coeff = UserSdkCoefficientWithSize()
            C.memmove(C.byref(coeff), blob[offset : offset + 1024], 1024)
            coeff.size = 1024
            result.append(coeff)
        return result

    def _on_state(self, state_ptr, context):
        del context
        if not state_ptr:
            return

    def _on_coefficient(self, coefficient_ptr, context):
        del context
        if not coefficient_ptr:
            return
        C.memmove(C.byref(self._pending_coefficient), coefficient_ptr, C.sizeof(ASeeVRCoefficient))
        self._coefficient_event.set()

    def _on_image(self, eye, image, size, width, height, timestamp, context):
        del context
        pc_arrival_timestamp = time.perf_counter()
        try:
            eye = int(eye)
            width = int(width)
            height = int(height)
            size = int(size)
            if eye not in (1, 2) or not image or width <= 0 or height <= 0 or size <= 0:
                return
            nbytes = min(size, width * height, 20_000_000)
            data = C.string_at(C.cast(image, C.c_void_p).value, nbytes)
            frame = EyeImageFrame(eye, data, width, height, int(timestamp), pc_arrival_timestamp)
            self._put_latest(self.image_queue, frame)
            self.image_packet_count += 1
            self._first_callback_event.set()
        except Exception as exc:
            if not self.error:
                self.error = f"image callback: {exc}"

    def _on_gaze(self, eyes_ptr, context):
        del context
        if not eyes_ptr:
            return
        pc_arrival_timestamp = time.perf_counter()
        try:
            eyes = eyes_ptr.contents
            gyro_data = eyes.gyro
            sample = {
                "pc_arrival_timestamp": pc_arrival_timestamp,
                "device_timestamp": int(eyes.timestamp),
                "gaze_x": float(eyes.recom_gaze.gaze_point.x),
                "gaze_y": float(eyes.recom_gaze.gaze_point.y),
                "gaze_z": float(eyes.recom_gaze.gaze_point.z),
                "left_pupil_x": float(eyes.left_pupil.pupil_center.x),
                "left_pupil_y": float(eyes.left_pupil.pupil_center.y),
                "right_pupil_x": float(eyes.right_pupil.pupil_center.x),
                "right_pupil_y": float(eyes.right_pupil.pupil_center.y),
                "left_pupil_diameter_mm": float(eyes.left_pupil.pupil_diameter_mm),
                "right_pupil_diameter_mm": float(eyes.right_pupil.pupil_diameter_mm),
                "left_blink": int(eyes.left_ex_data.blink),
                "right_blink": int(eyes.right_ex_data.blink),
                "left_openness": float(eyes.left_ex_data.openness),
                "right_openness": float(eyes.right_ex_data.openness),
                "gyro_timestamp": int(gyro_data.timestamp),
                "gyro_x": float(gyro_data.gyro[0]),
                "gyro_y": float(gyro_data.gyro[1]),
                "gyro_z": float(gyro_data.gyro[2]),
                "accel_x": float(gyro_data.accel[0]),
                "accel_y": float(gyro_data.accel[1]),
                "accel_z": float(gyro_data.accel[2]),
                "mag_x": float(gyro_data.mag[0]),
                "mag_y": float(gyro_data.mag[1]),
                "mag_z": float(gyro_data.mag[2]),
            }
            self._put_drop_oldest(self.data_queue, sample)
            self.eye_packet_count += 1
            self._first_callback_event.set()
        except Exception as exc:
            if not self.error:
                self.error = f"gaze callback: {exc}"

    def _add_dll_dir(self, path: Path):
        if path.exists() and hasattr(os, "add_dll_directory"):
            self._dll_dirs.append(os.add_dll_directory(str(path)))
        os.environ["PATH"] = str(path) + os.pathsep + os.environ.get("PATH", "")

    @staticmethod
    def _put_latest(q: queue.Queue, item):
        while True:
            try:
                q.put_nowait(item)
                return
            except queue.Full:
                try:
                    q.get_nowait()
                except queue.Empty:
                    return

    @staticmethod
    def _put_drop_oldest(q: queue.Queue, item):
        while True:
            try:
                q.put_nowait(item)
                return
            except queue.Full:
                try:
                    q.get_nowait()
                except queue.Empty:
                    return

    def _clear_queues(self):
        for q in (self.data_queue, self.image_queue):
            while True:
                try:
                    q.get_nowait()
                except queue.Empty:
                    break

    @staticmethod
    def _port_is_open(port: int, host: str = "127.0.0.1") -> bool:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.4)
        try:
            sock.connect((host, port))
            return True
        except OSError:
            return False
        finally:
            sock.close()

    def _start_runtime_if_needed(self, port: int):
        if self._port_is_open(port):
            return
        runtime_exe = self._runtime_dir / "Runtime.exe"
        if not runtime_exe.is_file():
            raise FileNotFoundError(f"Runtime.exe not found: {runtime_exe}")
        flags = 0x00000008 if sys.platform == "win32" else 0
        self._runtime_proc = subprocess.Popen(
            [str(runtime_exe)],
            cwd=str(self._runtime_dir),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=flags,
        )
        self._runtime_owned = True
        deadline = time.perf_counter() + 10.0
        while time.perf_counter() < deadline:
            if self._port_is_open(port):
                return
            time.sleep(0.2)
        raise RuntimeError(f"Runtime.exe did not open port {port} within 10s")

    def _stop_runtime_processes(self):
        if self._runtime_proc is not None:
            try:
                self._runtime_proc.terminate()
                self._runtime_proc.wait(timeout=3.0)
            except Exception:
                try:
                    self._runtime_proc.kill()
                except Exception:
                    pass
            self._runtime_proc = None
            self._runtime_owned = False

        if sys.platform != "win32":
            return
        try:
            subprocess.run(
                ["taskkill", "/IM", "Runtime.exe", "/F"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3.0,
                check=False,
            )
        except Exception:
            pass
        time.sleep(1.0)
