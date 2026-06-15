"""I7VR SDK 封装 — 对应 aSeeVRClient.h / aSeeVRUtility.h

通过 TCP 连接到本地 Runtime.exe (端口 5777)，异步接收眼动数据。
用法与 sdk_wrapper.py 的 wrapper 保持一致的 dict 输出格式。

Runtime.exe 在 connect() 时自动启动（以 sdk 根目录的 runtime/ 为工作目录）。
"""

import ctypes
import os
import queue
import socket
import subprocess
import sys
import threading
import time

from sdk_vr_types import (
    ASeeVRCallbackType, ASeeVRStateCode, ASeeVRReturnCode,
    ASeeVREye, ASeeVREyeDataItemType,
    ASeeVRPoint2D, ASeeVRPoint3D,
    ASeeVRInitParam, ASeeVRLanuchParam, ASeeVRCoefficient, ASeeVRState,
    StateCallback, EyeDataCallback, CoefficientCallback,
)


class VrWrapper:
    """VR SDK 客户端封装。

    connect() 会先尝试启动 Runtime.exe（如果端口未监听），disconnect() 时关闭。
    """

    DEFAULT_PORT = 5777
    RUNTIME_STARTUP_TIMEOUT = 15.0  # 等待 Runtime.exe 启动的秒数

    def __init__(self):
        self._dll: ctypes.WinDLL | None = None
        self._sdk_dir = ""
        self._runtime_dir = ""
        self._runtime_proc: subprocess.Popen | None = None
        self._runtime_owned = False  # 是否由本进程启动了 Runtime.exe
        self._connected = False
        self._started = False
        self._coefficient_file = ""

        # 回调保持引用防止被 GC
        self._cb_state = StateCallback(self._on_state)
        self._cb_eye_data = EyeDataCallback(self._on_eye_data)
        self._cb_coefficient = CoefficientCallback(self._on_coefficient)

        # 异步状态
        self._start_error: int | None = None
        self._started_event = threading.Event()

        self.data_queue: queue.Queue = queue.Queue()
        self.error: str | None = None
        self.eye_packet_count = 0

    # ── public API ──────────────────────────────────────────

    def load_library(self, sdk_dir: str):
        """加载 aSeeVRClient.dll。

        sdk_dir: aSeeVRClient/bin 所在目录。
        同时探测 runtime/ 目录用于自动启动 Runtime.exe。
        """
        self._sdk_dir = os.path.abspath(sdk_dir)
        dll_path = os.path.join(self._sdk_dir, "aSeeVRClient.dll")
        if not os.path.isfile(dll_path):
            raise FileNotFoundError(f"aSeeVRClient.dll not found in {self._sdk_dir}")

        # 构造 runtime 目录路径: sdk_dir/../../runtime/
        vr_root = os.path.dirname(os.path.dirname(self._sdk_dir))
        self._runtime_dir = os.path.join(vr_root, "runtime")
        if not os.path.isdir(self._runtime_dir):
            self._runtime_dir = os.path.join(vr_root)

        self._dll = ctypes.WinDLL(dll_path)

        # 声明函数签名（确保参数和返回值类型正确）
        self._dll.aSeeVR_connect_server.argtypes = [ctypes.POINTER(ASeeVRInitParam)]
        self._dll.aSeeVR_connect_server.restype = ctypes.c_int

        self._dll.aSeeVR_register_callback.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.py_object]
        self._dll.aSeeVR_register_callback.restype = ctypes.c_int

        self._dll.aSeeVR_start.argtypes = [ctypes.POINTER(ASeeVRCoefficient), ctypes.POINTER(ASeeVRLanuchParam)]
        self._dll.aSeeVR_start.restype = ctypes.c_int

        self._dll.aSeeVR_stop.argtypes = []
        self._dll.aSeeVR_stop.restype = ctypes.c_int

        self._dll.aSeeVR_disconnect_server.argtypes = []
        self._dll.aSeeVR_disconnect_server.restype = ctypes.c_int

        self._dll.aSeeVR_get_coefficient.argtypes = []
        self._dll.aSeeVR_get_coefficient.restype = ctypes.c_int

        # utility functions
        self._dll.aSeeVR_get_point2d.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ASeeVRPoint2D)]
        self._dll.aSeeVR_get_point2d.restype = ctypes.c_int

        self._dll.aSeeVR_get_float.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self._dll.aSeeVR_get_float.restype = ctypes.c_int

        self._dll.aSeeVR_get_int32.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int32)]
        self._dll.aSeeVR_get_int32.restype = ctypes.c_int

        self._dll.aSeeVR_get_int64.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int64)]
        self._dll.aSeeVR_get_int64.restype = ctypes.c_int

    def connect(self, port: int = DEFAULT_PORT):
        """连接到 Runtime 服务。

        如果端口未监听则自动启动 Runtime.exe。
        """
        if self._dll is None:
            raise RuntimeError("VR SDK library not loaded — call load_library() first")

        # 1. 确保 Runtime.exe 正在运行
        if not self._port_is_open(port):
            self._start_runtime(port)

        # 2. 连接
        param = ASeeVRInitParam()
        param.mode = 1  # control mode
        param.ports[0] = port

        ret = self._dll.aSeeVR_connect_server(ctypes.byref(param))
        if ret != ASeeVRReturnCode.success:
            code_map = {v: k for k, v in vars(ASeeVRReturnCode).items() if isinstance(v, int)}
            code_name = code_map.get(ret, str(ret))
            raise RuntimeError(
                f"aSeeVR_connect_server failed: {ret} ({code_name}). "
                f"Ensure Runtime.exe is running on port {port}."
            )

        self._dll.aSeeVR_register_callback(
            ASeeVRCallbackType.state, self._cb_state, self)
        self._dll.aSeeVR_register_callback(
            ASeeVRCallbackType.eye_data, self._cb_eye_data, self)
        self._dll.aSeeVR_register_callback(
            ASeeVRCallbackType.coefficient, self._cb_coefficient, self)

        # 3. 获取标定系数并缓存到文件（start 需要提交系数）
        self._coefficient_file = os.path.join(self._sdk_dir, "_vr_coefficient.dat")
        self._dll.aSeeVR_get_coefficient()
        time.sleep(0.5)
        self._connected = True

    def start(self, enable_iris: bool = False, eye_mode: int = 3):
        """启动眼动数据流。

        Args:
            enable_iris: 是否启用虹膜识别
            eye_mode: 1=左眼, 2=右眼, 3=双眼
        """
        if self._dll is None:
            raise RuntimeError("VR SDK library not loaded")
        self._started_event.clear()
        self._start_error = None

        coe = ASeeVRCoefficient()
        if os.path.isfile(self._coefficient_file):
            with open(self._coefficient_file, "rb") as f:
                raw = f.read(2048)
                ctypes.memmove(coe.buf, raw, len(raw))

        launch = ASeeVRLanuchParam()
        launch.enable_iris = 1 if enable_iris else 0
        launch.eye = eye_mode

        ret = self._dll.aSeeVR_start(ctypes.byref(coe), ctypes.byref(launch))
        if ret != ASeeVRReturnCode.success:
            raise RuntimeError(f"aSeeVR_start failed: {ret}")

        if not self._started_event.wait(timeout=5.0):
            raise RuntimeError("aSeeVR_start timed out: no state callback received")
        if self._start_error and self._start_error != 0:
            raise RuntimeError(f"aSeeVR_start reported error: {self._start_error}")
        self._started = True

    def stop(self):
        """停止眼动数据流。"""
        if self._dll is None:
            return
        try:
            self._dll.aSeeVR_stop()
        except Exception:
            pass
        self._started = False

    def disconnect(self):
        """断开与 Runtime 的连接；如果是本进程启动的 Runtime，也会关闭它。"""
        if self._dll is None:
            return
        self.stop()
        try:
            self._dll.aSeeVR_disconnect_server()
        except Exception:
            pass
        self._connected = False
        self._stop_runtime()

    # ── Runtime.exe 生命周期 ────────────────────────────────

    @staticmethod
    def _port_is_open(port: int, host: str = "127.0.0.1") -> bool:
        """检查端口是否已被监听。"""
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        try:
            s.connect((host, port))
            s.close()
            return True
        except (ConnectionRefusedError, OSError, socket.timeout):
            return False

    def _start_runtime(self, port: int):
        """启动 Runtime.exe 并等待其端口就绪。"""
        runtime_exe = os.path.join(self._runtime_dir, "Runtime.exe")
        if not os.path.isfile(runtime_exe):
            raise FileNotFoundError(
                f"Runtime.exe not found at {runtime_exe}. "
                "Please ensure the VR SDK runtime directory is correct."
            )

        print(f"[VrSdk] Starting Runtime.exe from {self._runtime_dir} …")
        try:
            # Runtime.exe 是 Qt5 GUI 应用，使用 DETACHED_PROCESS
            flags = 0x00000008 if sys.platform == "win32" else 0  # DETACHED_PROCESS
            self._runtime_proc = subprocess.Popen(
                [runtime_exe],
                cwd=self._runtime_dir,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
                creationflags=flags,
            )
        except Exception as exc:
            raise RuntimeError(
                f"Failed to launch Runtime.exe: {exc}\n"
                f"Try starting {runtime_exe} manually first."
            )

        self._runtime_owned = True

        # 等待端口就绪
        deadline = time.perf_counter() + self.RUNTIME_STARTUP_TIMEOUT
        while time.perf_counter() < deadline:
            if self._port_is_open(port):
                print(f"[VrSdk] Runtime.exe ready on port {port}")
                return
            if self._runtime_proc.poll() is not None:
                raise RuntimeError(
                    f"Runtime.exe exited with code {self._runtime_proc.returncode}. "
                    f"Check that the VR SDK installation at {self._runtime_dir} is complete."
                )
            time.sleep(0.5)

        # 超时：Runtime 进程在跑但端口未开放 → 很可能是 VR 硬件未连接
        raise RuntimeError(
            f"Runtime.exe started but did not open port {port} within "
            f"{self.RUNTIME_STARTUP_TIMEOUT}s.\n\n"
            "This usually means the VR headset is NOT connected or NOT recognized.\n"
            "Please:\n"
            "  1. Ensure the VR headset is plugged in and powered on.\n"
            "  2. Check Device Manager for the eye tracker hardware.\n"
            "  3. Start Runtime.exe manually (it will show a GUI window):\n"
            f"     {runtime_exe}\n"
            "  4. Once Runtime shows the device as connected, re-run this script."
        )

    def _stop_runtime(self):
        """安全关闭本进程启动的 Runtime.exe。"""
        if not self._runtime_owned or self._runtime_proc is None:
            return
        try:
            self._runtime_proc.terminate()
            self._runtime_proc.wait(timeout=5)
        except Exception:
            try:
                self._runtime_proc.kill()
            except Exception:
                pass
        self._runtime_proc = None
        self._runtime_owned = False

    # ── callbacks ───────────────────────────────────────────

    def _on_state(self, pstate, context):
        state = ctypes.cast(pstate, ctypes.POINTER(ASeeVRState)).contents
        code = state.code
        err = state.error
        self_ref = context
        print(f"[VrSdk] state code={code} error={err}")
        if code == ASeeVRStateCode.api_start:
            self_ref._start_error = err
            self_ref._started_event.set()
        elif code == ASeeVRStateCode.api_stop:
            pass

    def _on_eye_data(self, peye_data, context):
        self_ref = context
        try:
            sample = self_ref._extract_eye_data(peye_data)
            self_ref.data_queue.put(sample)
            self_ref.eye_packet_count += 1
        except Exception as exc:
            if not self_ref.error:
                self_ref.error = f"eye_data extraction: {exc}"

    def _on_coefficient(self, pcoe, context):
        coe = ctypes.cast(pcoe, ctypes.POINTER(ASeeVRCoefficient)).contents
        self_ref = context
        try:
            with open(self_ref._coefficient_file, "wb") as fp:
                fp.write(bytes(coe.buf))
        except Exception as exc:
            print(f"[VrSdk] failed to save coefficient: {exc}")

    # ── data extraction ─────────────────────────────────────

    def _extract_eye_data(self, peye_data) -> dict:
        """从 opaque aSeeVREyeData* 中提取与 Glasses SDK gaze_callback 同名格式的 dict。"""
        pc_arrival_timestamp = time.perf_counter()
        dll = self._dll

        # ── 使用 undefine_eye 获取推荐眼数据 ──
        def _get_pt2d(eye, item_type) -> tuple[float, float]:
            pt = ASeeVRPoint2D()
            dll.aSeeVR_get_point2d(peye_data, eye, item_type, ctypes.byref(pt))
            return (pt.x, pt.y)

        def _get_float(eye, item_type) -> float:
            v = ctypes.c_float()
            dll.aSeeVR_get_float(peye_data, eye, item_type, ctypes.byref(v))
            return v.value

        def _get_int32(eye, item_type) -> int:
            v = ctypes.c_int32()
            dll.aSeeVR_get_int32(peye_data, eye, item_type, ctypes.byref(v))
            return v.value

        def _get_int64(eye, item_type) -> int:
            v = ctypes.c_int64()
            dll.aSeeVR_get_int64(peye_data, eye, item_type, ctypes.byref(v))
            return v.value

        E = ASeeVREye
        T = ASeeVREyeDataItemType

        # 推荐眼注视点
        gaze_x, gaze_y = _get_pt2d(E.undefine_eye, T.gaze)

        # 左右瞳孔中心
        lp_x, lp_y = _get_pt2d(E.left_eye, T.pupil_center)
        rp_x, rp_y = _get_pt2d(E.right_eye, T.pupil_center)

        # 瞳孔直径 mm
        lpd = _get_float(E.left_eye, T.pupil_diameter_mm)
        rpd = _get_float(E.right_eye, T.pupil_diameter_mm)

        # 眨眼 & 睁眼度
        l_blink = _get_int32(E.left_eye, T.blink)
        r_blink = _get_int32(E.right_eye, T.blink)
        l_openness = _get_float(E.left_eye, T.openness)
        r_openness = _get_float(E.right_eye, T.openness)

        # 时间戳
        timestamp = _get_int64(E.undefine_eye, T.timestamp)

        return {
            "pc_arrival_timestamp": pc_arrival_timestamp,
            "device_timestamp": int(timestamp),
            "gaze_x": float(gaze_x),
            "gaze_y": float(gaze_y),
            "gaze_z": 0.0,
            "left_pupil_x": float(lp_x),
            "left_pupil_y": float(lp_y),
            "right_pupil_x": float(rp_x),
            "right_pupil_y": float(rp_y),
            "left_pupil_diameter_mm": float(lpd),
            "right_pupil_diameter_mm": float(rpd),
            "left_blink": int(l_blink),
            "right_blink": int(r_blink),
            "left_openness": float(l_openness),
            "right_openness": float(r_openness),
            "gyro_timestamp": 0,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0,
            "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0,
            "mag_x": 0.0, "mag_y": 0.0, "mag_z": 0.0,
        }
