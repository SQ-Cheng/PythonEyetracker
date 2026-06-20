from __future__ import annotations

import ctypes as C
import os
import queue
import threading
import time
from pathlib import Path

import cv2


RUNTIME_DIR = Path(r"C:\7invensun\aSeeVR_UserSDK\runtime")
SDK_CONFIG = Path(r"C:\7invensun\aSeeVR_UserSDK\runtime\devices\Droolon\config")
DLL_DIR_HANDLES = []


def add_dll_dir(path: Path) -> None:
    if path.exists() and hasattr(os, "add_dll_directory"):
        DLL_DIR_HANDLES.append(os.add_dll_directory(str(path)))


ImageCb = C.WINFUNCTYPE(None, C.c_int, C.POINTER(C.c_ubyte), C.c_int, C.c_int, C.c_int, C.c_longlong, C.c_void_p)


def load_dll() -> C.WinDLL:
    os.environ["PATH"] = (
        str(RUNTIME_DIR)
        + os.pathsep
        + str(SDK_CONFIG)
        + os.pathsep
        + str(SDK_CONFIG / "armeabi")
        + os.pathsep
        + os.environ.get("PATH", "")
    )
    add_dll_dir(RUNTIME_DIR)
    add_dll_dir(SDK_CONFIG)
    add_dll_dir(SDK_CONFIG / "armeabi")
    dll = C.WinDLL(str(RUNTIME_DIR / "USERSDK.dll"))
    dll._7_CALL_init_sdk.argtypes = [C.c_char_p, C.c_int, C.c_int]
    dll._7_CALL_init_sdk.restype = C.c_int
    dll._7_CALL_set_camera_image_callback.argtypes = [C.c_void_p, C.c_void_p]
    dll._7_CALL_set_camera_image_callback.restype = C.c_int
    dll._7_CALL_start_camera.argtypes = [C.c_int, C.c_int]
    dll._7_CALL_start_camera.restype = C.c_int
    dll._7_CALL_stop_camera.argtypes = []
    dll._7_CALL_stop_camera.restype = C.c_int
    dll._7_CALL_release.argtypes = []
    dll._7_CALL_release.restype = C.c_int
    return dll


def try_cv2(label: str) -> list[dict]:
    rows = []
    for api_name, api in [("DSHOW", cv2.CAP_DSHOW), ("MSMF", cv2.CAP_MSMF)]:
        for idx in range(4):
            cap = cv2.VideoCapture(idx, api)
            opened = cap.isOpened()
            ok = False
            shape = None
            if opened:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 120)
                ok, frame = cap.read()
                if ok and frame is not None:
                    shape = tuple(frame.shape)
            rows.append(
                {
                    "label": label,
                    "api": api_name,
                    "idx": idx,
                    "opened": opened,
                    "read_ok": ok,
                    "shape": shape,
                    "w": cap.get(cv2.CAP_PROP_FRAME_WIDTH) if opened else 0,
                    "h": cap.get(cv2.CAP_PROP_FRAME_HEIGHT) if opened else 0,
                    "fps": cap.get(cv2.CAP_PROP_FPS) if opened else 0,
                }
            )
            cap.release()
    return rows


def main() -> int:
    frames: "queue.Queue[tuple[int, int, int, int, int, float]]" = queue.Queue()
    counter = {"n": 0}

    def on_image(eye, data_ptr, size, width, height, timestamp, context):
        seq = counter["n"]
        counter["n"] += 1
        if seq < 12:
            frames.put((eye, size, width, height, timestamp, time.perf_counter()))

    print("cv2 before usersdk")
    before = try_cv2("before")
    for row in before:
        print(row)

    dll = load_dll()
    cb = ImageCb(on_image)
    started = False
    try:
        ret = dll._7_CALL_init_sdk(str(SDK_CONFIG).encode("mbcs"), 3, 0)
        print("_7_CALL_init_sdk ->", ret)
        ret = dll._7_CALL_set_camera_image_callback(C.cast(cb, C.c_void_p), None)
        print("_7_CALL_set_camera_image_callback ->", ret)
        ret = dll._7_CALL_start_camera(17178, 17156)
        print("_7_CALL_start_camera ->", ret)
        started = ret == 0
        time.sleep(0.5)
        print("usersdk frames so far", counter["n"])
        while not frames.empty():
            print("usersdk frame", frames.get_nowait())

        print("cv2 while usersdk")
        during = try_cv2("during")
        for row in during:
            print(row)
        time.sleep(0.5)
        print("usersdk frames final", counter["n"])
    finally:
        if started:
            print("_7_CALL_stop_camera ->", dll._7_CALL_stop_camera())
        print("_7_CALL_release ->", dll._7_CALL_release())
        _ = cb
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
