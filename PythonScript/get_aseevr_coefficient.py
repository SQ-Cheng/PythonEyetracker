from __future__ import annotations

import argparse
import ctypes as C
import os
import sys
import threading
from pathlib import Path

from example_paths import resolve_vr_sdk_root


SUCCESS = 0
MODE_CONTROL = 1
CALLBACK_COEFFICIENT = 3
DLL_DIR_HANDLES = []


class ASeeVRInitParam(C.Structure):
    _fields_ = [
        ("mode", C.c_int),
        ("ports", C.c_int16 * 10),
    ]


class ASeeVRCoefficient(C.Structure):
    _fields_ = [
        ("buf", C.c_uint8 * 2048),
    ]


COEFFICIENT_CALLBACK = C.WINFUNCTYPE(None, C.POINTER(ASeeVRCoefficient), C.c_void_p)


def add_dll_dirs(*paths: Path) -> None:
    for path in paths:
        if path.exists() and hasattr(os, "add_dll_directory"):
            DLL_DIR_HANDLES.append(os.add_dll_directory(str(path)))


def load_sdk(sdk_root: Path) -> C.WinDLL:
    client_dir = sdk_root / "aSeeVRClient" / "bin"
    runtime_dir = sdk_root / "runtime"
    dll_path = client_dir / "aSeeVRClient.dll"
    add_dll_dirs(client_dir, runtime_dir)
    dll = C.WinDLL(str(dll_path))
    dll.aSeeVR_register_callback.argtypes = [C.c_int, C.c_void_p, C.c_void_p]
    dll.aSeeVR_register_callback.restype = C.c_int
    dll.aSeeVR_connect_server.argtypes = [C.POINTER(ASeeVRInitParam)]
    dll.aSeeVR_connect_server.restype = C.c_int
    dll.aSeeVR_disconnect_server.argtypes = []
    dll.aSeeVR_disconnect_server.restype = C.c_int
    dll.aSeeVR_get_coefficient.argtypes = []
    dll.aSeeVR_get_coefficient.restype = C.c_int
    return dll


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-root", type=Path, default=resolve_vr_sdk_root())
    parser.add_argument("--port", type=int, default=5777)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).with_name("aseevr_coefficient.bin"),
    )
    args = parser.parse_args()

    dll = load_sdk(args.sdk_root)
    ready = threading.Event()
    coefficient = ASeeVRCoefficient()

    def on_coefficient(ptr: C.POINTER(ASeeVRCoefficient), context: int) -> None:
        if ptr:
            C.memmove(C.byref(coefficient), ptr, C.sizeof(coefficient))
            ready.set()
            print("coefficient callback: received 2048 bytes", flush=True)

    cb = COEFFICIENT_CALLBACK(on_coefficient)
    connected = False
    try:
        param = ASeeVRInitParam()
        param.mode = MODE_CONTROL
        param.ports[0] = args.port
        ret = dll.aSeeVR_connect_server(C.byref(param))
        print(f"aSeeVR_connect_server -> {ret}", flush=True)
        if ret != SUCCESS:
            return 2
        connected = True

        ret = dll.aSeeVR_register_callback(CALLBACK_COEFFICIENT, C.cast(cb, C.c_void_p), None)
        print(f"aSeeVR_register_callback(3) -> {ret}", flush=True)
        if ret != SUCCESS:
            return 3

        ret = dll.aSeeVR_get_coefficient()
        print(f"aSeeVR_get_coefficient -> {ret}", flush=True)
        if ret != SUCCESS:
            return 4

        if not ready.wait(args.timeout):
            print("No coefficient callback before timeout.", flush=True)
            return 5

        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_bytes(bytes(coefficient.buf))
        print(f"wrote coefficient: {args.output} ({args.output.stat().st_size} bytes)", flush=True)
        return 0
    finally:
        if connected:
            print(f"aSeeVR_disconnect_server -> {dll.aSeeVR_disconnect_server()}", flush=True)
        _ = cb
        _ = DLL_DIR_HANDLES


if __name__ == "__main__":
    sys.exit(main())
