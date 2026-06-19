"""
Test capture for aSeeVR eye-image callback.

This script does not modify the SDK folder. It loads aSeeVRClient.dll through
ctypes, registers the eye_image callback, copies each callback buffer
immediately, and writes frames as dependency-free PGM grayscale images.

Expected callback payload from aSeeVRTypes.h:
    flag      int32   1 = left eye, 2 = right eye
    width     int32   image width in pixels
    height    int32   image height in pixels
    data      uint8*  grayscale image data
    timestamp int64   image timestamp

The header does not expose stride/pitch, so the only practical interpretation
is tightly packed 8-bit grayscale with width * height bytes per frame.

Prerequisites:
    1. Use 64-bit Python, matching the x64 SDK DLL.
    2. Start runtime\\Runtime.exe separately and connect the eye tracker.
    3. Keep the SDK at the default path or pass --sdk-root.
"""

from __future__ import annotations

import argparse
import ctypes as C
import os
import platform
import queue
import sys
import threading
import time
from pathlib import Path


SUCCESS = 0

CALLBACK_STATE = 0
CALLBACK_EYE_DATA = 1
CALLBACK_EYE_IMAGE = 2
CALLBACK_COEFFICIENT = 3

MODE_CONTROL = 1

EYE_FLAG_NAMES = {
    1: "left",
    2: "right",
}

DLL_DIR_HANDLES = []


class ASeeVRState(C.Structure):
    _fields_ = [
        ("code", C.c_int32),
        ("error", C.c_int32),
    ]


class ASeeVRImage(C.Structure):
    _fields_ = [
        ("flag", C.c_int32),
        ("width", C.c_int32),
        ("height", C.c_int32),
        ("data", C.POINTER(C.c_uint8)),
        ("timestamp", C.c_int64),
    ]


class ASeeVRInitParam(C.Structure):
    _fields_ = [
        ("mode", C.c_int),
        ("ports", C.c_int16 * 10),
    ]


class ASeeVRCoefficient(C.Structure):
    _fields_ = [
        ("buf", C.c_uint8 * 2048),
    ]


STATE_CALLBACK = C.WINFUNCTYPE(None, C.POINTER(ASeeVRState), C.c_void_p)
IMAGE_CALLBACK = C.WINFUNCTYPE(None, C.POINTER(ASeeVRImage), C.c_void_p)
COEFFICIENT_CALLBACK = C.WINFUNCTYPE(None, C.POINTER(ASeeVRCoefficient), C.c_void_p)


def add_dll_dirs(*paths: Path) -> None:
    for path in paths:
        if path.exists() and hasattr(os, "add_dll_directory"):
            DLL_DIR_HANDLES.append(os.add_dll_directory(str(path)))


def load_sdk(sdk_root: Path) -> C.WinDLL:
    client_dir = sdk_root / "aSeeVRClient" / "bin"
    runtime_dir = sdk_root / "runtime"
    dll_path = client_dir / "aSeeVRClient.dll"

    if not dll_path.exists():
        raise FileNotFoundError(f"Cannot find {dll_path}")

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

    # The C++ header has a default second argument. ctypes must pass it explicitly.
    dll.aSeeVR_start.argtypes = [C.POINTER(ASeeVRCoefficient), C.c_void_p]
    dll.aSeeVR_start.restype = C.c_int

    dll.aSeeVR_stop.argtypes = []
    dll.aSeeVR_stop.restype = C.c_int

    return dll


def check_abi() -> None:
    if platform.architecture()[0] != "64bit":
        raise RuntimeError("Use 64-bit Python; this SDK ships x64 DLLs.")

    image_size = C.sizeof(ASeeVRImage)
    data_offset = ASeeVRImage.data.offset
    timestamp_offset = ASeeVRImage.timestamp.offset

    print(
        "ASeeVRImage ABI:",
        f"sizeof={image_size}",
        f"data_offset={data_offset}",
        f"timestamp_offset={timestamp_offset}",
    )

    # MSVC x64 layout: int32,int32,int32,padding,pointer,int64 = 32 bytes.
    if image_size != 32 or data_offset != 16 or timestamp_offset != 24:
        raise RuntimeError(
            "ASeeVRImage layout does not match the x64 SDK header expectation."
        )


def write_pgm(path: Path, width: int, height: int, data: bytes, comment: str) -> None:
    header = f"P5\n# {comment}\n{width} {height}\n255\n".encode("ascii")
    with path.open("wb") as file:
        file.write(header)
        file.write(data)


def frame_writer(
    frames: "queue.Queue[tuple[int, int, int, int, bytes]]",
    output_dir: Path,
    max_frames: int,
    done: threading.Event,
) -> None:
    saved = 0
    output_dir.mkdir(parents=True, exist_ok=True)

    while saved < max_frames and not done.is_set():
        try:
            flag, width, height, timestamp, data = frames.get(timeout=0.2)
        except queue.Empty:
            continue

        eye = EYE_FLAG_NAMES.get(flag, f"flag{flag}")
        filename = f"{saved:04d}_{eye}_{width}x{height}_{timestamp}.pgm"
        write_pgm(
            output_dir / filename,
            width,
            height,
            data,
            f"flag={flag} eye={eye} timestamp={timestamp}",
        )
        saved += 1
        print(f"saved {filename}")

    done.set()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sdk-root",
        type=Path,
        default=Path(r"E:\7invensun\aSeeVR_UserSDK"),
        help="Root folder of aSeeVR_UserSDK.",
    )
    parser.add_argument("--port", type=int, default=5777)
    parser.add_argument("--max-frames", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--coefficient-timeout", type=float, default=5.0)
    parser.add_argument(
        "--max-frame-bytes",
        type=int,
        default=10_000_000,
        help="Safety limit for width * height before copying callback memory.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).with_name("eye_image_callback_frames"),
    )
    args = parser.parse_args()

    check_abi()

    dll = load_sdk(args.sdk_root)

    frames: "queue.Queue[tuple[int, int, int, int, bytes]]" = queue.Queue()
    done = threading.Event()
    coefficient_ready = threading.Event()
    coefficient = ASeeVRCoefficient()

    # Keep callback objects alive for the entire SDK session.
    callbacks = []

    def on_state(state_ptr: C.POINTER(ASeeVRState), context: int) -> None:
        if not state_ptr:
            return
        state = state_ptr.contents
        print(f"state callback: code={state.code} error={state.error}")

    def on_coefficient(
        coefficient_ptr: C.POINTER(ASeeVRCoefficient),
        context: int,
    ) -> None:
        if not coefficient_ptr:
            return
        C.memmove(C.byref(coefficient), coefficient_ptr, C.sizeof(coefficient))
        coefficient_ready.set()
        print("coefficient callback: received 2048 bytes")

    def on_eye_image(image_ptr: C.POINTER(ASeeVRImage), context: int) -> None:
        if done.is_set() or not image_ptr:
            return

        image = image_ptr.contents
        width = int(image.width)
        height = int(image.height)
        expected_size = width * height

        if width <= 0 or height <= 0 or expected_size > args.max_frame_bytes:
            print(
                "ignored image callback with suspicious dimensions:",
                f"flag={image.flag}",
                f"width={width}",
                f"height={height}",
                f"timestamp={image.timestamp}",
            )
            return

        if not image.data:
            print("ignored image callback with null data pointer")
            return

        # Critical: copy immediately. The SDK does not document pointer lifetime.
        data_address = C.cast(image.data, C.c_void_p).value
        if not data_address:
            print("ignored image callback with null data address")
            return
        frame_bytes = C.string_at(data_address, expected_size)
        frames.put((int(image.flag), width, height, int(image.timestamp), frame_bytes))

    state_cb = STATE_CALLBACK(on_state)
    coeff_cb = COEFFICIENT_CALLBACK(on_coefficient)
    image_cb = IMAGE_CALLBACK(on_eye_image)
    callbacks.extend([state_cb, coeff_cb, image_cb])

    writer = threading.Thread(
        target=frame_writer,
        args=(frames, args.output_dir, args.max_frames, done),
        daemon=True,
    )
    writer.start()

    connected = False
    started = False

    try:
        param = ASeeVRInitParam()
        param.mode = MODE_CONTROL
        param.ports[0] = args.port

        ret = dll.aSeeVR_connect_server(C.byref(param))
        print(f"aSeeVR_connect_server -> {ret}")
        if ret != SUCCESS:
            return 2
        connected = True

        registrations = [
            (CALLBACK_STATE, state_cb),
            (CALLBACK_EYE_IMAGE, image_cb),
            (CALLBACK_COEFFICIENT, coeff_cb),
        ]
        for callback_type, callback in registrations:
            ret = dll.aSeeVR_register_callback(
                callback_type,
                C.cast(callback, C.c_void_p),
                None,
            )
            print(f"aSeeVR_register_callback({callback_type}) -> {ret}")
            if ret != SUCCESS:
                return 3

        ret = dll.aSeeVR_get_coefficient()
        print(f"aSeeVR_get_coefficient -> {ret}")
        if ret != SUCCESS:
            return 4

        if not coefficient_ready.wait(args.coefficient_timeout):
            print("No coefficient callback received before timeout.")
            return 5

        ret = dll.aSeeVR_start(C.byref(coefficient), None)
        print(f"aSeeVR_start -> {ret}")
        if ret != SUCCESS:
            return 6
        started = True

        deadline = time.monotonic() + args.timeout
        while not done.is_set() and time.monotonic() < deadline:
            time.sleep(0.05)

        done.set()
        writer.join(timeout=2.0)
        print(f"frames written to: {args.output_dir}")
        return 0

    finally:
        done.set()
        if started:
            print(f"aSeeVR_stop -> {dll.aSeeVR_stop()}")
        if connected:
            print(f"aSeeVR_disconnect_server -> {dll.aSeeVR_disconnect_server()}")
        _ = callbacks


if __name__ == "__main__":
    sys.exit(main())
