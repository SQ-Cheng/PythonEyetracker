from __future__ import annotations

import argparse
import ctypes as C
import os
import queue
import sys
import threading
import time
from pathlib import Path


SUCCESS = 0
DLL_DIR_HANDLES = []


def add_dll_dir(path: Path) -> None:
    if path.exists() and hasattr(os, "add_dll_directory"):
        DLL_DIR_HANDLES.append(os.add_dll_directory(str(path)))


def write_pgm(path: Path, width: int, height: int, data: bytes, comment: str) -> None:
    header = f"P5\n# {comment}\n{width} {height}\n255\n".encode("ascii")
    with path.open("wb") as file:
        file.write(header)
        file.write(data)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--runtime-dir",
        type=Path,
        default=Path(r"C:\7invensun\aSeeVR_UserSDK\runtime"),
    )
    parser.add_argument(
        "--sdk-config",
        type=Path,
        default=Path(r"C:\7invensun\aSeeVR_UserSDK\runtime\devices\Droolon\config"),
    )
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--max-frames", type=int, default=8)
    parser.add_argument("--pid1", type=int, default=17178)
    parser.add_argument("--pid2", type=int, default=17156)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).with_name("usersdk_camera_callback_frames"),
    )
    args = parser.parse_args()

    add_dll_dir(args.runtime_dir)
    add_dll_dir(args.sdk_config)
    add_dll_dir(args.sdk_config / "armeabi")
    dll = C.WinDLL(str(args.runtime_dir / "USERSDK.dll"))

    # Inferred from wrapper assembly and Runtime logs:
    #   _7_CALL_init_sdk(path, trackingMode, isClass)
    #   _7_CALL_set_camera_image_callback(cb, context)
    #   _7_CALL_start_camera(pid1, pid2)
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

    # This mirrors the public Glasses image callback signature. It is an educated
    # probe, not a documented VR USERSDK contract.
    CAMERA_IMAGE_CALLBACK = C.WINFUNCTYPE(
        None,
        C.c_int,
        C.POINTER(C.c_uint8),
        C.c_int,
        C.c_int,
        C.c_int,
        C.c_longlong,
        C.c_void_p,
    )

    frames: "queue.Queue[tuple[int, int, int, int, int, bytes]]" = queue.Queue()
    done = threading.Event()
    counters = {"callback": 0, "saved": 0}

    def on_camera_image(
        eye: int,
        image: C.POINTER(C.c_uint8),
        size: int,
        width: int,
        height: int,
        timestamp: int,
        context: int,
    ) -> None:
        counters["callback"] += 1
        if counters["callback"] <= 5:
            print(
                "camera image callback:",
                f"count={counters['callback']}",
                f"eye={eye}",
                f"size={size}",
                f"width={width}",
                f"height={height}",
                f"timestamp={timestamp}",
                f"ptr={C.cast(image, C.c_void_p).value if image else None}",
                flush=True,
            )
        if done.is_set() or counters["saved"] >= args.max_frames:
            return
        if not image or size <= 0 or width <= 0 or height <= 0:
            return
        if size > 20_000_000 or width * height > 20_000_000:
            return
        data = C.string_at(C.cast(image, C.c_void_p).value, min(size, width * height))
        frames.put((eye, size, width, height, int(timestamp), data))

    cb = CAMERA_IMAGE_CALLBACK(on_camera_image)

    writer_done = threading.Event()

    def writer() -> None:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        while not writer_done.is_set() and counters["saved"] < args.max_frames:
            try:
                eye, size, width, height, timestamp, data = frames.get(timeout=0.2)
            except queue.Empty:
                continue
            name = f"{counters['saved']:04d}_eye{eye}_{width}x{height}_{size}_{timestamp}.pgm"
            write_pgm(
                args.output_dir / name,
                width,
                height,
                data,
                f"eye={eye} size={size} timestamp={timestamp}",
            )
            counters["saved"] += 1
            print(f"saved {name}", flush=True)
        writer_done.set()

    thread = threading.Thread(target=writer, daemon=True)
    thread.start()

    started = False
    try:
        config_bytes = str(args.sdk_config).encode("mbcs")
        ret = dll._7_CALL_init_sdk(config_bytes, 3, 0)
        print(f"_7_CALL_init_sdk -> {ret}", flush=True)
        if ret != SUCCESS:
            return 2

        ret = dll._7_CALL_set_camera_image_callback(C.cast(cb, C.c_void_p), None)
        print(f"_7_CALL_set_camera_image_callback -> {ret}", flush=True)
        if ret != SUCCESS:
            return 3

        ret = dll._7_CALL_start_camera(args.pid1, args.pid2)
        print(f"_7_CALL_start_camera({args.pid1}, {args.pid2}) -> {ret}", flush=True)
        if ret != SUCCESS:
            return 4
        started = True

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and counters["saved"] < args.max_frames:
            time.sleep(0.05)

        print(
            "camera callback counts:",
            f"callbacks={counters['callback']}",
            f"saved={counters['saved']}",
            flush=True,
        )
        return 0
    finally:
        done.set()
        writer_done.set()
        thread.join(timeout=2.0)
        if started:
            try:
                print(f"_7_CALL_stop_camera -> {dll._7_CALL_stop_camera()}", flush=True)
            except Exception as exc:
                print(f"_7_CALL_stop_camera failed: {exc!r}", flush=True)
        try:
            print(f"_7_CALL_release -> {dll._7_CALL_release()}", flush=True)
        except Exception as exc:
            print(f"_7_CALL_release failed: {exc!r}", flush=True)
        _ = cb
        _ = DLL_DIR_HANDLES


if __name__ == "__main__":
    sys.exit(main())
