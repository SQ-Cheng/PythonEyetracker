from __future__ import annotations

import argparse
import csv
import ctypes as C
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from sdk_types import py_7i_eye_data_ex_t


SUCCESS = 0
DLL_DIR_HANDLES = []


class UserSdkCoefficient(C.Structure):
    _fields_ = [("buf", C.c_ubyte * 1024)]


class UserSdkCoefficientWithSize(C.Structure):
    _fields_ = [
        ("buf", C.c_ubyte * 1024),
        ("size", C.c_int),
    ]


class ASeeVrCoefficientBlob(C.Structure):
    _fields_ = [("buf", C.c_ubyte * 2048)]


@dataclass
class ImageFrame:
    seq: int
    pc_perf: float
    eye: int
    size: int
    width: int
    height: int
    timestamp: int
    data: bytes


@dataclass
class GazeSample:
    seq: int
    pc_perf: float
    timestamp: int
    recommend: int
    recom_gaze_x: float
    recom_gaze_y: float
    recom_gaze_z: float
    recom_re: float
    left_gaze_x: float
    left_gaze_y: float
    left_re: float
    right_gaze_x: float
    right_gaze_y: float
    right_re: float
    left_pupil_x: float
    left_pupil_y: float
    left_pupil_diameter: float
    left_pupil_diameter_mm: float
    right_pupil_x: float
    right_pupil_y: float
    right_pupil_diameter: float
    right_pupil_diameter_mm: float
    left_blink: int
    right_blink: int
    left_openness: float
    right_openness: float


def add_dll_dir(path: Path) -> None:
    if path.exists() and hasattr(os, "add_dll_directory"):
        DLL_DIR_HANDLES.append(os.add_dll_directory(str(path)))


def write_pgm(path: Path, width: int, height: int, data: bytes, comment: str) -> None:
    header = f"P5\n# {comment}\n{width} {height}\n255\n".encode("ascii")
    with path.open("wb") as file:
        file.write(header)
        file.write(data)


def load_usersdk(runtime_dir: Path, sdk_config: Path) -> C.WinDLL:
    add_dll_dir(runtime_dir)
    add_dll_dir(sdk_config)
    add_dll_dir(sdk_config / "armeabi")
    dll = C.WinDLL(str(runtime_dir / "USERSDK.dll"))

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

    try:
        dll._7_CALL_set_data.argtypes = [C.c_char_p, C.c_int, C.c_void_p, C.c_int]
        dll._7_CALL_set_data.restype = C.c_int
        dll._7_CALL_get_data.argtypes = [C.c_char_p, C.c_int, C.c_void_p, C.POINTER(C.c_int)]
        dll._7_CALL_get_data.restype = C.c_int
    except AttributeError:
        pass

    # Inferred from wrapper assembly and similar public SDK API:
    # int start_tracking(int eye, const coefficient* coe)
    dll._7_CALL_start_tracking.argtypes = [C.c_int, C.c_void_p]
    dll._7_CALL_start_tracking.restype = C.c_int
    dll._7_CALL_stop_tracking.argtypes = []
    dll._7_CALL_stop_tracking.restype = C.c_int

    return dll


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
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).with_name("usersdk_sync_probe"),
    )
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--max-images", type=int, default=24)
    parser.add_argument("--max-gaze", type=int, default=2000)
    parser.add_argument("--pid1", type=int, default=17178)
    parser.add_argument("--pid2", type=int, default=17156)
    parser.add_argument(
        "--coefficient-bin",
        type=Path,
        help="2048-byte aSeeVR coefficient blob; first 1024 bytes are used for eye 1, second 1024 for eye 2.",
    )
    parser.add_argument(
        "--coefficient-layout",
        choices=("split1024", "split1024-sized", "swap1024", "swap1024-sized", "full2048"),
        default="split1024",
        help="How to pass the 2048-byte aSeeVR coefficient to _7_CALL_start_tracking.",
    )
    parser.add_argument(
        "--tracking-mode",
        choices=("none", "set-data", "start-tracking"),
        default="none",
        help="How to feed coefficients into low-level USERSDK tracking.",
    )
    parser.add_argument(
        "--tracking-eyes",
        choices=("left", "right", "both"),
        default="both",
        help="Which eye(s) to pass to _7_CALL_start_tracking when tracking-mode=start-tracking.",
    )
    parser.add_argument(
        "--start-tracking-null",
        action="store_true",
        help="Also call _7_CALL_start_tracking(1/2, NULL); this is experimental.",
    )
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    frame_dir = args.output_dir / "frames"
    frame_dir.mkdir(parents=True, exist_ok=True)
    image_csv_path = args.output_dir / "image_frames.csv"
    gaze_csv_path = args.output_dir / "gaze_samples.csv"

    dll = load_usersdk(args.runtime_dir, args.sdk_config)
    coefficients: list[C.Structure] = []
    if args.coefficient_bin:
        blob = args.coefficient_bin.read_bytes()
        if len(blob) != 2048:
            raise ValueError(f"Expected 2048-byte coefficient, got {len(blob)} bytes: {args.coefficient_bin}")
        if args.coefficient_layout == "full2048":
            coeff = ASeeVrCoefficientBlob()
            C.memmove(C.byref(coeff), blob, 2048)
            coefficients = [coeff, coeff]
        else:
            offsets = (0, 1024) if args.coefficient_layout.startswith("split1024") else (1024, 0)
            coeff_cls = (
                UserSdkCoefficientWithSize
                if args.coefficient_layout.endswith("-sized")
                else UserSdkCoefficient
            )
            for offset in offsets:
                coeff = coeff_cls()
                C.memmove(C.byref(coeff), blob[offset : offset + 1024], 1024)
                if hasattr(coeff, "size"):
                    coeff.size = 1024
                coefficients.append(coeff)

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
    GAZE_CALLBACK = C.WINFUNCTYPE(
        None,
        C.POINTER(py_7i_eye_data_ex_t),
        C.c_void_p,
    )

    image_queue: "queue.Queue[ImageFrame]" = queue.Queue(maxsize=512)
    gaze_queue: "queue.Queue[GazeSample]" = queue.Queue(maxsize=4096)
    done = threading.Event()
    counters = {
        "image": 0,
        "image_saved": 0,
        "image_dropped": 0,
        "gaze": 0,
        "gaze_dropped": 0,
    }

    def on_image(
        eye: int,
        image: C.POINTER(C.c_uint8),
        size: int,
        width: int,
        height: int,
        timestamp: int,
        context: int,
    ) -> None:
        seq = counters["image"]
        counters["image"] += 1
        pc_perf = time.perf_counter()
        if seq < 6:
            print(
                "image callback:",
                f"seq={seq}",
                f"eye={eye}",
                f"ts={timestamp}",
                f"{width}x{height}",
                f"size={size}",
                flush=True,
            )
        if done.is_set() or counters["image_saved"] >= args.max_images:
            return
        if not image or size <= 0 or width <= 0 or height <= 0:
            return
        if size > 20_000_000 or width * height > 20_000_000:
            return
        data = C.string_at(C.cast(image, C.c_void_p).value, min(size, width * height))
        try:
            image_queue.put_nowait(
                ImageFrame(seq, pc_perf, eye, size, width, height, int(timestamp), data)
            )
        except queue.Full:
            counters["image_dropped"] += 1

    def on_gaze(eyes_ptr: C.POINTER(py_7i_eye_data_ex_t), context: int) -> None:
        if not eyes_ptr:
            return
        seq = counters["gaze"]
        counters["gaze"] += 1
        pc_perf = time.perf_counter()
        eyes = eyes_ptr.contents
        sample = GazeSample(
            seq=seq,
            pc_perf=pc_perf,
            timestamp=int(eyes.timestamp),
            recommend=int(eyes.recommend),
            recom_gaze_x=float(eyes.recom_gaze.gaze_point.x),
            recom_gaze_y=float(eyes.recom_gaze.gaze_point.y),
            recom_gaze_z=float(eyes.recom_gaze.gaze_point.z),
            recom_re=float(eyes.recom_gaze.re),
            left_gaze_x=float(eyes.left_gaze.gaze_point.x),
            left_gaze_y=float(eyes.left_gaze.gaze_point.y),
            left_re=float(eyes.left_gaze.re),
            right_gaze_x=float(eyes.right_gaze.gaze_point.x),
            right_gaze_y=float(eyes.right_gaze.gaze_point.y),
            right_re=float(eyes.right_gaze.re),
            left_pupil_x=float(eyes.left_pupil.pupil_center.x),
            left_pupil_y=float(eyes.left_pupil.pupil_center.y),
            left_pupil_diameter=float(eyes.left_pupil.pupil_diameter),
            left_pupil_diameter_mm=float(eyes.left_pupil.pupil_diameter_mm),
            right_pupil_x=float(eyes.right_pupil.pupil_center.x),
            right_pupil_y=float(eyes.right_pupil.pupil_center.y),
            right_pupil_diameter=float(eyes.right_pupil.pupil_diameter),
            right_pupil_diameter_mm=float(eyes.right_pupil.pupil_diameter_mm),
            left_blink=int(eyes.left_ex_data.blink),
            right_blink=int(eyes.right_ex_data.blink),
            left_openness=float(eyes.left_ex_data.openness),
            right_openness=float(eyes.right_ex_data.openness),
        )
        if seq < 6:
            print(
                "gaze callback:",
                f"seq={seq}",
                f"ts={sample.timestamp}",
                f"recommend={sample.recommend}",
                f"gaze=({sample.recom_gaze_x:.3f},{sample.recom_gaze_y:.3f})",
                f"pupilL=({sample.left_pupil_x:.3f},{sample.left_pupil_y:.3f})",
                flush=True,
            )
        try:
            gaze_queue.put_nowait(sample)
        except queue.Full:
            counters["gaze_dropped"] += 1

    image_cb = CAMERA_IMAGE_CALLBACK(on_image)
    gaze_cb = GAZE_CALLBACK(on_gaze)

    def writer() -> None:
        with image_csv_path.open("w", newline="", encoding="utf-8") as image_file, gaze_csv_path.open(
            "w", newline="", encoding="utf-8"
        ) as gaze_file:
            image_writer = csv.writer(image_file)
            gaze_writer = csv.writer(gaze_file)
            image_writer.writerow(
                [
                    "seq",
                    "pc_perf",
                    "eye",
                    "device_timestamp",
                    "width",
                    "height",
                    "size",
                    "frame_path",
                ]
            )
            gaze_writer.writerow([field.name for field in GazeSample.__dataclass_fields__.values()])

            while not done.is_set() or not image_queue.empty() or not gaze_queue.empty():
                try:
                    frame = image_queue.get(timeout=0.05)
                    filename = (
                        f"{frame.seq:06d}_eye{frame.eye}_"
                        f"{frame.width}x{frame.height}_{frame.timestamp}.pgm"
                    )
                    write_pgm(
                        frame_dir / filename,
                        frame.width,
                        frame.height,
                        frame.data,
                        f"seq={frame.seq} eye={frame.eye} timestamp={frame.timestamp} pc_perf={frame.pc_perf:.9f}",
                    )
                    counters["image_saved"] += 1
                    image_writer.writerow(
                        [
                            frame.seq,
                            f"{frame.pc_perf:.9f}",
                            frame.eye,
                            frame.timestamp,
                            frame.width,
                            frame.height,
                            frame.size,
                            str(frame_dir / filename),
                        ]
                    )
                except queue.Empty:
                    pass

                while True:
                    try:
                        sample = gaze_queue.get_nowait()
                    except queue.Empty:
                        break
                    gaze_writer.writerow(
                        [
                            sample.seq,
                            f"{sample.pc_perf:.9f}",
                            sample.timestamp,
                            sample.recommend,
                            f"{sample.recom_gaze_x:.9f}",
                            f"{sample.recom_gaze_y:.9f}",
                            f"{sample.recom_gaze_z:.9f}",
                            f"{sample.recom_re:.9f}",
                            f"{sample.left_gaze_x:.9f}",
                            f"{sample.left_gaze_y:.9f}",
                            f"{sample.left_re:.9f}",
                            f"{sample.right_gaze_x:.9f}",
                            f"{sample.right_gaze_y:.9f}",
                            f"{sample.right_re:.9f}",
                            f"{sample.left_pupil_x:.9f}",
                            f"{sample.left_pupil_y:.9f}",
                            f"{sample.left_pupil_diameter:.9f}",
                            f"{sample.left_pupil_diameter_mm:.9f}",
                            f"{sample.right_pupil_x:.9f}",
                            f"{sample.right_pupil_y:.9f}",
                            f"{sample.right_pupil_diameter:.9f}",
                            f"{sample.right_pupil_diameter_mm:.9f}",
                            sample.left_blink,
                            sample.right_blink,
                            f"{sample.left_openness:.9f}",
                            f"{sample.right_openness:.9f}",
                        ]
                    )
                image_file.flush()
                gaze_file.flush()

    writer_thread = threading.Thread(target=writer, daemon=True)
    writer_thread.start()

    started_camera = False
    started_gaze = False
    started_tracking = False
    try:
        config_bytes = str(args.sdk_config).encode("mbcs")
        ret = dll._7_CALL_init_sdk(config_bytes, 3, 0)
        print(f"_7_CALL_init_sdk -> {ret}", flush=True)
        if ret != SUCCESS:
            return 2

        ret = dll._7_CALL_set_camera_image_callback(C.cast(image_cb, C.c_void_p), None)
        print(f"_7_CALL_set_camera_image_callback -> {ret}", flush=True)
        if ret != SUCCESS:
            return 3

        ret = dll._7_CALL_start_camera(args.pid1, args.pid2)
        print(f"_7_CALL_start_camera({args.pid1}, {args.pid2}) -> {ret}", flush=True)
        if ret != SUCCESS:
            return 4
        started_camera = True

        ret = dll._7_CALL_start_eye_gaze_callback_ex(C.cast(gaze_cb, C.c_void_p), None)
        print(f"_7_CALL_start_eye_gaze_callback_ex -> {ret}", flush=True)
        if ret == SUCCESS:
            started_gaze = True

        if args.tracking_mode == "set-data":
            if len(coefficients) < 2:
                raise ValueError("--tracking-mode set-data requires --coefficient-bin")
            if not hasattr(dll, "_7_CALL_set_data"):
                raise RuntimeError("This USERSDK.dll does not export _7_CALL_set_data/_7_CALL_get_data.")
            for flag, coeff in ((b"biLeft", coefficients[0]), (b"biRight", coefficients[1])):
                ret = dll._7_CALL_set_data(flag, len(flag), C.byref(coeff), C.sizeof(coeff))
                print(f"_7_CALL_set_data({flag.decode('ascii')}, {C.sizeof(coeff)}) -> {ret}", flush=True)
            verify_left = UserSdkCoefficient()
            verify_right = UserSdkCoefficient()
            for flag, coeff in ((b"biLeft", verify_left), (b"biRight", verify_right)):
                size = C.c_int(C.sizeof(coeff))
                ret = dll._7_CALL_get_data(flag, len(flag), C.byref(coeff), C.byref(size))
                print(f"_7_CALL_get_data({flag.decode('ascii')}) -> {ret}, size={size.value}", flush=True)
            started_tracking = True
        elif args.tracking_mode == "start-tracking" and coefficients:
            eye_coeffs = []
            if args.tracking_eyes in ("left", "both"):
                eye_coeffs.append((1, coefficients[0]))
            if args.tracking_eyes in ("right", "both"):
                eye_coeffs.append((2, coefficients[1]))
            for eye, coeff in eye_coeffs:
                print(
                    f"calling _7_CALL_start_tracking({eye}, {args.coefficient_layout}[{eye - 1}])",
                    flush=True,
                )
                ret = dll._7_CALL_start_tracking(eye, C.byref(coeff))
                print(
                    f"_7_CALL_start_tracking({eye}, {args.coefficient_layout}[{eye - 1}]) -> {ret}",
                    flush=True,
                )
            started_tracking = True
        elif args.tracking_mode == "start-tracking" and args.start_tracking_null:
            for eye in (1, 2):
                ret = dll._7_CALL_start_tracking(eye, None)
                print(f"_7_CALL_start_tracking({eye}, NULL) -> {ret}", flush=True)
            started_tracking = True

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            if counters["image_saved"] >= args.max_images and counters["gaze"] >= args.max_gaze:
                break
            time.sleep(0.05)

        print(
            "sync callback counts:",
            f"images={counters['image']}",
            f"saved_images={counters['image_saved']}",
            f"image_dropped={counters['image_dropped']}",
            f"gaze={counters['gaze']}",
            f"gaze_dropped={counters['gaze_dropped']}",
            flush=True,
        )
        print(f"output_dir={args.output_dir}", flush=True)
        return 0
    finally:
        done.set()
        writer_thread.join(timeout=5.0)
        if started_tracking:
            try:
                print(f"_7_CALL_stop_tracking -> {dll._7_CALL_stop_tracking()}", flush=True)
            except Exception as exc:
                print(f"_7_CALL_stop_tracking failed: {exc!r}", flush=True)
        if started_gaze:
            try:
                print(f"_7_CALL_stop_eye_callback -> {dll._7_CALL_stop_eye_callback()}", flush=True)
            except Exception as exc:
                print(f"_7_CALL_stop_eye_callback failed: {exc!r}", flush=True)
        if started_camera:
            try:
                print(f"_7_CALL_stop_camera -> {dll._7_CALL_stop_camera()}", flush=True)
            except Exception as exc:
                print(f"_7_CALL_stop_camera failed: {exc!r}", flush=True)
        try:
            print(f"_7_CALL_release -> {dll._7_CALL_release()}", flush=True)
        except Exception as exc:
            print(f"_7_CALL_release failed: {exc!r}", flush=True)
        _ = (image_cb, gaze_cb, DLL_DIR_HANDLES)


if __name__ == "__main__":
    sys.exit(main())
