from __future__ import annotations

import argparse
import bisect
import ctypes as C
import csv
import json
import os
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


RUNTIME_DIR = Path(r"C:\7invensun\aSeeVR_UserSDK\runtime")
SDK_CONFIG = Path(r"C:\7invensun\aSeeVR_UserSDK\runtime\devices\Droolon\config")
DLL_DIR_HANDLES = []


@dataclass
class SdkFrame:
    seq: int
    pc_perf: float
    eye: int
    timestamp: int
    width: int
    height: int
    data: np.ndarray


@dataclass
class CvFrame:
    seq: int
    pc_perf: float
    api: str
    index: int
    width: int
    height: int
    gray: np.ndarray


class StdoutRedirect:
    def __init__(self, log_path: Path) -> None:
        self.log_path = log_path
        self.original_fd: int | None = None
        self.log_fd: int | None = None

    def __enter__(self) -> "StdoutRedirect":
        sys.stdout.flush()
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.original_fd = os.dup(1)
        self.log_fd = os.open(str(self.log_path), os.O_CREAT | os.O_TRUNC | os.O_WRONLY)
        os.dup2(self.log_fd, 1)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        sys.stdout.flush()
        if self.original_fd is not None:
            os.dup2(self.original_fd, 1)
            os.close(self.original_fd)
            self.original_fd = None
        if self.log_fd is not None:
            os.close(self.log_fd)
            self.log_fd = None

    def say(self, text: str) -> None:
        if self.original_fd is None:
            print(text, flush=True)
            return
        os.write(self.original_fd, (text + "\n").encode("utf-8", errors="replace"))


def add_dll_dir(path: Path) -> None:
    if path.exists() and hasattr(os, "add_dll_directory"):
        DLL_DIR_HANDLES.append(os.add_dll_directory(str(path)))


def load_dll(runtime_dir: Path, sdk_config: Path) -> C.WinDLL:
    os.environ["PATH"] = (
        str(runtime_dir)
        + os.pathsep
        + str(sdk_config)
        + os.pathsep
        + str(sdk_config / "armeabi")
        + os.pathsep
        + os.environ.get("PATH", "")
    )
    add_dll_dir(runtime_dir)
    add_dll_dir(sdk_config)
    add_dll_dir(sdk_config / "armeabi")
    dll = C.WinDLL(str(runtime_dir / "USERSDK.dll"))
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


def parse_indices(text: str) -> list[int]:
    return [int(item.strip()) for item in text.split(",") if item.strip()]


def make_cap(api: str, index: int, width: int, height: int, fps: int) -> cv2.VideoCapture:
    api_id = {"DSHOW": cv2.CAP_DSHOW, "MSMF": cv2.CAP_MSMF}[api]
    cap = cv2.VideoCapture(index, api_id)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def capture_cv2(
    api: str,
    index: int,
    args: argparse.Namespace,
    done: threading.Event,
    frames: list[CvFrame],
    status: dict,
) -> None:
    cap = make_cap(api, index, args.cv_width, args.cv_height, args.cv_fps)
    status.update(
        {
            "api": api,
            "index": index,
            "opened": bool(cap.isOpened()),
            "width": float(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) if cap.isOpened() else 0.0,
            "height": float(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) if cap.isOpened() else 0.0,
            "fps": float(cap.get(cv2.CAP_PROP_FPS)) if cap.isOpened() else 0.0,
            "captured": 0,
        }
    )
    seq = 0
    try:
        while not done.is_set() and seq < args.max_cv_per_index and cap.isOpened():
            ok, frame = cap.read()
            pc = time.perf_counter()
            if not ok or frame is None:
                time.sleep(0.002)
                continue
            if frame.ndim == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame.copy()
            frames.append(CvFrame(seq, pc, api, index, gray.shape[1], gray.shape[0], gray))
            seq += 1
            status["captured"] = seq
    finally:
        cap.release()


def transform_image(img: np.ndarray, name: str) -> np.ndarray:
    if name == "none":
        return img
    if name == "flip_x":
        return cv2.flip(img, 1)
    if name == "flip_y":
        return cv2.flip(img, 0)
    if name == "rot180":
        return cv2.flip(img, -1)
    raise ValueError(name)


def corr_score(a: np.ndarray, b: np.ndarray) -> tuple[float, bool]:
    af = a.astype(np.float32).reshape(-1)
    bf = b.astype(np.float32).reshape(-1)
    af -= float(af.mean())
    bf -= float(bf.mean())
    denom = float(np.linalg.norm(af) * np.linalg.norm(bf))
    if denom < 1e-6:
        return 0.0, False
    corr = float(np.dot(af, bf) / denom)
    if -corr > corr:
        return -corr, True
    return corr, False


def compare_pair(sdk: SdkFrame, cvf: CvFrame) -> tuple[float, str, bool]:
    resized = cv2.resize(cvf.gray, (sdk.width, sdk.height), interpolation=cv2.INTER_AREA)
    best = (-1.0, "none", False)
    for transform in ("none", "flip_x", "flip_y", "rot180"):
        score, inverted = corr_score(sdk.data, transform_image(resized, transform))
        if score > best[0]:
            best = (score, transform, inverted)
    return best


def analyze_matches(
    sdk_frames: list[SdkFrame],
    cv_frames: list[CvFrame],
    output_dir: Path,
    max_dt: float,
) -> list[dict]:
    by_source: dict[tuple[str, int], list[CvFrame]] = defaultdict(list)
    for frame in cv_frames:
        by_source[(frame.api, frame.index)].append(frame)
    for frames in by_source.values():
        frames.sort(key=lambda item: item.pc_perf)

    summary_rows = []
    montage_items = []

    for (api, index), frames in sorted(by_source.items()):
        times = [frame.pc_perf for frame in frames]
        for eye in (1, 2):
            candidates = [frame for frame in sdk_frames if frame.eye == eye]
            if len(candidates) > 240:
                step = max(1, len(candidates) // 240)
                candidates = candidates[::step]

            best_by_sdk = []
            for sdk in candidates:
                pos = bisect.bisect_left(times, sdk.pc_perf)
                near = []
                for j in range(max(0, pos - 3), min(len(frames), pos + 4)):
                    dt = frames[j].pc_perf - sdk.pc_perf
                    if abs(dt) <= max_dt:
                        near.append((abs(dt), dt, frames[j]))
                if not near:
                    continue
                best_local = (-1.0, None, 0.0, "none", False)
                for _abs_dt, dt, cvf in near:
                    score, transform, inverted = compare_pair(sdk, cvf)
                    if score > best_local[0]:
                        best_local = (score, cvf, dt, transform, inverted)
                if best_local[1] is not None:
                    best_by_sdk.append((sdk, *best_local))

            scores = [item[1] for item in best_by_sdk]
            dts = [item[3] for item in best_by_sdk]
            best_item = max(best_by_sdk, key=lambda item: item[1], default=None)
            row = {
                "api": api,
                "index": index,
                "sdk_eye": eye,
                "sdk_samples_compared": len(candidates),
                "matched_within_window": len(best_by_sdk),
                "median_corr": float(np.median(scores)) if scores else 0.0,
                "p90_corr": float(np.percentile(scores, 90)) if scores else 0.0,
                "best_corr": float(best_item[1]) if best_item else 0.0,
                "best_dt_ms": float(best_item[3] * 1000.0) if best_item else 0.0,
                "median_abs_dt_ms": float(np.median(np.abs(dts)) * 1000.0) if dts else 0.0,
                "best_transform": best_item[4] if best_item else "",
                "best_inverted": bool(best_item[5]) if best_item else False,
            }
            summary_rows.append(row)
            if best_item is not None:
                montage_items.append((row, best_item))

    montage_items.sort(key=lambda item: item[0]["best_corr"], reverse=True)
    save_montage(output_dir / "best_frame_matches.png", montage_items[:8])
    return summary_rows


def save_montage(path: Path, items: list[tuple[dict, tuple]]) -> None:
    if not items:
        return
    tile_w, tile_h = 640, 300
    canvas = np.full((tile_h * len(items), tile_w, 3), 245, dtype=np.uint8)
    for row_idx, (row, item) in enumerate(items):
        sdk, score, cvf, dt, transform, inverted = item
        sdk_img = cv2.resize(sdk.data, (240, 180), interpolation=cv2.INTER_NEAREST)
        cv_small = cv2.resize(cvf.gray, (sdk.width, sdk.height), interpolation=cv2.INTER_AREA)
        cv_small = transform_image(cv_small, transform)
        if inverted:
            cv_small = 255 - cv_small
        cv_img = cv2.resize(cv_small, (240, 180), interpolation=cv2.INTER_NEAREST)
        sdk_bgr = cv2.cvtColor(sdk_img, cv2.COLOR_GRAY2BGR)
        cv_bgr = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        y0 = row_idx * tile_h
        canvas[y0 + 70 : y0 + 250, 20:260] = sdk_bgr
        canvas[y0 + 70 : y0 + 250, 300:540] = cv_bgr
        label = (
            f"{row['api']}[{row['index']}] eye={row['sdk_eye']} "
            f"corr={score:.3f} dt={dt*1000:.2f}ms {transform}"
            + (" inverted" if inverted else "")
        )
        cv2.putText(canvas, label, (20, y0 + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (20, 30, 40), 1)
        cv2.putText(canvas, "SDK 80x60", (20, y0 + 270), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (70, 80, 90), 1)
        cv2.putText(canvas, "OpenCV 640x480 -> 80x60", (300, y0 + 270), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (70, 80, 90), 1)
    cv2.imwrite(str(path), canvas)


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime-dir", type=Path, default=RUNTIME_DIR)
    parser.add_argument("--sdk-config", type=Path, default=SDK_CONFIG)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--api", choices=("DSHOW", "MSMF"), default="DSHOW")
    parser.add_argument("--indices", default="0,1,2,3")
    parser.add_argument("--cv-width", type=int, default=640)
    parser.add_argument("--cv-height", type=int, default=480)
    parser.add_argument("--cv-fps", type=int, default=120)
    parser.add_argument("--max-sdk-frames", type=int, default=1200)
    parser.add_argument("--max-cv-per-index", type=int, default=240)
    parser.add_argument("--match-window-ms", type=float, default=40.0)
    parser.add_argument("--sdk-warmup-frames", type=int, default=40)
    parser.add_argument("--sdk-warmup-timeout", type=float, default=2.0)
    parser.add_argument("--pid1", type=int, default=17178)
    parser.add_argument("--pid2", type=int, default=17156)
    args = parser.parse_args()

    if args.output_dir is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output_dir = Path(__file__).with_name(f"cv2_sdk_match_{stamp}")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    sdk_frames: list[SdkFrame] = []
    cv_frames: list[CvFrame] = []
    cv_status: list[dict] = []
    sdk_lock = threading.Lock()
    done = threading.Event()
    counters = {"sdk": 0}
    sdk_count_before_cv = 0

    IMAGE_CB = C.WINFUNCTYPE(
        None,
        C.c_int,
        C.POINTER(C.c_uint8),
        C.c_int,
        C.c_int,
        C.c_int,
        C.c_longlong,
        C.c_void_p,
    )

    def on_image(eye, data_ptr, size, width, height, timestamp, context) -> None:
        seq = counters["sdk"]
        counters["sdk"] += 1
        if done.is_set() or len(sdk_frames) >= args.max_sdk_frames:
            return
        if not data_ptr or size <= 0 or width <= 0 or height <= 0:
            return
        nbytes = min(int(size), int(width) * int(height))
        raw = C.string_at(C.cast(data_ptr, C.c_void_p).value, nbytes)
        arr = np.frombuffer(raw, dtype=np.uint8).copy().reshape((int(height), int(width)))
        with sdk_lock:
            sdk_frames.append(SdkFrame(seq, time.perf_counter(), int(eye), int(timestamp), int(width), int(height), arr))

    cb = IMAGE_CB(on_image)

    started = False
    dll = None
    stdout_log = args.output_dir / "sdk_stdout.log"
    with StdoutRedirect(stdout_log) as out:
        out.say(f"Output directory: {args.output_dir}")
        out.say(f"SDK stdout is redirected to: {stdout_log}")
        try:
            dll = load_dll(args.runtime_dir, args.sdk_config)
            ret = dll._7_CALL_init_sdk(str(args.sdk_config).encode("mbcs"), 3, 0)
            out.say(f"_7_CALL_init_sdk -> {ret}")
            if ret != 0:
                return 2
            ret = dll._7_CALL_set_camera_image_callback(C.cast(cb, C.c_void_p), None)
            out.say(f"_7_CALL_set_camera_image_callback -> {ret}")
            if ret != 0:
                return 3
            ret = dll._7_CALL_start_camera(args.pid1, args.pid2)
            out.say(f"_7_CALL_start_camera({args.pid1}, {args.pid2}) -> {ret}")
            if ret != 0:
                return 4
            started = True

            warmup_deadline = time.monotonic() + args.sdk_warmup_timeout
            while len(sdk_frames) < args.sdk_warmup_frames and time.monotonic() < warmup_deadline:
                time.sleep(0.02)
            sdk_count_before_cv = len(sdk_frames)
            out.say(f"sdk frames before opening cv2 -> {sdk_count_before_cv}")
            threads = []
            for index in parse_indices(args.indices):
                status: dict = {}
                cv_status.append(status)
                thread = threading.Thread(
                    target=capture_cv2,
                    args=(args.api, index, args, done, cv_frames, status),
                    daemon=True,
                )
                threads.append(thread)
                thread.start()

            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                time.sleep(0.05)
            done.set()
            for thread in threads:
                thread.join(timeout=2.0)
        finally:
            done.set()
            if started and dll is not None:
                try:
                    out.say(f"_7_CALL_stop_camera -> {dll._7_CALL_stop_camera()}")
                except Exception as exc:
                    out.say(f"_7_CALL_stop_camera failed: {exc!r}")
            if dll is not None:
                try:
                    out.say(f"_7_CALL_release -> {dll._7_CALL_release()}")
                except Exception as exc:
                    out.say(f"_7_CALL_release failed: {exc!r}")
            _ = cb

    match_rows = analyze_matches(
        sdk_frames,
        cv_frames,
        args.output_dir,
        max_dt=args.match_window_ms / 1000.0,
    )
    write_csv(args.output_dir / "cv_status.csv", cv_status)
    write_csv(args.output_dir / "match_summary.csv", match_rows)

    summary = {
        "output_dir": str(args.output_dir),
        "sdk_frames": len(sdk_frames),
        "sdk_frame_shapes": sorted({f"{frame.width}x{frame.height}" for frame in sdk_frames}),
        "cv_frames": len(cv_frames),
        "sdk_count_before_cv": sdk_count_before_cv,
        "cv_status": cv_status,
        "best_matches": sorted(match_rows, key=lambda row: row["best_corr"], reverse=True)[:8],
        "stdout_log": str(stdout_log),
    }
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(json.dumps(summary, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
