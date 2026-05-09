#!/usr/bin/env python3
"""
Matplotlib sync-capture reviewer.

The review phase uses only PC receive timestamps. The first 10 seconds are
discarded, then rising-edge feet are detected and matched on the shared PC
perf-counter timeline.
"""

import argparse
import csv
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, CheckButtons, Slider

from example_paths import LOG_DIR


DISCARD_INITIAL_SECONDS = 10.0
PAIR_MAX_GAP_SECONDS = 0.5
EDGE_REFRACTORY_SECONDS = 0.4
EDGE_FOOT_LOOKBACK_SECONDS = 0.10
EDGE_FOOT_THRESHOLD_MULTIPLIER = 1.0
DEFAULT_THRESHOLD_MULTIPLIER = 3.0


def read_csv_rows(path):
    with open(path, "r", newline="") as fp:
        return list(csv.DictReader(fp))


def find_newest_capture_dir():
    if not os.path.isdir(LOG_DIR):
        return None

    newest_path = None
    newest_mtime = None
    for entry in os.scandir(LOG_DIR):
        if not entry.is_dir() or not entry.name.startswith("sync_capture_"):
            continue
        try:
            mtime = entry.stat().st_mtime
        except OSError:
            continue
        if newest_mtime is None or mtime > newest_mtime:
            newest_mtime = mtime
            newest_path = entry.path
    return newest_path


def load_capture(capture_dir):
    sensor_rows = read_csv_rows(os.path.join(capture_dir, "sensor_ppg.csv"))
    eye_rows = read_csv_rows(os.path.join(capture_dir, "right_eye_brightness.csv"))
    return trim_initial_seconds(sensor_rows, eye_rows, DISCARD_INITIAL_SECONDS)


def trim_initial_seconds(sensor_rows, eye_rows, seconds):
    first_times = []
    if sensor_rows:
        first_times.append(float(sensor_rows[0]["sensor_pc_timestamp"]))
    if eye_rows:
        first_times.append(float(eye_rows[0]["eye_pc_timestamp"]))
    if not first_times:
        return sensor_rows, eye_rows, 0.0

    cutoff = min(first_times) + seconds
    sensor_rows = [row for row in sensor_rows if float(row["sensor_pc_timestamp"]) >= cutoff]
    eye_rows = [row for row in eye_rows if float(row["eye_pc_timestamp"]) >= cutoff]
    return sensor_rows, eye_rows, cutoff


def moving_average(values, window_samples):
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return values

    window_samples = max(3, int(window_samples))
    if window_samples % 2 == 0:
        window_samples += 1
    if window_samples >= values.size:
        return np.full_like(values, np.mean(values), dtype=np.float64)

    pad = window_samples // 2
    kernel = np.ones(window_samples, dtype=np.float64) / float(window_samples)
    padded = np.pad(values, (pad, pad), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def build_centered_signal(times, values, baseline_window_seconds):
    times = np.asarray(times, dtype=np.float64)
    values = np.asarray(values, dtype=np.float64)
    if times.size == 0:
        empty = np.array([], dtype=np.float64)
        return empty, empty, empty, 1e-6

    sample_dt = np.median(np.diff(times)) if times.size >= 2 else 0.01
    baseline_samples = max(5, int(round(baseline_window_seconds / max(sample_dt, 1e-6))))
    baseline = moving_average(values, baseline_samples)
    centered = values - baseline

    median = float(np.median(centered))
    mad = float(np.median(np.abs(centered - median)))
    sigma = max(1.4826 * mad, 1e-6)
    strength = np.maximum(0.0, centered / sigma)
    return centered, baseline, strength, sigma


def refine_edge_foot_index(times, centered, strict_index, sigma):
    if strict_index <= 0:
        return strict_index

    foot_threshold = EDGE_FOOT_THRESHOLD_MULTIPLIER * sigma
    lookback_start = times[strict_index] - EDGE_FOOT_LOOKBACK_SECONDS
    foot_index = strict_index
    while foot_index > 0 and times[foot_index - 1] >= lookback_start:
        if centered[foot_index - 1] <= foot_threshold:
            break
        foot_index -= 1
    return foot_index


def detect_edges(times, values, baseline_window_seconds, threshold_multiplier):
    centered, baseline, strength, sigma = build_centered_signal(
        times,
        values,
        baseline_window_seconds,
    )
    threshold = threshold_multiplier * sigma
    events = []
    index = 0

    while index < len(times):
        if centered[index] <= threshold:
            index += 1
            continue

        strict_index = index
        foot_index = refine_edge_foot_index(times, centered, strict_index, sigma)
        events.append({
            "index": foot_index,
            "strict_index": strict_index,
            "time": float(times[foot_index]),
            "strength": float(strength[strict_index]),
            "foot_strength": float(strength[foot_index]),
        })

        skip_until = times[foot_index] + EDGE_REFRACTORY_SECONDS
        index = strict_index + 1
        while index < len(times) and times[index] < skip_until:
            index += 1

    return events, {
        "centered": centered,
        "baseline": baseline,
        "strength": strength,
        "sigma": sigma,
        "threshold": threshold,
    }


def load_sensor_trace(sensor_rows):
    return {
        "times": np.array([float(row["sensor_pc_timestamp"]) for row in sensor_rows], dtype=np.float64),
        "red": np.array([float(row["red"]) for row in sensor_rows], dtype=np.float64),
        "ir": np.array([float(row["ir"]) for row in sensor_rows], dtype=np.float64),
        "green": np.array([float(row["green"]) for row in sensor_rows], dtype=np.float64),
    }


def load_eye_trace(eye_rows):
    return {
        "times": np.array([float(row["eye_pc_timestamp"]) for row in eye_rows], dtype=np.float64),
        "brightness": np.array([float(row["mean_brightness"]) for row in eye_rows], dtype=np.float64),
    }


def detect_ppg_events(sensor_trace, threshold_multiplier):
    events, analysis = detect_edges(
        sensor_trace["times"],
        sensor_trace["green"],
        baseline_window_seconds=0.4,
        threshold_multiplier=threshold_multiplier,
    )
    return events, analysis


def detect_eye_events(eye_trace, threshold_multiplier):
    events, analysis = detect_edges(
        eye_trace["times"],
        eye_trace["brightness"],
        baseline_window_seconds=0.25,
        threshold_multiplier=threshold_multiplier,
    )
    return events, analysis


def match_events(ppg_events, eye_events):
    matches = []
    used_ppg = set()
    for eye_index, eye_event in enumerate(eye_events):
        best_ppg_index = None
        best_gap = None
        for ppg_index, ppg_event in enumerate(ppg_events):
            if ppg_index in used_ppg:
                continue
            gap = abs(ppg_event["time"] - eye_event["time"])
            if gap > PAIR_MAX_GAP_SECONDS:
                continue
            if best_gap is None or gap < best_gap:
                best_gap = gap
                best_ppg_index = ppg_index
        if best_ppg_index is None:
            continue

        ppg_event = ppg_events[best_ppg_index]
        used_ppg.add(best_ppg_index)
        matches.append({
            "selected": True,
            "eye_index": eye_index,
            "ppg_index": best_ppg_index,
            "eye_time": eye_event["time"],
            "ppg_time": ppg_event["time"],
            "delta_ms": (ppg_event["time"] - eye_event["time"]) * 1000.0,
            "eye_strength": eye_event["strength"],
            "ppg_strength": ppg_event["strength"],
        })
    return matches


def normalize(values):
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return values
    lo = float(np.percentile(values, 2))
    hi = float(np.percentile(values, 98))
    if hi <= lo:
        return np.zeros_like(values)
    return np.clip((values - lo) / (hi - lo), 0.0, 1.0)


class SyncCaptureReviewer:
    def __init__(self, capture_dir, sensor_rows, eye_rows, cutoff_time):
        self.capture_dir = capture_dir
        self.cutoff_time = cutoff_time
        self.sensor_trace = load_sensor_trace(sensor_rows)
        self.eye_trace = load_eye_trace(eye_rows)
        self.threshold = DEFAULT_THRESHOLD_MULTIPLIER

        self.ppg_events = []
        self.eye_events = []
        self.matches = []
        self.ppg_analysis = {}
        self.eye_analysis = {}

        self.fig = None
        self.axes = None
        self.check_axis = None
        self.check_buttons = None
        self.threshold_slider = None
        self.reset_button = None
        self.summary_text = None
        self.match_artists = []

    def show(self):
        self.fig, self.axes = plt.subplots(3, 1, figsize=(15, 9), sharex=True)
        plt.subplots_adjust(left=0.07, right=0.75, top=0.91, bottom=0.16, hspace=0.20)

        slider_axis = self.fig.add_axes([0.18, 0.065, 0.43, 0.03])
        reset_axis = self.fig.add_axes([0.63, 0.055, 0.10, 0.055])
        self.check_axis = self.fig.add_axes([0.77, 0.12, 0.21, 0.73])

        self.threshold_slider = Slider(
            slider_axis,
            "Noise x",
            1.0,
            8.0,
            valinit=self.threshold,
            valstep=0.1,
        )
        self.reset_button = Button(reset_axis, "Select all")
        self.summary_text = self.fig.text(0.07, 0.965, "", fontsize=10, va="top")

        self.threshold_slider.on_changed(self._on_threshold_changed)
        self.reset_button.on_clicked(self._on_select_all)
        self.fig.canvas.mpl_connect("button_press_event", self._on_plot_click)

        self._recompute()
        plt.show(block=True)

    def _on_threshold_changed(self, value):
        self.threshold = float(value)
        self._recompute()

    def _on_select_all(self, _event):
        for match in self.matches:
            match["selected"] = True
        self._refresh(rebuild_checks=True)

    def _on_check_clicked(self, label):
        try:
            index = int(label.split()[0])
        except (ValueError, IndexError):
            return
        if 0 <= index < len(self.matches):
            self.matches[index]["selected"] = not self.matches[index]["selected"]
            self._refresh(rebuild_checks=False)

    def _on_plot_click(self, event):
        if event.inaxes not in self.axes or event.xdata is None or not self.matches:
            return

        base_time = self._base_time()
        candidates = []
        for index, match in enumerate(self.matches):
            candidates.append((abs(event.xdata - (match["ppg_time"] - base_time)), index))
            candidates.append((abs(event.xdata - (match["eye_time"] - base_time)), index))
        distance, match_index = min(candidates, key=lambda item: item[0])
        x_min, x_max = event.inaxes.get_xlim()
        tolerance = max((x_max - x_min) * 0.01, 0.05)
        if distance <= tolerance:
            self.matches[match_index]["selected"] = not self.matches[match_index]["selected"]
            self._refresh(rebuild_checks=True)

    def _recompute(self):
        self.ppg_events, self.ppg_analysis = detect_ppg_events(self.sensor_trace, self.threshold)
        self.eye_events, self.eye_analysis = detect_eye_events(self.eye_trace, self.threshold)
        self.matches = match_events(self.ppg_events, self.eye_events)
        self._refresh(rebuild_checks=True)

    def _refresh(self, rebuild_checks):
        for artist in self.match_artists:
            try:
                artist.remove()
            except ValueError:
                pass
        self.match_artists = []

        for axis in self.axes:
            axis.clear()
            axis.grid(True, alpha=0.25)

        self._draw_traces()
        self._draw_matches()
        self._draw_summary()
        if rebuild_checks:
            self._draw_match_selector()
        self.fig.canvas.draw_idle()

    def _base_time(self):
        candidates = []
        if self.sensor_trace["times"].size:
            candidates.append(float(self.sensor_trace["times"][0]))
        if self.eye_trace["times"].size:
            candidates.append(float(self.eye_trace["times"][0]))
        return min(candidates) if candidates else self.cutoff_time

    def _draw_traces(self):
        base_time = self._base_time()
        sensor_t = self.sensor_trace["times"] - base_time
        eye_t = self.eye_trace["times"] - base_time
        ppg_axis, eye_axis, strength_axis = self.axes

        if sensor_t.size:
            green_norm = normalize(self.sensor_trace["green"])
            ppg_axis.plot(sensor_t, normalize(self.sensor_trace["red"]), color="tab:red", lw=0.9, label="Red")
            ppg_axis.plot(sensor_t, normalize(self.sensor_trace["ir"]), color="tab:orange", lw=0.9, label="IR")
            ppg_axis.plot(sensor_t, green_norm, color="tab:green", lw=1.1, label="Green")
            strength_axis.plot(sensor_t, self.ppg_analysis["strength"], color="tab:green", lw=1.0, label="PPG green")
            ppg_axis.plot(
                [event["time"] - base_time for event in self.ppg_events],
                [green_norm[event["index"]] for event in self.ppg_events],
                "o",
                color="tab:green",
                ms=4,
                alpha=0.8,
            )

        if eye_t.size:
            eye_axis.plot(eye_t, self.eye_trace["brightness"], color="black", lw=0.9, label="Brightness")
            strength_axis.plot(eye_t, self.eye_analysis["strength"], color="tab:blue", lw=1.0, label="Right eye")
            eye_axis.plot(
                [event["time"] - base_time for event in self.eye_events],
                [self.eye_trace["brightness"][event["index"]] for event in self.eye_events],
                "o",
                color="tab:blue",
                ms=4,
                alpha=0.8,
            )

        ppg_axis.set_title("PPG channels (normalized, green used for detection)")
        eye_axis.set_title("Right-eye brightness")
        strength_axis.set_title("Rising-edge breakout strength")
        strength_axis.set_xlabel("Time after first kept PC timestamp (s)")
        ppg_axis.set_ylabel("PPG norm")
        eye_axis.set_ylabel("Brightness")
        strength_axis.set_ylabel("Noise x")
        strength_axis.axhline(self.threshold, color="gray", ls="--", lw=1.0, alpha=0.8)
        ppg_axis.legend(loc="upper right")
        eye_axis.legend(loc="upper right")
        strength_axis.legend(loc="upper right")

    def _draw_matches(self):
        base_time = self._base_time()
        for index, match in enumerate(self.matches):
            selected = match["selected"]
            color = "tab:green" if selected else "tab:red"
            alpha = 0.8 if selected else 0.22
            linewidth = 1.4 if selected else 0.9
            ppg_x = match["ppg_time"] - base_time
            eye_x = match["eye_time"] - base_time
            self.match_artists.append(self.axes[0].axvline(ppg_x, color=color, alpha=alpha, lw=linewidth))
            self.match_artists.append(self.axes[1].axvline(eye_x, color=color, alpha=alpha, lw=linewidth))
            self.match_artists.append(
                self.axes[2].axvspan(min(ppg_x, eye_x), max(ppg_x, eye_x), color=color, alpha=alpha * 0.15)
            )
            self.axes[2].text(
                (ppg_x + eye_x) * 0.5,
                0.98,
                str(index),
                transform=self.axes[2].get_xaxis_transform(),
                ha="center",
                va="top",
                fontsize=8,
                color=color,
            )

    def _draw_summary(self):
        selected = [match for match in self.matches if match["selected"]]
        if selected:
            deltas = np.array([match["delta_ms"] for match in selected], dtype=np.float64)
            result = f"selected={len(selected)} | avg={np.mean(deltas):.3f} ms | std={np.std(deltas):.3f} ms"
        else:
            result = "selected=0 | avg/std unavailable"

        self.summary_text.set_text(
            f"{os.path.basename(self.capture_dir)} | first {DISCARD_INITIAL_SECONDS:.0f}s discarded | "
            f"PPG edges={len(self.ppg_events)} | Eye edges={len(self.eye_events)} | "
            f"matched={len(self.matches)} | {result}"
        )

    def _draw_match_selector(self):
        self.check_axis.clear()
        self.check_axis.set_title("Matched edges", fontsize=10)
        self.check_axis.set_axis_off()

        if not self.matches:
            self.check_axis.text(0.02, 0.98, "No matches", va="top", fontsize=9)
            self.check_buttons = None
            return

        base_time = self._base_time()
        labels = [
            (
                f"{index:02d} "
                f"d={match['delta_ms']:7.3f}ms "
                f"E={match['eye_time'] - base_time:6.3f}s "
                f"P={match['ppg_time'] - base_time:6.3f}s"
            )
            for index, match in enumerate(self.matches)
        ]
        states = [match["selected"] for match in self.matches]
        self.check_buttons = CheckButtons(self.check_axis, labels, states)
        self.check_buttons.on_clicked(self._on_check_clicked)
        for text in self.check_buttons.labels:
            text.set_fontsize(7)
            text.set_family("monospace")


def parse_args():
    parser = argparse.ArgumentParser(description="Review a sync_capture_* directory.")
    parser.add_argument(
        "capture_dir",
        nargs="?",
        default=None,
        help="Capture directory. Defaults to the newest log/sync_capture_* directory.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    capture_dir = args.capture_dir or find_newest_capture_dir()
    if not capture_dir:
        print("No sync_capture_* directory found under log/.", file=sys.stderr)
        sys.exit(1)
    if not os.path.isdir(capture_dir):
        print(f"Capture directory not found: {capture_dir}", file=sys.stderr)
        sys.exit(1)

    sensor_rows, eye_rows, cutoff_time = load_capture(capture_dir)
    if not sensor_rows or not eye_rows:
        print(f"No usable rows remain after discarding the first {DISCARD_INITIAL_SECONDS:.0f}s.", file=sys.stderr)
        sys.exit(1)

    reviewer = SyncCaptureReviewer(capture_dir, sensor_rows, eye_rows, cutoff_time)
    reviewer.show()


if __name__ == "__main__":
    main()
