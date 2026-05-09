#!/usr/bin/env python3
"""
Matplotlib-based offline review and offset estimation for sync captures.

Run without arguments to open the newest log/sync_capture_* directory:
    python example/sync_capture_review.py
"""

import argparse
import csv
import json
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider


DEFAULT_TOP_N = 10
PAIR_MAX_GAP_SECONDS = 0.5
EDGE_REFRACTORY_SECONDS = 0.4
EDGE_FOOT_LOOKBACK_SECONDS = 0.10
EDGE_FOOT_THRESHOLD_MULTIPLIER = 1.0
PPG_DETECTION_CHANNEL = "Green"


def read_csv_rows(path):
    with open(path, "r", newline="") as fp:
        return list(csv.DictReader(fp))


def load_capture(capture_dir):
    sensor_rows = read_csv_rows(os.path.join(capture_dir, "sensor_ppg.csv"))
    eye_rows = read_csv_rows(os.path.join(capture_dir, "right_eye_brightness.csv"))
    metadata_path = os.path.join(capture_dir, "capture_metadata.json")
    metadata = {}
    if os.path.isfile(metadata_path):
        with open(metadata_path, "r", encoding="utf-8") as fp:
            metadata = json.load(fp)
    return {
        "sensor_rows": sensor_rows,
        "eye_rows": eye_rows,
        "metadata": metadata,
    }


def find_newest_capture_dir():
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    log_root = os.path.join(project_root, "log")
    if not os.path.isdir(log_root):
        return None

    newest_path = None
    newest_mtime = None
    for entry in os.scandir(log_root):
        if not entry.is_dir() or not entry.name.startswith("sync_capture_"):
            continue
        try:
            stat = entry.stat()
        except OSError:
            continue
        if newest_mtime is None or stat.st_mtime > newest_mtime:
            newest_mtime = stat.st_mtime
            newest_path = entry.path
    return newest_path


def _moving_average(values, window_samples):
    if len(values) == 0:
        return values
    window_samples = max(3, int(window_samples))
    if window_samples % 2 == 0:
        window_samples += 1
    if window_samples >= len(values):
        return np.full_like(values, np.mean(values), dtype=np.float64)
    kernel = np.ones(window_samples, dtype=np.float64) / float(window_samples)
    pad = window_samples // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def _build_centered_signal(times, values, baseline_window_seconds):
    if len(times) == 0:
        empty = np.array([], dtype=np.float64)
        return empty, empty, empty, 1e-6

    baseline_dt = np.median(np.diff(times)) if len(times) >= 2 else 0.01
    baseline_samples = max(5, int(round(baseline_window_seconds / max(baseline_dt, 1e-6))))
    baseline = _moving_average(values, baseline_samples)
    centered = values - baseline

    amplitude_med = float(np.median(centered))
    amplitude_mad = float(np.median(np.abs(centered - amplitude_med)))
    amplitude_sigma = max(1.4826 * amplitude_mad, 1e-6)
    breakout_strength = np.maximum(0.0, centered / amplitude_sigma)
    return centered, baseline, breakout_strength, amplitude_sigma


def _refine_edge_foot_index(times, centered, strict_idx, amplitude_sigma):
    if strict_idx <= 0:
        return strict_idx

    foot_threshold = EDGE_FOOT_THRESHOLD_MULTIPLIER * amplitude_sigma
    lookback_start_time = times[strict_idx] - EDGE_FOOT_LOOKBACK_SECONDS
    foot_idx = strict_idx
    while foot_idx > 0 and times[foot_idx - 1] >= lookback_start_time:
        if centered[foot_idx - 1] <= foot_threshold:
            break
        foot_idx -= 1
    return foot_idx


def _detect_edges(times, values, baseline_window_seconds, threshold_multiplier, refractory_seconds):
    centered, _baseline, breakout_strength, amplitude_sigma = _build_centered_signal(
        times,
        values,
        baseline_window_seconds,
    )
    threshold_value = threshold_multiplier * amplitude_sigma
    events = []
    idx = 0
    while idx < len(times):
        if centered[idx] <= threshold_value:
            idx += 1
            continue

        strict_idx = idx
        foot_idx = _refine_edge_foot_index(times, centered, strict_idx, amplitude_sigma)
        events.append({
            "index": foot_idx,
            "strict_index": strict_idx,
            "time": float(times[foot_idx]),
            "strict_time": float(times[strict_idx]),
            "value": float(values[foot_idx]),
            "centered_value": float(centered[foot_idx]),
            "foot_strength": float(breakout_strength[foot_idx]),
            "strength": float(breakout_strength[strict_idx]),
            "strict_strength": float(breakout_strength[strict_idx]),
            "noise_sigma": float(amplitude_sigma),
            "threshold_value": float(threshold_value),
        })
        skip_until = times[foot_idx] + refractory_seconds
        idx = strict_idx + 1
        while idx < len(times) and times[idx] < skip_until:
            idx += 1
    return events, centered, breakout_strength


def detect_ppg_events(sensor_rows, threshold_multiplier):
    times = np.array([float(row["sensor_pc_timestamp"]) for row in sensor_rows], dtype=np.float64)
    red = np.array([float(row["red"]) for row in sensor_rows], dtype=np.float64)
    ir = np.array([float(row["ir"]) for row in sensor_rows], dtype=np.float64)
    green = np.array([float(row["green"]) for row in sensor_rows], dtype=np.float64)

    centered_green, _, strength_green, sigma_green = _build_centered_signal(times, green, baseline_window_seconds=0.4)

    if len(times) == 0:
        breakout_mask = np.array([], dtype=bool)
    else:
        breakout_mask = centered_green > threshold_multiplier * sigma_green

    events = []
    idx = 0
    while idx < len(times):
        if not breakout_mask[idx]:
            idx += 1
            continue

        strict_idx = idx
        foot_idx = _refine_edge_foot_index(times, centered_green, strict_idx, sigma_green)
        events.append({
            "index": foot_idx,
            "strict_index": strict_idx,
            "time": float(times[foot_idx]),
            "strict_time": float(times[strict_idx]),
            "foot_strength": float(strength_green[foot_idx]),
            "strength": float(strength_green[strict_idx]),
            "strict_strength": float(strength_green[strict_idx]),
            "channel": PPG_DETECTION_CHANNEL,
            "centered_value": float(centered_green[foot_idx]),
            "noise_sigma": float(sigma_green),
            "threshold_value": float(threshold_multiplier * sigma_green),
        })
        skip_until = times[foot_idx] + EDGE_REFRACTORY_SECONDS
        idx = strict_idx + 1
        while idx < len(times) and times[idx] < skip_until:
            idx += 1

    traces = {
        "times": times,
        "red": red,
        "ir": ir,
        "green": green,
        "combined_strength": strength_green,
    }
    return events, traces


def detect_eye_events(eye_rows, threshold_multiplier):
    time_column = "eye_device_pc_timestamp" if "eye_device_pc_timestamp" in eye_rows[0] else "pc_perf_timestamp"
    times = np.array([float(row[time_column]) for row in eye_rows], dtype=np.float64)
    brightness = np.array([float(row["mean_brightness"]) for row in eye_rows], dtype=np.float64)
    events, centered, breakout_strength = _detect_edges(
        times,
        brightness,
        baseline_window_seconds=0.25,
        threshold_multiplier=threshold_multiplier,
        refractory_seconds=EDGE_REFRACTORY_SECONDS,
    )
    traces = {
        "times": times,
        "brightness": brightness,
        "centered": centered,
        "breakout_strength": breakout_strength,
    }
    return events, traces


def match_events(ppg_events, eye_events, max_gap_seconds):
    matches = []
    used_ppg = set()
    for eye_event in eye_events:
        best = None
        best_gap = None
        for idx, ppg_event in enumerate(ppg_events):
            if idx in used_ppg:
                continue
            gap = abs(ppg_event["time"] - eye_event["time"])
            if gap > max_gap_seconds:
                continue
            if best_gap is None or gap < best_gap:
                best_gap = gap
                best = (idx, ppg_event)
        if best is None:
            continue
        idx, ppg_event = best
        used_ppg.add(idx)
        delta_ms = (ppg_event["time"] - eye_event["time"]) * 1000.0
        matches.append({
            "eye_time": eye_event["time"],
            "ppg_time": ppg_event["time"],
            "delta_ms": delta_ms,
            "eye_strength": eye_event["strength"],
            "ppg_strength": ppg_event["strength"],
            "pair_strength": min(eye_event["strength"], ppg_event["strength"]),
            "ppg_channel": ppg_event["channel"],
            "enabled": True,
        })
    matches.sort(key=lambda item: item["pair_strength"], reverse=True)
    return matches


def selected_matches(matches, min_strength, top_n):
    filtered = [m for m in matches if m["enabled"] and m["pair_strength"] >= min_strength]
    filtered.sort(key=lambda item: item["pair_strength"], reverse=True)
    return filtered[:max(1, int(top_n))]


def normalize_trace(values):
    values = np.asarray(values, dtype=np.float64)
    if len(values) == 0:
        return values
    lower = float(np.percentile(values, 2))
    upper = float(np.percentile(values, 98))
    if upper <= lower:
        return np.zeros_like(values)
    return np.clip((values - lower) / (upper - lower), 0.0, 1.0)


class SyncMatplotlibReviewer:
    def __init__(self, capture_dir, data, top_n):
        self.capture_dir = capture_dir
        self.data = data
        self.initial_top_n = int(top_n)
        self.threshold = 3.0
        self.ppg_events = []
        self.eye_events = []
        self.matches = []
        self.ppg_traces = {}
        self.eye_traces = {}
        self.fig = None
        self.axes = None
        self.slider_threshold = None
        self.slider_top_n = None
        self.button_reset = None
        self.match_artists = []
        self.summary_text = None
        self.detail_text = None

    def show(self):
        plt.ion()
        self.fig, self.axes = plt.subplots(3, 1, figsize=(15, 9), sharex=True)
        plt.subplots_adjust(left=0.07, right=0.82, top=0.9, bottom=0.22, hspace=0.18)

        ax_threshold = plt.axes([0.20, 0.10, 0.48, 0.03])
        ax_top_n = plt.axes([0.20, 0.055, 0.48, 0.03])
        ax_reset = plt.axes([0.72, 0.055, 0.10, 0.075])
        self.slider_threshold = Slider(ax_threshold, "Noise x", 1.0, 8.0, valinit=self.threshold, valstep=0.1)
        self.slider_top_n = Slider(ax_top_n, "Top N", 1, 50, valinit=self.initial_top_n, valstep=1)
        self.button_reset = Button(ax_reset, "Reset")
        self.summary_text = self.fig.text(0.07, 0.945, "", fontsize=10, va="top")
        self.detail_text = self.fig.text(0.835, 0.87, "", fontsize=9, va="top", family="monospace")

        self.slider_threshold.on_changed(self._on_threshold_changed)
        self.slider_top_n.on_changed(self._on_top_n_changed)
        self.button_reset.on_clicked(self._on_reset_clicked)
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)

        self._recompute(reset_enabled=True)
        self.fig.canvas.draw_idle()
        plt.show(block=True)

    def _on_threshold_changed(self, value):
        self.threshold = float(value)
        self._recompute(reset_enabled=True)

    def _on_top_n_changed(self, _value):
        self._refresh_display()

    def _on_reset_clicked(self, _event):
        for match in self.matches:
            match["enabled"] = True
        self._refresh_display()

    def _on_click(self, event):
        if event.inaxes not in self.axes or event.xdata is None or not self.matches:
            return
        t0 = self._base_time()
        candidates = []
        for idx, match in enumerate(self.matches):
            ppg_x = match["ppg_time"] - t0
            eye_x = match["eye_time"] - t0
            candidates.append((abs(event.xdata - ppg_x), idx))
            candidates.append((abs(event.xdata - eye_x), idx))
        distance, match_idx = min(candidates, key=lambda item: item[0])
        x_min, x_max = event.inaxes.get_xlim()
        click_tolerance = max((x_max - x_min) * 0.01, 0.05)
        if distance <= click_tolerance:
            self.matches[match_idx]["enabled"] = not self.matches[match_idx]["enabled"]
            self._refresh_display()

    def _recompute(self, reset_enabled):
        self.ppg_events, self.ppg_traces = detect_ppg_events(self.data["sensor_rows"], self.threshold)
        self.eye_events, self.eye_traces = detect_eye_events(self.data["eye_rows"], self.threshold)
        old_enabled = {
            (round(match["ppg_time"], 6), round(match["eye_time"], 6)): match.get("enabled", True)
            for match in self.matches
        }
        self.matches = match_events(self.ppg_events, self.eye_events, PAIR_MAX_GAP_SECONDS)
        if not reset_enabled:
            for match in self.matches:
                key = (round(match["ppg_time"], 6), round(match["eye_time"], 6))
                match["enabled"] = old_enabled.get(key, True)
        self._refresh_display()

    def _refresh_display(self):
        for artist in self.match_artists:
            try:
                artist.remove()
            except ValueError:
                pass
        self.match_artists = []

        for ax in self.axes:
            ax.clear()
            ax.grid(True, alpha=0.25)

        self._draw_signals()
        self._draw_matches()
        self._draw_summary()
        self.fig.canvas.draw_idle()

    def _base_time(self):
        ppg_times = self.ppg_traces.get("times", np.array([], dtype=np.float64))
        eye_times = self.eye_traces.get("times", np.array([], dtype=np.float64))
        candidates = []
        if len(ppg_times):
            candidates.append(float(ppg_times[0]))
        if len(eye_times):
            candidates.append(float(eye_times[0]))
        return min(candidates) if candidates else 0.0

    def _draw_signals(self):
        ax_ppg, ax_eye, ax_strength = self.axes
        t0 = self._base_time()

        ppg_times = self.ppg_traces.get("times", np.array([], dtype=np.float64)) - t0
        if len(ppg_times):
            ax_ppg.plot(
                ppg_times,
                normalize_trace(self.ppg_traces["red"]),
                color="tab:red",
                lw=0.9,
                marker=".",
                markersize=2.0,
                label="Red",
            )
            ax_ppg.plot(
                ppg_times,
                normalize_trace(self.ppg_traces["ir"]),
                color="tab:orange",
                lw=0.9,
                marker=".",
                markersize=2.0,
                label="IR",
            )
            ax_ppg.plot(
                ppg_times,
                normalize_trace(self.ppg_traces["green"]),
                color="tab:green",
                lw=1.1,
                marker=".",
                markersize=2.4,
                label="Green",
            )
            ax_strength.plot(
                ppg_times,
                self.ppg_traces["combined_strength"],
                color="tab:green",
                lw=1.0,
                marker=".",
                markersize=2.0,
                alpha=0.65,
                label="Green PPG breakout",
            )

        eye_times = self.eye_traces.get("times", np.array([], dtype=np.float64)) - t0
        if len(eye_times):
            ax_eye.plot(
                eye_times,
                self.eye_traces["brightness"],
                color="black",
                lw=0.9,
                marker=".",
                markersize=2.0,
                alpha=0.75,
                label="Brightness",
            )
            ax_eye_twin = ax_eye.twinx()
            ax_eye_twin.plot(
                eye_times,
                self.eye_traces["breakout_strength"],
                color="tab:blue",
                lw=1.0,
                marker=".",
                markersize=2.0,
                alpha=0.65,
                label="Eye breakout",
            )
            ax_eye_twin.set_ylabel("Eye breakout")
            ax_eye_twin.tick_params(axis="y", labelsize=8)
            self.match_artists.append(ax_eye_twin)
            ax_strength.plot(
                eye_times,
                self.eye_traces["breakout_strength"],
                color="tab:blue",
                lw=1.0,
                marker=".",
                markersize=2.0,
                alpha=0.65,
                label="Eye breakout",
            )

        ax_ppg.set_title("PPG channels (normalized for review)")
        ax_eye.set_title("Right-eye brightness")
        ax_strength.set_title("Centered-signal breakout strength")
        ax_strength.set_xlabel("Time from capture start (s)")
        ax_ppg.set_ylabel("PPG norm")
        ax_eye.set_ylabel("Brightness")
        ax_strength.set_ylabel("Noise multiples")
        ax_strength.axhline(self.threshold, color="gray", ls="--", lw=1.0, alpha=0.8)
        ax_ppg.legend(loc="upper right")
        ax_eye.legend(loc="upper right")
        ax_strength.legend(loc="upper right")

    def _draw_matches(self):
        selected = selected_matches(self.matches, self.threshold, int(self.slider_top_n.val))
        selected_keys = {(match["ppg_time"], match["eye_time"]) for match in selected}
        t0 = self._base_time()

        for match in self.matches:
            enabled = match.get("enabled", True)
            is_selected = (match["ppg_time"], match["eye_time"]) in selected_keys
            color = "tab:green" if is_selected else ("tab:gray" if enabled else "tab:red")
            alpha = 0.85 if is_selected else (0.25 if enabled else 0.18)
            linewidth = 1.7 if is_selected else 0.9
            ppg_x = match["ppg_time"] - t0
            eye_x = match["eye_time"] - t0

            self.match_artists.append(self.axes[0].axvline(ppg_x, color=color, lw=linewidth, alpha=alpha))
            self.match_artists.append(self.axes[1].axvline(eye_x, color=color, lw=linewidth, alpha=alpha))
            self.match_artists.append(self.axes[2].axvspan(min(ppg_x, eye_x), max(ppg_x, eye_x), color=color, alpha=alpha * 0.15))

    def _draw_summary(self):
        top_n = int(self.slider_top_n.val)
        selected = selected_matches(self.matches, self.threshold, top_n)
        if selected:
            deltas = np.array([match["delta_ms"] for match in selected], dtype=np.float64)
            avg_ms = float(np.mean(deltas))
            std_ms = float(np.std(deltas, ddof=0))
            result_text = f"Top {len(selected)} offset: avg={avg_ms:.3f} ms, std={std_ms:.3f} ms"
        else:
            result_text = "No selected matches at this noise threshold"

        self.summary_text.set_text(
            f"{os.path.basename(self.capture_dir)} | "
            f"PPG edges={len(self.ppg_events)} | Eye edges={len(self.eye_events)} | "
            f"Matches={len(self.matches)} | {result_text} | "
            "Click a matched edge to include/exclude it."
        )

        selected = selected_matches(self.matches, self.threshold, top_n)
        lines = ["Selected matches", "stren  delta_ms  ch"]
        for match in selected[:18]:
            lines.append(f"{match['pair_strength']:5.2f} {match['delta_ms']:8.3f}  {match['ppg_channel']}")
        if len(selected) > 18:
            lines.append(f"... {len(selected) - 18} more")
        self.detail_text.set_text("\n".join(lines))


def parse_args():
    parser = argparse.ArgumentParser(description="Review a sync capture directory.")
    parser.add_argument(
        "capture_dir",
        nargs="?",
        default=None,
        help="Path to a sync_capture_YYYYMMDD_HHMMSS folder. Defaults to the newest capture.",
    )
    parser.add_argument("--top-n", type=int, default=DEFAULT_TOP_N)
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

    data = load_capture(capture_dir)
    if not data["sensor_rows"] or not data["eye_rows"]:
        print(f"Capture is missing sensor or right-eye rows: {capture_dir}", file=sys.stderr)
        sys.exit(1)

    reviewer = SyncMatplotlibReviewer(capture_dir, data, args.top_n)
    reviewer.show()


if __name__ == "__main__":
    main()
