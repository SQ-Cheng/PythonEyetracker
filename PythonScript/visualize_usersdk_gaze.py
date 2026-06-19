from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


@dataclass
class Sample:
    seq: int
    timestamp: int
    t: float
    recommend: int
    recom_x: float
    recom_y: float
    left_x: float
    left_y: float
    right_x: float
    right_y: float
    left_pupil_x: float
    left_pupil_y: float
    right_pupil_x: float
    right_pupil_y: float
    left_re: float
    right_re: float


def load_font(size: int) -> ImageFont.ImageFont:
    for name in ("C:/Windows/Fonts/consola.ttf", "C:/Windows/Fonts/arial.ttf"):
        path = Path(name)
        if path.exists():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def valid_xy(x: float, y: float) -> bool:
    return math.isfinite(x) and math.isfinite(y) and (abs(x) > 1e-9 or abs(y) > 1e-9)


def lerp(a: int, b: int, u: float) -> int:
    return int(a + (b - a) * max(0.0, min(1.0, u)))


def time_color(u: float) -> tuple[int, int, int]:
    # Blue -> green -> orange/red.
    if u < 0.5:
        v = u / 0.5
        return (lerp(38, 42, v), lerp(104, 166, v), lerp(214, 80, v))
    v = (u - 0.5) / 0.5
    return (lerp(42, 220, v), lerp(166, 79, v), lerp(80, 57, v))


def read_samples(path: Path) -> list[Sample]:
    rows = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
    if not rows:
        return []
    t0 = int(rows[0]["timestamp"])
    samples = []
    for row in rows:
        ts = int(row["timestamp"])
        samples.append(
            Sample(
                seq=int(row["seq"]),
                timestamp=ts,
                t=(ts - t0) / 1_000_000.0,
                recommend=int(row["recommend"]),
                recom_x=float(row["recom_gaze_x"]),
                recom_y=float(row["recom_gaze_y"]),
                left_x=float(row["left_gaze_x"]),
                left_y=float(row["left_gaze_y"]),
                right_x=float(row["right_gaze_x"]),
                right_y=float(row["right_gaze_y"]),
                left_pupil_x=float(row["left_pupil_x"]),
                left_pupil_y=float(row["left_pupil_y"]),
                right_pupil_x=float(row["right_pupil_x"]),
                right_pupil_y=float(row["right_pupil_y"]),
                left_re=float(row["left_re"]),
                right_re=float(row["right_re"]),
            )
        )
    return samples


def rect_map(rect: tuple[int, int, int, int], x: float, y: float) -> tuple[int, int]:
    x0, y0, x1, y1 = rect
    return (int(x0 + x * (x1 - x0)), int(y0 + y * (y1 - y0)))


def draw_panel(draw: ImageDraw.ImageDraw, rect: tuple[int, int, int, int], title: str, font, title_font) -> None:
    x0, y0, x1, y1 = rect
    draw.rectangle(rect, outline=(198, 204, 213), width=1)
    draw.text((x0 + 12, y0 + 8), title, fill=(25, 31, 40), font=title_font)
    draw.line((x0, y0 + 36, x1, y0 + 36), fill=(226, 230, 236), width=1)


def draw_grid(draw: ImageDraw.ImageDraw, rect: tuple[int, int, int, int], font) -> None:
    x0, y0, x1, y1 = rect
    for i in range(1, 4):
        x = x0 + (x1 - x0) * i // 4
        y = y0 + (y1 - y0) * i // 4
        draw.line((x, y0, x, y1), fill=(232, 236, 241))
        draw.line((x0, y, x1, y), fill=(232, 236, 241))
    for value in (0.0, 0.5, 1.0):
        x = x0 + int(value * (x1 - x0))
        y = y0 + int(value * (y1 - y0))
        draw.text((x - 8, y1 + 4), f"{value:.1f}", fill=(91, 99, 112), font=font)
        draw.text((x0 - 34, y - 7), f"{value:.1f}", fill=(91, 99, 112), font=font)


def draw_path(
    draw: ImageDraw.ImageDraw,
    rect: tuple[int, int, int, int],
    samples: list[Sample],
    xy_getter,
    radius: int = 3,
) -> None:
    valid = [(s, *xy_getter(s)) for s in samples if valid_xy(*xy_getter(s))]
    if not valid:
        return
    max_t = max(s.t for s, _, _ in valid) or 1.0
    prev = None
    for s, x, y in valid:
        p = rect_map(rect, x, y)
        c = time_color(s.t / max_t)
        if prev is not None:
            prev_s, prev_p = prev
            if s.t - prev_s.t < 0.08:
                draw.line((prev_p[0], prev_p[1], p[0], p[1]), fill=c, width=2)
        draw.ellipse((p[0] - radius, p[1] - radius, p[0] + radius, p[1] + radius), fill=c)
        prev = (s, p)
    first = rect_map(rect, valid[0][1], valid[0][2])
    last = rect_map(rect, valid[-1][1], valid[-1][2])
    draw.ellipse((first[0] - 7, first[1] - 7, first[0] + 7, first[1] + 7), outline=(38, 104, 214), width=3)
    draw.rectangle((last[0] - 6, last[1] - 6, last[0] + 6, last[1] + 6), outline=(220, 79, 57), width=3)


def draw_timeseries(
    draw: ImageDraw.ImageDraw,
    rect: tuple[int, int, int, int],
    samples: list[Sample],
    field: str,
    color: tuple[int, int, int],
) -> None:
    vals = [(s.t, getattr(s, field)) for s in samples if abs(getattr(s, field)) > 1e-9]
    if len(vals) < 2:
        return
    tmax = max(t for t, _ in vals) or 1.0
    x0, y0, x1, y1 = rect
    pts = [(int(x0 + t / tmax * (x1 - x0)), int(y0 + v * (y1 - y0))) for t, v in vals]
    for a, b in zip(pts, pts[1:]):
        if abs(a[0] - b[0]) < (x1 - x0) / 8:
            draw.line((a[0], a[1], b[0], b[1]), fill=color, width=2)


def summarize(samples: list[Sample]) -> list[str]:
    def count(getter) -> int:
        return sum(1 for s in samples if valid_xy(*getter(s)))

    lines = [
        f"samples: {len(samples)}",
        f"duration: {samples[-1].t:.2f}s" if samples else "duration: n/a",
        f"recommended gaze valid: {count(lambda s: (s.recom_x, s.recom_y))}",
        f"left gaze valid:        {count(lambda s: (s.left_x, s.left_y))}",
        f"right gaze valid:       {count(lambda s: (s.right_x, s.right_y))}",
        f"left pupil valid:       {count(lambda s: (s.left_pupil_x, s.left_pupil_y))}",
        f"right pupil valid:      {count(lambda s: (s.right_pupil_x, s.right_pupil_y))}",
    ]
    rec = [s for s in samples if valid_xy(s.recom_x, s.recom_y)]
    if rec:
        xs = [s.recom_x for s in rec]
        ys = [s.recom_y for s in rec]
        lines.append(f"recommended x range: {min(xs):.3f} .. {max(xs):.3f}")
        lines.append(f"recommended y range: {min(ys):.3f} .. {max(ys):.3f}")
        lines.append(f"first valid rec: t={rec[0].t:.3f}s ({rec[0].recom_x:.3f}, {rec[0].recom_y:.3f})")
        lines.append(f"last valid rec:  t={rec[-1].t:.3f}s ({rec[-1].recom_x:.3f}, {rec[-1].recom_y:.3f})")
    return lines


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()

    gaze_csv = args.output_dir / "gaze_samples.csv"
    samples = read_samples(gaze_csv)
    if not samples:
        raise SystemExit(f"No gaze samples found in {gaze_csv}")

    out = args.out or (args.output_dir / "gaze_visualization.png")
    summary_path = args.output_dir / "gaze_visualization_summary.txt"

    W, H = 1800, 1320
    img = Image.new("RGB", (W, H), (248, 250, 252))
    draw = ImageDraw.Draw(img)
    font = load_font(20)
    small = load_font(16)
    title_font = load_font(26)
    header = load_font(34)

    draw.text((34, 24), f"USERSDK Gaze Visualization - {args.output_dir.name}", fill=(18, 24, 33), font=header)
    draw.text((36, 68), "Time color: blue = start, green = middle, red = end. Coordinates are normalized 0..1.", fill=(82, 91, 105), font=font)

    left_panel = (40, 230, 880, 890)
    right_panel = (920, 230, 1760, 890)
    bottom_panel = (40, 930, 1760, 1260)

    draw_panel(draw, left_panel, "Recommended Gaze Path", font, title_font)
    draw_panel(draw, right_panel, "Left / Right Gaze Paths", font, title_font)
    draw_panel(draw, bottom_panel, "Recommended Gaze Over Time", font, title_font)

    path_rect = (left_panel[0] + 70, left_panel[1] + 72, left_panel[2] - 40, left_panel[3] - 58)
    draw_grid(draw, path_rect, small)
    draw_path(draw, path_rect, samples, lambda s: (s.recom_x, s.recom_y), radius=3)
    draw.text((path_rect[0], path_rect[3] + 28), "circle = first valid point, square = last valid point", fill=(82, 91, 105), font=small)

    lr_rect = (right_panel[0] + 70, right_panel[1] + 72, right_panel[2] - 40, right_panel[3] - 58)
    draw_grid(draw, lr_rect, small)
    draw_path(draw, lr_rect, samples, lambda s: (s.left_x, s.left_y), radius=2)
    # Right gaze as black outlined dots so both streams can be compared without hiding the time gradient.
    right_valid = [s for s in samples if valid_xy(s.right_x, s.right_y)]
    for s in right_valid[:: max(1, len(right_valid) // 350)]:
        p = rect_map(lr_rect, s.right_x, s.right_y)
        draw.ellipse((p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3), outline=(25, 31, 40), width=1)
    draw.text((lr_rect[0], lr_rect[3] + 28), "colored line = left gaze, black rings = sampled right gaze", fill=(82, 91, 105), font=small)

    ts_rect = (bottom_panel[0] + 80, bottom_panel[1] + 70, bottom_panel[2] - 40, bottom_panel[3] - 50)
    draw_grid(draw, ts_rect, small)
    draw_timeseries(draw, ts_rect, samples, "recom_x", (38, 104, 214))
    draw_timeseries(draw, ts_rect, samples, "recom_y", (220, 79, 57))
    draw.text((ts_rect[0] + 8, ts_rect[1] + 8), "blue: x", fill=(38, 104, 214), font=font)
    draw.text((ts_rect[0] + 110, ts_rect[1] + 8), "red: y", fill=(220, 79, 57), font=font)
    draw.text((ts_rect[0], ts_rect[3] + 22), "time from start (left to right); normalized coordinate value (top=0, bottom=1)", fill=(82, 91, 105), font=small)

    summary_lines = summarize(samples)
    summary_x = 40
    summary_y = 100
    for i, line in enumerate(summary_lines):
        column = i // 4
        row = i % 4
        draw.text((summary_x + column * 430, summary_y + row * 24), line, fill=(25, 31, 40), font=small)

    img.save(out)
    summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    print(f"wrote {out}")
    print(f"wrote {summary_path}")
    for line in summary_lines:
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
