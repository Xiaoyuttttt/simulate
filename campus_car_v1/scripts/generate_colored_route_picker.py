#!/usr/bin/env python3
"""Generate a full-campus, georeferenced color map from the original 3DGS PLY."""

from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import json
import math
import time
import traceback
from pathlib import Path

import numpy as np
from PIL import Image, ImageEnhance
from plyfile import PlyData


C0 = 0.28209479177387814
parser = argparse.ArgumentParser()
parser.add_argument("--config", required=True)
args = parser.parse_args()
cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
root = Path(cfg["project_root"])
out = Path(cfg["output_dir"])
out.mkdir(parents=True, exist_ok=True)
started = time.monotonic()


def stamp(message: str) -> None:
    print(f"[{time.monotonic() - started:8.2f}s] {message}", flush=True)


def field(vertex, name: str) -> np.ndarray:
    if name not in vertex.dtype.names:
        raise KeyError(f"PLY缺少属性：{name}")
    return np.asarray(vertex[name], dtype=np.float32)


def reference_points(path: Path) -> list[dict]:
    result = []
    if path.is_file():
        with path.open("r", encoding="utf-8-sig", newline="") as handle:
            for row in csv.DictReader(handle):
                result.append({"x": float(row["x_hint"]), "y": float(row["y_hint"])})
    return result


try:
    ply_path = Path(cfg["campus_3dgs_ply"])
    stamp(f"读取完整彩色3DGS：{ply_path}")
    vertex = PlyData.read(str(ply_path), mmap="c")["vertex"].data
    total = len(vertex)
    stamp(f"高斯数量：{total:,}")
    x, y = field(vertex, "x"), field(vertex, "y")
    m = cfg["map"]
    sample_step = max(1, total // 2_000_000)
    sample_x, sample_y = x[::sample_step], y[::sample_step]
    low = float(m["auto_bounds_percentile_low"])
    high = float(m["auto_bounds_percentile_high"])
    pad = float(m["bounds_padding_m"])
    x_min, x_max = np.percentile(sample_x, [low, high]).tolist()
    y_min, y_max = np.percentile(sample_y, [low, high]).tolist()
    x_min, x_max, y_min, y_max = x_min - pad, x_max + pad, y_min - pad, y_max + pad

    # Always retain the old reference segment so the operator can compare it.
    ref = reference_points(Path(cfg["reference_waypoints_csv"]))
    if ref:
        x_min = min(x_min, min(p["x"] for p in ref) - pad)
        x_max = max(x_max, max(p["x"] for p in ref) + pad)
        y_min = min(y_min, min(p["y"] for p in ref) - pad)
        y_max = max(y_max, max(p["y"] for p in ref) + pad)

    span_x, span_y = x_max - x_min, y_max - y_min
    target_mpp = float(m["target_meters_per_pixel"])
    max_dim = int(m["max_image_dimension_px"])
    scale = min(1.0 / target_mpp, max_dim / max(span_x, span_y))
    width = max(512, int(math.ceil(span_x * scale)))
    height = max(512, int(math.ceil(span_y * scale)))
    pixels = width * height
    stamp(f"地图范围：X[{x_min:.2f},{x_max:.2f}] Y[{y_min:.2f},{y_max:.2f}]")
    stamp(f"地图分辨率：{width}x{height}，约{span_x / width:.3f}米/像素")

    chunk = int(m.get("point_chunk_size", 2_000_000))
    weight_sum = np.zeros(pixels, dtype=np.float64)
    color_sum = [np.zeros(pixels, dtype=np.float64) for _ in range(3)]
    used = 0
    names = vertex.dtype.names or ()
    has_opacity = "opacity" in names
    for begin in range(0, total, chunk):
        end = min(begin + chunk, total)
        xx, yy = x[begin:end], y[begin:end]
        keep = (xx >= x_min) & (xx <= x_max) & (yy >= y_min) & (yy <= y_max)
        if not keep.any():
            continue
        xx, yy = xx[keep], yy[keep]
        col = np.floor((xx - x_min) / span_x * width).astype(np.int64)
        row = np.floor((y_max - yy) / span_y * height).astype(np.int64)
        col, row = np.clip(col, 0, width - 1), np.clip(row, 0, height - 1)
        index = row * width + col
        if has_opacity:
            op = field(vertex[begin:end], "opacity")[keep]
            weights = 1.0 / (1.0 + np.exp(-np.clip(op, -15.0, 15.0)))
            weights = np.maximum(weights, 0.02).astype(np.float32)
        else:
            weights = np.ones(len(index), dtype=np.float32)
        weight_sum += np.bincount(index, weights=weights, minlength=pixels)
        for ci, name in enumerate(("f_dc_0", "f_dc_1", "f_dc_2")):
            channel = np.clip(0.5 + C0 * field(vertex[begin:end], name)[keep], 0.0, 1.0)
            color_sum[ci] += np.bincount(index, weights=weights * channel, minlength=pixels)
        used += len(index)
        stamp(f"颜色投影：{end:,}/{total:,}")

    occupied = weight_sum > 0
    if occupied.sum() < 10_000:
        raise RuntimeError("彩色地图有效像素过少")
    rgb = np.zeros((pixels, 3), dtype=np.uint8)
    rgb[:] = (8, 11, 15)
    for ci in range(3):
        values = np.zeros(pixels, dtype=np.float32)
        values[occupied] = (color_sum[ci][occupied] / weight_sum[occupied]).astype(np.float32)
        # Mild gamma lift helps the unlit SH0 colors read like an aerial map.
        values = np.power(np.clip(values, 0.0, 1.0), 0.82)
        rgb[:, ci] = np.uint8(np.clip(values * 255.0, 0, 255))
        color_sum[ci] = None
    image = Image.fromarray(rgb.reshape(height, width, 3), mode="RGB")
    image = ImageEnhance.Color(image).enhance(1.15)
    image = ImageEnhance.Contrast(image).enhance(1.08)
    image_path = out / "route_map_color.png"
    image.save(image_path, optimize=True)

    metadata = {
        "coordinate_frame": "Isaac/Usd, Z-up, meters",
        "source": str(ply_path),
        "source_kind": "original colored 3DGS PLY, SH degree-0 top-down aggregation",
        "image": image_path.name,
        "image_width_px": width,
        "image_height_px": height,
        "bounds": {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
        "orientation": {"image_right": "+X", "image_up": "+Y"},
        "meters_per_pixel": {"x": span_x / width, "y": span_y / height},
        "gaussians_total": total,
        "gaussians_in_bounds": used,
        "occupied_pixels": int(occupied.sum()),
        "map_image_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
    }
    metadata_path = out / "route_map_metadata.json"
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")

    template = (root / "route_picker/index.template.html").read_text(encoding="utf-8")
    image_uri = "data:image/png;base64," + base64.b64encode(image_path.read_bytes()).decode("ascii")
    html = template.replace("__META_JSON__", json.dumps(metadata, ensure_ascii=False, separators=(",", ":")))
    html = html.replace("__REFERENCE_JSON__", json.dumps(ref, ensure_ascii=False, separators=(",", ":")))
    html = html.replace("__IMAGE_DATA_URI__", image_uri)
    picker = out / "route_picker_color.html"
    picker.write_text(html, encoding="utf-8")
    print(f"COLOR_ROUTE_MAP={image_path}")
    print(f"COLOR_ROUTE_PICKER={picker}")
    print(f"METADATA={metadata_path}")
    print("COLOR_ROUTE_PICKER_PASS")
except Exception as exc:
    failure = {"status": "FAILED", "exception_type": type(exc).__name__, "exception": str(exc), "traceback": traceback.format_exc()}
    (out / "color_failure.json").write_text(json.dumps(failure, ensure_ascii=False, indent=2), encoding="utf-8")
    traceback.print_exc()
    raise
