#!/usr/bin/env python3
"""Build a georeferenced top-down mesh map and a self-contained route picker."""

from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import json
import time
import traceback
from pathlib import Path

import numpy as np


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


from isaacsim import SimulationApp


app = SimulationApp(
    launch_config={
        "headless": True,
        "renderer": "RaytracedLighting",
        "width": 320,
        "height": 180,
        "multi_gpu": False,
        "max_gpu_count": 1,
        "sync_loads": True,
        "disable_viewport_updates": True,
        "extra_args": [
            "--/renderer/multiGpu/enabled=false",
            "--/renderer/multiGpu/autoEnable=false",
            "--/renderer/multiGpu/maxGpuCount=1",
        ],
    }
)


def transform_chunk(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return points @ matrix[:3, :3] + matrix[3, :3]


def load_reference(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    result = []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        for row in csv.DictReader(handle):
            result.append({"x": float(row["x_hint"]), "y": float(row["y_hint"])})
    return result


try:
    from PIL import Image, ImageEnhance, ImageFilter
    from pxr import Usd, UsdGeom

    map_cfg = cfg["map"]
    x_min, x_max = float(map_cfg["x_min"]), float(map_cfg["x_max"])
    y_min, y_max = float(map_cfg["y_min"]), float(map_cfg["y_max"])
    z_min, z_max = float(map_cfg["z_min"]), float(map_cfg["z_max"])
    width, height = int(map_cfg["width_px"]), int(map_cfg["height_px"])
    chunk_size = int(map_cfg.get("point_chunk_size", 1_000_000))
    if not (x_max > x_min and y_max > y_min and z_max > z_min):
        raise ValueError("route_map.json bounds are invalid")
    if width < 200 or height < 200:
        raise ValueError("route map resolution is too small")

    campus = Path(cfg["campus_usd"])
    stamp(f"打开校园USD：{campus}")
    stage = Usd.Stage.Open(str(campus))
    if not stage:
        raise RuntimeError(f"无法打开校园USD：{campus}")
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    count = np.zeros((height, width), dtype=np.uint32)
    min_z = np.full((height, width), np.inf, dtype=np.float32)
    max_z = np.full((height, width), -np.inf, dtype=np.float32)
    vertices_total = vertices_used = 0
    mesh_records = []

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        raw = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if not raw:
            continue
        points = np.asarray(raw, dtype=np.float32)
        matrix = np.asarray(xform_cache.GetLocalToWorldTransform(prim), dtype=np.float64)
        vertices_total += len(points)
        used_here = 0
        stamp(f"栅格化 {prim.GetPath()}：{len(points):,} 顶点")
        for begin in range(0, len(points), chunk_size):
            world = transform_chunk(points[begin:begin + chunk_size].astype(np.float64), matrix)
            keep = (
                (world[:, 0] >= x_min) & (world[:, 0] <= x_max)
                & (world[:, 1] >= y_min) & (world[:, 1] <= y_max)
                & (world[:, 2] >= z_min) & (world[:, 2] <= z_max)
            )
            world = world[keep]
            if not len(world):
                continue
            col = np.floor((world[:, 0] - x_min) / (x_max - x_min) * width).astype(np.int64)
            row = np.floor((y_max - world[:, 1]) / (y_max - y_min) * height).astype(np.int64)
            col = np.clip(col, 0, width - 1)
            row = np.clip(row, 0, height - 1)
            z = world[:, 2].astype(np.float32)
            np.add.at(count, (row, col), 1)
            np.minimum.at(min_z, (row, col), z)
            np.maximum.at(max_z, (row, col), z)
            used_here += len(world)
        vertices_used += used_here
        mesh_records.append({"prim": str(prim.GetPath()), "vertices": len(points), "vertices_in_map": used_here})

    occupied = count > 0
    if occupied.sum() < 1000:
        raise RuntimeError(f"地图范围内只有 {int(occupied.sum())} 个有效像素，请调整route_map.json范围")

    # Mesh is gray and untextured.  Combine log vertex density, low-surface
    # elevation and vertical spread to make roads/building edges readable.
    density = np.log1p(count.astype(np.float32))
    d_hi = float(np.percentile(density[occupied], 99.0))
    density = np.clip(density / max(d_hi, 1e-6), 0.0, 1.0)
    elevation = np.zeros_like(density)
    elevation[occupied] = np.clip((min_z[occupied] - z_min) / (z_max - z_min), 0.0, 1.0)
    spread = np.zeros_like(density)
    spread[occupied] = np.clip((max_z[occupied] - min_z[occupied]) / 2.0, 0.0, 1.0)
    gray = 22.0 + 172.0 * density + 42.0 * elevation - 35.0 * spread
    gray[~occupied] = 12.0
    base = Image.fromarray(np.uint8(np.clip(gray, 0, 255)), mode="L")
    base = base.filter(ImageFilter.MaxFilter(3)).filter(ImageFilter.GaussianBlur(0.45))
    base = ImageEnhance.Contrast(base).enhance(1.35)
    rgb = Image.merge("RGB", (base, base, base))
    image_path = out / "route_map.png"
    rgb.save(image_path, optimize=True)

    metadata = {
        "coordinate_frame": "Isaac/Usd, Z-up, meters",
        "image": image_path.name,
        "image_width_px": width,
        "image_height_px": height,
        "bounds": {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
        "vertical_filter_m": {"z_min": z_min, "z_max": z_max},
        "orientation": {"image_right": "+X", "image_up": "+Y"},
        "pixel_to_world": {
            "x": "x_min + pixel_x / image_width * (x_max - x_min)",
            "y": "y_max - pixel_y / image_height * (y_max - y_min)",
        },
        "meters_per_pixel": {"x": (x_max - x_min) / width, "y": (y_max - y_min) / height},
        "vertices_total": vertices_total,
        "vertices_used": vertices_used,
        "occupied_pixels": int(occupied.sum()),
        "mesh_records": mesh_records,
        "map_image_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
        "note": "Map is a georeferenced top-down raster of gray mesh vertices, not a textured RGB render.",
    }
    metadata_path = out / "route_map_metadata.json"
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")

    template_path = root / "route_picker/index.template.html"
    template = template_path.read_text(encoding="utf-8")
    image_uri = "data:image/png;base64," + base64.b64encode(image_path.read_bytes()).decode("ascii")
    reference = load_reference(Path(cfg["reference_waypoints_csv"]))
    html = template.replace("__META_JSON__", json.dumps(metadata, ensure_ascii=False, separators=(",", ":")))
    html = html.replace("__REFERENCE_JSON__", json.dumps(reference, ensure_ascii=False, separators=(",", ":")))
    html = html.replace("__IMAGE_DATA_URI__", image_uri)
    picker_path = out / "route_picker.html"
    picker_path.write_text(html, encoding="utf-8")

    stamp(f"地图有效像素：{int(occupied.sum()):,}，使用顶点：{vertices_used:,}")
    print(f"ROUTE_MAP={image_path}")
    print(f"ROUTE_PICKER={picker_path}")
    print(f"METADATA={metadata_path}")
    print("ROUTE_PICKER_PASS")
except Exception as exc:
    failure = {
        "status": "FAILED",
        "exception_type": type(exc).__name__,
        "exception": str(exc),
        "traceback": traceback.format_exc(),
    }
    (out / "failure.json").write_text(json.dumps(failure, ensure_ascii=False, indent=2), encoding="utf-8")
    print("ROUTE_PICKER_EXCEPTION", file=__import__("sys").stderr)
    traceback.print_exc()
    raise
finally:
    app.close()
