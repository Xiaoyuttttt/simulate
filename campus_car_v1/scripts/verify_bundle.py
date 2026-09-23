#!/usr/bin/env python3
"""Static checks that do not require Isaac Sim."""

from __future__ import annotations

import csv
import json
import py_compile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
required = [
    ROOT / "README_zh.md",
    ROOT / "config/placement.json",
    ROOT / "config/waypoints.csv",
    ROOT / "config/original_html_camera_track.json",
    ROOT / "config/route_map.json",
    ROOT / "config/route_control_points.csv",
    ROOT / "route_picker/index.template.html",
    ROOT / "scripts/00_check_isaac51.sh",
    ROOT / "scripts/run_placement.sh",
    ROOT / "scripts/place_car_p1.py",
    ROOT / "scripts/ground_all_waypoints.py",
    ROOT / "scripts/run_ground_waypoints.sh",
    ROOT / "scripts/build_car_animation.py",
    ROOT / "scripts/run_car_animation.sh",
    ROOT / "scripts/render_first_person.py",
    ROOT / "scripts/run_first_person.sh",
    ROOT / "scripts/export_3dgs_camera_path.py",
    ROOT / "scripts/render_3dgs_camera_preview.py",
    ROOT / "scripts/run_3dgs_camera_preview.sh",
    ROOT / "scripts/run_3dgs_full_video.sh",
    ROOT / "scripts/export_html_reference_keyframes.py",
    ROOT / "scripts/run_html_reference_preview.sh",
    ROOT / "scripts/generate_route_picker.py",
    ROOT / "scripts/generate_colored_route_picker.py",
    ROOT / "scripts/run_route_picker.sh",
    ROOT / "scripts/ground_control_route.py",
    ROOT / "scripts/run_control_route.sh",
    ROOT / "scripts/render_control_route_preview.py",
    ROOT / "scripts/run_control_route_preview.sh",
    ROOT / "scripts/run_control_3dgs_hq_video.sh",
    ROOT / "scripts/run_control_3dgs_local_preview.sh",
    ROOT / "scripts/export_camera_height_comparison.py",
    ROOT / "scripts/run_camera_height_comparison.sh",
    ROOT / "scripts/crop_3dgs_route_region.py",
    ROOT / "scripts/run_high_camera_small_region.sh",
    ROOT / "scripts/run_camera_height_ab_videos.sh",
    ROOT / "scripts/run_straight_physics_demo.py",
    ROOT / "scripts/run_straight_physics_demo.sh",
]
missing = [str(path) for path in required if not path.is_file()]
if missing:
    raise SystemExit("缺少文件：\n" + "\n".join(missing))

cfg = json.loads((ROOT / "config/placement.json").read_text(encoding="utf-8"))
with (ROOT / "config/waypoints.csv").open("r", encoding="utf-8", newline="") as handle:
    rows = list(csv.DictReader(handle))
if len(rows) != 7:
    raise SystemExit(f"需要7个路径点，当前{len(rows)}个")
expected = {"point_id", "time_s", "x_hint", "y_hint", "z_hint"}
if set(rows[0]) != expected:
    raise SystemExit(f"路径列错误：{set(rows[0])}")
if not 0.0 <= float(cfg["car"]["ground_clearance_m"]) <= 0.05:
    raise SystemExit("ground_clearance_m必须在0到0.05之间")

py_compile.compile(str(ROOT / "scripts/place_car_p1.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/ground_all_waypoints.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/build_car_animation.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/render_first_person.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/export_3dgs_camera_path.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/render_3dgs_camera_preview.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/export_html_reference_keyframes.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/generate_route_picker.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/generate_colored_route_picker.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/ground_control_route.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/render_control_route_preview.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/export_camera_height_comparison.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/crop_3dgs_route_region.py"), doraise=True)
py_compile.compile(str(ROOT / "scripts/run_straight_physics_demo.py"), doraise=True)
print("CAMPUS_CAR_V1_STATIC_PASS")
