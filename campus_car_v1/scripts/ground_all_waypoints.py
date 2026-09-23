#!/usr/bin/env python3
"""Snap seven approximate XY waypoints to one continuous traversable surface."""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
import sys
import time
import traceback
from pathlib import Path

import numpy as np


parser = argparse.ArgumentParser()
parser.add_argument("--config", required=True)
ARGS = parser.parse_args()
CFG = json.loads(Path(ARGS.config).read_text(encoding="utf-8"))
ROOT = Path(CFG["project_root"])
OUTPUT_DIR = ROOT / "outputs/ground_waypoints"
STAGE_PATH = ROOT / "stages/campus_ground_waypoints.usd"
started = time.monotonic()


def stamp(message: str) -> None:
    print(f"[{time.monotonic() - started:8.2f}s] {message}", flush=True)


from isaacsim import SimulationApp


simulation_app = SimulationApp(
    launch_config={
        "headless": True,
        "renderer": CFG["render"]["renderer"],
        "width": int(CFG["render"]["width"]),
        "height": int(CFG["render"]["height"]),
        "multi_gpu": False,
        "max_gpu_count": 1,
        "sync_loads": True,
        "disable_viewport_updates": False,
        "extra_args": [
            "--/renderer/multiGpu/enabled=false",
            "--/renderer/multiGpu/autoEnable=false",
            "--/renderer/multiGpu/maxGpuCount=1",
        ],
    }
)


def read_waypoints(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != 7:
        raise ValueError(f"需要恰好7个路径点，当前为{len(rows)}个")
    parsed = []
    for row in rows:
        parsed.append({
            "point_id": int(row["point_id"]),
            "time_s": float(row["time_s"]),
            "x_hint": float(row["x_hint"]),
            "y_hint": float(row["y_hint"]),
            "z_hint": float(row["z_hint"]),
        })
    return parsed


def wait_stage(context, timeout_s: float = 180.0) -> None:
    quiet = 0
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and quiet < 3:
        simulation_app.update()
        status = context.get_stage_loading_status()
        quiet = quiet + 1 if len(status) >= 3 and status[2] == 0 else 0
    if quiet < 3:
        raise TimeoutError("校园Stage依赖加载超时")


def transform_points(points: np.ndarray, matrix) -> np.ndarray:
    hom = np.ones((len(points), 4), dtype=np.float64)
    hom[:, :3] = points
    return (hom @ np.asarray(matrix, dtype=np.float64))[:, :3]


def query_grid(x: float, y: float, radius: float, step: float) -> np.ndarray:
    axis = np.arange(-radius, radius + 0.5 * step, step)
    offsets = np.asarray(
        [(dx, dy) for dx in axis for dy in axis if dx * dx + dy * dy <= radius * radius],
        dtype=np.float64,
    )
    offsets = offsets[np.argsort(np.linalg.norm(offsets, axis=1))]
    return offsets + np.asarray([x, y])


def collect_candidates(stage, waypoints: list[dict]) -> tuple[list[list[dict]], dict]:
    from pxr import Usd, UsdGeom

    probe = CFG["ground_probe"]
    radius = float(probe["search_radius_m"])
    step = float(probe["grid_step_m"])
    max_slope = float(probe["max_slope_deg"])
    slope_cos = math.cos(math.radians(max_slope))
    p1_ground_z = float(probe["calibrated_p1_ground_z"])
    relative_z_hint_weight = float(probe.get("relative_z_hint_weight", 0.25))
    chunk_size = int(probe["triangle_chunk_size"])
    grids = [query_grid(w["x_hint"], w["y_hint"], radius, step) for w in waypoints]
    # The supplied Z values came from the old camera/path convention and are
    # not absolute ground heights.  P1 has already been measured successfully
    # on this mesh, so use it as the vertical datum.  Preserve only relative Z
    # deltas as a weak ranking cue; never reject a surface because of z_hint.
    expected_z = [
        p1_ground_z + (w["z_hint"] - waypoints[0]["z_hint"])
        for w in waypoints
    ]
    all_candidates: list[list[dict]] = [[] for _ in waypoints]
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    scene_min = np.full(3, np.inf)
    scene_max = np.full(3, -np.inf)
    mesh_info = []

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        points_raw = mesh.GetPointsAttr().Get()
        counts_raw = mesh.GetFaceVertexCountsAttr().Get()
        indices_raw = mesh.GetFaceVertexIndicesAttr().Get()
        if not points_raw or not counts_raw or not indices_raw:
            continue
        counts = np.asarray(counts_raw, dtype=np.int64)
        indices = np.asarray(indices_raw, dtype=np.int64)
        if not np.all(counts == 3) or len(indices) != len(counts) * 3:
            raise RuntimeError(f"Mesh不是全三角面：{prim.GetPath()}")
        world = transform_points(
            np.asarray(points_raw, dtype=np.float64),
            xform_cache.GetLocalToWorldTransform(prim),
        )
        scene_min = np.minimum(scene_min, world.min(axis=0))
        scene_max = np.maximum(scene_max, world.max(axis=0))
        triangles = indices.reshape(-1, 3)
        mesh_info.append({
            "path": str(prim.GetPath()),
            "vertices": len(world),
            "faces": len(triangles),
            "bounds_min": world.min(axis=0).tolist(),
            "bounds_max": world.max(axis=0).tolist(),
        })
        stamp(f"扫描Mesh：{prim.GetPath()}，{len(triangles)}个三角面")

        for begin in range(0, len(triangles), chunk_size):
            tri = triangles[begin:begin + chunk_size]
            a, b, c = world[tri[:, 0]], world[tri[:, 1]], world[tri[:, 2]]
            tri_min = np.minimum(np.minimum(a[:, :2], b[:, :2]), c[:, :2])
            tri_max = np.maximum(np.maximum(a[:, :2], b[:, :2]), c[:, :2])
            for wi, waypoint in enumerate(waypoints):
                x, y = waypoint["x_hint"], waypoint["y_hint"]
                near = (
                    (tri_max[:, 0] >= x - radius) & (tri_min[:, 0] <= x + radius)
                    & (tri_max[:, 1] >= y - radius) & (tri_min[:, 1] <= y + radius)
                )
                if not near.any():
                    continue
                p0, p1, p2 = a[near], b[near], c[near]
                e0, e1 = p1 - p0, p2 - p0
                normal = np.cross(e0, e1)
                normal_len = np.linalg.norm(normal, axis=1)
                up = np.abs(normal[:, 2]) / np.maximum(normal_len, 1e-12)
                planar = (normal_len > 1e-10) & (up >= slope_cos)
                if not planar.any():
                    continue
                p0, e0, e1, up = p0[planar], e0[planar], e1[planar], up[planar]
                denom = e0[:, 0] * e1[:, 1] - e0[:, 1] * e1[:, 0]
                valid = np.abs(denom) > 1e-12
                p0, e0, e1, up, denom = (
                    value[valid] for value in (p0, e0, e1, up, denom)
                )
                for qx, qy in grids[wi]:
                    vx, vy = qx - p0[:, 0], qy - p0[:, 1]
                    u = (vx * e1[:, 1] - vy * e1[:, 0]) / denom
                    v = (e0[:, 0] * vy - e0[:, 1] * vx) / denom
                    inside = (u >= -1e-7) & (v >= -1e-7) & (u + v <= 1.0000001)
                    if not inside.any():
                        continue
                    z_values = p0[inside, 2] + u[inside] * e0[inside, 2] + v[inside] * e1[inside, 2]
                    for z, up_value in zip(z_values.tolist(), up[inside].tolist()):
                        height_error = abs(z - expected_z[wi])
                        distance = math.hypot(qx - x, qy - y)
                        slope = math.degrees(math.acos(min(max(up_value, 0.0), 1.0)))
                        all_candidates[wi].append({
                            "x": qx, "y": qy, "z": z,
                            "distance_xy_m": distance,
                            "slope_deg": slope,
                            "height_error_m": height_error,
                            "mesh_prim": str(prim.GetPath()),
                            "local_cost": (
                                distance + 0.02 * slope
                                + relative_z_hint_weight * height_error
                            ),
                        })
            if begin % (chunk_size * 20) == 0:
                stamp(f"三角面进度：{min(begin + chunk_size, len(triangles))}/{len(triangles)}")

    # Keep elevation diversity.  Simply taking the globally best 40 candidates
    # can retain only a roof/tree layer close to the XY hint and discard the
    # actual road slightly farther away.  Deduplicate triangle-edge hits, then
    # preserve the best candidates from every 25 cm elevation band.
    z_bin_m = float(probe.get("candidate_z_bin_m", 0.25))
    per_bin = int(probe.get("candidates_per_z_bin", 8))
    max_candidates = int(probe.get("max_candidates_per_waypoint", 320))
    raw_candidate_counts = [len(items) for items in all_candidates]
    for waypoint_index, candidates in enumerate(all_candidates):
        deduplicated = {}
        for item in candidates:
            key = (round(item["x"], 4), round(item["y"], 4), round(item["z"], 4))
            previous = deduplicated.get(key)
            if previous is None or item["local_cost"] < previous["local_cost"]:
                deduplicated[key] = item
        bins = {}
        for item in deduplicated.values():
            z_key = math.floor(item["z"] / z_bin_m)
            bins.setdefault(z_key, []).append(item)
        diverse = []
        for items in bins.values():
            items.sort(key=lambda value: value["local_cost"])
            diverse.extend(items[:per_bin])
        diverse.sort(key=lambda value: value["local_cost"])
        all_candidates[waypoint_index] = diverse[:max_candidates]
    diagnostics = {
        "scene_bounds_min": scene_min.tolist(),
        "scene_bounds_max": scene_max.tolist(),
        "mesh_prims": mesh_info,
        "z_policy": {
            "absolute_z_hint_used_for_filtering": False,
            "calibrated_p1_ground_z": p1_ground_z,
            "relative_expected_ground_z": expected_z,
            "relative_z_hint_weight": relative_z_hint_weight,
        },
        "candidate_counts": [len(c) for c in all_candidates],
        "raw_candidate_counts": raw_candidate_counts,
        "candidate_retention": {
            "z_bin_m": z_bin_m,
            "candidates_per_z_bin": per_bin,
            "max_candidates_per_waypoint": max_candidates,
        },
        "candidates": all_candidates,
    }
    return all_candidates, diagnostics


def choose_continuous_route(candidates: list[list[dict]]) -> tuple[list[dict], dict]:
    if any(not options for options in candidates):
        missing = [index + 1 for index, options in enumerate(candidates) if not options]
        raise RuntimeError(f"这些路径点附近没有地面候选：{missing}")
    costs = [np.full(len(options), np.inf) for options in candidates]
    parents = [np.full(len(options), -1, dtype=np.int64) for options in candidates]
    costs[0] = np.asarray([item["local_cost"] for item in candidates[0]])
    probe = CFG["ground_probe"]
    max_route_grade = float(probe.get("max_route_grade", 0.40))
    min_transition_height = float(probe.get("min_transition_height_m", 1.0))
    rejected = 0
    for index in range(1, len(candidates)):
        for j, current in enumerate(candidates[index]):
            for k, previous in enumerate(candidates[index - 1]):
                horizontal = math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
                dz = abs(current["z"] - previous["z"])
                max_dz = max(min_transition_height, horizontal * max_route_grade)
                if dz > max_dz:
                    rejected += 1
                    continue
                lateral_change = abs(current["x"] - previous["x"])
                transition = 3.0 * dz + 0.25 * lateral_change
                value = costs[index - 1][k] + current["local_cost"] + transition
                if value < costs[index][j]:
                    costs[index][j] = value
                    parents[index][j] = k
        if not np.isfinite(costs[index]).any():
            raise RuntimeError(f"P{index}到P{index + 1}之间没有满足连续坡度的地面路径")
    selected_indices = [int(np.argmin(costs[-1]))]
    for index in range(len(candidates) - 1, 0, -1):
        selected_indices.append(int(parents[index][selected_indices[-1]]))
    selected_indices.reverse()
    selected = [candidates[i][choice] for i, choice in enumerate(selected_indices)]
    return selected, {
        "selected_candidate_indices": selected_indices,
        "total_cost": float(np.min(costs[-1])),
        "rejected_height_transitions": rejected,
        "max_route_grade": max_route_grade,
        "min_transition_height_m": min_transition_height,
    }


def add_route_visuals(stage, selected: list[dict]) -> None:
    from pxr import Gf, UsdGeom, Vt

    points = [Gf.Vec3f(item["x"], item["y"], item["z"] + 0.12) for item in selected]
    curve = UsdGeom.BasisCurves.Define(stage, "/World/GroundRoute")
    curve.CreateTypeAttr(UsdGeom.Tokens.linear)
    curve.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)
    curve.CreateCurveVertexCountsAttr(Vt.IntArray([len(points)]))
    curve.CreatePointsAttr(Vt.Vec3fArray(points))
    curve.CreateWidthsAttr(Vt.FloatArray([0.16] * len(points)))
    UsdGeom.Gprim(curve.GetPrim()).CreateDisplayColorAttr(
        Vt.Vec3fArray([Gf.Vec3f(0.9, 0.06, 0.03)])
    )
    for index, item in enumerate(selected, start=1):
        marker = UsdGeom.Sphere.Define(stage, f"/World/RouteMarkers/P{index}")
        marker.CreateRadiusAttr(0.28)
        UsdGeom.Xformable(marker.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(item["x"], item["y"], item["z"] + 0.28)
        )
        UsdGeom.Gprim(marker.GetPrim()).CreateDisplayColorAttr(
            Vt.Vec3fArray([Gf.Vec3f(0.95, 0.12, 0.02)])
        )


def wait_task(task: asyncio.Task, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while not task.done() and time.monotonic() < deadline:
        simulation_app.update()
    if not task.done():
        task.cancel()
        raise TimeoutError("路线截图超时")
    task.result()


async def capture(viewport, path: Path, eye: np.ndarray, target: np.ndarray) -> None:
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import capture_viewport_to_file

    set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp", viewport_api=viewport)
    for _ in range(8):
        simulation_app.update()
    result = capture_viewport_to_file(viewport, str(path), is_hdr=False)
    await result.wait_for_result()


def capture_route(selected: list[dict]) -> None:
    from omni.kit.viewport.utility import get_active_viewport

    xyz = np.asarray([[p["x"], p["y"], p["z"]] for p in selected])
    center = xyz.mean(axis=0)
    span = max(float(np.ptp(xyz[:, 0])), float(np.ptp(xyz[:, 1])), 10.0)
    viewport = get_active_viewport()
    viewport.set_texture_resolution((int(CFG["render"]["width"]), int(CFG["render"]["height"])))
    views = {
        "route_overview.png": center + np.asarray([0.0, 0.0, span * 1.25]),
        "route_side.png": center + np.asarray([span * 0.75, 0.0, span * 0.30]),
    }
    for name, eye in views.items():
        stamp(f"截图：{name}")
        task = asyncio.ensure_future(capture(viewport, OUTPUT_DIR / name, eye, center))
        wait_task(task, 185.0)


exit_code = 1
try:
    import importlib.metadata
    import omni.usd
    from pxr import UsdGeom, UsdLux

    version = importlib.metadata.version("isaacsim")
    if not version.startswith("5.1."):
        raise RuntimeError(f"要求Isaac Sim 5.1.x，当前为{version}")
    campus = Path(CFG["campus_usd"])
    if not campus.is_file():
        raise FileNotFoundError(f"第二阶段要求第一阶段已验证的校园USD：{campus}")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    STAGE_PATH.parent.mkdir(parents=True, exist_ok=True)
    waypoints = read_waypoints(Path(CFG["waypoints_csv"]))

    context = omni.usd.get_context()
    context.new_stage()
    simulation_app.update()
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    campus_root = UsdGeom.Xform.Define(stage, "/World/Campus")
    campus_root.GetPrim().GetReferences().AddReference(str(campus))
    wait_stage(context)
    stamp("校园加载完成，开始七点联合探地")

    candidates, diagnostics = collect_candidates(stage, waypoints)
    diagnostics_path = OUTPUT_DIR / "ground_waypoints_diagnostics.json"
    # Preserve the expensive probe results even if continuous-route selection
    # fails.  This makes a failed remote run actionable without rerunning the
    # 1.1 GB campus mesh just to recover candidate heights.
    diagnostics_path.write_text(
        json.dumps(diagnostics, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    selected_surfaces, route_diag = choose_continuous_route(candidates)
    # Route XY is user-authored and must never be moved by ground probing.
    # Nearby mesh samples provide elevation and slope only.  This prevents a
    # flat sidewalk or side road from silently replacing the intended road.
    selected = []
    for source, surface in zip(waypoints, selected_surfaces):
        locked = dict(surface)
        locked["probe_x"] = surface["x"]
        locked["probe_y"] = surface["y"]
        locked["x"] = source["x_hint"]
        locked["y"] = source["y_hint"]
        locked["xy_locked_to_original"] = True
        selected.append(locked)
    diagnostics["route_selection"] = route_diag
    diagnostics["selected_surfaces"] = selected_surfaces
    diagnostics["selected"] = selected
    diagnostics["xy_policy"] = "LOCK_ORIGINAL_XY; nearby mesh samples supply Z only"
    diagnostics_path.write_text(
        json.dumps(diagnostics, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    rows = []
    for source, ground in zip(waypoints, selected):
        rows.append({
            "point_id": source["point_id"], "time_s": source["time_s"],
            "original_x": source["x_hint"], "original_y": source["y_hint"], "original_z": source["z_hint"],
            "ground_x": ground["x"], "ground_y": ground["y"], "ground_z": ground["z"],
            "probe_x": ground["probe_x"], "probe_y": ground["probe_y"],
            "xy_correction_m": ground["distance_xy_m"], "slope_deg": ground["slope_deg"],
            "xy_locked_to_original": True,
            "status": "PASS",
        })
    with (OUTPUT_DIR / "ground_waypoints.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    add_route_visuals(stage, selected)
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
    dome.CreateIntensityAttr(650.0)
    stage.GetRootLayer().Export(str(STAGE_PATH))
    capture_route(selected)
    print(json.dumps({"selected": selected, "route": route_diag}, ensure_ascii=False, indent=2), flush=True)
    print("GROUND_WAYPOINTS_PASS", flush=True)
    exit_code = 0
except BaseException as exc:
    exit_code = 30
    print("GROUND_WAYPOINTS_EXCEPTION", flush=True)
    traceback.print_exc()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUTPUT_DIR / "failure.json").write_text(json.dumps({
        "status": "FAILED", "exception_type": type(exc).__name__,
        "exception": str(exc), "traceback": traceback.format_exc(),
    }, ensure_ascii=False, indent=2), encoding="utf-8")
finally:
    try:
        simulation_app.close()
    except BaseException:
        traceback.print_exc()

sys.exit(exit_code)
