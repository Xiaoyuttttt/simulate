#!/usr/bin/env python3
"""Place a simple four-wheel car at P1 and snap it to the campus mesh."""

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


def load_config() -> tuple[argparse.Namespace, dict]:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config_path = Path(args.config).resolve()
    cfg = json.loads(config_path.read_text(encoding="utf-8"))
    cfg["_config_path"] = str(config_path)
    return args, cfg


_args, CFG = load_config()
RENDER = CFG["render"]
started = time.monotonic()


def stamp(message: str) -> None:
    print(f"[{time.monotonic() - started:8.2f}s] {message}", flush=True)


from isaacsim import SimulationApp


stamp("启动txy自己的 Isaac Sim 5.1（Headless、无NuRec、无VNC）")
simulation_app = SimulationApp(
    launch_config={
        "headless": True,
        "renderer": RENDER["renderer"],
        "width": int(RENDER["width"]),
        "height": int(RENDER["height"]),
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


def read_waypoints(path: Path) -> list[dict[str, float | int]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) < 2:
        raise ValueError("waypoints.csv 至少需要P1和P2")
    required = ("point_id", "time_s", "x_hint", "y_hint", "z_hint")
    if any(key not in rows[0] for key in required):
        raise ValueError(f"waypoints.csv 必须包含：{', '.join(required)}")
    parsed = []
    for row in rows:
        parsed.append(
            {
                "point_id": int(row["point_id"]),
                "time_s": float(row["time_s"]),
                "x_hint": float(row["x_hint"]),
                "y_hint": float(row["y_hint"]),
                "z_hint": float(row["z_hint"]),
            }
        )
    return parsed


async def convert_obj_to_usd(source: Path, destination: Path) -> None:
    import omni.kit.asset_converter

    destination.parent.mkdir(parents=True, exist_ok=True)
    converter = omni.kit.asset_converter.get_instance()
    context = omni.kit.asset_converter.AssetConverterContext()
    context.ignore_materials = True
    context.ignore_animation = True
    context.ignore_cameras = True
    context.single_mesh = False

    def progress(current_step: int, total: int) -> None:
        print(f"OBJ_CONVERT_PROGRESS={current_step}/{total}", flush=True)

    task = converter.create_converter_task(
        str(source), str(destination), progress, context
    )
    ok = await task.wait_until_finished()
    if not ok or not destination.is_file():
        status = task.get_status() if hasattr(task, "get_status") else "unknown"
        raise RuntimeError(f"OBJ转USD失败：status={status}")


def wait_for_task(task: asyncio.Task, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while not task.done() and time.monotonic() < deadline:
        simulation_app.update()
    if not task.done():
        task.cancel()
        raise TimeoutError(f"异步任务超过{timeout_s:.0f}秒")
    task.result()


def prepare_campus_asset() -> Path:
    campus_usd = Path(CFG["campus_usd"])
    if campus_usd.is_file() and campus_usd.stat().st_size > 0:
        print(f"CAMPUS_ASSET=USD:{campus_usd}", flush=True)
        return campus_usd

    campus_obj = Path(CFG["campus_obj"])
    if not campus_obj.is_file():
        raise FileNotFoundError(
            f"USD和OBJ都不存在：{campus_usd} ; {campus_obj}"
        )
    cached = Path(CFG["project_root"]) / "assets/cache/library_AG_from_obj.usd"
    if not cached.is_file():
        stamp(f"USD不可用，开始一次性转换OBJ：{campus_obj}")
        task = asyncio.ensure_future(convert_obj_to_usd(campus_obj, cached))
        wait_for_task(task, 900.0)
    print(f"CAMPUS_ASSET=CONVERTED_USD:{cached}", flush=True)
    return cached


def wait_stage_loading(context, timeout_s: float = 180.0) -> None:
    quiet = 0
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and quiet < 3:
        simulation_app.update()
        status = context.get_stage_loading_status()
        if len(status) >= 3 and status[2] == 0:
            quiet += 1
        else:
            quiet = 0
    if quiet < 3:
        raise TimeoutError("校园Stage依赖加载超时")


def transform_points(points: np.ndarray, matrix) -> np.ndarray:
    """Gf matrices use row-vector convention; translation is in the last row."""
    mat = np.asarray(matrix, dtype=np.float64)
    hom = np.ones((len(points), 4), dtype=np.float64)
    hom[:, :3] = points
    return (hom @ mat)[:, :3]


def build_query_grid(x: float, y: float, radius: float, step: float) -> np.ndarray:
    values = np.arange(-radius, radius + step * 0.5, step)
    offsets = np.asarray(
        [(dx, dy) for dx in values for dy in values if dx * dx + dy * dy <= radius * radius],
        dtype=np.float64,
    )
    order = np.argsort(np.linalg.norm(offsets, axis=1))
    return offsets[order] + np.asarray([x, y], dtype=np.float64)


def probe_ground(stage, p1: dict, output_dir: Path) -> tuple[dict, dict]:
    from pxr import Usd, UsdGeom

    probe = CFG["ground_probe"]
    query_xy = build_query_grid(
        p1["x_hint"], p1["y_hint"],
        float(probe["search_radius_m"]), float(probe["grid_step_m"]),
    )
    expected_z = p1["z_hint"] - float(probe["expected_camera_height_m"])
    slope_cos = math.cos(math.radians(float(probe["max_slope_deg"])))
    chunk_size = int(probe["triangle_chunk_size"])
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    candidates: list[dict] = []
    diagnostics = {
        "query_center_xy": [p1["x_hint"], p1["y_hint"]],
        "query_count": int(len(query_xy)),
        "expected_ground_z_from_hint": expected_z,
        "mesh_prims": [],
        "skipped_non_triangular_prims": [],
        "candidate_count": 0,
    }
    scene_min = np.asarray([np.inf, np.inf, np.inf], dtype=np.float64)
    scene_max = np.asarray([-np.inf, -np.inf, -np.inf], dtype=np.float64)

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        raw_points = mesh.GetPointsAttr().Get()
        raw_counts = mesh.GetFaceVertexCountsAttr().Get()
        raw_indices = mesh.GetFaceVertexIndicesAttr().Get()
        if not raw_points or not raw_counts or not raw_indices:
            continue
        points = np.asarray(raw_points, dtype=np.float64)
        counts = np.asarray(raw_counts, dtype=np.int64)
        indices = np.asarray(raw_indices, dtype=np.int64)
        world = transform_points(points, xform_cache.GetLocalToWorldTransform(prim))
        scene_min = np.minimum(scene_min, world.min(axis=0))
        scene_max = np.maximum(scene_max, world.max(axis=0))
        prim_info = {
            "path": str(prim.GetPath()),
            "vertices": int(len(world)),
            "faces": int(len(counts)),
            "bounds_min": world.min(axis=0).tolist(),
            "bounds_max": world.max(axis=0).tolist(),
        }
        diagnostics["mesh_prims"].append(prim_info)

        if not np.all(counts == 3) or len(indices) != 3 * len(counts):
            diagnostics["skipped_non_triangular_prims"].append(str(prim.GetPath()))
            continue
        triangles = indices.reshape(-1, 3)
        for begin in range(0, len(triangles), chunk_size):
            tri = triangles[begin : begin + chunk_size]
            p0, p2a, p2b = world[tri[:, 0]], world[tri[:, 1]], world[tri[:, 2]]
            e0 = p2a - p0
            e1 = p2b - p0
            cross = np.cross(e0, e1)
            norm = np.linalg.norm(cross, axis=1)
            up = np.abs(cross[:, 2]) / np.maximum(norm, 1e-12)
            planar = (norm > 1e-10) & (up >= slope_cos)
            if not planar.any():
                continue
            p0 = p0[planar]
            e0 = e0[planar]
            e1 = e1[planar]
            up = up[planar]
            denom = e0[:, 0] * e1[:, 1] - e0[:, 1] * e1[:, 0]
            valid_denom = np.abs(denom) > 1e-12
            if not valid_denom.any():
                continue
            p0, e0, e1, up, denom = (
                item[valid_denom] for item in (p0, e0, e1, up, denom)
            )
            for qx, qy in query_xy:
                vx = qx - p0[:, 0]
                vy = qy - p0[:, 1]
                a = (vx * e1[:, 1] - vy * e1[:, 0]) / denom
                b = (e0[:, 0] * vy - e0[:, 1] * vx) / denom
                inside = (a >= -1e-7) & (b >= -1e-7) & (a + b <= 1.0000001)
                if not inside.any():
                    continue
                z_values = p0[inside, 2] + a[inside] * e0[inside, 2] + b[inside] * e1[inside, 2]
                up_values = up[inside]
                for z, up_value in zip(z_values.tolist(), up_values.tolist()):
                    height_error = abs(z - expected_z)
                    if height_error > float(probe["max_height_error_m"]):
                        continue
                    distance = math.hypot(qx - p1["x_hint"], qy - p1["y_hint"])
                    slope_deg = math.degrees(math.acos(min(max(up_value, 0.0), 1.0)))
                    candidates.append(
                        {
                            "x": qx, "y": qy, "z": z,
                            "distance_xy_m": distance,
                            "slope_deg": slope_deg,
                            "height_error_m": height_error,
                            "mesh_prim": str(prim.GetPath()),
                            "score": distance + 0.25 * height_error + 0.02 * slope_deg,
                        }
                    )

    diagnostics["scene_bounds_min"] = scene_min.tolist()
    diagnostics["scene_bounds_max"] = scene_max.tolist()
    diagnostics["p1_xy_inside_scene_bounds"] = bool(
        scene_min[0] <= p1["x_hint"] <= scene_max[0]
        and scene_min[1] <= p1["y_hint"] <= scene_max[1]
    )
    candidates.sort(key=lambda item: item["score"])
    diagnostics["candidate_count"] = len(candidates)
    diagnostics["best_candidates"] = candidates[:20]
    diag_path = output_dir / "ground_probe_diagnostics.json"
    diag_path.write_text(json.dumps(diagnostics, ensure_ascii=False, indent=2), encoding="utf-8")

    if not diagnostics["mesh_prims"]:
        raise RuntimeError("Stage中没有可探测的UsdGeom.Mesh；详见ground_probe_diagnostics.json")
    if not diagnostics["p1_xy_inside_scene_bounds"]:
        raise RuntimeError("P1的XY不在校园Mesh包围盒内；需要先做坐标系校准")
    if not candidates:
        raise RuntimeError("P1附近没有可靠的向上三角面；未放置小车，详见诊断JSON")
    return candidates[0], diagnostics


def set_display_color(gprim, rgb: tuple[float, float, float]) -> None:
    from pxr import Gf, UsdGeom, Vt

    UsdGeom.Gprim(gprim.GetPrim()).CreateDisplayColorAttr(
        Vt.Vec3fArray([Gf.Vec3f(*rgb)])
    )


def add_translate(schema, value) -> None:
    from pxr import UsdGeom

    UsdGeom.Xformable(schema.GetPrim()).AddTranslateOp().Set(value)


def add_scale(schema, value) -> None:
    from pxr import UsdGeom

    UsdGeom.Xformable(schema.GetPrim()).AddScaleOp().Set(value)


def add_simple_car(stage, x: float, y: float, root_z: float, yaw: float) -> list[dict]:
    from pxr import Gf, UsdGeom

    car_cfg = CFG["car"]
    length = float(car_cfg["length_m"])
    width = float(car_cfg["width_m"])
    chassis_h = float(car_cfg["chassis_height_m"])
    wheel_r = float(car_cfg["wheel_radius_m"])
    wheel_w = float(car_cfg["wheel_width_m"])

    root = UsdGeom.Xform.Define(stage, "/World/Car")
    root_xformable = UsdGeom.Xformable(root.GetPrim())
    root_xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, root_z))
    root_xformable.AddOrientOp().Set(
        Gf.Quatf(math.cos(yaw * 0.5), Gf.Vec3f(0.0, 0.0, math.sin(yaw * 0.5)))
    )

    chassis = UsdGeom.Cube.Define(stage, "/World/Car/Chassis")
    chassis.CreateSizeAttr(1.0)
    add_translate(chassis, Gf.Vec3d(0.0, 0.0, wheel_r + 0.02))
    add_scale(chassis, Gf.Vec3d(length, width, chassis_h))
    set_display_color(chassis, (0.08, 0.28, 0.85))

    wheel_records = []
    axle_x = length * 0.34
    wheel_y = width * 0.5 + wheel_w * 0.40
    for name, wx, wy in (
        ("FrontLeft", axle_x, wheel_y),
        ("FrontRight", axle_x, -wheel_y),
        ("RearLeft", -axle_x, wheel_y),
        ("RearRight", -axle_x, -wheel_y),
    ):
        wheel = UsdGeom.Cylinder.Define(stage, f"/World/Car/Wheels/{name}")
        wheel.CreateAxisAttr(UsdGeom.Tokens.y)
        wheel.CreateRadiusAttr(wheel_r)
        wheel.CreateHeightAttr(wheel_w)
        add_translate(wheel, Gf.Vec3d(wx, wy, 0.0))
        set_display_color(wheel, (0.025, 0.025, 0.025))
        wheel_records.append({"name": name, "local_center": [wx, wy, 0.0]})

    mast = UsdGeom.Cube.Define(stage, "/World/Car/CameraMount")
    mast.CreateSizeAttr(1.0)
    add_translate(mast, Gf.Vec3d(0.25, 0.0, 0.60))
    add_scale(mast, Gf.Vec3d(0.08, 0.08, 0.80))
    set_display_color(mast, (0.9, 0.5, 0.04))
    return wheel_records


async def capture_view(viewport, output: Path, eye: np.ndarray, target: np.ndarray) -> None:
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import capture_viewport_to_file

    set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp", viewport_api=viewport)
    for _ in range(8):
        simulation_app.update()
    capture = capture_viewport_to_file(viewport, str(output), is_hdr=False)
    # Kit advances this awaitable from SimulationApp.update(). Wrapping it in
    # asyncio.wait_for() requires a conventional running loop and fails under
    # Isaac Sim 5.1 with "no running event loop". The caller already enforces
    # a hard 185-second deadline while pumping Kit updates.
    await capture.wait_for_result()


def capture_views(position: np.ndarray, yaw: float, output_dir: Path) -> None:
    from omni.kit.viewport.utility import get_active_viewport

    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("Headless模式没有active viewport")
    viewport.set_texture_resolution((int(RENDER["width"]), int(RENDER["height"])))
    heading = np.asarray([math.cos(yaw), math.sin(yaw), 0.0])
    side = np.asarray([-heading[1], heading[0], 0.0])
    target = position + np.asarray([0.0, 0.0, 0.25])
    views = {
        "close.png": position - 3.0 * heading + 2.0 * side + np.asarray([0.0, 0.0, 1.5]),
        "side.png": position + 4.0 * side + np.asarray([0.0, 0.0, 1.3]),
        "top.png": position + np.asarray([0.0, 0.0, 8.0]),
    }
    for name, eye in views.items():
        stamp(f"截图：{name}")
        task = asyncio.ensure_future(capture_view(viewport, output_dir / name, eye, target))
        wait_for_task(task, 185.0)


exit_code = 1
try:
    import importlib.metadata
    import omni.usd
    from pxr import Gf, UsdGeom, UsdLux

    version = importlib.metadata.version("isaacsim")
    print(f"ISAAC_SIM_VERSION={version}", flush=True)
    if not version.startswith("5.1."):
        raise RuntimeError(f"要求Isaac Sim 5.1.x，当前为{version}；禁止使用其他环境")

    output_dir = Path(CFG["output_dir"])
    output_dir.mkdir(parents=True, exist_ok=True)
    stage_path = Path(CFG["stage_path"])
    stage_path.parent.mkdir(parents=True, exist_ok=True)
    waypoints = read_waypoints(Path(CFG["waypoints_csv"]))
    p1, p2 = waypoints[0], waypoints[1]
    delta = np.asarray([p2["x_hint"] - p1["x_hint"], p2["y_hint"] - p1["y_hint"]])
    if np.linalg.norm(delta) < 1e-6:
        raise ValueError("P1和P2的XY重合，无法确定车头方向")
    yaw = math.atan2(delta[1], delta[0])

    campus_asset = prepare_campus_asset()
    context = omni.usd.get_context()
    context.new_stage()
    simulation_app.update()
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    campus = UsdGeom.Xform.Define(stage, "/World/Campus")
    if not campus.GetPrim().GetReferences().AddReference(str(campus_asset)):
        raise RuntimeError(f"无法引用校园USD：{campus_asset}")
    wait_stage_loading(context)
    stamp("校园Stage加载完成，开始探地")

    ground, diagnostics = probe_ground(stage, p1, output_dir)
    car_cfg = CFG["car"]
    wheel_r = float(car_cfg["wheel_radius_m"])
    clearance = float(car_cfg["ground_clearance_m"])
    root_z = float(ground["z"]) + wheel_r + clearance
    wheel_records = add_simple_car(stage, ground["x"], ground["y"], root_z, yaw)

    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
    dome.CreateIntensityAttr(650.0)
    distant = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    distant.CreateIntensityAttr(2200.0)
    UsdGeom.Xformable(distant.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(-45.0, 25.0, 20.0)
    )

    stage.GetRootLayer().Export(str(stage_path))
    stamp(f"已保存Stage：{stage_path}")
    position = np.asarray([ground["x"], ground["y"], root_z], dtype=np.float64)
    capture_views(position, yaw, output_dir)

    qx, qy = 0.0, 0.0
    qz, qw = math.sin(yaw * 0.5), math.cos(yaw * 0.5)
    wheel_gap = root_z - wheel_r - float(ground["z"])
    placement = {
        "status": "PASS",
        "frame": "Isaac/Usd, Z-up, meters",
        "quaternion_order": "xyzw",
        "isaac_sim_version": version,
        "campus_asset": str(campus_asset),
        "source_waypoint": p1,
        "heading_source_waypoint": p2,
        "original_z_used_for_placement": False,
        "robot_pose": {
            "x": float(ground["x"]), "y": float(ground["y"]), "z": root_z,
            "qx": qx, "qy": qy, "qz": qz, "qw": qw,
        },
        "yaw_rad": yaw,
        "ground_z": float(ground["z"]),
        "ground_probe": ground,
        "wheel_lowest_point_z": root_z - wheel_r,
        "wheel_ground_gap_m": wheel_gap,
        "wheel_records": wheel_records,
        "stage_path": str(stage_path),
        "check_images": [str(output_dir / name) for name in ("close.png", "side.png", "top.png")],
        "validation": {
            "p1_xy_inside_scene_bounds": diagnostics["p1_xy_inside_scene_bounds"],
            "ground_candidate_found": True,
            "ground_slope_deg": ground["slope_deg"],
            "wheel_not_below_ground": wheel_gap >= -1e-4,
            "wheel_gap_le_5cm": wheel_gap <= 0.05,
            "heading_defined_by_p1_p2": True,
        },
    }
    if not all(placement["validation"].values()):
        placement["status"] = "FAILED_VALIDATION"
    (output_dir / "placement.json").write_text(
        json.dumps(placement, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(placement, ensure_ascii=False, indent=2), flush=True)
    if placement["status"] != "PASS":
        raise RuntimeError("放置结果未通过几何验收")
    print("CAMPUS_CAR_P1_PLACEMENT_PASS", flush=True)
    exit_code = 0
except BaseException as exc:
    exit_code = 20
    print("CAMPUS_CAR_P1_EXCEPTION", flush=True)
    traceback.print_exc()
    try:
        failure_path = Path(CFG["output_dir"]) / "failure.json"
        failure_path.parent.mkdir(parents=True, exist_ok=True)
        failure_path.write_text(
            json.dumps(
                {
                    "status": "FAILED",
                    "exception_type": type(exc).__name__,
                    "exception": str(exc),
                    "traceback": traceback.format_exc(),
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        print(f"FAILURE_JSON={failure_path}", flush=True)
    except BaseException:
        traceback.print_exc()
finally:
    try:
        simulation_app.close()
    except BaseException:
        print("SIMULATION_APP_CLOSE_EXCEPTION", flush=True)
        traceback.print_exc()

sys.exit(exit_code)
