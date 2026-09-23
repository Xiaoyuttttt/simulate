#!/usr/bin/env python3
"""Create a frame-by-frame kinematic car animation from grounded waypoints."""
from __future__ import annotations
import argparse, csv, json, math, sys, traceback
from pathlib import Path
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--config", required=True)
parser.add_argument("--fps", type=float, default=15.0)
parser.add_argument("--source-csv", default=None)
parser.add_argument("--output-subdir", default="car_animation")
parser.add_argument("--stage-name", default="campus_car_animated.usd")
args = parser.parse_args()
cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
root = Path(cfg["project_root"])
source_csv = Path(args.source_csv) if args.source_csv else root / "outputs/ground_waypoints/ground_waypoints.csv"
output_dir = root / "outputs" / args.output_subdir
stage_path = root / "stages" / args.stage_name

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True, "renderer": cfg["render"]["renderer"],
                                "width": 640, "height": 360, "multi_gpu": False,
                                "max_gpu_count": 1})

def read_grounded():
    if not source_csv.is_file():
        raise FileNotFoundError(f"请先完成第二阶段：{source_csv}")
    with source_csv.open("r", encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 2:
        raise RuntimeError(f"地面路线至少需要2行，当前为{len(rows)}")
    x_key = "ground_x" if "ground_x" in rows[0] else "x"
    y_key = "ground_y" if "ground_y" in rows[0] else "y"
    return [{"time_s": float(r["time_s"]), "x": float(r[x_key]),
             "y": float(r[y_key]), "ground_z": float(r["ground_z"])} for r in rows]

def make_trajectory(points, fps):
    kt = np.asarray([p["time_s"] for p in points]); kx = np.asarray([p["x"] for p in points])
    ky = np.asarray([p["y"] for p in points]); kz = np.asarray([p["ground_z"] for p in points])
    if np.any(np.diff(kt) <= 0): raise ValueError("关键帧时间必须严格递增")
    count = int(round((kt[-1] - kt[0]) * fps)) + 1
    t = np.linspace(kt[0], kt[-1], count)
    x, y, gz = np.interp(t, kt, kx), np.interp(t, kt, ky), np.interp(t, kt, kz)
    yaw = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    body_z = gz + float(cfg["car"]["wheel_radius_m"]) + float(cfg["car"]["ground_clearance_m"])
    distance = np.zeros(count); distance[1:] = np.cumsum(np.hypot(np.diff(x), np.diff(y)))
    return [{"frame": i, "time_s": float(t[i]), "x": float(x[i]), "y": float(y[i]),
             "ground_z": float(gz[i]), "body_z": float(body_z[i]), "yaw_rad": float(yaw[i]),
             "distance_m": float(distance[i]),
             "wheel_ground_gap_m": float(cfg["car"]["ground_clearance_m"])} for i in range(count)]

def color(prim, rgb):
    from pxr import Gf, UsdGeom, Vt
    UsdGeom.Gprim(prim.GetPrim()).CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*rgb)]))

def translate(prim, value):
    from pxr import UsdGeom
    UsdGeom.Xformable(prim.GetPrim()).AddTranslateOp().Set(value)

def scale(prim, value):
    from pxr import UsdGeom
    UsdGeom.Xformable(prim.GetPrim()).AddScaleOp().Set(value)

def add_car(stage, trajectory, fps):
    from pxr import Gf, UsdGeom
    c = cfg["car"]; length, width = float(c["length_m"]), float(c["width_m"])
    wheel_r, wheel_w = float(c["wheel_radius_m"]), float(c["wheel_width_m"])
    root_prim = UsdGeom.Xform.Define(stage, "/World/Car")
    xf = UsdGeom.Xformable(root_prim.GetPrim()); move = xf.AddTranslateOp(); turn = xf.AddOrientOp()
    for p in trajectory:
        tc = float(p["frame"]); half = p["yaw_rad"] * 0.5
        move.Set(Gf.Vec3d(p["x"], p["y"], p["body_z"]), tc)
        turn.Set(Gf.Quatf(math.cos(half), Gf.Vec3f(0, 0, math.sin(half))), tc)
    chassis = UsdGeom.Cube.Define(stage, "/World/Car/Chassis"); chassis.CreateSizeAttr(1.0)
    translate(chassis, Gf.Vec3d(0, 0, wheel_r + 0.02))
    scale(chassis, Gf.Vec3d(length, width, float(c["chassis_height_m"]))); color(chassis, (0.08, 0.28, 0.85))
    axle_x, wheel_y = length * 0.34, width * 0.5 + wheel_w * 0.40
    for name, wx, wy in (("FrontLeft", axle_x, wheel_y), ("FrontRight", axle_x, -wheel_y),
                         ("RearLeft", -axle_x, wheel_y), ("RearRight", -axle_x, -wheel_y)):
        wheel = UsdGeom.Cylinder.Define(stage, f"/World/Car/Wheels/{name}")
        wheel.CreateAxisAttr(UsdGeom.Tokens.y); wheel.CreateRadiusAttr(wheel_r); wheel.CreateHeightAttr(wheel_w)
        translate(wheel, Gf.Vec3d(wx, wy, 0)); color(wheel, (0.025, 0.025, 0.025))
    mast = UsdGeom.Cube.Define(stage, "/World/Car/CameraMount"); mast.CreateSizeAttr(1.0)
    translate(mast, Gf.Vec3d(0.25, 0, 0.60)); scale(mast, Gf.Vec3d(0.08, 0.08, 0.80)); color(mast, (0.9, 0.5, 0.04))
    stage.SetTimeCodesPerSecond(fps); stage.SetFramesPerSecond(fps)
    stage.SetStartTimeCode(0); stage.SetEndTimeCode(len(trajectory) - 1)

exit_code = 1
try:
    import importlib.metadata, omni.usd
    from pxr import UsdGeom, UsdLux
    version = importlib.metadata.version("isaacsim")
    if not version.startswith("5.1."): raise RuntimeError(f"要求Isaac Sim 5.1.x，当前为{version}")
    trajectory = make_trajectory(read_grounded(), args.fps)
    output_dir.mkdir(parents=True, exist_ok=True); stage_path.parent.mkdir(parents=True, exist_ok=True)
    context = omni.usd.get_context(); context.new_stage(); simulation_app.update(); stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    campus = UsdGeom.Xform.Define(stage, "/World/Campus")
    campus.GetPrim().GetReferences().AddReference(str(Path(cfg["campus_usd"])))
    add_car(stage, trajectory, args.fps)
    UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr(650.0)
    stage.GetRootLayer().Export(str(stage_path))
    trajectory_path = output_dir / "trajectory_frames.csv"
    with trajectory_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(trajectory[0])); writer.writeheader(); writer.writerows(trajectory)
    summary = {"status": "PASS", "isaac_sim_version": version, "fps": args.fps,
               "frame_count": len(trajectory), "duration_s": trajectory[-1]["time_s"],
               "distance_m": trajectory[-1]["distance_m"], "stage": str(stage_path),
               "trajectory": str(trajectory_path), "motion_type": "kinematic USD time-sampled root pose",
               "uses_old_absolute_z": False,
               "wheel_ground_gap_m": float(cfg["car"]["ground_clearance_m"])}
    (output_dir / "animation_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True); print("CAR_ANIMATION_PASS", flush=True)
    exit_code = 0
except BaseException as exc:
    traceback.print_exc(); output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "failure.json").write_text(json.dumps({"status": "FAILED", "exception_type": type(exc).__name__,
        "exception": str(exc), "traceback": traceback.format_exc()}, ensure_ascii=False, indent=2), encoding="utf-8")
finally:
    try: simulation_app.close()
    except BaseException: pass
sys.exit(exit_code)
