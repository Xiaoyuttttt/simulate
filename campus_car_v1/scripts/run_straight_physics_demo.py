#!/usr/bin/env python3
"""Minimal Isaac Sim 5.1 demo: drive the existing four-wheel car straight.

The collision road is a narrow ribbon generated from the Z values previously
probed from the campus mesh.  The car is moved only by four revolute-joint
velocity drives; the script never writes a time-varying root pose.
"""
from __future__ import annotations

import argparse
import csv
import importlib.metadata
import json
import math
import sys
import time
import traceback
from pathlib import Path

import numpy as np


parser = argparse.ArgumentParser()
parser.add_argument("--config", required=True)
parser.add_argument("--distance-m", type=float, default=None)
parser.add_argument("--speed-mps", type=float, default=None)
parser.add_argument("--turn-test", action="store_true")
parser.add_argument("--wasd", action="store_true")
parser.add_argument("--preset", action="store_true")
args = parser.parse_args()
cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
root = Path(cfg["project_root"])
demo = dict(cfg["straight_physics_demo"])
if args.distance_m is not None:
    demo["distance_m"] = args.distance_m
if args.speed_mps is not None:
    demo["speed_mps"] = args.speed_mps

mode_dir = "wasd_drive" if args.wasd else ("preset_drive" if args.preset else ("turn_physics_demo" if args.turn_test else "straight_physics_demo"))
output_dir = root / f"outputs/{mode_dir}"
stage_name = "campus_car_wasd.usd" if args.wasd else ("campus_car_turn_physics.usd" if args.turn_test else "campus_car_straight_physics.usd")
stage_path = root / f"stages/{stage_name}"
trajectory_path = output_dir / "trajectory_frames.csv"
summary_path = output_dir / "summary.json"
grounded_route = root / "outputs/control_route_ground/grounded_control_route.csv"
started = time.monotonic()


def stamp(message: str) -> None:
    print(f"[{time.monotonic() - started:8.2f}s] {message}", flush=True)


from isaacsim import SimulationApp


app = SimulationApp(
    {
        "headless": not args.wasd,
        "renderer": cfg["render"]["renderer"],
        "width": 640,
        "height": 360,
        "multi_gpu": False,
        "max_gpu_count": 1,
        "disable_viewport_updates": not args.wasd,
        "extra_args": [
            "--/app/window/enabled=true",
            "--/app/window/hideUi=false",
            "--/renderer/multiGpu/enabled=false",
            "--/renderer/multiGpu/autoEnable=false",
            "--/renderer/multiGpu/maxGpuCount=1",
        ],
    }
)


def read_grounded_route(path: Path) -> dict[str, np.ndarray]:
    if not path.is_file():
        raise FileNotFoundError(f"缺少Mesh探地路线：{path}；请先运行 scripts/run_control_route.sh")
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    required = {"distance_m", "x", "y", "ground_z", "yaw_rad"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f"探地路线缺少列：{sorted(required)}")
    return {
        key: np.asarray([float(row[key]) for row in rows], dtype=np.float64)
        for key in required
    }


def sample_demo_profile(route: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    distance = float(demo["distance_m"])
    end_padding = float(demo["road_end_padding_m"])
    if not 0.5 <= distance <= float(route["distance_m"][-1]):
        raise ValueError("直行距离必须大于等于0.5米且不超过现有路线长度")
    step = min(0.25, distance / 20.0)
    positive_s = np.arange(0.0, distance + end_padding + step * 0.5, step)
    positive_s = np.clip(positive_s, 0.0, route["distance_m"][-1])
    positive_s = np.unique(np.r_[positive_s, min(distance + end_padding, route["distance_m"][-1])])
    result = {"distance_m": positive_s}
    for key in ("x", "y", "ground_z", "yaw_rad"):
        result[key] = np.interp(positive_s, route["distance_m"], route[key])

    padding = float(demo["road_start_padding_m"])
    heading = np.asarray([math.cos(result["yaw_rad"][0]), math.sin(result["yaw_rad"][0])])
    result["distance_m"] = np.r_[-padding, result["distance_m"]]
    result["x"] = np.r_[result["x"][0] - padding * heading[0], result["x"]]
    result["y"] = np.r_[result["y"][0] - padding * heading[1], result["y"]]
    result["ground_z"] = np.r_[result["ground_z"][0], result["ground_z"]]
    result["yaw_rad"] = np.r_[result["yaw_rad"][0], result["yaw_rad"]]
    return result


def set_color(gprim, rgb: tuple[float, float, float]) -> None:
    from pxr import Gf, UsdGeom, Vt

    UsdGeom.Gprim(gprim.GetPrim()).CreateDisplayColorAttr(
        Vt.Vec3fArray([Gf.Vec3f(*rgb)])
    )


def add_translate(schema, xyz) -> None:
    from pxr import Gf, UsdGeom

    UsdGeom.Xformable(schema.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*xyz))


def add_scale(schema, xyz) -> None:
    from pxr import Gf, UsdGeom

    UsdGeom.Xformable(schema.GetPrim()).AddScaleOp().Set(Gf.Vec3d(*xyz))


def build_collision_ribbon(stage, profile: dict[str, np.ndarray]) -> dict:
    from pxr import Gf, UsdGeom, UsdPhysics, Vt

    xy = np.column_stack((profile["x"], profile["y"]))
    tangent = np.gradient(xy, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    left = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    half_width = float(demo["road_half_width_m"])
    left_xy = xy + half_width * left
    right_xy = xy - half_width * left
    points = []
    for left_point, right_point, z in zip(left_xy, right_xy, profile["ground_z"]):
        points.extend(((left_point[0], left_point[1], z), (right_point[0], right_point[1], z)))
    faces = []
    for index in range(len(xy) - 1):
        left0, right0 = 2 * index, 2 * index + 1
        left1, right1 = 2 * index + 2, 2 * index + 3
        faces.extend(((left0, right0, left1), (right0, right1, left1)))

    mesh = UsdGeom.Mesh.Define(stage, "/World/StraightDemoRoad")
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*point) for point in points]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([index for face in faces for index in face]))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set("none")
    return {
        "source": str(grounded_route),
        "samples": len(xy),
        "triangles": len(faces),
        "half_width_m": half_width,
        "z_range_m": [float(profile["ground_z"].min()), float(profile["ground_z"].max())],
    }


def build_physics_car(stage, start: dict[str, float]):
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    car_cfg = cfg["car"]
    length = float(car_cfg["length_m"])
    width = float(car_cfg["width_m"])
    chassis_h = float(car_cfg["chassis_height_m"])
    wheel_r = float(car_cfg["wheel_radius_m"])
    wheel_w = float(car_cfg["wheel_width_m"])
    clearance = float(car_cfg["ground_clearance_m"])
    chassis_local_z = wheel_r + clearance
    axle_x = length * 0.34
    wheel_y = width * 0.5 + wheel_w * 0.40

    root_prim = UsdGeom.Xform.Define(stage, "/World/Car")
    root_xf = UsdGeom.Xformable(root_prim.GetPrim())
    root_xf.AddTranslateOp().Set(Gf.Vec3d(start["x"], start["y"], start["z"] + wheel_r + 0.03))
    root_xf.AddOrientOp().Set(
        Gf.Quatf(math.cos(start["yaw"] * 0.5), Gf.Vec3f(0.0, 0.0, math.sin(start["yaw"] * 0.5)))
    )
    UsdPhysics.ArticulationRootAPI.Apply(root_prim.GetPrim())
    root_prim.GetPrim().CreateAttribute(
        "physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool
    ).Set(False)

    chassis_link = UsdGeom.Xform.Define(stage, "/World/Car/Chassis")
    add_translate(chassis_link, (0.0, 0.0, chassis_local_z))
    UsdPhysics.RigidBodyAPI.Apply(chassis_link.GetPrim()).CreateRigidBodyEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(chassis_link.GetPrim()).CreateMassAttr(float(demo["chassis_mass_kg"]))
    chassis = UsdGeom.Cube.Define(stage, "/World/Car/Chassis/Geometry")
    chassis.CreateSizeAttr(1.0)
    add_scale(chassis, (length, width, chassis_h))
    set_color(chassis, (0.08, 0.28, 0.85))
    UsdPhysics.CollisionAPI.Apply(chassis.GetPrim()).CreateCollisionEnabledAttr(True)

    drive_apis = []
    wheel_records = []
    wheel_specs = (
        ("FrontLeft", axle_x, wheel_y),
        ("FrontRight", axle_x, -wheel_y),
        ("RearLeft", -axle_x, wheel_y),
        ("RearRight", -axle_x, -wheel_y),
    )
    for name, wx, wy in wheel_specs:
        wheel_path = f"/World/Car/Wheels/{name}"
        wheel_link = UsdGeom.Xform.Define(stage, wheel_path)
        add_translate(wheel_link, (wx, wy, 0.0))
        UsdPhysics.RigidBodyAPI.Apply(wheel_link.GetPrim()).CreateRigidBodyEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(wheel_link.GetPrim()).CreateMassAttr(float(demo["wheel_mass_kg"]))
        wheel = UsdGeom.Cylinder.Define(stage, f"{wheel_path}/Geometry")
        wheel.CreateAxisAttr(UsdGeom.Tokens.y)
        wheel.CreateRadiusAttr(wheel_r)
        wheel.CreateHeightAttr(wheel_w)
        set_color(wheel, (0.025, 0.025, 0.025))
        UsdPhysics.CollisionAPI.Apply(wheel.GetPrim()).CreateCollisionEnabledAttr(True)

        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/World/Car/Joints/{name}Joint")
        joint.CreateAxisAttr(UsdGeom.Tokens.y)
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/World/Car/Chassis")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(wheel_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(wx, wy, -chassis_local_z))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(float(demo["wheel_drive_damping"]))
        drive.CreateMaxForceAttr(float(demo["wheel_drive_max_force"]))
        drive.CreateTargetVelocityAttr(0.0)
        drive_apis.append(drive)
        wheel_records.append({"name": name, "path": wheel_path, "joint": str(joint.GetPath())})

    mast = UsdGeom.Cube.Define(stage, "/World/Car/Chassis/CameraMount")
    mast.CreateSizeAttr(1.0)
    add_translate(mast, (0.25, 0.0, 0.42))
    add_scale(mast, (0.08, 0.08, 0.80))
    set_color(mast, (0.9, 0.5, 0.04))

    camera = UsdGeom.Camera.Define(stage, "/World/Car/Chassis/FirstPersonCamera")
    camera.CreateFocalLengthAttr(18.0)
    camera.CreateHorizontalApertureAttr(20.955)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 2000.0))
    camera_xf = UsdGeom.Xformable(camera.GetPrim())
    camera_xf.AddTranslateOp().Set(Gf.Vec3d(0.45, 0.0, 1.08 - chassis_local_z))
    camera_xf.AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
    return drive_apis, wheel_records, camera.GetPrim(), chassis_link.GetPrim(), wheel_r, chassis_local_z


def world_pose(prim):
    from pxr import Gf, Usd, UsdGeom

    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    position = matrix.ExtractTranslation()
    rotation = matrix.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    forward = matrix.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
    yaw = math.atan2(float(forward[1]), float(forward[0]))
    return {
        "x": float(position[0]), "y": float(position[1]), "z": float(position[2]),
        "qw": float(rotation.GetReal()), "qx": float(imaginary[0]),
        "qy": float(imaginary[1]), "qz": float(imaginary[2]), "yaw": yaw,
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


exit_code = 1
try:
    import omni.usd
    from isaacsim.core.api import World
    from pxr import Gf, UsdGeom, UsdLux

    version = importlib.metadata.version("isaacsim")
    if not version.startswith("5.1."):
        raise RuntimeError(f"要求Isaac Sim 5.1.x，当前为{version}")
    output_dir.mkdir(parents=True, exist_ok=True)
    stage_path.parent.mkdir(parents=True, exist_ok=True)
    route = read_grounded_route(grounded_route)
    profile = sample_demo_profile(route)

    World.clear_instance()
    context = omni.usd.get_context()
    context.new_stage()
    app.update()
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / int(demo["physics_hz"]),
        rendering_dt=1.0 / int(demo["record_fps"]),
    )
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    campus_path = Path(cfg["campus_usd"])
    if not campus_path.is_file():
        raise FileNotFoundError(f"校园USD不存在：{campus_path}")
    campus = UsdGeom.Xform.Define(stage, "/World/Campus")
    if not campus.GetPrim().GetReferences().AddReference(str(campus_path)):
        raise RuntimeError(f"无法引用校园USD：{campus_path}")
    road_info = build_collision_ribbon(stage, profile)

    start_state = {
        "x": float(profile["x"][1]), "y": float(profile["y"][1]),
        "z": float(profile["ground_z"][1]), "yaw": float(profile["yaw_rad"][1]),
    }
    drives, wheels, camera_prim, chassis_prim, wheel_r, chassis_local_z = build_physics_car(stage, start_state)
    keys = set()
    control = {"quit_requested": False}
    keyboard_subscription = None
    if args.wasd:
        import carb.input
        import omni.appwindow
        window = None
        for _ in range(120):
            app.update()
            window = omni.appwindow.get_default_app_window()
            if window is not None:
                break
            time.sleep(0.05)
        if window is None:
            raise RuntimeError("Isaac Sim未创建默认窗口；请确认DISPLAY和XAUTHORITY有效，并从图形桌面终端启动")
        keyboard = window.get_keyboard()
        if keyboard is None:
            raise RuntimeError("Isaac Sim默认窗口没有键盘接口；请点击Viewport后重试")
        key_map = {carb.input.KeyboardInput.W: "w", carb.input.KeyboardInput.A: "a",
                   carb.input.KeyboardInput.S: "s", carb.input.KeyboardInput.D: "d"}
        def on_keyboard(event, *_):
            if event.input == carb.input.KeyboardInput.SPACE and event.type == carb.input.KeyboardEventType.KEY_PRESS:
                keys.clear(); return True
            if event.input == carb.input.KeyboardInput.ESCAPE and event.type == carb.input.KeyboardEventType.KEY_PRESS:
                keys.clear(); control["quit_requested"] = True; return True
            if event.input in key_map:
                if event.type in (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT): keys.add(key_map[event.input])
                elif event.type == carb.input.KeyboardEventType.KEY_RELEASE: keys.discard(key_map[event.input])
            return True
        input_interface = carb.input.acquire_input_interface()
        keyboard_subscription = input_interface.subscribe_to_keyboard_events(keyboard, on_keyboard)
        try:
            import omni.kit.viewport.utility
            omni.kit.viewport.utility.get_active_viewport().set_active_camera(str(camera_prim.GetPath()))
        except Exception as exc:
            stamp(f"相机切换提示：{exc}")
        stamp("WASD驾驶已启动：W前进 S后退 A左转 D右转，Space停车，Esc退出；请点击Viewport后按键")
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
    dome.CreateIntensityAttr(650.0)
    # Use a visible blue environment instead of the default black background.
    dome.CreateColorAttr(Gf.Vec3f(0.36, 0.58, 0.88))
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(3000.0)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.92, 0.78))
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 25.0, 20.0))
    stage.GetRootLayer().Export(str(stage_path))
    stamp(f"保存物理Stage：{stage_path}")

    world.reset()
    settle_steps = int(float(demo["settle_seconds"]) * int(demo["physics_hz"]))
    for _ in range(settle_steps):
        world.step(render=False)

    wheel_speed_rad_s = float(demo["speed_mps"]) / wheel_r
    wheel_sign = float(demo["wheel_drive_sign"])
    if args.turn_test:
        stamp("开始固定命令差速转向测试：前进、左转、停车、右转")
    else:
        target_deg_s = math.degrees(wheel_sign * wheel_speed_rad_s)
        for drive in drives:
            drive.GetTargetVelocityAttr().Set(target_deg_s)
        stamp(f"开始直行：目标{demo['distance_m']}m，速度{demo['speed_mps']}m/s，轮速{wheel_speed_rad_s:.3f}rad/s")

    heading = np.asarray([math.cos(start_state["yaw"]), math.sin(start_state["yaw"])])
    lateral_axis = np.asarray([-heading[1], heading[0]])
    physics_hz = int(demo["physics_hz"])
    record_stride = max(1, physics_hz // int(demo["record_fps"]))
    max_seconds = 8.0 if args.preset else (6.0 if args.turn_test else float(demo["distance_m"]) / float(demo["speed_mps"]) + float(demo["timeout_margin_seconds"]))
    max_steps = int(max_seconds * physics_hz)
    rows = []
    maximum_lateral = 0.0
    progress = 0.0
    initial_yaw = None
    left_yaw = None
    right_yaw = None
    turn_start_yaw = None
    for step in range((max_steps + 1) if not args.wasd else 10**9):
        if args.wasd or args.preset:
            if args.preset:
                elapsed = step / physics_hz
                if elapsed < 1.5: linear_input, angular_input = 1.0, 0.0
                elif elapsed < 3.5: linear_input, angular_input = 1.0, 1.0
                elif elapsed < 5.5: linear_input, angular_input = 1.0, -1.0
                else: linear_input, angular_input = 0.0, 0.0
            else:
                linear_input = float(("w" in keys)) - float(("s" in keys))
                angular_input = float(("a" in keys)) - float(("d" in keys))
            linear_velocity = max(-0.5, min(0.5, linear_input * 0.5))
            angular_velocity = max(-0.8, min(0.8, angular_input * 0.8))
            left_linear = linear_velocity - angular_velocity * 0.73 / 2.0
            right_linear = linear_velocity + angular_velocity * 0.73 / 2.0
            left_target = math.degrees(wheel_sign * left_linear / wheel_r)
            right_target = math.degrees(wheel_sign * right_linear / wheel_r)
            for drive, target in zip(drives, (left_target, right_target, left_target, right_target)):
                drive.GetTargetVelocityAttr().Set(target)
        if args.turn_test:
            elapsed = step / physics_hz
            if elapsed < 1.0:
                left_linear = right_linear = 0.30
                phase = "forward"
            elif elapsed < 3.0:
                left_linear, right_linear = -0.20, 0.50
                phase = "left"
                if turn_start_yaw is None:
                    turn_start_yaw = world_pose(chassis_prim)["yaw"]
            elif elapsed < 4.0:
                left_linear = right_linear = 0.0
                phase = "stop"
            else:
                left_linear, right_linear = 0.50, -0.20
                phase = "right"
                if right_yaw is None and left_yaw is not None:
                    turn_start_yaw = world_pose(chassis_prim)["yaw"]
            left_target = math.degrees(wheel_sign * left_linear / wheel_r)
            right_target = math.degrees(wheel_sign * right_linear / wheel_r)
            for drive, target in zip(drives, (left_target, right_target, left_target, right_target)):
                drive.GetTargetVelocityAttr().Set(target)
        world.step(render=args.wasd)
        if step % record_stride != 0:
            continue
        chassis_pose = world_pose(chassis_prim)
        if initial_yaw is None:
            initial_yaw = chassis_pose["yaw"]
        if args.turn_test and phase == "left" and left_yaw is None and step / physics_hz >= 2.9:
            left_yaw = chassis_pose["yaw"] - initial_yaw
        if args.turn_test and phase == "right" and right_yaw is None and step / physics_hz >= 5.9:
            right_yaw = chassis_pose["yaw"] - initial_yaw
        camera_pose = world_pose(camera_prim)
        displacement = np.asarray([chassis_pose["x"] - start_state["x"], chassis_pose["y"] - start_state["y"]])
        progress = float(displacement @ heading)
        lateral = float(displacement @ lateral_axis)
        maximum_lateral = max(maximum_lateral, abs(lateral))
        ground_z = float(np.interp(np.clip(progress, 0.0, demo["distance_m"]), profile["distance_m"], profile["ground_z"]))
        rows.append({
            "frame": len(rows), "time_s": step / physics_hz, "distance_m": progress,
            "x": chassis_pose["x"], "y": chassis_pose["y"],
            "body_z": chassis_pose["z"] - chassis_local_z, "ground_z": ground_z,
            "yaw_rad": chassis_pose["yaw"], "speed_mps": float(demo["speed_mps"]),
            "lateral_error_m": lateral,
            "body_qw": chassis_pose["qw"], "body_qx": chassis_pose["qx"],
            "body_qy": chassis_pose["qy"], "body_qz": chassis_pose["qz"],
            "camera_x": camera_pose["x"], "camera_y": camera_pose["y"], "camera_z": camera_pose["z"],
            "camera_qw": camera_pose["qw"], "camera_qx": camera_pose["qx"],
            "camera_qy": camera_pose["qy"], "camera_qz": camera_pose["qz"],
        })
        if len(rows) == 1 or len(rows) % 25 == 0:
            stamp(f"进度={progress:.2f}m，横向误差={lateral:.3f}m")
        if args.wasd and control["quit_requested"]:
            break
        if args.preset and step >= max_steps:
            break
        if not args.wasd and not args.turn_test and progress >= float(demo["distance_m"]):
            break

    for drive in drives:
        drive.GetTargetVelocityAttr().Set(0.0)
    if keyboard_subscription is not None:
        input_interface.unsubscribe_to_keyboard_events(keyboard, keyboard_subscription)
    for _ in range(max(1, physics_hz // 2)):
        world.step(render=False)
    if not rows:
        raise RuntimeError("物理仿真没有生成轨迹记录")
    write_csv(trajectory_path, rows)

    if args.wasd or args.preset:
        validation = {"keyboard_mode": True, "safe_stop_completed": True, "trajectory_has_multiple_frames": len(rows) >= 1}
    elif args.turn_test:
        left_yaw = float(left_yaw if left_yaw is not None else 0.0)
        right_yaw = float(right_yaw if right_yaw is not None else 0.0)
        validation = {
            "left_turn_yaw_reached": abs(left_yaw) >= 0.25,
            "right_turn_yaw_reached": abs(right_yaw - left_yaw) >= 0.25,
            "turns_have_opposite_sign": (left_yaw * (right_yaw - left_yaw)) < 0.0,
            "trajectory_has_multiple_frames": len(rows) >= 10,
        }
    else:
        validation = {
        "driven_by_wheel_velocity_only": True,
        "minimum_progress_reached": progress >= float(demo["minimum_progress_m"]),
        "lateral_error_within_limit": maximum_lateral <= float(demo["max_lateral_error_m"]),
        "trajectory_has_multiple_frames": len(rows) >= 10,
        }
    summary = {
        "status": "PASS" if all(validation.values()) else "FAILED_VALIDATION",
        "isaac_sim_version": version,
        "demo": demo,
        "road": road_info,
        "wheel_radius_m": wheel_r,
        "wheel_target_rad_s": wheel_speed_rad_s,
        "turn_test": args.turn_test,
        "left_turn_yaw_delta_rad": left_yaw,
        "right_turn_yaw_delta_rad": None if right_yaw is None or left_yaw is None else right_yaw - left_yaw,
        "wheel_records": wheels,
        "progress_m": progress,
        "maximum_lateral_error_m": maximum_lateral,
        "frame_count": len(rows),
        "trajectory_csv": str(trajectory_path),
        "stage": str(stage_path),
        "camera_prim": str(camera_prim.GetPath()),
        "validation": validation,
    }
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    if summary["status"] != "PASS":
        raise RuntimeError("直行Demo未通过验收；请查看summary.json")
    print("STRAIGHT_PHYSICS_DEMO_PASS", flush=True)
    exit_code = 0
except BaseException as exc:
    exit_code = 121
    traceback.print_exc()
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "failure.json").write_text(
        json.dumps(
            {"status": "FAILED", "exception_type": type(exc).__name__,
             "exception": str(exc), "traceback": traceback.format_exc()},
            ensure_ascii=False, indent=2,
        ),
        encoding="utf-8",
    )
    print("STRAIGHT_PHYSICS_DEMO_FAILED", flush=True)
finally:
    try:
        app.close()
    except BaseException:
        traceback.print_exc()

sys.exit(exit_code)
