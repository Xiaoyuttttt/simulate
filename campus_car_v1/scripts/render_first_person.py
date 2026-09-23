#!/usr/bin/env python3
"""Render the animated car's rigidly mounted first-person camera."""
from __future__ import annotations
import argparse, asyncio, importlib.metadata, json, shutil, sys, time, traceback
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--config", required=True)
parser.add_argument("--width", type=int, default=960)
parser.add_argument("--height", type=int, default=540)
parser.add_argument("--input-stage", default=None)
parser.add_argument("--output-dir", default=None)
args = parser.parse_args()
cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
root = Path(cfg["project_root"])
output_dir = Path(args.output_dir) if args.output_dir else root / "outputs/first_person"
input_stage = Path(args.input_stage) if args.input_stage else root / "stages/campus_car_animated.usd"
output_stage = output_dir / "campus_car_first_person.usd"
summary_file = output_dir / "summary.json"
frames_dir = output_dir / "frames"
started = time.monotonic()

def stamp(message): print(f"[{time.monotonic()-started:8.2f}s] {message}", flush=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True, "renderer": cfg["render"]["renderer"],
    "width": args.width, "height": args.height, "multi_gpu": False, "max_gpu_count": 1,
    "sync_loads": True, "disable_viewport_updates": False,
    "extra_args": ["--/renderer/multiGpu/enabled=false", "--/renderer/multiGpu/autoEnable=false",
                    "--/rtx/materialDb/syncLoads=True", "--/rtx/hydra/materialSyncLoads=True",
                    "--/omni.kit.plugin/syncUsdLoads=True", "--/renderer/raytracing/fractionalCutoutOpacity=0"]})

def wait_stage(context, timeout_s=180.0):
    deadline, quiet = time.monotonic()+timeout_s, 0
    while time.monotonic() < deadline and quiet < 3:
        simulation_app.update(); status = context.get_stage_loading_status()
        quiet = quiet + 1 if len(status) >= 3 and status[2] == 0 else 0
    if quiet < 3: raise TimeoutError("动画Stage依赖加载超时")

def wait_task(task, timeout_s=185.0):
    deadline = time.monotonic()+timeout_s
    while not task.done() and time.monotonic() < deadline: simulation_app.update()
    if not task.done(): task.cancel(); raise TimeoutError("单帧截图超时")
    task.result()

async def capture(viewport, output):
    from omni.kit.viewport.utility import capture_viewport_to_file
    result = capture_viewport_to_file(viewport, str(output), is_hdr=False)
    await result.wait_for_result()

exit_code = 1
try:
    import omni.timeline, omni.usd
    from omni.kit.viewport.utility import get_active_viewport
    from pxr import Gf, UsdGeom
    version = importlib.metadata.version("isaacsim")
    if not version.startswith("5.1."): raise RuntimeError(f"要求Isaac Sim 5.1.x，当前为{version}")
    if not input_stage.is_file(): raise FileNotFoundError(f"请先完成第三阶段：{input_stage}")
    if frames_dir.exists(): shutil.rmtree(frames_dir)
    frames_dir.mkdir(parents=True, exist_ok=True); output_stage.parent.mkdir(parents=True, exist_ok=True)
    context = omni.usd.get_context(); context.open_stage(str(input_stage)); wait_stage(context)
    for _ in range(30): simulation_app.update()
    stage = context.get_stage(); car = stage.GetPrimAtPath("/World/Car")
    if not car.IsValid(): raise RuntimeError("动画Stage中没有/World/Car")
    fps = float(stage.GetFramesPerSecond()); start_frame = int(stage.GetStartTimeCode()); end_frame = int(stage.GetEndTimeCode())
    if end_frame < start_frame: raise RuntimeError("动画Stage时间范围无效")
    camera = UsdGeom.Camera.Define(stage, "/World/Car/FirstPersonCamera")
    camera.CreateFocalLengthAttr(18.0); camera.CreateHorizontalApertureAttr(20.955)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 2000.0))
    camera_xf = UsdGeom.Xformable(camera.GetPrim())
    camera_xf.AddTranslateOp().Set(Gf.Vec3d(0.45, 0.0, 1.08))
    # Camera looks down local -Z. This quaternion maps -Z to car +X and local
    # +Y to world +Z, giving a forward-looking, upright vehicle camera.
    camera_xf.AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
    stage.GetRootLayer().Export(str(output_stage)); stamp(f"已保存车载相机Stage：{output_stage}")
    viewport = get_active_viewport()
    if viewport is None: raise RuntimeError("Headless模式没有active viewport")
    viewport.set_texture_resolution((args.width, args.height))
    viewport.set_active_camera("/World/Car/FirstPersonCamera")
    timeline = omni.timeline.get_timeline_interface(); timeline.stop()
    total = end_frame - start_frame + 1
    for sequence, frame in enumerate(range(start_frame, end_frame+1)):
        timeline.set_current_time(frame / fps)
        for _ in range(3): simulation_app.update()
        path = frames_dir / f"frame_{sequence:04d}.png"
        task = asyncio.ensure_future(capture(viewport, path)); wait_task(task)
        if sequence == 0 or (sequence+1) % 25 == 0 or sequence+1 == total:
            stamp(f"第一视角帧：{sequence+1}/{total}")
    summary = {"status": "PASS", "isaac_sim_version": version, "input_stage": str(input_stage),
        "camera_stage": str(output_stage), "camera_prim": "/World/Car/FirstPersonCamera",
        "camera_local_translation_xyz": [0.45, 0.0, 1.08], "resolution": [args.width, args.height],
        "fps": fps, "frame_count": total, "frames_dir": str(frames_dir),
        "expected_video": str(root / "outputs/first_person/campus_car_first_person.mp4")}
    summary_file.parent.mkdir(parents=True, exist_ok=True)
    summary_file.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    print("FIRST_PERSON_FRAMES_PASS", flush=True); exit_code = 0
except BaseException as exc:
    traceback.print_exc(); summary_file.parent.mkdir(parents=True, exist_ok=True)
    (summary_file.parent / "failure.json").write_text(json.dumps({"status":"FAILED",
        "exception_type":type(exc).__name__, "exception":str(exc), "traceback":traceback.format_exc()},
        ensure_ascii=False, indent=2), encoding="utf-8")
finally:
    try: simulation_app.close()
    except BaseException: pass
sys.exit(exit_code)
