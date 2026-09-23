#!/usr/bin/env python3
"""Drive the imported Scout v2 USD through the grounded campus route in PhysX."""
from __future__ import annotations

import argparse, csv, importlib.metadata, json, math, time, traceback
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--config", default=str(ROOT / "config/placement.json"))
parser.add_argument("--grounded-route", default=str(ROOT / "outputs/scout_v2_route_ground/grounded_control_route.csv"))
parser.add_argument("--output-dir", default=str(ROOT / "outputs/scout_v2_route"))
parser.add_argument("--fps", type=int, default=10)
parser.add_argument("--width", type=int, default=960)
parser.add_argument("--height", type=int, default=540)
parser.add_argument("--no-render", action="store_true")
parser.add_argument("--no-campus", action="store_true", help="Omit the campus visual stage for physics smoke tests")
parser.add_argument("--max-seconds", type=float, default=0.0, help="Short partial run for smoke testing")
args = parser.parse_args()
cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
out = Path(args.output_dir); out.mkdir(parents=True, exist_ok=True)
started = time.monotonic()
def log(s): print(f"[{time.monotonic()-started:7.1f}s] {s}", flush=True)

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": cfg["render"]["renderer"],
    "width": args.width, "height": args.height, "multi_gpu": False,
    "max_gpu_count": 1, "disable_viewport_updates": False,
    "extra_args": ["--/renderer/multiGpu/enabled=false", "--/renderer/multiGpu/autoEnable=false"]})
exit_code = 1
try:
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import GroundPlane
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, Vt

    ver = importlib.metadata.version("isaacsim")
    if not ver.startswith("5.1."): raise RuntimeError(f"需要Isaac Sim 5.1.x，检测到{ver}")
    asset = ROOT / "assets/scout_v2_isaac/Isaac_models/scout_v2_base.usd"
    if not asset.is_file(): raise FileNotFoundError(asset)
    with Path(args.grounded_route).open("r", encoding="utf-8-sig", newline="") as f:
        records = list(csv.DictReader(f))
    if len(records) < 3: raise ValueError("探地路线少于3个采样点")
    route = {k: np.asarray([float(r[k]) for r in records]) for k in ("distance_m", "x", "y", "ground_z", "speed_mps")}
    fps = max(1, args.fps); hz = 120; stride = max(1, hz // fps)
    World.clear_instance(); context = omni.usd.get_context(); context.new_stage(); app.update()
    world = World(stage_units_in_meters=1.0, physics_dt=1.0/hz, rendering_dt=1.0/fps)
    stage = context.get_stage(); UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    # Use Isaac's native PhysX ground-plane helper for the fail-safe contact surface.
    # A hand-authored USD quad can render correctly but may not be cooked as a collider.
    world.scene.add(GroundPlane(prim_path="/World/FallbackGround", name="fallback_ground",
        size=2000.0, z_position=float(route["ground_z"].min())-0.08, visible=False))
    if not args.no_campus:
        campus = UsdGeom.Xform.Define(stage, "/World/Campus")
        if not campus.GetPrim().GetReferences().AddReference(str(Path(cfg["campus_usd"]))): raise RuntimeError("校园USD引用失败")

    # A thin hidden collision ribbon ties the tires to the terrain heights sampled from the campus mesh.
    xy = np.column_stack((route["x"], route["y"])); tangent = np.gradient(xy, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    lateral = np.column_stack((-tangent[:, 1], tangent[:, 0])); half = 1.25
    pts=[]
    for p,n,z in zip(xy,lateral,route["ground_z"]): pts.extend(((p[0]+half*n[0],p[1]+half*n[1],z),(p[0]-half*n[0],p[1]-half*n[1],z)))
    faces=[]
    for i in range(len(xy)-1): faces.extend(((2*i,2*i+1,2*i+2),(2*i+1,2*i+3,2*i+2)))
    road=UsdGeom.Mesh.Define(stage,"/World/RouteCollision")
    road.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts])); road.CreateFaceVertexCountsAttr(Vt.IntArray([3]*len(faces)))
    road.CreateFaceVertexIndicesAttr(Vt.IntArray([j for face in faces for j in face])); road.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    road.CreateDoubleSidedAttr(True); road.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdPhysics.CollisionAPI.Apply(road.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(road.GetPrim()).CreateApproximationAttr().Set("none")
    # Safety catcher below the surveyed corridor. The route mesh remains the
    # contact surface; this invisible plane prevents a failed mesh cook from
    # letting the robot fall out of the world.

    # The upstream USD is authored in centimetres; scale the whole robot exactly once at the placement root.
    first_yaw = math.atan2(route["y"][2]-route["y"][0], route["x"][2]-route["x"][0])
    place = UsdGeom.Xform.Define(stage,"/World/ScoutPlacement")
    xf=UsdGeom.Xformable(place.GetPrim()); place_translate=xf.AddTranslateOp(); place_translate.Set(Gf.Vec3d(float(route["x"][0]),float(route["y"][0]),0.0))
    xf.AddOrientOp().Set(Gf.Quatf(math.cos(first_yaw/2),Gf.Vec3f(0,0,math.sin(first_yaw/2)))); xf.AddScaleOp().Set(Gf.Vec3f(0.01))
    scout=UsdGeom.Xform.Define(stage,"/World/ScoutPlacement/Scout")
    if not scout.GetPrim().GetReferences().AddReference(str(asset)): raise RuntimeError("Scout USD引用失败")
    cache=UsdGeom.XformCache(Usd.TimeCode.Default())
    base_matrix=cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/ScoutPlacement/Scout/base_link"))
    base_origin=base_matrix.ExtractTranslation(); base_rotation=base_matrix.ExtractRotationQuat()
    wheel_paths=[f"/World/ScoutPlacement/Scout/base_link/{side}_wheel_joint" for side in ("front_left","front_right","rear_left","rear_right")]
    drives=[]; wheel_signs=[]
    for path in wheel_paths:
        prim=stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid(): raise RuntimeError(f"Scout车轮关节不存在: {path}")
        drive=UsdPhysics.DriveAPI.Get(prim,"angular")
        if not drive: raise RuntimeError(f"车轮关节缺少angular DriveAPI: {path}")
        # Imported left/right wheel frames may use opposite joint axes. Compute
        # each sign from the actual USD joint frame and tire contact kinematics.
        axis_name=str(prim.GetAttribute("physics:axis").Get())
        axis_vec={"X":Gf.Vec3d(1,0,0),"Y":Gf.Vec3d(0,1,0),"Z":Gf.Vec3d(0,0,1)}.get(axis_name)
        if axis_vec is None: raise RuntimeError(f"未知的转轴方向: {axis_name} ({path})")
        local_rot=prim.GetAttribute("physics:localRot0").Get()
        local_axis=local_rot.Transform(Gf.Vec3f(axis_vec))
        axis_world=base_rotation.Transform(Gf.Vec3d(local_axis))
        contact_velocity=Gf.Cross(axis_world,Gf.Vec3d(0,0,-1))
        forward_world=Gf.Vec3d(math.cos(first_yaw),math.sin(first_yaw),0)
        # The asset's revolute joint frame uses the opposite sign to the
        # geometric contact-velocity convention for forward rolling.
        wheel_signs.append(-1.0 if Gf.Dot(contact_velocity,forward_world)>=0 else 1.0)
        max_force=drive.GetMaxForceAttr().Get()
        if max_force is None or float(max_force)<1000.0: drive.GetMaxForceAttr().Set(1000.0)
        # Source USD sets 1e7 damping, which overwhelms the velocity controller.
        drive.GetDampingAttr().Set(20.0)
        drives.append(drive)
    wheel_shape=stage.GetPrimAtPath("/World/ScoutPlacement/Scout/front_left_wheel_link/collisions")
    wheel_r=float(wheel_shape.GetAttribute("radius").Get())*0.01 if wheel_shape and wheel_shape.IsValid() else 0.165
    if not 0.07 < wheel_r < 0.30: wheel_r=0.165
    # Place the tire bottoms on the sampled road rather than guessing the chassis origin.
    cache=UsdGeom.XformCache(Usd.TimeCode.Default()); wheel_centers=[]
    for path in wheel_paths:
        joint=stage.GetPrimAtPath(path); anchor=joint.GetAttribute("physics:localPos0").Get()
        wheel_centers.append(base_origin+base_rotation.Transform(Gf.Vec3d(float(anchor[0])*0.01,float(anchor[1])*0.01,float(anchor[2])*0.01)))
    min_wheel_center_z=min(float(p[2]) for p in wheel_centers)
    place_translate.Set(Gf.Vec3d(float(route["x"][0]),float(route["y"][0]),float(route["ground_z"][0])+wheel_r-min_wheel_center_z))
    track=abs(float(wheel_centers[0][1]-wheel_centers[1][1]))
    if not 0.25 < track < 0.9: track=0.49

    # The source robot has no camera prims: add a body-mounted forward camera and a world-space chase camera.
    front=UsdGeom.Camera.Define(stage,"/World/FirstPersonCamera")
    front.CreateFocalLengthAttr(18.0); front.CreateHorizontalApertureAttr(20.955); front.CreateClippingRangeAttr(Gf.Vec2f(0.05,1500.0))
    fx=UsdGeom.Xformable(front.GetPrim()); ftrans=fx.AddTranslateOp(); forient=fx.AddOrientOp()
    chase=UsdGeom.Camera.Define(stage,"/World/ThirdPersonCamera"); chase.CreateFocalLengthAttr(28.0); chase.CreateHorizontalApertureAttr(20.955); chase.CreateClippingRangeAttr(Gf.Vec2f(0.1,2500.0))
    cx=UsdGeom.Xformable(chase.GetPrim()); ctrans=cx.AddTranslateOp(); corient=cx.AddOrientOp()
    dome=UsdLux.DomeLight.Define(stage,"/World/Lights/Environment"); dome.CreateIntensityAttr(700.0); dome.CreateColorAttr(Gf.Vec3f(0.38,0.48,0.62))
    sun=UsdLux.DistantLight.Define(stage,"/World/Lights/Sun"); sun.CreateIntensityAttr(2500.0); sun.CreateColorAttr(Gf.Vec3f(1.0,0.91,0.78)); UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45,20,10))
    stage.GetRootLayer().Export(str(out/"scout_route.usd"))
    scout_art=world.scene.add(SingleArticulation(prim_path="/World/ScoutPlacement/Scout",name="scout_v2",reset_xform_properties=False))
    world.reset()
    world.play()
    dof_names=list(scout_art.dof_names)
    dof_indices=[scout_art.get_dof_index(path.rsplit("/",1)[-1]) for path in wheel_paths]
    log(f"Articulation DOFs={dof_names}; wheel indices={dof_indices}")
    for _ in range(240): world.step(render=False,update_fabric=True)

    import omni.replicator.core as rep
    if not args.no_render:
        fp_rp=rep.create.render_product(str(front.GetPath()),(args.width,args.height),name="ScoutFirstPerson")
        tp_rp=rep.create.render_product(str(chase.GetPath()),(args.width,args.height),name="ScoutThirdPerson")
        fp_writer=rep.WriterRegistry.get("BasicWriter"); fp_writer.initialize(output_dir=str(out/"first_person_frames"),rgb=True); fp_writer.attach(fp_rp)
        tp_writer=rep.WriterRegistry.get("BasicWriter"); tp_writer.initialize(output_dir=str(out/"third_person_frames"),rgb=True); tp_writer.attach(tp_rp)

    root_path="/World/ScoutPlacement/Scout"; rows=[]; progress_index=0; record_i=0
    duration=float(route["distance_m"][-1]/max(np.median(route["speed_mps"]),0.1))+20.0
    max_steps=int(math.ceil((min(duration,args.max_seconds) if args.max_seconds>0 else duration)*fps))
    for step in range(max_steps):
        robot_positions,robot_orientations=scout_art._articulation_view.get_world_poses(usd=False)
        pos=Gf.Vec3d(*[float(q) for q in robot_positions[0]])
        qw,qx,qy,qz=[float(q) for q in robot_orientations[0]]; robot_q=Gf.Quatd(qw,Gf.Vec3d(qx,qy,qz))
        fwd=robot_q.Transform(Gf.Vec3d(1,0,0)); yaw=math.atan2(float(fwd[1]),float(fwd[0]))
        d2=(route["x"][progress_index:]-float(pos[0]))**2+(route["y"][progress_index:]-float(pos[1]))**2
        progress_index += int(np.argmin(d2)); progress_index=min(progress_index,len(route["x"])-1)
        s_now=float(route["distance_m"][progress_index]); target_s=min(float(route["distance_m"][-1]),s_now+max(0.8,0.9*float(route["speed_mps"][progress_index])))
        ti=min(int(np.searchsorted(route["distance_m"],target_s)),len(route["x"])-1)
        dx=float(route["x"][ti]-pos[0]); dy=float(route["y"][ti]-pos[1]); c,s=math.cos(yaw),math.sin(yaw)
        local_y=-s*dx+c*dy; look=max(math.hypot(dx,dy),0.35); curvature=2.0*local_y/(look*look)
        v=float(route["speed_mps"][progress_index]); omega=float(np.clip(v*curvature,-1.3,1.3))
        vl=v-omega*track/2; vr=v+omega*track/2
        if progress_index>=len(route["x"])-1 and math.hypot(dx,dy)<0.45: vl=vr=0.0
        wheel_targets=np.asarray([sign*linear/wheel_r for linear,sign in zip((vl,vr,vl,vr),wheel_signs)],dtype=np.float32)
        scout_art.apply_action(ArticulationAction(joint_velocities=wheel_targets,joint_indices=np.asarray(dof_indices,dtype=np.int32)))
        for _ in range(stride): world.step(render=False,update_fabric=True)
        wheel_actual=scout_art.get_joint_velocities(joint_indices=np.asarray(dof_indices,dtype=np.int32))
        robot_positions,robot_orientations=scout_art._articulation_view.get_world_poses(usd=False)
        pos=Gf.Vec3d(*[float(q) for q in robot_positions[0]])
        qw,qx,qy,qz=[float(q) for q in robot_orientations[0]]; robot_q=Gf.Quatd(qw,Gf.Vec3d(qx,qy,qz))
        fwd=robot_q.Transform(Gf.Vec3d(1,0,0)); yaw=math.atan2(float(fwd[1]),float(fwd[0]))
        first_eye=Gf.Vec3d(float(pos[0])+0.48*math.cos(yaw),float(pos[1])+0.48*math.sin(yaw),float(pos[2])+0.48)
        ftrans.Set(first_eye)
        first_q=Gf.Rotation(Gf.Vec3d(0,0,-1),Gf.Vec3d(math.cos(yaw),math.sin(yaw),0)).GetQuat()
        forient.Set(Gf.Quatf(float(first_q.GetReal()),Gf.Vec3f(first_q.GetImaginary())))
        target=Gf.Vec3d(float(pos[0]),float(pos[1]),float(pos[2])+0.30); cam=Gf.Vec3d(float(pos[0])-5*math.cos(yaw),float(pos[1])-5*math.sin(yaw),float(pos[2])+2.3)
        forward=target-cam; forward.Normalize(); ctrans.Set(cam)
        camera_q=Gf.Rotation(Gf.Vec3d(0,0,-1),forward).GetQuat()
        corient.Set(Gf.Quatf(float(camera_q.GetReal()),Gf.Vec3f(camera_q.GetImaginary())))
        rows.append({"frame":record_i,"time_s":record_i/fps,"route_index":progress_index,"distance_m":s_now,"x":float(pos[0]),"y":float(pos[1]),"body_z":float(pos[2]),"ground_z":float(route["ground_z"][progress_index]),"yaw_rad":yaw,"speed_target_mps":v,"left_wheel_target_mps":vl,"right_wheel_target_mps":vr,"wheel_actual_rad_s":";".join(f"{float(q):.3f}" for q in wheel_actual)})
        if not args.no_render:
            app.update(); rep.orchestrator.step(rt_subframes=1,pause_timeline=False,delta_time=1.0/fps,wait_for_render=True)
        record_i+=1
        if record_i%50==0: log(f"物理路线进度 {s_now:.1f}/{route['distance_m'][-1]:.1f} m，帧 {record_i}")
        if progress_index>=len(route["x"])-1 and math.hypot(dx,dy)<0.45: break
    for drive in drives: drive.GetTargetVelocityAttr().Set(0.0)
    with (out/"trajectory_frames.csv").open("w",encoding="utf-8",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    traveled=float(rows[-1]["distance_m"]) if rows else 0.0
    complete=bool(rows) and traveled>=float(route["distance_m"][-1])*0.85
    partial=bool(rows) and args.max_seconds>0
    summary={"status":"PASS" if complete else ("PARTIAL_TEST" if partial else "FAILED_VALIDATION"),"vehicle":"AgileX Scout v2 USD","asset":str(asset),"asset_source":"https://github.com/leonlime/scout_v2_isaac","asset_stage_units":"centimetres; applied scale 0.01","wheel_radius_m":wheel_r,"track_width_m":track,"wheel_axis_forward_signs":wheel_signs,"wheel_dof_names":dof_names,"wheel_dof_indices":dof_indices,"isaac_sim_version":ver,"route_csv":str(Path(args.grounded_route)),"route_distance_m":float(route["distance_m"][-1]),"traveled_route_progress_m":traveled,"frames":len(rows),"fps":fps,"first_person_frames":str(out/"first_person_frames"),"third_person_frames":str(out/"third_person_frames"),"trajectory_csv":str(out/"trajectory_frames.csv"),"rendered":not args.no_render}
    (out/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8"); print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
    print("SCOUT_ROUTE_SIM_PASS" if summary["status"]=="PASS" else ("SCOUT_ROUTE_SIM_PARTIAL" if partial else "SCOUT_ROUTE_SIM_FAILED"),flush=True)
    exit_code=0 if complete or partial else 2
except BaseException as exc:
    traceback.print_exc(); (out/"failure.json").write_text(json.dumps({"status":"FAILED","exception":str(exc),"traceback":traceback.format_exc()},ensure_ascii=False,indent=2),encoding="utf-8")
finally:
    try: app.close()
    except BaseException: pass
raise SystemExit(exit_code)
