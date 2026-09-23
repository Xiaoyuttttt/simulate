#!/usr/bin/env python3
"""Convert animated car poses to OpenCV-style 3DGS camera records."""
import argparse, csv, json, math
from pathlib import Path

p = argparse.ArgumentParser(); p.add_argument("--config", required=True)
p.add_argument("--source-csv", default=None); p.add_argument("--output", default=None)
p.add_argument("--width", type=int, default=960); p.add_argument("--height", type=int, default=540)
p.add_argument("--target-fps", type=float, default=0.0)
p.add_argument("--camera-forward-m", type=float, default=0.45)
p.add_argument("--camera-height-body-m", type=float, default=1.08)
p.add_argument("--camera-height-ground-m", type=float, default=None)
p.add_argument("--pitch-down-deg", type=float, default=0.0)
p.add_argument("--max-duration-s", type=float, default=0.0); a = p.parse_args()
cfg = json.loads(Path(a.config).read_text(encoding="utf-8")); root = Path(cfg["project_root"])
source = Path(a.source_csv) if a.source_csv else root / "outputs/car_animation/trajectory_frames.csv"
output = Path(a.output) if a.output else root / "outputs/3dgs_camera/camera_path.json"; summary_path = output.with_name("camera_path_summary.json")
width, height, focal_mm, aperture_mm = a.width, a.height, 18.0, 20.955
fx = width * focal_mm / aperture_mm
if not source.is_file(): raise SystemExit(f"请先完成第三阶段：{source}")
with source.open("r", encoding="utf-8-sig", newline="") as f: rows = list(csv.DictReader(f))
if a.max_duration_s > 0:
    rows = [row for row in rows if float(row["time_s"]) <= a.max_duration_s + 1e-9]
    if not rows: raise SystemExit("指定时长内没有轨迹帧")
if a.target_fps > 0 and len(rows) > 1:
    duration = float(rows[-1]["time_s"])
    target_times = [i / a.target_fps for i in range(int(round(duration * a.target_fps)) + 1)]
    source_times = [float(row["time_s"]) for row in rows]
    selected=[]; cursor=0
    for target_time in target_times:
        while cursor+1 < len(rows) and abs(source_times[cursor+1]-target_time) <= abs(source_times[cursor]-target_time): cursor += 1
        selected.append(rows[cursor])
    rows = selected
records = []
pitch = math.radians(a.pitch_down_deg); cp, sp = math.cos(pitch), math.sin(pitch)
camera_pose_fields = ("camera_x","camera_y","camera_z","camera_qw","camera_qx","camera_qy","camera_qz")
use_recorded_camera_pose = bool(rows) and all(field in rows[0] and rows[0][field] != "" for field in camera_pose_fields)

def rotate_by_quaternion(vector, quaternion):
    w,x,y,z = quaternion; vx,vy,vz = vector
    norm = math.sqrt(w*w+x*x+y*y+z*z)
    if norm < 1e-12: raise ValueError("相机四元数长度为0")
    w,x,y,z = w/norm,x/norm,y/norm,z/norm
    # q * [0,v] * conjugate(q), expanded as a 3x3 rotation.
    return [
        (1-2*(y*y+z*z))*vx + 2*(x*y-z*w)*vy + 2*(x*z+y*w)*vz,
        2*(x*y+z*w)*vx + (1-2*(x*x+z*z))*vy + 2*(y*z-x*w)*vz,
        2*(x*z-y*w)*vx + 2*(y*z+x*w)*vy + (1-2*(x*x+y*y))*vz,
    ]

for sequence,row in enumerate(rows):
    if use_recorded_camera_pose:
        eye = [float(row["camera_x"]),float(row["camera_y"]),float(row["camera_z"])]
        quaternion = [float(row["camera_qw"]),float(row["camera_qx"]),float(row["camera_qy"]),float(row["camera_qz"])]
        forward = rotate_by_quaternion([0.0,0.0,-1.0],quaternion)
        up = rotate_by_quaternion([0.0,1.0,0.0],quaternion)
    else:
        yaw = float(row["yaw_rad"]); c, s = math.cos(yaw), math.sin(yaw)
        if a.camera_height_ground_m is None:
            camera_z = float(row["body_z"]) + a.camera_height_body_m
        else:
            camera_z = float(row["ground_z"]) + a.camera_height_ground_m
        eye = [float(row["x"]) + a.camera_forward_m*c, float(row["y"]) + a.camera_forward_m*s, camera_z]
        forward = [c*cp, s*cp, -sp]; up = [c*sp, s*sp, cp]
    records.append({"frame_id": sequence, "source_frame_id": int(row["frame"]), "time_s": float(row["time_s"]),
        "eye": eye, "forward": forward, "up": up,
        "target": [eye[i]+forward[i] for i in range(3)], "fx": fx, "fy": fx,
        "cx": width/2, "cy": height/2, "width": width, "height": height})
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
indices = [0, len(records)//4, len(records)//2, 3*len(records)//4, len(records)-1]
summary = {"status":"PASS", "frame_count":len(records), "preview_indices":indices,
    "source_csv":str(source), "target_fps":a.target_fps,
    "camera_pose_source":"recorded_world_pose" if use_recorded_camera_pose else "reconstructed_from_body_pose",
    "max_duration_s":a.max_duration_s,
    "camera_height_reference":"ground" if a.camera_height_ground_m is not None else "body_root",
    "camera_height_m":a.camera_height_ground_m if a.camera_height_ground_m is not None else a.camera_height_body_m,
    "camera_local_translation_body_xyz_m":None if a.camera_height_ground_m is not None else [a.camera_forward_m,0.0,a.camera_height_body_m],
    "pitch_down_deg":a.pitch_down_deg,
    "camera_path":str(output), "coordinate_frame":"campus mesh/PLY world, Z-up",
    "camera_axes":"OpenCV right-down-forward", "intrinsics":{"fx":fx,"fy":fx,"cx":width/2,"cy":height/2}}
summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(summary, ensure_ascii=False, indent=2)); print("CAMERA_PATH_EXPORT_PASS")
