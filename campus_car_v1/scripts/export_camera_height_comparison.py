#!/usr/bin/env python3
"""Export 15 records comparing low, medium and high route cameras."""
import argparse,csv,json,math
from pathlib import Path
p=argparse.ArgumentParser(); p.add_argument("--config",required=True); p.add_argument("--output",required=True); p.add_argument("--width",type=int,default=1280); p.add_argument("--height",type=int,default=720)
p.add_argument("--variant",choices=("all","low","mid","high"),default="all"); a=p.parse_args()
cfg=json.loads(Path(a.config).read_text(encoding="utf-8")); root=Path(cfg["project_root"])
source=root/"outputs/control_route_animation/trajectory_frames.csv"
with source.open("r",encoding="utf-8-sig",newline="") as f: rows=list(csv.DictReader(f))
if len(rows)<5: raise SystemExit(f"轨迹帧不足：{len(rows)}")
indices=[0,len(rows)//4,len(rows)//2,3*len(rows)//4,len(rows)-1]
variants=[("LOW_1.08m_pitch0",1.08,0.0),("MID_2.50m_pitch10",2.50,10.0),("HIGH_4.00m_pitch18",4.00,18.0)]
if a.variant != "all":
    wanted={"low":"LOW_","mid":"MID_","high":"HIGH_"}[a.variant]
    variants=[v for v in variants if v[0].startswith(wanted)]
focal_mm,aperture_mm=18.0,20.955; fx=a.width*focal_mm/aperture_mm; records=[]
for label,height,pitch_deg in variants:
    pitch=math.radians(pitch_deg); cp,sp=math.cos(pitch),math.sin(pitch)
    for route_order,index in enumerate(indices):
        row=rows[index]; yaw=float(row["yaw_rad"]); c,s=math.cos(yaw),math.sin(yaw)
        # Comparison heights are exact camera heights above the probed ground,
        # not offsets above the already elevated chassis root.
        eye=[float(row["x"])+.45*c,float(row["y"])+.45*s,float(row["ground_z"])+height]
        forward=[c*cp,s*cp,-sp]; up=[c*sp,s*sp,cp]
        records.append({"frame_id":len(records),"source_frame_id":int(row["frame"]),"time_s":float(row["time_s"]),"camera_label":f"{label} route_{route_order+1}/5","eye":eye,"forward":forward,"up":up,"target":[eye[i]+forward[i] for i in range(3)],"fx":fx,"fy":fx,"cx":a.width/2,"cy":a.height/2,"width":a.width,"height":a.height})
out=Path(a.output); out.parent.mkdir(parents=True,exist_ok=True); out.write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding="utf-8")
summary={"status":"PASS","source":str(source),"route_indices":indices,"height_reference":"camera_above_ground","variants":[{"label":v[0],"height_m":v[1],"pitch_down_deg":v[2]} for v in variants],"records":len(records),"output":str(out)}
out.with_name("height_comparison_camera_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8"); print(json.dumps(summary,ensure_ascii=False,indent=2)); print("CAMERA_HEIGHT_COMPARISON_EXPORT_PASS")
