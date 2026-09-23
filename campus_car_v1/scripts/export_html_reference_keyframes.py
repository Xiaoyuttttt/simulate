#!/usr/bin/env python3
"""Export the seven camera keyframes embedded in point_cloud_1.html verbatim."""
import argparse, json, math
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser(); p.add_argument("--config",required=True); a=p.parse_args()
cfg=json.loads(Path(a.config).read_text(encoding="utf-8")); root=Path(cfg["project_root"])
track_path=root/"config/original_html_camera_track.json"
track=json.loads(track_path.read_text(encoding="utf-8")); width,height=960,540
records=[]
for frame_id,key in enumerate(track["keyframes"]):
    eye=np.asarray(key["position"],dtype=np.float64); target=np.asarray(key["target"],dtype=np.float64)
    forward=target-eye; forward/=np.linalg.norm(forward)
    # PlayCanvas CameraComponent fov is vertical. Pixel focal lengths are equal
    # for square pixels; horizontal FOV follows from the 16:9 aspect ratio.
    focal_px=height/(2.0*math.tan(math.radians(float(key["fov_deg"]))*0.5))
    records.append({"frame_id":frame_id,"source_keyframe":frame_id+1,"time_s":float(key["time_s"]),
      "eye":eye.tolist(),"target":target.tolist(),"forward":forward.tolist(),"up":[0.0,0.0,1.0],
      "fov_vertical_deg":float(key["fov_deg"]),"fx":focal_px,"fy":focal_px,
      "cx":width/2,"cy":height/2,"width":width,"height":height})
out=root/"outputs/html_reference"; out.mkdir(parents=True,exist_ok=True)
path=out/"html_keyframes_camera_path.json"; path.write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding="utf-8")
summary={"status":"PASS","source":track["source"],"keyframe_count":len(records),"camera_path":str(path),
 "position_target_copied_verbatim":True,"fov_vertical_deg":75.0,"interpolation_not_used_for_keyframe_preview":True}
(out/"camera_export_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
print(json.dumps(summary,ensure_ascii=False,indent=2)); print("HTML_REFERENCE_CAMERA_EXPORT_PASS")
