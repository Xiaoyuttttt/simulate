#!/usr/bin/env python3
"""Crop a Gaussian PLY to a camera-route XY bounding box without changing attributes."""
import argparse,json
from pathlib import Path
import numpy as np
from plyfile import PlyData,PlyElement

p=argparse.ArgumentParser()
p.add_argument("--input",required=True)
p.add_argument("--camera-path",required=True)
p.add_argument("--output",required=True)
p.add_argument("--margin",type=float,default=40.0)
a=p.parse_args()

cameras=json.loads(Path(a.camera_path).read_text(encoding="utf-8"))
eyes=np.asarray([r["eye"] for r in cameras],dtype=np.float64)
lo=eyes[:,:2].min(axis=0)-a.margin
hi=eyes[:,:2].max(axis=0)+a.margin
print(f"裁剪范围：X [{lo[0]:.3f}, {hi[0]:.3f}] Y [{lo[1]:.3f}, {hi[1]:.3f}]",flush=True)

source=PlyData.read(a.input)
vertex=source["vertex"].data
x=np.asarray(vertex["x"]); y=np.asarray(vertex["y"])
mask=(x>=lo[0])&(x<=hi[0])&(y>=lo[1])&(y<=hi[1])
cropped=vertex[mask]
if len(cropped)==0:
    raise SystemExit("裁剪区域内没有高斯")

out=Path(a.output); out.parent.mkdir(parents=True,exist_ok=True)
element=PlyElement.describe(cropped,"vertex")
PlyData([element],text=False,byte_order="<",comments=list(source.comments),obj_info=list(source.obj_info)).write(out)
summary={"status":"PASS","input":a.input,"output":str(out),"source_gaussians":len(vertex),"cropped_gaussians":len(cropped),"retained_ratio":len(cropped)/len(vertex),"margin_m":a.margin,"bounds_xy":{"min":lo.tolist(),"max":hi.tolist()},"all_vertex_properties_preserved":list(vertex.dtype.names)==list(cropped.dtype.names),"properties":list(cropped.dtype.names)}
out.with_suffix(".summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
print(json.dumps(summary,ensure_ascii=False,indent=2)); print("ROUTE_REGION_PLY_CROP_PASS")
