#!/usr/bin/env python3
"""Densify manually surveyed XY and obtain Z from the campus mesh without moving XY."""
from __future__ import annotations
import argparse, csv, json, math, sys, time, traceback
from pathlib import Path
import numpy as np

parser=argparse.ArgumentParser(); parser.add_argument("--config",required=True)
parser.add_argument("--input-csv",default=None)
parser.add_argument("--output-dir",default=None)
a=parser.parse_args()
cfg=json.loads(Path(a.config).read_text(encoding="utf-8")); root=Path(cfg["project_root"])
out=Path(a.output_dir) if a.output_dir else root/"outputs/control_route_ground"
out.mkdir(parents=True,exist_ok=True); started=time.monotonic()
def stamp(s): print(f"[{time.monotonic()-started:8.2f}s] {s}",flush=True)
from isaacsim import SimulationApp
app=SimulationApp({"headless":True,"renderer":cfg["render"]["renderer"],"width":640,"height":360,
                   "multi_gpu":False,"max_gpu_count":1,"disable_viewport_updates":True})

def read_control_points(path):
    with Path(path).open("r",encoding="utf-8-sig",newline="") as f: rows=list(csv.DictReader(f))
    if len(rows)<2: raise ValueError("路线控制点至少需要2个")
    points=[]
    for r in rows:
        points.append({"point_id":int(r["point_id"]),"x":float(r["x_world"]),"y":float(r["y_world"]),
                       "speed":float(r["speed_mps"]),"route_name":r.get("route_name","route")})
    return points

def densify(control,spacing):
    xy=np.asarray([[p["x"],p["y"]] for p in control],dtype=np.float64)
    seg=np.linalg.norm(np.diff(xy,axis=0),axis=1); s=np.r_[0,np.cumsum(seg)]
    samples=np.arange(0,s[-1],spacing); samples=np.r_[samples,s[-1]]
    x=np.interp(samples,s,xy[:,0]); y=np.interp(samples,s,xy[:,1])
    speed=np.interp(samples,s,np.asarray([p["speed"] for p in control]))
    return samples,x,y,speed

def transform(points,matrix):
    hom=np.ones((len(points),4),dtype=np.float64); hom[:,:3]=points
    return (hom@np.asarray(matrix,dtype=np.float64))[:,:3]

def collect_corridor(stage,x,y,half_width,chunk_size):
    from pxr import Usd,UsdGeom
    xmin,xmax=x.min()-half_width,x.max()+half_width; ymin,ymax=y.min()-half_width,y.max()+half_width
    cache=UsdGeom.XformCache(Usd.TimeCode.Default()); pieces=[]; scanned=kept=0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh): continue
        mesh=UsdGeom.Mesh(prim); pr=mesh.GetPointsAttr().Get(); cr=mesh.GetFaceVertexCountsAttr().Get(); ir=mesh.GetFaceVertexIndicesAttr().Get()
        if not pr or not cr or not ir: continue
        counts=np.asarray(cr,dtype=np.int64); indices=np.asarray(ir,dtype=np.int64)
        if not np.all(counts==3): continue
        world=transform(np.asarray(pr,dtype=np.float64),cache.GetLocalToWorldTransform(prim)); tri=indices.reshape(-1,3)
        stamp(f"扫描路线走廊：{prim.GetPath()}，{len(tri):,}个三角面")
        for begin in range(0,len(tri),chunk_size):
            ids=tri[begin:begin+chunk_size]; p0,p1,p2=world[ids[:,0]],world[ids[:,1]],world[ids[:,2]]; scanned+=len(ids)
            tmin=np.minimum(np.minimum(p0[:,:2],p1[:,:2]),p2[:,:2]); tmax=np.maximum(np.maximum(p0[:,:2],p1[:,:2]),p2[:,:2])
            k=(tmax[:,0]>=xmin)&(tmin[:,0]<=xmax)&(tmax[:,1]>=ymin)&(tmin[:,1]<=ymax)
            if k.any(): pieces.append((p0[k],p1[k],p2[k])); kept+=int(k.sum())
    if not pieces: raise RuntimeError("路线走廊内没有Mesh三角面")
    return tuple(np.concatenate([p[i] for p in pieces],axis=0) for i in range(3)),{"triangles_scanned":scanned,"triangles_in_corridor":kept,"bounds_xy":[xmin,xmax,ymin,ymax]}

def probe_point(qx,qy,triangles,offsets,rcfg):
    p0,p1,p2=triangles; e0=p1-p0; e1=p2-p0
    normal=np.cross(e0,e1); nl=np.linalg.norm(normal,axis=1); up=np.abs(normal[:,2])/np.maximum(nl,1e-12)
    slope_limit=math.cos(math.radians(float(rcfg["max_surface_slope_deg"])))
    denom=e0[:,0]*e1[:,1]-e0[:,1]*e1[:,0]
    base=(nl>1e-10)&(up>=slope_limit)&(np.abs(denom)>1e-12)
    hits=[]
    for dx,dy in offsets:
        x,y=qx+dx,qy+dy; vx=x-p0[:,0]; vy=y-p0[:,1]
        u=(vx*e1[:,1]-vy*e1[:,0])/np.where(np.abs(denom)>1e-12,denom,1.0)
        v=(e0[:,0]*vy-e0[:,1]*vx)/np.where(np.abs(denom)>1e-12,denom,1.0)
        inside=base&(u>=-1e-7)&(v>=-1e-7)&(u+v<=1.0000001)
        if not inside.any(): continue
        zz=p0[inside,2]+u[inside]*e0[inside,2]+v[inside]*e1[inside,2]
        uu=up[inside]
        for z,upv in zip(zz.tolist(),uu.tolist()):
            if float(rcfg["min_ground_z_m"])<=z<=float(rcfg["max_ground_z_m"]):
                hits.append({"z":z,"offset":math.hypot(dx,dy),"slope":math.degrees(math.acos(np.clip(upv,0,1)))})
        if hits and dx==0 and dy==0: break
    return hits

def choose_heights(all_hits,distance,rcfg):
    chosen=[None]*len(all_hits); previous=None
    for i,hits in enumerate(all_hits):
        if not hits: continue
        unique={round(h["z"],4):h for h in sorted(hits,key=lambda h:(h["offset"],h["slope"]))}
        candidates=list(unique.values())
        if previous is None:
            chosen[i]=min(candidates,key=lambda h:(abs(h["z"]-float(cfg["ground_probe"]["calibrated_p1_ground_z"])),h["offset"],h["slope"]))
        else:
            ds=max(distance[i]-distance[i-1],1e-3); allowed=float(rcfg["max_grade"])*ds+0.15
            valid=[h for h in candidates if abs(h["z"]-previous)<=allowed]
            chosen[i]=min(valid or candidates,key=lambda h:(abs(h["z"]-previous),h["offset"],h["slope"]))
        previous=chosen[i]["z"]
    z=np.asarray([np.nan if h is None else h["z"] for h in chosen]); valid=np.isfinite(z)
    if valid.sum()<2: raise RuntimeError("有效地面交点少于2个")
    z[~valid]=np.interp(distance[~valid],distance[valid],z[valid])
    raw_z=z.copy()
    # Reject isolated mesh fragments with a robust spatial median before the
    # low-pass filter.  The route is a road, not a terrain-following drone path.
    median_window=int(rcfg.get("robust_z_median_window",31)); median_window=max(1,median_window|1)
    if median_window>1:
        half=median_window//2
        z=np.asarray([np.median(raw_z[max(0,i-half):min(len(raw_z),i+half+1)]) for i in range(len(raw_z))])
    window=int(rcfg.get("z_smoothing_window",21)); window=max(1,window|1)
    if window>1:
        pad=window//2; z=np.convolve(np.pad(z,(pad,pad),mode="edge"),np.ones(window)/window,mode="valid")
    # Enforce the same grade bound in both travel directions so a single bad
    # layer cannot create a ramp that only passes a forward-only clamp.
    grade=float(rcfg.get("output_max_grade",0.08))
    for i in range(1,len(z)):
        dz=grade*max(distance[i]-distance[i-1],1e-6); z[i]=np.clip(z[i],z[i-1]-dz,z[i-1]+dz)
    for i in range(len(z)-2,-1,-1):
        dz=grade*max(distance[i+1]-distance[i],1e-6); z[i]=np.clip(z[i],z[i+1]-dz,z[i+1]+dz)
    return chosen,z,valid,raw_z

exit_code=1
try:
    import omni.usd
    from pxr import Usd
    rcfg=cfg["control_route"]
    input_csv=Path(a.input_csv) if a.input_csv else Path(rcfg["input_csv"])
    control=read_control_points(input_csv)
    distance,x,y,speed=densify(control,float(rcfg["sample_spacing_m"])); stamp(f"路线{distance[-1]:.2f}米，加密为{len(x)}点，XY严格锁定")
    stage=Usd.Stage.Open(str(Path(cfg["campus_usd"]))); 
    if not stage: raise RuntimeError("校园USD打开失败")
    triangles,mesh_diag=collect_corridor(stage,x,y,float(rcfg["corridor_half_width_m"]),int(cfg["ground_probe"]["triangle_chunk_size"]))
    radius=float(rcfg["nearby_probe_radius_m"]); step=float(rcfg["nearby_probe_step_m"])
    axis=np.arange(-radius,radius+step*.5,step); offsets=[(0.,0.)]+sorted([(dx,dy) for dx in axis for dy in axis if 0<dx*dx+dy*dy<=radius*radius],key=lambda p:p[0]*p[0]+p[1]*p[1])
    hits=[]
    for i,(qx,qy) in enumerate(zip(x,y)):
        hits.append(probe_point(qx,qy,triangles,offsets,rcfg))
        if i==0 or (i+1)%25==0 or i+1==len(x): stamp(f"地面探测：{i+1}/{len(x)}")
    chosen,z,direct,raw_z=choose_heights(hits,distance,rcfg); direct_ratio=float(direct.mean())
    if direct_ratio<float(rcfg["minimum_direct_hit_ratio"]): raise RuntimeError(f"地面有效率{direct_ratio:.1%}低于阈值")
    yaw=np.unwrap(np.arctan2(np.gradient(y),np.gradient(x))); dt=np.zeros(len(x)); dt[1:]=np.diff(distance)/np.maximum(speed[1:],0.05); times=np.cumsum(dt)
    rows=[]
    for i in range(len(x)):
        h=chosen[i]; rows.append({"frame":i,"time_s":times[i],"distance_m":distance[i],"x":x[i],"y":y[i],"raw_ground_z":raw_z[i],"ground_z":z[i],"yaw_rad":yaw[i],"speed_mps":speed[i],"xy_locked":1,"z_status":"DIRECT" if direct[i] and h and h["offset"]<1e-8 else ("NEARBY_Z" if direct[i] else "INTERPOLATED_Z"),"z_probe_offset_m":"" if h is None else h["offset"]})
    csv_path=out/"grounded_control_route.csv"
    with csv_path.open("w",encoding="utf-8",newline="") as f: w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    from PIL import Image,ImageDraw
    W,H=1200,650; im=Image.new("RGB",(W,H),(18,23,31)); d=ImageDraw.Draw(im); margin=55
    # Preserve one common meters-to-pixels scale.  Independent X/Y fitting
    # exaggerated this almost-straight route into a large zigzag.
    span_x=max(x.max()-x.min(),1e-6); span_y=max(y.max()-y.min(),1e-6)
    map_scale=min((W-2*margin)/span_x,(H-2*margin)/span_y)
    cx=(x.min()+x.max())*.5; cy=(y.min()+y.max())*.5
    def px(qx): return W*.5+(qx-cx)*map_scale
    def py(qy): return H*.5-(qy-cy)*map_scale
    path=[(px(qx),py(qy)) for qx,qy in zip(x,y)]; d.line(path,fill=(52,229,141),width=4)
    for p in control: d.ellipse((px(p["x"])-6,py(p["y"])-6,px(p["x"])+6,py(p["y"])+6),fill=(255,196,74))
    d.text((20,18),f"Locked XY route  length={distance[-1]:.2f}m  samples={len(x)}  direct-Z={direct_ratio:.1%}",fill=(235,240,248)); im.save(out/"route_xy_check.png")
    profile=Image.new("RGB",(1200,500),(18,23,31)); pd=ImageDraw.Draw(profile); zlo,zhi=float(z.min()),float(z.max()); pts=[(margin+s/distance[-1]*(1200-2*margin),450-(zz-zlo)/max(zhi-zlo,.01)*380) for s,zz in zip(distance,z)]; pd.line(pts,fill=(78,174,255),width=3); pd.text((20,18),f"Ground profile  z=[{zlo:.3f},{zhi:.3f}]m",fill=(235,240,248)); profile.save(out/"route_z_profile.png")
    raw_grade=float(np.max(np.abs(np.diff(raw_z)/np.maximum(np.diff(distance),1e-6))))
    final_grade=float(np.max(np.abs(np.diff(z)/np.maximum(np.diff(distance),1e-6))))
    diag={"status":"PASS","input_csv":str(input_csv),"control_points":len(control),"samples":len(x),"distance_m":distance[-1],"duration_s":times[-1],"spacing_m":float(rcfg["sample_spacing_m"]),"direct_z_ratio":direct_ratio,"xy_modified":False,"raw_z_range_m":[float(raw_z.min()),float(raw_z.max())],"final_z_range_m":[float(z.min()),float(z.max())],"raw_max_grade":raw_grade,"final_max_grade":final_grade,"mesh":mesh_diag,"output":str(csv_path)}
    (out/"summary.json").write_text(json.dumps(diag,ensure_ascii=False,indent=2),encoding="utf-8"); print(json.dumps(diag,ensure_ascii=False,indent=2)); print("CONTROL_ROUTE_GROUND_PASS"); exit_code=0
except BaseException as e:
    traceback.print_exc(); (out/"failure.json").write_text(json.dumps({"status":"FAILED","exception":str(e),"traceback":traceback.format_exc()},ensure_ascii=False,indent=2),encoding="utf-8")
finally:
    try: app.close()
    except BaseException: pass
sys.exit(exit_code)
