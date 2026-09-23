#!/usr/bin/env python3
"""Render five calibration frames from the car camera against campus 3DGS."""
import argparse, json
from pathlib import Path
import numpy as np, torch
from PIL import Image, ImageDraw
from plyfile import PlyData
from gsplat import rasterization

C0 = 0.28209479177387814
p = argparse.ArgumentParser(); p.add_argument("--ply", required=True); p.add_argument("--camera-path", required=True)
p.add_argument("--output-dir", required=True); p.add_argument("--max-gaussians", type=int, default=5_000_000)
p.add_argument("--device", default="cuda:0"); p.add_argument("--all-frames", action="store_true")
p.add_argument("--resume", action="store_true"); p.add_argument("--preview-all-records", action="store_true")
p.add_argument("--route-crop-margin", type=float, default=0.0)
p.add_argument("--preserve-all-valid", action="store_true")
p.add_argument("--max-scale-quantile", type=float, default=0.0)
p.add_argument("--rasterize-mode", choices=("classic","antialiased"), default="classic"); a = p.parse_args()
out = Path(a.output_dir); frames_dir = out / ("background_frames" if a.all_frames else "preview_frames")
frames_dir.mkdir(parents=True, exist_ok=True)

def fields(v, names): return np.stack([np.asarray(v[n], dtype=np.float32) for n in names], axis=-1)
def viewmat(eye, forward, up):
    forward = forward / np.linalg.norm(forward); up = up-forward*np.dot(up, forward); up /= np.linalg.norm(up)
    right = np.cross(forward, up); right /= np.linalg.norm(right); rotation = np.stack((right, -up, forward), axis=0)
    view = np.eye(4, dtype=np.float32); view[:3,:3] = rotation; view[:3,3] = -rotation @ eye; return view

print("读取3DGS PLY……", flush=True); vertex = PlyData.read(a.ply)["vertex"].data; total = len(vertex)
cameras=json.loads(Path(a.camera_path).read_text(encoding="utf-8"))
means = fields(vertex,["x","y","z"]); quats = fields(vertex,["rot_0","rot_1","rot_2","rot_3"])
scales = np.exp(fields(vertex,["scale_0","scale_1","scale_2"])); opacity = 1/(1+np.exp(-np.asarray(vertex["opacity"],dtype=np.float32)))
colors = np.clip(0.5+C0*fields(vertex,["f_dc_0","f_dc_1","f_dc_2"]),0,1)
finite = np.isfinite(means).all(1)&np.isfinite(scales).all(1)&np.isfinite(quats).all(1)&np.isfinite(colors).all(1)&np.isfinite(opacity)
crop_bounds=None
if a.route_crop_margin > 0:
    eyes=np.asarray([record["eye"] for record in cameras],dtype=np.float32); margin=float(a.route_crop_margin)
    crop_min=eyes.min(axis=0)-margin; crop_max=eyes.max(axis=0)+margin
    # Keep a generous vertical range while cropping the horizontal campus
    # extent around the driven route.  This retains all nearby detail instead
    # of randomly spending the budget on distant campus regions.
    route_local=(means[:,0]>=crop_min[0])&(means[:,0]<=crop_max[0])&(means[:,1]>=crop_min[1])&(means[:,1]<=crop_max[1])
    finite &= route_local; crop_bounds={"min":crop_min.tolist(),"max":crop_max.tolist()}
finite_total=int(finite.sum()); scale_cap=None
if a.preserve_all_valid:
    keep=finite
else:
    cap = float(np.quantile(scales[finite].max(1),0.995)); keep = finite&(opacity>0.01)&(scales.max(1)<=cap)
if a.max_scale_quantile > 0:
    if not 0 < a.max_scale_quantile < 1:
        raise SystemExit("--max-scale-quantile必须位于0和1之间，或使用0关闭")
    scale_cap=float(np.quantile(scales[finite].max(1),a.max_scale_quantile))
    keep &= scales.max(1) <= scale_cap
means,quats,scales,opacity,colors = means[keep],quats[keep],scales[keep],opacity[keep],colors[keep]
valid_total = len(means)
if a.max_gaussians > 0 and valid_total > a.max_gaussians:
    rng = np.random.default_rng(20260902); idx = np.sort(rng.choice(valid_total, a.max_gaussians, replace=False))
    means,quats,scales,opacity,colors = means[idx],quats[idx],scales[idx],opacity[idx],colors[idx]
low, high = np.quantile(means,.01,axis=0), np.quantile(means,.99,axis=0); center=(low+high)*.5; radius=float(np.max(high-low)*.5)
means=(means-center)/radius; scales=scales/radius; device=torch.device(a.device)
means=torch.from_numpy(means).float().to(device); quats=torch.from_numpy(quats).float().to(device)
quats=quats/torch.linalg.norm(quats,dim=-1,keepdim=True).clamp_min(1e-8)
scales=torch.from_numpy(scales).float().to(device); opacity=torch.from_numpy(opacity).float().to(device); colors=torch.from_numpy(colors).float().to(device)
indices=(list(range(len(cameras))) if (a.all_frames or a.preview_all_records) else [0,len(cameras)//4,len(cameras)//2,3*len(cameras)//4,len(cameras)-1])
images=[]
with torch.inference_mode():
  for index in indices:
    r=cameras[index]; eye=(np.asarray(r["eye"],dtype=np.float64)-center)/radius
    frame_path=frames_dir/f"frame_{index:04d}.png"
    resume_valid=False
    if a.resume and frame_path.is_file() and frame_path.stat().st_size > 1000:
      try:
        with Image.open(frame_path) as existing: resume_valid=existing.size==(int(r["width"]),int(r["height"]))
      except Exception: resume_valid=False
    if resume_valid:
      if not a.all_frames: images.append(Image.open(frame_path).convert("RGB"))
      print(f"跳过已有帧：{index}",flush=True); continue
    vm=viewmat(eye,np.asarray(r["forward"],dtype=np.float64),np.asarray(r["up"],dtype=np.float64))
    K=np.asarray([[r["fx"],0,r["cx"]],[0,r["fy"],r["cy"]],[0,0,1]],dtype=np.float32)
    rendered,alpha,_=rasterization(means=means,quats=quats,scales=scales,opacities=opacity,colors=colors,
      viewmats=torch.from_numpy(vm)[None].to(device),Ks=torch.from_numpy(K)[None].to(device),
      width=int(r["width"]),height=int(r["height"]),sh_degree=None,packed=True,rasterize_mode=a.rasterize_mode)
    arr=(rendered[0,...,:3].clamp(0,1)*255).byte().cpu().numpy(); image=Image.fromarray(arr,"RGB")
    if not a.all_frames:
      label=r.get("camera_label",f"frame {index}")
      draw=ImageDraw.Draw(image); draw.rectangle((0,0,420,32),fill=(0,0,0)); draw.text((8,8),f"{label}  t={r['time_s']:.2f}s",fill=(255,255,0))
    image.save(frame_path)
    if not a.all_frames: images.append(image)
    if not a.all_frames or index == 0 or (index+1)%10 == 0 or index+1 == len(cameras):
      print(f"渲染进度：{index+1}/{len(cameras)}",flush=True)
if not a.all_frames:
  sheet_width=int(cameras[indices[0]]["width"]); sheet_height=int(cameras[indices[0]]["height"])
  sheet=Image.new("RGB",(sheet_width,sheet_height*len(images)))
  for i,img in enumerate(images): sheet.paste(img,(0,i*sheet_height))
  sheet.save(out/"camera_preview_strip.png")
summary={"status":"PASS","ply":a.ply,"source_gaussians":total,"finite_gaussians_before_filter":finite_total,"valid_gaussians_before_sampling":valid_total,"retained_gaussians":len(means),"route_crop_margin_m":a.route_crop_margin,"route_crop_bounds":crop_bounds,"preserve_all_valid":a.preserve_all_valid,"max_scale_quantile":a.max_scale_quantile,"max_scale_cap_world_units":scale_cap,"removed_by_deterministic_filters":finite_total-valid_total,"rasterize_mode":a.rasterize_mode,"indices":indices,
 "mode":"full" if a.all_frames else "preview", "frames_dir":str(frames_dir)}
if not a.all_frames: summary["preview_strip"]=str(out/"camera_preview_strip.png")
(out/("full_summary.json" if a.all_frames else "summary.json")).write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
print(json.dumps(summary,ensure_ascii=False,indent=2)); print("THREEDGS_CAMERA_FULL_PASS" if a.all_frames else "THREEDGS_CAMERA_PREVIEW_PASS")
