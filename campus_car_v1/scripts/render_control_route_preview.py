#!/usr/bin/env python3
"""Render nine chase-camera diagnostics from the new locked-XY car stage."""
from __future__ import annotations
import argparse,asyncio,csv,json,math,sys,time,traceback
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser(); p.add_argument("--config",required=True); p.add_argument("--views",type=int,default=9); a=p.parse_args()
cfg=json.loads(Path(a.config).read_text(encoding="utf-8")); root=Path(cfg["project_root"])
stage_path=root/"stages/campus_car_control_route.usd"; trajectory_path=root/"outputs/control_route_animation/trajectory_frames.csv"
out=root/"outputs/control_route_preview"; out.mkdir(parents=True,exist_ok=True); started=time.monotonic()
def stamp(s): print(f"[{time.monotonic()-started:8.2f}s] {s}",flush=True)
from isaacsim import SimulationApp
app=SimulationApp({"headless":True,"renderer":cfg["render"]["renderer"],"width":960,"height":540,
                   "multi_gpu":False,"max_gpu_count":1,"sync_loads":True,"disable_viewport_updates":False,
                   "extra_args":["--/renderer/multiGpu/enabled=false","--/renderer/multiGpu/autoEnable=false"]})

def wait_stage(context,timeout=180):
    deadline=time.monotonic()+timeout; quiet=0
    while time.monotonic()<deadline and quiet<3:
        app.update(); status=context.get_stage_loading_status(); quiet=quiet+1 if len(status)>=3 and status[2]==0 else 0
    if quiet<3: raise TimeoutError("Stage依赖加载超时")
def wait_task(task,timeout=185):
    deadline=time.monotonic()+timeout
    while not task.done() and time.monotonic()<deadline: app.update()
    if not task.done(): task.cancel(); raise TimeoutError("截图超时")
    task.result()
async def capture(viewport,path):
    from omni.kit.viewport.utility import capture_viewport_to_file
    c=capture_viewport_to_file(viewport,str(path),is_hdr=False); await c.wait_for_result()

exit_code=1
try:
    import omni.timeline,omni.usd
    from omni.kit.viewport.utility import get_active_viewport
    from isaacsim.core.utils.viewports import set_camera_view
    from PIL import Image,ImageDraw
    if not stage_path.is_file(): raise FileNotFoundError(f"缺少动画Stage：{stage_path}")
    if not trajectory_path.is_file(): raise FileNotFoundError(f"缺少轨迹：{trajectory_path}")
    with trajectory_path.open("r",encoding="utf-8-sig",newline="") as f: rows=list(csv.DictReader(f))
    if len(rows)<2: raise RuntimeError("动画轨迹为空")
    indices=np.unique(np.linspace(0,len(rows)-1,max(2,a.views)).round().astype(int)).tolist()
    context=omni.usd.get_context(); context.open_stage(str(stage_path)); wait_stage(context)
    viewport=get_active_viewport();
    if viewport is None: raise RuntimeError("没有active viewport")
    viewport.set_texture_resolution((960,540)); timeline=omni.timeline.get_timeline_interface(); timeline.stop()
    frame_paths=[]
    for order,index in enumerate(indices):
        r=rows[index]; x,y,z,yaw=(float(r["x"]),float(r["y"]),float(r["body_z"]),float(r["yaw_rad"]))
        forward=np.asarray([math.cos(yaw),math.sin(yaw),0.0]); eye=np.asarray([x,y,z])-4.0*forward+np.asarray([0,0,2.4]); target=np.asarray([x,y,z])+2.0*forward+np.asarray([0,0,.35])
        timeline.set_current_time(float(r["time_s"])); set_camera_view(eye=eye.tolist(),target=target.tolist(),camera_prim_path="/OmniverseKit_Persp",viewport_api=viewport)
        for _ in range(5): app.update()
        fp=out/f"preview_{order:02d}_frame_{index:04d}.png"; task=asyncio.ensure_future(capture(viewport,fp)); wait_task(task); frame_paths.append(fp); stamp(f"诊断帧 {order+1}/{len(indices)}")
    thumbs=[]
    for order,fp in enumerate(frame_paths):
        im=Image.open(fp).convert("RGB").resize((480,270)); ImageDraw.Draw(im).rectangle((0,0,170,28),fill=(0,0,0)); ImageDraw.Draw(im).text((8,7),f"route {order+1}/{len(frame_paths)}",fill=(255,255,255)); thumbs.append(im)
    cols=3; rows_n=math.ceil(len(thumbs)/cols); sheet=Image.new("RGB",(cols*480,rows_n*270),(12,15,20))
    for i,im in enumerate(thumbs): sheet.paste(im,((i%cols)*480,(i//cols)*270))
    sheet_path=out/"control_route_preview_sheet.png"; sheet.save(sheet_path)
    summary={"status":"PASS","stage":str(stage_path),"trajectory":str(trajectory_path),"sample_indices":indices,"frames":[str(x) for x in frame_paths],"sheet":str(sheet_path),"purpose":"height and route diagnostic only"}
    (out/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8"); print(json.dumps(summary,ensure_ascii=False,indent=2)); print("CONTROL_ROUTE_PREVIEW_PASS"); exit_code=0
except BaseException as e:
    traceback.print_exc(); (out/"failure.json").write_text(json.dumps({"status":"FAILED","exception":str(e),"traceback":traceback.format_exc()},ensure_ascii=False,indent=2),encoding="utf-8")
finally:
    try: app.close()
    except BaseException: pass
sys.exit(exit_code)
