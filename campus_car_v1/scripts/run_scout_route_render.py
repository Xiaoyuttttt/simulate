#!/usr/bin/env python3
"""Render the SCOUT visual model along the grounded route from two cameras."""
import argparse, asyncio, csv, json, math, time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(); p.add_argument('--config',default=str(ROOT/'config/placement.json')); p.add_argument('--route',default=str(ROOT/'outputs/scout_v2_route_ground/grounded_control_route.csv')); p.add_argument('--out',default=str(ROOT/'outputs/scout_v2_render')); p.add_argument('--fps',type=int,default=10); p.add_argument('--width',type=int,default=960); p.add_argument('--height',type=int,default=540); p.add_argument('--no-campus',action='store_true')
a=p.parse_args(); out=Path(a.out); out.mkdir(parents=True,exist_ok=True); cfg=json.loads(Path(a.config).read_text())
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':cfg['render']['renderer'],'width':a.width,'height':a.height,'multi_gpu':False,'max_gpu_count':1,'extra_args':['--/renderer/multiGpu/enabled=false','--/renderer/multiGpu/autoEnable=false']})
try:
 import omni.usd
 from isaacsim.sensors.camera import Camera
 from PIL import Image
 from pxr import Gf, UsdGeom, UsdLux
 from isaacsim.core.api import World
 with open(a.route,encoding='utf-8-sig',newline='') as f: rr=list(csv.DictReader(f))
 x=np.array([float(r['x']) for r in rr]); y=np.array([float(r['y']) for r in rr]); z=np.array([float(r['ground_z']) for r in rr]); s=np.array([float(r['distance_m']) for r in rr]);
 ctx=omni.usd.get_context(); ctx.new_stage(); app.update(); world=World(stage_units_in_meters=1.0,physics_dt=1/60,rendering_dt=1/a.fps); stage=ctx.get_stage(); UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage,1.0); UsdGeom.Xform.Define(stage,'/World')
 if not a.no_campus:
  campus=UsdGeom.Xform.Define(stage,'/World/Campus'); campus.GetPrim().GetReferences().AddReference(str(Path(cfg['campus_usd'])))
 else:
  from isaacsim.core.api.objects import GroundPlane
  world.scene.add(GroundPlane(prim_path='/World/Ground',name='ground',size=2000.0,z_position=float(z.min())-.08,visible=True))
 vis=UsdGeom.Xform.Define(stage,'/World/ScoutVisual'); vis.GetPrim().GetReferences().AddReference(str(ROOT/'assets/scout_v2_isaac/Isaac_models/scout_v2_base.usd')); vx=UsdGeom.Xformable(vis.GetPrim()); vt=vx.AddTranslateOp(); vo=vx.AddOrientOp(); vs=vx.AddScaleOp(); vs.Set(Gf.Vec3f(.01))
 fp=UsdGeom.Camera.Define(stage,'/World/FirstPersonCamera'); fp.CreateFocalLengthAttr(18.0); fp.CreateClippingRangeAttr(Gf.Vec2f(.05,1500)); fxf=UsdGeom.Xformable(fp.GetPrim()); fpt=fxf.AddTranslateOp(); fpo=fxf.AddOrientOp()
 tp=UsdGeom.Camera.Define(stage,'/World/ThirdPersonCamera'); tp.CreateFocalLengthAttr(28.0); tp.CreateClippingRangeAttr(Gf.Vec2f(.1,2500)); txf=UsdGeom.Xformable(tp.GetPrim()); tpt=txf.AddTranslateOp(); tpo=txf.AddOrientOp()
 dome=UsdLux.DomeLight.Define(stage,'/World/Lights/Sky'); dome.CreateIntensityAttr(800.0); dome.CreateColorAttr(Gf.Vec3f(.42,.58,.82)); sun=UsdLux.DistantLight.Define(stage,'/World/Lights/Sun'); sun.CreateIntensityAttr(2200.0); sun.CreateColorAttr(Gf.Vec3f(1,.92,.78)); UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45,20,10))
 fp_sensor=Camera(prim_path=str(fp.GetPath()),name='scout_first_sensor',resolution=(a.width,a.height))
 tp_sensor=Camera(prim_path=str(tp.GetPath()),name='scout_third_sensor',resolution=(a.width,a.height))
 fp_sensor.initialize(); tp_sensor.initialize()
 fp_dir=out/'first_person_frames'; tp_dir=out/'third_person_frames'; fp_dir.mkdir(exist_ok=True); tp_dir.mkdir(exist_ok=True)
 n=int(math.ceil(s[-1]/max(np.median([float(r['speed_mps']) for r in rr]),.1)*a.fps)); rows=[]
 for i in range(n):
  dist=min(s[-1],i/a.fps*.8); j=int(np.searchsorted(s,dist)); j=min(j,len(s)-1); yaw=math.atan2(y[min(j+1,len(y)-1)]-y[max(0,j-1)],x[min(j+1,len(x)-1)]-x[max(0,j-1)]); pos=Gf.Vec3d(x[j],y[j],z[j]+.235); vt.Set(pos); vo.Set(Gf.Quatf(math.cos(yaw/2),Gf.Vec3f(0,0,math.sin(yaw/2))))
  eye=Gf.Vec3d(pos[0]+.48*math.cos(yaw),pos[1]+.48*math.sin(yaw),pos[2]+.48); fpt.Set(eye); q=Gf.Rotation(Gf.Vec3d(0,0,-1),Gf.Vec3d(math.cos(yaw),math.sin(yaw),0)).GetQuat(); fpo.Set(Gf.Quatf(float(q.GetReal()),Gf.Vec3f(q.GetImaginary())))
  cam=Gf.Vec3d(pos[0]-5*math.cos(yaw),pos[1]-5*math.sin(yaw),pos[2]+2.5); target=Gf.Vec3d(pos[0],pos[1],pos[2]+.25); q=Gf.Rotation(Gf.Vec3d(0,0,-1),target-cam).GetQuat(); tpt.Set(cam); tpo.Set(Gf.Quatf(float(q.GetReal()),Gf.Vec3f(q.GetImaginary())))
  rows.append({'frame':i,'time_s':i/a.fps,'distance_m':dist,'x':x[j],'y':y[j],'ground_z':z[j],'yaw_rad':yaw}); app.update()
  for sensor, folder in ((fp_sensor,fp_dir),(tp_sensor,tp_dir)):
   for _ in range(2): app.update()
   rgba=np.asarray(sensor.get_rgba())
   Image.fromarray(rgba[:,:,:3].astype(np.uint8)).save(folder/f'frame_{i:05d}.png')
 with open(out/'trajectory_render.csv','w',newline='',encoding='utf-8') as f: w=csv.DictWriter(f,fieldnames=rows[0]); w.writeheader(); w.writerows(rows)
 (out/'summary.json').write_text(json.dumps({'status':'PASS','mode':'route_visual_render_validation','route_distance_m':float(s[-1]),'frames':n,'fps':a.fps,'rendered':True},indent=2),encoding='utf-8'); print('SCOUT_ROUTE_RENDER_PASS')
finally: app.close()
