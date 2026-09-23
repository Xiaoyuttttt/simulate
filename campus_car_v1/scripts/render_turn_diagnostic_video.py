#!/usr/bin/env python3
import csv
import argparse
from pathlib import Path
import math, subprocess, tempfile
from PIL import Image, ImageDraw

parser = argparse.ArgumentParser()
parser.add_argument('--source', default=None)
parser.add_argument('--output', default=None)
parser.add_argument('--sky', action='store_true')
args = parser.parse_args()
root = Path('/mnt/16T_2/txy/campus_robot/project/campus_car_v1')
source = Path(args.source) if args.source else root / 'outputs/turn_physics_demo/trajectory_frames.csv'
output = Path(args.output) if args.output else root / 'outputs/turn_physics_demo/turn_diagnostic.mp4'
with source.open(newline='', encoding='utf-8-sig') as f:
    rows = list(csv.DictReader(f))
x = [float(r['x']) for r in rows]; y = [float(r['y']) for r in rows]
yaw = [float(r['yaw_rad']) for r in rows]; t = [float(r['time_s']) for r in rows]
margin = .8; W,H = 1000,650
lo_x, hi_x, lo_y, hi_y = min(x)-margin, max(x)+margin, min(y)-margin, max(y)+margin
scale = min(860.0/(hi_x-lo_x), 500.0/(hi_y-lo_y))
center_x, center_y = (lo_x+hi_x)/2.0, (lo_y+hi_y)/2.0
def xy(px,py):
    return (int(W/2 + (px-center_x)*scale), int(H/2 - (py-center_y)*scale + 25))
output.parent.mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory() as td:
    for i in range(len(rows)):
        if args.sky:
            im=Image.new('RGB',(W,H)); pix=im.load()
            for py in range(H):
                if py < 390:
                    u=py/390.0; col=(int(55*(1-u)+185*u), int(145*(1-u)+220*u), int(235*(1-u)+245*u))
                else:
                    u=(py-390)/(H-390); col=(int(92*(1-u)+45*u), int(145*(1-u)+75*u), int(78*(1-u)+38*u))
                for px in range(W): pix[px,py]=col
            d=ImageDraw.Draw(im)
            d.ellipse((790,55,865,130), fill=(255,235,150))
            d.line((0,390,W,390), fill=(220,235,220), width=3)
        else:
            im=Image.new('RGB',(W,H),'white'); d=ImageDraw.Draw(im)
        d.text((30,20),f'Four-wheel differential steering | t={t[i]:.1f}s | yaw={yaw[i]:+.2f} rad',fill='black')
        d.line([xy(px,py) for px,py in zip(x,y)],fill='#c0c8d0',width=2)
        d.line([xy(px,py) for px,py in zip(x[:i+1],y[:i+1])],fill='#1677ff',width=5)
        c,s=math.cos(yaw[i]),math.sin(yaw[i]); L,Wc=.9,.55
        pts=[xy(x[i]+px*c-py*s,y[i]+px*s+py*c) for px,py in [(L/2,Wc/2),(L/2,-Wc/2),(-L/2,-Wc/2),(-L/2,Wc/2)]]
        d.polygon(pts,fill='#1769aa',outline='#083b66')
        phase='forward' if t[i]<1 else ('left turn' if t[i]<3 else ('stop' if t[i]<4 else 'right turn'))
        d.text((30,50),phase,fill='#c0392b' if 'turn' in phase else 'black')
        im.save(Path(td)/f'frame_{i:04d}.png')
    subprocess.run(['ffmpeg','-y','-framerate','10','-i',f'{td}/frame_%04d.png','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
print(output)
