#!/usr/bin/env python3
"""Source-timed camera, semantic labels, accepted paths and real costmap replay.

This is a postprocessing overlay, not a camera-based buoy detector. Raw camera
files and all original timestamps remain unchanged. No data enters navigation.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation
from plot_task1 import TaskEvidence, figures

FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
FONTS={n:ImageFont.truetype(FONT,n) for n in (15,17,19,21,23,26,30)}
BG='#0b1424';PANEL='#152238';INK='#e5edf7';MUTED='#a9bad0'
TRACK='#22d3ee';PLAN='#e879f9';GOAL='#facc15';TRAFFIC='#f59e0b'
COLORS={0:'#cbd5e1',1:'#ff646c',2:'#46e89b',3:'#60a5fa',4:'#60a5fa'}
LABELS={0:'灭灯',1:'红灯',2:'绿灯',3:'蓝闪·入口',4:'蓝常亮·出口'}
STAGES={'APPROACH_ENTRY':'驶向入口','CIRCLE_ENTRY':'入口顺时针绕行','TRANSIT':'安全通行',
        'CIRCLE_EXIT':'出口逆时针绕行','COMPLETE':'任务完成','ABORTED':'任务终止'}
MOTIONS={'EXECUTING':'执行路径','PLANNING':'正在重规划','STARTING':'等待控制器',
         'CHECKING_TRAFFIC':'检查候选路径的交通冲突',
         'WAIT_ROUTE':'等待通道，定时重新比较',
         'STOPPING':'停车中','WAIT_CLEAR':'停等可行路径','COMPLETE':'完成后观察','ABORTED':'已终止'}


def prior_index(stamps,t):
    """Never use a future sample to fill a gap."""
    return bisect.bisect_right(stamps,t)-1


def camera_project(xyz,config):
    points=np.asarray(xyz,dtype=float).reshape(-1,3)
    pose=config['pose'];rotation=Rotation.from_euler('xyz',pose[3:]).as_matrix()
    local=(points-np.asarray(pose[:3]))@rotation
    if np.any(local[:,0]<=0):raise ValueError('Overlay point behind the camera')
    f=config['width']/(2*math.tan(config['horizontal_fov']/2))
    return np.column_stack((config['width']/2-f*local[:,1]/local[:,0],
                            config['height']/2-f*local[:,2]/local[:,0]))


def text(draw,xy,value,size=21,fill=INK):
    draw.text(xy,str(value),font=FONTS[size],fill=fill)


def dashed(draw,points,fill,width=3,dash=10,gap=7):
    p=np.asarray(points)
    if len(p)<2:return
    progress=0.
    for a,b in zip(p[:-1],p[1:]):
        length=float(np.linalg.norm(b-a))
        if length<1e-6:continue
        cursor=0.
        while cursor<length:
            phase=progress%(dash+gap)
            step=min(length-cursor,(dash if phase<dash else dash+gap)-phase)
            if step<1e-6:step=min(length-cursor,1e-4)
            if phase<dash:
                u=a+(b-a)*cursor/length;v=a+(b-a)*(cursor+step)/length
                draw.line([tuple(u),tuple(v)],fill=fill,width=width)
            progress+=step;cursor+=step


def star(draw,xy,r=10):
    x,y=xy
    pts=[(x+(r if i%2==0 else r*.43)*math.sin(i*math.pi/5),
          y-(r if i%2==0 else r*.43)*math.cos(i*math.pi/5)) for i in range(10)]
    draw.polygon(pts,fill=GOAL,outline='#111827')


class Review:
    def __init__(self,data):
        self.data=data
        config=data.scene.get('overview_camera')
        if not config:raise ValueError('Missing calibrated camera metadata; do not guess projection')
        self.config=config
        video=json.loads((data.run/'task1_video.json').read_text())
        self.frames=video['source_sim_stamps']['task1_overview']
        if not self.frames or any(b<=a for a,b in zip(self.frames,self.frames[1:])):
            raise ValueError('Camera timestamps must be strictly increasing')
        self.capture=cv2.VideoCapture(str(data.run/'task1_overview.mp4'))
        if int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))!=len(self.frames):
            raise ValueError('Camera timestamp count differs from MP4 frames')
        self.frame_index=-1;self.raw_image=None
        self.times=[r['sim'] for r in data.rows]
        self.events={}
        for e in data.events:self.events.setdefault(e['topic'],[]).append(e)
        for topic in self.events:self.events[topic].sort(key=lambda e:e['sim'])
        self.event_times={k:[e['sim'] for e in v] for k,v in self.events.items()}
        self.mission_times=[e['sim'] for e in data.mission]
        self.path_cache={};self.grid_images={}
        self.camera_size=(1264,790);self.local_size=596
        traffic_file=data.run/'traffic_guard.jsonl'
        self.traffic=[json.loads(l) for l in traffic_file.read_text().splitlines()] if traffic_file.exists() else []
        self.traffic_times=[r['evaluated_at'] for r in self.traffic]
        route_file=data.run/'route_decisions.jsonl'
        self.routes=[json.loads(l) for l in route_file.read_text().splitlines()] if route_file.exists() else []
        self.route_times=[r['evaluated_at'] for r in self.routes]

    def event(self,topic,t):
        i=prior_index(self.event_times.get(topic,[]),t)
        return self.events[topic][i] if i>=0 else None

    def path(self,event):
        key=(event['topic'],event['wall'])
        if key not in self.path_cache:self.path_cache[key]=self.data.path(event)
        return self.path_cache[key]

    def read_camera(self,t):
        wanted=prior_index(self.frames,t)
        if wanted<0:raise ValueError('No camera frame at replay start')
        if wanted<self.frame_index:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES,wanted);self.frame_index=wanted-1
        while self.frame_index<wanted:
            success,frame=self.capture.read()
            if not success:raise ValueError('Incomplete source camera video')
            self.frame_index+=1;self.raw_image=frame
        return Image.fromarray(cv2.cvtColor(self.raw_image,cv2.COLOR_BGR2RGB)).resize(self.camera_size,Image.LANCZOS),self.frames[wanted]

    def plan(self,t,status):
        event=self.event(self.data.plan_topic,t)
        if event is None:return None,False
        # An accepted path is active only for its matching mission generation.
        recent=self.data.mission[:prior_index(self.mission_times,t)+1]
        ready=next((e for e in reversed(recent) if e['event']=='path_ready'),None)
        invalid=any(e['event'] in ('stop_requested','aborted','planning') and e['sim']>ready['sim']
                    for e in recent) if ready else True
        active=bool(ready and not invalid and status['motion']=='EXECUTING' and ready['revision']==status['revision']
                    and abs(event['sim']-ready['sim'])<1.)
        return event,active

    def hull(self,pose):
        x,y,heading=pose;c,s=math.cos(heading),math.sin(heading)
        return np.asarray(self.data.scene['footprint'])@np.array([[c,s],[-s,c]])+[x,y]

    def entities(self,picture,t,project,labels=True):
        draw=ImageDraw.Draw(picture);sem=self.data.semantic(t)
        for b in sem['buoys']:
            x,y=project([[b['x'],b['y']]])[0];state=b['state'];color=COLORS[state]
            if not (-20<x<picture.width+20 and -20<y<picture.height+20):continue
            draw.ellipse((x-7,y-7,x+7,y+7),fill=color if b['lit'] else BG,outline=color,width=3)
            if labels:
                name=b['id'].replace('buoy_','B').replace('entry','入口').replace('exit','出口')
                label=name+' '+({3:'蓝闪',4:'蓝常亮'}.get(state,LABELS[state]))
                if state in (1,2,3):label+='·'+('亮' if b['lit'] else '暗')
                width=draw.textlength(label,font=FONTS[17])+14
                lx=min(max(2,x-width/2),picture.width-width-2);ly=min(max(2,y+11),picture.height-29)
                draw.rounded_rectangle((lx,ly,lx+width,ly+27),radius=5,fill=BG,outline=color,width=1)
                text(draw,(lx+7,ly),label,17,color)
        if 'dynamic_pose' in sem:
            x,y=sem['dynamic_pose'];corners=project([[x-1,y-1.5],[x+1,y-1.5],[x+1,y+1.5],[x-1,y+1.5]])
            draw.line([*map(tuple,corners),tuple(corners[0])],fill=TRAFFIC,width=3)
            cx,cy=project([[x,y]])[0]
            if 0<=cx<picture.width and 0<=cy<picture.height:text(draw,(cx+14,cy-14),'交通船',17,TRAFFIC)

    def tracks(self,picture,t,project,local=False):
        draw=ImageDraw.Draw(picture);i=prior_index(self.times,t)
        if i<0:return
        row=self.data.rows[i];status=self.data.status(t);event,active=self.plan(t,status)
        if event:
            points=project(self.path(event)[:,:2])
            dashed(draw,points,PLAN if active else '#8190a6',width=3,dash=10,gap=7)
        truth=project(self.data.truth[:i+1,:2])
        if len(truth)>1:draw.line(list(map(tuple,truth)),fill=TRACK,width=3)
        if local:
            prediction=self.event('/local_plan',t)
            if active and prediction and t-prediction['stamp']<1:
                dashed(draw,project(self.path(prediction)[:,:2]),'#f1f5f9',width=2,dash=3,gap=4)
        pose=[row['truth'][0],row['truth'][1],row['truth'][3]]
        hull=project(self.hull(pose));draw.line([*map(tuple,hull),tuple(hull[0])],fill='#ffffff',width=2)
        x,y,h=pose;arrow=project([[x,y],[x+2*math.cos(h),y+2*math.sin(h)]])
        draw.line(list(map(tuple,arrow)),fill=TRACK,width=3)
        target=status.get('target')
        if target:
            q=project([target[:2]])[0]
            if 0<=q[0]<picture.width and 0<=q[1]<picture.height:star(draw,q)

    def local_map(self,t):
        i=prior_index(self.times,t);row=self.data.rows[i];x,y=row['truth'][:2]
        size=self.local_size;span=30.;bounds=(x-span/2,y-span/2,x+span/2,y+span/2)
        project=lambda xy:np.column_stack(((np.asarray(xy)[:,0]-bounds[0])*size/span,
                                          (bounds[3]-np.asarray(xy)[:,1])*size/span))
        picture=Image.new('RGB',(size,size),'#dce9ef');latest=None
        for topic,cost in [('/map',False),('/local_costmap/costmap',True)]:
            event=self.event(topic,t)
            if not event:continue
            if abs(event['origin'][2])>1e-6:raise ValueError('Rotated grid origin')
            key=event['file']
            if key not in self.grid_images:
                values=self.data.grid(event).astype(int);rgba=np.zeros((*values.shape,4),dtype=np.uint8)
                if cost:
                    rgba[:,:,:3]=[76,94,122];rgba[:,:,3]=np.where(values>0,80+np.clip(values,0,100)*1.5,0).astype(np.uint8)
                    rgba[values>=99]=[20,27,38,255]
                else:
                    rgba[:]=[222,235,241,255];rgba[values<0]=[170,181,192,255];rgba[values>=65]=[55,65,81,255]
                self.grid_images[key]=Image.fromarray(np.flipud(rgba),'RGBA')
                # Only current map images are needed; retain at most four.
                if len(self.grid_images)>4:del self.grid_images[next(iter(self.grid_images))]
            source=self.grid_images[key];ox,oy=np.asarray(event['origin'][:2])+self.data.offset_for(event['frame']);res=event['resolution']
            # Output coordinates -> source grid pixels; avoids repeatedly resizing full global maps.
            affine=(span/size/res,0,(bounds[0]-ox)/res,0,span/size/res,
                    (oy+event['height']*res-bounds[3])/res)
            layer=source.transform((size,size),Image.AFFINE,affine,Image.NEAREST)
            picture.paste(layer,(0,0),layer)
            if cost:latest=event
        self.tracks(picture,t,project,local=True);self.entities(picture,t,project,labels=False)
        draw=ImageDraw.Draw(picture);draw.rectangle((0,0,size,35),fill=BG)
        text(draw,(12,3),'实际发布的局部代价地图 · 30 m × 30 m',19)
        draw.rectangle((0,size-33,size,size),fill=BG)
        age=t-latest['stamp'] if latest else math.nan
        text(draw,(12,size-31),f'源时间差 {age:.2f} s  |  黑：占据（含交通预测）',17)
        return picture

    def render(self,t):
        image=Image.new('RGB',(1920,1080),BG);draw=ImageDraw.Draw(image)
        text(draw,(22,12),'RobotX 2026 · Task 1  港区场景实测',30)
        course=self.data.scene['task1'];wind=self.data.scene['wind_mps'];wave=self.data.scene['wave_gain']
        traffic={'none':'无','cross':'横穿','block':'暂停后横穿'}[course['dynamic']]
        text(draw,(1020,21),f"单次灯变  |  交通：{traffic}  |  风 {wind:g} m/s  浪 gain {wave:g}",21)
        text(draw,(22,57),'Nav2 → MAVROS → ArduRover 4.4.0 → VRX WAM-V   /   视频为仿真记录与数据标注',19,MUTED)
        camera,source_t=self.read_camera(t)
        def project(xy):
            xyz=np.column_stack((np.asarray(xy),np.full(len(xy),.5)))
            return camera_project(xyz,self.config)*np.asarray(self.camera_size)/[self.config['width'],self.config['height']]
        self.tracks(camera,source_t,project);self.entities(camera,source_t,project)
        cd=ImageDraw.Draw(camera);cd.rectangle((0,0,camera.width,37),fill=BG)
        text(cd,(12,3),f'VRX 实际相机 + 标注   源时间 {source_t:.2f} s   图像时间差 {t-source_t:.2f} s',19)
        image.paste(camera,(20,100));image.paste(self.local_map(t),(1304,100))
        draw=ImageDraw.Draw(image);draw.rounded_rectangle((1304,714,1900,1060),radius=10,fill=PANEL)
        status=self.data.status(t);sem=self.data.semantic(t);i=prior_index(self.times,t);row=self.data.rows[i]
        text(draw,(1322,727),f"任务时间 {t-self.data.t0:6.1f} s    回放 2.5×",23)
        motion=MOTIONS.get(status['motion'],status['motion'])
        if status.get('route_purpose')=='observation':motion+='（前移观察）'
        text(draw,(1322,766),STAGES.get(status['stage'],status['stage'])+' / '+motion,23)
        text(draw,(1322,805),f"灯态版本 {sem['revision']}   已变灯 {sem.get('changes_applied',0)} / 1 次",21)
        _,active=self.plan(t,status)
        text(draw,(1322,841),'紫虚线：当前执行路径' if active else '灰虚线：上一条路径，当前未执行',21,PLAN if active else MUTED)
        speed=math.hypot(row['truth'][4],row['truth'][5])
        text(draw,(1322,878),f"实测船速 {speed:.3f} m/s   净空 {row.get('clearance_m',math.nan):.2f} m",21)
        text(draw,(1322,915),f"推进器 {row.get('left_n',math.nan):+.1f} / {row.get('right_n',math.nan):+.1f} N",21)
        updates=[e for e in self.data.mission if e['event']=='semantic_update_reported' and e['sim']<=t]
        if updates:
            change=course['changes'][0];bid=change['id'].replace('buoy_','B')
            text(draw,(1322,952),f"灯变：{bid} → {LABELS[change['state']]}  已上报",21,COLORS[change['state']])
        else:text(draw,(1322,952),'灯态：尚未触发本场唯一一次变化',21,MUTED)
        if status['stage'] in ('COMPLETE','ABORTED'):
            label='独立裁判：通过' if self.data.result['passed'] else '独立裁判：未通过（完整保留）'
            text(draw,(1322,998),label,21,GOAL)
        elif self.traffic:
            index=prior_index(self.traffic_times,t)
            assessment=self.traffic[index] if index>=0 else None
            if assessment and t-assessment['source_stamp']<=.4:
                label='LiDAR 20 s 近场：未见冲突' if assessment['clear'] else 'LiDAR 20 s 近场：停车让行'
                text(draw,(1322,998),label,21,MUTED if assessment['clear'] else TRAFFIC)
            else:text(draw,(1322,998),'LiDAR 交通预测：等待新观测',21,TRAFFIC)
        draw.rounded_rectangle((20,910,1284,1060),radius=10,fill=PANEL)
        dashed(draw,[[40,937],[110,937]],PLAN,width=4);text(draw,(122,921),'规划路径',21)
        draw.line((290,937,360,937),fill=TRACK,width=4);text(draw,(372,921),'实际航迹（VRX 真值）',21)
        star(draw,(688,937),12);text(draw,(712,921),'当前阶段末端目标',21)
        draw.line((992,937,1052,937),fill='#ffffff',width=2);text(draw,(1064,921),'船体轮廓',21)
        text(draw,(40,965),'红灯留在船右舷   绿灯留在船左舷   灭灯浮标仍是实体障碍',21)
        route_index=prior_index(self.route_times,t)
        route=self.routes[route_index] if route_index>=0 else None
        if (status['stage']=='TRANSIT' and route and 0<=t-route['source_stamp']<=1.
                and route['revision']==status['revision']):
            reference=f"{route['reference_eta_s']:.0f} s" if route['reference_eta_s'] is not None else '无有效参考'
            wait=f"{route['wait_s']:.0f} s" if route['wait_s'] is not None else '60 s 内无清空预测'
            if route.get('observation_budget_s',0)>0:
                reference+='（释放未知）';wait=f"观测总预算 {route['observation_budget_s']:.0f} s"
            text(draw,(40,1004),f"候选行驶 ETA {route['eta_s']:.0f} s   原路 {reference}   候选等待 {wait}   切换代价 {route['switch_cost_s']:.0f} s",19,MUTED)
        else:
            text(draw,(40,1004),'灯标文字保持语义颜色；“亮／暗”表示闪烁相位。灯态来自已知观测，非视觉识别。',19,MUTED)
        return image,source_t


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path)
    p.add_argument('--preview',type=float,help='One preview at seconds after task start; no video output')
    a=p.parse_args();data=TaskEvidence(a.run);review=Review(data)
    if a.preview is not None:
        t=min(data.rows[-1]['sim'],data.t0+a.preview);image,_=review.render(t)
        output=data.output/'task1_review_preview.png';image.save(output);print(output);return
    figures(data)
    start=max(data.t0,review.frames[0]);end=min(data.rows[-1]['sim'],review.frames[-1])
    times=np.arange(start,end+.001,.5);path=data.output/'task1_annotated.mp4';partial=path.with_suffix('.partial.mp4')
    cmd=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24',
         '-s','1920x1080','-r','5','-i','-','-an','-c:v','libx264','-preset','fast','-threads','2',
         '-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(partial)]
    process=subprocess.Popen(cmd,stdin=subprocess.PIPE);sources=[]
    try:
        for i,t in enumerate(times):
            image,source_t=review.render(float(t));process.stdin.write(image.tobytes());sources.append(source_t)
            if i in (0,len(times)-1):image.save(data.output/('task1_review_start.png' if i==0 else 'task1_review_end.png'))
            if i%100==0:print(f'Annotated frames {i+1}/{len(times)}',flush=True)
    finally:
        process.stdin.close();code=process.wait(timeout=60);review.capture.release()
    if code:raise RuntimeError('Video encoder failed')
    partial.replace(path)
    manifest={'run':str(data.run),'output':path.name,'frames':len(times),'fps':5,'simulation_speed':2.5,
              'source_sim_range':[float(times[0]),float(times[-1])],'camera_source_stamps':sources,
              'max_camera_age_s':float(np.max(times-np.asarray(sources))),
              'plan_topic':data.plan_topic,'scope':'Recorded camera with known-state labels, accepted Nav2 paths and independent VRX truth. Not a vision detector.',
              'source_sha256':{name:hashlib.sha256((data.run/name).read_bytes()).hexdigest() for name in
                  ['task1_overview.mp4','task1_video.json','task1.jsonl','task1_judge.jsonl','task1_mission.jsonl','visualization/raw/events.jsonl','generated/scene.json']},
              'output_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    if review.routes:
        manifest['source_sha256']['route_decisions.jsonl']=hashlib.sha256((data.run/'route_decisions.jsonl').read_bytes()).hexdigest()
    (data.output/'task1_annotated.json').write_text(json.dumps(manifest,indent=2)+'\n');print(path,flush=True)


if __name__=='__main__':main()
