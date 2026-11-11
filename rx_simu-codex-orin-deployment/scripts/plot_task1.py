#!/usr/bin/env python3
"""Source-timed Task1 maps, route/endpoint figures and semantic-event replay."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
from plot_navigation import Evidence, BLUE, ORANGE, PURPLE, GREEN, setup, draw_path, read_lines

COLORS={0:'#252a34',1:'#dc2626',2:'#059669',3:'#2563eb',4:'#2563eb'}
TRACK, PLAN = '#0891b2', '#c026d3'
STATE_LABELS={0:'灭灯',1:'红灯',2:'绿灯',3:'蓝闪·入口',4:'蓝常亮·出口'}


class TaskEvidence(Evidence):
    def __init__(self,run):
        self.run=run.resolve();self.output=self.run/'visualization';self.output.mkdir(exist_ok=True)
        self.raw=self.output/'raw';self.events=read_lines(self.raw/'events.jsonl')
        self.trace=read_lines(self.run/'task1.jsonl');self.mission=read_lines(self.run/'task1_mission.jsonl')
        self.result=json.loads((self.run/'task1_result.json').read_text())
        self.scene=json.loads((self.run/'generated/scene.json').read_text())
        self.area=json.loads((self.run/'generated/water_area.json').read_text())
        self.offset=np.asarray(json.loads((self.run/'map_alignment.json').read_text())['offset_xy_m'])
        self.judge=read_lines(self.run/'task1_judge.jsonl')
        self.scenes=sorted([r for r in self.judge if r['type']=='scene'],key=lambda r:r['stamp'])
        self.statuses=sorted([r for r in self.judge if r['type']=='status'],key=lambda r:r['stamp'])
        self.scene_times=[r['stamp'] for r in self.scenes];self.status_times=[r['stamp'] for r in self.statuses]
        start=next((e['sim'] for e in self.mission if e['event']=='started'),0.)
        self.rows=[r for r in self.trace if 'truth' in r and r['sim']>=start and r['phase']!='cleanup']
        if not self.rows:raise ValueError('No truth trace')
        self.active=[r for r in self.rows if not r['phase'].startswith('COMPLETE')]
        self.truth=np.asarray([r['truth'] for r in self.rows]);self.ekf=np.asarray([r.get('ekf',[np.nan]*7) for r in self.rows])
        self.t0=self.rows[0]['sim'];self.grid_cache={}
        self.plan_topic='/task1/reference_path' if any(e['topic']=='/task1/reference_path' for e in self.events) else '/plan'
        self.plans=[e for e in self.events if e['topic']==self.plan_topic and e.get('poses')]
        if not self.plans:raise ValueError('No recorded reference paths')
        self.target=np.asarray((self.result.get('completion') or self.result.get('mission_status') or {})['target'])
        self.focus=min(self.rows,key=lambda r:r.get('clearance_m',math.inf))
        self.dynamic_focus=None
        if self.scene['task1']['dynamic']!='none':
            from usv_sim.geometry import vessel_clearance
            def moving_distance(row):
                xy=self.semantic(row['sim'])['dynamic_pose'];p=row['truth']
                return vessel_clearance(self.scene['footprint'],p[0],p[1],p[3],[dict(x=xy[0],y=xy[1],size=[2.,3.])])
            self.dynamic_focus=min(self.rows,key=moving_distance);self.focus=self.dynamic_focus
        self.bounds=[-6,73,-23,24] if self.scene['task1'].get('course')=='harbor' else [-4,69,-17,17]

    def semantic(self,t):
        i=bisect.bisect_right(self.scene_times,t)-1
        return self.scenes[max(0,i)]['data']

    def status(self,t):
        i=bisect.bisect_right(self.status_times,t)-1
        return self.statuses[max(0,i)]['data']

    def markers(self,ax,t,labels=True):
        data=self.semantic(t)
        for b in data['buoys']:
            ax.add_patch(Rectangle((b['x']-.254,b['y']-.254),.508,.508,facecolor='#4b5563',zorder=7))
            color=COLORS[b['state']]
            ax.scatter(b['x'],b['y'],s=55,marker='D',edgecolor=color,linewidth=1.7,
                       facecolor=color if b['lit'] else 'white',zorder=9)
            if labels:ax.text(b['x'],b['y']+.9,b['id'].replace('buoy_','B')+' '+STATE_LABELS[b['state']],ha='center',fontsize=8,zorder=10,clip_on=True)
        if 'dynamic_pose' in data:
            x,y=data['dynamic_pose'];ax.add_patch(Rectangle((x-1,y-1.5),2,3,facecolor='#f59e0b',edgecolor='#92400e',alpha=.8,zorder=8))


def figures(data):
    fig,axes=plt.subplots(2,1,figsize=(14,9),gridspec_kw={'height_ratios':[3,1]},constrained_layout=True)
    ax=axes[0];data.map(ax,data.focus['wall'])
    for i,event in enumerate(data.plans):draw_path(ax,data,event,PLAN,'已接受规划路径（分段）' if i==0 else None,style='--',width=1.4,alpha=.65)
    ax.plot(data.truth[:,0],data.truth[:,1],color=TRACK,lw=2,label='实际轨迹：VRX 真值',zorder=6)
    ax.plot(data.ekf[:,0],data.ekf[:,1],'--',color=ORANGE,lw=1,label='飞控 EKF',zorder=6)
    data.markers(ax,data.rows[-1]['sim']);data.target_marker(ax)
    setup(ax,'Task 1：参考路径、实际轨迹与期望末端（浮标颜色为结束时灯态）',data.bounds);ax.legend(loc='lower left',ncol=4,fontsize=9)
    ax=axes[1]
    phases=['APPROACH_ENTRY','CIRCLE_ENTRY','TRANSIT','CIRCLE_EXIT','COMPLETE','ABORTED']
    times=[r['stamp']-data.t0 for r in data.statuses if r['stamp']>=data.t0]
    vals=[phases.index(r['data']['stage']) if r['data']['stage'] in phases else -1 for r in data.statuses if r['stamp']>=data.t0]
    ax.step(times,vals,where='post',color=BLUE)
    for e in data.mission:
        if e['event']=='semantic_update_reported':ax.axvline(e['sim']-data.t0,color='#dc2626',ls='--',label='灯态变化')
        if e['event']=='stop_requested':ax.scatter(e['sim']-data.t0,phases.index(e['stage']) if e['stage'] in phases else -1,color=ORANGE,marker='x')
    ax.set(yticks=range(len(phases)),yticklabels=phases,xlabel='任务开始后的仿真时间 / s',title='阶段与停车／灯态事件');ax.grid(alpha=.2)
    fig.savefig(data.output/'task1_trajectory.png',dpi=150);plt.close(fig)
    if data.result.get('completion'):
        p=np.asarray(data.result['completion']['pose']);target=data.target
        fig,ax=plt.subplots(figsize=(6,6),constrained_layout=True);ax.set_facecolor('#f4f8fb')
        ax.add_patch(Circle(target[:2],.5,facecolor='#eee7fb',edgecolor=PURPLE,ls='--',label='0.5 m 验收半径'))
        completed=data.result['completion']['stamp']
        before=np.asarray([r['truth'] for r in data.rows if completed-20<=r['sim']<=completed])
        after=np.asarray([r['truth'] for r in data.rows if r['sim']>=completed])
        if len(before):ax.plot(before[:,0],before[:,1],color=TRACK,lw=2,label='到点前真值航迹')
        if len(after):ax.plot(after[:,0],after[:,1],'--',color='#64748b',lw=2,label='到点后漂移')
        data.target_marker(ax,arrow_length=.6)
        ax.scatter(p[0],p[1],color=BLUE,s=60,label='完成瞬间真值',zorder=10)
        ax.arrow(p[0],p[1],.6*math.cos(p[2]),.6*math.sin(p[2]),width=.02,color=BLUE)
        errors=data.result['endpoint_error']
        setup(ax,f"末端误差 {errors['position_m']:.3f} m / {errors['yaw_rad']:.3f} rad",
              [target[0]-1.5,target[0]+1.5,target[1]-1.5,target[1]+1.5]);ax.legend(fontsize=9)
        fig.savefig(data.output/'task1_endpoint.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(16,7),constrained_layout=True);row=data.focus
    for ax,topic,title in zip(axes,[None,'/global_costmap/costmap','/local_costmap/costmap'],['静态地图','全局代价地图','局部代价地图']):
        data.map(ax,row['wall'],topic);data.markers(ax,row['sim']);data.hull(ax,[*row['truth'][:2],row['truth'][3]])
        draw_path(ax,data,data.latest(data.plan_topic,row['wall']),PLAN,'最近已接受路径',style='--')
        if topic=='/local_costmap/costmap':draw_path(ax,data,data.latest('/local_plan',row['wall']),PURPLE,'DWB 预测')
        bounds=data.bounds if topic is None else [row['truth'][0]-16,row['truth'][0]+16,row['truth'][1]-16,row['truth'][1]+16]
        setup(ax,title,bounds)
    fig.suptitle(f"实际发布的地图 · t={row['sim']-data.t0:.1f}s；浮标实体始终保留，空心灯标表示闪烁暗相位")
    fig.savefig(data.output/'task1_maps.png',dpi=150);plt.close(fig)


def replay(data):
    sim=np.asarray([r['sim'] for r in data.rows]);times=np.arange(sim[0],sim[-1]+.1,.5)
    fig,axes=plt.subplots(1,2,figsize=(12.8,6.4),dpi=100);fig.subplots_adjust(left=.06,right=.97,bottom=.12,top=.86,wspace=.2)
    title=fig.suptitle('');fig.text(.5,.025,'青实线：真值　橙虚线：EKF　紫虚线：当前参考路径　紫星号：末端目标\n'
        '浮标始终存在；空心表示闪烁暗相位，深灰为 OFF。0.5 仿真秒采样，2.5 倍速。',ha='center',fontsize=10)
    output=data.output/'task1_replay.mp4'
    partial=data.output/'task1_replay.partial.mp4'
    process=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgba','-s','1280x640',
                               '-r','5','-i','-','-an','-c:v','libx264','-threads','2','-crf','22','-pix_fmt','yuv420p',
                               '-movflags','+faststart',str(partial)],stdin=subprocess.PIPE)
    try:
        for frame,t in enumerate(times):
            index=max(0,bisect.bisect_right(sim,t)-1);row=data.rows[index];status=data.status(t)
            for ax,topic in zip(axes,['/global_costmap/costmap','/local_costmap/costmap']):
                ax.clear();data.map(ax,row['wall'],topic);data.markers(ax,t,labels=ax is axes[0])
                ax.plot(data.truth[:index+1,0],data.truth[:index+1,1],color=TRACK,lw=2,zorder=6)
                ax.plot(data.ekf[:index+1,0],data.ekf[:index+1,1],'--',color=ORANGE,lw=1,zorder=6)
                if status['motion']=='EXECUTING':draw_path(ax,data,data.latest(data.plan_topic,row['wall']),PLAN,None,style='--')
                data.hull(ax,[*row['truth'][:2],row['truth'][3]]);data.target_marker(ax,label=False)
                bounds=data.bounds if ax is axes[0] else [row['truth'][0]-10,row['truth'][0]+10,row['truth'][1]-10,row['truth'][1]+10]
                setup(ax,'任务全景' if ax is axes[0] else '局部感知与控制',bounds)
            title.set_text(f"RobotX 2026 Task 1 · {t-data.t0:.1f}s · {status['stage']} / {status['motion']} · 灯态版本 {status['revision']}")
            fig.canvas.draw();process.stdin.write(fig.canvas.buffer_rgba())
            if frame%100==0:print(f'Replay frames: {frame+1}/{len(times)}',flush=True)
    finally:process.stdin.close();code=process.wait(timeout=30);plt.close(fig)
    if code:raise RuntimeError('Video encoding failed')
    partial.replace(output)
    excerpts(data)


def excerpts(data):
    output=data.output/'task1_replay.mp4'
    fallback=data.dynamic_focus['sim'] if data.dynamic_focus else data.t0+30.
    change=next((e['sim'] for e in data.mission if e['event']=='semantic_update_reported'),fallback)
    excerpts={'task1_excerpt.gif':change}
    if data.dynamic_focus:excerpts['task1_dynamic.gif']=data.dynamic_focus['sim']
    if not data.result['passed']:
        failure=next((e['sim'] for e in data.mission if e['event']=='stop_requested' and
                      'controller failed' in e.get('reason','')),data.rows[-1]['sim']-15.)
        excerpts['task1_failure.gif']=failure
    for name,at in excerpts.items():
        start=max(0.,(at-data.t0-10)/2.5)
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-ss',str(start),'-t','18','-i',str(output),
                        '-filter_complex','[0:v]scale=768:-1,split[a][b];[a]palettegen[p];[b][p]paletteuse',
                        '-loop','0',str(data.output/name)],check=True)


def camera_video(data):
    source=data.run/'task1_overview.mp4'
    if source.exists():
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(source),'-an',
                        '-c:v','libx264','-preset','fast','-crf','23','-threads','2','-pix_fmt','yuv420p',
                        '-movflags','+faststart',str(data.output/'task1_vrx.mp4')],check=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--animate',action='store_true');a=p.parse_args()
    data=TaskEvidence(a.run);figures(data)
    if a.animate:replay(data);camera_video(data)
    files=[data.run/'task1.jsonl',data.run/'task1_result.json',data.run/'task1_judge.jsonl',data.raw/'events.jsonl']
    if (data.run/'task1_overview.mp4').exists():files.append(data.run/'task1_overview.mp4')
    manifest={'run_dir':str(data.run),'passed':data.result['passed'],'reference_paths':len(data.plans),
              'meaning':'Spatial Nav2 reference paths; actual vessel trajectory from independent VRX truth. EKF remains the navigation input.',
              'source_sha256':{str(f.relative_to(data.run)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},
              'artifact_sha256':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(data.output.glob('task1_*'))
                                 if f.suffix in ('.png','.mp4','.gif')},
              'video_sources':{'task1_replay.mp4':'Recorded navigation data rendered at 2.5x simulation speed',
                               'task1_vrx.mp4':'Gazebo overview camera, original frame sequence; source sim stamps are burned in'}}
    (data.output/'task1_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(data.output)


if __name__=='__main__':main()
