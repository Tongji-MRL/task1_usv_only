#!/usr/bin/env python3
"""Independent traffic-yield evidence and a focused plot from a finished run.

Scene/physics truth is used here only for evaluation. This script publishes no
ROS messages, never supplies target motion to the navigation guard, and does
not substitute offline prediction for closed-loop completion.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import numpy as np
from usv_sim.geometry import vessel_clearance


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def analyze(run):
    run=Path(run)
    guards=rows(run/'traffic_guard.jsonl')
    events=rows(run/'task1_mission.jsonl')
    motion=[r for r in rows(run/'task1.jsonl') if 'truth' in r and r.get('phase')!='cleanup']
    scene=json.loads((run/'generated/scene.json').read_text())
    result=json.loads((run/'task1_result.json').read_text())
    maps=json.loads((run/'dynamic_map_result.json').read_text())
    scenes=[r for r in rows(run/'task1_judge.jsonl') if r['type']=='scene' and 'dynamic_pose' in r['data']]
    scene_times=[r['stamp'] for r in scenes]
    def body_at(t):
        i=bisect.bisect_right(scene_times,t)-1
        if i<0:raise ValueError('No prior physical obstacle state')
        xy=scenes[i]['data']['dynamic_pose']
        return dict(x=xy[0],y=xy[1],size=[2.,3.])
    yields=[e for e in events if e['event']=='traffic_yield']
    assessments={g['source_stamp']:g for g in guards}
    responses=[]
    for event in yields:
        guard=assessments[event['source_stamp']]
        stop=next(s for s in result['stops'] if abs(s['start_sim']-event['sim'])<.001)
        body=body_at(event['source_stamp'])
        center=np.mean(guard['risk']['bounds'],axis=0)
        responses.append(dict(request=event,stop=stop,source_age_at_request_s=event['sim']-event['source_stamp'],
                              sensor_estimated_velocity_mps=guard['risk']['velocity'],
                              sensor_box_center_error_m=float(np.linalg.norm(center-[body['x'],body['y']]))))
    times=np.array([g['source_stamp'] for g in guards])
    delay=np.array([g['evaluated_at']-g['source_stamp'] for g in guards])
    report=dict(run=str(run.resolve()),task_passed=result['passed'],collision_contacts=result['collision_contacts'],
                min_all_obstacle_clearance_m=result['min_clearance_m'],
                min_traffic_clearance_m=maps.get('min_moving_obstacle_clearance_m'),
                endpoint_error=result.get('endpoint_error'),traffic_responses=responses,
                dynamic_map={k:v for k,v in maps.items() if k!='samples'},
                freshness_stops=[e for e in events if e['event']=='stop_requested' and 'stale' in e.get('reason','')],
                source_timing=dict(strictly_increasing=bool(np.all(np.diff(times)>0)),
                                   maximum_gap_s=float(np.max(np.diff(times))),
                                   processing_delay_max_s=float(delay.max()),
                                   processing_delay_p95_s=float(np.quantile(delay,.95))),
                merged_tracks=sum(len(g.get('merged_tracks',[])) for g in guards),
                retired_tracks=sum(len(g.get('retired_tracks',[])) for g in guards),
                method='Guard uses LiDAR, FCU TF, surveyed static map/buoys and accepted Nav2 path. Truth body association and clearance in this report are independent evaluation only.',
                evidence_sha256={name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in
                                 ('traffic_guard.jsonl','task1_result.json','dynamic_map_result.json','task1_mission.jsonl')})
    (run/'traffic_avoidance.json').write_text(json.dumps(report,indent=2)+'\n')
    if not yields:return report
    first=responses[0];start=first['request']['sim'];resume=first['stop']['resume_sim']
    end=min(motion[-1]['sim'],(resume or start+80)+25)
    window=[r for r in motion if start-15<=r['sim']<=end]
    truth=np.array([r['truth'] for r in window]);stamps=np.array([r['sim'] for r in window])
    clearance=[];traffic=[]
    for r in window:
        body=body_at(r['sim']);p=r['truth'];traffic.append([body['x'],body['y']])
        clearance.append(vessel_clearance(scene['footprint'],p[0],p[1],p[3],[body]))
    traffic=np.asarray(traffic)
    fig=plt.figure(figsize=(14,8),layout='constrained');grid=fig.add_gridspec(3,2,width_ratios=[1.05,1.])
    ax=fig.add_subplot(grid[:,0]);ax.set_aspect('equal');ax.grid(alpha=.2)
    raw=rows(run/'visualization/raw/events.jsonl')
    paths=[r for r in raw if r['topic']=='/task1/reference_path' and r['sim']<=start]
    if paths:
        path=np.asarray(paths[-1]['poses']);ax.plot(path[:,0],path[:,1],'--',color='#b546bd',label='Accepted path before yield')
    ax.plot(truth[:,0],truth[:,1],color='#009eaf',lw=2,label='WAM-V actual (truth)')
    ax.plot(traffic[:,0],traffic[:,1],':',color='#d38300',lw=2,label='Traffic actual (evaluation only)')
    first_guard=assessments[first['request']['source_stamp']]
    ax.add_patch(Polygon(first_guard['corridor'],facecolor='#f2bf67',edgecolor='#d38300',alpha=.25,
                         label='Predicted reachable hull corridor'))
    box=np.asarray(first_guard['risk']['bounds']);vel=np.asarray(first_guard['risk']['velocity'])
    horizon=first_guard.get('horizon',12.)
    for offset,style,label in ((0,'-','LiDAR measured envelope'),(horizon,'--',f'{horizon:g} s constant-velocity prediction')):
        a,b=box+offset*vel
        ax.add_patch(Rectangle(a,*(b-a),fill=False,edgecolor='#ba7800',linestyle=style,lw=2,label=label))
    for t,label in ((start,'YIELD'),(start+first['stop']['stopped_dwell_confirmed_delay_s'],'STOPPED'),(resume,'RESUME')):
        if t is None:continue
        row=min(window,key=lambda r:abs(r['sim']-t));p=row['truth'];c,s=math.cos(p[3]),math.sin(p[3])
        hull=np.asarray(scene['footprint'])@[[c,s],[-s,c]]+p[:2]
        ax.add_patch(Polygon(hull,fill=False,edgecolor='#007887',lw=1.2))
        ax.annotate(label,xy=p[:2],xytext=(8,12 if label=='YIELD' else -20 if label=='STOPPED' else 0),
                    textcoords='offset points',fontsize=9)
    buoys={b['id']:dict(b) for b in scene['task1']['buoys']}
    for event in events:
        if event['event']=='semantic_update_reported' and event['sim']<=start:
            for b in event['changes']:buoys[b['id']].update(b)
    colors={0:'#888888',1:'#d95e58',2:'#3d995c',3:'#428dd1',4:'#428dd1'}
    for b in buoys.values():
        ax.scatter([b['x']],[b['y']],s=35,c=colors[b['state']])
        if 26<=b['x']<=40 and -9<=b['y']<=13:
            ax.annotate(b['id'].replace('buoy_','B')+' '+{0:'OFF',1:'RED',2:'GREEN',3:'BLUE',4:'BLUE'}[b['state']],
                        (b['x'],b['y']),xytext=(4,5),textcoords='offset points',fontsize=8)
    ax.set(xlim=(23,43),ylim=(-10,19),xlabel='Map east [m]',ylabel='Map north [m]',title='Traffic yield: measured, predicted and actual')
    ax.legend(loc='lower left',fontsize=8)
    values=[(np.hypot(truth[:,4],truth[:,5]),'Actual vessel speed [m/s]'),
            (np.asarray([[r.get('left_n',math.nan),r.get('right_n',math.nan)] for r in window]),'Actual left/right thrust [N]'),
            (np.asarray(clearance),'Physical clearance to traffic [m]')]
    for i,(value,label) in enumerate(values):
        axis=fig.add_subplot(grid[i,1]);axis.plot(stamps-start,value,lw=1.4)
        if i==0:axis.plot(stamps-start,[r.get('preview',[0])[0] for r in window],'--',label='Command vx');axis.legend()
        axis.axvline(0,color='#d38300',ls='--')
        if resume:axis.axvline(resume-start,color='#009eaf',ls=':')
        axis.axvspan(0,(resume or end)-start,color='#f2bf67',alpha=.15)
        axis.set(ylabel=label,xlabel='Seconds from LiDAR yield request');axis.grid(alpha=.25)
    fig.suptitle(f"Task {'PASS' if result['passed'] else 'NOT PASSED'} | contacts {result['collision_contacts']} | "
                 f"minimum traffic clearance {report['min_traffic_clearance_m']:.2f} m",fontsize=14)
    output=run/'visualization';output.mkdir(exist_ok=True)
    fig.savefig(output/'traffic_avoidance.png',dpi=160);plt.close(fig)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args()
    result=analyze(args.run)
    print(json.dumps({k:v for k,v in result.items() if k not in ('evidence_sha256','traffic_responses')},indent=2))
