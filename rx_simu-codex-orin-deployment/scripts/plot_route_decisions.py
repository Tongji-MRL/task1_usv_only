#!/usr/bin/env python3
"""Compare recorded transit paths and evaluate along-route ETA checkpoints.

No truth reaches navigation. Actual arrival is the first forward crossing of a
checkpoint's normal, within 1.5 m laterally and 1.2 rad of its path heading.
Only uninterrupted execution is evaluated; cancelled future arrivals are not
invented. Checkpoints are 5 m apart, starting 2 m from the assessed pose.
"""
import argparse
import hashlib
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from plot_task1 import TaskEvidence, setup, TRACK, PLAN
from analyze_route_efficiency import summarize
from robotx_task1.route_policy import arrival_profile, RouteConfig, wrap


def transit(data):
    start=next(e['sim'] for e in data.mission if e['event']=='stage_target' and e['stage']=='TRANSIT')
    end=next((e['sim'] for e in data.mission if e['event']=='stage_target' and e['stage']=='CIRCLE_EXIT'),data.rows[-1]['sim'])
    rows=[e for e in data.rows if start<=e['sim']<=end]
    return start,end,rows


def checkpoints(data):
    decisions=[json.loads(l) for l in (data.run/'route_decisions.jsonl').open()]
    results=[]
    for selection in data.mission:
        if selection['event']!='route_selected' or selection['stage']!='TRANSIT':continue
        decision=next(r for r in decisions if r['source_stamp']==selection['source_stamp']
                      and r['path_stamp']==selection['path_stamp'])
        config=next(r['configuration'] for r in decisions if r['path_stamp']==selection['path_stamp'] and 'configuration' in r)
        # Historical records before the terminal-cap change ran at full speed.
        config=dict(config);config.setdefault('terminal_ratio',1.)
        plan=next((e for e in data.plans if abs(e['stamp']-selection['path_stamp'])<1e-6),None)
        if plan is None:continue
        execution=next((e for e in data.mission if e['event']=='execution_started' and e['sim']>=selection['sim']),None)
        if execution is None:continue
        end=next(e for e in data.mission if e['sim']>execution['sim'] and e['event'] in ('stop_requested','controller_result'))
        rows=[r for r in data.rows if execution['sim']<=r['sim']<=end['sim']]
        if len(rows)<2:continue
        profile=arrival_profile(data.path(plan),decision['pose'],RouteConfig(**config))
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(profile['poses'][:,:2],axis=0),axis=1))]
        times=np.array([r['sim']-execution['sim'] for r in rows]);truth=np.array([r['truth'] for r in rows])
        arrivals=[];after=0
        for at in np.arange(2.,arc[-1],5.):
            index=int(np.searchsorted(arc,at));pose=profile['poses'][index]
            direction=np.array([np.cos(pose[2]),np.sin(pose[2])]);normal=np.array([-direction[1],direction[0]])
            delta=truth[:,:2]-pose[:2];progress=delta@direction;lateral=np.abs(delta@normal)
            crosses=np.flatnonzero((progress[:-1]<0)&(progress[1:]>=0)&(lateral[1:]<=1.5)
                                  &(np.abs(wrap(truth[1:,3]-pose[2]))<=1.2))+1
            crosses=crosses[crosses>=after]
            if not len(crosses):continue
            j=int(crosses[0]);after=j
            fraction=-progress[j-1]/(progress[j]-progress[j-1])
            actual=float(times[j-1]+fraction*(times[j]-times[j-1]))
            arrivals.append(dict(distance_m=float(arc[index]),predicted_s=float(profile['eta'][index]),
                                 earliest_s=float(profile['earliest'][index]),latest_s=float(profile['latest'][index]),
                                 actual_s=actual,error_s=actual-float(profile['eta'][index])))
        results.append(dict(path_stamp=selection['path_stamp'],execution_start_s=execution['sim'],
                            execution_duration_s=end['sim']-execution['sim'],end_event=end['event'],
                            arrivals=arrivals,profile={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in profile.items()},
                            distance_m=arc.tolist()))
    return results


def main():
    p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('current',type=Path)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    old,new=TaskEvidence(args.baseline),TaskEvidence(args.current)
    fig,axes=plt.subplots(1,2,figsize=(15,6),constrained_layout=True)
    metrics=[]
    for ax,data,title in zip(axes,[old,new],['改进前','改进后']):
        start,end,rows=transit(data);motion=np.array([r['truth'] for r in rows]);m=summarize(data.run);metrics.append(m)
        data.map(ax,rows[-1]['wall']);data.markers(ax,end)
        for i,e in enumerate(e for e in data.plans if start<=e['sim']<=end):
            path=data.path(e);ax.plot(path[:,0],path[:,1],'--',color=PLAN,alpha=.55,lw=1,label='已接受参考路径' if i==0 else None)
        ax.plot(motion[:,0],motion[:,1],color=TRACK,lw=2.5,label='VRX 实际穿越航迹',zorder=11)
        shown=set()
        for event in data.mission:
            if event['stage']!='TRANSIT':continue
            if event['event'] in ('traffic_yield','route_wait_selected'):
                kind=event['event'];row=min(rows,key=lambda r:abs(r['sim']-event['sim']))
                label=('近场交通保护停车' if kind=='traffic_yield' else '策略停等（含原地观察）') if kind not in shown else None
                ax.scatter(*row['truth'][:2],marker='s' if kind=='traffic_yield' else 'o',s=65,
                           facecolors='#d97706' if kind=='traffic_yield' else 'none',edgecolors='#d97706',zorder=12,label=label)
                shown.add(kind)
        ax.scatter(55,5,marker='*',s=160,color='#7c3aed',label='穿越目标',zorder=12)
        setup(ax,f"{title}：{m['transit_distance_m']:.1f} m / {m['transit_duration_s']:.1f} s\n"
              f"策略停等 {m['route_wait_decisions']} 次；通道切换 {m['geometric_transit_switches']} 次\n"
              f"反向进度 {m['transit_reverse_progress_m']:.2f} m；整场超时停车 {m['stale_stops']} 次",[0,70,-23,24])
        ax.legend(loc='lower left',fontsize=9)
    fig.suptitle('同一港区、一次灯变、横穿障碍 · 灯色为穿越结束时状态')
    fig.savefig(args.output/'route_comparison.png',dpi=140);plt.close(fig)
    arrivals=checkpoints(new)
    best=max(arrivals,key=lambda r:len(r['arrivals']))
    profile=best['profile'];points=best['arrivals']
    fig,ax=plt.subplots(figsize=(10,5),constrained_layout=True)
    ax.fill_between(best['distance_m'],profile['earliest'],profile['latest'],color='#c7d2fe',alpha=.6,label='模型时间区间（未作统计校准）')
    ax.plot(best['distance_m'],profile['eta'],color='#4f46e5',label='预测沿途 ETA')
    ax.plot([r['distance_m'] for r in points],[r['actual_s'] for r in points],'o-',color=TRACK,label='实际正向通过检查点')
    ax.axhline(best['execution_duration_s'],color='#d97706',ls='--',label='本段执行结束，后续不计入误差')
    ax.set(xlabel='沿候选路径距离 / m',ylabel='从本次启动执行开始的时间 / s',
           title=f"沿途 ETA 核验 · 路径时间戳 {best['path_stamp']:.3f} s · 检查点最多的连续执行段")
    ax.grid(alpha=.2);ax.legend();fig.savefig(args.output/'route_eta.png',dpi=140);plt.close(fig)
    all_points=[p for route in arrivals for p in route['arrivals']]
    result=dict(metrics=metrics,checkpoint_method=__doc__,routes=arrivals,
                checkpoint_count=len(all_points),mean_absolute_eta_error_s=float(np.mean([abs(r['error_s']) for r in all_points])) if all_points else None,
                interval_coverage=float(np.mean([r['earliest_s']<=r['actual_s']<=r['latest_s'] for r in all_points])) if all_points else None,
                source_sha256={name:hashlib.sha256((new.run/name).read_bytes()).hexdigest() for name in
                    ['route_decisions.jsonl','task1_mission.jsonl','task1.jsonl','visualization/raw/events.jsonl']})
    (args.output/'route_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ['checkpoint_count','mean_absolute_eta_error_s','interval_coverage']}))


if __name__=='__main__':main()
