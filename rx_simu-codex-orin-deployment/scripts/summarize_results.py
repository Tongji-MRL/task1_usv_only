#!/usr/bin/env python3
"""Build a compact, reproducible ledger from retained raw integration evidence."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

ROOT=Path(__file__).resolve().parents[1]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def metrics(path):
    rows=[r for line in path.read_text().splitlines() if 'truth' in (r:=json.loads(line))]
    active=[r for r in rows if r['phase']=='navigation']
    if not active:return {}
    pairs=[r for r in active if 'ekf' in r]
    error=[math.dist(r['truth'][:2],r['ekf'][:2]) for r in pairs]
    report={'samples':len(active),'ekf_position_rmse_m':float(np.sqrt(np.mean(np.square(error)))) if error else None,
            'peak_true_speed_mps':max(math.hypot(*r['truth'][4:6]) for r in active),
            'real_time_factor':(active[-1]['sim']-active[0]['sim'])/(active[-1]['wall']-active[0]['wall']) if len(active)>1 else None,
            'true_heave_peak_to_peak_m':float(np.ptp([r['truth'][2] for r in active])),
            'max_scan_source_age_s':max(r.get('scan_age',0) for r in active)}
    if 'truth_rpy' in active[-1]:
        report['true_roll_pitch_peak_to_peak_rad']=np.ptp([r['truth_rpy'][:2] for r in active if 'truth_rpy' in r],axis=0).tolist()
    after=[r for r in rows if r['phase']=='post_goal']
    if after:
        start=after[0]['wall']
        # Early traces kept the same phase during final HOLD/disarm cleanup.
        # Limit them to the intended observation window; newer traces tag cleanup.
        after=[r for r in after if r['wall']<start+10]
        def stable(predicate):
            beginning=None
            for row in after:
                if predicate(row):
                    if beginning is None:beginning=row['wall']
                    if row['wall']-beginning>=.5:return beginning-start
                else:beginning=None
            return None
        report['post_goal']={'thrust_measurement':sorted(set(r.get('thrust_source','command_topic_only') for r in after)),
                             'stable_exact_zero_thrust_delay_s':stable(lambda r:abs(r.get('left_n',999))+abs(r.get('right_n',999))<1e-6),
                             'stable_speed_below_002_delay_s':stable(lambda r:math.hypot(*r['truth'][4:6])<.02 and abs(r['truth'][6])<.02),
                             'final_thrust_n':[after[-1].get('left_n'),after[-1].get('right_n')],
                             'travel_distance_m':sum(math.dist(a['truth'][:2],b['truth'][:2]) for a,b in zip(after,after[1:]))}
    return report


def plot(item):
    run=Path(item['run_dir']);name=item['case']['name'];trace=run/(name+'.jsonl')
    events=[json.loads(line) for line in trace.read_text().splitlines()]
    rows=[r for r in events if 'truth' in r and r['phase'] in ('navigation','post_goal')]
    if not rows:return
    a=np.array([r['truth'] for r in rows]);e=np.array([r.get('ekf',[np.nan]*7) for r in rows]);t=np.array([r['wall'] for r in rows]);t-=t[0]
    scene=json.loads((run/'generated/scene.json').read_text())
    fig,ax=plt.subplots(1,2,figsize=(11,4.5),layout='constrained')
    ax[0].plot(a[:,0],a[:,1],label='VRX truth');ax[0].plot(e[:,0],e[:,1],'--',label='FCU EKF',alpha=.8)
    for obj in scene['obstacles']:
        sx,sy=obj['size'];ax[0].add_patch(Rectangle((obj['x']-sx/2,obj['y']-sy/2),sx,sy,color='tab:red',alpha=.5))
    goals=item['case'].get('goals',[])
    result=item.get('result') or {}
    if not goals and isinstance(result,dict) and 'target' in result:goals=[result['target']]
    for x,y,_ in goals:ax[0].plot(x,y,'kx')
    moving=[r['position'] for r in events if r.get('event')=='move_dynamic_obstacle' and r['position']!=[15,20]]
    if moving:
        positions=np.asarray(moving)
        ax[0].plot(positions[:,0],positions[:,1],':',color='tab:red',label='Scripted obstacle centre')
    nearest=min(range(len(rows)),key=lambda i:rows[i].get('clearance_m',math.inf))
    angle=a[nearest,3];rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    hull=np.asarray(scene['footprint'])@rotation.T+a[nearest,:2]
    ax[0].add_patch(Polygon(hull,fill=False,color='tab:blue',alpha=.7,label='Hull at closest pass'))
    previous=[r for r in events if r.get('event')=='move_dynamic_obstacle' and r['wall']<=rows[nearest]['wall']]
    if previous:
        x,y=previous[-1]['position']
        ax[0].add_patch(Rectangle((x-1,y-1),2,2,color='tab:red',alpha=.3))
    ax[0].set(xlabel='East (m)',ylabel='North (m)',title=name,aspect='equal')
    ax[0].set_xlim(a[:,0].min()-4,a[:,0].max()+4);ax[0].set_ylim(a[:,1].min()-4,a[:,1].max()+4)
    ax[0].legend();ax[0].grid(alpha=.3)
    ax[1].plot(t,np.linalg.norm(a[:,4:6],axis=1),label='Actual planar speed')
    ax[1].plot(t,[r.get('preview',[0,0])[0] for r in rows],label='Command speed')
    ax[1].set(xlabel='Wall time since action start (s)',ylabel='Speed (m/s)');ax[1].grid(alpha=.3);ax[1].legend()
    fig.savefig(run/'trajectory.png',dpi=150);plt.close(fig)


def navigation_table(output):
    lines=['# 最新导航测量结果','',
           '由 `scripts/summarize_results.py` 从保留的运行记录生成。仅展示每个名称最近一次尝试；历史失败见 `results.json` 的 `attempts`。',
           '位置、航向和净空均使用 VRX 真值独立评估。取消和不可达用例的终点误差不作为到点验收。',
           '', '| 用例 | 判定 / action 状态 | 位置误差 m | 航向误差 rad | 最小净空 m | 10 s 漂移 m | 日志 |',
           '|---|---|---:|---:|---:|---:|---|']
    for name,entry in output['latest'].items():
        if 'goals' not in entry['case'] and name!='dynamic_navigation':continue
        result=entry.get('result') or {}
        values=[f'{result[key]:.3f}' if key in result else '—' for key in
                ('true_position_error_m','true_yaw_error_rad','min_planar_clearance_m','drift_10s_m')]
        verdict='通过' if result.get('passed') is True else '失败 / 未完成'
        run=Path(entry['run_dir']).relative_to(ROOT)
        lines.append(f"| {name} | {verdict} / {result.get('status','—')} | "+' | '.join(values)+
                     f' | [{run.name}](../{run}/{name}_result.json) |')
    lines.extend(['','action 4 = 成功，5 = 取消，6 = 中止。验收条件不只检查 action 状态，还检查真值误差、碰撞、途经点和零指令延迟。',
                  '推力归零和物理停止观测见 `results.json` 的 `metrics.post_goal`；`thrust_measurement` 区分输入推力命令与物理插件反馈，`null` 表示观察窗口内未达到该条件。',''])
    (ROOT/'docs/results_summary.md').write_text('\n'.join(lines))


def main():
    ledger=json.loads((ROOT/'runs/suite_results.json').read_text());latest={}
    for item in ledger:latest[item['case']['name']]=item
    output={'attempts':ledger,'latest':{}}
    for name,item in latest.items():
        run=Path(item['run_dir']);entry=dict(item);path=run/(name+'.jsonl')
        if path.exists():entry['metrics']=metrics(path)
        entry['evidence_sha256']={str(p.relative_to(run)):sha(p) for p in
            [path,run/(name+'_result.json'),run/'generated/usv_nav.sdf',run/'generated/nav2_wamv.yaml',
             run/'provenance.json',run/'source_snapshot.tar.gz',run/'map_alignment.json',
             run/'rover.parm',run/'thrust_observation.json',run/'generated/scene.json',
             run/'generated/water.pgm',run/'generated/water.yaml',
             run/'visualization/manifest.json',run/'visualization/recording.json',
             run/'overview.png',run/'overview.mp4'] if p.exists()}
        if path.exists() and ('goals' in item['case'] or name=='dynamic_navigation'):
            plot(item)
            if (run/'trajectory.png').exists():entry['evidence_sha256']['trajectory.png']=sha(run/'trajectory.png')
        output['latest'][name]=entry
    for name in ['single_1','single_2','single_3','multi_1','multi_2','multi_3','static_obstacle','wind','waves','wind_waves']:
        if name in latest:
            result=latest[name].get('result') or {}
            print(name, result.get('passed'),result.get('true_position_error_m'),result.get('true_yaw_error_rad'))
    (ROOT/'docs/results.json').write_text(json.dumps(output,indent=2)+'\n')
    navigation_table(output)


if __name__=='__main__':main()
