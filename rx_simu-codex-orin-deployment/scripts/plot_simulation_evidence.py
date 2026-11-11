#!/usr/bin/env python3
"""Recreate compact scientific figures from the tracked evidence JSON alone."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def geometry(items,output):
    fig,axs=plt.subplots(1,3,figsize=(12,4.3),layout='constrained',sharey=True)
    for ax,h in zip(axs,(.3,.9,1.8)):
        a=np.array([[next(c for c in r['result']['cases'] if c['height_m']==h and c['body_distance_m']==d)
            ['observations']['/scan']['hit_fraction'] for d in (3.,5.,8.,12.,16.)] for r in items])
        ax.imshow(a,vmin=0,vmax=1,cmap='YlGn');ax.set_xticks(range(5),['3*','5','8','12','16'])
        ax.set_yticks(range(len(items)),[x['label'] for x in items])
        ax.set(title=f'Obstacle height {h:g} m',xlabel='Distance from vessel center (m)')
        for (i,j),v in np.ndenumerate(a):
            ax.text(j,i,f'{v*100:.0f}%',ha='center',va='center',color='white' if v>.65 else 'black',fontsize=9)
    fig.suptitle('Physical box detected in projected LaserScan: fraction of observed frames')
    fig.supxlabel('* 3 m fixture intersects the conservative hull envelope; self-region rejection is expected.',fontsize=9)
    fig.savefig(output/'mid360_visibility.png',dpi=150);plt.close(fig)


def loiter(items,output):
    items=[r for r in items if r['result'].get('measurement_completed')]
    if not items:return
    fig,axes=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
    for item in items:
        r=item['result'];p=np.asarray(r['truth_curve_sim_xy']);p[:,0]-=p[0,0]
        error=np.linalg.norm(p[:,1:]-r['anchor_map_xy_m'],axis=1)
        axes[1 if r['mode']=='zero_velocity' else 0].plot(p[:,0],error,label=item['label'])
    for ax in axes:
        ax.set(xlabel='Simulation time after entry (s)',ylabel='Truth error to fixed target (m)')
        ax.grid(alpha=.3);ax.legend(fontsize=8)
    axes[0].axhline(.3,color='black',linestyle='--',linewidth=.8)
    axes[0].set_title('Native LOITER (dashed: 0.3 m radius)')
    axes[1].set_title('GUIDED zero velocity comparison')
    fig.savefig(output/'loiter_comparison.png',dpi=150);plt.close(fig)


def gnss(items,output):
    if not items:return
    profiles=items[-1]['result']['profiles'];x=np.arange(len(profiles));fig,ax=plt.subplots(figsize=(12,4.5),layout='constrained')
    ax.bar(x-.18,[p['localization_max_m'] for p in profiles],.36,label='Max EKF vs truth error')
    ax.bar(x+.18,[p['original_target_truth_error_max_m'] for p in profiles],.36,label='Max truth error to original hold target')
    ax.set_xticks(x,[p['profile']+'\n'+'/'.join(p['modes']) for p in profiles],fontsize=8)
    ax.set(ylabel='Error (m)',title='Sequential GPS_INPUT experiment; each phase includes its transition')
    ax.legend();ax.grid(axis='y',alpha=.25)
    fig.savefig(output/'gnss_degradation.png',dpi=150);plt.close(fig)


def main():
    p=argparse.ArgumentParser();p.add_argument('evidence',type=Path);a=p.parse_args()
    rows=json.loads(a.evidence.read_text())['runs'];output=a.evidence.parent
    g=[r for r in rows if r['kind']=='geometry']
    if g:geometry(g,output)
    loiter([r for r in rows if r['kind']=='loiter'],output)
    gnss([r for r in rows if r['kind']=='gnss'],output)


if __name__=='__main__':main()
