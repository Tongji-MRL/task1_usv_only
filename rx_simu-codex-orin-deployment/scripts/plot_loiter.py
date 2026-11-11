#!/usr/bin/env python3
"""Plot fixed-anchor station-keeping error from the independent recorder."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np


def main():
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path)
    run=parser.parse_args().run
    result=json.loads((run/'loiter_result.json').read_text())
    if not result.get('measurement_completed'):raise ValueError('No completed measurement')
    rows=[json.loads(line) for line in (run/'loiter.jsonl').read_text().splitlines()]
    rows=[r for r in rows if r.get('phase')=='hold_measurement' and 'truth' in r]
    t=np.array([r['sim'] for r in rows]);t-=t[0]
    xy=np.array([r['truth'][:2] for r in rows])
    anchor=np.array(result['anchor_map_xy_m'])
    radius=result['parameters']['LOIT_RADIUS']
    fig,axes=plt.subplots(1,3,figsize=(14,4.5),layout='constrained')
    ax=axes[0]
    ax.plot(xy[:,0],xy[:,1],label='VRX actual trajectory',color='tab:blue')
    ax.scatter(*anchor,marker='x',s=75,color='black',label='FCU hold target')
    ax.add_patch(Circle(anchor,radius,fill=False,linestyle='--',color='tab:orange',label='LOIT_RADIUS'))
    ax.set(aspect='equal',xlabel='map east (m)',ylabel='map north (m)',title='Fixed target and actual position')
    ax.legend(fontsize=8);ax.grid(alpha=.3)
    axes[1].plot(t,np.linalg.norm(xy-anchor,axis=1),label='Truth position error')
    axes[1].axhline(radius,linestyle='--',color='tab:orange',label='LOIT_RADIUS')
    axes[1].set(xlabel='simulation time from entry (s)',ylabel='error (m)',title='Position error to fixed anchor')
    axes[1].legend(fontsize=8);axes[1].grid(alpha=.3)
    for side in ('left','right'):
        axes[2].plot(t,[r.get(side+'_n',np.nan) for r in rows],label=side)
    axes[2].set(xlabel='simulation time from entry (s)',ylabel='force (N)',title='Thruster model feedback')
    axes[2].legend(fontsize=8);axes[2].grid(alpha=.3)
    fig.suptitle(f"Rover 4.4.0 {result['mode']}: wind {result['wind_mps']} m/s, wave gain {result['wave_gain']}")
    path=run/'loiter_measurement.png';fig.savefig(path,dpi=160);plt.close(fig);print(path)


if __name__=='__main__':main()
