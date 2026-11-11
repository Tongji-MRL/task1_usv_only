#!/usr/bin/env python3
"""Export a recorded TRANSIT map/FCU pose for the standalone search diagnostic."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('stamp',type=float)
    p.add_argument('--reference-stamp',type=float,help='Check this accepted map-frame path against the recorded full-hull costs')
    p.add_argument('--no-search',action='store_true',help='Export/check only; do not run the two search diagnostics')
    a=p.parse_args();run=a.run;t=a.stamp;raw=run/'visualization/raw'
    def records(name):return [json.loads(l) for l in (run/name).read_text().splitlines()]
    event=max((e for e in records('visualization/raw/events.jsonl') if e['topic']=='/global_costmap/costmap' and e['stamp']<=t),key=lambda e:e['stamp'])
    if event['frame']!='map':raise ValueError('Expected map coordinates')
    pose=min((e for e in records('task1.jsonl') if 'ekf' in e),key=lambda e:abs(e['sim']-t))['ekf']
    mission=records('task1_mission.jsonl')
    goal=max((e for e in mission if e['event']=='planning' and e['stage']=='TRANSIT' and e['sim']<=t),key=lambda e:e['sim'])['target']
    judge=records('task1_judge.jsonl')
    status=max((e for e in judge if e['type']=='status' and e['stamp']<=t),key=lambda e:e['stamp'])['data']
    scene=max((e for e in judge if e['type']=='scene' and e['stamp']<=t),key=lambda e:e['stamp'])['data']
    config=json.loads((run/'generated/nav2_wamv.yaml').read_text())
    fp=json.loads(config['global_costmap']['global_costmap']['ros__parameters']['footprint'])
    pad=config['global_costmap']['global_costmap']['ros__parameters']['footprint_padding']
    fp=[[math.copysign(abs(v)+pad,v) for v in xy] for xy in fp]
    with np.load(raw/event['file']) as z:grid=z['data'].astype(int)
    # Inverse of Nav2 OccupancyGrid translation. Values 1..98 are quantized;
    # lethal/unknown/inscribed costs, and hence collision decisions, remain exact.
    grid=np.where(grid<0,255,np.where(grid==100,254,np.where(grid==99,253,np.where(grid==0,0,1+(grid-1)*251/97)))).astype(int)
    out=run/'search_replay';out.mkdir(exist_ok=True);data=out/f'{t:.3f}.txt'
    with data.open('w') as f:
        f.write(f"{event['width']} {event['height']} {event['resolution']} {event['origin'][0]} {event['origin'][1]}\n")
        f.write(' '.join(map(str,[pose[0],pose[1],pose[3],*goal,status['passed_colors']]))+'\n')
        f.write(str(len(scene['buoys']))+'\n')
        for b in scene['buoys']:f.write(f"{b['id']} {b['x']} {b['y']} {b['radius']} {b['state']}\n")
        f.write(str(len(fp))+'\n')
        for xy in fp:f.write(' '.join(map(str,xy))+'\n')
        np.savetxt(f,grid,fmt='%d')
    results={}
    task=config['planner_server']['ros__parameters']['Task1']
    visibility=[str(task.get(k,0.)) for k in ('visibility_clearance','lidar_x','lidar_y')]
    for mode in (() if a.no_search else ('euclidean','dubins')):
        r=subprocess.run([str(ROOT/'build/task1_search/replay'),str(data),mode,*visibility],capture_output=True,text=True,check=True)
        results[mode]=r.stdout.strip()
    if a.reference_stamp is not None:
        path=next(e for e in records('visualization/raw/events.jsonl') if e['topic']=='/task1/reference_path'
                  and abs(e['stamp']-a.reference_stamp)<1e-6 and e['frame']=='map')
        reference=out/f'{t:.3f}.reference.txt';np.savetxt(reference,np.asarray(path['poses']).reshape(-1,3))
        r=subprocess.run([str(ROOT/'build/task1_search/replay'),str(data),'reference',str(reference),*visibility],
                         capture_output=True,text=True,check=True)
        results['reference_check']=json.loads(r.stdout);results['reference_stamp']=a.reference_stamp
    results.update(map_stamp=event['stamp'],requested_stamp=t,source=str(run),limitation='Recorded OccupancyGrid quantizes nonlethal costs; offline search only.')
    (out/f'{t:.3f}.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results))

if __name__=='__main__':main()
