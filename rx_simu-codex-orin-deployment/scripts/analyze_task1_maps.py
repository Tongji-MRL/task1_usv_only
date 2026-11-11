#!/usr/bin/env python3
"""Measure the moving obstacle in recorded maps, independently of task control."""
import argparse
import bisect
import json
import math
from pathlib import Path
import numpy as np


def buoy_persistence(run):
    run=Path(run);raw=run/'visualization/raw'
    scenario=json.loads((run/'generated/task1_scenario.json').read_text())
    if not (raw/'events.jsonl').exists():return {'passed':False,'reason':'Map recording was not enabled'}
    mission=[json.loads(l) for l in (run/'task1_mission.jsonl').read_text().splitlines()]
    start=next((e['sim'] for e in mission if e['event']=='started'),None)
    if start is None:return {'passed':False,'reason':'No task start'}
    counts={b['id']:dict(observed=0,missing=0) for b in scenario['buoys']};missing=[]
    for event in map(json.loads,(raw/'events.jsonl').read_text().splitlines()):
        if event['topic']!='/global_costmap/costmap' or event['stamp']<start:continue
        if event['frame']!='map':raise ValueError('Task1 persistence requires the configured map frame')
        with np.load(raw/event['file']) as data:grid=data['data']
        ox,oy=event['origin'][:2];res=event['resolution']
        for b in scenario['buoys']:
            radius=b['radius']+res
            x0=max(0,int(math.floor((b['x']-radius-ox)/res)));x1=min(grid.shape[1],int(math.ceil((b['x']+radius-ox)/res)))
            y0=max(0,int(math.floor((b['y']-radius-oy)/res)));y1=min(grid.shape[0],int(math.ceil((b['y']+radius-oy)/res)))
            present=x0<x1 and y0<y1 and np.any(grid[y0:y1,x0:x1]>=100)
            counts[b['id']]['observed']+=1;counts[b['id']]['missing']+=int(not present)
            if not present and len(missing)<20:missing.append(dict(stamp=event['stamp'],id=b['id']))
    result=dict(passed=all(v['observed']>0 and v['missing']==0 for v in counts.values()),buoys=counts,
                initially_off=[b['id'] for b in scenario['buoys'] if b['state']==0],first_missing=missing,
                criterion='Every known physical buoy remains lethal in every recorded global map after task start, independently of its light phase.')
    (run/'buoy_map_result.json').write_text(json.dumps(result,indent=2)+'\n');return result


def analyze(run):
    run=Path(run);scene=json.loads((run/'generated/scene.json').read_text())
    if scene['task1']['dynamic']=='none':return None
    def unavailable(reason):
        report={'passed':False,'status':'not_evaluated','reason':reason}
        (run/'dynamic_map_result.json').write_text(json.dumps(report,indent=2)+'\n')
        return report
    raw=run/'visualization/raw';events_file=raw/'events.jsonl'
    if not events_file.exists():return unavailable('Map recording was not enabled')
    if not (run/'map_alignment.json').exists():return unavailable('FCU/map alignment unavailable; navigation did not become ready')
    events=[json.loads(l) for l in events_file.read_text().splitlines()]
    records=[json.loads(l) for l in (run/'task1_judge.jsonl').read_text().splitlines()]
    states=sorted((r for r in records if r['type']=='scene' and 'dynamic_pose' in r['data']),key=lambda r:r['stamp'])
    if not states:return unavailable('No source-timed moving-obstacle state recorded')
    times=[r['stamp'] for r in states]
    origin=np.asarray(json.loads((run/'map_alignment.json').read_text())['offset_xy_m'])
    x=states[0]['data']['dynamic_pose'][0];counts=[]
    for event in events:
        if event['topic']!='/global_costmap/costmap':continue
        index=bisect.bisect_right(times,event['stamp'])-1
        if index<0:continue
        offset=origin if event['frame']=='odom' else np.zeros(2)
        if event['frame'] not in ('map','odom'):raise ValueError('Unknown costmap frame')
        ox,oy=np.asarray(event['origin'][:2])+offset;res=event['resolution']
        with np.load(raw/event['file']) as data:grid=data['data']
        x0=max(0,int(math.floor((x-1.2-ox)/res)));x1=min(grid.shape[1],int(math.ceil((x+1.2-ox)/res)))
        y0=max(0,int(math.floor((-1.7-oy)/res)));y1=min(grid.shape[0],int(math.ceil((1.7-oy)/res)))
        if x0>=x1 or y0>=y1:continue
        counts.append(dict(stamp=event['stamp'],lethal_cells=int(np.count_nonzero(grid[y0:y1,x0:x1]>=100)),
                           moving_y=states[index]['data']['dynamic_pose'][1]))
    marked=[c for c in counts if abs(c['moving_y'])<3.2 and c['lethal_cells']>0]
    left=next((r['stamp'] for r in states if r['data']['dynamic_pose'][1]>3.5),None)
    cleared=next((c for c in counts if left is not None and c['stamp']>=left and c['lethal_cells']==0),None)
    from usv_sim.geometry import vessel_clearance
    distances=[]
    for row in records:
        if row['type']!='truth':continue
        i=bisect.bisect_right(times,row['stamp'])-1
        if i<0:continue
        p=row['pose'];xy=states[i]['data']['dynamic_pose']
        distances.append(vessel_clearance(scene['footprint'],p[0],p[1],p[2],[dict(x=xy[0],y=xy[1],size=[2.,3.])]))
    report=dict(passed=bool(marked and cleared),region=[x-1.2,x+1.2,-1.7,1.7],
                marking_observed=bool(marked),
                clearing_status='not_observed_target_departure' if left is None else 'observed_clear' if cleared else 'not_observed_clear',
                peak_lethal_cells=max((c['lethal_cells'] for c in marked),default=0),
                obstacle_left_region_sim=left,first_clear_sim=cleared['stamp'] if cleared else None,
                clear_delay_s=cleared['stamp']-left if cleared else None,
                min_moving_obstacle_clearance_m=min(distances) if distances else None,samples=counts,
                criterion='Actual global costmap marks the moving body at its crossing, then clears its former region after it leaves.')
    if scene['task1']['dynamic']=='block':
        mission=[json.loads(l) for l in (run/'task1_mission.jsonl').read_text().splitlines()]
        transit=next((e['sim'] for e in mission if e['event']=='stage_target' and e['stage']=='TRANSIT'),None)
        plan=next((e for e in events if e['topic']=='/plan' and e.get('poses') and transit is not None and e['stamp']>transit),None)
        held=[s['stamp'] for s in states if abs(s['data']['dynamic_pose'][1])<1e-6]
        if plan and held:
            parameters=json.loads((run/'generated/nav2_wamv.yaml').read_text())
            padding=parameters['local_costmap']['local_costmap']['ros__parameters']['footprint_padding']
            padded=[[math.copysign(abs(v)+padding,v) for v in p] for p in scene['footprint']]
            obstacle=[dict(x=x,y=0.,size=[2.,3.])]
            minimum=lambda footprint:min(vessel_clearance(footprint,*p,obstacle) for p in plan['poses'])
            pad_clearance=minimum(padded)
            result=json.loads((run/'task1_result.json').read_text())
            report['block_response']={
                'initial_transit_plan_sim':plan['stamp'],'observed_hold_sim':[min(held),max(held)],
                'initial_plan_physical_clearance_m':minimum(scene['footprint']),
                'initial_plan_padded_clearance_m':pad_clearance,
                'initial_plan_requires_avoidance':pad_clearance<=0.,
                'mission_stops_while_held':[s for s in result['stops'] if min(held)<=s['start_sim']<=max(held)],
                'interpretation':'Holding an obstacle nearby does not prove wait/resume behavior. The initial planned footprint must actually intersect the later held obstacle to establish a blocked-route challenge.'}
            # A semantic light change can invalidate the first transit plan.
            # Also inspect the route actually accepted before the hold began.
            references=[e for e in events if e['topic']=='/task1/reference_path' and e.get('poses')
                        and transit<=e['sim']<=min(held)]
            active=max(references,key=lambda e:e['sim']) if references else None
            if active:
                active_clearance=min(vessel_clearance(padded,*p,obstacle) for p in active['poses'])
                report['block_response'].update(active_reference_path_stamp=active['stamp'],
                    active_reference_accepted_sim=active['sim'],active_reference_padded_clearance_m=active_clearance,
                    active_reference_requires_avoidance=active_clearance<=0.)
            report['block_response']['interpretation']='The first transit plan may predate the light change. The accepted reference before the hold is checked separately; an intersecting padded hull establishes a blocked-route challenge. Geometry alone does not prove successful wait/resume or task completion.'
    (run/'dynamic_map_result.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--buoys',action='store_true');a=p.parse_args()
    report=buoy_persistence(a.run) if a.buoys else analyze(a.run)
    if report is None:raise SystemExit('This run has no moving obstacle; use --buoys for the known buoy layer')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'}))
    if not report['passed']:raise SystemExit(1)
