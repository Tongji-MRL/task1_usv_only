#!/usr/bin/env python3
"""Counterfactual scoring at recorded route acceptances, without publishing ROS.

Only recorded FCU poses, measured traffic tracks and accepted paths are used.
Reference identity is discarded at semantic changes and phase/goal changes.
This evaluates decisions on old observations, not closed-loop performance.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from robotx_task1.route_policy import assess_routes
from robotx_task1.traffic import Track


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();run=args.run
    def rows(name):return [json.loads(line) for line in (run/name).open()]
    paths=[e for e in rows('visualization/raw/events.jsonl') if e['topic']=='/task1/reference_path']
    guards=rows('traffic_guard.jsonl');mission=rows('task1_mission.jsonl')
    fp=json.loads((run/'generated/scene.json').read_text())['footprint']
    output=[]
    for index,path in enumerate(paths):
        ready=min((m for m in mission if m['event']=='path_ready'),key=lambda m:abs(m['sim']-path['sim']))
        if ready['stage']!='TRANSIT':continue
        reference=paths[index-1] if index else None
        if reference is not None:
            invalidation=any(reference['sim']<m['sim']<=path['sim'] and
                             m['event'] in ('semantic_update_reported','stage_target') for m in mission)
            if invalidation:reference=None
        g=min(guards,key=lambda r:abs(r['source_stamp']-path['sim']))
        tracks=[Track(t['id'],t['stamp'],np.asarray(t['bounds']),np.asarray(t['velocity']),t['moving']) for t in g['tracks']]
        decision,profile=assess_routes(np.asarray(path['poses']),np.asarray(reference['poses']) if reference else None,
                                     g['pose'],fp,tracks,g['source_stamp'])
        output.append(dict(source_stamp=g['source_stamp'],path_stamp=path['stamp'],
                           reference_stamp=reference['stamp'] if reference else None,**decision))
    data=dict(source_run=str(run.resolve()),limitation='Read-only counterfactual at old route acceptances, not live validation.',decisions=output)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps(data,indent=2))


if __name__=='__main__':main()
