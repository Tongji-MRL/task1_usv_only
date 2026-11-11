#!/usr/bin/env python3
"""Audit actual native waiting episodes independently from the mission policy."""
import argparse
import json
import math
from pathlib import Path

from usv_sim.map_origin import local_xy


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def analyze(run):
    run=Path(run);scene=json.loads((run/'generated/scene.json').read_text())
    requests=read_rows(run/'task1_stationkeeping.jsonl')
    events=read_rows(run/'task1_mission.jsonl')
    rows=[r for r in read_rows(run/'task1.jsonl') if 'truth' in r and r.get('phase')!='cleanup']
    episodes=[]
    for request in requests:
        if request['request']!='LOITER':continue
        release=next((r for r in requests if r['wall']>request['wall'] and r['request']!='LOITER'),None)
        end=release['wall'] if release else math.inf
        samples=[r for r in rows if request['wall']<=r['wall']<end
                 and r.get('fcu_state',{}).get('mode')=='LOITER'
                 and r['fcu_state'].get('wall_age_s',math.inf)<1.1]
        targets=[];seen=set()
        for r in samples:
            target=r.get('POSITION_TARGET_GLOBAL_INT')
            if (target and r['wall']>=samples[0]['wall']+1.
                    and target['time_boot_ms'] not in seen):
                targets.append(local_xy(target['lat_int']/1e7,target['lon_int']/1e7,*scene['origin'][:2]))
                seen.add(target['time_boot_ms'])
        fixed=max((math.dist(p,targets[0]) for p in targets),default=None)
        errors=[math.dist(r['truth'][:2],targets[0]) for r in samples] if targets else []
        during=[e for e in events if request['wall']<=e['wall']<end]
        dispatched=any(e['event'] in ('path_ready','execution_started') for e in during)
        item=dict(request_sim=request['sim'],confirmed=bool(samples),
                  observed_hold_duration_s=samples[-1]['sim']-samples[0]['sim'] if samples else 0.,
                  target_samples=len(targets),target_change_max_m=fixed,
                  target_map_xy_m=targets[0] if targets else None,
                  truth_error_max_m=max(errors) if errors else None,
                  navigation_dispatched_before_release=dispatched,
                  release_request=release['request'] if release else None,
                  release_reason=release['reason'] if release else None)
        item['passed']=bool(samples and len(targets)>=2 and fixed<=.03 and not dispatched)
        if release and release['request']=='GUIDED':
            next_hold=next((r['wall'] for r in requests if r['wall']>release['wall'] and r['request']=='LOITER'),math.inf)
            after=[e for e in events if release['wall']<=e['wall']<next_hold]
            confirmed=next((e for e in after if e['event']=='station_guided_confirmed'),None)
            dispatch=next((e for e in after if e['event']=='path_ready'),None)
            planning=next((e for e in after if confirmed and e['wall']>=confirmed['wall'] and e['event']=='planning'),None)
            item['guided_confirmed_sim']=confirmed['sim'] if confirmed else None
            item['new_plan_request_sim']=planning['sim'] if planning else None
            item['new_path_dispatched_sim']=dispatch['sim'] if dispatch else None
            item['fresh_plan_after_guided']=bool(confirmed and planning and
                                                (dispatch is None or planning['wall']<=dispatch['wall']))
            item['passed'] &= item['fresh_plan_after_guided']
        episodes.append(item)
    result=dict(exercised=bool(episodes),passed=bool(episodes) and all(e['passed'] for e in episodes),
                status='measured' if episodes else 'not_exercised',episodes=episodes,
                criteria='Actual LOITER confirmation, fixed FCU target, no FollowPath dispatch during hold; on normal release, GUIDED confirmation and a new planning request precede dispatch.',
                note='Truth error, task rules, collisions and source health are separate measurements; a mode audit does not replace them.')
    (run/'task1_stationkeeping_result.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
    result=analyze(a.run);print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
