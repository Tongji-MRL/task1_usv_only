"""Measure command, physical force and actual motion independently from recordings."""
import math


def stop_evidence(events, rows):
    rows=[r for r in rows if 'sim' in r and 'truth' in r and r.get('phase')!='cleanup']
    output=[]
    for event in events:
        if event['event']!='stop_requested':continue
        start=event['sim']
        resumed=next((e['sim'] for e in events if e['sim']>start and e['event']=='execution_started'),None)
        segment=[r for r in rows if r['sim']>=start and (resumed is None or r['sim']<resumed)]
        if not segment:continue
        zero=next((r['sim'] for r in segment if 'preview' in r and max(map(abs,r['preview']))<1e-6),None)
        force=next((r['sim'] for r in segment if r.get('thrust_source')=='thruster_feedback' and
                    max(abs(r['left_n']),abs(r['right_n']))<=1.),None)
        still_since=None;stopped=None;distance=0.;stop_distance=None;last=segment[0]
        force_since=None;force_quiet=None
        for r in segment:
            distance+=math.dist(last['truth'][:2],r['truth'][:2]);last=r
            t=r['truth']
            if r.get('truth_age',math.inf)<=.2 and math.hypot(t[4],t[5])<.05 and abs(t[6])<.05:
                if still_since is None:still_since=r['sim']
                if r['sim']-still_since>=1. and stopped is None:stopped=r['sim'];stop_distance=distance
            else:still_since=None
            if r.get('thrust_source')=='thruster_feedback' and max(abs(r['left_n']),abs(r['right_n']))<=1.:
                if force_since is None:force_since=r['sim']
                if r['sim']-force_since>=1. and force_quiet is None:force_quiet=r['sim']
            else:force_since=None
        frames=[r['frame'] for r in segment if 'frame' in r]
        advancing=len(frames)>1 and frames[-1]>frames[0] and segment[-1]['sim']-segment[0]['sim']>.5
        item=dict(reason=event.get('reason'),start_sim=start,resume_sim=resumed,
                  zero_command_delay_s=None if zero is None else zero-start,
                  first_force_below_1N_delay_s=None if force is None else force-start,
                  force_below_1N_dwell_confirmed_delay_s=None if force_quiet is None else force_quiet-start,
                  force_peak_before_resume_N=max(max(abs(r.get('left_n',0)),abs(r.get('right_n',0))) for r in segment),
                  stopped_dwell_confirmed_delay_s=None if stopped is None else stopped-start,
                  distance_to_stop_m=stop_distance,physics_advancing=advancing,
                  passed=bool(zero is not None and zero-start<=.6 and stopped is not None and advancing))
        output.append(item)
    return output
