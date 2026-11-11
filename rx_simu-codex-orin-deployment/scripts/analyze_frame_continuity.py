#!/usr/bin/env python3
"""Check a recorded real FCU/physics counter across the former 16-bit boundary."""
import argparse
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path)
    p.add_argument('--trace',default='task1.jsonl');a=p.parse_args()
    rows=[json.loads(line) for line in (a.run/a.trace).read_text().splitlines()]
    rows=[r for r in rows if 'frame' in r and 'truth' in r and 'ekf_age' in r]
    frames=[r['frame'] for r in rows];cross=next((i for i,r in enumerate(rows) if r['frame']>65535),None)
    resets=[dict(sim=b['sim'],before=a['frame'],after=b['frame']) for a,b in zip(rows,rows[1:]) if b['frame']<a['frame']]
    after=rows[cross:] if cross is not None else []
    result=dict(first_frame=frames[0],final_frame=frames[-1],counter_backwards=resets,
                cross_sim_s=rows[cross]['sim'] if cross is not None else None,
                samples_after_crossing=len(after),final_odom_age_s=rows[-1]['ekf_age'],
                final_scan_age_s=rows[-1].get('scan_age'),
                max_odom_age_after_crossing_s=max((r['ekf_age'] for r in after),default=None),
                max_scan_age_after_crossing_s=max((r.get('scan_age',999) for r in after),default=None),
                firmware_revision=json.loads((a.run/'provenance.json').read_text())['firmware_revision'])
    result['passed']=(frames[0]<65535<frames[-1] and not resets and len(after)>10
                       and 0<=result['final_odom_age_s']<.5 and 0<=result['final_scan_age_s']<.5)
    (a.run/'protocol_continuity.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
