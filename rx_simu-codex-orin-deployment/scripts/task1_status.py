#!/usr/bin/env python3
"""Read actual process identities and recent physical evidence; no ROS required."""
import json
from pathlib import Path
import time
from sim_session import ROOT, process_start


def tail_json(path,kind=None):
    if not path.exists():return None
    with path.open('rb') as stream:
        stream.seek(max(0,path.stat().st_size-65536));lines=stream.read().splitlines()
    for line in reversed(lines):
        try:
            row=json.loads(line)
            if kind is None or row.get('type')==kind:return row
        except (ValueError,UnicodeDecodeError):pass
    return None


def main():
    pidfile=ROOT/'runs/session.json';active=json.loads(pidfile.read_text()) if pidfile.exists() else None
    latest=ROOT/'runs/latest.txt'
    run=Path(active['run_dir']) if active else Path(latest.read_text().strip()) if latest.exists() else None
    processes={p['name']:process_start(p['pid'])==p['start'] for p in active['processes']} if active else {}
    output={'managed_session':active is not None,'processes_alive':processes,'run_dir':str(run) if run else None}
    if run:
        physical=run/'task1.jsonl';sample=tail_json(physical)
        console=tail_json(run/'test_console.log') or {}
        status=tail_json(run/'task1_judge.jsonl','status')
        output['task_status']=status['data'] if status else console.get('status')
        if sample:
            output['last_physics_sample']={k:sample[k] for k in ['sim','phase','frame','preview','fcu_state'] if k in sample}
            output['evidence_file_age_s']=round(time.time()-physical.stat().st_mtime,2)
            if output['task_status']:
                output['status_lag_behind_physics_s']=round(max(0.,sample['sim']-output['task_status']['sim']),3)
        result=run/'task1_result.json'
        if result.exists():
            j=json.loads(result.read_text());output['finished_result']={k:j[k] for k in ['passed','endpoint_error','circles','min_clearance_m']}
            output['finished_result']['kind']='normal_task'
        fault=run/'task1_fault_result.json'
        if fault.exists():
            j=json.loads(fault.read_text())
            output['normal_task_result']=output.pop('finished_result',None)
            output['finished_result']={k:j[k] for k in ['fault','passed','error','fcu_alive','stops'] if k in j}
            output['finished_result']['kind']='fault_response'
    ledgers=sorted((ROOT/'runs').glob('task1_suite_*.json'))
    if ledgers:
        items=json.loads(ledgers[-1].read_text());output['latest_batch_ledger']=str(ledgers[-1])
        output['recorded_completed_cases']=len(items)
        output['recorded_passed_cases']=sum(i.get('fault_result',i.get('result',{})).get('passed',False) for i in items)
    print(json.dumps(output,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
