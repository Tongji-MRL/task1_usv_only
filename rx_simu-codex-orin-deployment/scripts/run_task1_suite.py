#!/usr/bin/env python3
"""Cold-start Task1 cases; keep source snapshots, failures and independent results."""
import argparse
import datetime
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
from task1_cases import cases as selected_cases,case_key,FAULTS

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--group',choices=('base','dynamic','all'),default='base')
    p.add_argument('--seed',type=int,choices=(1,2,3));p.add_argument('--changes',type=int,choices=(0,1,2))
    p.add_argument('--dynamic',choices=('none','cross','block'));p.add_argument('--record-maps',action='store_true')
    p.add_argument('--course',choices=('baseline','harbor'),default='baseline')
    p.add_argument('--wind',type=float,default=0.);p.add_argument('--wave-gain',type=float,default=0.)
    p.add_argument('--lidar',choices=('vrx','mid360'),default='vrx')
    p.add_argument('--mid360-mount',choices=('mast','bow','low_bow'),default='mast')
    p.add_argument('--task1-loiter',action='store_true')
    p.add_argument('--record-sensors',action='store_true',help='Retain source clouds, scan, TF and odometry in a rosbag')
    p.add_argument('--visibility-guard',action='store_true',help='Test the experimental lidar stand-off separately')
    p.add_argument('--stress',action='store_true',help='Run two-change stress cases outside the confirmed match setting')
    p.add_argument('--skip-case',action='append',default=[],help='Omit an already recorded case ID, e.g. s1_c0_none')
    p.add_argument('--fault',choices=FAULTS)
    p.add_argument('--stop-on-failure',action='store_true')
    p.add_argument('--no-video',action='store_true');a=p.parse_args()
    if (ROOT/'runs/session.json').exists():raise RuntimeError('Stop the current managed session before running a cold-start suite')
    if a.changes==2 and not a.stress:p.error('--changes 2 requires --stress; competition cases change once')
    if a.stress and (a.fault or a.changes in (0,1)):p.error('--stress is only for two-change normal tasks')
    if a.course=='harbor' and a.stress:p.error('Harbor cases follow the one-change match constraint')
    cases=selected_cases(a.group,a.stress)
    cases=[c for c in cases if all(getattr(a,k) is None or c[k]==getattr(a,k) for k in ('seed','changes','dynamic'))]
    if a.fault:cases=[dict(seed=1,changes=0,dynamic='none',fault=a.fault)]
    for case in cases:case.update(course=a.course,wind=a.wind,wave_gain=a.wave_gain,
                                 lidar=a.lidar,mid360_mount=a.mid360_mount,native_loiter=a.task1_loiter)
    known={case_key(c) for c in cases}
    if set(a.skip_case)-known:p.error('Unknown or filtered --skip-case: '+', '.join(sorted(set(a.skip_case)-known)))
    cases=[c for c in cases if case_key(c) not in a.skip_case]
    if not cases:raise ValueError('No matching cases')
    ledger=ROOT/'runs'/('task1_suite_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
    results=[]
    for case in cases:
        name='task1_'+case_key(case)
        if a.lidar=='mid360':name+='_'+a.lidar+'_'+a.mid360_mount
        if a.task1_loiter:name+='_loiter'
        if a.fault:name+='_'+a.fault
        command=[sys.executable,str(ROOT/'scripts/sim_session.py'),'start','--task1','--navigation','--control',
                 '--name',name,'--seed',str(case['seed']),'--changes',str(case['changes']),'--dynamic',case['dynamic'],
                 '--course',a.course,'--wind',str(a.wind),'--wave-gain',str(a.wave_gain)]
        command+=['--lidar',a.lidar,'--mid360-mount',a.mid360_mount]
        if a.task1_loiter:command.append('--task1-loiter')
        if a.fault=='no_path':command.append('--task1-blocked')
        if a.visibility_guard:command.append('--task1-visibility-guard')
        old=(ROOT/'runs/latest.txt').read_text().strip() if (ROOT/'runs/latest.txt').exists() else None
        supervisor=subprocess.Popen(command,stdout=(ROOT/'log'/f'{name}_supervisor.log').open('w'),stderr=subprocess.STDOUT)
        recorders=[];run=None;item={'case':case}
        try:
            deadline=time.monotonic()+90
            while time.monotonic()<deadline:
                current=(ROOT/'runs/latest.txt').read_text().strip() if (ROOT/'runs/latest.txt').exists() else None
                if current and current!=old:run=Path(current);break
                if supervisor.poll() is not None:raise RuntimeError('Simulation supervisor exited during startup')
                time.sleep(.2)
            if run is None:raise TimeoutError('Simulation startup')
            item['run_dir']=str(run);print(json.dumps(item),flush=True)
            if a.record_maps:
                recorders.append(subprocess.Popen([sys.executable,str(ROOT/'scripts/record_navigation.py'),'--run-dir',str(run)],
                                           stdout=(run/'record_navigation.log').open('w'),stderr=subprocess.STDOUT))
            if a.record_sensors:
                recorders.append(subprocess.Popen(['ros2','bag','record','--use-sim-time','--storage','sqlite3',
                    '--output',str(run/'sensors'),'/livox/lidar' if a.lidar=='mid360' else '/usv/lidar/points','/usv/lidar/filtered','/usv/lidar/marking','/usv/lidar/clearing','/scan','/tf','/tf_static',
                    '/clock','/usv/odom','/usv/cmd_vel','/task1/reference_path','/task1/traffic_safety',
                    '/task1/observations','/map','/task1/candidate_path','/task1/traffic_costmap','/task1/route_assessment',
                    '/task1/context','/task1/motion_permission','/task1/speed_limit','/usv/stationkeeping_status'],
                    stdout=(run/'record_sensors.log').open('w'),stderr=subprocess.STDOUT))
            command=[sys.executable,str(ROOT/'scripts/validate_task1.py')]
            if a.fault:command=[sys.executable,str(ROOT/'scripts/validate_task1_fault.py'),a.fault]
            elif a.no_video:command.append('--no-video')
            result=subprocess.run(command,stdout=(run/'test_console.log').open('w'),stderr=subprocess.STDOUT,timeout=2500)
            item['exit_code']=result.returncode
        except Exception as error:item['error']=repr(error)
        finally:
            owned={}
            session=ROOT/'runs/session.json'
            if run and session.exists():
                state=json.loads(session.read_text())
                if Path(state['run_dir'])==run:
                    owned=state
                    (run/'owned_processes.json').write_text(json.dumps(state,indent=2)+'\n')
            for recorder in recorders:
                recorder.send_signal(signal.SIGINT)
                try:recorder.wait(timeout=10)
                except subprocess.TimeoutExpired:recorder.terminate();recorder.wait(timeout=5)
            subprocess.run([sys.executable,str(ROOT/'scripts/sim_session.py'),'stop'],check=False)
            try:supervisor.wait(timeout=12)
            except subprocess.TimeoutExpired:supervisor.terminate();supervisor.wait(timeout=5)
            if run:
                groups={p['pid'] for p in owned.get('processes',[])}
                remaining=[line for line in subprocess.check_output(['ps','-eo','pid=,pgid=,stat='],text=True).splitlines()
                           if int(line.split()[1]) in groups and not line.split()[2].startswith('Z')]
                item['shutdown']=dict(session_removed=not session.exists(),owned_groups_alive=remaining,
                                      passed=bool(groups) and not remaining and not session.exists())
                (run/'shutdown_result.json').write_text(json.dumps(item['shutdown'],indent=2)+'\n')
        if run and (run/'task1_judge.jsonl').exists():
            from robotx_task1.judge import evaluate
            try:item['result']=evaluate(run)
            except Exception as error:item['judge_error']=repr(error)
        if a.fault and run and (run/'task1_fault_result.json').exists():
            item['fault_result']=json.loads((run/'task1_fault_result.json').read_text())
        if run and case['dynamic']!='none' and a.record_maps:
            from analyze_task1_maps import analyze
            try:item['map_result']={k:v for k,v in analyze(run).items() if k!='samples'}
            except Exception as error:item['map_result']={'passed':False,'error':repr(error)}
        results.append(item);ledger.write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(item),flush=True)
        if a.stop_on_failure and (not item.get('fault_result' if a.fault else 'result',{}).get('passed',False)
                                 or not item.get('map_result',{'passed':True})['passed']
                                 or not item.get('shutdown',{}).get('passed',False)):break
    print('Ledger:',ledger,flush=True)
    if not all(r.get('fault_result' if a.fault else 'result',{}).get('passed',False) and
               r.get('map_result',{'passed':True})['passed'] and
               r.get('shutdown',{}).get('passed',False) for r in results):raise SystemExit(1)


if __name__=='__main__':main()
