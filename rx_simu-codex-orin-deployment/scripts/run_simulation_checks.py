#!/usr/bin/env python3
"""Cold-start bounded physical checks and retain every failing experiment."""
import argparse
import datetime
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]


def cases(group):
    common=['--navigation','--control','--lidar','mid360','--mid360-mount','low_bow']
    if group=='sensor':
        yield 'mid360_supported_geometry',common+['--task1','--course','harbor'],[
            ['validate_lidar_geometry.py','--azimuths']]
    if group=='hold':
        for name,extra,test in [
            ('repeat1',[],[]),('repeat2',[],[]),('repeat3',[],[]),
            ('zero',[],['--mode','zero_velocity','--error-budget','10']),
            ('moving',[],['--moving-entry']),('waves',['--wave-gain','.08'],[])]:
            yield 'mid360_hold_'+name,common+['--wind','3']+extra,[
                ['validate_loiter.py','--duration','120','--radius','.3','--error-budget','.8']+test]
    if group=='gnss':
        yield 'mid360_loiter_gnss',common+['--wind','3','--gps-input'],[['validate_loiter_gnss.py']]
    if group=='station':
        yield 'stationkeeping_supported_mid360',common+['--task1','--task1-loiter','--course','harbor','--wind','3'],[
            ['validate_stationkeeping.py']]
        yield 'stationkeeping_cloud_exit_mid360',common+['--task1','--task1-loiter','--course','harbor','--wind','3'],[
            ['validate_stationkeeping.py','--case','cloud_exit']]
    if group=='timing':
        yield 'mid360_guided_input_faults',common,[['validate_faults.py']]
        for name in ('pause','reset'):
            yield 'mid360_'+name,common,[['validate_environment.py',name]]


def run(name,options,commands):
    old=(ROOT/'runs/latest.txt').read_text().strip()
    supervisor=subprocess.Popen([sys.executable,str(ROOT/'scripts/sim_session.py'),'start','--name',name]+options,
        stdout=(ROOT/'log'/f'{name}_supervisor.log').open('w'),stderr=subprocess.STDOUT)
    item={'name':name,'options':options,'commands':commands};directory=None
    try:
        deadline=time.monotonic()+90
        while time.monotonic()<deadline:
            p=ROOT/'runs/session.json'
            state=json.loads(p.read_text()) if p.exists() else {}
            current=(ROOT/'runs/latest.txt').read_text().strip()
            ready=any(p['name']=='cloud_filter' for p in state.get('processes',[]))
            if current!=old and state.get('run_dir')==current and ready:
                directory=Path(current);break
            if supervisor.poll() is not None:raise RuntimeError('Supervisor exited during startup')
            time.sleep(.2)
        if directory is None:raise TimeoutError('Simulation startup')
        item['run_dir']=str(directory);print(json.dumps(item),flush=True)
        item['exit_codes']=[]
        for command in commands:
            result=subprocess.run([sys.executable,str(ROOT/'scripts'/command[0])]+command[1:],
                stdout=(directory/('check_'+Path(command[0]).stem+'.log')).open('w'),stderr=subprocess.STDOUT,timeout=900)
            item['exit_codes'].append(result.returncode)
            if result.returncode:break
    except Exception as error:item['error']=repr(error)
    finally:
        subprocess.run([sys.executable,str(ROOT/'scripts/sim_session.py'),'stop'],check=False)
        supervisor.wait(timeout=20)
        if directory and (directory/'shutdown_audit.json').exists():
            item['shutdown']=json.loads((directory/'shutdown_audit.json').read_text())
    return item


def main():
    p=argparse.ArgumentParser();p.add_argument('group',choices=('sensor','hold','gnss','station','timing'))
    p.add_argument('--case',help='Select a case name from this group');a=p.parse_args()
    if (ROOT/'runs/session.json').exists():raise RuntimeError('An existing managed simulation is active')
    selected=[c for c in cases(a.group) if not a.case or c[0]==a.case]
    if not selected:raise ValueError('No matching check')
    ledger=ROOT/'runs'/('simulation_checks_'+a.group+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
    results=[]
    for name,options,commands in selected:
        results.append(run(name,options,commands))
        ledger.write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results[-1]),flush=True)
    print('Ledger:',ledger,flush=True)
    if any(r.get('error') or not r.get('exit_codes') or any(r['exit_codes']) for r in results):raise SystemExit(1)


if __name__=='__main__':main()
