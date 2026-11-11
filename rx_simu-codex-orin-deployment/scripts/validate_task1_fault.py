#!/usr/bin/env python3
"""Inject faults into owned Task1 processes; leave SITL and physics advancing."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import time
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from robotx_task1_interfaces.msg import BuoyArray
from robotx_task1.safety_evidence import stop_evidence
from validate_task1 import TaskRunner
from sim_session import process_start, ROOT
from task1_cases import FAULTS


def main():
    p=argparse.ArgumentParser();p.add_argument('fault',choices=FAULTS)
    a=p.parse_args();rclpy.init();r=TaskRunner();report={'fault':a.fault,'passed':False};suspended=None
    try:
        r.ready();r.navigation_ready();r.arm()
        obs=[None];r.node.create_subscription(BuoyArray,'/task1/observations',lambda m:obs.__setitem__(0,m),10)
        scene_pub=r.node.create_publisher(String,'/task1/scene_command',10)
        start=r.node.create_client(Trigger,'/task1/start');r.wait(start.service_is_ready,10,'start service')
        future=start.call_async(Trigger.Request());r.wait(future.done,10,'start response')
        if not future.result().success:raise RuntimeError(future.result().message)
        if a.fault=='no_path':
            r.wait(lambda:any(e['event']=='plan_failed' for e in r.events) and r.status and
                   r.status['motion'] in ('WAIT_CLEAR','PLANNING'),45,'blocked planner must reject and remain stopped')
        else:
            r.wait(lambda:r.rows and r.rows[-1].get('preview',[0])[0]>.15 and
                   math.hypot(*r.rows[-1]['truth'][4:6])>.15,90,'moving vessel before fault')
        late=[None]
        if a.fault=='late_change':
            def approaching():
                if not obs[0] or not r.status or r.status['stage']!='TRANSIT' or r.status['motion']!='EXECUTING':return False
                p=r.rows[-1]['ekf'];c,s=math.cos(p[3]),math.sin(p[3])
                if abs(p[6])>.05 or r.rows[-1].get('preview',[0])[0]<.15:return False
                for b in obs[0].buoys:
                    dx,dy=b.position.x-p[0],b.position.y-p[1];ahead=dx*c+dy*s;side=-dx*s+dy*c
                    if 2.5<ahead<3.5 and math.hypot(dx,dy)<12 and (b.state==1 and side<0 or b.state==2 and side>0):
                        late[0]=dict(id=b.id,state=b.state,ahead_m=ahead,distance_m=math.hypot(dx,dy));return True
                return False
            r.wait(approaching,360,'late change requires an approaching classified buoy')
            report['late_target']=late[0]
        now=r.node.get_clock().now().nanoseconds/1e9
        event={'event':'stop_requested','sim':now,'reason':a.fault};report['injection']=event
        state=json.loads((ROOT/'runs/session.json').read_text())
        def signal_owned(name,sig):
            child=next(c for c in state['processes'] if c['name']==name)
            if process_start(child['pid'])!=child['start']:raise RuntimeError('Owned process identity changed')
            os.killpg(child['pid'],sig);return child
        replay=None;snapshot=copy.deepcopy(obs[0])
        if a.fault in ('observation_gap','reordered_observation'):
            suspended=signal_owned('task1_observer',signal.SIGSTOP)
            if a.fault=='reordered_observation':
                if snapshot is None:raise RuntimeError('No observation to replay with original timestamp')
                replay=r.node.create_publisher(BuoyArray,'/task1/observations',10)
        elif a.fault=='mission_exit':signal_owned('task1_mission',signal.SIGTERM)
        elif a.fault=='traffic_gap':suspended=signal_owned('traffic_guard',signal.SIGSTOP)
        elif a.fault=='traffic_exit':signal_owned('traffic_guard',signal.SIGTERM)
        elif a.fault=='cancel':
            c=r.node.create_client(Trigger,'/task1/cancel');c.call_async(Trigger.Request())
        elif a.fault=='role_change':
            scene_pub.publish(String(data=json.dumps({'op':'set_states','states':{'entry':0,'buoy_01':3}})))
        elif a.fault=='late_change':
            scene_pub.publish(String(data=json.dumps({'op':'set_states','states':{late[0]['id']:3-late[0]['state']}})))
        end=time.monotonic()+20
        while r.node.get_clock().now().nanoseconds/1e9-now<10:
            if time.monotonic()>end:raise TimeoutError('Simulation did not advance during fault')
            if replay:replay.publish(snapshot)  # Deliberately unchanged source stamp; never restamp stale data.
            r.spin(.1)
        report['stops']=stop_evidence([event]+[e for e in r.events if e['event']=='execution_started' and e['sim']>now],r.rows)
        report['status_after_fault']=r.status
        first=next(x for x in r.rows if x.get('sim',-1)>=now and 'truth' in x)
        last=r.rows[-1];report['fault_window_displacement_m']=math.dist(first['truth'][:2],last['truth'][:2])
        report['fcu_alive']=bool(last.get('fcu_state',{}).get('connected') and last['fcu_state']['wall_age_s']<2.)
        report['collision_messages']=len(r.contacts)
        if a.fault=='no_path':
            # A search longer than the retry interval can re-enter PLANNING in the same tick.
            # Check actual failed plans and motion, rather than requiring a transient status sample.
            report['plan_failures']=sum(e['event']=='plan_failed' for e in r.events)
            report['passed']=bool(report['plan_failures'] and r.status['motion'] in ('WAIT_CLEAR','PLANNING') and
                                  report['fault_window_displacement_m']<.3 and
                                  all(max(map(abs,x.get('preview',[0,0])))<1e-6 for x in r.rows if x.get('sim',-1)>=now))
        else:report['passed']=bool(report['stops'] and all(s['passed'] for s in report['stops']))
        if suspended:
            os.killpg(suspended['pid'],signal.SIGCONT);suspended=None
            r.wait(lambda:r.status and r.status['motion']=='EXECUTING' and r.status['sim']>now+10,30,'fresh observation recovery')
            report['recovered_with_fresh_input']=True
        if a.fault in ('cancel','role_change'):report['passed'] &= r.status['stage']=='ABORTED'
        if a.fault=='late_change':
            report['passed'] &= any(e['event']=='semantic_update_reported' and e['sim']>=now for e in r.events)
            report['passed'] &= not any(e['event']=='ekf_passage' and not e['legal'] and e['sim']>=now for e in r.events)
        report['passed'] &= report['fcu_alive'] and not r.contacts
    except Exception as error:report['error']=repr(error);report['passed']=False
    finally:
        if suspended:os.killpg(suspended['pid'],signal.SIGCONT)
        cancel=r.node.create_client(Trigger,'/task1/cancel')
        if cancel.service_is_ready():cancel.call_async(Trigger.Request());r.spin(.5)
        (r.run/'task1_fault_result.json').write_text(json.dumps(report,indent=2)+'\n')
        r.finish();rclpy.try_shutdown()
    print(json.dumps(report),flush=True)
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
