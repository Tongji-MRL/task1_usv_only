#!/usr/bin/env python3
"""Mode ownership/failure fixture using real MAVROS/FCU/VRX and source data.

This deliberately substitutes the mission lease producer and input relays. It
tests the bridge contract, not a complete autonomous Task1 mission. No truth
position or evaluator thrust is ever supplied to the controller.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from robotx_task1_interfaces.msg import MotionPermission
from pymavlink import mavutil

from validate import Runner, ROOT
from validate_faults import stopping_observation
from validate_environment import gz_service


class StationFixture(Runner):
    def __init__(self):
        super().__init__('stationkeeping')
        self.session=json.loads((ROOT/'runs/session.json').read_text())
        if not self.scene.get('task1',{}).get('native_loiter'):
            raise RuntimeError('Start with --task1 --task1-loiter --control')
        self.fault=None;self.action='resume';self.allow=False;self.process=None
        self.requests=[];self.results=[]
        self.mode_client=self.node.create_client(SetMode,'/mavros/set_mode')
        self.pub=self.node.create_publisher(MotionPermission,'/task1/motion_permission',10)
        self.relays={t:self.node.create_publisher(c,'/station_fixture/'+n,qos_profile_sensor_data)
                     for t,c,n in [('/scan',LaserScan,'scan'),('/mavros/local_position/odom',Odometry,'odom'),
                                   ('/mavros/state',State,'state')]}
        self.node.create_subscription(String,'/usv/stationkeeping_status',
                                      lambda m:self.requests.append(json.loads(m.data)),10)
        self.timer=self.node.create_timer(.05,self.lease)

    def lease(self):
        if self.fault=='permission':return
        m=MotionPermission();m.header.stamp=self.node.get_clock().now().to_msg();m.header.frame_id='map'
        m.revision=1;m.allow=self.allow;m.reason='explicit station-keeping integration fixture'
        m.station_action=self.action;self.pub.publish(m)

    def receive(self,topic,msg):
        super().receive(topic,msg)
        if not hasattr(self,'relays') or topic not in self.relays:return
        kind={'/scan':'scan','/mavros/local_position/odom':'odom','/mavros/state':'state'}[topic]
        if self.fault==kind:return
        if kind=='scan' and self.fault in ('all_nan','stale_scan','missing_tf'):
            msg=copy.deepcopy(msg)
            if self.fault=='all_nan':msg.ranges=[math.nan]*len(msg.ranges)
            elif self.fault=='stale_scan':msg.header.stamp.sec-=2
            else:msg.header.frame_id='fixture_missing_frame'
        if kind=='odom' and self.fault=='excursion':
            msg=copy.deepcopy(msg);msg.pose.pose.position.x+=1.0
        self.relays[topic].publish(msg)

    def detach_mission_bridge(self):
        for p in self.session['processes']:
            if p['name']=='task1_mission':os.killpg(p['pid'],signal.SIGINT)
        launch=next(p for p in self.session['processes'] if p['name']=='navigation')
        lines=subprocess.check_output(['ps','-u',str(os.getuid()),'-o','pid=,pgid=,args='],text=True)
        for line in lines.splitlines():
            pid,group,command=line.strip().split(maxsplit=2)
            if int(group)==launch['pid'] and '/lib/usv_nav2/rover_bridge ' in command:
                os.kill(int(pid),signal.SIGINT)
        self.spin(1.)

    def stop_bridge(self):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            self.process.wait(timeout=5)
        self.process=None

    def new_bridge(self,label):
        self.mode('HOLD');self.mav.mav.command_long_send(1,1,400,0,0,0,0,0,0,0,0);self.spin(.6)
        self.stop_bridge();self.fault=None;self.action='resume';self.allow=False
        command=['ros2','run','usv_nav2','rover_bridge','--ros-args','-r','__node:=station_fixture_bridge',
                 '-p','use_sim_time:=true','-p','enable_control:=true','-p','require_task_permission:=true',
                 '-p','enable_stationkeeping:=true','-p','expected_parent_frame:='+self.data['/mavros/local_position/odom'].header.frame_id]
        for name in ('scan','odom','state'):command+=['-p',name+'_topic:=/station_fixture/'+name]
        self.process=subprocess.Popen(command,start_new_session=True,
            stdout=(self.run/('station_bridge_'+label+'.log')).open('w'),stderr=subprocess.STDOUT)
        self.spin(3.);self.arm();self.spin(1.)
        if self.process.poll() is not None:raise RuntimeError('Station bridge exited')

    def mode(self,name):
        if not self.mode_client.wait_for_service(timeout_sec=4):raise RuntimeError('MAVROS mode service unavailable')
        start=time.monotonic();future=self.mode_client.call_async(SetMode.Request(custom_mode=name))
        self.wait(future.done,5,'mode service')
        if not future.result().mode_sent:raise RuntimeError('Mode not sent')
        self.wait(lambda:self.data['/mavros/state'].mode==name and self.received['/mavros/state']>=start,
                  6,'mode acknowledgement '+name)

    def enter(self):
        self.phase='enter_loiter';self.action='hold';start=time.monotonic()
        self.wait(lambda:self.data['/mavros/state'].mode=='LOITER' and self.received['/mavros/state']>=start,
                  8,'automatic LOITER entry')
        self.spin(3.)
        if self.data['/mavros/state'].mode!='LOITER':raise RuntimeError('LOITER not retained before fault')

    def run_case(self,name):
        self.new_bridge(name);self.enter();self.phase=name
        start=time.monotonic();begin=len(self.rows);frame=self.rows[-1]['frame']
        expected='HOLD';budget=.6
        if name=='resume':self.action='resume';expected='GUIDED'
        elif name=='cancel':self.action='stop'
        elif name=='manual':self.mode('MANUAL');expected='MANUAL';budget=None
        elif name=='disarm':
            self.mav.mav.command_long_send(1,1,400,0,0,0,0,0,0,0,0);expected='LOITER';budget=None
        elif name=='bridge_exit':os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(timeout=5);budget=None
        elif name=='mavros_exit':
            p=next(p for p in self.session['processes'] if p['name']=='mavros')
            os.killpg(p['pid'],signal.SIGKILL);budget=None
        elif name=='cloud_exit':
            p=next(p for p in self.session['processes'] if p['name']=='cloud_filter')
            os.killpg(p['pid'],signal.SIGKILL)
        elif name=='pause':
            budget=None  # Physics pause is not a sensor-drop latency clock.
            gz_service('/world/usv_nav/control','gz.msgs.WorldControl','pause: true')
            self.spin(1.2)
            gz_service('/world/usv_nav/control','gz.msgs.WorldControl','pause: false')
            resumed=time.monotonic()
        else:self.fault=name;budget=2.1 if name=='state' else .6
        self.spin(7.)
        rows=self.rows[begin:];requests=[r for r in self.requests if r['wall']>=start]
        mode_samples=[r for r in rows if r.get('HEARTBEAT',{}).get('custom_mode')==
                      {'HOLD':4,'GUIDED':15,'MANUAL':0,'LOITER':5}[expected]]
        request_delay=next((r['wall']-start for r in requests if r['request']==expected),None)
        out=dict(case=name,expected_mode=expected,mode_request_delay_s=request_delay,request_budget_s=budget,
                 mode_observed_delay_s=mode_samples[0]['wall']-start if mode_samples else None,
                 observed_final_mode_id=rows[-1].get('HEARTBEAT',{}).get('custom_mode'),
                 frame_advance=rows[-1]['frame']-frame,collision_messages=len(self.contacts),
                 stopping=stopping_observation(rows,start),requests=requests)
        out['passed']=bool(mode_samples and out['frame_advance']>500 and not self.contacts)
        if budget is not None:out['passed'] &= request_delay is not None and request_delay<=budget
        if name=='pause':
            out['hold_observed_after_resume_s']=next((q['wall']-resumed for q in mode_samples if q['wall']>=resumed),None)
            out['passed'] &= (out['hold_observed_after_resume_s'] is not None
                              and out['hold_observed_after_resume_s']<2.1)
        if name=='resume':
            out['cached_command_replayed']=any(sum(map(abs,r.get('preview',[0,0])))>1e-6 for r in rows)
            out['passed'] &= not out['cached_command_replayed']
            self.allow=True;self.spin(.15);self.phase='fresh_command_after_resume'
            for _ in range(30):self.command(.1,0.);self.spin(.05)
            out['fresh_command_accepted']=self.rows[-1].get('preview',[0])[0]>.05
            out['passed'] &= out['fresh_command_accepted']
            self.allow=False;self.spin(.7)
        elif name=='manual':out['passed'] &= not requests
        else:
            out['passed'] &= all(abs(x)<1e-6 for x in out['stopping']['final_thrust_n'])
        if name=='disarm':out['passed'] &= not rows[-1]['fcu_state']['armed']
        self.results.append(out)
        (self.run/'stationkeeping_result.json').write_text(json.dumps(self.results,indent=2)+'\n')
        print(json.dumps(out),flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',action='append',choices=(
        'resume','cancel','permission','scan','odom','state','all_nan','stale_scan','missing_tf','pause',
        'manual','disarm','excursion','bridge_exit','mavros_exit','cloud_exit'));a=p.parse_args()
    rclpy.init();r=StationFixture()
    try:
        r.ready();r.navigation_ready();r.detach_mission_bridge()
        for name in a.case or ('resume','cancel','permission','scan','odom','state','all_nan',
                              'stale_scan','missing_tf','pause','manual','disarm','excursion','bridge_exit','mavros_exit'):
            r.run_case(name)
        if not all(c['passed'] for c in r.results):raise RuntimeError('Station-keeping checks failed; retained evidence')
    finally:
        r.stop_bridge();r.close();rclpy.try_shutdown()


if __name__=='__main__':main()
