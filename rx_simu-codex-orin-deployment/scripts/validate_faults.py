#!/usr/bin/env python3
"""Drop/corrupt individual real inputs to a single active navigation bridge."""
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
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from validate import Runner, ROOT


def stopping_observation(rows, start):
    """Keep command acknowledgement, actuator zero and physical rest separate."""
    def first(predicate):
        return next((r['wall']-start for r in rows if predicate(r)),None)
    return {'observed_duration_s':rows[-1]['wall']-start,
            'thrust_measurement':rows[-1].get('thrust_source','command_topic_only'),
            'initial_velocity':[rows[0]['truth'][4],rows[0]['truth'][6]],
            'zero_thrust_delay_s':first(lambda r:abs(r.get('left_n',999))+abs(r.get('right_n',999))<1e-7),
            'physical_rest_delay_s':first(lambda r:math.hypot(*r['truth'][4:6])<.02 and abs(r['truth'][6])<.02),
            'travel_distance_m':sum(math.dist(a['truth'][:2],b['truth'][:2]) for a,b in zip(rows,rows[1:])),
            'final_thrust_n':[rows[-1].get('left_n'),rows[-1].get('right_n')]}


def main():
    rclpy.init();runner=Runner('fault_inputs');process=None;results=[]
    try:
        runner.ready();runner.navigation_ready()
        session=json.loads((ROOT/'runs/session.json').read_text())
        launch=next(p for p in reversed(session['processes']) if p['name']=='navigation')
        # Find only this managed launch's bridge, leaving EKF/TF/physics alive.
        children=subprocess.check_output(['ps','-u',str(os.getuid()),'-o','pid=,pgid=,args='],text=True)
        for line in children.splitlines():
            pid,pgid,command=line.strip().split(maxsplit=2)
            if int(pgid)==launch['pid'] and '/lib/usv_nav2/rover_bridge ' in command:
                os.kill(int(pid),signal.SIGINT)
        runner.spin(1)
        publishers={topic:runner.node.create_publisher(cls,'/fault/'+name,qos_profile_sensor_data)
                    for topic,cls,name in [('/scan',LaserScan,'scan'),('/mavros/local_position/odom',Odometry,'odom'),('/mavros/state',State,'state')]}
        fault=[None];original=runner.receive;latest_output=[None]
        def receive(topic,msg):
            original(topic,msg)
            if topic in publishers:
                name={'/scan':'scan','/mavros/local_position/odom':'odom','/mavros/state':'state'}[topic]
                if fault[0]==name:return
                if topic=='/scan' and fault[0] in ('all_nan','stale_scan','tf_missing'):
                    msg=copy.deepcopy(msg)
                    if fault[0]=='all_nan':msg.ranges=[math.nan]*len(msg.ranges)
                    elif fault[0]=='stale_scan':msg.header.stamp.sec-=2
                    else:msg.header.frame_id='deliberately_missing_sensor'
                publishers[topic].publish(msg)
        runner.receive=receive
        command=runner.node.create_publisher(Twist,'/fault/command',10)
        runner.node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',lambda m:latest_output.__setitem__(0,(time.monotonic(),m.velocity.x,m.yaw_rate)),10)
        process=subprocess.Popen(['ros2','run','usv_nav2','rover_bridge','--ros-args',
            '-r','__node:=fault_bridge','-p','use_sim_time:=true','-p','enable_control:=true',
            '-p','command_topic:=/fault/command','-p','scan_topic:=/fault/scan',
            '-p','expected_parent_frame:='+runner.data['/mavros/local_position/odom'].header.frame_id,
            '-p','odom_topic:=/fault/odom','-p','state_topic:=/fault/state'],
            start_new_session=True,stdout=(runner.run/'fault_bridge.log').open('w'),stderr=subprocess.STDOUT)
        runner.spin(2);runner.arm()
        for name in ['command','odom','scan','state','all_nan','stale_scan','tf_missing']:
            fault[0]=None;runner.phase='healthy_before_'+name
            deadline=time.monotonic()+6
            while time.monotonic()<deadline:
                msg=Twist();msg.angular.z=.15;command.publish(msg);runner.spin(.05)
                if latest_output[0] and abs(latest_output[0][2])>.1 and time.monotonic()-latest_output[0][0]<.1:break
            if not latest_output[0] or abs(latest_output[0][2])<.1:raise RuntimeError('Could not establish nonzero healthy control before '+name)
            # Let actual yaw motion build before dropping the input.
            for _ in range(40):
                msg=Twist();msg.angular.z=.15;command.publish(msg);runner.spin(.05)
            runner.phase='fault_'+name;fault[0]=name;start=time.monotonic();zero=None
            budget=2.1 if name=='state' else .6
            while time.monotonic()-start<budget+1:
                if name!='command':
                    msg=Twist();msg.angular.z=.15;command.publish(msg)
                runner.spin(.02)
                output=latest_output[0]
                if output and output[0]>=start and abs(output[1])+abs(output[2])<1e-7:
                    zero=output[0]-start;break
            results.append({'fault':name,'zero_output_wall_s':zero,'budget_s':budget,'passed':zero is not None and zero<=budget})
            # Keep the fault active while measuring actual angular deceleration.
            runner.spin(2)
            truth=runner.data['/usv/ground_truth']
            results[-1]['yaw_rate_after_2s']=truth.twist.twist.angular.z
            results[-1]['thrust_after_2s']=[runner.rows[-1][side+'_n'] for side in ('left','right')]
            results[-1]['stopping']=stopping_observation(
                [r for r in runner.rows if r['phase']=='fault_'+name and 'truth' in r],start)
        fault[0]=None
        runner.phase='before_bridge_exit'
        for _ in range(50):
            msg=Twist();msg.angular.z=.15;command.publish(msg);runner.spin(.05)
        before=runner.data['/usv/ground_truth'].pose.pose.position
        start_xy=[before.x,before.y]
        start=time.monotonic();before_sim=runner.rows[-1]['sim'];runner.phase='bridge_exited'
        os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5);process=None
        runner.spin(10)
        after=runner.data['/usv/ground_truth']
        results.append({'fault':'bridge_process_killed','observation_s':10,
                        'physical_simulation_continued':runner.rows[-1]['sim']-before_sim>5,
                        'drift_m':math.hypot(after.pose.pose.position.x-start_xy[0],after.pose.pose.position.y-start_xy[1]),
                        'final_yaw_rate':after.twist.twist.angular.z,
                        'final_thrust':[runner.rows[-1][side+'_n'] for side in ('left','right')],
                        'stopping':stopping_observation([r for r in runner.rows if r['phase']=='bridge_exited' and 'truth' in r],start)})
        results[-1]['passed']=results[-1]['physical_simulation_continued'] and all(abs(x)<1e-7 for x in results[-1]['final_thrust'])
        (runner.run/'fault_inputs_result.json').write_text(json.dumps(results,indent=2)+'\n')
        print(json.dumps(results,indent=2))
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT);process.wait(timeout=5)
        runner.close();rclpy.shutdown()
    if not all(r['passed'] for r in results):raise SystemExit(1)


if __name__=='__main__':main()
