#!/usr/bin/env python3
"""Integration checks that keep Gazebo and ArduRover running during failures."""
import argparse
import json
import math
import os
import signal
import socket
import struct
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from mavros_msgs.msg import OverrideRCIn
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from validate import ROOT, Runner, yaw


def gz_service(path, request_type, request):
    result=subprocess.run(['gz','service','-s',path,'--reqtype',request_type,
                           '--reptype','gz.msgs.Boolean','--timeout','5000','--req',request],
                          capture_output=True,text=True,timeout=8)
    if result.returncode or 'true' not in result.stdout:
        raise RuntimeError(result.stdout+result.stderr)


def move(runner,x,y):
    gz_service('/world/usv_nav/set_pose','gz.msgs.Pose',
               f'name: "dynamic_obstacle" position {{ x: {x} y: {y} z: 1.5 }} orientation {{ w: 1 }}')
    next(obj for obj in runner.scene['obstacles'] if obj['name']=='dynamic_obstacle').update(x=x,y=y)
    runner.log.write(json.dumps({'event':'move_dynamic_obstacle','wall':time.monotonic(),
                                'sim':runner.node.get_clock().now().nanoseconds/1e9,'position':[x,y]})+'\n')


def dynamic_map(runner):
    latest=[None]
    runner.node.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda msg:latest.__setitem__(0,msg),
                                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    runner.wait(lambda:latest[0] is not None,10,'No global costmap')
    def occupied():
        m=latest[0];r=m.info.resolution;o=m.info.origin.position
        a=np.asarray(m.data).reshape(m.info.height,m.info.width)
        # Include the visible near faces of the 2 m box, not just its occluded centre.
        return int(np.count_nonzero(a[int((2.8-o.y)/r):int((5.2-o.y)/r)+1,
                                      int((2.8-o.x)/r):int((5.2-o.x)/r)+1]>=99))
    runner.phase='dynamic_baseline';runner.spin(3);before=occupied()
    move(runner,4,4);runner.phase='dynamic_appears';start=time.monotonic()
    runner.wait(lambda:occupied()>0,10,'Moved obstacle not marked in costmap')
    marked=occupied();mark_time=time.monotonic()-start;runner.spin(3)
    move(runner,15,20);runner.phase='dynamic_removed';start=time.monotonic()
    runner.wait(lambda:occupied()==0,15,'Removed obstacle still marked')
    return {'baseline_lethal_cells':before,'marked_lethal_cells':marked,'mark_delay_s':mark_time,
            'clear_delay_s':time.monotonic()-start,'remaining_lethal_cells':occupied(),
            'passed':before==0 and marked>0 and occupied()==0}


def pause(runner):
    runner.arm();runner.phase='before_pause'
    for _ in range(60):runner.command(0,.12);runner.spin(.05)
    runner.phase='paused';before=runner.rows[-1]
    gz_service('/world/usv_nav/control','gz.msgs.WorldControl','pause: true')
    start=time.monotonic();zero=None
    while time.monotonic()-start<2:
        runner.spin(.02)
        preview=runner.data['/usv/setpoint_preview']
        if abs(preview.velocity.x)+abs(preview.yaw_rate)<1e-7 and zero is None:zero=time.monotonic()-start
    frozen=runner.rows[-1]
    gz_service('/world/usv_nav/control','gz.msgs.WorldControl','pause: false')
    runner.phase='resumed_without_command';runner.spin(4)
    rows=[r for r in runner.rows if r['phase']=='resumed_without_command']
    replay=any(abs(r.get('preview',[0,0])[1])>.01 for r in rows)
    return {'zero_output_wall_s':zero,'paused_sim_delta_s':frozen['sim']-before['sim'],
            'resumed_sim_delta_s':runner.rows[-1]['sim']-frozen['sim'],'replayed_old_command':replay,
            'passed':zero is not None and zero<=.6 and not replay
                     and frozen['sim']-before['sim']<.5 and runner.rows[-1]['sim']-frozen['sim']>2,
            'note':'Paused physics is not evidence of physical stopping.'}


def mavros_exit(runner,process_name='mavros'):
    runner.arm();runner.phase='before_mavros_exit'
    for _ in range(100):runner.command(.2,0);runner.spin(.05)
    session=json.loads((ROOT/'runs/session.json').read_text())
    process=next(p for p in session['processes'] if p['name']==process_name)
    if process_name=='sitl':runner.mav_available=False
    before=runner.rows[-1];start=time.monotonic();os.killpg(process['pid'],signal.SIGKILL)
    runner.phase='mavros_exited';runner.spin(12)
    rows=[r for r in runner.rows if r['phase']=='mavros_exited']
    zero=next((r['wall']-start for r in rows if abs(r.get('left_n',999))+abs(r.get('right_n',999))<1e-7),None)
    stopped=next((r['wall']-start for r in rows if math.hypot(*r['truth'][4:6])<.02 and abs(r['truth'][6])<.02),None)
    after=runner.rows[-1]
    mode=runner.telemetry.get('HEARTBEAT',{}).get('custom_mode')
    active=abs(before.get('left_n',0))+abs(before.get('right_n',0))>1
    return {'initial_thrust_n':[before.get('left_n'),before.get('right_n')],
            'initial_forward_velocity_mps':before['truth'][4],
            'initially_active':active,
            'thrust_measurement':after.get('thrust_source','command_topic_only'),
            'thrust_zero_wall_s':zero,'speed_below_002_wall_s':stopped,
            'displacement_m':math.dist(before['truth'][:2],after['truth'][:2]),
            'sim_advanced_s':after['sim']-before['sim'],'final_thrust_n':[after['left_n'],after['right_n']],
            'final_native_mode':mode,
            'passed':active and zero is not None and after['sim']-before['sim']>8 and mode==4
                     and after.get('thrust_source')=='thruster_feedback'
                     and abs(after['left_n'])+abs(after['right_n'])<1e-6}


def sitl_exit(runner):
    result=mavros_exit(runner,'sitl')
    result['passed']=result['initially_active'] and result['thrust_zero_wall_s'] is not None and result['sim_advanced_s']>8 \
                     and result['thrust_measurement']=='thruster_feedback' \
                     and all(abs(force)<1e-7 for force in result['final_thrust_n'])
    result['note']='Tests plugin command clearing with continuing external physics.'
    return result


def gnss(runner):
    client=runner.node.create_client(SetParameters,'/gnss_injector/set_parameters')
    if not client.wait_for_service(timeout_sec=10):raise RuntimeError('Start with --gps-input')
    runner.arm();results=[]
    goal=NavigateToPose.Goal();goal.pose=PoseStamped();goal.pose.header.frame_id='map'
    goal.pose.header.stamp=runner.node.get_clock().now().to_msg();goal.pose.pose.position.y=-25.
    goal.pose.pose.orientation.z=-math.sqrt(.5);goal.pose.pose.orientation.w=math.sqrt(.5)
    if not runner.single.wait_for_server(timeout_sec=10):raise RuntimeError('Nav2 action unavailable')
    request=runner.single.send_goal_async(goal);runner.wait(request.done,10,'GNSS test goal acknowledgement')
    handle=request.result()
    if not handle.accepted:raise RuntimeError('GNSS test goal rejected')
    action=handle.get_result_async()
    profiles=[('fixed',6,.02,.02,.1,False,8),('float',5,.3,.3,.1,False,12),
              ('ordinary',3,1.5,1.5,.3,False,15),('no_fix',1,1.5,1.5,.1,False,12),
              ('lost',1,1.5,1.5,.3,True,15),
              ('recover',6,.02,.02,.1,False,20)]
    for name,fix,sigma,accuracy,delay,drop,duration in profiles:
        values=dict(fix_type=fix,horizontal_sigma=sigma,reported_accuracy=accuracy,delay_s=delay,drop=drop)
        request=SetParameters.Request(parameters=[Parameter(key,value=value).to_parameter_msg() for key,value in values.items()])
        future=client.call_async(request);runner.wait(future.done,5,'GNSS profile update')
        if not all(item.successful for item in future.result().results):raise RuntimeError(str(future.result()))
        runner.phase='gnss_'+name;start=len(runner.rows);end=time.monotonic()+duration
        while time.monotonic()<end:
            # Nav2 owns the command stream throughout the quality changes.
            runner.spin(.05)
        rows=runner.rows[start:]
        errors=[math.dist(r['truth'][:2],r['ekf'][:2]) for r in rows if 'truth' in r and 'ekf' in r]
        results.append({'profile':name,'settings':values,'position_rmse_m':float(np.sqrt(np.mean(np.square(errors)))),
                        'position_error_max_m':max(errors),'last_gate':rows[-1].get('gate'),
                        'nonzero_preview_samples':sum(abs(r.get('preview',[0,0])[0])>.01 for r in rows),
                        'samples':len(rows),'reported_fix_types':sorted(set(r.get('GPS_RAW_INT',{}).get('fix_type',-1) for r in rows)),
                        'last_ekf_status':rows[-1].get('EKF_STATUS_REPORT'),
                        'nav2_action_status':action.result().status if action.done() else 'running',
                        'minimum_clearance_m':min(r['clearance_m'] for r in rows)})
    status=action.result().status if action.done() else 'running'
    if not action.done():
        handle.cancel_goal_async();runner.wait(action.done,10,'GNSS test cancellation')
    position=runner.data['/usv/ground_truth'].pose.pose.position
    final_error=math.hypot(position.x,position.y+25)
    # Continue the same physics/FCU run across the legacy 16-bit frame boundary.
    # End navigation first; this is a protocol endurance check, not station keeping.
    runner.mav.mav.set_mode_send(1,1,4);runner.spin(.5)
    runner.mav.mav.command_long_send(1,1,400,0,0,0,0,0,0,0,0);runner.spin(1)
    runner.phase='endurance_after_gnss'
    initial_resets=(runner.run/'gazebo.log').read_text().count('ArduPilot controller has reset')
    runner.wait(lambda:runner.data.get('/usv/sitl_frame') is not None and runner.data['/usv/sitl_frame'].data>=70000,
                210,'Did not cross the 16-bit frame boundary with live physics')
    extra_resets=(runner.run/'gazebo.log').read_text().count('ArduPilot controller has reset')-initial_resets
    final=runner.rows[-1]
    fresh=all(-1e-6<=final.get(key,999)<.5 for key in ('ekf_age','scan_age'))
    return {'profiles':results,'action_status_before_test_cleanup':status,
            'target':[0,-25,-math.pi/2],'final_true_position_error_m':final_error,
            'collision_messages':len(runner.contacts),
            'endurance':{'final_frame':runner.data['/usv/sitl_frame'].data,'new_controller_resets':extra_resets,
                         'final_odom_age_s':final.get('ekf_age'),'final_scan_age_s':final.get('scan_age'),
                         'passed':extra_resets==0 and fresh},
            'note':'No new RTK/EKF gate or automatic retry policy is implemented.'}


def motors(runner):
    # Standard ArduRover motor-test command, not direct Gazebo actuator control.
    runner.arm();runner.mav.mav.set_mode_send(1,1,4);runner.spin(1);results=[]
    for side,number in [('left',3),('right',4)]:
        for pwm in (1600,1400,1500):
            runner.phase=f'motor_{side}_{pwm}'
            runner.mav.mav.command_long_send(1,1,209,0,number,1,pwm,3,0,0,0)
            runner.spin(2)
            row=runner.rows[-1];expected=(pwm-1500)*.6 if 1000<=pwm<=2000 else 0
            actual=row['left_n' if side=='left' else 'right_n']
            results.append({'side':side,'pwm':pwm,'expected_n':expected,'actual_n':actual,
                            'thrust_measurement':row.get('thrust_source','command_topic_only'),
                            'other_n':row['right_n' if side=='left' else 'left_n'],'truth_yaw_rate':row['truth'][6],
                            'passed':abs(actual-expected)<1e-6 and row.get('thrust_source')=='thruster_feedback'})
            runner.phase='motor_timeout';runner.spin(3)
    # Exercise both motors together through the FCU's normal manual RC mapping.
    # The navigation bridge yields control outside GUIDED.
    runner.mav.mav.set_mode_send(1,1,0);runner.spin(1)
    override=runner.node.create_publisher(OverrideRCIn,'/mavros/rc/override',10)
    message=OverrideRCIn();message.channels=[65535]*18;message.channels[0]=1500;message.channels[2]=1500
    override.publish(message)
    runner.mav.mav.command_long_send(1,1,400,0,1,0,0,0,0,0,0);runner.spin(2)
    for pwm in (1600,1400,1500):
        runner.phase=f'both_motors_rc_{pwm}';end=time.monotonic()+3
        while time.monotonic()<end:
            message.channels[2]=pwm;override.publish(message)
            runner.spin(.05)
        row=runner.rows[-1];sign=(pwm>1500)-(pwm<1500)
        forces=[row['left_n'],row['right_n']]
        results.append({'side':'both','rc_throttle_pwm':pwm,'actual_n':forces,'forward_velocity':row['truth'][4],
                        'thrust_measurement':row.get('thrust_source','command_topic_only'),
                        'servo_pwm':row.get('SERVO_OUTPUT_RAW'),
                        'passed':row.get('thrust_source')=='thruster_feedback' and
                                 (all(f*sign>0 for f in forces) if sign else all(abs(f)<1e-6 for f in forces))})
    message.channels=[0]*18;override.publish(message)
    return results


def protocol(runner):
    """Negative protocol fixtures, with the FCU stopped and no valid thrust commands."""
    session=json.loads((ROOT/'runs/session.json').read_text())
    process=next(p for p in session['processes'] if p['name']=='sitl')
    runner.mav_available=False;os.killpg(process['pid'],signal.SIGKILL);runner.spin(2)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',0));sock.setblocking(False)
    frame=100000;results=[]
    try:
        for case in ('out_of_range_pwm','disabled_pwm','truncated','bad_magic'):
            runner.phase=case;start=len(runner.rows);deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                frame+=1;pwm=[0]*16
                if case=='out_of_range_pwm':pwm[0]=900;pwm[2]=2100
                packet=struct.pack('<HHI16H',1 if case=='bad_magic' else 18458,250,frame,*pwm)
                if case=='truncated':packet=packet[:9]
                sock.sendto(packet,('127.0.0.1',9002))
                runner.spin(.02)
                try:
                    while sock.recv(65535):pass
                except BlockingIOError:pass
            rows=runner.rows[start:]
            maximum=max(abs(r.get('left_n',999))+abs(r.get('right_n',999)) for r in rows)
            results.append({'case':case,'maximum_total_absolute_thrust_n':maximum,
                            'thrust_measurement':rows[-1].get('thrust_source','command_topic_only'),
                            'sim_advanced_s':rows[-1]['sim']-rows[0]['sim'],
                            'passed':maximum<1e-6 and rows[-1]['sim']-rows[0]['sim']>.1
                                     and rows[-1].get('thrust_source')=='thruster_feedback'})
    finally:sock.close()
    return results


def disarm(runner):
    runner.arm();runner.phase='before_disarm'
    for _ in range(100):runner.command(.2,0);runner.spin(.05)
    before=runner.rows[-1];start=time.monotonic()
    runner.mav.mav.command_long_send(1,1,400,0,0,0,0,0,0,0,0)
    runner.phase='disarmed';runner.spin(8)
    rows=[r for r in runner.rows if r['phase']=='disarmed']
    zero=next((r['wall']-start for r in rows if abs(r.get('left_n',999))+abs(r.get('right_n',999))<1e-7),None)
    stopped=next((r['wall']-start for r in rows if math.hypot(*r['truth'][4:6])<.02 and abs(r['truth'][6])<.02),None)
    after=rows[-1]
    return {'initial_thrust_n':[before.get('left_n'),before.get('right_n')],
            'zero_thrust_delay_s':zero,'speed_below_002_delay_s':stopped,
            'drift_m':math.dist(before['truth'][:2],after['truth'][:2]),
            'sim_advanced_s':after['sim']-before['sim'],
            'final_thrust_n':[after.get('left_n'),after.get('right_n')],
            'final_armed':after.get('fcu_state',{}).get('armed'),
            'thrust_measurement':after.get('thrust_source','command_topic_only'),
            'passed':abs(before.get('left_n',0))+abs(before.get('right_n',0))>1 and zero is not None
                     and after.get('fcu_state',{}).get('armed') is False
                     and after.get('thrust_source')=='thruster_feedback'
                     and abs(after.get('left_n',999))+abs(after.get('right_n',999))<1e-7
                     and after['sim']-before['sim']>5}


def collision_sensor(runner):
    runner.phase='contact_positive_control'
    move(runner,0,.7);runner.spin(3)
    return {'collision_messages':len(runner.contacts),'contact_messages':runner.contact_messages,
            'passed':len(runner.contacts)>0,
            'note':'Intentional overlap in an isolated disarmed scene validates contact detection.'}


def reset(runner):
    runner.arm();runner.phase='before_reset'
    for _ in range(60):runner.command(0.,.12);runner.spin(.05)
    before=runner.rows[-1]
    active_before=sum(map(abs,before.get('preview',[0.,0.])))>.05
    gz_service('/world/usv_nav/control','gz.msgs.WorldControl','reset { all: true }')
    runner.phase='after_reset';runner.spin(12)
    rows=[r for r in runner.rows if r['phase']=='after_reset']
    latest=rows[-1]
    return {'before_sim_s':before['sim'],'minimum_sim_after_reset_s':min(r['sim'] for r in rows),
            'final_sim_s':latest['sim'],'final_odom_age_s':latest.get('ekf_age'),
            'final_scan_age_s':latest.get('scan_age'),'final_gate':latest.get('gate'),
            'nonzero_preview_samples':sum(sum(abs(v) for v in r.get('preview',[0,0]))>.001 for r in rows),
            'nonzero_command_before_reset':active_before,
            'passed':active_before and 0<=latest.get('ekf_age',999)<.5 and 0<=latest.get('scan_age',999)<.5
                and not any(sum(map(abs,r.get('preview',[0,0])))>.001 for r in rows
                            if r['sim']<before['sim'])}


def dynamic_navigation(runner):
    original=runner.spin;start=[None];moved=[False];removed=[False];last_move=[-1.]
    grid=[None]
    runner.node.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda msg:grid.__setitem__(0,msg),
                                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def spin(duration=.05):
        original(duration)
        if runner.phase!='navigation':return
        simulation_time=runner.node.get_clock().now().nanoseconds/1e9
        if start[0] is None:start[0]=simulation_time
        elapsed=simulation_time-start[0]
        if 3<elapsed<28 and elapsed-last_move[0]>.5:
            move(runner,6,7-.14*(elapsed-3));moved[0]=True;last_move[0]=elapsed
            obj=next(o for o in runner.scene['obstacles'] if o['name']=='dynamic_obstacle')
            scan=runner.data['/scan'];pose=runner.rows[-1]['ekf'];r=np.asarray(scan.ranges)
            angles=scan.angle_min+np.arange(len(r))*scan.angle_increment+pose[3]
            finite=np.isfinite(r);points=np.stack([pose[0]+r[finite]*np.cos(angles[finite]),pose[1]+r[finite]*np.sin(angles[finite])],axis=1)
            observation={'event':'dynamic_observation','wall':time.monotonic(),
                         'box_xy':[obj['x'],obj['y']],'scan_points_near_box':int(np.count_nonzero(np.linalg.norm(points-[obj['x'],obj['y']],axis=1)<1.6))}
            if grid[0] is not None:
                m=grid[0];values=np.asarray(m.data).reshape(m.info.height,m.info.width);iy,ix=np.nonzero(values>=99)
                lethal=np.stack([m.info.origin.position.x+(ix+.5)*m.info.resolution,m.info.origin.position.y+(iy+.5)*m.info.resolution],axis=1)
                observation['lethal_cells_near_box']=int(np.count_nonzero(np.linalg.norm(lethal-[obj['x'],obj['y']],axis=1)<1.6))
                if len(lethal):observation['nearest_lethal_to_box_m']=float(np.min(np.linalg.norm(lethal-[obj['x'],obj['y']],axis=1)))
            runner.log.write(json.dumps(observation)+'\n')
        if elapsed>28 and not removed[0]:move(runner,15,20);removed[0]=True
    runner.spin=spin
    target=[12,8,0]
    result=runner.goal([target])
    result.update(target=target,obstacle_inserted=moved[0],obstacle_removed=removed[0],
                  obstacle_profile={'clock':'Gazebo simulation time','appearance_s':3,'removal_s':28,
                                    'initial_xy_m':[6,7],'velocity_xy_mps':[0,-.14],
                                    'pose_update_period_s':.5})
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('case',choices=['dynamic_map','dynamic_navigation','pause','mavros_exit','sitl_exit','gnss','motors','protocol','disarm','collision_sensor','reset'])
    args=parser.parse_args();rclpy.init();runner=Runner(args.case)
    try:
        runner.ready();runner.navigation_ready()
        result=globals()[args.case](runner)
        (runner.run/(args.case+'_result.json')).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    finally:runner.close();rclpy.shutdown()
    if result.get('passed') is False:raise SystemExit(1)


if __name__=='__main__':main()
