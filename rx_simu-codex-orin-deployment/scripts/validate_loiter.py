#!/usr/bin/env python3
"""Measure native Rover 4.4 LOITER, without injecting a truth position target."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from mavros_msgs.srv import SetMode
from pymavlink import mavutil

from validate import ROOT, Runner
from usv_sim.map_origin import local_xy


class LoiterRunner(Runner):
    def __init__(self):
        super().__init__('loiter')
        session = json.loads((ROOT/'runs/session.json').read_text())
        if session['firmware_revision'] != 'd0356316eb0fa34ed1c9f39f7b059b9ae7ba4664':
            raise RuntimeError('This test only targets the pinned local SITL')
        if str(self.run) != session['run_dir']:
            raise RuntimeError('Session changed while attaching')
        if any(p['name'] == 'task1_mission' for p in session['processes']):
            raise RuntimeError('Independent LOITER test must not run alongside a Task1 mission')
        self.parameters = {}
        self.parameter_received = {}
        self.parameter_attempts = []
        self.targets = []
        self.mode_events = []
        self.mode_client = self.node.create_client(SetMode, '/mavros/set_mode')
        self.mav.message_hooks.append(self.on_mav)

    def on_mav(self, connection, message):
        if (message.get_srcSystem(), message.get_srcComponent()) != (1, 1):
            return
        if message.get_type() == 'PARAM_VALUE':
            self.parameters[message.param_id] = float(message.param_value)
            self.parameter_received[message.param_id] = time.monotonic()
        elif message.get_type() == 'POSITION_TARGET_GLOBAL_INT':
            x, y = local_xy(message.lat_int/1e7, message.lon_int/1e7,
                            *self.scene['origin'][:2])
            self.targets.append(dict(wall=time.monotonic(), sim=self.node.get_clock().now().nanoseconds/1e9,
                                     phase=self.phase, xy=[x,y], message=message.to_dict()))

    def set_parameter(self,name,value):
        # A startup PARAM_SET can be dropped while the FCU is initializing.
        # Retry the idempotent request, but require a fresh matching FCU reply.
        start=time.monotonic();last=-math.inf
        while time.monotonic()-start<10.:
            now=time.monotonic()
            if now-last>=1.:
                self.parameter_attempts.append(dict(name=name,value=value,wall=now))
                self.mav.mav.param_set_send(1,1,name.encode(),value,mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                last=now
            self.spin(.05)
            if (self.parameter_received.get(name,0)>=start
                    and abs(self.parameters.get(name,math.inf)-value)<.001):return
        raise TimeoutError(name+' not acknowledged; last value '+str(self.parameters.get(name)))

    def mode(self, name):
        if not self.mode_client.wait_for_service(timeout_sec=5):
            raise TimeoutError('MAVROS set_mode unavailable')
        start = time.monotonic()
        future = self.mode_client.call_async(SetMode.Request(base_mode=0, custom_mode=name))
        self.wait(future.done, 5, 'Mode request service timeout')
        if not future.result().mode_sent:
            raise RuntimeError('MAVROS did not send mode '+name)
        self.wait(lambda: self.data['/mavros/state'].mode == name and
                  self.received['/mavros/state'] >= start, 8, 'FCU did not enter '+name)
        self.mode_events.append(dict(request=name, confirmed=True,
                                     phase=self.phase, wall=time.monotonic(), latency_s=time.monotonic()-start))

    def observe(self, duration, expected_mode, speed=None):
        begin = self.node.get_clock().now().nanoseconds/1e9
        wall_deadline = time.monotonic()+max(30., duration*3.)
        frames = []
        while self.node.get_clock().now().nanoseconds/1e9-begin < duration:
            if time.monotonic() > wall_deadline:
                raise TimeoutError('Simulation clock did not advance at a usable rate')
            if speed is not None:
                self.command(speed, 0.)
            self.spin(.05)
            state = self.data['/mavros/state']
            if not state.connected or not state.armed or state.mode != expected_mode:
                raise RuntimeError('FCU left the requested armed mode')
            if any(time.monotonic()-self.received[t] > .6 for t in
                   ('/usv/ground_truth','/mavros/local_position/odom')):
                raise RuntimeError('Truth/EKF stopped updating')
            sim_now=self.node.get_clock().now().nanoseconds/1e9
            if any(not -.05 <= sim_now-self.data[t].header.stamp.sec-self.data[t].header.stamp.nanosec/1e9 <= .5
                   for t in ('/usv/ground_truth','/mavros/local_position/odom')):
                raise RuntimeError('Truth/EKF source timestamps are stale')
            if self.contacts:
                raise RuntimeError('Contact detected during LOITER experiment')
            if self.rows and self.rows[-1].get('clearance_m', 99.) < 1.:
                raise RuntimeError('Independent evaluation boundary reached: clearance <1 m')
            frames.append(self.data.get('/usv/sitl_frame').data)
        # The plugin reports its 250 Hz counter once per second. The number of
        # distinct telemetry messages is not the number of physics steps.
        if len(set(frames)) < max(2,duration*.5) or frames[-1]-frames[0] < duration*100:
            raise RuntimeError('Insufficient advancing physics/FCU frames')

    def run_case(self, args):
        self.ready()
        for name in ('FRAME_CLASS','LOIT_RADIUS','LOIT_TYPE','LOIT_SPEED_GAIN','WP_SPEED','ATC_STR_RAT_MAX'):
            self.mav.mav.param_request_read_send(1, 1, name.encode(), -1)
        self.wait(lambda: all(k in self.parameters for k in
                             ('FRAME_CLASS','LOIT_RADIUS','LOIT_TYPE','LOIT_SPEED_GAIN')), 10, 'Parameters not received')
        if self.parameters['FRAME_CLASS'] != 2:
            raise RuntimeError('Boat frame required')
        for name,value in [('WP_SPEED',args.wp_speed),('ATC_STR_RAT_MAX',args.yaw_rate_deg)]:
            self.set_parameter(name,value)
        if args.radius is not None:
            self.set_parameter('LOIT_RADIUS',args.radius)
        self.mav.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,
                                       mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,500000,0,0,0,0,0)
        self.arm()
        self.phase = 'entry_motion' if args.moving_entry else 'settle'
        self.observe(10., 'GUIDED', .25 if args.moving_entry else 0.)
        self.targets.clear()
        self.phase = 'mode_entry'
        self.mode('LOITER' if args.mode == 'loiter' else 'GUIDED')
        self.phase = 'hold_measurement'
        begin = len(self.rows)
        self.observe(args.duration, 'LOITER' if args.mode == 'loiter' else 'GUIDED',
                     None if args.mode == 'loiter' else 0.)
        samples = self.rows[begin:]
        targets = [t for t in self.targets if t['phase'] == 'hold_measurement']
        if args.mode == 'loiter' and len(targets) < args.duration/2:
            raise RuntimeError('Insufficient FCU-reported hold target telemetry')
        anchor = targets[0]['xy'] if args.mode=='loiter' and targets else samples[0]['truth'][:2]
        # A moving anchor would hide drift. Native LOITER must retain one target.
        anchor_shift = max((math.dist(t['xy'], anchor) for t in targets), default=0.)
        if anchor_shift > .03:
            raise RuntimeError('FCU hold target changed during observation')
        errors = np.array([math.dist(r['truth'][:2], anchor) for r in samples])
        ekf_errors = np.array([math.dist(r['ekf'][:2], anchor) for r in samples
                               if 'map_alignment_xy' in r])
        result = dict(mode=args.mode, duration_sim_s=samples[-1]['sim']-samples[0]['sim'],
                      sample_count=len(samples), firmware_revision=json.loads((self.run/'provenance.json').read_text())['firmware_revision'],
                      parameters=self.parameters, moving_entry=args.moving_entry,
                      wind_mps=self.scene['wind_mps'], wave_gain=self.scene['wave_gain'],
                      anchor_source='FCU POSITION_TARGET_GLOBAL_INT' if targets else 'initial truth for drift evaluation only',
                      anchor_map_xy_m=anchor, target_change_max_m=anchor_shift,
                      truth_error_rms_m=float(np.sqrt(np.mean(errors**2))),
                      truth_error_p95_m=float(np.percentile(errors,95)),
                      truth_error_max_m=float(errors.max()), truth_error_final_m=float(errors[-1]),
                      ekf_error_max_m=float(ekf_errors.max()) if len(ekf_errors) else None,
                      fraction_outside_loit_radius=float(np.mean(errors>self.parameters['LOIT_RADIUS'])),
                      min_clearance_m=min(r['clearance_m'] for r in samples),
                      truth_speed_max_mps=max(math.hypot(*r['truth'][4:6]) for r in samples),
                      truth_yaw_rate_max_rad_s=max(abs(r['truth'][6]) for r in samples),
                      max_abs_thruster_force_n=max(abs(r.get(k,0.)) for r in samples for k in ('left_n','right_n')),
                      thrust_saturation_samples=sum(any(abs(r.get(k,0.))>=self.scene['thrust_limit_n']-.1
                                                        for k in ('left_n','right_n')) for r in samples),
                      collision_messages=len(self.contacts), mode_events=self.mode_events,
                      measurement_completed=True, accuracy_budget_m=args.error_budget,
                      accuracy_pass=bool(errors.max()<=args.error_budget),
                      task1_automatic_wait_integration=False)
        self.phase = 'resume_guided'
        self.mode('GUIDED')
        self.observe(5., 'GUIDED', .15)
        self.phase = 'resume_stop'
        self.observe(6., 'GUIDED', 0.)
        result['guided_resume_confirmed'] = True
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('loiter','zero_velocity'), default='loiter')
    parser.add_argument('--duration', type=float, default=180.)
    parser.add_argument('--moving-entry', action='store_true')
    parser.add_argument('--radius', type=float)
    parser.add_argument('--wp-speed',type=float,default=.3)
    parser.add_argument('--yaw-rate-deg',type=float,default=23.)
    parser.add_argument('--error-budget', type=float, default=3., help='Experiment threshold, not a Task1 clearance guarantee')
    args = parser.parse_args()
    if (args.duration<10 or args.error_budget<=0 or not 0<args.wp_speed<=.4 or not 0<args.yaw_rate_deg<=23.
            or (args.radius is not None and not 0<args.radius<=20)):
        parser.error('Invalid duration / error budget / LOIT_RADIUS')
    rclpy.init()
    runner = LoiterRunner()
    result = dict(measurement_completed=False)
    try:
        result = runner.run_case(args)
    except Exception as error:
        result['error'] = str(error)
        raise
    finally:
        # Cleanup is excluded from the station-keeping measurement window.
        try:
            runner.phase='cleanup'
            runner.mode('HOLD')
            runner.mav.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)
            runner.spin(1.)
        finally:
            result['parameters']=runner.parameters
            result['parameter_attempts']=runner.parameter_attempts
            (runner.run/'loiter_targets.json').write_text(json.dumps(runner.targets,indent=2)+'\n')
            (runner.run/'loiter_result.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result,indent=2),flush=True)
            runner.log.close()
            runner.node.destroy_node()
            rclpy.shutdown()
    if not result.get('accuracy_pass',False):raise SystemExit(1)


if __name__ == '__main__':
    main()
