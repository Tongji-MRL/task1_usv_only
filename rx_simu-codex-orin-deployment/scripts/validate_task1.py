#!/usr/bin/env python3
"""Run one normally-armed Task1 attempt; retain thrust, EKF, truth and video."""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from robotx_task1_interfaces.msg import MotionPermission
from pymavlink import mavutil
from validate import Runner


class TaskRunner(Runner):
    def __init__(self,video=True):
        super().__init__('task1')
        # The evaluation process observes navigation; it never publishes velocity/setpoints.
        self.node.destroy_publisher(self.raw);self.node.destroy_publisher(self.twist)
        self.status=None;self.events=[];self.scene_state=None;self.writers={};self.frame_counts={};self.frame_stamps={}
        self.station_log=(self.run/'task1_stationkeeping.jsonl').open('w')
        self.costmaps={}
        for topic in ('/global_costmap/costmap','/local_costmap/costmap'):
            self.node.create_subscription(OccupancyGrid,topic,lambda m,t=topic:self.costmaps.__setitem__(t,m),
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(String,'/task1/status',self.on_status,10)
        self.node.create_subscription(String,'/task1/events',lambda m:self.events.append(json.loads(m.data)),10)
        self.node.create_subscription(String,'/task1/scene_state',self.on_scene,10)
        self.node.create_subscription(String,'/usv/stationkeeping_status',self.on_station,10)
        self.node.create_subscription(MotionPermission,'/task1/motion_permission',lambda m:self.receive('/task1/motion_permission',m),10)
        if video:
            for topic,name in (('/usv/overview/image','task1_overview'),('/task1/beacon_view','task1_beacon')):
                self.node.create_subscription(Image,topic,lambda m,n=name:self.on_image(n,m),qos_profile_sensor_data)

    def on_status(self,msg):
        self.status=json.loads(msg.data);self.phase=self.status['stage']+'/'+self.status['motion']

    def on_station(self,msg):
        row=json.loads(msg.data);row['received_wall']=time.monotonic()
        self.station_log.write(json.dumps(row,allow_nan=False)+'\n');self.station_log.flush()

    def navigation_ready(self):
        super().navigation_ready()
        report={}
        def maps_ready():
            report.clear()
            for topic in ('/global_costmap/costmap','/local_costmap/costmap'):
                msg=self.costmaps.get(topic)
                if msg is None or msg.header.frame_id!='map':return False
                grid=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
                origin=msg.info.origin.position;res=msg.info.resolution;missing=[];checked=[]
                for b in self.scene['task1']['buoys']:
                    x=int(math.floor((b['x']-origin.x)/res));y=int(math.floor((b['y']-origin.y)/res))
                    if not(0<=x<msg.info.width and 0<=y<msg.info.height):
                        if topic.startswith('/global'):missing.append(b['id'])
                        continue
                    radius=int(math.ceil(b['radius']/res))+1
                    window=grid[max(0,y-radius):y+radius+1,max(0,x-radius):x+radius+1]
                    checked.append(b['id'])
                    if not np.any(window>=100):missing.append(b['id'])
                report[topic]=dict(source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                   checked=checked,missing=missing)
            return len(report)==2 and all(r['checked'] and not r['missing'] for r in report.values())
        try:self.wait(maps_ready,20,'physical buoys retained in both costmaps before arming')
        finally:(self.run/'startup_costmap_check.json').write_text(json.dumps(report,indent=2)+'\n')

    def on_scene(self,msg):
        self.scene_state=json.loads(msg.data)
        if 'dynamic_pose' in self.scene_state:
            for o in self.scene['obstacles']:
                if o['name']=='task1_moving_obstacle':o['x'],o['y']=self.scene_state['dynamic_pose']

    def on_image(self,name,msg):
        a=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        bgr=cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if msg.encoding=='rgb8' else a.copy()
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        cv2.putText(bgr,f'Simulation {stamp:.1f}s | {self.phase}',(16,30),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2)
        if name not in self.writers:
            self.writers[name]=cv2.VideoWriter(str(self.run/(name+'.mp4')),cv2.VideoWriter_fourcc(*'mp4v'),5,(msg.width,msg.height))
            self.frame_counts[name]=0;self.frame_stamps[name]=[];cv2.imwrite(str(self.run/(name+'.png')),bgr)
        self.writers[name].write(bgr);self.frame_counts[name]+=1;self.frame_stamps[name].append(stamp)

    def record(self):
        super().record()
        if self.rows:
            # Parent log includes physical observations; task event logs retain revision and permissions.
            self.rows[-1]['task_status']=self.status

    def finish(self):
        # Cleanup occurs only after the 10 s observation window, outside task scoring.
        self.phase='cleanup'
        self.mav.mav.set_mode_send(1,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,4)
        self.spin(.3)
        self.mav.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)
        self.spin(.3)
        for writer in self.writers.values():writer.release()
        (self.run/'task1_video.json').write_text(json.dumps({
            'frames':self.frame_counts,'configured_camera_update_rate_hz':5,'encoded_fps':5,
            'source_sim_stamps':self.frame_stamps,
            'timing_note':'Received frames are encoded in order. Delivery may be below the configured rate; use source stamps, not MP4 elapsed time, for measurements.'},indent=2)+'\n')
        self.mav.close();self.log.close();self.station_log.close();self.node.destroy_node()


def main():
    p=argparse.ArgumentParser();p.add_argument('--no-video',action='store_true')
    p.add_argument('--wall-timeout',type=float,default=2400.);p.add_argument('--cancel-after',type=float)
    a=p.parse_args();rclpy.init();runner=TaskRunner(not a.no_video)
    outcome={'passed':False};start=time.monotonic()
    try:
        runner.ready();runner.navigation_ready();runner.arm()
        client=runner.node.create_client(Trigger,'/task1/start')
        if not client.wait_for_service(timeout_sec=15):raise TimeoutError('Task1 service missing')
        request=client.call_async(Trigger.Request());runner.wait(request.done,10,'Task1 start response')
        if not request.result().success:raise RuntimeError(request.result().message)
        sim_start=runner.node.get_clock().now().nanoseconds/1e9;last_print=-1e9;cancelled=False
        while time.monotonic()-start<a.wall_timeout:
            runner.spin()
            sim=runner.node.get_clock().now().nanoseconds/1e9
            if sim-last_print>=10:
                print(json.dumps({'sim':sim,'status':runner.status}),flush=True);last_print=sim
            if runner.contacts and not cancelled:
                print(json.dumps({'test_harness_abort':'physical contact','sim':sim}),flush=True)
                cancel=runner.node.create_client(Trigger,'/task1/cancel');cancel.call_async(Trigger.Request());cancelled=True
            if a.cancel_after and sim-sim_start>a.cancel_after and not cancelled:
                cancel=runner.node.create_client(Trigger,'/task1/cancel');cancel.call_async(Trigger.Request());cancelled=True
            if runner.status and runner.status['stage'] in ('COMPLETE','ABORTED'):break
        else:raise TimeoutError('Task1 wall watchdog')
        completion=runner.node.get_clock().now().nanoseconds/1e9
        while runner.node.get_clock().now().nanoseconds/1e9-completion<10:runner.spin()
        outcome=dict(status=runner.status,elapsed_wall_s=time.monotonic()-start,events=runner.events,
                     completed=runner.status['stage']=='COMPLETE',collision_messages=len(runner.contacts),
                     min_clearance_m=min((r['clearance_m'] for r in runner.rows if 'clearance_m' in r),default=None))
    except Exception as error:
        outcome['error']=repr(error);print(repr(error),flush=True)
        cancel=runner.node.create_client(Trigger,'/task1/cancel')
        if cancel.service_is_ready():cancel.call_async(Trigger.Request());runner.spin(2.)
    finally:
        (runner.run/'task1_execution.json').write_text(json.dumps(outcome,indent=2)+'\n')
        runner.finish();rclpy.try_shutdown()
    print(json.dumps(outcome),flush=True)
    if not outcome.get('completed'):raise SystemExit(1)


if __name__=='__main__':main()
