#!/usr/bin/env python3
"""Run the real Nav2 VoxelLayer on a retained sensor bag in an isolated ROS domain."""
import argparse
import bisect
import datetime
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.serialization import deserialize_message
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from matplotlib.path import Path as Polygon
from prepare_task1 import voxel_obstacles


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--offset',type=float,default=250.)
    p.add_argument('--rate',type=float,default=2.);a=p.parse_args();r=a.run.resolve()
    if os.environ.get('ROS_DOMAIN_ID')!='43':raise RuntimeError('Use isolated ROS_DOMAIN_ID=43')
    out=r/'voxel_replay'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S');out.mkdir(parents=True);scene=json.loads((r/'generated/scene.json').read_text())
    sensor=ET.parse(r/'generated/usv_nav.sdf').find(".//sensor[@name='lidar_wamv_sensor']")
    scene.update(lidar_vertical_min_rad=float(sensor.findtext('.//vertical/min_angle')),
                 lidar_vertical_max_rad=float(sensor.findtext('.//vertical/max_angle')))
    (out/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    params=dict(use_sim_time=True,global_frame='map',robot_base_frame='base_link',rolling_window=True,
                width=30,height=30,resolution=.05,update_frequency=5.,publish_frequency=5.,
                footprint=json.dumps(scene['footprint']),footprint_padding=.5,track_unknown_space=False,
                always_send_full_costmap=True,transform_tolerance=.3,plugins=['obstacle_layer'],
                obstacle_layer=voxel_obstacles())
    (out/'params.json').write_text(json.dumps({'/**':{'ros__parameters':params}},indent=2)+'\n')
    physics=[json.loads(l) for l in (r/'task1.jsonl').read_text().splitlines()];physics=[d for d in physics if 'ekf' in d]
    times=[d['sim'] for d in physics];footprint=np.asarray(scene['footprint']);footprint+=np.sign(footprint)*.5
    rclpy.init();node=rclpy.create_node('voxel_replay_evidence',parameter_overrides=[Parameter('use_sim_time',value=True)])
    samples=[];last=[-1e9]
    def grid(msg):
        t=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if t<280 or t-last[0]<.4:return
        last[0]=t;i=bisect.bisect_left(times,t);row=min(physics[max(0,i-1):i+1],key=lambda d:abs(d['sim']-t));p=row['ekf'];h=p[3]
        g=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width);iy,ix=np.nonzero(g>=100)
        xy=np.column_stack([ix,iy])*msg.info.resolution+[msg.info.origin.position.x,msg.info.origin.position.y]+msg.info.resolution/2
        hull=footprint@np.array([[math.cos(h),math.sin(h)],[-math.sin(h),math.cos(h)]])+p[:2]
        count=int(np.count_nonzero(Polygon(hull).contains_points(xy)))
        name=f'grid_{t:.3f}.npz';np.savez_compressed(out/name,data=g)
        samples.append(dict(stamp=t,lethal_in_padded_hull=count,lethal_total=len(ix),file=name,
                            origin=[msg.info.origin.position.x,msg.info.origin.position.y],resolution=msg.info.resolution))
    node.create_subscription(OccupancyGrid,'/costmap/costmap',grid,10)
    static=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    transforms={}
    for db in sorted((r/'sensors').glob('*.db3')):
        con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
        for data, in con.execute("SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name='/tf_static' ORDER BY m.timestamp"):
            for tr in deserialize_message(data,TFMessage).transforms:transforms[tr.child_frame_id]=tr
        con.close()
    static.publish(TFMessage(transforms=list(transforms.values())))
    children=[]
    def launch(command,name):
        child=subprocess.Popen(command,stdout=(out/(name+'.log')).open('w'),stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child);return child
    def spin_until(predicate,timeout,description):
        deadline=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>deadline:raise TimeoutError(description)
            rclpy.spin_once(node,timeout_sec=.05)
    try:
        launch(['ros2','run','usv_sim','cloud_filter','--ros-args','-p','use_sim_time:=true','-p',
                'publish_clearing_cloud:=true','-p','scene_file:='+str(out/'scene.json')],'filter')
        root=Path(__file__).resolve().parents[1]
        launch(['ros2','run','pointcloud_to_laserscan','pointcloud_to_laserscan_node','--ros-args',
                '--params-file',str(root/'config/cloud_to_scan.yaml'),'-r','cloud_in:=/usv/lidar/filtered',
                '-r','scan:=/usv/scan_raw'],'cloud_to_scan')
        launch(['ros2','run','nav2_costmap_2d','nav2_costmap_2d','--ros-args','--params-file',str(out/'params.json')],'costmap')
        client=node.create_client(ChangeState,'/costmap/costmap/change_state')
        spin_until(client.service_is_ready,15,'costmap lifecycle service')
        future=client.call_async(ChangeState.Request(transition=Transition(id=1)))
        spin_until(future.done,15,'costmap configure')
        if not future.result().success:raise RuntimeError('configure rejected')
        player=launch(['ros2','bag','play',str(r/'sensors'),'--start-offset',str(a.offset),'--rate',str(a.rate),
                       '--topics','/usv/lidar/points','/tf','/clock','--disable-keyboard-controls'],'player')
        spin_until(lambda:node.get_clock().now().nanoseconds>int(a.offset*1e9),15,'recorded clock')
        future=client.call_async(ChangeState.Request(transition=Transition(id=3)))
        spin_until(future.done,20,'costmap activate')
        if not future.result().success:raise RuntimeError('activate rejected')
        spin_until(lambda:player.poll() is not None,150,'bag completion')
        if player.returncode:raise RuntimeError('bag player failed')
    finally:
        for child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        for child in children:
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        (out/'result.json').write_text(json.dumps(dict(source=str(r),samples=samples,
            limitation='Actual Nav2 layer replay with recorded EKF TF; no controller or SITL. This is map evidence only.'),indent=2)+'\n')
        node.destroy_node();rclpy.try_shutdown()
    print(json.dumps(dict(samples=len(samples),first_hull_occupied=next((s for s in samples if s['lethal_in_padded_hull']),None))))
    if not samples:raise RuntimeError('No published costmap received')

if __name__=='__main__':main()
