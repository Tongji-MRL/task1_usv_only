#!/usr/bin/env python3
"""Replay recorded sensor/FCU TF/path only through the traffic guard, no control.

TF is buffered offline for interpolation at the original measurement time.
There is no Gazebo pose/scene subscription or physical-target association.
This diagnoses perception and prediction; it is not a closed-loop pass.
"""
import argparse
import bisect
import json
from pathlib import Path
import sqlite3

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException
from scipy.spatial.transform import Rotation
from usv_sim.cloud_filter import xyz_view
from robotx_task1.traffic import Tracker, StaticMap, corridor, conflict, exclude_surveyed_markers, tracking_points
from robotx_task1.traffic_guard import seconds, pose_xyyaw


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path)
    p.add_argument('--start',type=float,default=290.);p.add_argument('--end',type=float,default=355.)
    p.add_argument('--horizon',type=float,default=20.)
    p.add_argument('--observation-range',type=float,default=20.)
    p.add_argument('--continuation-range',type=float,default=40.)
    p.add_argument('--output',type=Path);a=p.parse_args()
    rclpy.init();tf=Buffer(cache_time=Duration(seconds=2000.));paths=[];clouds=[]
    for db in sorted((a.run/'sensors').glob('*.db3')):
        conn=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
        topics={i:n for i,n in conn.execute('SELECT id,name FROM topics')}
        for i,name in topics.items():
            if name in ('/tf','/tf_static'):
                for payload, in conn.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',(i,)):
                    for t in deserialize_message(payload,TFMessage).transforms:
                        (tf.set_transform_static if name=='/tf_static' else tf.set_transform)(t,'recorded FCU TF')
            elif name in ('/task1/reference_path','/task1/candidate_path'):
                for payload,received in conn.execute('SELECT data,timestamp FROM messages WHERE topic_id=? ORDER BY timestamp',(i,)):
                    m=deserialize_message(payload,NavPath)
                    path=[]
                    for q in m.poses:
                        v=q.pose.orientation
                        path.append([q.pose.position.x,q.pose.position.y,
                                     np.arctan2(2*(v.w*v.z+v.x*v.y),1-2*(v.y*v.y+v.z*v.z))])
                    # A planner's header can predate its publication. Do not
                    # expose that candidate to earlier sensor samples.
                    paths.append((received/1e9,np.asarray(path)))
            elif name=='/usv/lidar/marking':
                for payload, in conn.execute('SELECT data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp',
                                             (i,int(a.start*1e9),int(a.end*1e9))):
                    clouds.append(deserialize_message(payload,PointCloud2))
        conn.close()
    paths.sort(key=lambda x:x[0]);times=[p[0] for p in paths]
    footprint=json.loads((a.run/'generated/scene.json').read_text())['footprint']
    raw=a.run/'visualization/raw'
    map_event=next(json.loads(l) for l in (raw/'events.jsonl').read_text().splitlines() if json.loads(l)['topic']=='/map')
    static_map=StaticMap(np.load(raw/map_event['file'])['data'],map_event['resolution'],map_event['origin'])
    survey=next(json.loads(l) for l in (a.run/'task1_mission.jsonl').read_text().splitlines()
                if json.loads(l)['event']=='observations_reported')
    markers=np.array([[b['x'],b['y'],b['radius']] for b in survey['buoys']])
    config=json.loads((a.run/'generated/nav2_wamv.yaml').read_text())
    speed=config['controller_server']['ros__parameters']['FollowPath']['max_vel_x']
    tracker=Tracker(horizon=a.horizon,observation_range=a.observation_range);samples=[];errors=[]
    for m in sorted(clouds,key=lambda m:seconds(m.header.stamp)):
        stamp=seconds(m.header.stamp)
        if stamp<survey['sim']:continue
        if stamp<=tracker.stamp:continue
        try:
            t=tf.lookup_transform('map',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
            boat=tf.lookup_transform('map','base_link',Time.from_msg(m.header.stamp)).transform
        except TransformException as e:
            errors.append(dict(stamp=stamp,error=str(e)));continue
        xyz=xyz_view(m).reshape(-1,3);xyz=xyz[np.isfinite(xyz).all(axis=1)]
        q=t.rotation;xyz=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(xyz)+[t.translation.x,t.translation.y,t.translation.z]
        xyz=static_map.unmapped(xyz)
        xyz=exclude_surveyed_markers(xyz,markers)
        xyz=tracking_points(xyz,[t.translation.x,t.translation.y],tracker.tracks,stamp,
                            a.observation_range,a.continuation_range)
        pose=pose_xyyaw(boat);index=bisect.bisect_right(times,stamp)-1
        path=paths[index][1] if index>=0 else None
        tracks=tracker.update(xyz,stamp,np.asarray(pose[:2]));swept=corridor(footprint,pose,path,a.horizon*speed)
        risk=conflict(tracks,stamp,swept,horizon=a.horizon)
        samples.append(dict(source_stamp=stamp,pose=pose,risk=risk,clear=risk is None,
                            tracks=[dict(id=t.id,stamp=t.stamp,bounds=t.bounds.tolist(),velocity=t.velocity.tolist(),moving=t.moving) for t in tracks]))
    out=dict(source=str(a.run),horizon=a.horizon,observation_range=a.observation_range,continuation_range=a.continuation_range,speed_bound=speed,mode='offline recorded LiDAR, FCU TF, accepted paths, static map and surveyed fixed buoy observations; no traffic truth or closed-loop claim',
             samples=samples,transform_errors=errors,first_risk=next((s for s in samples if s['risk']),None))
    output=a.output or a.run/'traffic_sensor_replay.json';output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(dict(output=str(output),samples=len(samples),transform_errors=len(errors),first_risk=out['first_risk']),indent=2))
    rclpy.shutdown()


if __name__=='__main__':main()
