#!/usr/bin/env python3
"""Offline physical association of moving-box returns; never publishes navigation input."""
import argparse
import bisect
import json
import sqlite3
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2,LaserScan
from usv_sim.cloud_filter import xyz_view


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--start',type=float,default=300.)
    p.add_argument('--end',type=float,default=360.);a=p.parse_args();r=a.run
    def rows(name):return [json.loads(l) for l in (r/name).read_text().splitlines()]
    physics=sorted((x for x in rows('task1.jsonl') if 'truth_rpy' in x and 'ekf' in x),key=lambda x:x['sim'])
    scenes=sorted((x for x in rows('task1_judge.jsonl') if x['type']=='scene'),key=lambda x:x['stamp'])
    pt=[x['sim'] for x in physics];st=[x['stamp'] for x in scenes]
    lever=np.asarray(json.loads((r/'generated/scene.json').read_text())['lidar_xyz'])
    def nearest(data,times,t):
        i=bisect.bisect_left(times,t);return min(data[max(0,i-1):i+1],key=lambda x:abs(x.get('sim',x.get('stamp'))-t))
    result=[]
    measured=('/usv/lidar/points','/usv/lidar/filtered','/usv/lidar/marking','/scan')
    for db in sorted((r/'sensors').glob('*.db3')):
        conn=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
        topics={i:n for i,n in conn.execute('SELECT id,name FROM topics') if n in measured}
        for i,topic in topics.items():
            cls=LaserScan if topic=='/scan' else PointCloud2
            for payload, in conn.execute('SELECT data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp',
                                        (i,int(a.start*1e9),int(a.end*1e9))):
                m=deserialize_message(payload,cls);t=m.header.stamp.sec+m.header.stamp.nanosec/1e9
                q=nearest(physics,pt,t);scene=nearest(scenes,st,t)['data'];box=np.asarray(scene['dynamic_pose'])
                rot=Rotation.from_euler('xyz',q['truth_rpy']);sensor=rot.apply(lever)+q['truth'][:3]
                if topic=='/scan':
                    dist=np.asarray(m.ranges);ang=m.angle_min+np.arange(len(dist))*m.angle_increment
                    xyz=np.column_stack([dist*np.cos(ang),dist*np.sin(ang),np.zeros(len(dist))]);xyz=xyz[np.isfinite(xyz).all(axis=1)]
                    world=Rotation.from_euler('z',q['ekf'][3]).apply(xyz)+q['ekf'][:3]
                else:
                    xyz=xyz_view(m).reshape(-1,3);xyz=xyz[np.isfinite(xyz).all(axis=1)]
                    world=rot.apply(xyz+lever)+q['truth'][:3]
                hit=np.all(np.abs(world[:,:2]-box)<[1.3,1.8],axis=1)
                overflight=[]
                if topic!='/scan':
                    for point in world[hit]:
                        delta=point-sensor;enter=0.;leave=1.
                        for k,half in ((0,1.),(1,1.5)):
                            if abs(delta[k])<1e-9:
                                if abs(sensor[k]-box[k])>half:leave=-1.
                                continue
                            cuts=sorted(((box[k]-half-sensor[k])/delta[k],(box[k]+half-sensor[k])/delta[k]))
                            enter=max(enter,cuts[0]);leave=min(leave,cuts[1])
                        if enter<leave and sensor[2]+enter*delta[2]>.92:
                            overflight.append((leave-enter)*float(np.linalg.norm(delta[:2])))
                result.append(dict(topic=topic,stamp=t,hits=int(np.count_nonzero(hit)),
                                   projected_ray_overflight_max_m=max(overflight,default=0.),
                                   max_return_y_m=float(world[hit,1].max()) if np.any(hit) else None,
                                   body_leading_edge_y_m=float(box[1]+1.5),
                                   leading_edge_gap_y_m=float(box[1]+1.5-world[hit,1].max()) if np.any(hit) else None,
                                   return_world_z_range_m=[float(world[hit,2].min()),float(world[hit,2].max())] if np.any(hit) else None,
                                   box_xy=box.tolist(),sensor_xy=sensor[:2].tolist(),
                                   sensor_to_box_center_m=float(np.linalg.norm(sensor[:2]-box)),
                                   physical_clearance_m=q['clearance_m'],pose_time_offset_s=q['sim']-t))
        conn.close()
    out=dict(source=str(r),association='Truth pose transforms raw/filtered/marking clouds for independent physical evaluation; scan uses recorded FCU EKF map transform. Box AABB expanded 0.3 m. No control input is produced.',samples=sorted(result,key=lambda x:(x['stamp'],x['topic'])))
    (r/'dynamic_lidar_diagnostic.json').write_text(json.dumps(out,indent=2)+'\n')
    for target in range(int(a.start),int(a.end)+1,2):
        samples=[x for x in result if abs(x['stamp']-target)<.4]
        if samples:print(json.dumps(dict(stamp=target,hits={topic:max((x['hits'] for x in samples if x['topic']==topic),default=None)
            for topic in measured},range_m=samples[0]['sensor_to_box_center_m'],clearance_m=samples[0]['physical_clearance_m'])))

if __name__=='__main__':main()
