#!/usr/bin/env python3
"""Measure buoy returns at raw cloud, filtered cloud and guarded LaserScan stages."""
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, LaserScan
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
from usv_sim.cloud_filter import xyz_view

ROOT=Path(__file__).resolve().parents[1]


def main():
    run=Path((ROOT/'runs/latest.txt').read_text().strip());scene=json.loads((run/'generated/scene.json').read_text())
    buoys=scene['task1']['buoys'];rclpy.init()
    node=rclpy.create_node('task1_lidar_evidence',parameter_overrides=[Parameter('use_sim_time',value=True)])
    tf=Buffer();listener=TransformListener(tf,node);rows=[];pending={}
    topics=('/livox/lidar' if scene.get('lidar_model')=='mid360' else '/usv/lidar/points',
            '/usv/lidar/filtered','/scan')
    for topic in topics:
        node.create_subscription(LaserScan if topic=='/scan' else PointCloud2,topic,
                                 lambda m,t=topic:pending.__setitem__(t,m),qos_profile_sensor_data)
    start=time.monotonic()
    while time.monotonic()-start<30:
        rclpy.spin_once(node,timeout_sec=.02)
        for topic,msg in list(pending.items()):
            try:tr=tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
            except Exception:continue
            del pending[topic]
            if topic=='/scan':
                ranges=np.asarray(msg.ranges);angles=msg.angle_min+np.arange(len(ranges))*msg.angle_increment
                xyz=np.column_stack([ranges*np.cos(angles),ranges*np.sin(angles),np.zeros_like(ranges)])
            else:xyz=xyz_view(msg).reshape(-1,3)
            xyz=xyz[np.isfinite(xyz).all(axis=1)];q=tr.rotation;p=tr.translation
            world=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(xyz)+[p.x,p.y,p.z]
            hits={b['id']:int(np.sum(np.linalg.norm(world[:,:2]-[b['x'],b['y']],axis=1)<.6)) for b in buoys}
            rows.append(dict(topic=topic,stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                             finite_points=len(xyz),hits=hits,sensor_map_xyz=[p.x,p.y,p.z]))
    summary={t:{b['id']:max((r['hits'][b['id']] for r in rows if r['topic']==t),default=0) for b in buoys} for t in topics}
    common=[b['id'] for b in buoys if all(summary[t][b['id']]>0 for t in topics)]
    report=dict(passed=bool(common),detected_through_entire_chain=common,max_hits=summary,samples=rows,
                association='map TF from FCU EKF, 0.6 m association radius; finite physical sensor returns, not costmap marks')
    (run/'lidar_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'}));node.destroy_node();rclpy.try_shutdown()
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
