#!/usr/bin/env python3
"""Disarmed physical visibility / clearing fixtures; never inject navigation data."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import ClearEntireCostmap
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from validate import Runner
from validate_environment import gz_service
from usv_sim.cloud_filter import xyz_view


class GeometryCheck(Runner):
    def __init__(self):
        super().__init__('lidar_geometry')
        self.tf=Buffer();self.listener=TransformListener(self.tf,self.node)
        self.pending={};self.samples=[];self.target=None;self.grid=None
        self.raw_topic='/livox/lidar' if self.scene.get('lidar_model')=='mid360' else '/usv/lidar/points'
        self.topics=(self.raw_topic,'/usv/lidar/marking','/scan')
        self.clear_client=self.node.create_client(ClearEntireCostmap,'/global_costmap/clear_entirely_global_costmap')
        for topic in self.topics:
            self.node.create_subscription(LaserScan if topic=='/scan' else PointCloud2,topic,
                lambda m,t=topic:self.pending.__setitem__(t,m),qos_profile_sensor_data)
        self.node.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:setattr(self,'grid',m),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def spin(self,duration=.05):
        super().spin(duration)
        if self.target is None:return
        for topic,msg in list(self.pending.items()):
            try:tr=self.tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
            except Exception:continue
            del self.pending[topic]
            if topic=='/scan':
                values=np.asarray(msg.ranges);a=msg.angle_min+np.arange(len(values))*msg.angle_increment
                points=np.column_stack([values*np.cos(a),values*np.sin(a),np.zeros(len(a))])
            else:points=xyz_view(msg).reshape(-1,3)
            points=points[np.isfinite(points).all(axis=1)];q=tr.rotation;p=tr.translation
            world=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(points)+[p.x,p.y,p.z]
            # World-axis-aligned 1 m box plus 0.15 m association tolerance.
            hit=np.all(np.abs(world[:,:2]-self.target[:2])<=.65,axis=1)
            self.samples.append(dict(phase=self.phase,topic=topic,stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                hits=int(hit.sum()),finite_points=len(points),sensor_z=float(p.z)))

    def create(self,height):
        xml=f'<sdf version="1.9"><model name="lidar_probe"><static>true</static><pose>45 0 {height/2} 0 0 0</pose><link name="body">'
        for tag in ('visual','collision'):
            xml+=f'<{tag} name="{tag}"><geometry><box><size>1 1 {height}</size></box></geometry></{tag}>'
        xml+='</link></model></sdf>'
        gz_service('/world/usv_nav/create','gz.msgs.EntityFactory','sdf: '+json.dumps(xml))

    def move(self,x,y,height):
        gz_service('/world/usv_nav/set_pose','gz.msgs.Pose',
            f'name: "lidar_probe" position {{x:{x} y:{y} z:{height/2}}} orientation {{w:1}}')

    def remove(self):
        gz_service('/world/usv_nav/remove','gz.msgs.Entity','name: "lidar_probe" type: MODEL')

    def background(self,x,y):
        # A real tall reflector behind the old obstacle supplies finite rays.
        xml=(f'<sdf version="1.9"><model name="lidar_background"><static>true</static>'
             f'<pose>{x} {y} 1.5 0 0 0</pose><link name="body">')
        for tag in ('visual','collision'):
            xml+=f'<{tag} name="{tag}"><geometry><box><size>1 8 3</size></box></geometry></{tag}>'
        xml+='</link></model></sdf>'
        gz_service('/world/usv_nav/create','gz.msgs.EntityFactory','sdf: '+json.dumps(xml))

    def clearing_case(self,origin,background=False):
        if background:self.background(origin[0]+18.,origin[1])
        self.target=[origin[0]+12.,origin[1],.9];self.move(*self.target);self.isolate_case()
        self.phase='clearing_present_'+str(background);self.spin(5.);before=self.occupied()
        self.move(45.,30.,.9);self.phase='clearing_removed_'+str(background)
        series=[];start=time.monotonic()
        while time.monotonic()-start<15:
            self.spin(.1);series.append([time.monotonic()-start,self.occupied()])
        result=dict(background=background,before_lethal_cells=before,series=series,
                    after_lethal_cells=self.occupied(),cleared=before is not None and before>0 and self.occupied()==0)
        if background:gz_service('/world/usv_nav/remove','gz.msgs.Entity','name: "lidar_background" type: MODEL')
        return result

    def counts(self,phase):
        return {t:dict(frames=len(s:=[r for r in self.samples if r['phase']==phase and r['topic']==t]),
            max_hits=max((r['hits'] for r in s),default=0),hit_fraction=sum(r['hits']>0 for r in s)/max(1,len(s)))
            for t in self.topics}

    def occupied(self):
        m=self.grid
        if m is None:return None
        a=np.asarray(m.data).reshape(m.info.height,m.info.width);res=m.info.resolution;o=m.info.origin.position
        x,y=self.target[:2];ix=int((x-o.x)/res);iy=int((y-o.y)/res);n=int(math.ceil(.7/res))
        return int(np.count_nonzero(a[max(0,iy-n):iy+n+1,max(0,ix-n):ix+n+1]>=99))

    def isolate_case(self):
        # Disarmed test fixture isolation only. Never clear during the removal
        # measurement or use this service to make a navigation task pass.
        if not self.clear_client.wait_for_service(timeout_sec=3):raise RuntimeError('Costmap clear unavailable')
        f=self.clear_client.call_async(ClearEntireCostmap.Request())
        self.wait(f.done,3,'Fixture isolation clear');self.spin(.3)


def main():
    p=argparse.ArgumentParser();p.add_argument('--dwell',type=float,default=4.)
    p.add_argument('--azimuths',action='store_true');a=p.parse_args()
    rclpy.init();r=GeometryCheck();results=[];directions=[];created=False
    try:
        r.ready();r.navigation_ready()
        if r.data['/mavros/state'].armed:raise RuntimeError('Disarmed fixture only')
        origin=r.rows[-1]['ekf'][:2]
        for height in (.3,.9,1.8):
            r.create(height);created=True
            for distance in (3.,5.,8.,12.,16.):
                x,y=origin[0]+distance,origin[1]
                r.target=[x,y,height];r.phase=f'h{height}_d{distance}';r.move(x,y,height);r.isolate_case()
                r.spin(.7);r.samples=[s for s in r.samples if s['phase']!=r.phase]
                start=time.monotonic()
                while time.monotonic()-start<a.dwell:r.spin()
                results.append(dict(height_m=height,body_distance_m=distance,position=r.target.copy(),
                                    observations=r.counts(r.phase),lethal_cells=r.occupied()))
            r.remove();created=False;r.spin(.5)
        if a.azimuths:
            for height in (.3,.9):
                r.create(height);created=True
                for degrees in range(0,360,45):
                    angle=math.radians(degrees)
                    r.target=[origin[0]+5*math.cos(angle),origin[1]+5*math.sin(angle),height]
                    r.phase=f'azimuth_h{height}_{degrees}';r.move(*r.target);r.isolate_case();r.spin(.7)
                    r.samples=[s for s in r.samples if s['phase']!=r.phase];r.spin(a.dwell)
                    directions.append(dict(height_m=height,body_distance_m=5.,azimuth_deg=degrees,
                                           observations=r.counts(r.phase),lethal_cells=r.occupied()))
                r.remove();created=False;r.spin(.5)
        # This checks the real costmap's retained evidence without forcing free space.
        r.create(.9);created=True
        clearing=[r.clearing_case(origin,background) for background in (False,True)]
        low=next(q for q in results if q['height_m']==.3 and q['body_distance_m']==5.)
        out=dict(lidar_model=r.scene.get('lidar_model','vrx'),lidar_xyz=r.scene['lidar_xyz'],
                 firmware_revision=json.loads((r.run/'provenance.json').read_text())['firmware_revision'],
                 cases=results,azimuths=directions,
                 near_low_obstacle_detected=all(v['hit_fraction']>=.5 for v in low['observations'].values()),
                 clearing=clearing,
                 evidence_note='Physical test boxes only; FCU TF associates returns. Undetected/uncleared cases remain failures, not fabricated free rays.')
        (r.run/'lidar_geometry_result.json').write_text(json.dumps(out,indent=2)+'\n')
        (r.run/'lidar_geometry_samples.json').write_text(json.dumps(r.samples)+'\n')
        print(json.dumps(out,indent=2),flush=True)
    finally:
        if created:r.remove()
        r.close();rclpy.try_shutdown()


if __name__=='__main__':main()
