#!/usr/bin/env python3
"""Check live Garden MID-360 output, projected scan and capture a point-cloud plot."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan

from usv_sim.cloud_filter import xyz_view
from usv_sim.livox_filter import point_times

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--duration',type=float,default=20.)
    args=parser.parse_args()
    run=Path((ROOT/'runs/latest.txt').read_text().strip())
    scene=json.loads((run/'generated/scene.json').read_text())
    if scene.get('lidar_model')!='mid360':raise RuntimeError('MID-360 session required')
    rclpy.init();node=rclpy.create_node('validate_mid360',parameter_overrides=[Parameter('use_sim_time',value=True)])
    raw=[];filtered=[];scans=[];snapshots=[];failures=[]
    expected=[('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('tag',16,2),('line',17,2),('timestamp',18,8)]

    def cloud(msg):
        try:
            if msg.height!=1 or msg.point_step!=26 or [(f.name,f.offset,f.datatype) for f in msg.fields]!=expected:
                raise ValueError('Driver PointCloud2 layout mismatch')
            points=xyz_view(msg).reshape(-1,3);times,base=point_times(msg)
            if not np.isfinite(points).all():raise ValueError('Raw no-return points must be omitted')
            age=(node.get_clock().now().nanoseconds-base)/1e9
            row=dict(stamp=base/1e9,width=msg.width,age_s=age,
                     slice_count=len(np.unique(times)),span_s=float(np.ptp(times)/1e9) if len(times) else 0.)
            if len(raw) and row['stamp']<=raw[-1]['stamp']:raise ValueError('Non-monotonic sensor stamp')
            raw.append(row)
            if len(snapshots)<6 and (not snapshots or row['stamp']-snapshots[-1][0]>=1.):
                snapshots.append((row['stamp'],points.copy()))
        except Exception as error:
            failures.append(str(error))

    def scan(msg):
        a=np.asarray(msg.ranges)
        scans.append(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                          valid_hits=int(np.isfinite(a).sum()),unknown=int(np.isnan(a).sum()),
                          infinite=int(np.isinf(a).sum())))

    node.create_subscription(PointCloud2,'/livox/lidar',cloud,qos_profile_sensor_data)
    node.create_subscription(PointCloud2,'/usv/lidar/filtered',lambda m:filtered.append(
        dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,width=m.width)),qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
    start=time.monotonic();observing=None
    while time.monotonic()-start<args.duration+90:
        rclpy.spin_once(node,timeout_sec=.05)
        if raw and scans and observing is None:observing=time.monotonic()
        if observing is not None and time.monotonic()-observing>=args.duration:break
    if len(raw)<10:failures.append('Raw lidar did not produce sufficient frames')
    if len(scans)<10:failures.append('Guarded projected scan did not produce sufficient frames')
    if any(r['infinite'] for r in scans):failures.append('Unobserved directions were marked as infinite/free')
    raw_stamps={round(r['stamp'],6) for r in raw}
    # The first few already queued output frames can predate this subscriber.
    aligned=[r for r in scans if raw and r['stamp']>=raw[0]['stamp']]
    missing=sum(round(r['stamp'],6) not in raw_stamps for r in aligned)
    if missing:failures.append('Projected scan stamps did not match captured original frames')
    intervals=np.diff([r['stamp'] for r in raw])
    scan_intervals=np.diff([r['stamp'] for r in scans])
    scan_hz=(len(scans)-1)/(scans[-1]['stamp']-scans[0]['stamp']) if len(scans)>1 else 0.
    if len(intervals) and not 9<=1/np.median(intervals)<=11:
        failures.append('Raw simulated frame rate differs from 10 Hz')
    if scan_hz<8 or (len(scan_intervals) and max(scan_intervals)>.3):
        failures.append('Projected scan throughput or maximum gap exceeded the configured budget')
    if any(r['age_s']<0 or r['age_s']>.5 for r in raw):
        failures.append('Raw source age exceeded the unchanged 0.5 s input deadline')
    if not any(r['valid_hits'] for r in scans):
        failures.append('Known visible obstacle never appeared in the scan')
    result=dict(passed=not failures,failures=failures,frames=len(raw),filtered_frames=len(filtered),scan_frames=len(scans),
                median_sim_hz=float(1/np.median(intervals)) if len(intervals) else None,
                effective_scan_hz=scan_hz,
                max_scan_gap_s=float(max(scan_intervals)) if len(scan_intervals) else None,
                max_frame_gap_s=float(max(intervals)) if len(intervals) else None,
                valid_returns_per_frame=[min((r['width'] for r in raw),default=0),max((r['width'] for r in raw),default=0)],
                max_frame_age_s=max((r['age_s'] for r in raw),default=None),
                slices_observed=sorted(set(r['slice_count'] for r in raw)),
                max_point_time_span_s=max((r['span_s'] for r in raw),default=None),
                matched_source_stamps=len(aligned)-missing,simulator=scene['lidar_simulation'],
                navigation_closed_loop_validated=False,real_hardware_validated=False)
    (run/'mid360_result.json').write_text(json.dumps(result,indent=2)+'\n')
    (run/'mid360_samples.json').write_text(json.dumps(dict(raw=raw,filtered=filtered,scans=scans))+'\n')
    if snapshots:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(2,3,figsize=(12,7),layout='constrained')
        for ax,(stamp,points) in zip(axes.flat,snapshots):
            ax.scatter(points[:,0],points[:,1],c=points[:,2],s=2,vmin=-2,vmax=8,cmap='viridis')
            ax.set(xlim=(-40,40),ylim=(-40,40),aspect='equal',xlabel='sensor x (m)',ylabel='sensor y (m)',
                   title=f't={stamp:.2f} s, {len(points)} returns')
            ax.grid(alpha=.2)
        fig.suptitle('MID-360 approximation: six separate frames (colour = sensor z)')
        fig.savefig(run/'mid360_pointcloud.png',dpi=150);plt.close(fig)
        np.savez_compressed(run/'mid360_snapshot.npz',**{f'frame_{i}':p for i,(_,p) in enumerate(snapshots)})
    node.destroy_node();rclpy.shutdown()
    print(json.dumps(result,indent=2),flush=True)
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
