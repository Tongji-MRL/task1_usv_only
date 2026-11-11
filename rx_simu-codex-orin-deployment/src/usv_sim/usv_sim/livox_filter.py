"""Unorganized Livox PointCloud2 adapter; no synthetic no-return rays.

Deskew uses only the FCU-derived TF tree, in <=5 ms groups. The reference stamp
remains the driver's first-point stamp. This does not implement LiDAR odometry.
"""
import copy
import json
import time
from collections import OrderedDict, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener, TransformException

from .cloud_filter import inside_polygon, xyz_view
from .return_support import ReturnSupport


def transform_points(points, transform):
    q = transform.transform.rotation
    p = transform.transform.translation
    return Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(points)+[p.x,p.y,p.z]


def point_times(msg):
    fields = {f.name:f for f in msg.fields}
    f = fields.get('timestamp')
    if f is None or f.datatype != PointField.FLOAT64:
        raise ValueError('Livox driver-format float64 timestamp (absolute ns) required')
    base = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
    if msg.width*msg.height == 0:
        return np.empty(0,dtype=np.float64),base
    times = np.ndarray((msg.height,msg.width), dtype=('>' if msg.is_bigendian else '<')+'f8',
                       buffer=msg.data, offset=f.offset,
                       strides=(msg.row_step,msg.point_step)).reshape(-1).copy()
    if not np.isfinite(times).all() or np.any(times < base-1000) or np.any(times > base+120000000):
        raise ValueError('Point timestamps are outside the original frame; check clock synchronization')
    return times, base


def unknown_empty_bins(ranges):
    """An empty projection bin is not evidence of a measured free-water ray."""
    values = np.asarray(ranges, dtype=np.float32).copy()
    values[np.isinf(values)] = np.nan
    return values


class LivoxFilter(Node):
    def __init__(self):
        super().__init__('livox_filter')
        defaults = dict(footprint='[]', body_frame='base_link', stabilized_frame='base_stabilized',
                        fixed_frame='odom', marking_min_height=.25, marking_max_height=3.,
                        input_timeout=.5, input_topic='/livox/lidar')
        for k,v in defaults.items():
            self.declare_parameter(k,v)
        self.p = {k:self.get_parameter(k).value for k in defaults}
        self.polygon = json.loads(self.p['footprint'])
        if len(self.polygon)<3:
            raise ValueError('Physical hull polygon required')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.cache = OrderedDict()
        self.pending = deque(maxlen=8)
        self.support=ReturnSupport()
        self.filtered = self.create_publisher(PointCloud2,'/usv/lidar/filtered',qos_profile_sensor_data)
        self.marking = self.create_publisher(PointCloud2,'/usv/lidar/marking',qos_profile_sensor_data)
        self.clearing = self.create_publisher(PointCloud2,'/usv/lidar/clearing',qos_profile_sensor_data)
        self.scan = self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.create_subscription(PointCloud2,self.p['input_topic'],self.on_cloud,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/usv/scan_raw',self.on_scan,qos_profile_sensor_data)
        self.create_timer(.02,self.process_pending)

    def fresh(self, header):
        age = (self.get_clock().now().nanoseconds-(header.stamp.sec*1000000000+header.stamp.nanosec))/1e9
        return 0<=age<=self.p['input_timeout']

    @staticmethod
    def cloud(header, points):
        out = PointCloud2()
        out.header=copy.deepcopy(header)
        out.height=1;out.width=len(points);out.point_step=12;out.row_step=12*len(points)
        out.fields=[PointField(name=k,offset=4*i,datatype=PointField.FLOAT32,count=1)
                    for i,k in enumerate(('x','y','z'))]
        out.data=np.asarray(points,dtype='<f4').tobytes();out.is_dense=True
        return out

    def on_cloud(self, msg):
        self.pending.append((msg,time.monotonic()))

    def process_pending(self):
        while self.pending:
            msg,received=self.pending[0]
            if not self.fresh(msg.header) or time.monotonic()-received>self.p['input_timeout']:
                self.pending.popleft()
                continue
            if not self.process_cloud(msg):
                break  # TF has not caught up to this frame's last measurement.
            self.pending.popleft()

    def process_cloud(self, msg):
        if not self.fresh(msg.header):
            return True
        try:
            raw=xyz_view(msg).reshape(-1,3)
            times, base=point_times(msg)
            stamp=Time.from_msg(msg.header.stamp)
            mount=self.tf.lookup_transform(self.p['body_frame'],msg.header.frame_id,stamp)
            finite=np.isfinite(raw).all(axis=1)&(np.linalg.norm(raw,axis=1)>=.1)
            indices=np.flatnonzero(finite)
            finite[indices]=~inside_polygon(transform_points(raw[indices],mount),self.polygon)
            observed=raw[finite]
            points=observed.copy();times=times[finite]
            # Upper-bounded number of TF queries also works with real per-point
            # times, not only the simulator's five discrete time slices.
            bins=np.floor((times-base)/5000000).astype(np.int64)
            measured_times={}
            for b in np.unique(bins):
                selected=bins==b
                measurement_ns=int(np.mean(times[selected]))
                movement=self.tf.lookup_transform_full(msg.header.frame_id,stamp,
                          msg.header.frame_id,Time(nanoseconds=measurement_ns),self.p['fixed_frame'])
                points[selected]=transform_points(points[selected],movement)
                measured_times[b]=measurement_ns
            stable_tf=self.tf.lookup_transform(self.p['stabilized_frame'],msg.header.frame_id,stamp)
            stable=transform_points(points,stable_tf) if len(points) else points
            keep=(stable[:,2]>=self.p['marking_min_height'])&(stable[:,2]<=self.p['marking_max_height'])
            fixed_tf=self.tf.lookup_transform(self.p['fixed_frame'],msg.header.frame_id,stamp)
            supported=self.support.select(transform_points(points[keep],fixed_tf),base/1e9)
            accepted=np.ones(len(points),bool);accepted[np.flatnonzero(keep)]=supported
            # Scans and costmaps see the same supported marking surface. Keep
            # other heights for genuine finite clearing rays, never max-range
            # substitutes. Rejected returns also cannot clear an obstacle.
            out=self.cloud(msg.header,points[accepted])
            key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
            self.cache[key]=True
            while len(self.cache)>20:
                self.cache.popitem(last=False)
            self.filtered.publish(out)
            # Clearing rays use their original measurement times and sensor
            # coordinates, not the deskewed cloud's frame-start ray origin.
            # Real hardware times are grouped within 5 ms; simulated slices
            # share an exact timestamp. No current-time restamping is used.
            for b,measurement_ns in measured_times.items():
                selected=(bins==b)&accepted
                if not np.any(selected):continue
                ray_header=copy.deepcopy(msg.header)
                ray_header.stamp=Time(nanoseconds=measurement_ns).to_msg()
                self.clearing.publish(self.cloud(ray_header,observed[selected]))
            self.marking.publish(self.cloud(msg.header,points[keep&accepted]))
            return True
        except TransformException as error:
            self.get_logger().warning(str(error),throttle_duration_sec=2.)
            return False
        except (ValueError,TypeError) as error:
            self.get_logger().warning(str(error),throttle_duration_sec=2.)
            return True

    def on_scan(self,msg):
        key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if key not in self.cache or not self.fresh(msg.header):
            return
        msg.ranges=unknown_empty_bins(msg.ranges).tolist()
        self.scan.publish(msg)


def main():
    rclpy.init();node=LivoxFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
