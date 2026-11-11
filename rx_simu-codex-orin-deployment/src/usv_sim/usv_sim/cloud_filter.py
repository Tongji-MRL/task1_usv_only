"""Remove the simulated vessel from LiDAR without rewriting source timestamps."""
import json
import copy
from collections import OrderedDict
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation
from .clearing_cloud import clearing_points, ray_directions


def xyz_view(msg):
    fields = {field.name: field for field in msg.fields}
    if not all(key in fields and fields[key].datatype == 7 for key in ('x', 'y', 'z')):
        raise ValueError('Expected float32 x/y/z fields')
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': [('>' if msg.is_bigendian else '<')+'f4']*3,
                      'offsets': [fields[key].offset for key in ('x', 'y', 'z')],
                      'itemsize': msg.point_step})
    records = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                         strides=(msg.row_step, msg.point_step))
    return np.stack([records[key] for key in ('x', 'y', 'z')], axis=-1)


def inside_polygon(points, polygon, padding=0.1):
    polygon = np.asarray(polygon)
    edges = np.roll(polygon, -1, axis=0)-polygon
    delta = points[..., None, :2]-polygon
    cross = edges[:, 0]*delta[..., 1]-edges[:, 1]*delta[..., 0]
    return np.all(cross >= -padding*np.linalg.norm(edges, axis=1), axis=-1)


class CloudFilter(Node):
    def __init__(self):
        super().__init__('cloud_filter')
        self.declare_parameter('scene_file', '')
        self.declare_parameter('publish_clearing_cloud',False)
        self.declare_parameter('marking_min_height',.25)
        self.declare_parameter('marking_max_height',3.)
        self.marking_height=(self.get_parameter('marking_min_height').value,self.get_parameter('marking_max_height').value)
        with open(self.get_parameter('scene_file').value) as stream:
            scene = json.load(stream)
        self.polygon = scene['footprint']
        self.offset = np.array(scene['lidar_xyz'])
        self.shape = (scene.get('lidar_beams', 16), scene.get('lidar_samples', 1875))
        self.clearing_publisher=None
        self.marking_publisher=None
        if self.get_parameter('publish_clearing_cloud').value:
            self.directions=ray_directions(self.shape,scene['lidar_vertical_min_rad'],scene['lidar_vertical_max_rad'])
            self.clearing_publisher=self.create_publisher(PointCloud2,'/usv/lidar/clearing',qos_profile_sensor_data)
            self.marking_publisher=self.create_publisher(PointCloud2,'/usv/lidar/marking',qos_profile_sensor_data)
        self.clouds = OrderedDict()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(PointCloud2, '/usv/lidar/filtered', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/usv/lidar/points', self.on_cloud, qos_profile_sensor_data)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/usv/scan_raw', self.on_scan, qos_profile_sensor_data)

    def on_cloud(self, msg):
        try:
            xyz = xyz_view(msg)
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            self.clouds[key] = (xyz, msg.header.frame_id)
            while len(self.clouds) > 20:
                self.clouds.popitem(last=False)
            finite = np.isfinite(xyz).all(axis=-1)
            # The fixed VRX LiDAR is aligned with base_link; only the known lever arm applies.
            body_points = xyz+self.offset
            remove = np.zeros(finite.shape, dtype=bool)
            remove[finite] = inside_polygon(body_points[finite], self.polygon)
            data = bytearray(msg.data)
            for field in msg.fields:
                if field.name in ('x', 'y', 'z'):
                    values = np.ndarray((msg.height, msg.width),
                                        dtype=('>' if msg.is_bigendian else '<')+'f4',
                                        buffer=data, offset=field.offset,
                                        strides=(msg.row_step, msg.point_step))
                    values[remove] = np.nan
            msg.data = bytes(data)
            msg.is_dense = False
            self.publisher.publish(msg)
            if self.clearing_publisher:
                points=clearing_points(xyz,remove,self.directions)
                clearing=copy.deepcopy(msg);data=bytearray(msg.data)
                for i,key in enumerate(('x','y','z')):
                    field=next(f for f in msg.fields if f.name==key)
                    values=np.ndarray((msg.height,msg.width),dtype=('>' if msg.is_bigendian else '<')+'f4',
                        buffer=data,offset=field.offset,strides=(msg.row_step,msg.point_step))
                    values[:]=points[...,i]
                clearing.data=bytes(data);self.clearing_publisher.publish(clearing)
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=2)

    def on_scan(self, scan):
        key = (scan.header.stamp.sec, scan.header.stamp.nanosec)
        if key not in self.clouds:
            return
        xyz, source_frame = self.clouds[key]
        # This adapter is deliberately tied to the generated organized 16-beam VRX sensor.
        if xyz.shape[:2] != self.shape:
            self.get_logger().error('Unexpected VRX LiDAR layout; no guarded scan published', throttle_duration_sec=2)
            return
        try:
            transform = self.tf.lookup_transform(scan.header.frame_id, source_frame, Time.from_msg(scan.header.stamp))
        except TransformException:
            return
        q = transform.transform.rotation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        if self.marking_publisher:
            # Use the same gravity-aligned height band as the scan converter.
            # Absolute map z has an FCU datum offset and is not water height.
            finite=xyz.reshape(-1,3);finite=finite[np.isfinite(finite).all(axis=1)]
            finite=finite[~inside_polygon(finite+self.offset,self.polygon)]
            tr=transform.transform.translation
            stable=rotation.apply(finite)+[tr.x,tr.y,tr.z]
            keep=(stable[:,2]>=self.marking_height[0])&(stable[:,2]<=self.marking_height[1])
            marked=PointCloud2();marked.header=copy.deepcopy(scan.header);marked.header.frame_id=source_frame
            marked.height=1;marked.width=int(np.count_nonzero(keep));marked.point_step=12;marked.row_step=12*marked.width
            marked.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]
            marked.is_dense=True;marked.data=np.asarray(finite[keep],dtype='<f4').tobytes()
            self.marking_publisher.publish(marked)
        angles = np.linspace(-math.pi, math.pi, xyz.shape[1])
        directions = rotation.apply(np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1))
        projected = np.arctan2(directions[:, 1], directions[:, 0])
        indices = np.floor((projected-scan.angle_min)/scan.angle_increment).astype(int)
        center = xyz[7:9]
        distance = np.linalg.norm(center, axis=-1)
        observed_free = np.all((np.isposinf(distance) | (distance >= scan.range_max)) &
                               ~np.isnan(center).any(axis=-1), axis=0)
        valid_bins = (indices >= 0) & (indices < len(scan.ranges))
        total = np.bincount(indices[valid_bins], minlength=len(scan.ranges))
        clear = np.bincount(indices[valid_bins & observed_free], minlength=len(scan.ranges))
        ranges = np.array(scan.ranges)
        # A converter's empty bin is not proof of free water. Require real no-hit rays.
        unknown = np.isposinf(ranges) & ((total == 0) | (clear != total))
        ranges[unknown] = np.nan
        scan.ranges = ranges.astype(np.float32).tolist()
        self.scan_publisher.publish(scan)


def main():
    rclpy.init()
    node = CloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
