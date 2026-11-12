"""Buoy landmark map from the filtered MID-360 cloud and the forward color camera.

Publishes /task1/landmarks (BuoyArray, odometry frame) and /task1/obstacles
(PointCloud2, 0.5 m cells of LiDAR objects too large to be a buoy, such as the
Task 3 platform; same frame and stamp). The LiDAR gives position
and identity, the camera only the lamp color. The header stamp is the older of
the last processed LiDAR frame and the last processed image: if either sensor
stops, the message ages and the mission holds. Nothing is restamped.
"""
import json
import math
from array import array
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from robotx_task1_interfaces.msg import Buoy, BuoyArray
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from . import survey_core as core
from .landmark_map import LandmarkMap, ObstacleGrid
from .survey_bag import cloud_arrays, image_rgb, stamp_ns

DEFAULTS = dict(
    lidar_topic='/usv/lidar/filtered', odom_topic='/usv/odom', image_topic='/camera/color/image_raw',
    info_topic='/camera/color/camera_info', base_frame='base_link', odom_frame='odom',
    window=0.5, tolerance=0.35, min_points=5, max_extent=1.0, gate=0.5,
    min_range=1.0, max_range=20.0, min_z=-1.0, max_z=2.5,
    # Reliable color range measured in the survey (plan step 1). 0 disables the camera:
    # every buoy then stays unclassified and the mission will not pass any of them.
    color_range=0.0, roi_width=0.5, roi_height=0.5, lamp_offset=0.0,
    s_min=0.45, v_min=0.45, min_pixels=4, input_timeout=0.5, publish_rate=10.0,
    # Course boundary: JSON [[x, y], ...] of the lettered buoys in boundary_frame (map: fixed by
    # the configured map origin). Clusters outside it, or within corner_radius of a lettered
    # buoy, are ignored, so the boundary buoys never become landmarks. Empty: no boundary.
    boundary='', boundary_frame='map', corner_radius=1.0,
    # Known buoys that are not Task 1 buoys (e.g. the start gate), JSON [[x, y], ...] in
    # boundary_frame: never landmarks, published as obstacle points so the mission avoids them.
    markers='')


class BuoyMapper(Node):
    def __init__(self, **kwargs):
        super().__init__('buoy_mapper', **kwargs)
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.boundary = json.loads(self.p['boundary']) if self.p['boundary'] else None
        self.marker_source = json.loads(self.p['markers']) if self.p['markers'] else []
        self.markers = []
        self.boundary_ready = self.boundary is None and not self.marker_source
        self.map = LandmarkMap(gate=self.p['gate'], corner_radius=self.p['corner_radius'])
        self.grid = ObstacleGrid(corner_radius=self.p['corner_radius'])
        self.tree, self.poses, self.camera, self.info = core.StaticTree(), deque(maxlen=400), None, None
        self.origin_ns, self.window_index, self.pending = None, None, []
        self.lidar_stamp = self.image_stamp = None
        self.counts = dict(clouds=0, clouds_without_pose=0, images=0, images_without_pose=0, labels=0)
        if self.p['color_range'] <= 0.0:
            self.get_logger().warning('color_range is 0: camera classification is disabled, all buoys stay UNKNOWN')
        latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(BuoyArray, '/task1/landmarks', 10)
        self.obstacle_publisher = self.create_publisher(PointCloud2, '/task1/obstacles', 10)
        self.status = self.create_publisher(String, '/task1/mapper_status', 10)
        self.create_subscription(TFMessage, '/tf_static', self.on_static, latched)
        self.create_subscription(Odometry, self.p['odom_topic'], self.on_odom, 50)
        self.create_subscription(PointCloud2, self.p['lidar_topic'], self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.p['info_topic'], self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.p['image_topic'], self.on_image, qos_profile_sensor_data)
        self.create_timer(1.0/self.p['publish_rate'], self.publish)

    def seconds(self, ns):
        if self.origin_ns is None:
            self.origin_ns = ns
        return (ns-self.origin_ns)/1e9

    def fresh(self, ns):
        age = (self.get_clock().now().nanoseconds-ns)/1e9
        return 0.0 <= age <= self.p['input_timeout']

    def on_static(self, message):
        for item in message.transforms:
            t, q = item.transform.translation, item.transform.rotation
            self.tree.add(item.header.frame_id, item.child_frame_id, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])

    def on_odom(self, message):
        if message.header.frame_id != self.p['odom_frame'] or message.child_frame_id != self.p['base_frame']:
            self.get_logger().error('Unexpected odometry frames; ignored', throttle_duration_sec=5.0)
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if all(math.isfinite(v) for v in values):
            self.poses.append((self.seconds(stamp_ns(message.header.stamp)), values[:3], values[3:]))

    def pose_at(self, time):
        """(position, Rotation) interpolated at a source time, or None."""
        if len(self.poses) < 2:
            return None
        try:
            track = core.PoseTrack([s[0] for s in self.poses], [s[1] for s in self.poses],
                                   [s[2] for s in self.poses])
        except ValueError:
            return None
        position, rotation, valid = track.at([time])
        return (position[0], rotation[0]) if valid[0] else None

    def apply_boundary(self):
        """Move the configured boundary and markers into the odometry frame once map -> odom is known."""
        points = (self.boundary or [])+self.marker_source
        moved = core.polygon_in_frame(self.tree, points, self.p['odom_frame'], self.p['boundary_frame'])
        if moved is None:
            self.get_logger().warning(f"No {self.p['boundary_frame']} -> {self.p['odom_frame']} transform on "
                                      '/tf_static yet (map_origin); LiDAR frames are not used until then',
                                      throttle_duration_sec=5.0)
            return False
        count = len(self.boundary or [])
        polygon = moved[:count] if self.boundary else None
        self.markers = moved[count:]
        self.map.boundary, self.grid.boundary, self.boundary_ready = polygon, polygon, True
        self.map.markers = self.grid.markers = self.markers
        self.get_logger().info(f'Course boundary in {self.p["odom_frame"]}: {polygon}; markers: {self.markers}')
        return True

    def on_cloud(self, message):
        ns = stamp_ns(message.header.stamp)
        if not self.fresh(ns):
            return
        if not self.boundary_ready and not self.apply_boundary():
            return
        self.counts['clouds'] += 1
        mount = self.tree.lookup(self.p['base_frame'], message.header.frame_id)
        time = self.seconds(ns)
        pose = self.pose_at(time)
        if mount is None or pose is None:
            self.counts['clouds_without_pose'] += 1
            self.get_logger().warning('No mounting transform or no odometry at the LiDAR stamp',
                                      throttle_duration_sec=5.0)
            return
        try:
            xyz = cloud_arrays(message)[0]
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.0)
            return
        # The filtered cloud is already deskewed to its header stamp by livox_filter.
        body = mount[0].apply(xyz)+mount[1]
        distance = np.hypot(body[:, 0], body[:, 1])
        level = pose[1].apply(body[(distance >= self.p['min_range']) & (distance <= self.p['max_range'])])
        level = level[(level[:, 2] >= self.p['min_z']) & (level[:, 2] <= self.p['max_z'])]
        index = int(math.floor(time/self.p['window']))
        if self.window_index is not None and index != self.window_index and self.pending:
            clusters, groups = core.cluster(np.vstack(self.pending), self.p['tolerance'], members=True)
            found = core.candidates(clusters, self.p['min_points'], self.p['max_extent'])
            self.map.update_lidar(time, pose[0][:2], found)
            # Too large for a buoy: platform, dock. Kept as obstacle cells for the planner.
            large = [g[:, :2] for c, g in zip(clusters, groups)
                     if c.extent > self.p['max_extent'] and c.count >= self.p['min_points']]
            self.grid.update(np.vstack(large) if large else np.empty((0, 2)))
            self.pending = []
        self.window_index = index
        self.pending.append(np.column_stack([level[:, :2]+pose[0][:2], level[:, 2]]))
        self.lidar_stamp = ns

    def on_info(self, message):
        if message.k[0] > 0:
            self.info = message

    def on_image(self, message):
        ns = stamp_ns(message.header.stamp)
        if self.p['color_range'] <= 0.0 or not self.fresh(ns):
            return
        self.counts['images'] += 1
        pose = self.pose_at(self.seconds(ns))
        if self.camera is None and self.info is not None:
            mount = self.tree.lookup(self.p['base_frame'], message.header.frame_id)
            if mount is not None and (self.info.width, self.info.height) == (message.width, message.height):
                k = self.info.k
                self.camera = core.Camera(k[0], k[4], k[2], k[5], message.width, message.height, *mount)
        if pose is None or self.camera is None:
            self.counts['images_without_pose'] += 1
            self.get_logger().warning('No camera model, mounting transform or odometry at the image stamp',
                                      throttle_duration_sec=5.0)
            return
        rois = {}
        for landmark in self.map.snapshot():
            if math.dist(pose[0][:2], (landmark['x'], landmark['y'])) > self.p['color_range']:
                continue
            roi = self.camera.roi(core.level_to_body((landmark['x'], landmark['y']),
                                                     landmark['z_top']+self.p['lamp_offset'], pose[0], pose[1]),
                                  self.p['roi_width'], self.p['roi_height'])
            if roi is not None and roi[2] > roi[0] and roi[3] > roi[1]:
                rois[landmark['id']] = roi
        blocked = core.overlapping(rois)
        try:
            rgb = image_rgb(message) if set(rois)-blocked else None
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.0)
            return
        for name, roi in rois.items():
            if name in blocked:
                continue  # Two lamps in one region: neither is judged in this frame.
            label = core.classify_roi(rgb[roi[1]:roi[3], roi[0]:roi[2]], self.p['s_min'], self.p['v_min'],
                                      self.p['min_pixels'])['label']
            self.counts['labels'] += 1
            if self.map.observe_color(name, self.seconds(ns), label):
                self.get_logger().info(f'{name} classified')
        self.image_stamp = ns

    def publish(self):
        stamps = [self.lidar_stamp]+([self.image_stamp] if self.p['color_range'] > 0.0 else [])
        if any(s is None for s in stamps):
            return
        message = BuoyArray()
        message.header.stamp = Time(nanoseconds=min(stamps)).to_msg()
        message.header.frame_id = self.p['odom_frame']
        message.revision = self.map.revision
        message.survey_complete = False  # Never claimed: the map is built while under way.
        for landmark in self.map.snapshot():
            item = Buoy()
            item.id, item.state = landmark['id'], landmark['state']
            item.position.x, item.position.y, item.position.z = landmark['x'], landmark['y'], landmark['z_top']
            item.collision_radius = landmark['radius']
            message.buoys.append(item)
        self.publisher.publish(message)
        cells = np.array(self.grid.points()+[tuple(q) for q in self.markers], dtype='<f4').reshape(-1, 2)
        obstacles = PointCloud2()
        obstacles.header = message.header
        obstacles.height, obstacles.width = 1, len(cells)
        obstacles.point_step, obstacles.row_step = 12, 12*len(cells)
        obstacles.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                            for i, n in enumerate('xyz')]
        # array('B'): the message's fast path; assigning bytes type-checks every byte.
        obstacles.data = array('B', np.column_stack([cells, np.zeros(len(cells), '<f4')]).astype('<f4').tobytes())
        obstacles.is_dense = True
        self.obstacle_publisher.publish(obstacles)
        self.status.publish(String(data=json.dumps(dict(
            self.counts, landmarks=len(message.buoys), obstacle_cells=len(cells), rejected=self.map.rejected,
            camera_ready=self.camera is not None, revision=self.map.revision))))


def main(args=None):
    rclpy.init(args=args)
    node = BuoyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
