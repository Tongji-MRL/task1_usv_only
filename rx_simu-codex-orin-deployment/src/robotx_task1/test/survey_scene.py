"""Synthetic approach run for survey tests: duck-typed messages, no ROS.

A vessel drives towards three buoys. The simulated LiDAR returns buoy points
only within LIDAR_LIMIT of the sensor and the simulated lamp only shows its
color within COLOR_LIMIT of the camera, so the analysis has known answers.
"""
import heapq
import math
from types import SimpleNamespace as NS

import numpy as np

EPOCH_NS = 1_800_000_000*10**9
LIDAR_LIMIT, COLOR_LIMIT = 12.0, 8.0
LIDAR_MOUNT, CAMERA_MOUNT = (0.2, 0.0, 0.4), (0.3, 0.0, 0.5)
LAMP_Z = 0.9  # Above the base_link origin.
WIDTH, HEIGHT, FOCAL = 320, 240, 300.0
BUOYS = dict(red=(20.0, 1.2), green=(20.0, -1.2), off=(23.0, 0.3))
COLORS = dict(red=(235, 30, 25), green=(20, 230, 90))
TYPES = {'/livox/lidar': 'sensor_msgs/msg/PointCloud2', '/usv/odom': 'nav_msgs/msg/Odometry',
         '/tf_static': 'tf2_msgs/msg/TFMessage', '/mavros/state': 'mavros_msgs/msg/State',
         '/mavros/gpsstatus/gps1/raw': 'mavros_msgs/msg/GPSRAW',
         '/camera/color/image_raw': 'sensor_msgs/msg/Image',
         '/camera/color/camera_info': 'sensor_msgs/msg/CameraInfo'}
POINT = np.dtype(dict(names=['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
                      formats=['<f4', '<f4', '<f4', '<f4', 'u1', 'u1', '<f8'],
                      offsets=[0, 4, 8, 12, 16, 17, 18], itemsize=26))  # Livox driver layout.


def stamp(ns):
    return NS(sec=int(ns//10**9), nanosec=int(ns % 10**9))


def header(ns, frame):
    return NS(stamp=stamp(ns), frame_id=frame)


def pose(t):
    """x, y, yaw of base_link in odom; the EKF altitude is a constant 30 m."""
    return 0.25*t, 0.3*math.sin(0.2*t), 0.15*math.sin(0.5*t)


def to_body(xy, t):
    x, y, yaw = pose(t)
    dx, dy = xy[0]-x, xy[1]-y
    return dx*math.cos(yaw)+dy*math.sin(yaw), -dx*math.sin(yaw)+dy*math.cos(yaw)


def transform(parent, child, xyz, quaternion=(0., 0., 0., 1.)):
    return NS(header=header(EPOCH_NS, parent), child_frame_id=child, transform=NS(
        translation=NS(x=xyz[0], y=xyz[1], z=xyz[2]),
        rotation=NS(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])))


class Scene:
    def __init__(self, duration=60.0, lidar_clock_offset_ns=0, static_tf=True, encoding='bgr8'):
        self.duration, self.offset, self.static_tf = duration, lidar_clock_offset_ns, static_tf
        self.encoding = encoding
        self.topics = dict(TYPES)
        if not static_tf:
            del self.topics['/tf_static']

    def odometry(self):
        for i in range(-10, int(self.duration*20)+11):
            t = i/20
            x, y, yaw = pose(t)
            ns = EPOCH_NS+int(round(t*1e9))
            yield '/usv/odom', ns+10_000_000, NS(
                header=header(ns, 'odom'), child_frame_id='base_link', pose=NS(pose=NS(
                    position=NS(x=x, y=y, z=30.0),
                    orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))))

    def clouds(self):
        rng = np.random.default_rng(1)
        for i in range(int(self.duration*10)):
            t = i/10
            rows = [(-0.3, 0.0, -0.2, 0.0)]  # A return from the vessel's own hull.
            for xy in BUOYS.values():
                if math.dist(to_body(xy, t), LIDAR_MOUNT[:2]) > LIDAR_LIMIT:
                    continue
                for dt in np.linspace(0.0, 0.095, 12):
                    angle = rng.uniform(0, 2*math.pi)
                    bx, by = to_body((xy[0]+0.2*math.cos(angle), xy[1]+0.2*math.sin(angle)), t+dt)
                    rows.append((bx-LIDAR_MOUNT[0], by-LIDAR_MOUNT[1],
                                 rng.uniform(0.0, LAMP_Z)-LIDAR_MOUNT[2], dt))
            # A continuous quay wall 14 m to port: buoy-sized points, but far too wide.
            wall = [x for x in np.arange(-10.0, 30.0, 0.15) if abs(x-pose(t)[0]) <= 6.0]
            for k, x in enumerate(wall):
                dt = 0.09*k/len(wall)
                bx, by = to_body((x, 14.0), t+dt)
                rows.append((bx-LIDAR_MOUNT[0], by-LIDAR_MOUNT[1], 0.3, dt))
            ns = EPOCH_NS+int(round(t*1e9))+self.offset
            points = np.zeros(len(rows), POINT)
            rows = np.array(rows)
            points['x'], points['y'], points['z'] = rows[:, 0], rows[:, 1], rows[:, 2]
            points['timestamp'] = ns+rows[:, 3]*1e9
            yield '/livox/lidar', ns-self.offset+110_000_000, NS(
                header=header(ns, 'livox_frame'), height=1, width=len(rows), is_bigendian=False,
                point_step=26, row_step=26*len(rows), data=points.tobytes(), fields=[
                    NS(name=name, offset=POINT.fields[name][1], count=1,
                       datatype={'<f4': 7, '|u1': 2, '<f8': 8}[POINT.fields[name][0].str])
                    for name in POINT.names])

    def image(self, t):
        rgb = np.empty((HEIGHT, WIDTH, 3), np.uint8)
        rgb[:] = (90, 100, 110)
        for name, xy in BUOYS.items():
            if name not in COLORS or int(math.floor(t)) % 2:
                continue  # Unlit buoy, or the dark second of the 1 s on / 1 s off cycle.
            bx, by = to_body(xy, t)
            depth = bx-CAMERA_MOUNT[0]
            u = WIDTH/2-FOCAL*(by-CAMERA_MOUNT[1])/depth
            v = HEIGHT/2-FOCAL*(LAMP_Z-CAMERA_MOUNT[2])/depth
            half = max(1, int(round(FOCAL*0.15/depth/2)))
            # Beyond the color limit the lamp is a washed-out bright spot.
            color = COLORS[name] if math.hypot(depth, by) <= COLOR_LIMIT else (205, 190, 190)
            rgb[max(0, int(v)-half):int(v)+half+1, max(0, int(u)-half):int(u)+half+1] = color
        return rgb

    def images(self):
        channels = 4 if self.encoding.endswith('a8') else 3
        step = WIDTH*channels+4  # Row padding must be ignored by the decoder.
        for i in range(int(self.duration*5)):
            t = i/5
            pixels = self.image(t)
            if channels == 4:
                pixels = np.dstack([pixels, np.full((HEIGHT, WIDTH), 255, np.uint8)])
            if self.encoding.startswith('bgr'):
                pixels = pixels[..., [2, 1, 0]+[3]*(channels-3)]
            data = np.zeros((HEIGHT, step), np.uint8)
            data[:, :WIDTH*channels] = pixels.reshape(HEIGHT, -1)
            ns = EPOCH_NS+int(round(t*1e9))
            yield '/camera/color/image_raw', ns+30_000_000, NS(
                header=header(ns, 'camera_color_optical_frame'), height=HEIGHT, width=WIDTH,
                encoding=self.encoding, step=step, data=data.tobytes())

    def slow(self):
        if self.static_tf:
            yield '/tf_static', EPOCH_NS, NS(transforms=[
                transform('base_link', 'livox_frame', LIDAR_MOUNT),
                transform('base_link', 'camera_link', CAMERA_MOUNT),
                transform('camera_link', 'camera_color_optical_frame', (0., 0., 0.),
                          (-0.5, 0.5, -0.5, 0.5))])
        yield '/camera/color/camera_info', EPOCH_NS, NS(
            header=header(EPOCH_NS, 'camera_color_optical_frame'), width=WIDTH, height=HEIGHT,
            k=[FOCAL, 0., WIDTH/2, 0., FOCAL, HEIGHT/2, 0., 0., 1.], d=[0.]*5)
        for second in range(int(self.duration)):
            ns = EPOCH_NS+second*10**9
            yield '/mavros/state', ns, NS(header=header(ns, ''), connected=True, armed=False,
                                         mode='MANUAL')
            yield '/mavros/gpsstatus/gps1/raw', ns, NS(header=header(ns, 'base_link'),
                                                      fix_type=6 if second >= 5 else 5, yaw=9000)

    def messages(self, topics, min_period=0.0):
        streams = [self.odometry(), self.clouds(), self.images(), self.slow()]
        last = {}
        for topic, receive, message in heapq.merge(*streams, key=lambda item: item[1]):
            if topic not in topics or receive-last.get(topic, -10**18) < min_period*1e9:
                continue
            last[topic] = receive
            yield topic, receive, message
