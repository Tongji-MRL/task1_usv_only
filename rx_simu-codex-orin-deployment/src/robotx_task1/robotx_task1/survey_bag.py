"""Measure LiDAR buoy range and camera color range from a recorded survey bag.

Offline only: reads a rosbag2 directory, never publishes and never commands
the vehicle. Field procedure: docs/task1/usv_only_survey.md.

The numbers come from the recorded FCU odometry, the mounting transforms and
the sensor data alone. Which landmark is which buoy, and its true lamp state,
must be supplied by the operator in a labels file; nothing is guessed.
"""
import argparse
import json
import math
import struct
import sys
import zlib
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from . import survey_core as core

FRAME_NS = 100_000_000  # One 10 Hz Livox frame.


@dataclass
class Config:
    lidar_topic: str = '/livox/lidar'
    odom_topic: str = '/usv/odom'
    image_topic: str = ''  # Empty: the color stream found in the bag.
    info_topic: str = ''
    state_topic: str = '/mavros/state'
    gps_topic: str = '/mavros/gpsstatus/gps1/raw'  # GPS_RAW_INT via mavros_extras, if recorded.
    base_frame: str = 'base_link'
    # x y z roll pitch yaw of the sensor body in base_link (static_transform_publisher
    # convention); used only when the bag's /tf_static lacks the transform.
    lidar_mount: tuple = None
    camera_mount: tuple = None
    intrinsics: tuple = None  # fx fy cx cy, only when camera_info is missing.
    lidar_stamp: str = 'header'
    window: float = 0.5
    tolerance: float = 0.35
    min_points: int = 5
    max_extent: float = 1.0
    gate: float = 0.5
    min_windows: int = 10
    min_range: float = 1.0
    max_range: float = 30.0
    min_z: float = -1.0
    max_z: float = 2.5
    bin_width: float = 1.0
    min_rate: float = 0.9
    min_bin_windows: int = 6
    image_rate: float = 10.0
    roi_width: float = 0.5
    roi_height: float = 0.5
    lamp_offset: float = 0.0
    lamp_height: float = None  # Above base_link; default is the LiDAR cluster top.
    max_color_range: float = 25.0
    s_min: float = 0.45
    v_min: float = 0.45
    min_pixels: int = 4
    min_frames: int = 10
    labels: dict = field(default_factory=dict)
    crops_dir: str = ''
    crops_per_bin: int = 2


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def mount_pose(values):
    """(Rotation, translation) from x y z roll pitch yaw."""
    x, y, z, roll, pitch, yaw = (float(v) for v in values)
    return Rotation.from_euler('xyz', [roll, pitch, yaw]), np.array([x, y, z])


def cloud_arrays(message):
    """PointCloud2 -> xyz (n,3) and per-point absolute ns (float64) or None."""
    fields = {f.name: f for f in message.fields}
    if any(k not in fields or fields[k].datatype != 7 for k in 'xyz'):
        raise ValueError('PointCloud2 needs float32 x/y/z fields')
    order = '>' if message.is_bigendian else '<'
    names, formats = ['x', 'y', 'z'], [order+'f4']*3
    timed = 'timestamp' in fields and fields['timestamp'].datatype == 8
    if timed:
        names.append('timestamp')
        formats.append(order+'f8')
    count = int(message.width)*int(message.height)
    if not count:
        return np.empty((0, 3)), (np.empty(0) if timed else None)
    dtype = np.dtype(dict(names=names, formats=formats, offsets=[fields[n].offset for n in names],
                          itemsize=message.point_step))
    records = np.ndarray((message.height, message.width), dtype=dtype,
                         buffer=np.frombuffer(message.data, np.uint8),
                         strides=(message.row_step, message.point_step)).reshape(-1)
    xyz = np.column_stack([records[k] for k in 'xyz']).astype(float)
    return xyz, (records['timestamp'].astype(float) if timed else None)


def image_rgb(message):
    """sensor_msgs Image or CompressedImage -> HxWx3 uint8 RGB."""
    if not hasattr(message, 'encoding'):
        data = np.frombuffer(message.data, np.uint8)
        try:
            import cv2
            return cv2.imdecode(data, cv2.IMREAD_COLOR)[..., ::-1]
        except ImportError:
            import io
            from PIL import Image
            return np.asarray(Image.open(io.BytesIO(data.tobytes())).convert('RGB'))
    layout = dict(rgb8=(3, False), bgr8=(3, True), rgba8=(4, False), bgra8=(4, True))
    if message.encoding not in layout:
        raise ValueError(f'Unsupported color encoding {message.encoding!r}; '
                         f'configure the camera for one of {sorted(layout)}')
    channels, reverse = layout[message.encoding]
    rows = np.ndarray((message.height, message.step), np.uint8,
                      buffer=np.frombuffer(message.data, np.uint8))
    rgb = rows[:, :message.width*channels].reshape(message.height, message.width, channels)[..., :3]
    return rgb[..., ::-1] if reverse else rgb


def write_png(path, rgb):
    rgb = np.ascontiguousarray(rgb, np.uint8)
    height, width = rgb.shape[:2]

    def chunk(tag, data):
        return struct.pack('>I', len(data))+tag+data+struct.pack('>I', zlib.crc32(tag+data))
    raw = b''.join(b'\x00'+rgb[row].tobytes() for row in range(height))
    Path(path).write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(
        b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))+chunk(
        b'IDAT', zlib.compress(raw))+chunk(b'IEND', b''))


class TopicStats:
    def __init__(self):
        self.stamps, self.receives = [], []

    def add(self, stamp, receive):
        self.stamps.append(stamp)
        self.receives.append(receive)

    def summary(self):
        if not self.stamps:
            return dict(count=0)
        gaps = np.diff(np.array(self.stamps, np.int64))/1e9
        offset = (np.array(self.stamps, np.int64)-np.array(self.receives, np.int64))/1e9
        return dict(count=len(self.stamps),
                    rate_hz=round(1./float(np.median(gaps)), 2) if len(gaps) and np.median(gaps) > 0 else None,
                    max_gap_s=round(float(gaps.max()), 3) if len(gaps) else None,
                    # Header stamp minus bag receive time; seconds or more means the
                    # source clock is not the computer's clock.
                    stamp_minus_receive_s=round(float(np.median(offset)), 3))


class Rosbag2Bag:
    """rosbag2_py reader (ROS 2 Humble environment)."""

    def __init__(self, path):
        import rosbag2_py  # noqa: F401
        self.path = str(path)
        self.storage = 'sqlite3'
        metadata = Path(path)/'metadata.yaml'
        if metadata.is_file():
            import yaml
            info = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
            self.storage = info.get('storage_identifier') or self.storage
        self.topics = {t.name: t.type for t in self.reader().get_all_topics_and_types()}

    def reader(self):
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=self.path, storage_id=self.storage),
                    rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                                output_serialization_format='cdr'))
        return reader

    def messages(self, topics, min_period=0.0):
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        topics = [t for t in topics if t in self.topics]
        if not topics:
            return
        types = {t: get_message(self.topics[t]) for t in topics}
        reader = self.reader()
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
        last = {}
        while reader.has_next():
            topic, data, receive = reader.read_next()
            if receive-last.get(topic, -10**18) < min_period*1e9:
                continue  # Skipped before the costly deserialization.
            last[topic] = receive
            yield topic, receive, deserialize_message(data, types[topic])


# mavros_msgs is not part of the pure-Python reader's built-in definitions.
STATE_MSG = """std_msgs/Header header
bool connected
bool armed
bool guided
bool manual_input
string mode
uint8 system_status
"""


class RosbagsBag:
    """Pure-Python reader (pip package 'rosbags') for computers without ROS.

    Topics whose message type it does not know (other mavros_msgs) are skipped.
    """

    def __init__(self, path):
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_types_from_msg, get_typestore
        self.typestore = get_typestore(Stores.ROS2_HUMBLE)
        self.typestore.register(get_types_from_msg(STATE_MSG, 'mavros_msgs/msg/State'))
        self.reader = lambda: AnyReader([Path(path)], default_typestore=self.typestore)
        with self.reader() as reader:
            self.topics = {c.topic: c.msgtype for c in reader.connections}

    def messages(self, topics, min_period=0.0):
        with self.reader() as reader:
            connections = [c for c in reader.connections
                           if c.topic in topics and c.msgtype in self.typestore.types]
            if not connections:
                return
            last = {}
            for connection, receive, data in reader.messages(connections=connections):
                if receive-last.get(connection.topic, -10**18) < min_period*1e9:
                    continue
                last[connection.topic] = receive
                yield connection.topic, receive, reader.deserialize(data, connection.msgtype)


def open_bag(path):
    try:
        return Rosbag2Bag(path)
    except ImportError:
        pass
    try:
        return RosbagsBag(path)
    except ImportError:
        raise SystemExit('No bag reader: source the ROS 2 Humble environment '
                         '(scripts/hardware_env.sh), or on a computer without ROS run '
                         '"python3 -m pip install rosbags".')


def normalized_labels(labels):
    """{id: dict(truth=..., x=..., y=...)} from either strings or template entries."""
    result = {}
    for name, value in labels.items():
        entry = dict(value) if isinstance(value, dict) else dict(truth=value)
        truth = entry.get('truth') or ''
        if truth and truth != 'ignore' and truth not in core.TRUTH:
            raise ValueError(f'Label {name}: unknown truth {truth!r}; use one of '
                             f'{sorted(core.TRUTH)+["ignore"]} or leave it empty')
        if truth:
            result[name] = dict(entry, truth=truth)
    return result


def read_context(bag, cfg, problems):
    """Odometry track, mounting transforms, camera_info and vehicle state."""
    stamps, positions, quaternions, frames = [], [], [], Counter()
    tree, info, modes, armed = core.StaticTree(), None, Counter(), False
    fixes, gps_yaw = Counter(), False
    image_topic, info_topic = core.find_color_topics(bag.topics)
    image_topic, info_topic = cfg.image_topic or image_topic, cfg.info_topic or info_topic
    stats = defaultdict(TopicStats)
    wanted = [cfg.odom_topic, '/tf_static', cfg.state_topic, cfg.gps_topic]+([info_topic] if info_topic else [])
    for topic, receive, message in bag.messages(wanted):
        if topic == '/tf_static':
            for item in message.transforms:
                t, q = item.transform.translation, item.transform.rotation
                tree.add(item.header.frame_id, item.child_frame_id, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            continue
        if topic == info_topic:
            info = info or message
            continue
        stats[topic].add(stamp_ns(message.header.stamp), receive)
        if topic == cfg.odom_topic:
            p, q = message.pose.pose.position, message.pose.pose.orientation
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            if all(math.isfinite(v) for v in values) and abs(np.linalg.norm(values[3:])-1.) < 1e-3:
                stamps.append(stamp_ns(message.header.stamp))
                positions.append(values[:3])
                quaternions.append(values[3:])
                frames[f'{message.header.frame_id}->{message.child_frame_id}'] += 1
        elif topic == cfg.state_topic:
            modes[message.mode] += 1
            armed = armed or bool(message.armed)
        elif topic == cfg.gps_topic:
            fixes[str(int(message.fix_type))] += 1
            gps_yaw = gps_yaw or getattr(message, 'yaw', 0) not in (0, 65535)
    if len(stamps) < 2:
        problems.append(f'No usable odometry on {cfg.odom_topic}; nothing can be placed in a fixed frame.')
        return None
    if len(frames) != 1:
        problems.append(f'Odometry frame pairs changed during the recording: {dict(frames)}.')
    origin = stamps[0]
    track = core.PoseTrack((np.array(stamps, np.int64)-origin)/1e9, positions, quaternions)
    # GPS_RAW_INT fix_type counts (6 = RTK fixed, 5 = RTK float); empty if not recorded.
    vehicle = dict(odom_frames=sorted(frames), modes=sorted(modes), armed_seen=armed,
                   gps_fix_types=dict(fixes), gps_yaw_reported=gps_yaw)
    return dict(origin=origin, track=track, tree=tree, info=info, vehicle=vehicle, stats=stats,
                image_topic=image_topic, info_topic=info_topic)


def lidar_windows(bag, cfg, context, problems):
    """Cluster the LiDAR in fixed windows; returns (windows, summary)."""
    track, origin, stats = context['track'], context['origin'], context['stats'][cfg.lidar_topic]
    summary = dict(stamp_source=cfg.lidar_stamp, frame=None, mount_source=None, frames=0,
                   frames_used=0, frames_without_point_time=0, windows=0, start=None)
    mount, windows, pending, index = None, [], [], None

    def flush():
        middle = (index+0.5)*cfg.window
        position, _, valid = track.at([middle])
        if valid[0]:
            found = core.candidates(core.cluster(np.vstack(pending), cfg.tolerance),
                                    cfg.min_points, cfg.max_extent)
            windows.append(core.Window(middle, position[0, :2], found))

    for _, receive, message in bag.messages([cfg.lidar_topic]):
        header = stamp_ns(message.header.stamp)
        stats.add(header, receive)
        summary['frames'] += 1
        if mount is None:
            summary['frame'] = message.header.frame_id
            mount = context['tree'].lookup(cfg.base_frame, message.header.frame_id)
            summary['mount_source'] = 'tf_static'
            if mount is None and cfg.lidar_mount is not None:
                mount, summary['mount_source'] = mount_pose(cfg.lidar_mount), 'argument'
            if mount is None:
                summary['mount_source'] = None
                problems.append(
                    f'No /tf_static transform {cfg.base_frame} -> {message.header.frame_id} in the bag. '
                    'Pass the measured mounting with --lidar-xyz-rpy X Y Z ROLL PITCH YAW; '
                    'it is not guessed.')
                return [], summary
        xyz, point_ns = cloud_arrays(message)
        offsets = None if point_ns is None else point_ns-float(header)
        if offsets is None or (len(offsets) and (offsets.min() < -1e3 or offsets.max() > 1.2*FRAME_NS)):
            # Same acceptance window as livox_filter; without it the frame cannot be deskewed.
            summary['frames_without_point_time'] += 1
            offsets = np.zeros(len(xyz))
        base = header if cfg.lidar_stamp == 'header' else receive-FRAME_NS
        times = (base-origin)/1e9+offsets/1e9
        body = mount[0].apply(xyz)+mount[1]
        distance = np.hypot(body[:, 0], body[:, 1])
        keep = np.isfinite(xyz).all(axis=1) & (distance >= cfg.min_range) & (distance <= cfg.max_range)
        world, valid = core.to_world(xyz[keep], times[keep], track, *mount)
        if not track.at([(base-origin)/1e9])[2][0]:
            continue
        summary['frames_used'] += 1
        if summary['start'] is None:
            position, rotation, _ = track.at([(base-origin)/1e9])
            summary['start'] = (position[0], rotation[0].as_euler('zyx')[0])
        current = int(math.floor((base-origin)/1e9/cfg.window))
        if index is not None and current != index:
            flush()
            pending = []
        index = current
        pending.append(world[valid & (world[:, 2] >= cfg.min_z) & (world[:, 2] <= cfg.max_z)])
    if pending:
        flush()
    summary['windows'] = len(windows)
    lag = stats.summary().get('stamp_minus_receive_s')
    if lag is not None and abs(lag) > 1.0:
        if cfg.lidar_stamp == 'header':
            problems.append(
                f'LiDAR header stamps differ from the receive time by {lag:.1f} s: the sensor clock is '
                'not synchronized to the computer. livox_filter rejects such frames, so /scan and the '
                'control gate will not work until this is fixed. For an approximate range measurement '
                'from this bag only, rerun with --lidar-stamp receive.')
        else:
            problems.append(
                f'Approximate result: LiDAR frames were placed by bag receive time because the sensor '
                f'clock is off by {lag:.1f} s. The live pipeline (livox_filter) still rejects these '
                'frames; fix the clock synchronization before any controlled run.')
    if summary['frames'] and summary['frames_without_point_time'] == summary['frames']:
        problems.append('No valid per-point "timestamp" (float64 absolute ns) in the LiDAR frames: '
                        'check xfer_format=0; livox_filter requires this field.')
    return windows, summary


def describe_landmarks(windows, landmarks, summary, labels, cfg, problems):
    position, yaw = summary['start']
    verdict = core.detection_ok(cfg.min_rate, cfg.min_bin_windows)
    for landmark in landmarks:
        dx, dy = landmark['x']-position[0], landmark['y']-position[1]
        # Relative to the vessel pose at the first LiDAR frame, to identify buoys on site.
        landmark['ahead'] = round(dx*math.cos(yaw)+dy*math.sin(yaw), 2)
        landmark['left'] = round(-dx*math.sin(yaw)+dy*math.cos(yaw), 2)
        table = core.detection_table(windows, (landmark['x'], landmark['y']), cfg.gate,
                                     cfg.bin_width, cfg.max_range)
        landmark.update(table=table, stable_band=core.stable_band(table, verdict),
                        farthest_detection=core.farthest_detection(table), truth='')
        label = labels.get(landmark['id'])
        if label is None:
            continue
        if 'x' in label and math.dist((label['x'], label['y']), (landmark['x'], landmark['y'])) > 1.0:
            problems.append(f"Label {landmark['id']} ignored: the landmark moved more than 1 m from the "
                            'labelled position (different bag or parameters); relabel from the new template.')
            continue
        landmark['truth'] = label['truth']
    for name in sorted(set(labels)-{l['id'] for l in landmarks}):
        problems.append(f'Label {name} does not match any landmark in this analysis.')


def save_crop(directory, rgb, roi, name):
    u0, v0, u1, v1 = roi
    pad_u, pad_v = 2*(u1-u0), 2*(v1-v0)
    top, left = max(0, v0-pad_v), max(0, u0-pad_u)
    crop = np.array(rgb[top:v1+pad_v, left:u1+pad_u])
    scale = max(1, int(math.ceil(160/max(crop.shape[:2]))))
    crop = crop.repeat(scale, axis=0).repeat(scale, axis=1)
    a, b, c, d = ((v0-top)*scale, (v1-top)*scale-1, (u0-left)*scale, (u1-left)*scale-1)
    for rows, columns in ((slice(a, a+1), slice(c, d+1)), (slice(b, b+1), slice(c, d+1)),
                          (slice(a, b+1), slice(c, c+1)), (slice(a, b+1), slice(d, d+1))):
        crop[rows, columns] = (255, 255, 0)  # The judged region.
    Path(directory).mkdir(parents=True, exist_ok=True)
    write_png(Path(directory)/name, crop)


def camera_samples(bag, cfg, context, landmarks, problems):
    """Classify the lamp region of every landmark in every usable color frame."""
    topic, track, origin = context['image_topic'], context['track'], context['origin']
    summary = dict(image_topic=topic, encoding=None, width=None, height=None, mount_source=None,
                   intrinsics_source=None, frames=0, frames_used=0, landmarks={})
    if not topic:
        problems.append('No color image topic (.../color/image_raw) in the bag; camera range not measured.')
        return summary
    stats, camera, samples = context['stats'][topic], None, defaultdict(list)
    overlaps, white, saved = Counter(), Counter(), Counter()
    for _, receive, message in bag.messages([topic], 1./cfg.image_rate):
        header = stamp_ns(message.header.stamp)
        stats.add(header, receive)
        summary['frames'] += 1
        rgb = None
        if camera is None:
            rgb = image_rgb(message)
            summary.update(encoding=getattr(message, 'encoding', getattr(message, 'format', None)),
                           height=rgb.shape[0], width=rgb.shape[1])
            pose = context['tree'].lookup(cfg.base_frame, message.header.frame_id)
            summary['mount_source'] = 'tf_static'
            if pose is None and cfg.camera_mount is not None:
                rotation, translation = mount_pose(cfg.camera_mount)
                pose = rotation*Rotation.from_matrix(core.BODY_FROM_OPTICAL), translation
                summary['mount_source'] = 'argument'
            info, values = context['info'], cfg.intrinsics
            summary['intrinsics_source'] = 'argument' if values else 'camera_info'
            if not values and info is not None and info.k[0] > 0:
                values = (info.k[0], info.k[4], info.k[2], info.k[5])
                if (info.width, info.height) != (rgb.shape[1], rgb.shape[0]):
                    problems.append('camera_info resolution differs from the recorded image.')
                if any(abs(float(d)) > 1e-6 for d in info.d):
                    summary['distortion'] = [float(d) for d in info.d]
            if pose is None or not values:
                summary['mount_source'] = summary['mount_source'] if pose else None
                summary['intrinsics_source'] = summary['intrinsics_source'] if values else None
                if pose is None:
                    problems.append(
                        f'No /tf_static transform {cfg.base_frame} -> {message.header.frame_id} in the '
                        'bag. Pass the measured camera mounting with --camera-xyz-rpy X Y Z ROLL PITCH '
                        'YAW (camera body: x forward, y left, z up); it is not guessed.')
                if not values:
                    problems.append('No valid camera_info in the bag; pass --intrinsics FX FY CX CY.')
                return summary
            camera = core.Camera(*values, rgb.shape[1], rgb.shape[0], *pose)
        position, rotation, valid = track.at([(header-origin)/1e9])
        if not valid[0]:
            continue
        summary['frames_used'] += 1
        rois = {}
        for landmark in landmarks:
            distance = math.dist(position[0, :2], (landmark['x'], landmark['y']))
            if distance > cfg.max_color_range:
                continue
            height = cfg.lamp_height if cfg.lamp_height is not None else landmark['z_top']
            roi = camera.roi(core.level_to_body((landmark['x'], landmark['y']), height+cfg.lamp_offset,
                                                position[0], rotation[0]), cfg.roi_width, cfg.roi_height)
            if roi is not None and roi[2] > roi[0] and roi[3] > roi[1]:
                rois[landmark['id']] = (roi, distance)
        blocked = core.overlapping({name: roi for name, (roi, _) in rois.items()})
        for name, (roi, distance) in rois.items():
            if name in blocked:
                overlaps[name] += 1
                continue
            if rgb is None:
                rgb = image_rgb(message)
            result = core.classify_roi(rgb[roi[1]:roi[3], roi[0]:roi[2]], cfg.s_min, cfg.v_min,
                                       cfg.min_pixels)
            samples[name].append((distance, result['label']))
            white[name] += result['label'] == 'none' and result['white'] >= cfg.min_pixels
            key = (name, int(distance/cfg.bin_width), result['label'])
            if cfg.crops_dir and saved[key] < cfg.crops_per_bin:
                saved[key] += 1
                save_crop(cfg.crops_dir, rgb, roi, f"{name}_{distance:05.1f}m_{result['label']}_"
                                                   f"{(header-origin)/1e9:07.2f}s.png")
    for landmark in landmarks:
        name = landmark['id']
        table = core.color_table(samples[name], cfg.bin_width, cfg.max_color_range)
        entry = dict(truth=landmark['truth'], frames=len(samples[name]), overlap_skipped=overlaps[name],
                     white_only_frames=int(white[name]), table=table, color_band=None)
        if landmark['truth'] in core.TRUTH:
            entry['color_band'] = core.stable_band(table, core.color_ok(landmark['truth'], cfg.min_frames))
        summary['landmarks'][name] = entry
    return summary


def headline(per_landmark, basis):
    values = [v for v in per_landmark.values() if v is not None]
    return dict(per_landmark=per_landmark, basis=basis,
                min=min(values) if values else None,
                median=float(np.median(values)) if values else None)


def analyze(bag, cfg):
    """Run the whole measurement; bag provides .topics and .messages(topics, min_period)."""
    labels, problems = normalized_labels(cfg.labels), []
    report = dict(problems=problems, config=asdict(cfg), topics={}, vehicle={},
                  lidar=dict(landmarks=[]), camera=dict(landmarks={}),
                  headline=dict(lidar_stable_range_m=headline({}, None),
                                camera_color_range_m=headline({}, None)))
    context = read_context(bag, cfg, problems)
    if context is None:
        return report
    report['vehicle'] = context['vehicle']
    windows, summary = lidar_windows(bag, cfg, context, problems)
    landmarks = core.build_landmarks(windows, cfg.gate, cfg.min_windows) if windows else []
    if landmarks:
        describe_landmarks(windows, landmarks, summary, labels, cfg, problems)
    elif summary['mount_source'] and summary['frames_used']:
        problems.append('No persistent buoy-sized LiDAR cluster found; check --min-z/--max-z, the '
                        'mounting transform and that the vessel approached the buoys.')
    summary.pop('start')
    report['lidar'] = dict(summary, landmarks=landmarks,
                           track=[[round(float(v), 2) for v in w.boat] for w in windows[::4]])
    if landmarks:
        report['camera'] = camera_samples(bag, cfg, context, landmarks, problems)
    report['topics'] = {topic: stats.summary() for topic, stats in sorted(context['stats'].items())}
    report['duration_s'] = round(float(context['track'].times[-1]), 1)
    # Unlabelled landmarks may be quay fragments or moored boats, so the LiDAR
    # headline uses labelled buoys when labels exist. Color always needs truth.
    chosen = [l for l in landmarks if l['truth'] and l['truth'] != 'ignore']
    basis = 'labelled buoys' if chosen else 'all landmarks (no labels given)'
    report['headline'] = dict(
        lidar_stable_range_m=headline(
            {l['id']: l['stable_band']['far'] if l['stable_band'] else None
             for l in (chosen or [l for l in landmarks if l['truth'] != 'ignore'])}, basis),
        camera_color_range_m=headline(
            {name: entry['color_band']['far'] if entry['color_band'] else None
             for name, entry in report['camera']['landmarks'].items()
             if entry['truth'] in core.TRUTH and entry['truth'] != 'off'}, 'labelled lit buoys'))
    return report


def format_report(report):
    def number(value, form='{:.1f}'):
        return '-' if value is None else form.format(value)
    lines = []
    for problem in report['problems']:
        lines.append(f'PROBLEM: {problem}')
    lidar, camera = report['lidar'], report['camera']
    lines.append(f"LiDAR frames used {lidar.get('frames_used', 0)}/{lidar.get('frames', 0)}, "
                 f"windows {lidar.get('windows', 0)}, stamps from {lidar.get('stamp_source')}, "
                 f"mount from {lidar.get('mount_source')}")
    lines.append('id   truth        ahead    left  hits  size  z_bot  z_top  lidar_band_m  lidar_max  color_band_m')
    for landmark in lidar['landmarks']:
        band = landmark['stable_band']
        color = camera['landmarks'].get(landmark['id'], {}).get('color_band')
        lines.append(
            f"{landmark['id']:<4} {landmark['truth'] or '?':<11} {landmark['ahead']:>6.1f} {landmark['left']:>7.1f} "
            f"{landmark['hits']:>5} {landmark['extent']:>5.2f} {landmark['z_bottom']:>6.2f} {landmark['z_top']:>6.2f}  "
            f"{(number(band['near'])+'-'+number(band['far'])) if band else '-':>12} "
            f"{number(landmark['farthest_detection']):>10}  "
            f"{(number(color['near'])+'-'+number(color['far'])) if color else '-':>12}")
    for title, key in (('LiDAR stable range', 'lidar_stable_range_m'),
                       ('Camera color range', 'camera_color_range_m')):
        item = report['headline'][key]
        lines.append(f"{title}: min {number(item['min'])} m, median {number(item['median'])} m "
                     f"over {len(item['per_landmark'])} landmark(s) [{item['basis']}]")
    if not any(l['truth'] for l in lidar['landmarks']) and lidar['landmarks']:
        lines.append('Fill labels_template.json (truth: red, green, blue_flash, blue_steady, off, ignore) '
                     'and rerun with --labels to obtain the camera color range.')
    return '\n'.join(lines)


def plot_landmarks(report, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    figure, axis = plt.subplots(figsize=(8, 8))
    track = np.array(report['lidar'].get('track') or np.empty((0, 2)))
    if len(track):
        axis.plot(track[:, 0], track[:, 1], '-', color='0.5', label='vessel track')
        axis.plot(track[0, 0], track[0, 1], 'k>', label='start')
    colors = dict(red='red', green='green', blue_flash='blue', blue_steady='navy', off='black')
    for landmark in report['lidar']['landmarks']:
        axis.plot(landmark['x'], landmark['y'], 'o', color=colors.get(landmark['truth'], 'orange'))
        axis.annotate(landmark['id'], (landmark['x'], landmark['y']), textcoords='offset points', xytext=(5, 5))
    axis.set_aspect('equal')
    axis.grid(True)
    axis.set_xlabel('odom x east [m]')
    axis.set_ylabel('odom y north [m]')
    axis.legend(loc='best')
    figure.savefig(path, dpi=120)
    plt.close(figure)
    return True


def write_outputs(report, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory/'report.json').write_text(json.dumps(report, indent=1)+'\n')
    landmarks = report['lidar']['landmarks']
    if landmarks:
        template = {l['id']: dict(truth=l['truth'], x=round(l['x'], 2), y=round(l['y'], 2),
                                  ahead=l['ahead'], left=l['left'], hits=l['hits']) for l in landmarks}
        (directory/'labels_template.json').write_text(json.dumps(template, indent=1)+'\n')
        plot_landmarks(report, directory/'landmarks.png')


def main(argv=None):
    defaults = Config()
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('bag', help='rosbag2 directory recorded by scripts/record_task1_survey.sh')
    parser.add_argument('--output', help='result directory (default: <bag>_analysis)')
    parser.add_argument('--labels', help='labels_template.json with the truth fields filled in')
    parser.add_argument('--crops', action='store_true', help='save judged image regions for inspection')
    parser.add_argument('--lidar-xyz-rpy', dest='lidar_mount', nargs=6, type=float, metavar='V')
    parser.add_argument('--camera-xyz-rpy', dest='camera_mount', nargs=6, type=float, metavar='V')
    parser.add_argument('--intrinsics', nargs=4, type=float, metavar=('FX', 'FY', 'CX', 'CY'))
    parser.add_argument('--lidar-stamp', choices=('header', 'receive'), default='header')
    for name in ('lidar_topic', 'odom_topic', 'image_topic', 'info_topic', 'base_frame'):
        parser.add_argument('--'+name.replace('_', '-'), default=getattr(defaults, name))
    for name in ('window', 'tolerance', 'max_extent', 'gate', 'min_range', 'max_range', 'min_z', 'max_z',
                 'bin_width', 'min_rate', 'image_rate', 'roi_width', 'roi_height', 'lamp_offset',
                 'lamp_height', 'max_color_range', 's_min', 'v_min'):
        parser.add_argument('--'+name.replace('_', '-'), type=float, default=getattr(defaults, name))
    for name in ('min_points', 'min_windows', 'min_bin_windows', 'min_pixels', 'min_frames'):
        parser.add_argument('--'+name.replace('_', '-'), type=int, default=getattr(defaults, name))
    args = vars(parser.parse_args(argv))
    path = Path(args.pop('bag')).resolve()
    output = Path(args.pop('output') or f'{path}_analysis')
    labels = args.pop('labels')
    crops = args.pop('crops')
    for name in ('lidar_mount', 'camera_mount', 'intrinsics'):
        args[name] = tuple(args[name]) if args[name] else None
    cfg = Config(labels=json.loads(Path(labels).read_text()) if labels else {},
                 crops_dir=str(output/'crops') if crops else '', **args)
    report = analyze(open_bag(path), cfg)
    report['bag'] = str(path)
    write_outputs(report, output)
    print(format_report(report))
    print(f'Results: {output}')
    return 0 if report['lidar']['landmarks'] else 1


if __name__ == '__main__':
    sys.exit(main())
