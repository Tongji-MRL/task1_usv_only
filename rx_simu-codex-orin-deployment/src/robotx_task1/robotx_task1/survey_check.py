"""Pre-recording check of the live sensor topics for the Task 1 survey.

Subscribes only: it publishes nothing and never arms, changes mode or commands
the vehicle. It reports whether the streams needed by survey_analyze (and later
by buoy_mapper) are present, fresh and correctly formatted, and lists the
topics to record.
"""
import argparse
import json
import sys
import time

import numpy as np

from . import survey_core as core

MANDATORY = ('/livox/lidar', '/usv/odom', '/tf', '/tf_static', '/mavros/state')
# Recorded when present; none of them is a command topic.
OPTIONAL = ('/mavros/local_position/odom', '/mavros/global_position/global',
            '/mavros/global_position/raw/fix', '/mavros/gpsstatus/gps1/raw',
            '/mavros/gpsstatus/gps2/raw', '/mavros/imu/data', '/usv/lidar/filtered', '/scan',
            '/usv/map_alignment', '/usv/control_status')
ENCODINGS = ('rgb8', 'bgr8', 'rgba8', 'bgra8')
GPS_TOPIC = '/mavros/gpsstatus/gps1/raw'  # mavros_extras gps_status plugin (GPS_RAW_INT).


def record_topics(available):
    """(topics to record, missing mandatory items) from a name->type map."""
    image, info = core.find_color_topics(available)
    topics = [t for t in MANDATORY if t in available]
    missing = [t for t in MANDATORY if t not in available]
    for topic, label in ((image, 'color image (.../color/image_raw)'), (info, 'color camera_info')):
        if topic:
            topics.append(topic)
        else:
            missing.append(label)
    return topics+[t for t in OPTIONAL if t in available], missing


def timing(stream):
    """(rate Hz, median header stamp minus receive time in s)."""
    stamps, receives = np.array(stream['stamps'], np.int64), np.array(stream['receives'], np.int64)
    gaps = np.diff(stamps)/1e9
    rate = 1./float(np.median(gaps)) if len(gaps) and np.median(gaps) > 0 else 0.
    return rate, float(np.median(stamps-receives))/1e9


def evaluate(seen, base_frame='base_link', odom_frame='odom'):
    """List of (PASS | WARN | FAIL, text). FAIL blocks a useful recording."""
    results = []

    def add(ok, text, level='FAIL'):
        results.append(('PASS' if ok else level, text))

    def present(key, text):
        stream = seen.get(key)
        if stream and len(stream.get('stamps', ())) >= 2:
            return stream
        add(False, text)
        return None

    lidar = present('lidar', 'No /livox/lidar messages: start the MID-360 driver '
                             '(docs/hardware_deployment.md step 8 D).')
    if lidar:
        rate, lag = timing(lidar)
        add(8. <= rate <= 12., f'LiDAR rate {rate:.1f} Hz (expected about 10).', 'WARN')
        fields = lidar['fields']
        add(all(fields.get(k) == 7 for k in 'xyz') and fields.get('timestamp') == 8,
            'LiDAR PointCloud2 has float32 x/y/z and float64 timestamp '
            f'(found {sorted(fields)}); the driver must run with xfer_format=0.')
        # livox_filter accepts a frame only if 0 <= now - stamp <= 0.5 s.
        add(-0.5 <= lag <= 0., f'LiDAR clock: header stamp is {-lag:.2f} s behind the receive time '
            '(must be 0 to 0.5 s). Otherwise the sensor clock is not synchronized and '
            'livox_filter rejects every frame.')
        offsets = lidar.get('point_offsets')
        if fields.get('timestamp') == 8:
            add(bool(offsets) and offsets[0] >= -1e-6 and offsets[1] <= 0.12,
                f'LiDAR per-point times lie within the frame (offsets {offsets} s; allowed 0 to 0.12).')
        add(seen['static'].lookup(base_frame, lidar['frame']) is not None,
            f"Mounting transform {base_frame} -> {lidar['frame']} on /tf_static. If missing, start "
            'sensor_tf.launch.py (step 8 C) or pass --lidar-xyz-rpy to survey_analyze.', 'WARN')
    odom = present('odom', 'No /usv/odom messages: start MAVROS and the navigation preview '
                           '(steps 8 A and 8 E); odom_tf relays the FCU EKF odometry.')
    if odom:
        rate, lag = timing(odom)
        add(rate >= 10., f'Odometry rate {rate:.1f} Hz (at least 10 wanted for interpolation).', 'WARN')
        add(-0.5 <= lag <= 0.05, f'Odometry stamp is {-lag:.2f} s behind the receive time (0 to 0.5 s).')
        add(list(odom['frames']) == [odom_frame, base_frame],
            f"Odometry frames {'->'.join(odom['frames'])} (expected {odom_frame}->{base_frame}).")
    image = present('image', 'No color image messages (.../color/image_raw): start the RealSense '
                             'driver with the color stream enabled.')
    if image:
        rate, lag = timing(image)
        add(rate >= 5., f"Color image rate {rate:.1f} Hz on {image['topic']} (at least 5 wanted).", 'WARN')
        add(image['encoding'] in ENCODINGS, f"Color encoding {image['encoding']} "
            f"({image['width']}x{image['height']}); supported: {', '.join(ENCODINGS)}.")
        add(abs(lag) <= 0.5, f'Color image stamp differs from the receive time by {lag:+.2f} s '
            '(within 0.5 s wanted; a larger value misplaces the lamp region).', 'WARN')
        info = seen.get('info')
        add(bool(info) and info['fx'] > 0 and (info['width'], info['height']) ==
            (image['width'], image['height']),
            f'camera_info has intrinsics for the recorded resolution (found {info}).')
        add(seen['static'].lookup(base_frame, image['frame']) is not None,
            f"Mounting transform {base_frame} -> {image['frame']} on /tf_static. If missing, publish "
            'the measured camera mounting or pass --camera-xyz-rpy to survey_analyze.', 'WARN')
    state = seen.get('state')
    if not state:
        add(False, 'No /mavros/state messages: start MAVROS (step 8 A).')
    else:
        add(bool(state['connected']), f"FCU connected={state['connected']}, mode={state['mode'] or '?'}, "
            f"armed={state['armed']} (read only; this tool never changes it).")
    gps = seen.get('gps')
    if gps:
        add(gps['fix_type'] == 6, f"GPS fix_type {gps['fix_type']} on {GPS_TOPIC} (6 = RTK fixed, "
            f"5 = RTK float, 3 = 3D); GPS yaw field {gps['yaw']} (0 = receiver gives no yaw). "
            'Landmark positions inherit the position and heading error.', 'WARN')
    else:
        add(False, f'RTK fix type not observable: {GPS_TOPIC} is not published (gps_status plugin of '
            'mavros_extras not loaded). Read the fix type on the ground station and write it in '
            'notes.md.', 'WARN')
    return results


def collect(duration):
    """Listen for `duration` seconds and return the observation dictionary."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    from .survey_bag import cloud_arrays, stamp_ns

    node = rclpy.create_node('survey_check')
    seen = dict(topics={}, lidar=None, odom=None, image=None, info=None, state=None, gps=None,
                static=core.StaticTree())

    def stream(key, message, **extra):
        entry = seen[key] or dict(stamps=[], receives=[])
        entry['stamps'].append(stamp_ns(message.header.stamp))
        entry['receives'].append(node.get_clock().now().nanoseconds)
        entry.update(extra)
        seen[key] = entry
        return entry

    def on_lidar(message):
        entry = stream('lidar', message, frame=message.header.frame_id,
                       fields={f.name: f.datatype for f in message.fields})
        try:
            times = cloud_arrays(message)[1]
        except ValueError:
            times = None
        if times is not None and len(times):
            offsets = (times-float(stamp_ns(message.header.stamp)))/1e9
            old = entry.get('point_offsets') or [float('inf'), float('-inf')]
            entry['point_offsets'] = [round(min(old[0], float(offsets.min())), 4),
                                      round(max(old[1], float(offsets.max())), 4)]

    def on_static(message):
        for item in message.transforms:
            t, q = item.transform.translation, item.transform.rotation
            seen['static'].add(item.header.frame_id, item.child_frame_id, [t.x, t.y, t.z],
                               [q.x, q.y, q.z, q.w])

    def on_info(message):
        seen['info'] = dict(fx=float(message.k[0]), width=int(message.width), height=int(message.height))

    def on_state(message):
        seen['state'] = dict(connected=bool(message.connected), armed=bool(message.armed), mode=message.mode)

    def on_gps(message):
        seen['gps'] = dict(fix_type=int(message.fix_type), yaw=getattr(message, 'yaw', None))

    latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    fixed = {'/livox/lidar': on_lidar,
             '/usv/odom': lambda m: stream('odom', m, frames=[m.header.frame_id, m.child_frame_id]),
             '/tf_static': on_static, '/mavros/state': on_state, GPS_TOPIC: on_gps}
    subscribed = set()
    deadline, end = time.monotonic()+2.0, time.monotonic()+2.0+duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() < deadline and subscribed:
                continue
            # Topics may appear late; subscribe as they are discovered.
            seen['topics'] = {name: types[0] for name, types in node.get_topic_names_and_types()}
            image, info = core.find_color_topics(seen['topics'])
            wanted = dict(fixed)
            if image and seen['topics'][image] == 'sensor_msgs/msg/Image':
                wanted[image] = lambda m, topic=image: stream(
                    'image', m, topic=topic, encoding=m.encoding, width=int(m.width),
                    height=int(m.height), frame=m.header.frame_id)
            if info:
                wanted[info] = on_info
            for topic, callback in wanted.items():
                if topic in seen['topics'] and topic not in subscribed:
                    subscribed.add(topic)
                    node.create_subscription(get_message(seen['topics'][topic]), topic, callback,
                                             latched if topic == '/tf_static' else qos_profile_sensor_data)
            deadline = time.monotonic()+1.0
    finally:
        node.destroy_node()
    return seen


def main(argv=None):
    import rclpy
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--duration', type=float, default=8.0, help='listening time in seconds')
    parser.add_argument('--json', help='write the result and the topics to record to this file')
    args = parser.parse_args(remove_ros_args(sys.argv if argv is None else ['survey_check']+argv)[1:])
    rclpy.init()
    try:
        seen = collect(args.duration)
    finally:
        rclpy.try_shutdown()
    results = evaluate(seen)
    topics, missing = record_topics(seen['topics'])
    for level, text in results:
        print(f'[{level}] {text}')
    for item in missing:
        print(f'[FAIL] Not published: {item}')
    failed = bool(missing) or any(level == 'FAIL' for level, _ in results)
    if args.json:
        seen.pop('static')
        with open(args.json, 'w') as stream:
            json.dump(dict(passed=not failed, results=results, missing=missing, record_topics=topics,
                           observed=seen), stream, indent=1)
    print('Topics to record: '+' '.join(topics))
    print('CHECK FAILED' if failed else 'CHECK PASSED (warnings above still need attention)')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
