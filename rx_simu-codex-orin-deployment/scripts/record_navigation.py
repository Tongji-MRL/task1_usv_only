#!/usr/bin/env python3
"""Passively retain Nav2 paths and occupancy grids with their source stamps."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    output = args.run_dir.resolve() / 'visualization' / 'raw'
    output.mkdir(parents=True, exist_ok=True)
    log = (output / 'events.jsonl').open('x')
    counts = Counter()
    previous = {}
    rclpy.init()
    node = rclpy.create_node('navigation_visualization_recorder',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    running = [True]
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: running.__setitem__(0, False))

    def receive(topic, msg):
        now = time.monotonic()
        # Local predictions run at controller frequency; retain up to 5 Hz.
        if isinstance(msg, NavPath) and topic != '/plan':
            if now - previous.get(topic, -math.inf) < 0.2:
                return
        previous[topic] = now
        event = {'topic': topic, 'wall': now,
                 'sim': node.get_clock().now().nanoseconds / 1e9,
                 'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                 'frame': msg.header.frame_id}
        if isinstance(msg, NavPath):
            event.update(kind='path', poses=[
                [p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)] for p in msg.poses])
            frames = {p.header.frame_id for p in msg.poses if p.header.frame_id}
            if frames - {msg.header.frame_id}:
                raise ValueError('Mixed path pose frames: ' + str(frames))
        else:
            name = topic.strip('/').replace('/', '_') + f'_{counts[topic]:05d}.npz'
            values = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            np.savez_compressed(output / name, data=values)
            origin = msg.info.origin
            event.update(kind='grid', file=name, resolution=msg.info.resolution,
                         width=msg.info.width, height=msg.info.height,
                         origin=[origin.position.x, origin.position.y, yaw(origin.orientation)])
        counts[topic] += 1
        log.write(json.dumps(event, allow_nan=False) + '\n')
        log.flush()

    for topic in ('/plan', '/task1/reference_path', '/local_plan', '/transformed_global_plan'):
        node.create_subscription(NavPath, topic, lambda msg, t=topic: receive(t, msg),
                                 qos_profile_sensor_data)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic in ('/map', '/global_costmap/costmap', '/local_costmap/costmap'):
        node.create_subscription(OccupancyGrid, topic, lambda msg, t=topic: receive(t, msg), qos)
    (output.parent / 'ready.json').write_text(json.dumps({'run_dir': str(args.run_dir.resolve())}) + '\n')
    print('Recording Nav2 paths and maps:', output, flush=True)
    try:
        while running[0] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        log.close()
        summary = {'counts': dict(counts), 'path_sample_limit_hz': {'/local_plan': 5, '/transformed_global_plan': 5},
                   'timestamps': 'Source stamps and monotonic receive time retained; no re-stamping.',
                   'frames': 'Raw message frames retained; plotter uses the run map_alignment.json for odom.'}
        (output.parent / 'recording.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
