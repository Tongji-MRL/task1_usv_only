#!/usr/bin/env python3
"""Validate measured boat settings and write preview-only navigation parameters."""
import argparse
import json
import math
import os
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
REQUIRED = (
    'USV_FCU_URL', 'USV_FCU_SYSID', 'USV_LIDAR_FRAME',
    'USV_LIDAR_X', 'USV_LIDAR_Y', 'USV_LIDAR_Z',
    'USV_LIDAR_ROLL', 'USV_LIDAR_PITCH', 'USV_LIDAR_YAW',
    'USV_FOOTPRINT', 'USV_MARK_MIN_Z', 'USV_MARK_MAX_Z',
    'USV_MAP_LAT', 'USV_MAP_LON', 'USV_MAP_YAML',
    'USV_MAX_SPEED', 'USV_MAX_YAW_RATE',
)


def settings(env):
    missing = [name for name in REQUIRED if not env.get(name, '').strip()]
    if missing:
        raise ValueError('Fill measured settings in .env.boat: ' + ', '.join(missing))

    def number(name):
        value = float(env[name])
        if not math.isfinite(value):
            raise ValueError(name + ' must be finite')
        return value

    if not 1 <= int(env['USV_FCU_SYSID']) <= 255:
        raise ValueError('USV_FCU_SYSID must be between 1 and 255')
    if env['USV_LIDAR_FRAME'] in ('map', 'odom', 'base_link', 'base_stabilized'):
        raise ValueError('USV_LIDAR_FRAME must name a separate sensor frame')
    for suffix in ('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'):
        number('USV_LIDAR_' + suffix)
    if not -90 < number('USV_MAP_LAT') < 90 or not -180 <= number('USV_MAP_LON') <= 180:
        raise ValueError('Map origin must be valid WGS84 latitude/longitude')
    if not Path(env['USV_MAP_YAML']).is_file():
        raise ValueError('Create the measured water map first: ' + env['USV_MAP_YAML'])
    hull = json.loads(env['USV_FOOTPRINT'])
    if not isinstance(hull, list) or len(hull) < 3 or any(
            not isinstance(p, list) or len(p) != 2 or
            any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in p)
            for p in hull):
        raise ValueError('USV_FOOTPRINT needs at least three finite [x,y] points')
    # Require an ordered convex envelope: it must include all vessel protrusions.
    turns = [(b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0])
             for a, b, c in zip(hull, hull[1:]+hull[:1], hull[2:]+hull[:2])]
    if not (all(x > 1e-9 for x in turns) or all(x < -1e-9 for x in turns)):
        raise ValueError('USV_FOOTPRINT must be an ordered, nondegenerate convex envelope')
    # A locally consistent turn alone also admits star polygons. Check every
    # other vertex is on the same interior side of each directed edge.
    side = 1 if turns[0] > 0 else -1
    for a, b in zip(hull, hull[1:]+hull[:1]):
        if any(side*((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])) < -1e-9
               for p in hull):
            raise ValueError('USV_FOOTPRINT edges must not cross')
    low, high = number('USV_MARK_MIN_Z'), number('USV_MARK_MAX_Z')
    if low >= high:
        raise ValueError('USV_MARK_MIN_Z must be below USV_MARK_MAX_Z')
    speed, yaw_rate = number('USV_MAX_SPEED'), number('USV_MAX_YAW_RATE')
    if not 0 < speed <= .4 or not 0 < yaw_rate <= .4:
        raise ValueError('Initial trial limits must be in (0, 0.4] m/s and rad/s')
    return hull, low, high, speed, yaw_rate


def generate(env, root=ROOT):
    hull, low, high, speed, yaw_rate = settings(env)
    cfg = yaml.safe_load((root/'src/usv_nav2/config/nav2_params.yaml').read_text())
    cfg['odom_tf']['ros__parameters']['expected_parent_frame'] = 'odom'
    cfg['rover_bridge']['ros__parameters'].update(
        expected_parent_frame='odom', enable_control=False,
        require_task_permission=False, enable_stationkeeping=False,
        max_speed=speed, max_yaw_rate=yaw_rate)
    cfg['controller_server']['ros__parameters']['FollowPath'].update(
        max_vel_x=speed, max_speed_xy=speed, max_vel_theta=yaw_rate)
    for name in ('local_costmap', 'global_costmap'):
        cost = cfg[name][name]['ros__parameters']
        cost.update(footprint=json.dumps(hull), footprint_padding=.2)
        cost['inflation_layer']['inflation_radius'] = 1.0
    cfg['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = 'odom'
    lidar = {'livox_filter': {'ros__parameters': dict(
        use_sim_time=False, footprint=json.dumps(hull), body_frame='base_link',
        stabilized_frame='base_stabilized', fixed_frame='odom', input_topic='/livox/lidar',
        marking_min_height=low, marking_max_height=high, input_timeout=.5)}}
    out = root/'runs/hardware_config'
    outputs = [(out/'nav2.yaml', cfg), (out/'livox_filter.yaml', lidar)]
    existing = [str(path) for path, _ in outputs if path.exists()]
    if existing:
        raise FileExistsError('Back up existing configuration before regenerating: ' + ', '.join(existing))
    out.mkdir(parents=True, exist_ok=True)
    for path, data in outputs:
        with path.open('x') as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
    return [str(path) for path, _ in outputs]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='validate only; write no files')
    args = parser.parse_args()
    try:
        if args.check:
            settings(os.environ)
            print('Measured settings are complete; hardware/data validity still needs runtime checks.')
        else:
            print('\n'.join(generate(os.environ)))
            print('Generated preview profile: enable_control=false, stationkeeping=false.')
    except (ValueError, TypeError, OSError) as error:
        parser.exit(2, str(error) + '\n')


if __name__ == '__main__':
    main()
