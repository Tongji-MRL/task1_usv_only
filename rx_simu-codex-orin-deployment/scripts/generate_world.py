#!/usr/bin/env python3
"""Generate an isolated VRX world and conservative footprint from collision geometry."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import sdformat13 as sdf

ROOT = Path(__file__).resolve().parents[1]


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def set_text(parent, tag, text):
    child = parent.find(tag)
    if child is None:
        child = element(parent, tag)
    child.text = str(text)


def footprint(model_file):
    root = sdf.Root()
    errors = root.load(str(model_file))
    if errors:
        raise RuntimeError(errors)
    model = root.model()
    points = []
    for i in range(model.link_count()):
        link = model.link_by_index(i)
        for j in range(link.collision_count()):
            collision = link.collision_by_index(j)
            pose = link.semantic_pose().resolve() * collision.semantic_pose().resolve()
            q, p = pose.rot(), pose.pos()
            rotation = Rotation.from_quat([q.x(), q.y(), q.z(), q.w()])
            geometry = collision.geometry()
            if geometry.box_shape() is not None:
                size = geometry.box_shape().size()
                vertices = [[x * size.x()/2, y * size.y()/2, z * size.z()/2]
                            for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
            elif geometry.cylinder_shape() is not None:
                cylinder = geometry.cylinder_shape()
                # Circumscribed polygon encloses the cylinder, including propellers.
                radius = cylinder.radius() / math.cos(math.pi / 32)
                vertices = [[radius*math.cos(t), radius*math.sin(t), z*cylinder.length()/2]
                            for t in np.linspace(0, 2*math.pi, 32, endpoint=False)
                            for z in (-1, 1)]
            elif geometry.sphere_shape() is not None:
                radius = geometry.sphere_shape().radius()
                vertices = [[x*radius, y*radius, z*radius]
                            for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
            else:
                raise ValueError('Unsupported collision geometry: ' + collision.name())
            points.extend((rotation.apply(vertices) + [p.x(), p.y(), p.z()])[:, :2])
    points = np.array(points)
    return points[ConvexHull(points).vertices].round(5).tolist()


def box(world, name, x, y, sx=2.0, sy=2.0, height=3.0):
    model = element(world, 'model', name=name)
    element(model, 'static', 'true')
    element(model, 'pose', f'{x} {y} {height/2} 0 0 0')
    link = element(model, 'link', name='body')
    for kind in ('visual', 'collision'):
        part = element(link, kind, name=kind)
        element(element(element(part, 'geometry'), 'box'), 'size', f'{sx} {sy} {height}')
        if kind == 'visual':
            mat = element(part, 'material')
            element(mat, 'diffuse', '0.9 0.25 0.1 1')
    return {'name': name, 'x': x, 'y': y, 'size': [sx, sy]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT/'runs/generated')
    parser.add_argument('--wind', type=float, default=0.0)
    parser.add_argument('--wave-gain', type=float, default=0.0)
    parser.add_argument('--lockstep', action='store_true')
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    vrx = Path(os.environ.get('VRX_ROOT', '/home/zjy/vrx_docking'))
    source = vrx/'vrx_urdf/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro'
    urdf = subprocess.check_output([
        'xacro', str(source), 'namespace:=wamv', 'locked:=false', 'thruster_config:=H',
        'vrx_sensors_enabled:=false', 'camera_enabled:=false', 'gps_enabled:=false',
        'imu_enabled:=false', 'lidar_enabled:=true', 'ground_truth_enabled:=true'])
    (out/'wamv.urdf').write_bytes(urdf)
    base = subprocess.check_output(['gz', 'sdf', '--force-version', '13', '-p', str(out/'wamv.urdf')])
    (out/'wamv_base.sdf').write_bytes(base)
    polygon = footprint(out/'wamv_base.sdf')
    boat = ET.fromstring(base).find('model')
    boat.set('name', 'wamv')
    element(boat, 'pose', '0 0 0.1 0 0 0')
    for plugin in list(boat.findall('plugin')):
        name = plugin.get('name', '')
        if any(key in name for key in ('DetachableJoint', 'PosePublisher', 'JointStatePublisher')):
            boat.remove(plugin)
        elif name.endswith('Thruster'):
            set_text(plugin, 'max_thrust_cmd', 300)
            set_text(plugin, 'min_thrust_cmd', -300)
        elif name.endswith('OdometryPublisher'):
            set_text(plugin, 'odom_publish_frequency', 50)
            set_text(plugin, 'odom_topic', '/usv/ground_truth')
            set_text(plugin, 'dimensions', 3)
    # A dedicated origin link stays FLU; the IMU sensor itself is rotated to FRD.
    link = element(boat, 'link', name='sitl_imu_link')
    element(link, 'pose', '0 0 0 0 0 0')
    inertial = element(link, 'inertial')
    element(inertial, 'mass', 0.001)
    inertia = element(inertial, 'inertia')
    for name in ('ixx', 'iyy', 'izz'):
        element(inertia, name, 0.000001)
    joint = element(boat, 'joint', name='sitl_imu_joint', type='fixed')
    element(joint, 'parent', 'wamv/base_link')
    element(joint, 'child', 'sitl_imu_link')
    imu = element(link, 'sensor', name='sitl_imu', type='imu')
    element(imu, 'pose', f'0 0 0 {math.pi} 0 0')
    element(imu, 'always_on', 'true')
    element(imu, 'update_rate', 250)
    element(imu, 'imu')
    plugin = element(boat, 'plugin', name='ArduPilotPlugin', filename='libArduPilotPlugin.so')
    for key, value in {
        'fdm_addr': '127.0.0.1', 'fdm_port_in': 9002,
        'have_32_channels': 'false', 'lock_step': str(args.lockstep).lower(),
        'no_time_sync': 'true', 'connectionTimeoutMaxCount': 5,
        'servo_frame_topic': '/usv/sitl_frame',
        'imuName': 'sitl_imu',
        'modelXYZToAirplaneXForwardZDown': f'0 0 0 {math.pi} 0 0',
        'gazeboXYZToNED': f'0 0 0 {math.pi} 0 {math.pi/2}',
    }.items():
        element(plugin, key, value)
    for channel, side in ((0, 'left'), (2, 'right')):
        control = element(plugin, 'control', channel=str(channel))
        for key, value in {
            'jointName': f'wamv/{side}_engine_propeller_joint', 'type': 'COMMAND',
            'cmd_topic': f'/wamv/thrusters/{side}/thrust',
            'servo_min': 1000, 'servo_max': 2000, 'multiplier': 600, 'offset': -0.5,
        }.items():
            element(control, key, value)
    sensor = boat.find(".//sensor[@name='lidar_wamv_sensor']")
    set_text(sensor, 'topic', '/usv/lidar')
    sensor.find('.//horizontal/samples').text = '720'
    # Keep all collision sensors for evaluation; no ground-truth TF enters Nav2.
    contact = boat.find(".//sensor[@type='contact']")
    if contact is not None:
        set_text(contact, 'topic', '/usv/contacts')

    tree = ET.parse(vrx/'vrx_gz/worlds/safe_docking_task.sdf')
    world = tree.getroot().find('world')
    world.set('name', 'usv_nav')
    for include in list(world.findall('include')):
        if include.findtext('uri') != 'coast_waves':
            world.remove(include)
    for p in list(world.findall('plugin')):
        if 'ScoringPlugin' in p.get('name', ''):
            world.remove(p)
        elif p.get('name') == 'vrx::USVWind':
            set_text(p, 'wind_mean_velocity', args.wind)
        elif p.get('name') == 'vrx::PublisherPlugin':
            p.find('message').text = '\n'.join(
                f'params {{ key: "{key}" value {{ type: DOUBLE double_value: {value} }} }}'
                for key, value in [('direction', 0), ('gain', args.wave_gain), ('period', 5), ('steepness', 0)])
    for pose in world.findall("gui/plugin/camera_pose"):
        pose.text = '-12 -18 15 0 0.55 0.95'
    dock = element(world, 'include')
    element(dock, 'uri', 'dock_2026_safe')
    element(dock, 'pose', '30 25 0 0 0 0')
    obstacles = [box(world, 'static_obstacle', 10, 0), box(world, 'dynamic_obstacle', 15, 20)]
    world.append(boat)
    observer = element(world, 'model', name='overview_camera')
    element(observer, 'static', 'true')
    element(observer, 'pose', '-15 -20 18 0 0.6 0.93')
    camera_link = element(observer, 'link', name='camera_link')
    camera_sensor = element(camera_link, 'sensor', name='overview', type='camera')
    element(camera_sensor, 'always_on', 'true')
    element(camera_sensor, 'update_rate', 1)
    element(camera_sensor, 'topic', '/usv/overview')
    camera = element(camera_sensor, 'camera')
    element(camera, 'horizontal_fov', 1.3)
    image = element(camera, 'image')
    element(image, 'width', 960)
    element(image, 'height', 540)
    element(image, 'format', 'R8G8B8')
    clip = element(camera, 'clip')
    element(clip, 'near', 0.1)
    element(clip, 'far', 200)
    ET.indent(tree)
    tree.write(out/'usv_nav.sdf', encoding='utf-8', xml_declaration=True)
    metadata = {
        'world': 'usv_nav', 'origin': [-33.724223, 150.679736, 0],
        'footprint': polygon, 'lidar_xyz': [0.7, -0.3, 1.8], 'obstacles': obstacles,
        'lidar_samples': 720, 'lidar_beams': 16,
        'lidar_vertical_min_rad': float(sensor.findtext('.//vertical/min_angle')),
        'lidar_vertical_max_rad': float(sensor.findtext('.//vertical/max_angle')),
        'wind_mps': args.wind, 'wave_gain': args.wave_gain, 'wave_period_s': 5,
        'thrust_limit_n': 300, 'lockstep': args.lockstep,
        'vrx_revision': subprocess.check_output(['git', '-C', str(vrx), 'rev-parse', 'HEAD'], text=True).strip(),
        'urdf_sha256': hashlib.sha256(urdf).hexdigest(),
    }
    (out/'scene.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps({'world': str(out/'usv_nav.sdf'), 'footprint': polygon}))


if __name__ == '__main__':
    main()
