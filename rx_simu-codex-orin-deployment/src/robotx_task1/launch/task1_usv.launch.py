"""USV-only Task 1: FCU odometry relay, control bridge, buoy mapper, mission, reporting.

Nav2 is deliberately not started: only one node may publish velocity commands.
MAVROS, the LiDAR driver with its mounting TF and the camera driver come from
the platform. With the default params_file (config/task1_usv_real.yaml, for
the team's Orin stack started by the USV state machine) this launch also
starts what that stack lacks, each only when its measured values are filled:
map_origin (map -> odom), the camera mounting TF, livox_filter with
pointcloud_to_laserscan (/usv/lidar/filtered, /scan), and the state-machine
link (progress, final SafePassageReport). Any launch argument given on the
command line overrides the file's task1 section. Both enable_control (bridge
output to the FCU) and enable_motion (mission output to the bridge) default to
false. See docs/task1/usv_only_runbook.md.
"""
import json
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Used when neither the command line nor the file's task1 section sets a value.
TASK = dict(
    enable_control=False, enable_motion=False, autostart=False, color_range=0.0, look_range='',
    require_rtk_fixed=False, image_topic='/camera/color/image_raw', info_topic='/camera/color/camera_info',
    min_z=-1.0, max_z=2.5, lamp_offset=0.0, cruise_speed=0.2, thread_speed=0.2, max_yaw_rate=0.2,
    boundary='', markers='', map_origin_latitude='', map_origin_longitude='', camera_frame='camera_link',
    camera_x='', camera_y='', camera_z='', camera_roll='', camera_pitch='', camera_yaw='',
    scan_range_max=20.0, enable_lidar_pipeline=True, enable_fsm_bridge=True, enable_robocommand=True,
    vehicle_id='USV1', ocs_host='', ocs_port=47800)
HELP = dict(
    enable_control='rover_bridge publishes setpoints to the FCU',
    enable_motion='usv_mission publishes velocity commands to the bridge',
    autostart='request the start at start-up instead of waiting for /task1/start',
    color_range='reliable color range measured in the survey [m]; 0 disables classification',
    look_range='stand-off for looking at a buoy [m]; default color_range - 0.5',
    require_rtk_fixed='thread gaps only with GPS fix_type 6 (needs the gps_status plugin)',
    min_z='lowest buoy point above base_link [m]', lamp_offset='lamp height above the LiDAR cluster top [m]',
    cruise_speed='must not exceed the bridge max_speed', max_yaw_rate='must not exceed the bridge max_yaw_rate',
    boundary='JSON [[x, y], ...] of the lettered boundary buoys in the map frame [m]; empty: none',
    markers='JSON [[x, y], ...] of known non-task buoys (start gate) in the map frame [m]; avoided',
    map_origin_latitude='map frame origin (WGS84); empty: map_origin is not started',
    camera_frame='camera body frame of the driver; camera_x..camera_yaw place it in base_link [m, rad]',
    enable_lidar_pipeline='start livox_filter and pointcloud_to_laserscan (needs the livox_filter section)',
    enable_fsm_bridge='send progress and the final SafePassageReport to the USV state machine',
    ocs_host='OCS address for report datagrams; empty: local only')
CAMERA = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def default_params():
    return os.path.join(get_package_share_directory('robotx_task1'), 'config', 'task1_usv_real.yaml')


def blank(value):
    return value is None or (isinstance(value, str) and not value.strip())


def flag(value):
    return value if isinstance(value, bool) else str(value).strip().lower() == 'true'


def text(value):
    """String parameter; YAML lists (boundary, markers) become JSON."""
    if blank(value):
        return ''
    return json.dumps(value) if isinstance(value, (list, dict)) else str(value)


def section(config, name):
    return dict((config.get(name) or {}).get('ros__parameters') or {})


def launch_nodes(context):
    path = LaunchConfiguration('params_file').perform(context) or default_params()
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    task = dict(TASK)
    task.update({k: v for k, v in (config.get('task1') or {}).items() if k in TASK})
    for name in TASK:
        given = LaunchConfiguration(name).perform(context)
        if given != '':
            task[name] = given
    notes = [LogInfo(msg=f'Task 1 parameters: {path}')]

    adapter = dict(section(config, 'odom_tf'), use_sim_time=False)
    bridge = dict(section(config, 'rover_bridge'), use_sim_time=False, enable_control=flag(task['enable_control']),
                  enable_stationkeeping=False, require_task_permission=False)
    color_range = float(task['color_range'])
    nodes = [
        Node(package='usv_nav2', executable='odom_tf', name='odom_tf', parameters=[adapter], output='screen'),
        Node(package='usv_nav2', executable='rover_bridge', name='rover_bridge', parameters=[bridge],
             output='screen'),
        Node(package='robotx_task1', executable='buoy_mapper', name='buoy_mapper', output='screen',
             parameters=[dict(color_range=color_range, image_topic=text(task['image_topic']),
                              info_topic=text(task['info_topic']), min_z=float(task['min_z']),
                              max_z=float(task['max_z']), lamp_offset=float(task['lamp_offset']),
                              boundary=text(task['boundary']), markers=text(task['markers']))]),
        Node(package='robotx_task1', executable='usv_mission', name='usv_mission', output='screen',
             parameters=[dict(enable_motion=flag(task['enable_motion']), autostart=flag(task['autostart']),
                              require_rtk_fixed=flag(task['require_rtk_fixed']),
                              look_range=float(task['look_range'] if not blank(task['look_range'])
                                               else max(2.0, color_range-0.5)),
                              cruise_speed=float(task['cruise_speed']), thread_speed=float(task['thread_speed']),
                              circle_speed=float(task['thread_speed']), max_yaw_rate=float(task['max_yaw_rate']),
                              boundary=text(task['boundary']))]),
    ]
    if flag(task['enable_robocommand']):
        nodes.append(Node(package='robotx_task1', executable='robocommand', name='robocommand', output='screen',
                          parameters=[dict(vehicle_id=text(task['vehicle_id']), ocs_host=text(task['ocs_host']),
                                           ocs_port=int(task['ocs_port']))]))
    if flag(task['enable_fsm_bridge']):
        nodes.append(Node(package='robotx_task1', executable='fsm_bridge', name='task1_fsm_bridge',
                          output='screen', parameters=[section(config, 'task1_fsm_bridge')]))

    latitude, longitude = task['map_origin_latitude'], task['map_origin_longitude']
    if blank(latitude) or blank(longitude):
        notes.append(LogInfo(msg='map_origin not started: map_origin_latitude/longitude are empty, '
                                 'so boundary and markers cannot be georeferenced'))
    else:
        nodes.append(Node(package='usv_sim', executable='map_origin', name='map_origin', output='screen',
                          parameters=[dict(origin_latitude=float(latitude), origin_longitude=float(longitude))]))

    mount = [task['camera_'+axis] for axis in CAMERA]
    if any(blank(v) for v in mount):
        notes.append(LogInfo(msg=f"No base_link -> {task['camera_frame']} TF published: camera_x..camera_yaw "
                                 'are not all set (no colour classification without a camera mount)'))
    else:
        arguments = [item for axis, v in zip(CAMERA, mount) for item in ('--'+axis, str(float(v)))]
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', name='camera_mount_tf',
                          output='screen', arguments=arguments+['--frame-id', 'base_link',
                                                                '--child-frame-id', text(task['camera_frame'])]))

    lidar = section(config, 'livox_filter')
    if flag(task['enable_lidar_pipeline']):
        try:
            hull = json.loads(lidar.get('footprint') or '[]')
            low, high = float(lidar['marking_min_height']), float(lidar['marking_max_height'])
        except (KeyError, TypeError, ValueError):
            hull, low, high = [], None, None
        if len(hull) < 3 or low is None:
            notes.append(LogInfo(msg='LiDAR filter and /scan not started: livox_filter footprint and '
                                     'marking heights are not set in the parameters file'))
        else:
            lidar.update(use_sim_time=False, marking_min_height=low, marking_max_height=high)
            nodes.append(Node(package='usv_sim', executable='livox_filter', name='livox_filter',
                              output='screen', parameters=[lidar]))
            # Projection of the filtered cloud; livox_filter checks it and publishes /scan.
            nodes.append(Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                              name='pointcloud_to_laserscan', output='screen',
                              remappings=[('cloud_in', '/usv/lidar/filtered'), ('scan', '/usv/scan_raw')],
                              parameters=[dict(use_sim_time=False, target_frame=lidar.get('stabilized_frame',
                                                                                          'base_stabilized'),
                                               transform_tolerance=0.1, min_height=low, max_height=high,
                                               angle_min=-3.141592653589793, angle_max=3.141592653589793,
                                               angle_increment=0.008726646259972, scan_time=0.1, range_min=0.3,
                                               range_max=float(task['scan_range_max']), use_inf=True,
                                               inf_epsilon=1.0)]))
    return notes+nodes


def generate_launch_description():
    arguments = [DeclareLaunchArgument('params_file', default_value='',
                                       description='parameters file; empty: config/task1_usv_real.yaml')]
    arguments += [DeclareLaunchArgument(name, default_value='', description=HELP.get(name, '')
                                        + f' (empty: file value, else {TASK[name]!r})') for name in TASK]
    return LaunchDescription(arguments+[OpaqueFunction(function=launch_nodes)])
