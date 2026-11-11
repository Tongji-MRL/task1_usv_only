"""Nav2 Humble using external MAVROS localization and an existing LaserScan."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_nodes(context):
    from launch.substitutions import LaunchConfiguration
    value = lambda name: LaunchConfiguration(name).perform(context)
    share = get_package_share_directory('usv_nav2')
    use_sim_time = value('use_sim_time').lower() == 'true'
    enable_control = value('enable_control').lower() == 'true'
    map_file = os.path.abspath(os.path.expanduser(value('map')))
    if not os.path.isfile(map_file):
        raise RuntimeError('map must name an existing map YAML file: ' + map_file)
    params_file = value('params_file')
    with open(params_file, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    for section in config.values():
        if 'ros__parameters' in section:
            section['ros__parameters']['use_sim_time'] = use_sim_time
    bt = config['bt_navigator']['ros__parameters']
    bt['default_nav_to_pose_bt_xml'] = os.path.join(share, 'behavior_trees', 'navigate_to_pose.xml')
    bt['default_nav_through_poses_bt_xml'] = os.path.join(share, 'behavior_trees', 'navigate_through_poses.xml')
    bt['odom_topic'] = value('nav_odom_topic')
    # Costmaps are child nodes created by controller/planner. Pass the full
    # parameter file so their nested namespaces are configured correctly.
    from launch_ros.descriptions import ParameterFile
    from nav2_common.launch import RewrittenYaml
    from usv_nav2.launch_config import scan_topic_rewrites
    rewrites = {'use_sim_time': str(use_sim_time).lower(),
                'odom_topic': value('nav_odom_topic')}
    rewrites.update(scan_topic_rewrites(config, value('scan_topic')))
    configured_params = ParameterFile(RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=rewrites,
        convert_types=True), allow_substs=True)
    nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': map_file, 'frame_id': 'map',
                          'use_sim_time': use_sim_time}], output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[configured_params], output='screen'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[configured_params], remappings=[('cmd_vel', '/usv/cmd_vel')], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[configured_params, bt], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', parameters=[{
                 'use_sim_time': use_sim_time, 'autostart': True,
                 'node_names': ['map_server', 'planner_server', 'controller_server', 'bt_navigator'],
             }], output='screen'),
    ]
    adapter = dict(config['odom_tf']['ros__parameters'])
    adapter['odom_topic'] = value('odom_topic')
    adapter['output_odom_topic'] = value('nav_odom_topic')
    adapter['publish_tf'] = value('publish_odom_tf').lower() == 'true'
    nodes.append(Node(package='usv_nav2', executable='odom_tf', name='odom_tf',
                      parameters=[adapter], output='screen'))
    bridge = dict(config['rover_bridge']['ros__parameters'])
    bridge.update(enable_control=enable_control, odom_topic=value('odom_topic'),
                  scan_topic=value('scan_topic'))
    nodes.append(Node(package='usv_nav2', executable='rover_bridge', name='rover_bridge',
                      parameters=[bridge], output='screen'))
    if value('rviz').lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                          arguments=['-d', os.path.join(share, 'rviz', 'navigation.rviz')],
                          parameters=[{'use_sim_time': use_sim_time}], output='screen'))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('usv_nav2')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Map YAML in the same metric frame as FCU localization'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('odom_topic', default_value='/mavros/local_position/odom'),
        DeclareLaunchArgument('nav_odom_topic', default_value='/usv/odom'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        DeclareLaunchArgument('enable_control', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_nodes),
    ])
