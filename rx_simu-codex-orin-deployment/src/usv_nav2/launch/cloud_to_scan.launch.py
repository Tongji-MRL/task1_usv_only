"""Optional PointCloud2 projection. Existing /scan users do not launch this."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    numeric = lambda name: ParameterValue(arg(name), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('target_frame', default_value='base_stabilized'),
        DeclareLaunchArgument('min_height', default_value='-0.5'),
        DeclareLaunchArgument('max_height', default_value='1.5'),
        DeclareLaunchArgument('range_min', default_value='0.3'),
        DeclareLaunchArgument('range_max', default_value='20.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan',
             remappings=[('cloud_in', arg('cloud_topic')), ('scan', arg('scan_topic'))],
             parameters=[{
                 'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool),
                 'target_frame': arg('target_frame'),
                 'transform_tolerance': 0.1,
                 'min_height': numeric('min_height'), 'max_height': numeric('max_height'),
                 'angle_min': -3.141592653589793, 'angle_max': 3.141592653589793,
                 'angle_increment': 0.008726646259972,
                 'scan_time': 0.1,
                 'range_min': numeric('range_min'), 'range_max': numeric('range_max'),
                 'use_inf': True, 'inf_epsilon': 1.0,
             }], output='screen'),
    ])
