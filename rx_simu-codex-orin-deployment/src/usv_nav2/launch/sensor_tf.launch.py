"""Publish measured lidar mounting extrinsics; no guessed mounting defaults."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declarations = [DeclareLaunchArgument(name, description='Measured mounting value; metres / radians')
                    for name in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]
    declarations.append(DeclareLaunchArgument('lidar_frame', description='Exact PointCloud2 / scan frame_id'))
    arguments = []
    for name in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        arguments += ['--' + name, LaunchConfiguration(name)]
    arguments += ['--frame-id', 'base_link', '--child-frame-id', LaunchConfiguration('lidar_frame')]
    return LaunchDescription(declarations + [
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='lidar_mount_tf', arguments=arguments, output='screen')])
