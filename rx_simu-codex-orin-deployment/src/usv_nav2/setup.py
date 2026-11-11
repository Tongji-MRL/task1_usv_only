from glob import glob
from setuptools import find_packages, setup

package_name = 'usv_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RobotX project',
    maintainer_email='admin@example.com',
    description='Nav2, MAVROS odometry, LaserScan and ArduPilot Rover integration.',
    license='MIT',
    entry_points={'console_scripts': [
        'odom_tf = usv_nav2.odom_tf:main',
        'rover_bridge = usv_nav2.rover_bridge:main',
        'make_water_map = usv_nav2.make_water_map:main',
    ]},
)
