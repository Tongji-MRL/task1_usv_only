"""USV-only Task 1 nodes in a real ROS 2 Humble environment; no hardware, no control.

Run on a separate ROS_DOMAIN_ID with the hardware environment sourced:
    python3 -m unittest discover -s src/robotx_task1/test_ros -p 'test_usv_nodes.py' -v
Catches API and message-field errors that the offline tests cannot see, and
checks that nothing moves without sensors.
"""
import json
import math
import time
import unittest

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from robotx_task1.buoy_mapper import BuoyMapper
from robotx_task1.robocommand import RoboCommand
from robotx_task1.usv_mission import UsvMission
from robotx_task1_interfaces.msg import Buoy, BuoyArray
from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


def odometry(node, x=0.0, age=0.0):
    message = Odometry()
    message.header.stamp = rclpy.time.Time(nanoseconds=node.get_clock().now().nanoseconds-int(age*1e9)).to_msg()
    message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = 1.0
    return message


def cloud(node, centers, extra=(), frame='livox_frame', age=0.05):
    """A cloud stamped `age` seconds ago, like a filtered LiDAR frame: the mapper only
    interpolates odometry, so a frame newer than the latest odometry sample is dropped."""
    points = np.array([[cx+0.15*math.cos(a), cy+0.15*math.sin(a), 0.2+0.05*i]
                       for cx, cy in centers for i, a in enumerate(np.linspace(0, 6, 12))]+list(extra),
                      dtype='<f4').reshape(-1, 3)
    message = PointCloud2()
    message.header.stamp = rclpy.time.Time(nanoseconds=node.get_clock().now().nanoseconds-int(age*1e9)).to_msg()
    message.header.frame_id = frame
    message.height, message.width, message.point_step = 1, len(points), 12
    message.row_step = 12*len(points)
    message.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
    message.data = points.tobytes()
    return message


class UsvNodeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.nodes = []
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for node in self.nodes:
            node.destroy_node()
        rclpy.try_shutdown()

    def make(self, kind, **parameters):
        node = kind(parameter_overrides=[Parameter(k, value=v) for k, v in parameters.items()])
        self.nodes.append(node)
        return node

    def test_buoy_constant_for_unclassified_exists(self):
        self.assertEqual(Buoy.UNKNOWN, 5)

    def test_mapper_builds_landmarks_from_clouds_and_stamps_with_sensor_time(self):
        mapper = self.make(BuoyMapper)  # color_range 0: LiDAR only.
        static = TransformStamped()
        static.header.frame_id, static.child_frame_id = 'base_link', 'livox_frame'
        static.transform.rotation.w = 1.0
        mapper.on_static(TFMessage(transforms=[static]))
        published, cells = [], []
        mapper.publisher = type('Publisher', (), dict(publish=lambda _, m: published.append(m)))()
        mapper.obstacle_publisher = type('Publisher', (), dict(publish=lambda _, m: cells.append(m)))()
        mapper.publish()
        self.assertEqual(published, [])  # Nothing processed yet: nothing is published.
        platform = [(3.0+0.1*i, -4.0, 0.4) for i in range(25)]  # A 2.5 m edge: not a buoy.
        for _ in range(40):
            mapper.on_odom(odometry(mapper))
            mapper.on_cloud(cloud(mapper, [(6.0, 0.0), (6.0, 1.9)], platform))
            time.sleep(0.1)
        mapper.publish()
        self.assertEqual(cells[-1].header, published[-1].header)
        self.assertEqual(cells[-1].header.frame_id, 'odom')
        self.assertGreaterEqual(cells[-1].width, 4)
        self.assertEqual(sorted(b.id for b in published[-1].buoys), ['B1', 'B2'])
        self.assertTrue(all(b.state == Buoy.UNKNOWN for b in published[-1].buoys))
        self.assertFalse(published[-1].survey_complete)
        stamp = published[-1].header.stamp
        self.assertEqual(stamp.sec*10**9+stamp.nanosec, mapper.lidar_stamp)

    def test_mapper_markers_are_obstacles_not_landmarks(self):
        # A start-gate buoy at map (16, 0); odom is 10 m east of map, so it lies at odom (6, 0).
        mapper = self.make(BuoyMapper, markers='[[16, 0]]')
        mount, origin = TransformStamped(), TransformStamped()
        mount.header.frame_id, mount.child_frame_id = 'base_link', 'livox_frame'
        mount.transform.rotation.w = 1.0
        origin.header.frame_id, origin.child_frame_id = 'map', 'odom'
        origin.transform.translation.x, origin.transform.rotation.w = 10.0, 1.0
        mapper.on_static(TFMessage(transforms=[mount, origin]))
        published, cells = [], []
        mapper.publisher = type('Publisher', (), dict(publish=lambda _, m: published.append(m)))()
        mapper.obstacle_publisher = type('Publisher', (), dict(publish=lambda _, m: cells.append(m)))()
        for _ in range(40):
            mapper.on_odom(odometry(mapper))
            mapper.on_cloud(cloud(mapper, [(6.0, 0.0), (6.0, 1.9)]))
            time.sleep(0.1)
        mapper.publish()
        self.assertEqual([(round(b.position.x, 1), round(b.position.y, 1)) for b in published[-1].buoys], [(6.0, 1.9)])
        points = np.frombuffer(bytes(cells[-1].data), dtype='<f4').reshape(-1, 3)
        self.assertTrue(any(abs(x-6.0) < 1e-3 and abs(y) < 1e-3 for x, y, _ in points), points)

    def test_mission_stays_still_without_inputs_even_after_a_start_request(self):
        mission = self.make(UsvMission, enable_motion=True)
        sent, status = [], []
        mission.command = type('Publisher', (), dict(publish=lambda _, m: sent.append(m)))()
        mission.status = type('Publisher', (), dict(publish=lambda _, m: status.append(json.loads(m.data))))()
        mission.on_start(Trigger.Request(), Trigger.Response())
        mission.tick()
        self.assertEqual((sent[-1].linear.x, sent[-1].angular.z), (0.0, 0.0))
        self.assertEqual(status[-1]['state'], 'WAIT_START')
        self.assertFalse(status[-1]['inputs_ok'])
        # Fresh inputs, but the FCU is disarmed in MANUAL: still no motion.
        mission.on_odom(odometry(mission))
        landmarks = BuoyArray()
        landmarks.header.stamp, landmarks.header.frame_id = mission.get_clock().now().to_msg(), 'odom'
        mission.on_landmarks(landmarks)
        empty = cloud(mission, [], frame='odom')
        mission.on_obstacles(empty)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = mission.get_clock().now().to_msg(), 'base_stabilized'
        scan.angle_increment, scan.ranges = 0.1, [float('nan')]*10
        mission.on_scan(scan)
        mission.on_state(State(connected=True, armed=False, mode='MANUAL'))
        mission.tick()
        self.assertTrue(status[-1]['inputs_ok'], status[-1])
        self.assertEqual((status[-1]['state'], sent[-1].linear.x, sent[-1].angular.z), ('WAIT_START', 0.0, 0.0))
        # Armed and GUIDED: the entry search starts moving; stale odometry then stops it.
        mission.on_state(State(connected=True, armed=True, mode='GUIDED'))
        for _ in range(3):
            mission.on_odom(odometry(mission))
            mission.tick()
        self.assertEqual(status[-1]['state'], 'SEARCH_ENTRY')
        self.assertTrue(sent[-1].linear.x > 0.0 or sent[-1].angular.z != 0.0, status[-1])
        self.assertLessEqual(sent[-1].linear.x, 0.3)
        mission.on_odom(odometry(mission, age=2.0))
        mission.tick()
        self.assertEqual((status[-1]['state'], sent[-1].angular.z), ('HOLD', 0.0))
        # Fresh odometry again, but the obstacle cells stopped: still no motion.
        mission.on_odom(odometry(mission))
        mission.obstacle_ns -= 3_000_000_000
        mission.tick()
        self.assertEqual((status[-1]['state'], sent[-1].angular.z), ('HOLD', 0.0))
        self.assertIn('obstacle', status[-1]['input_problem'])

    def test_boundary_must_be_georeferenced_before_motion(self):
        mission = self.make(UsvMission, boundary='[[0, 0], [30, 0], [30, 30], [0, 30]]')
        status = []
        mission.status = type('Publisher', (), dict(publish=lambda _, m: status.append(json.loads(m.data))))()
        mission.on_odom(odometry(mission))
        landmarks = BuoyArray()
        landmarks.header.stamp, landmarks.header.frame_id = mission.get_clock().now().to_msg(), 'odom'
        mission.on_landmarks(landmarks)
        mission.on_obstacles(cloud(mission, [], frame='odom'))
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = mission.get_clock().now().to_msg(), 'base_stabilized'
        scan.angle_increment, scan.ranges = 0.1, [float('nan')]*10
        mission.on_scan(scan)
        mission.on_state(State(connected=True, armed=True, mode='GUIDED'))
        mission.tick()
        self.assertFalse(status[-1]['inputs_ok'])
        self.assertIn('boundary', status[-1]['input_problem'])
        origin = TransformStamped()
        origin.header.frame_id, origin.child_frame_id = 'map', 'odom'
        origin.transform.translation.x, origin.transform.rotation.w = 10.0, 1.0
        mission.on_static(TFMessage(transforms=[origin]))
        mission.tick()
        self.assertTrue(status[-1]['inputs_ok'], status[-1])
        self.assertEqual(mission.mission.boundary[0], (-10.0, 0.0))

    def test_preview_mission_has_no_command_publisher(self):
        mission = self.make(UsvMission)
        self.assertIsNone(mission.command)
        self.assertEqual(mission.count_publishers('/usv/cmd_vel'), 0)
        self.assertIsInstance(Twist(), Twist)

    def test_reporter_withholds_heartbeat_without_position_and_sends_schema_fields(self):
        reporter = self.make(RoboCommand)
        sent = []
        reporter.local = type('Publisher', (), dict(publish=lambda _, m: sent.append(json.loads(m.data))))()
        reporter.tick()
        self.assertEqual(sent, [])
        reporter.on_fix(NavSatFix(latitude=1.28, longitude=103.85, altitude=5.0))
        reporter.on_odom(odometry(reporter))
        reporter.on_state(State(connected=True, armed=True, mode='GUIDED'))
        landmarks = BuoyArray()
        item = Buoy(id='B1', state=Buoy.FLASH_RED)
        item.position.x = 10.0
        landmarks.buoys.append(item)
        reporter.on_landmarks(landmarks)
        reporter.tick()
        self.assertEqual([p['kind'] for p in sent], ['heartbeat', 'safe_passage'])
        self.assertEqual(sent[0]['body']['state'], 'STATE_AUTO')
        self.assertEqual(sent[0]['body']['current_task'], 'TASK_NONE')
        self.assertEqual(sent[1]['body']['buoys'][0]['state'], 'BEACON_STATE_FLASHING_RED')


if __name__ == '__main__':
    unittest.main()
