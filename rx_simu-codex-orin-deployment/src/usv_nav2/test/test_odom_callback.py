"""Callback unit checks with message doubles; these do not exercise ROS or DDS."""

import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch


def vector(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


class Transform:
    def __init__(self):
        self.header = NS(stamp=None, frame_id='')
        self.child_frame_id = ''
        self.transform = NS(translation=vector(), rotation=NS(x=0., y=0., z=0., w=1.))


def load_callback():
    dependencies = {
        'rclpy': {}, 'rclpy.node': {'Node': object},
        'rclpy.qos': {'QoSProfile': Mock(), 'ReliabilityPolicy': NS(RELIABLE=1),
                      'qos_profile_sensor_data': object()},
        'geometry_msgs': {}, 'geometry_msgs.msg': {'TransformStamped': Transform},
        'nav_msgs': {}, 'nav_msgs.msg': {'Odometry': object},
        'tf2_ros': {'TransformBroadcaster': Mock()},
    }
    doubles = {}
    for name, attributes in dependencies.items():
        doubles[name] = ModuleType(name)
        doubles[name].__dict__.update(attributes)
    path = Path(__file__).resolve().parents[1] / 'usv_nav2/odom_tf.py'
    spec = importlib.util.spec_from_file_location('usv_nav2._odom_callback_tests', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, doubles):
        spec.loader.exec_module(module)
    return module.OdomTF


class OdometryCallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node_class = load_callback()

    def setUp(self):
        self.node = self.node_class.__new__(self.node_class)
        self.node.parent, self.node.body, self.node.stabilized = 'map', 'base_link', 'base_stabilized'
        self.node.max_age = 0.5
        self.node.tf = Mock()
        self.node.odom_pub = Mock()
        self.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
        self.node.get_logger = lambda: Mock()

    def message(self):
        return NS(
            header=NS(frame_id='map', stamp=NS(sec=9, nanosec=900_000_000)),
            child_frame_id='base_link',
            pose=NS(pose=NS(position=vector(3., -2., .1),
                            orientation=NS(x=0., y=0., z=0., w=1.)), covariance=[0.01] * 36),
            twist=NS(twist=NS(linear=vector(.3), angular=vector(z=.2)), covariance=[0.02] * 36))

    def test_disabling_tf_preserves_unchanged_odometry_relay(self):
        self.node.tf = None
        msg = self.message()
        self.node.on_odom(msg)
        self.assertIs(self.node.odom_pub.publish.call_args.args[0], msg)
        self.assertEqual(msg.header.stamp.nanosec, 900_000_000)
        self.assertEqual(msg.twist.twist.linear.x, .3)

    def test_tf_chain_uses_source_stamp_and_position(self):
        msg = self.message()
        self.node.on_odom(msg)
        level, body = self.node.tf.sendTransform.call_args.args[0]
        self.assertEqual((level.header.frame_id, level.child_frame_id), ('map', 'base_stabilized'))
        self.assertEqual((body.header.frame_id, body.child_frame_id), ('base_stabilized', 'base_link'))
        self.assertIs(level.header.stamp, msg.header.stamp)
        self.assertIs(body.header.stamp, msg.header.stamp)
        self.assertEqual((level.transform.translation.x, level.transform.translation.y), (3., -2.))

    def test_invalid_input_neither_relays_nor_publishes_tf(self):
        bad_messages = [self.message() for _ in range(5)]
        bad_messages[0].header.frame_id = 'other_map'
        bad_messages[1].header.stamp.sec = 8
        bad_messages[2].pose.pose.orientation.w = 0.
        bad_messages[3].twist.twist.linear.x = math.nan
        bad_messages[4].child_frame_id = 'other_body'
        for msg in bad_messages:
            with self.subTest(msg=msg):
                self.node.on_odom(msg)
                self.node.odom_pub.publish.assert_not_called()
                self.node.tf.sendTransform.assert_not_called()


if __name__ == '__main__':
    unittest.main()
