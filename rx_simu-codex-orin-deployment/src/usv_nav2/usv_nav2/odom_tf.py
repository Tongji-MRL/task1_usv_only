"""Relay validated MAVROS odometry with Reliable QoS and optionally publish TF."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from .core import split_yaw_attitude, stamp_is_fresh


class OdomTF(Node):
    def __init__(self):
        super().__init__('odom_tf')
        for name, value in {
            'odom_topic': '/mavros/local_position/odom',
            'output_odom_topic': '/usv/odom',
            'publish_tf': True,
            'expected_parent_frame': 'map',
            'body_frame': 'base_link',
            'stabilized_frame': 'base_stabilized',
            'max_message_age': 0.5,
        }.items():
            self.declare_parameter(name, value)
        self.parent = self.get_parameter('expected_parent_frame').value
        self.body = self.get_parameter('body_frame').value
        self.stabilized = self.get_parameter('stabilized_frame').value
        self.max_age = self.get_parameter('max_message_age').value
        if not math.isfinite(self.max_age) or self.max_age <= 0.0:
            raise ValueError('max_message_age must be positive')
        if not all((self.parent, self.body, self.stabilized)) or len(
                {self.parent, self.body, self.stabilized}) != 3:
            raise ValueError('TF frame names must be nonempty and distinct')
        input_topic = self.get_parameter('odom_topic').value
        output_topic = self.get_parameter('output_odom_topic').value
        if self.resolve_topic_name(input_topic) == self.resolve_topic_name(output_topic):
            raise ValueError('Input and output odometry topics must differ')
        self.tf = (TransformBroadcaster(self)
                   if self.get_parameter('publish_tf').value else None)
        self.odom_pub = self.create_publisher(
            Odometry, output_topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.sub = self.create_subscription(
            Odometry, input_topic, self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg):
        if msg.header.frame_id != self.parent or msg.child_frame_id != self.body:
            self.get_logger().error(
                'Odometry frames do not match configuration: '
                f'{msg.header.frame_id} -> {msg.child_frame_id}; '
                f'expected {self.parent} -> {self.body}. No odometry/TF published.',
                throttle_duration_sec=5.0)
            return
        now = self.get_clock().now().nanoseconds / 1e9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        if (not stamp_is_fresh(stamp, now, self.max_age) or
                not all(math.isfinite(value) for value in (
                    p.x, p.y, p.z, v.x, v.y, v.z, w.x, w.y, w.z))):
            self.get_logger().warn('Stale/invalid odometry; no odometry/TF published.',
                                   throttle_duration_sec=5.0)
            return
        try:
            yaw_q, residual_q = split_yaw_attitude((q.x, q.y, q.z, q.w))
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.0)
            return
        # Preserve the source message, including stamp, frames and covariance.
        # Humble Nav2 uses SystemDefaultsQoS for odometry subscriptions.
        self.odom_pub.publish(msg)
        if self.tf is None:
            return
        level = TransformStamped()
        level.header = msg.header
        level.child_frame_id = self.stabilized
        level.transform.translation.x = p.x
        level.transform.translation.y = p.y
        level.transform.translation.z = p.z
        level.transform.rotation.x, level.transform.rotation.y, \
            level.transform.rotation.z, level.transform.rotation.w = yaw_q
        body = TransformStamped()
        body.header.stamp = msg.header.stamp
        body.header.frame_id = self.stabilized
        body.child_frame_id = self.body
        body.transform.rotation.x, body.transform.rotation.y, \
            body.transform.rotation.z, body.transform.rotation.w = residual_q
        self.tf.sendTransform([level, body])


def main(args=None):
    rclpy.init(args=args)
    node = OdomTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
