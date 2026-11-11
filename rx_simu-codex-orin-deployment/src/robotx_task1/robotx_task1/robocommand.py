"""Vessel side of the RoboCommand reporting for Task 1.

Builds the Heartbeat (2 Hz) and SafePassageReport contents and sends them as
JSON datagrams to the team's Operator Control Station, which alone publishes
them to RoboCommand (scripts/ocs_relay.py). The same JSON is published on
/task1/robocommand for recording. If the FCU position is stale no heartbeat is
sent: an old position is never repeated as current.
"""
import json
import math
import socket

import rclpy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robotx_task1_interfaces.msg import BuoyArray
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from . import robocommand_core as rc

DEFAULTS = dict(vehicle_id='USV1', ocs_host='', ocs_port=47800, heartbeat_rate=2.0,
                fix_topic='/mavros/global_position/global', odom_topic='/usv/odom',
                state_topic='/mavros/state', input_timeout=2.0)


class RoboCommand(Node):
    def __init__(self, **kwargs):
        super().__init__('robocommand', **kwargs)
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if self.p['ocs_host'] else None
        if self.socket is None:
            self.get_logger().warning('ocs_host is empty: reports are only published locally, nothing reaches the OCS')
        self.fix = self.fix_at = self.odom = self.odom_at = self.fcu = self.fcu_at = None
        self.buoys, self.mission_state, self.schedule = [], '', rc.ReportSchedule()
        self.local = self.create_publisher(String, '/task1/robocommand', 10)
        self.create_subscription(NavSatFix, self.p['fix_topic'], self.on_fix, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['odom_topic'], self.on_odom, 20)
        self.create_subscription(State, self.p['state_topic'], self.on_state, qos_profile_sensor_data)
        self.create_subscription(BuoyArray, '/task1/landmarks', self.on_landmarks, 10)
        self.create_subscription(String, '/task1/mission_status', self.on_mission, 10)
        self.create_timer(1.0/self.p['heartbeat_rate'], self.tick)

    def seconds(self):
        return self.get_clock().now().nanoseconds/1e9

    def on_fix(self, message):
        if math.isfinite(message.latitude) and math.isfinite(message.longitude):
            self.fix, self.fix_at = message, self.seconds()

    def on_odom(self, message):
        self.odom, self.odom_at = message, self.seconds()

    def on_state(self, message):
        self.fcu, self.fcu_at = message, self.seconds()

    def on_landmarks(self, message):
        self.buoys = [dict(id=b.id, x=b.position.x, y=b.position.y, state=int(b.state)) for b in message.buoys]

    def on_mission(self, message):
        try:
            self.mission_state = json.loads(message.data).get('state', '')
        except ValueError:
            self.mission_state = ''

    def send(self, kind, body):
        packet = json.dumps(dict(vehicle_id=self.p['vehicle_id'], kind=kind, body=body))
        self.local.publish(String(data=packet))
        if self.socket is not None:
            try:
                self.socket.sendto(packet.encode(), (self.p['ocs_host'], int(self.p['ocs_port'])))
            except OSError as error:
                self.get_logger().error(f'Cannot send to the OCS: {error}', throttle_duration_sec=5.0)

    def tick(self):
        now = self.seconds()

        def fresh(at):
            return at is not None and now-at <= self.p['input_timeout']
        if not (fresh(self.fix_at) and fresh(self.odom_at)):
            self.get_logger().warning('No fresh FCU position: heartbeat withheld', throttle_duration_sec=5.0)
            return
        q, velocity = self.odom.pose.pose.orientation, self.odom.twist.twist.linear
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y-q.z*q.x))))
        fcu = self.fcu if fresh(self.fcu_at) else None
        state = rc.robot_state(fcu.connected, fcu.armed, fcu.mode) if fcu is not None else 'STATE_UNKNOWN'
        self.send('heartbeat', rc.heartbeat(
            self.fix.latitude, self.fix.longitude, math.hypot(velocity.x, velocity.y), yaw, roll, pitch,
            self.fix.altitude, state, rc.current_task(self.mission_state)))
        if self.schedule.due(now, self.buoys):
            position = self.odom.pose.pose.position
            self.send('safe_passage', rc.safe_passage(self.buoys, (position.x, position.y),
                                                      (self.fix.latitude, self.fix.longitude)))


def main(args=None):
    rclpy.init(args=args)
    node = RoboCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
