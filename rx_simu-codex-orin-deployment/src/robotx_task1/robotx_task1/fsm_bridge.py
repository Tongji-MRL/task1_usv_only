"""Task 1 <-> USV state machine link (see fsm_report for the message contract).

Reads /task1/mission_status, /task1/landmarks, the FCU GNSS fix and /usv/odom;
publishes progress and, once at the end, the SafePassageReport for the team's
USV state machine. It sends nothing to the OCS and never commands the vessel.
"""
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robotx_task1_interfaces.msg import BuoyArray
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from . import robocommand_core as rc
from .fsm_report import ReportEnvelope, StateMachineLink, StateMachineWatch

DEFAULTS = dict(team_id='TONG', vehicle_id='usv', report_seq_start=0,
                progress_topic='/tongji_mrl/twave/progress', result_topic='/tongji_mrl/twave/result',
                fix_topic='/mavros/global_position/global', odom_topic='/usv/odom', odom_frame='odom',
                fix_timeout=5.0, report_wait=5.0, refresh=2.0, repeat=1.0, rate=5.0,
                # > 0: exit this many seconds after FINISHED was first sent. The launch then shuts
                # down every Task 1 node and the sensor drivers it started, also when the state
                # machine does not stop it. 0: keep running until stopped.
                exit_after_end=0.0,
                # > 0: exit when the state machine node, once seen, is absent this long [s].
                state_machine_node='usv_state_machine_v2', state_machine_timeout=0.0)


class FsmBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('task1_fsm_bridge', **kwargs)
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.link = StateMachineLink(ReportEnvelope(self.p['team_id'], self.p['vehicle_id'],
                                                    int(self.p['report_seq_start'])),
                                     refresh=self.p['refresh'], repeat=self.p['repeat'],
                                     report_wait=self.p['report_wait'])
        self.status, self.buoys, self.fix, self.fix_at, self.xy = None, [], None, None, None
        self.watch, self.graph_checked = StateMachineWatch(self.p['state_machine_timeout']), 0.0
        self.progress_pub = self.create_publisher(String, self.p['progress_topic'], 10)
        self.result_pub = self.create_publisher(String, self.p['result_topic'], 10)
        self.create_subscription(String, '/task1/mission_status', self.on_status, 10)
        self.create_subscription(BuoyArray, '/task1/landmarks', self.on_landmarks, 10)
        self.create_subscription(NavSatFix, self.p['fix_topic'], self.on_fix, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['odom_topic'], self.on_odom, 20)
        self.create_timer(1.0/self.p['rate'], self.tick)
        self.get_logger().info(f"Task 1 progress -> {self.p['progress_topic']}, "
                               f"final SafePassageReport -> {self.p['result_topic']}")

    def on_status(self, message):
        try:
            value = json.loads(message.data)
        except ValueError:
            return
        if isinstance(value, dict):
            self.status = value

    def on_landmarks(self, message):
        if message.header.frame_id == self.p['odom_frame']:
            self.buoys = [dict(id=b.id, x=b.position.x, y=b.position.y, state=int(b.state)) for b in message.buoys]

    def on_fix(self, message):
        if math.isfinite(message.latitude) and math.isfinite(message.longitude):
            self.fix, self.fix_at = (message.latitude, message.longitude), time.monotonic()

    def on_odom(self, message):
        if message.header.frame_id == self.p['odom_frame']:
            p = message.pose.pose.position
            self.xy = (p.x, p.y)

    def report_body(self):
        """SafePassageReport body, georeferenced through the vessel's current fix, or None."""
        if self.fix is None or self.xy is None or time.monotonic()-self.fix_at > self.p['fix_timeout']:
            return None
        return rc.safe_passage(self.buoys, self.xy, self.fix)

    def tick(self):
        now = time.monotonic()
        if self.p['state_machine_timeout'] > 0.0 and now-self.graph_checked >= 1.0:
            self.graph_checked = now
            present = self.p['state_machine_node'] in self.get_node_names()
            if self.watch.update(now, present):
                self.get_logger().error(f"State machine node {self.p['state_machine_node']} has been gone for "
                                        f"{self.p['state_machine_timeout']:.0f} s: shutting down Task 1")
                raise SystemExit(1)
        if self.status is None:
            return
        for kind, payload in self.link.update(time.monotonic(), self.status, self.report_body(), time.time()):
            text = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
            if kind == 'result':
                self.result_pub.publish(String(data=text))
                self.get_logger().info('SafePassageReport sent to the state machine: '+text)
            else:
                self.progress_pub.publish(String(data=text))
        if self.link.missing_report:
            self.get_logger().error('No fresh GNSS fix at the end of Task 1: SafePassageReport not sent',
                                    throttle_duration_sec=30.0)
        wait = self.p['exit_after_end']
        if wait > 0.0 and self.link.finished_at is not None and time.monotonic()-self.link.finished_at >= wait:
            self.get_logger().info(f'Task 1 ended {wait:.0f} s ago: shutting down the Task 1 launch')
            raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = FsmBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
