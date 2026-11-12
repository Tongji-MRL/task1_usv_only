"""Task 1 <-> USV state machine link (see fsm_report for the message contract).

Reads /task1/mission_status, /task1/landmarks, the FCU's fused GNSS position and
/usv/odom; from the state machine the course_config and the Task 4 commands.
Publishes progress, the final SafePassageReport and the IncidentAcks for the
state machine, and for the Task 1 nodes /task1/course_boundary and
/task1/incident (latched, odometry frame). It sends nothing to the OCS and
never commands the vessel.

WGS84 positions (course corners, keep-out centre) are placed in the odometry
frame through the vessel's current fused fix and odometry position, the same
model the reports use the other way round; both come from the FCU EKF, which
uses RTK when corrections arrive and plain GPS otherwise.
"""
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from robotx_task1_interfaces.msg import BuoyArray
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from . import robocommand_core as rc
from .fsm_report import (IncidentLink, ReportEnvelope, StateMachineLink, StateMachineWatch, course_corners,
                         local_xy, task4_command)

DEFAULTS = dict(team_id='TONG', vehicle_id='usv', report_seq_start=0,
                progress_topic='/tongji_mrl/twave/progress', result_topic='/tongji_mrl/twave/result',
                fix_topic='/mavros/global_position/global', odom_topic='/usv/odom', odom_frame='odom',
                fix_timeout=5.0, report_wait=5.0, refresh=2.0, repeat=1.0, rate=5.0,
                # > 0: exit this many seconds after FINISHED was first sent. The launch then shuts
                # down every Task 1 node and the sensor drivers it started, also when the state
                # machine does not stop it. Not counted while Task 4 is active: the state machine
                # ignores FINISHED during Task 4 and needs it again after the resume.
                exit_after_end=0.0,
                # > 0: exit when the state machine node, once seen, is absent this long [s].
                state_machine_node='usv_state_machine_v2', state_machine_timeout=0.0,
                # The state machine publishes the saved course_config here as Task 1 starts.
                course_config_topic='/tongji_mrl/twave/course_config',
                # Fallback when no course_config arrives: JSON [[lat, lon], ...] of the corners.
                course_corners='',
                task_cmd_topic='/tongji_mrl/twave/task_cmd')

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FsmBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('task1_fsm_bridge', **kwargs)
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.envelope = ReportEnvelope(self.p['team_id'], self.p['vehicle_id'], int(self.p['report_seq_start']))
        self.link = StateMachineLink(self.envelope, refresh=self.p['refresh'], repeat=self.p['repeat'],
                                     report_wait=self.p['report_wait'])
        self.incident, self.zone_sent, self.resumed_at = IncidentLink(), None, None
        self.corners = course_corners(json.dumps({'corners': json.loads(self.p['course_corners'])})) \
            if self.p['course_corners'] else None
        self.corners_source = 'parameter' if self.corners else None
        self.boundary_sent = None
        self.status, self.buoys, self.fix, self.fix_at, self.xy = None, [], None, None, None
        self.watch, self.graph_checked = StateMachineWatch(self.p['state_machine_timeout']), 0.0
        self.progress_pub = self.create_publisher(String, self.p['progress_topic'], 10)
        self.result_pub = self.create_publisher(String, self.p['result_topic'], 10)
        self.boundary_pub = self.create_publisher(String, '/task1/course_boundary', LATCHED)
        self.incident_pub = self.create_publisher(String, '/task1/incident', LATCHED)
        self.create_subscription(String, '/task1/mission_status', self.on_status, 10)
        self.create_subscription(BuoyArray, '/task1/landmarks', self.on_landmarks, 10)
        self.create_subscription(NavSatFix, self.p['fix_topic'], self.on_fix, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['odom_topic'], self.on_odom, 20)
        # The state machine publishes course_config once, as it starts Task 1. A latched
        # (TRANSIENT_LOCAL) publisher reaches only the latched subscription, a volatile one
        # only the other: both are created so either publisher works.
        for qos in (LATCHED, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)):
            self.create_subscription(String, self.p['course_config_topic'], self.on_course_config, qos)
        self.create_subscription(String, self.p['task_cmd_topic'], self.on_task_cmd, 10)
        self.create_timer(1.0/self.p['rate'], self.tick)
        self.get_logger().info(f"Task 1 progress -> {self.p['progress_topic']}, reports and IncidentAcks -> "
                               f"{self.p['result_topic']}; course_config <- {self.p['course_config_topic']}, "
                               f"Task 4 <- {self.p['task_cmd_topic']}")

    # ---------------------------------------------------------------- inputs
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

    def on_course_config(self, message):
        corners = course_corners(message.data)
        if corners is None:
            self.get_logger().warning('course_config without at least three corners: ignored',
                                      throttle_duration_sec=10.0)
            return
        if corners != self.corners:
            self.corners, self.corners_source = corners, 'course_config'
            self.get_logger().info(f'Course corners from course_config: {corners}')

    def on_task_cmd(self, message):
        command = task4_command(message.data)
        if command is None:
            return
        seq, changed = self.incident.handle(command)
        if seq is not None:
            ack = self.envelope.incident_ack(seq, time.time())
            self.result_pub.publish(String(data=json.dumps(ack, separators=(',', ':'))))
            self.get_logger().info(f'IncidentAck for Task 4 {command[0]} (command_seq {seq}) sent')
        else:
            self.get_logger().warning('Task 4 resume without a preceding TASK4_BEGIN: no command_seq to ack')
        if changed and not self.incident.active:
            self.resumed_at = time.monotonic()
        if changed:
            self.zone_sent = None  # Published (again) by tick().

    # ---------------------------------------------------------------- outputs
    def placed(self):
        """(fix, odometry xy) to place WGS84 points in the odometry frame, or None."""
        if self.fix is None or self.xy is None or time.monotonic()-self.fix_at > self.p['fix_timeout']:
            return None
        return self.fix, self.xy

    def report_body(self):
        """SafePassageReport body, georeferenced through the vessel's current fix, or None."""
        ref = self.placed()
        return None if ref is None else rc.safe_passage(self.buoys, ref[1], ref[0])

    def publish_inputs(self):
        ref = self.placed()
        if self.corners is not None and ref is not None and self.boundary_sent != self.corners:
            polygon = [list(local_xy(la, lo, ref[0][0], ref[0][1], ref[1])) for la, lo in self.corners]
            self.boundary_pub.publish(String(data=json.dumps(dict(
                frame=self.p['odom_frame'], polygon=polygon, source=self.corners_source))))
            self.boundary_sent = self.corners
            self.get_logger().info(f'Course boundary in {self.p["odom_frame"]}: {polygon}')
        incident = self.incident
        want = (incident.active, incident.seq, incident.center, incident.radius, ref is not None)
        if self.zone_sent != want:
            zone = dict(active=incident.active, command_seq=incident.seq, x=None, y=None,
                        radius=incident.radius or 0.0)
            if incident.active and incident.center is not None and ref is not None:
                zone['x'], zone['y'] = local_xy(incident.center[0], incident.center[1], ref[0][0], ref[0][1], ref[1])
            self.incident_pub.publish(String(data=json.dumps(zone)))
            self.zone_sent = want
            self.get_logger().info('Task 4 zone for the mission: '+json.dumps(zone))

    def tick(self):
        now = time.monotonic()
        if self.p['state_machine_timeout'] > 0.0 and now-self.graph_checked >= 1.0:
            self.graph_checked = now
            present = self.p['state_machine_node'] in self.get_node_names()
            if self.watch.update(now, present):
                self.get_logger().error(f"State machine node {self.p['state_machine_node']} has been gone for "
                                        f"{self.p['state_machine_timeout']:.0f} s: shutting down Task 1")
                raise SystemExit(1)
        self.publish_inputs()
        if self.status is None:
            return
        for kind, payload in self.link.update(now, self.status, self.report_body(), time.time()):
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
        if wait > 0.0 and self.link.finished_at is not None and not self.incident.active:
            since = max(self.link.finished_at, self.resumed_at or self.link.finished_at)
            if now-since >= wait:
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
