"""Translate Nav2 Twist to MAVROS BODY_NED velocity + yaw-rate setpoints."""

import math
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from .core import (BODY_NED_FRAME, VELOCITY_YAW_RATE_MASK, CommandGate,
                   bounded_command, normalise_quaternion, stamp_is_fresh, scan_ranges_valid)
from .task_lease import TaskLease
from .stationkeeping import StationKeeping


class RoverBridge(Node):
    def __init__(self):
        super().__init__('rover_bridge')
        defaults = {
            'enable_control': False,
            'command_topic': '/usv/cmd_vel',
            'odom_topic': '/mavros/local_position/odom',
            'scan_topic': '/scan',
            'state_topic': '/mavros/state',
            'setpoint_topic': '/mavros/setpoint_raw/local',
            'expected_parent_frame': 'map',
            'body_frame': 'base_link',
            'max_speed': 0.4,
            'max_yaw_rate': 0.4,
            'command_timeout': 0.5,
            'odom_timeout': 0.5,
            'scan_timeout': 0.5,
            'state_timeout': 2.0,
            'publish_rate': 20.0,
            'require_task_permission': False,
            'enable_stationkeeping': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        for name in ('max_speed', 'max_yaw_rate', 'publish_rate'):
            if not math.isfinite(self.p[name]) or self.p[name] <= 0.0:
                raise ValueError(name + ' must be positive')
        self.gate = CommandGate(**{key: self.p[key] for key in (
            'command_timeout', 'odom_timeout', 'scan_timeout', 'state_timeout')})
        self.sensor_headers = {}
        self.sensor_valid_received={}
        self.fcu_state=None;self.position=None;self.station=None;self.last_heartbeat=-1e9
        self.mode_future=None
        self.task_lease = TaskLease()
        if self.p['require_task_permission']:
            from robotx_task1_interfaces.msg import MotionPermission
            self.create_subscription(MotionPermission, '/task1/motion_permission', self.on_permission, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Preview is always available. No FCU output publisher in preview mode.
        self.preview_pub = self.create_publisher(PositionTarget, '/usv/setpoint_preview', 10)
        self.status_pub = self.create_publisher(String, '/usv/control_status', 10)
        self.output_pub = (self.create_publisher(PositionTarget, self.p['setpoint_topic'], 10)
                           if self.p['enable_control'] else None)
        if self.p['enable_stationkeeping'] and self.output_pub is not None:
            if not self.p['require_task_permission']:raise ValueError('Station keeping needs a fresh task lease')
            from mavros_msgs.srv import SetMode
            from mavros_msgs.msg import Mavlink
            from pymavlink import mavutil
            self.station=StationKeeping()
            self.mode_client=self.create_client(SetMode,'/mavros/set_mode')
            self.station_status_pub=self.create_publisher(String,'/usv/stationkeeping_status',10)
            self.heartbeat_pub=self.create_publisher(Mavlink,'/uas1/mavlink_sink',10)
            self.heartbeat_codec=mavutil.mavlink.MAVLink(None,srcSystem=252,srcComponent=191)
        self.create_subscription(Twist, self.p['command_topic'], self.on_command, 10)
        self.create_subscription(State, self.p['state_topic'], self.on_state, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['odom_topic'], self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p['scan_topic'], self.on_scan, qos_profile_sensor_data)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / self.p['publish_rate'], self.tick,
                                      clock=self.steady_clock)
        self.last_status = None

    def on_command(self, msg):
        if self.p['require_task_permission'] and not self.task_lease.valid(
                self.get_clock().now().nanoseconds/1e9, time.monotonic()):
            self.gate.invalidate('command')
            return
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        try:
            command = bounded_command(values, self.p['max_speed'], self.p['max_yaw_rate'])
        except ValueError as error:
            self.gate.invalidate('command')
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            return
        self.gate.update_command(command, time.monotonic())

    def on_permission(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        if self.task_lease.update(msg.revision, msg.allow, stamp,
                                  self.get_clock().now().nanoseconds/1e9, time.monotonic(), msg.reason,
                                  getattr(msg,'station_action','')):
            self.gate.invalidate('command')

    def on_state(self, msg):
        self.fcu_state=msg
        if self.station:self.station.observe(msg.mode,msg.armed)
        self.gate.update_state(msg.connected, msg.armed, msg.mode, time.monotonic())

    def header_fresh(self, header, timeout):
        stamp = header.stamp.sec + header.stamp.nanosec / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        return stamp_is_fresh(stamp, now, timeout)

    def on_odom(self, msg):
        p, q, v, w = (msg.pose.pose.position, msg.pose.pose.orientation,
                      msg.twist.twist.linear, msg.twist.twist.angular)
        try:
            normalise_quaternion((q.x, q.y, q.z, q.w))
            valid = (self.header_fresh(msg.header, self.p['odom_timeout']) and
                     msg.header.frame_id == self.p['expected_parent_frame'] and
                     msg.child_frame_id == self.p['body_frame'] and
                     all(math.isfinite(x) for x in (p.x, p.y, p.z, v.x, v.y, v.z, w.x, w.y, w.z)))
        except ValueError:
            valid = False
        if valid:
            self.position=(p.x,p.y)
            self.gate.mark('odom', time.monotonic())
            self.sensor_valid_received['odom']=time.monotonic()
            self.sensor_headers['odom'] = msg.header
        else:
            self.gate.invalidate('odom')

    def on_scan(self, msg):
        valid = (self.header_fresh(msg.header, self.p['scan_timeout']) and
                 bool(msg.header.frame_id) and len(msg.ranges) > 0 and
                 math.isfinite(msg.angle_increment) and msg.angle_increment > 0.0 and
                 math.isfinite(msg.range_min) and math.isfinite(msg.range_max) and
                 0.0 <= msg.range_min < msg.range_max and
                 scan_ranges_valid(msg.ranges, msg.range_min, msg.range_max) and
                 self.tf_buffer.can_transform(self.p['body_frame'], msg.header.frame_id,
                                              Time.from_msg(msg.header.stamp)))
        if valid:
            self.gate.mark('scan', time.monotonic())
            self.sensor_valid_received['scan']=time.monotonic()
            self.sensor_headers['scan'] = msg.header
        else:
            self.gate.invalidate('scan')

    def setpoint(self, command):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['body_frame']
        msg.coordinate_frame = BODY_NED_FRAME
        msg.type_mask = VELOCITY_YAW_RATE_MASK
        msg.velocity.x, msg.yaw_rate = command
        msg.velocity.y = 0.0
        # Keep ROS signs: +x forward, +yaw_rate counterclockwise.
        # MAVROS setpoint_raw performs FRD/NED conversion once.
        return msg

    def tick(self):
        for source, header in self.sensor_headers.items():
            if not self.header_fresh(header, self.p[source + '_timeout']):
                self.gate.invalidate(source)
            elif source == 'scan' and not self.tf_buffer.can_transform(
                    self.p['body_frame'], header.frame_id, Time.from_msg(header.stamp)):
                self.gate.invalidate(source)
        if self.station:self.station_tick()
        command, reason = self.gate.evaluate(time.monotonic())
        if self.p['require_task_permission'] and not self.task_lease.valid(
                self.get_clock().now().nanoseconds/1e9, time.monotonic()):
            command, reason = (0.0, 0.0), 'task stopped: ' + self.task_lease.reason
            self.gate.invalidate('command')
        msg = self.setpoint(command)
        self.preview_pub.publish(msg)
        # Outside GUIDED / armed state, let the pilot / FCU mode own motion.
        if self.output_pub is not None and self.gate.eligible:
            self.output_pub.publish(msg)
        status = ('CONTROL: ' if self.output_pub is not None else 'PREVIEW: ') + reason
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def station_tick(self):
        from mavros_msgs.srv import SetMode
        from mavros.mavlink import convert_to_rosmsg
        from pymavlink import mavutil
        now=time.monotonic();sim=self.get_clock().now().nanoseconds/1e9
        state=self.fcu_state
        if state is None:return
        # An isolated empty MID-360 frame must not extend freshness. The
        # existing last-valid 0.5 s budget tolerates one scan gap in LOITER;
        # GUIDED's command gate still sends zero immediately on invalid input.
        healthy=(state.connected and self.task_lease.fresh(sim,now)
            and self.gate.received['state'] is not None
            and 0<=now-self.gate.received['state']<=self.p['state_timeout']
            and all(k in self.sensor_valid_received and k in self.sensor_headers
                    and 0<=now-self.sensor_valid_received[k]<=self.p[k+'_timeout']
                    and self.header_fresh(self.sensor_headers[k],self.p[k+'_timeout'])
                    for k in ('odom','scan')))
        request=self.station.decide(state.mode,state.armed,healthy,self.task_lease.station_action,self.position,now)
        if request:
            self.gate.invalidate('command')
            self.get_logger().info('station mode request '+request+': '+self.station.reason)
            self.station_status_pub.publish(String(data=json.dumps(dict(
                wall=now,sim=sim,request=request,reason=self.station.reason,owned=self.station.owned,
                failed=self.station.failed,anchor=self.station.anchor))))
            if self.mode_client.service_is_ready():
                self.mode_future=self.mode_client.call_async(SetMode.Request(base_mode=0,custom_mode=request))
        # In the optional profile SYSID_MYGCS=252. MAVROS's normal heartbeat
        # (255) cannot hide a dead bridge; this heartbeat still travels through
        # MAVROS, and native GCS failsafe owns loss of either process.
        if now-self.last_heartbeat>=.25 and not self.station.failed:
            msg=self.heartbeat_codec.heartbeat_encode(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,mavutil.mavlink.MAV_STATE_ACTIVE)
            msg.pack(self.heartbeat_codec);self.heartbeat_codec.seq=(self.heartbeat_codec.seq+1)%256
            self.heartbeat_pub.publish(convert_to_rosmsg(msg,self.get_clock().now().to_msg()))
            self.last_heartbeat=now

    def stop(self):
        if self.output_pub is not None and self.gate.eligible:
            self.output_pub.publish(self.setpoint((0.0, 0.0)))


def main(args=None):
    rclpy.init(args=args)
    node = RoverBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
