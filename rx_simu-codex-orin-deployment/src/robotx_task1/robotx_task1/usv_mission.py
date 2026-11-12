"""USV-only Safe Passage mission node.

Reads /task1/landmarks, /task1/obstacles, /usv/odom, /mavros/state and /scan, runs mission_core and
publishes Twist on /usv/cmd_vel for rover_bridge, which keeps all of its own
gates. This node never arms and never changes the FCU mode.

Motion needs all of: enable_motion:=true at start-up, a /task1/start request (or autostart),
fresh inputs, an armed FCU in GUIDED and no other publisher on the command
topic. With enable_motion false it only publishes the status (preview).
"""
import json
import math

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from robotx_task1_interfaces.msg import BuoyArray
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from . import track_control as tc
from .mission_core import DEFAULTS as MISSION_DEFAULTS
from .mission_core import Mission
from .survey_bag import cloud_arrays
from .survey_core import StaticTree, polygon_in_frame

DEFAULTS = dict(
    enable_motion=False, command_topic='/usv/cmd_vel', odom_topic='/usv/odom', scan_topic='/scan',
    state_topic='/mavros/state', gps_topic='/mavros/gpsstatus/gps1/raw',
    odom_frame='odom', base_frame='base_link', scan_frames=['base_stabilized', 'base_link'],
    odom_timeout=0.5, landmark_timeout=1.0, scan_timeout=0.5, state_timeout=2.0, gps_timeout=2.0,
    # True: threading a gap needs GPS_RAW_INT fix_type 6 (RTK fixed) from the gps_status plugin.
    require_rtk_fixed=False, stop_box_length=1.0, stop_box_half_width=0.45, rate=10.0,
    # Course boundary (same parameter as buoy_mapper): the route keeps clear of it.
    boundary='', boundary_frame='map',
    # True: the start request is made at start-up (the USV state machine starting this
    # launch is the start). Motion still needs every other gate below.
    autostart=False)
TUNABLE = ('cruise_speed', 'thread_speed', 'circle_speed', 'max_yaw_rate', 'search_yaw_rate', 'look_range',
           'look_min_range', 'look_timeout', 'align_tolerance', 'arrive_tolerance', 'search_step', 'total_timeout')


class UsvMission(Node):
    def __init__(self, **kwargs):
        super().__init__('usv_mission', **kwargs)
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        options = {}
        for name in TUNABLE:
            self.declare_parameter(name, float(MISSION_DEFAULTS[name]))
            options[name] = float(self.get_parameter(name).value)
        self.declare_parameter('circle_radii', list(MISSION_DEFAULTS['circle_radii']))
        options['circle_radii'] = tuple(self.get_parameter('circle_radii').value)
        self.mission = Mission(**options)
        self.pose = self.odom_ns = self.landmarks = self.landmark_ns = None
        self.obstacles, self.obstacle_ns = [], None
        self.boundary = json.loads(self.p['boundary']) if self.p['boundary'] else None
        self.boundary_ready, self.tree = self.boundary is None, StaticTree()
        self.fcu = self.fcu_at = self.scan = self.gps = self.gps_at = None
        self.command = (self.create_publisher(Twist, self.p['command_topic'], 10)
                        if self.p['enable_motion'] else None)
        self.status = self.create_publisher(String, '/task1/mission_status', 10)
        self.create_subscription(Odometry, self.p['odom_topic'], self.on_odom, 20)
        self.create_subscription(BuoyArray, '/task1/landmarks', self.on_landmarks, 10)
        self.create_subscription(PointCloud2, '/task1/obstacles', self.on_obstacles, 10)
        latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf_static', self.on_static, latched)
        self.create_subscription(State, self.p['state_topic'], self.on_state, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p['scan_topic'], self.on_scan, qos_profile_sensor_data)
        if self.p['require_rtk_fixed']:
            from mavros_msgs.msg import GPSRAW
            self.create_subscription(GPSRAW, self.p['gps_topic'], self.on_gps, qos_profile_sensor_data)
        else:
            self.get_logger().warning('require_rtk_fixed is false: gaps are threaded without an RTK check')
        self.create_service(Trigger, '/task1/start', self.on_start)
        if self.p['autostart']:
            self.mission.start()
            self.get_logger().info('autostart: start requested at start-up')
        self.create_timer(1.0/self.p['rate'], self.tick)
        self.get_logger().info('Motion output ' + ('ENABLED' if self.command else 'disabled (preview)'))

    def now(self):
        return self.get_clock().now().nanoseconds

    @staticmethod
    def ns(stamp):
        return stamp.sec*1_000_000_000+stamp.nanosec

    def on_odom(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if (message.header.frame_id != self.p['odom_frame'] or message.child_frame_id != self.p['base_frame']
                or not all(math.isfinite(v) for v in values)):
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.pose, self.odom_ns = (p.x, p.y, yaw), self.ns(message.header.stamp)

    def on_landmarks(self, message):
        if message.header.frame_id != self.p['odom_frame']:
            return
        self.landmarks = [dict(id=b.id, x=b.position.x, y=b.position.y, state=int(b.state),
                               radius=b.collision_radius) for b in message.buoys]
        self.landmark_ns = self.ns(message.header.stamp)

    def on_static(self, message):
        for item in message.transforms:
            t, q = item.transform.translation, item.transform.rotation
            self.tree.add(item.header.frame_id, item.child_frame_id, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])

    def on_obstacles(self, message):
        if message.header.frame_id != self.p['odom_frame']:
            return
        try:
            xyz = cloud_arrays(message)[0]
        except ValueError:
            return
        self.obstacles = [(float(x), float(y)) for x, y in xyz[:, :2]]
        self.obstacle_ns = self.ns(message.header.stamp)

    def on_state(self, message):
        self.fcu, self.fcu_at = message, self.now()

    def on_scan(self, message):
        self.scan = message

    def on_gps(self, message):
        self.gps, self.gps_at = message, self.now()

    def on_start(self, _request, response):
        self.mission.start()
        response.success, response.message = True, 'start requested; motion still needs every gate to be open'
        self.get_logger().info('Start requested')
        return response

    def inputs(self):
        """(inputs_ok, reason). Ages use the source stamps, never the receive time."""
        now = self.now()

        def fresh(stamp, timeout):
            return stamp is not None and 0.0 <= (now-stamp)/1e9 <= timeout
        if not fresh(self.odom_ns, self.p['odom_timeout']):
            return False, 'odometry stale'
        if not fresh(self.landmark_ns, self.p['landmark_timeout']):
            return False, 'landmarks stale (LiDAR, camera or mapper stopped)'
        if not fresh(self.obstacle_ns, self.p['landmark_timeout']):
            return False, 'obstacle cells stale (mapper stopped or too old)'
        if self.scan is None or not fresh(self.ns(self.scan.header.stamp), self.p['scan_timeout']):
            return False, 'scan stale'
        if self.scan.header.frame_id not in self.p['scan_frames']:
            return False, f'scan frame {self.scan.header.frame_id} is not base-aligned'
        if self.fcu is None or not fresh(self.fcu_at, self.p['state_timeout']) or not self.fcu.connected:
            return False, 'FCU state stale'
        if not self.boundary_ready:
            polygon = polygon_in_frame(self.tree, self.boundary, self.p['odom_frame'], self.p['boundary_frame'])
            if polygon is None:
                return False, f"course boundary not georeferenced (no {self.p['boundary_frame']} -> odom on /tf_static)"
            self.mission.set_boundary(polygon)
            self.boundary_ready = True
            self.get_logger().info(f'Course boundary in odom: {polygon}')
        if self.command is not None and self.count_publishers(self.p['command_topic']) > 1:
            return False, 'another node publishes velocity commands (stop Nav2 first)'
        return True, ''

    def tick(self):
        ok, reason = self.inputs()
        rtk_ok = True
        if self.p['require_rtk_fixed']:
            rtk_ok = (self.gps is not None and (self.now()-self.gps_at)/1e9 <= self.p['gps_timeout']
                      and self.gps.fix_type == 6)
        blocked = ok and tc.scan_blocked(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                                         self.p['stop_box_length'], self.p['stop_box_half_width'])
        mode, armed = (self.fcu.mode, self.fcu.armed) if self.fcu is not None else ('', False)
        result = self.mission.step(self.now()/1e9, self.pose or (0.0, 0.0, 0.0), self.landmarks or [],
                                   ok, mode, armed, rtk_ok, blocked, self.obstacles)
        if self.command is not None:
            twist = Twist()
            twist.linear.x, twist.angular.z = result['v'], result['w']
            self.command.publish(twist)
        for event in result['events']:
            self.get_logger().info('event: '+json.dumps(event))
        self.status.publish(String(data=json.dumps(dict(
            state=result['state'], detail=result['detail'], v=result['v'], w=result['w'], inputs_ok=ok,
            input_problem=reason, blocked=blocked, rtk_ok=rtk_ok, motion_enabled=self.command is not None,
            passed=self.mission.passed, assumed_off=sorted(self.mission.assumed_off), events=result['events']))))

    def stop(self):
        if self.command is not None:
            self.command.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = UsvMission()
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
