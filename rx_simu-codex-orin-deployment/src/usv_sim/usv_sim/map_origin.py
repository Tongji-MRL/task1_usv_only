"""Align a fixed georeferenced water map with the FCU's reported EKF origin."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import TransformStamped
from mavros_msgs.srv import CommandLong
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


def local_xy(latitude, longitude, origin_latitude=-33.724223, origin_longitude=150.679736):
    values=(latitude,longitude,origin_latitude,origin_longitude)
    if not all(math.isfinite(x) for x in values) or abs(latitude)>90 or abs(longitude)>180:
        raise ValueError('Invalid geographic origin')
    lat=math.radians(origin_latitude);a=6378137.;e2=6.6943799901413165e-3
    n=a/math.sqrt(1-e2*math.sin(lat)**2);m=a*(1-e2)/(1-e2*math.sin(lat)**2)**1.5
    return math.radians(longitude-origin_longitude)*n*math.cos(lat),math.radians(latitude-origin_latitude)*m


class MapOrigin(Node):
    def __init__(self):
        super().__init__('map_origin')
        self.declare_parameter('origin_latitude',-33.724223)
        self.declare_parameter('origin_longitude',150.679736)
        self.tf=StaticTransformBroadcaster(self);self.origin=None;self.request=None;self.request_at=0.
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher=self.create_publisher(String,'/usv/map_alignment',qos)
        self.create_subscription(GeoPointStamped,'/mavros/global_position/gp_origin',self.on_origin,qos)
        self.client=self.create_client(CommandLong,'/mavros/cmd/command')
        self.create_timer(2.,self.ask_origin)

    def ask_origin(self):
        if self.origin is not None or not self.client.service_is_ready():return
        if self.request is not None and not self.request.done():
            if time.monotonic()-self.request_at<6.:return
            # A dropped ROS service response can leave this future pending
            # forever, even after the FCU has initialized its EKF origin.
            # Retry only the read request; no transform is fabricated.
            self.request.cancel()
            self.get_logger().warning('FCU origin request response timed out; retrying read')
        # MAV_CMD_REQUEST_MESSAGE / GPS_GLOBAL_ORIGIN. This never changes the FCU origin.
        self.request=self.client.call_async(CommandLong.Request(command=512,param1=49.))
        self.request_at=time.monotonic()

    def on_origin(self,message):
        p=message.position
        try:
            x,y=local_xy(p.latitude,p.longitude,self.get_parameter('origin_latitude').value,
                         self.get_parameter('origin_longitude').value)
            if math.hypot(x,y)>1000:raise ValueError('FCU origin is outside this test area')
        except ValueError as error:
            self.get_logger().error(str(error));return
        if self.origin==(x,y):return
        self.origin=(x,y)
        transform=TransformStamped();transform.header.stamp=self.get_clock().now().to_msg()
        transform.header.frame_id='map';transform.child_frame_id='odom'
        transform.transform.translation.x=x;transform.transform.translation.y=y
        # A 2D water map shares the EKF's vertical datum; do not mix ellipsoid and MSL heights.
        transform.transform.rotation.w=1.
        self.tf.sendTransform(transform)
        record={'offset_xy_m':[x,y],'fcu_origin_lat_lon':[p.latitude,p.longitude],
                'map_origin_lat_lon':[self.get_parameter('origin_latitude').value,self.get_parameter('origin_longitude').value],
                'vertical_datum':'shared with FCU EKF; horizontal georeferencing only'}
        self.publisher.publish(String(data=json.dumps(record)))
        self.get_logger().info('map -> odom: '+json.dumps(record))


def main():
    rclpy.init();node=MapOrigin()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
