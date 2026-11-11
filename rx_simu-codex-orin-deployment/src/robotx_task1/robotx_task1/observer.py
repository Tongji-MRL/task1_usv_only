"""Explicit known-observation replacement for future UAV perception. No boat truth."""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robotx_task1_interfaces.msg import Buoy, BuoyArray
from .core import valid_snapshot


class KnownObserver(Node):
    def __init__(self):
        super().__init__('task1_known_observer')
        self.publisher=self.create_publisher(BuoyArray,'/task1/observations',10)
        self.last_stamp=-1.;self.last_revision=0
        self.create_subscription(String,'/task1/scene_state',self.receive,10)

    def receive(self,msg):
        try:
            data=json.loads(msg.data);stamp=float(data['stamp']);revision=int(data['revision'])
            if not valid_snapshot(data) or stamp<=self.last_stamp or revision<self.last_revision:return
            now=self.get_clock().now().nanoseconds/1e9
            if not -.1<=now-stamp<=.5:return
            output=BuoyArray();output.header.frame_id='map'
            output.header.stamp.sec=int(stamp);output.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
            output.revision=revision;output.survey_complete=data['active']
            for b in data['buoys']:
                item=Buoy();item.id=b['id'];item.position.x=float(b['x']);item.position.y=float(b['y'])
                item.state=int(b['state']);item.collision_radius=float(b['radius']);output.buoys.append(item)
            self.last_stamp=stamp;self.last_revision=revision;self.publisher.publish(output)
        except (ValueError,KeyError,TypeError) as error:
            self.get_logger().error('Invalid known observation: '+str(error),throttle_duration_sec=2.)


def main():
    rclpy.init();node=KnownObserver()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
