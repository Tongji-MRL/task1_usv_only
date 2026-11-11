"""Seeded GNSS measurements from VRX truth into ArduRover's MAVLink GPS backend."""
from collections import deque
import json
import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from pymavlink import mavutil


class GNSS(Node):
    def __init__(self):
        super().__init__('gnss_injector')
        for key,value in {'fix_type':6,'horizontal_sigma':0.02,'reported_accuracy':0.02,
                          'delay_s':0.1,'drop':False,'seed':42,'east_bias':0.,'north_bias':0.}.items():
            self.declare_parameter(key,value)
        self.random=random.Random(self.get_parameter('seed').value)
        self.previous=None;self.truth=None;self.queue=deque()
        self.publisher=self.create_publisher(String,'/usv/gnss_injection',10)
        self.create_subscription(Odometry,'/usv/ground_truth',self.on_truth,qos_profile_sensor_data)
        self.mav=None;self.last_connect=0
        self.create_timer(0.1,self.tick)

    def on_truth(self,msg):
        self.truth=msg

    def tick(self):
        if self.truth is None:return
        if self.mav is None:
            if time.monotonic()-self.last_connect<2:return
            self.last_connect=time.monotonic()
            try:self.mav=mavutil.mavlink_connection('tcp:127.0.0.1:5760',source_system=253,retries=1)
            except OSError:return
        # Drain telemetry so the TCP stream does not stall the FCU serial output.
        while self.mav.recv_match(blocking=False) is not None:pass
        now=self.get_clock().now().nanoseconds/1e9
        p=self.truth.pose.pose.position
        stamp=self.truth.header.stamp.sec+self.truth.header.stamp.nanosec/1e9
        if now-stamp>0.5:return
        velocity=[0.,0.,0.]
        if self.previous is not None and stamp>self.previous[0]:
            dt=stamp-self.previous[0]
            velocity=[(p.x-self.previous[1])/dt,(p.y-self.previous[2])/dt,(p.z-self.previous[3])/dt]
        self.previous=(stamp,p.x,p.y,p.z)
        sigma=max(0.,self.get_parameter('horizontal_sigma').value)
        east=p.x+self.random.gauss(0,sigma)+self.get_parameter('east_bias').value
        north=p.y+self.random.gauss(0,sigma)+self.get_parameter('north_bias').value
        # WGS84 radii at the fixed Sydney origin; <100 m local approximation.
        lat0=math.radians(-33.724223);a=6378137.;e2=6.6943799901413165e-3
        n=a/math.sqrt(1-e2*math.sin(lat0)**2);m=a*(1-e2)/(1-e2*math.sin(lat0)**2)**1.5
        lat=-33.724223+math.degrees(north/m);lon=150.679736+math.degrees(east/(n*math.cos(lat0)))
        sample={'stamp':stamp,'lat':lat,'lon':lon,'alt':p.z,'velocity':velocity,
                'fix_type':int(self.get_parameter('fix_type').value),
                'accuracy':max(.001,self.get_parameter('reported_accuracy').value),'sigma':sigma}
        sample['bias_en_m']=[self.get_parameter('east_bias').value,self.get_parameter('north_bias').value]
        self.queue.append((now+max(0.,self.get_parameter('delay_s').value),sample))
        while self.queue and self.queue[0][0]<=now:
            _,sample=self.queue.popleft()
            if self.get_parameter('drop').value:continue
            # Fixed synthetic GPS epoch; time_usec retains the measurement time.
            gps_seconds=1477267200+sample['stamp']
            week=int(gps_seconds//604800);week_ms=int(gps_seconds%604800*1000)
            east_v,north_v,up_v=sample['velocity']
            self.mav.mav.gps_input_send(int(sample['stamp']*1e6),0,0,week_ms,week,sample['fix_type'],
                round(sample['lat']*1e7),round(sample['lon']*1e7),sample['alt'],0.8,1.2,
                north_v,east_v,-up_v,0.04,sample['accuracy'],max(.03,sample['accuracy']),14)
            sample['sent_sim_time']=now
            self.publisher.publish(String(data=json.dumps(sample)))


def main():
    rclpy.init();node=GNSS()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node.mav:node.mav.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
