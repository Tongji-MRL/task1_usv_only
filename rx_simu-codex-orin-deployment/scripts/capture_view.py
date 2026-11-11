#!/usr/bin/env python3
"""Save the real headless Gazebo camera output (one PNG or a short MP4)."""
import argparse
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from validate import ROOT


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=1)
    args=parser.parse_args();run=Path((ROOT/'runs/latest.txt').read_text().strip())
    rclpy.init();node=rclpy.create_node('capture_validation_view',
        parameter_overrides=[Parameter('use_sim_time',value=True)]);writer=[None];count=[0]
    def receive(msg):
        if msg.encoding not in ('rgb8','bgr8'):raise ValueError(msg.encoding)
        a=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        bgr=cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if msg.encoding=='rgb8' else a
        if count[0]==0:cv2.imwrite(str(run/'overview.png'),bgr)
        if args.seconds>1:
            if writer[0] is None:writer[0]=cv2.VideoWriter(str(run/'overview.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),1,(msg.width,msg.height))
            writer[0].write(bgr)
        count[0]+=1
    node.create_subscription(Image,'/usv/overview/image',receive,qos_profile_sensor_data)
    start=time.monotonic()
    while time.monotonic()-start<max(5,args.seconds) and (args.seconds>1 or count[0]==0):
        rclpy.spin_once(node,timeout_sec=.2)
    if writer[0]:writer[0].release()
    node.destroy_node();rclpy.shutdown()
    if count[0]==0:raise RuntimeError('No overview image received')
    print(run/'overview.png',count[0],'frames')


if __name__=='__main__':main()
