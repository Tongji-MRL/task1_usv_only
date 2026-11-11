#!/usr/bin/env python3
"""Observe actual DDS QoS, source timestamps and TF ownership without control."""
import json
from collections import defaultdict
from pathlib import Path
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import PointCloud2,LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

ROOT=Path(__file__).resolve().parents[1]


def main():
    rclpy.init();node=rclpy.create_node('validation_graph_audit',parameter_overrides=[Parameter('use_sim_time',value=True)])
    run=Path((ROOT/'runs/latest.txt').read_text().strip())
    scene=json.loads((run/'generated/scene.json').read_text());mid=scene.get('lidar_model')=='mid360'
    raw='/livox/lidar' if mid else '/usv/lidar/points'
    stamps=defaultdict(set);ages=defaultdict(list);edges=defaultdict(set);point_stamps=set()
    def receive(topic,msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        now=node.get_clock().now().nanoseconds
        if now<=0:return
        stamps[topic].add(stamp);ages[topic].append((now-stamp)/1e9)
        if mid and topic==raw:
            from usv_sim.livox_filter import point_times
            point_stamps.update(int(t) for t in set(point_times(msg)[0]))
    topics={raw:PointCloud2,'/usv/lidar/filtered':PointCloud2,'/scan':LaserScan,
            '/mavros/local_position/odom':Odometry,'/usv/odom':Odometry}
    if 'task1' in json.loads((run/'generated/scene.json').read_text()):
        from robotx_task1_interfaces.msg import TrafficSafety
        topics.update({'/usv/lidar/marking':PointCloud2,'/usv/lidar/clearing':PointCloud2})
        topics['/task1/traffic_safety']=TrafficSafety
    for topic,cls in topics.items():node.create_subscription(cls,topic,lambda m,t=topic:receive(t,m),qos_profile_sensor_data)
    def tf(message,topic):
        for t in message.transforms:edges[topic].add(t.header.frame_id+' -> '+t.child_frame_id)
    node.create_subscription(TFMessage,'/tf',lambda m:tf(m,'/tf'),100)
    node.create_subscription(TFMessage,'/tf_static',lambda m:tf(m,'/tf_static'),QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    # A cold-start graph may exist before the FCU has produced usable EKF data.
    deadline=time.monotonic()+45
    while not all(stamps[topic] for topic in topics):
        if time.monotonic()>deadline:raise TimeoutError('Audit requires live samples on every measured topic')
        if Path((ROOT/'runs/latest.txt').read_text().strip())!=run:
            raise RuntimeError('Session changed during graph audit; retry in a stable session')
        rclpy.spin_once(node,timeout_sec=.05)
    stamps.clear();ages.clear();point_stamps.clear()
    end=time.monotonic()+10
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    result={'run_dir':str(run),'tf_topics':{},'topics':{},'use_sim_time':{},
            'tf_authority_note':'Use build/tf_audit/tf_audit for per-edge DDS GIDs; Humble rclpy callbacks do not expose MessageInfo.'}
    for topic in ('/tf','/tf_static'):
        publishers=node.get_publishers_info_by_topic(topic)
        result['tf_topics'][topic]={'edges':sorted(edges[topic]),'publisher_count':len(publishers),
                                   'publishers':[p.node_namespace+'/'+p.node_name for p in publishers]}
    for topic in topics:
        infos=node.get_publishers_info_by_topic(topic)
        result['topics'][topic]={'publishers':[{'node':p.node_namespace+'/'+p.node_name,'reliability':str(p.qos_profile.reliability)} for p in infos],
                                 'samples':len(stamps[topic]),'max_source_age_s':max(ages[topic],default=None)}
    derived=[('/scan',raw),('/usv/lidar/filtered',raw),('/usv/odom','/mavros/local_position/odom')]
    derived.extend((topic,raw) for topic in ('/usv/lidar/marking','/usv/lidar/clearing') if topic in topics)
    if '/task1/traffic_safety' in topics:
        derived.append(('/task1/traffic_safety','/usv/lidar/marking'))
        result['traffic_guard_subscriptions']=dict(node.get_subscriber_names_and_types_by_node('traffic_guard','/'))
    for out,source in derived:
        observed=stamps[out]
        source_times=point_stamps if mid and out=='/usv/lidar/clearing' else stamps[source]
        result['topics'][out]['source_stamp_basis']='original per-point time' if source_times is point_stamps else 'original message header'
        result['topics'][out]['source_stamps_matched']=len(observed&source_times)
        result['topics'][out]['source_stamps_unmatched']=len(observed-source_times)
        if observed and source_times:
            # DDS callbacks at the start/end can straddle the observation window.
            # Keep raw counts and separately test the common interior time range.
            begin=max(min(observed),min(source_times));end=min(max(observed),max(source_times))
            interior={t for t in observed if begin<t<end}
            result['topics'][out]['interior_source_stamps_matched']=len(interior&source_times)
            result['topics'][out]['interior_source_stamps_unmatched']=len(interior-source_times)
    for name,namespace in node.get_node_names_and_namespaces():
        full=(namespace.rstrip('/')+'/'+name)
        client=node.create_client(GetParameters,full+'/get_parameters')
        if not client.wait_for_service(timeout_sec=.1):continue
        future=client.call_async(GetParameters.Request(names=['use_sim_time']))
        deadline=time.monotonic()+1
        while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        if future.done() and future.result().values:
            value=future.result().values[0]
            result['use_sim_time'][full]=value.bool_value if value.type==1 else {'parameter_type':value.type,'note':'not a ROS bool parameter response'}
    if Path((ROOT/'runs/latest.txt').read_text().strip())!=run:
        raise RuntimeError('Session changed during graph audit; retry in a stable session')
    (run/'graph_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
