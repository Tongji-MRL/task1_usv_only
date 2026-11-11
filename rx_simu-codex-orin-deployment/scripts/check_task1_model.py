#!/usr/bin/env python3
"""Exercise actual rendered beacon states and save camera evidence without arming."""
import json
from pathlib import Path
import time
import subprocess
import cv2
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from mavros_msgs.msg import State
from robotx_task1.beacon_evidence import blink_evidence

ROOT=Path(__file__).resolve().parents[1]


def main():
    run=Path((ROOT/'runs/latest.txt').read_text().strip());rclpy.init()
    node=rclpy.create_node('task1_beacon_validation',parameter_overrides=[Parameter('use_sim_time',value=True)])
    pub=node.create_publisher(String,'/task1/scene_command',10)
    state=[None];rows=[];writer=[None];images={};mode=[None];view=['top']
    fcu=[None];node.create_subscription(State,'/mavros/state',lambda m:fcu.__setitem__(0,m),qos_profile_sensor_data)
    node.create_subscription(String,'/task1/scene_state',lambda m:state.__setitem__(0,json.loads(m.data)),10)
    def receive(msg):
        a=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        rgb=a if msg.encoding=='rgb8' else cv2.cvtColor(a,cv2.COLOR_BGR2RGB)
        bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
        if writer[0] is None:writer[0]=cv2.VideoWriter(str(run/'beacon_states.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),5,(msg.width,msg.height))
        writer[0].write(bgr)
        roi=rgb[msg.height//2-50:msg.height//2+50,msg.width//2-50:msg.width//2+50].astype(float)
        counts=[int(np.sum((roi[:,:,i]>100)&(roi[:,:,i]>1.8*roi[:,:,(i+1)%3])&(roi[:,:,i]>1.8*roi[:,:,(i+2)%3]))) for i in range(3)]
        body=rgb[msg.height//2-200:msg.height//2+200,msg.width//2-200:msg.width//2+200]
        white=int(np.sum((body.min(axis=2)>160)&((body.max(axis=2).astype(int)-body.min(axis=2))<30)))
        rows.append(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,requested=mode[0],view=view[0],
                         white_body_pixels=white,dominant_rgb_pixels=counts,
                         scene_stamp=state[0]['stamp'] if state[0] else None,
                         entry=next(b for b in state[0]['buoys'] if b['id']=='entry') if state[0] else None))
        key=(view[0],mode[0],max(counts)>10)
        channel=mode[0]-1 if mode[0] in (1,2) else 2
        state_matches=state[0] and next(b for b in state[0]['buoys'] if b['id']=='entry')['state']==mode[0]
        color_matches=max(counts)<=10 or (mode[0] in (1,2,3,4) and counts[channel]>10)
        if view[0] in ('top','side') and key not in images and state_matches and color_matches:
            images[key]=True;cv2.imwrite(str(run/f'beacon_{view[0]}_{mode[0]}_{int(max(counts)>10)}.png'),bgr)
    node.create_subscription(Image,'/task1/beacon_view',receive,qos_profile_sensor_data)
    start=time.monotonic()
    while (state[0] is None or fcu[0] is None) and time.monotonic()-start<20:rclpy.spin_once(node,timeout_sec=.1)
    if state[0] is None:raise RuntimeError('No authoritative scene state')
    if fcu[0] is None or not fcu[0].connected or fcu[0].armed:raise RuntimeError('Model checks require a connected, unarmed SITL')
    for semantic in range(5):
        mode[0]=semantic;pub.publish(String(data=json.dumps({'op':'set_states','states':{'entry':semantic}})))
        begin=node.get_clock().now().nanoseconds/1e9;end=time.monotonic()+20
        while node.get_clock().now().nanoseconds/1e9-begin<8:
            if time.monotonic()>end:raise TimeoutError('Camera test simulation did not advance')
            rclpy.spin_once(node,timeout_sec=.1)
    # Same real camera at vessel-level elevation: the recessed top light must be occluded.
    request='name: "beacon_camera", position: {x: 8, y: 0, z: 0.49}, orientation: {w: 1}'
    def camera_pose(request):
        result=subprocess.run(['gz','service','-s','/world/usv_nav/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean',
                               '--timeout','3000','--req',request],check=True,capture_output=True,text=True,timeout=5)
        if 'true' not in result.stdout:raise RuntimeError('Camera pose command failed: '+result.stdout)
    def spin_sim(duration):
        begin=node.get_clock().now().nanoseconds/1e9;deadline=time.monotonic()+duration*3+5
        while node.get_clock().now().nanoseconds/1e9-begin<duration:
            if time.monotonic()>deadline:raise TimeoutError('Simulation stopped during camera viewpoint check')
            rclpy.spin_once(node,timeout_sec=.1)
    view[0]='transition';camera_pose(request);spin_sim(1)
    view[0]='side';spin_sim(4);view[0]='transition'
    camera_pose('name: "beacon_camera", position: {x: 10, y: 0, z: 2}, orientation: {y: 0.7071067811865475, w: 0.7071067811865475}')
    pub.publish(String(data=json.dumps({'op':'set_states','states':{'entry':3}})))
    for _ in range(5):rclpy.spin_once(node,timeout_sec=.1)
    result={}
    for semantic in range(5):
        samples=[r for r in rows if r['view']=='top' and r['requested']==semantic and r['entry'] and r['entry']['state']==semantic]
        if samples:samples=[r for r in samples if r['stamp']>samples[0]['stamp']+.5]
        channel=semantic-1 if semantic in (1,2) else 2
        on=sum(r['dominant_rgb_pixels'][channel]>10 for r in samples)
        off=sum(max(r['dominant_rgb_pixels'])<=10 for r in samples)
        result[str(semantic)]=dict(frames=len(samples),on=on,off=off,
            passed=len(samples)>=10 and (off>=10 if semantic==0 else on>=10 if semantic==4 else on>=5 and off>=5))
        if semantic in (1,2,3):
            transitions=[b['stamp'] for a,b in zip(samples,samples[1:]) if (a['dominant_rgb_pixels'][channel]>10)!=(b['dominant_rgb_pixels'][channel]>10)]
            durations=[b-a for a,b in zip(transitions,transitions[1:])]
            result[str(semantic)]['received_transition_intervals_s']=durations
            result[str(semantic)]['phase_evidence']=blink_evidence(samples,channel)
            result[str(semantic)]['passed'] &= result[str(semantic)]['phase_evidence']['passed']
    side=[r for r in rows if r['view']=='side'];side_ok=len(side)>=10 and all(max(r['dominant_rgb_pixels'])<=10 for r in side)
    off_body=[r['white_body_pixels'] for r in rows if r['view']=='top' and r['requested']==0]
    body_ok=bool(off_body and min(off_body)>1000)
    report=dict(passed=all(r['passed'] for r in result.values()) and side_ok and body_ok,states=result,
                side_aperture_occluded=side_ok,off_body_visible=body_ok,samples=rows)
    (run/'model_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    if writer[0]:writer[0].release()
    node.destroy_node();rclpy.try_shutdown();print(json.dumps(result))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
