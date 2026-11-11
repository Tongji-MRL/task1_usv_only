#!/usr/bin/env python3
"""Exercise the real ROS/MAVROS/SITL chain and record independent VRX truth."""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import Float64, String, UInt32
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, ComputePathToPose
from lifecycle_msgs.srv import GetState
from usv_sim.geometry import vessel_clearance
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Runner:
    def __init__(self, label):
        self.run = Path((ROOT/'runs/latest.txt').read_text().strip())
        self.scene=json.loads((self.run/'generated/scene.json').read_text())
        model=ET.parse(self.run/'generated/usv_nav.sdf')
        self.thrust_factors={p.findtext('name'):float(p.findtext('fluid_density'))*
                            float(p.findtext('thrust_coefficient'))*float(p.findtext('propeller_diameter'))**4
                            for p in model.findall('.//plugin') if p.get('name')=='gz::sim::systems::Thruster'}
        (self.run/'thrust_observation.json').write_text(json.dumps({
            'force_from_feedback':'density * coefficient * diameter^4 * abs(omega) * omega',
            'factors':self.thrust_factors,
            'meaning':'Force applied by the configured Gazebo ForceCmd/velocity_control model, inferred from its PreUpdate feedback; not a physical load cell or shaft dynamics measurement.',
            'source':'https://github.com/gazebosim/gz-sim/blob/gz-sim7_7.9.0/src/systems/thruster/Thruster.cc'},indent=2)+'\n')
        self.node = rclpy.create_node('validation_'+label, parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.data = {}
        self.received = {}
        self.rows = []
        self.telemetry = {}
        self.mav_available = True
        self.contacts = []
        self.contact_messages = 0
        self.alignment = None
        self.log = (self.run/(label+'.jsonl')).open('w')
        self.last_record = 0
        self.phase = 'startup'
        for topic, cls in [('/usv/ground_truth', Odometry), ('/mavros/local_position/odom', Odometry),
                           ('/scan', LaserScan), ('/mavros/state', State), ('/usv/cmd_vel', Twist),
                           ('/usv/setpoint_preview', PositionTarget), ('/usv/control_status', String),
                           ('/usv/thrust_left', Float64), ('/usv/thrust_right', Float64), ('/usv/sitl_frame', UInt32)]:
            self.node.create_subscription(cls, topic, lambda msg,t=topic:self.receive(t,msg), qos_profile_sensor_data)
        for side in ('left','right'):
            topic='/usv/thruster_'+side+'_feedback'
            self.node.create_subscription(Float64,topic,lambda msg,t=topic:self.receive(t,msg),qos_profile_sensor_data)
        self.raw = self.node.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.twist = self.node.create_publisher(Twist, '/usv/cmd_vel', 10)
        self.node.create_subscription(Contacts,'/usv/contacts',self.on_contacts,qos_profile_sensor_data)
        self.node.create_subscription(String,'/usv/map_alignment',self.on_alignment,
                                      QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline=time.monotonic()+30
        while True:
            try:
                self.mav = mavutil.mavlink_connection('tcp:127.0.0.1:5762', source_system=254, retries=1)
                break
            except OSError:
                if time.monotonic()>deadline:raise
                time.sleep(.5)
        for message_id in (24,193,36,87):
            self.mav.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,
                                           message_id,500000,0,0,0,0,0)
        self.single = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.multiple = ActionClient(self.node, NavigateThroughPoses, '/navigate_through_poses')

    def receive(self, topic, msg):
        self.data[topic] = msg
        self.received[topic] = time.monotonic()

    def on_contacts(self,msg):
        self.contact_messages += 1
        if msg.contacts:self.contacts.append(str(msg))

    def on_alignment(self,msg):
        self.alignment=json.loads(msg.data)
        (self.run/'map_alignment.json').write_text(json.dumps(self.alignment,indent=2)+'\n')

    def record(self):
        now = time.monotonic()
        if now-self.last_record < 0.1:
            return
        self.last_record = now
        row = {'wall': now, 'sim': self.node.get_clock().now().nanoseconds/1e9, 'phase': self.phase}
        row['contact_messages']=self.contact_messages
        row['collision_messages']=len(self.contacts)
        for topic, key in [('/usv/ground_truth','truth'),('/mavros/local_position/odom','ekf')]:
            if topic in self.data:
                m=self.data[topic];p=m.pose.pose.position;v=m.twist.twist.linear
                row[key]=[p.x,p.y,p.z,yaw(m.pose.pose.orientation),v.x,v.y,m.twist.twist.angular.z]
                q=m.pose.pose.orientation
                row[key+'_rpy']=[math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
                                math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x)))),yaw(q)]
                row[key+'_age']=row['sim']-m.header.stamp.sec-m.header.stamp.nanosec/1e9
                if key=='ekf' and m.header.frame_id=='odom':
                    row['ekf_local']=row[key].copy()
                    if self.alignment is not None:
                        row[key][0]+=self.alignment['offset_xy_m'][0]
                        row[key][1]+=self.alignment['offset_xy_m'][1]
                        row['map_alignment_xy']=self.alignment['offset_xy_m']
        for topic,key in [('/usv/thrust_left','left_n'),('/usv/thrust_right','right_n'),('/usv/sitl_frame','frame'),('/usv/control_status','gate')]:
            if topic in self.data:row[key]=self.data[topic].data
        row['thrust_source']='command_topic_only'
        if all('/usv/thruster_'+side+'_feedback' in self.data for side in ('left','right')):
            row['thrust_source']='thruster_feedback'
            for side in ('left','right'):
                topic='/usv/thruster_'+side+'_feedback';omega=self.data[topic].data
                row['command_'+side+'_n']=row.get(side+'_n')
                row[side+'_n']=self.thrust_factors[side]*abs(omega)*omega
                row[side+'_feedback_age_s']=now-self.received[topic]
                if row[side+'_feedback_age_s']>.5:row['thrust_source']='stale_thruster_feedback'
        if '/usv/cmd_vel' in self.data:
            m=self.data['/usv/cmd_vel'];row['command']=[m.linear.x,m.angular.z]
        if '/usv/setpoint_preview' in self.data:
            m=self.data['/usv/setpoint_preview'];row['preview']=[m.velocity.x,m.yaw_rate]
        if '/scan' in self.data:
            m=self.data['/scan'];row['scan_age']=row['sim']-m.header.stamp.sec-m.header.stamp.nanosec/1e9
        if 'truth' in row:
            p=row['truth'];row['clearance_m']=vessel_clearance(self.scene['footprint'],p[0],p[1],p[3],self.scene['obstacles'])
        if '/mavros/state' in self.data:
            state=self.data['/mavros/state']
            row['fcu_state']={'connected':state.connected,'armed':state.armed,'mode':state.mode,
                              'wall_age_s':now-self.received['/mavros/state']}
        for name in ('GPS_RAW_INT','EKF_STATUS_REPORT','SERVO_OUTPUT_RAW','HEARTBEAT','POSITION_TARGET_GLOBAL_INT'):
            if name in self.telemetry:row[name]=self.telemetry[name]
        self.rows.append(row)
        self.log.write(json.dumps(row,allow_nan=False)+'\n');self.log.flush()

    def spin(self, duration=0.05):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            rclpy.spin_once(self.node,timeout_sec=min(0.02,max(0,end-time.monotonic())))
            # Keep receiving ACK/status without emitting an extra GCS heartbeat.
            while True:
                if not self.mav_available:break
                message=self.mav.recv_match(blocking=False)
                if message is None:break
                if message.get_type() in ('GPS_RAW_INT','EKF_STATUS_REPORT','SERVO_OUTPUT_RAW','HEARTBEAT','POSITION_TARGET_GLOBAL_INT') and message.get_srcSystem()==1 and message.get_srcComponent()==1:
                    self.telemetry[message.get_type()]=message.to_dict()
                if message.get_type()=='STATUSTEXT':
                    self.log.write(json.dumps({'status_text':message.text,'wall':time.monotonic()})+'\n')
            self.record()

    def wait(self, predicate, timeout, description):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.spin()
            if predicate():return
        raise TimeoutError(description)

    def ready(self, scan=True):
        def predicate():
            state=self.data.get('/mavros/state')
            needed=['/usv/ground_truth','/mavros/local_position/odom']+(['/scan'] if scan else [])
            return state is not None and state.connected and all(t in self.received and time.monotonic()-self.received[t]<0.5 for t in needed)
        self.wait(predicate,90,'Fresh EKF / VRX / scan not ready')
        p=self.data['/usv/ground_truth'].pose.pose.position
        if math.hypot(p.x,p.y)>40:raise RuntimeError('Unexpected vessel location')

    def arm(self):
        self.mav.mav.set_mode_send(1,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,15)
        self.wait(lambda:self.data.get('/mavros/state') and self.data['/mavros/state'].mode=='GUIDED',10,'GUIDED not entered')
        # EKF can publish fresh predictions before its normal pre-arm checks pass.
        # Retry normal arming while initialization settles; never force-arm.
        end=time.monotonic()+60
        while time.monotonic()<end:
            self.mav.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
            self.spin(2)
            if self.data['/mavros/state'].armed:return
        raise TimeoutError('Normal arming rejected; inspect STATUSTEXT')

    def command(self, speed, angular, direct=False):
        if direct:
            msg=PositionTarget();msg.header.stamp=self.node.get_clock().now().to_msg();msg.header.frame_id='base_link'
            msg.coordinate_frame=8;msg.type_mask=1511;msg.velocity.x=float(speed);msg.yaw_rate=float(angular)
            self.raw.publish(msg)
        else:
            msg=Twist();msg.linear.x=float(speed);msg.angular.z=float(angular);self.twist.publish(msg)

    def motion(self, direct):
        self.arm()
        summaries=[]
        for name,v,w,duration in [('forward',0.25,0,10),('stop',0,0,8),('left',0,0.2,8),('stop',0,0,8),('right',0,-0.2,8),('stop',0,0,8)]:
            self.phase=name
            start=len(self.rows);end=time.monotonic()+duration
            while time.monotonic()<end:
                self.command(v,w,direct);self.spin()
            values=self.rows[start:]
            summaries.append({'phase':name,'first':values[0] if values else {},'last':values[-1] if values else {}})
        return summaries

    def goal(self, poses, cancel_after=None, expected_status=4):
        self.navigation_ready()
        self.arm()
        self.phase='navigation'
        client=self.single if len(poses)==1 else self.multiple
        if not client.wait_for_server(timeout_sec=15):raise TimeoutError('Nav2 action unavailable')
        messages=[]
        for x,y,heading in poses:
            m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.node.get_clock().now().to_msg()
            m.pose.position.x=float(x);m.pose.position.y=float(y)
            m.pose.orientation.z=math.sin(heading/2);m.pose.orientation.w=math.cos(heading/2);messages.append(m)
        goal=NavigateToPose.Goal() if len(poses)==1 else NavigateThroughPoses.Goal()
        if len(poses)==1:goal.pose=messages[0]
        else:goal.poses=messages
        request=client.send_goal_async(goal)
        self.wait(request.done,15,'Goal acknowledgement timeout')
        handle=request.result()
        if not handle.accepted:raise RuntimeError('Goal rejected')
        future=handle.get_result_async();start=time.monotonic();cancelled=False
        while not future.done() and time.monotonic()-start<240:
            self.spin()
            if cancel_after and time.monotonic()-start>cancel_after and not cancelled:
                handle.cancel_goal_async();cancelled=True
        if not future.done():
            handle.cancel_goal_async();raise TimeoutError('Navigation exceeded 240 seconds')
        result=future.result();m=self.data['/usv/ground_truth'];p=m.pose.pose.position;target=poses[-1]
        error=math.hypot(p.x-target[0],p.y-target[1]);heading_error=abs(math.atan2(math.sin(yaw(m.pose.pose.orientation)-target[2]),math.cos(yaw(m.pose.pose.orientation)-target[2])))
        at_goal=[p.x,p.y,yaw(m.pose.pose.orientation)]
        self.phase='post_goal'
        completion=time.monotonic();zero_delay=None
        while time.monotonic()-completion<10:
            self.spin()
            preview=self.data.get('/usv/setpoint_preview')
            if preview and abs(preview.velocity.x)+abs(preview.yaw_rate)<1e-7 and zero_delay is None:
                zero_delay=time.monotonic()-completion
        after=self.data['/usv/ground_truth'].pose.pose.position
        clearance=min((row['clearance_m'] for row in self.rows if 'clearance_m' in row),default=None)
        waypoints=[]
        for x,y,h in poses[:-1]:
            closest=min((r for r in self.rows if r.get('phase')=='navigation' and 'truth' in r),
                        key=lambda r:math.dist(r['truth'][:2],[x,y]))['truth']
            waypoints.append({'target':[x,y,h],'closest_position_error_m':math.dist(closest[:2],[x,y]),
                              'yaw_error_at_closest_rad':abs(math.atan2(math.sin(closest[3]-h),math.cos(closest[3]-h)))})
        return {'status':result.status,'elapsed_wall_s':time.monotonic()-start-10,'true_position_error_m':error,
                'true_yaw_error_rad':heading_error,'at_goal':at_goal,'drift_10s_m':math.hypot(after.x-at_goal[0],after.y-at_goal[1]),
                'zero_command_delay_s':zero_delay,'min_planar_clearance_m':clearance,'expected_status':expected_status,
                'contact_messages':self.contact_messages,'collision_messages':len(self.contacts),
                'via_points':waypoints,
                'passed':result.status==expected_status and (expected_status!=4 or (error<=.5 and heading_error<=.25))
                         and all(p['closest_position_error_m']<=.5 and p['yaw_error_at_closest_rad']<=.25 for p in waypoints)
                         and clearance is not None and clearance>0 and not self.contacts
                         and zero_delay is not None and zero_delay<=.6}

    def plan(self):
        self.navigation_ready()
        self.phase='planning_preview'
        client=ActionClient(self.node,ComputePathToPose,'/compute_path_to_pose')
        if not client.wait_for_server(timeout_sec=10):raise TimeoutError('Planner unavailable')
        goal=ComputePathToPose.Goal();goal.goal.header.frame_id='map'
        goal.goal.header.stamp=self.node.get_clock().now().to_msg()
        goal.goal.pose.position.x=18.;goal.goal.pose.orientation.w=1.;goal.planner_id='GridBased'
        request=client.send_goal_async(goal);self.wait(request.done,10,'Planner acknowledgement')
        handle=request.result()
        if not handle.accepted:raise RuntimeError('Planning rejected')
        result=handle.get_result_async();self.wait(result.done,15,'Planning timeout')
        value=result.result();path=[[p.pose.position.x,p.pose.position.y] for p in value.result.path.poses]
        (self.run/'preview_path.json').write_text(json.dumps(path)+'\n')
        return {'status':value.status,'path_points':len(path),'passed':value.status==4 and len(path)>2,
                'max_lateral_deviation_m':max((abs(p[1]) for p in path),default=0)}

    def navigation_ready(self):
        client=self.node.create_client(GetState,'/bt_navigator/get_state')
        if not client.wait_for_service(timeout_sec=15):raise TimeoutError('Nav2 lifecycle unavailable')
        end=time.monotonic()+30
        while time.monotonic()<end:
            future=client.call_async(GetState.Request());self.wait(future.done,5,'Nav2 lifecycle response')
            if future.result().current_state.id==3:return
            self.spin(.5)
        raise TimeoutError('Nav2 did not become active')

    def close(self):
        # Cleanup HOLD/disarm must never count as the controller's stopping response.
        self.phase='cleanup'
        self.command(0,0,True)
        if self.mav_available:
            self.mav.mav.set_mode_send(1,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,4)
            self.spin(.3)
            self.mav.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)
        self.spin(.3);self.mav.close();self.log.close();self.node.destroy_node()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('case',choices=['motion','goal','probe','plan'])
    parser.add_argument('--direct',action='store_true',help='Stage B MAVROS test, with bridge output disabled')
    parser.add_argument('--goals',default='[[5,0,0]]')
    parser.add_argument('--cancel-after',type=float)
    parser.add_argument('--label',default='validation')
    parser.add_argument('--expect-status',type=int,default=4)
    args=parser.parse_args();rclpy.init();runner=Runner(args.label)
    try:
        runner.ready(scan=not args.direct)
        if args.case=='motion':result=runner.motion(args.direct)
        elif args.case=='goal':result=runner.goal(json.loads(args.goals),args.cancel_after,args.expect_status)
        elif args.case=='plan':result=runner.plan()
        else:runner.spin(10);result=runner.rows[-1]
        (runner.run/(args.label+'_result.json')).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    finally:
        runner.close();rclpy.shutdown()


if __name__=='__main__':
    main()
