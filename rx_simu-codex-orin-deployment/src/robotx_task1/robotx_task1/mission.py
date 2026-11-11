"""Task1 orchestration. All vessel pose/velocity comes from MAVROS EKF via TF."""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.msg import SpeedLimit
from mavros_msgs.msg import State
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from robotx_task1_interfaces.msg import BuoyArray, MissionContext, MotionPermission, TrafficSafety, RouteAssessment
from .core import CircleTracker, passage_events, wrap
from .traffic import TrafficLease
from .route_policy import observation_goal, bounded_observation_path, terminal_scale


def seconds(stamp):return stamp.sec+stamp.nanosec/1e9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Mission(Node):
    def __init__(self):
        super().__init__('task1_mission')
        self.declare_parameter('run_dir','');self.declare_parameter('circle_radius',6.)
        self.declare_parameter('stop_speed',.04);self.declare_parameter('stop_yaw_rate',.04)
        self.declare_parameter('stop_dwell',1.5)
        self.declare_parameter('traffic_yield_wait',90.)
        self.declare_parameter('enable_stationkeeping',False)
        self.enable_stationkeeping=self.get_parameter('enable_stationkeeping').value
        self.station_resume_until=-1e9
        self.station_hold_pending=False
        self.declare_parameter('circle_completion_margin',.05)
        # Reserve time for the 10 Hz mission tick, bridge forwarding and the
        # independent recorder within the 0.6 s interruption response budget.
        self.declare_parameter('observation_timeout',.35)
        self.observation_timeout=self.get_parameter('observation_timeout').value
        self.radius=self.get_parameter('circle_radius').value
        self.circle_minimum=2*math.pi+self.get_parameter('circle_completion_margin').value
        self.stop_speed=self.get_parameter('stop_speed').value
        self.stop_yaw=self.get_parameter('stop_yaw_rate').value;self.stop_dwell=self.get_parameter('stop_dwell').value
        self.traffic_yield_wait=self.get_parameter('traffic_yield_wait').value
        self.yield_until=None
        self.route_wait_until=None;self.route_wait_started=None;self.preferred_path_stamp=None
        self.route_assessment=None;self.route_stamp=-1.;self.route_wall=0.
        self.corridor_path=None;self.full_plan_failures=0;self.observation_distance=3.
        self.plan_is_observation=False;self.executing_observation=False;self.planned_target=None
        run=self.get_parameter('run_dir').value
        self.log=(Path(run)/'task1_mission.jsonl').open('w') if run else None
        self.obs=None;self.buoys=[];self.pose=None;self.odom=None;self.pose_stamp=-1.;self.pose_wall=0.
        self.state=None;self.state_wall=0.;self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.stage='WAIT_SURVEY';self.motion='IDLE';self.started=False;self.generation=0
        self.bits=0;self.circle=None;self.arc_index=0;self.entry=None;self.exit=None
        self.target=None;self.plan_future=None;self.send_future=None;self.result_future=None
        self.follow_handle=None;self.request_generation=0;self.cancel_requested=False
        self.still_since=None;self.last_retry=-1e9;self.started_at=None;self.last_path=None
        self.previous_pose=None;self.reason='waiting for start';self.last_revision=0
        self.last_event_pass={};self.obs_wall=0.;self.last_source_stamp=0.
        self.traffic=TrafficLease();self.traffic_message=None
        self.plan_client=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.follow_client=ActionClient(self,FollowPath,'/follow_path')
        self.context_pub=self.create_publisher(MissionContext,'/task1/context',10)
        self.permission_pub=self.create_publisher(MotionPermission,'/task1/motion_permission',10)
        self.speed_limit_pub=self.create_publisher(SpeedLimit,'/task1/speed_limit',1)
        self.status_pub=self.create_publisher(String,'/task1/status',10)
        self.events_pub=self.create_publisher(String,'/task1/events',10)
        self.path_pub=self.create_publisher(NavPath,'/task1/reference_path',10)
        self.candidate_pub=self.create_publisher(NavPath,'/task1/candidate_path',10)
        self.scene_pub=self.create_publisher(String,'/task1/scene_command',10)
        self.create_subscription(BuoyArray,'/task1/observations',self.on_observation,10)
        self.create_subscription(Odometry,'/usv/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(TrafficSafety,'/task1/traffic_safety',self.on_traffic,10)
        self.create_subscription(RouteAssessment,'/task1/route_assessment',self.on_route,10)
        self.create_subscription(State,'/mavros/state',self.on_state,qos_profile_sensor_data)
        self.create_service(Trigger,'/task1/start',self.start)
        self.create_service(Trigger,'/task1/cancel',self.cancel)
        self.create_timer(.1,self.tick)

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def on_state(self,msg):
        self.state=msg;self.state_wall=time.monotonic()

    def event(self,kind,**values):
        row=dict(sim=self.now(),wall=time.monotonic(),event=kind,stage=self.stage,
                 motion=self.motion,revision=self.last_revision,generation=self.generation,**values)
        text=json.dumps(row,allow_nan=False)
        if self.log:self.log.write(text+'\n');self.log.flush()
        self.events_pub.publish(String(data=text));self.get_logger().info(text)

    def start(self,request,response):
        if self.started:
            response.message='Task already started; restart session for a new attempt';return response
        if not self.state or not self.state.connected or not self.state.armed or self.state.mode!='GUIDED':
            response.message='Connected, normally armed GUIDED SITL required';return response
        self.started=True;self.started_at=self.now();self.motion='STOPPING'
        self.scene_pub.publish(String(data=json.dumps({'op':'activate'})))
        self.event('started',observer='known_state_substitute',robocommand='local_only')
        response.success=True;response.message='Waiting for complete surveyed observations';return response

    def cancel(self,request,response):
        self.abort('task cancelled');response.success=True;response.message=self.reason;return response

    def interrupt(self,reason):
        self.yield_until=None
        self.route_wait_until=None
        self.generation+=1;self.motion='STOPPING';self.reason=reason;self.still_since=None
        if self.follow_handle and not self.cancel_requested:
            self.follow_handle.cancel_goal_async();self.cancel_requested=True
        self.permission(False);self.event('stop_requested',reason=reason)

    def abort(self,reason):
        if self.stage=='ABORTED':return
        self.stage='ABORTED';self.interrupt(reason);self.motion='ABORTED';self.event('aborted',reason=reason)

    def on_observation(self,msg):
        t=seconds(msg.header.stamp)
        if t<=self.last_source_stamp or msg.revision<self.last_revision:return
        if msg.header.frame_id!='map' or not -.1<=self.now()-t<=self.observation_timeout:return
        data=[dict(id=b.id,x=b.position.x,y=b.position.y,state=b.state,radius=b.collision_radius) for b in msg.buoys]
        if len(data)!=10 or len({b['id'] for b in data})!=10:return
        if any(b['state'] not in range(5) or b['radius']<=0 or not all(math.isfinite(b[k]) for k in ('x','y','radius')) for b in data):return
        entries=[b for b in data if b['state']==3];exits=[b for b in data if b['state']==4]
        if len(entries)!=1 or len(exits)!=1:
            if self.started:self.abort('invalid entry/exit assignment')
            return
        if self.entry and (entries[0]['id']!=self.entry['id'] or exits[0]['id']!=self.exit['id']):
            self.abort('entry/exit reassigned during attempt');return
        changed=self.obs is not None and msg.revision!=self.last_revision
        old_states={b['id']:b['state'] for b in self.buoys}
        self.obs=msg;self.buoys=data;self.last_source_stamp=t;self.last_revision=msg.revision;self.obs_wall=time.monotonic()
        if changed and self.started and self.stage not in ('COMPLETE','ABORTED','WAIT_SURVEY'):
            self.preferred_path_stamp=None
            self.route_wait_started=None
            self.corridor_path=None;self.full_plan_failures=0;self.observation_distance=3.
            self.interrupt('semantic revision changed')
            self.event('semantic_update_reported',source_stamp=t,
                       changes=[b for b in data if old_states.get(b['id'])!=b['state']],report_sink='local_only')

    def on_odom(self,msg):
        t=seconds(msg.header.stamp)
        if t<=self.pose_stamp or not -.1<=self.now()-t<=.5:return
        try:
            transform=self.tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
        except Exception:return
        a=yaw(transform.rotation);p=msg.pose.pose.position;c,s=math.cos(a),math.sin(a)
        current=[transform.translation.x+c*p.x-s*p.y,transform.translation.y+s*p.x+c*p.y,
                 wrap(a+yaw(msg.pose.pose.orientation))]
        self.previous_pose=self.pose;self.pose=current;self.odom=msg
        self.pose_stamp=t;self.pose_wall=time.monotonic()
        if self.circle and self.stage in ('CIRCLE_ENTRY','CIRCLE_EXIT'):
            was=self.circle.complete;self.circle.update(current)
            if not was and self.circle.complete:self.event('circle_completed',angle=self.circle.angle)
        if self.stage=='TRANSIT' and self.previous_pose and self.obs:
            for e in passage_events(self.previous_pose,current,self.buoys):
                # Do not accumulate repeated boundary jitter at the same marker.
                if self.now()-self.last_event_pass.get(e['id'],-1e9)<2.:continue
                self.last_event_pass[e['id']]=self.now();self.event('ekf_passage',**e)
                if not e['legal']:self.abort('EKF detected wrong-side/ambiguous passage: '+e['id'])
                else:self.bits|=1 if e['state']==1 else 2

    def on_traffic(self,msg):
        if msg.header.frame_id!='map':return
        if self.traffic.update(seconds(msg.header.stamp),seconds(msg.path_stamp),msg.clear,msg.reason,
                               self.now(),time.monotonic()):self.traffic_message=msg

    def on_route(self,msg):
        stamp=seconds(msg.header.stamp)
        values=[getattr(msg,key) for key in ('eta_s','wait_s','reference_eta_s','reference_wait_s','switch_cost_s','conflict_eta_s','observation_budget_s')]
        if (msg.header.frame_id!='map' or stamp<=self.route_stamp or msg.revision!=self.last_revision
                or not -.05<=self.now()-stamp<=1. or not all(math.isfinite(v) for v in values)):
            return
        self.route_assessment=msg;self.route_stamp=stamp;self.route_wall=time.monotonic()

    def candidate_route_ready(self):
        if self.stage!='TRANSIT':return True
        decision=self.route_assessment
        expected=seconds(self.preferred_path_stamp) if self.preferred_path_stamp else 0.
        if (decision is None or not -.05<=self.now()-self.route_stamp<=1.
                or not 0<=time.monotonic()-self.route_wall<=1.
                or decision.revision!=self.last_revision
                or seconds(decision.path_stamp)!=seconds(self.last_path.header.stamp)
                or seconds(decision.reference_stamp)!=expected):
            self.reason='waiting for source-timed arrival assessment';return False
        elapsed=0. if self.route_wait_started is None else self.now()-self.route_wait_started
        observe_remaining=max(0.,decision.observation_budget_s-elapsed)
        budget_open=(self.yield_until is None or self.now()<self.yield_until) and elapsed<self.traffic_yield_wait
        prefer=decision.prefer_wait and budget_open
        observe=observe_remaining>0. and budget_open
        future_blocked=decision.wait_s!=0.
        if prefer or observe or future_blocked:
            if self.route_wait_started is None:self.route_wait_started=self.now()
            if self.yield_until is None:self.yield_until=self.route_wait_started+self.traffic_yield_wait
            predicted=decision.reference_wait_s if prefer else decision.wait_s
            wait=min(15.,max(5.,predicted))
            if observe:wait=min(15.,observe_remaining)
            if prefer:wait=min(wait,max(0.,self.yield_until-self.now()))
            self.route_wait_until=self.now()+wait
            self.motion='WAIT_ROUTE';self.reason=decision.reason
            self.event('route_wait_selected',source_stamp=self.route_stamp,
                       path_stamp=seconds(decision.path_stamp),reference_stamp=expected,
                       eta_s=decision.eta_s,wait_s=decision.wait_s,
                       reference_eta_s=decision.reference_eta_s,reference_wait_s=decision.reference_wait_s,
                       switch_cost_s=decision.switch_cost_s,prefer_reference=prefer,recheck_after_s=wait,
                       observation_budget_s=decision.observation_budget_s,observation_only=observe,
                       wait_elapsed_s=elapsed)
            return False
        return True

    def permission(self,allow):
        m=MotionPermission();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='map'
        m.revision=self.last_revision;m.allow=allow;m.reason=self.reason
        if self.enable_stationkeeping:
            m.station_action='resume'
            free=not any((self.follow_handle,self.send_future,self.result_future))
            valid=self.inputs_valid() and self.traffic.valid(self.now(),time.monotonic())
            holding=bool(self.state and (self.state.mode=='LOITER' or self.station_hold_pending))
            if (self.stage in ('COMPLETE','ABORTED') or not valid):m.station_action='stop'
            elif (self.started and free and (self.motion in ('WAIT_CLEAR','WAIT_ROUTE','ENTERING_HOLD')
                  or (holding and self.motion in ('PLANNING','CHECKING_TRAFFIC')))
                  and self.now()>=self.station_resume_until):
                v=self.odom.twist.twist.linear
                if self.traffic_message is not None and self.traffic_message.hold_clear:
                    # The entry speed limit is not a command to interrupt an
                    # ongoing native correction. Excursion/health remain gated.
                    if holding or math.hypot(v.x,v.y)<.15:m.station_action='hold'
                elif holding:m.station_action='stop'
            elif holding and self.motion!='RESUMING':
                m.station_action='stop'
            if m.station_action=='hold':self.station_hold_pending=self.state.mode!='LOITER'
        self.permission_pub.publish(m)

    def publish_speed_limit(self):
        target=self.planned_target or self.target
        scale=1.
        if self.stage=='TRANSIT' and self.pose is not None and target is not None:
            scale=float(terminal_scale(math.hypot(self.pose[0]-target[0],self.pose[1]-target[1])))
        m=SpeedLimit();m.header.stamp=self.get_clock().now().to_msg();m.percentage=True
        m.speed_limit=100.*scale;self.speed_limit_pub.publish(m)
        return m.speed_limit

    def publish_context(self):
        m=MissionContext();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='map'
        m.revision=self.last_revision;m.phase=self.stage;m.enforce_passage=self.stage=='TRANSIT';m.passed_colors=self.bits
        if self.stage=='TRANSIT' and self.preferred_path_stamp is not None:m.preferred_path_stamp=self.preferred_path_stamp
        if self.stage in ('CIRCLE_ENTRY','CIRCLE_EXIT') and self.circle:
            m.circle_center.x=float(self.circle.center[0]);m.circle_center.y=float(self.circle.center[1])
            m.circle_radius=self.radius;m.circle_direction=self.circle.direction
        self.context_pub.publish(m)

    def inputs_valid(self):
        return (self.obs is not None and self.obs.survey_complete and self.pose is not None and
                -.1<=self.now()-self.last_source_stamp<=self.observation_timeout and
                time.monotonic()-self.obs_wall<=self.observation_timeout and
                -.1<=self.now()-self.pose_stamp<=.5 and time.monotonic()-self.pose_wall<=.5)

    def input_ages(self):
        """Diagnostic only; record which original freshness gate actually failed."""
        sim,wall=self.now(),time.monotonic()
        values=dict(observation_source=sim-self.last_source_stamp,observation_receipt=wall-self.obs_wall,
                    ekf_source=sim-self.pose_stamp,ekf_receipt=wall-self.pose_wall,
                    traffic_source=sim-self.traffic.stamp,traffic_receipt=wall-self.traffic.received)
        return {key:(value if math.isfinite(value) else None) for key,value in values.items()}

    def traffic_permits_path(self):
        return (self.last_path is not None and
                self.traffic.permits(self.now(),time.monotonic(),seconds(self.last_path.header.stamp)))

    def select_target(self):
        if self.stage=='APPROACH_ENTRY':self.target=[self.entry['x']-self.radius,self.entry['y'],math.pi/2]
        elif self.stage=='TRANSIT':self.target=[self.exit['x']-self.radius,self.exit['y'],-math.pi/2]
        else:
            marker=self.entry if self.stage=='CIRCLE_ENTRY' else self.exit
            direction=-1 if self.stage=='CIRCLE_ENTRY' else 1
            sweep=min(self.arc_index,4)*math.pi/2+max(0,self.arc_index-4)*.12
            a=math.pi+direction*sweep
            self.target=[marker['x']+self.radius*math.cos(a),marker['y']+self.radius*math.sin(a),wrap(a+direction*math.pi/2)]

    def advance(self):
        self.yield_until=None
        self.route_wait_until=None;self.route_wait_started=None;self.preferred_path_stamp=None
        self.corridor_path=None;self.full_plan_failures=0;self.observation_distance=3.
        if self.stage=='APPROACH_ENTRY':
            self.stage='CIRCLE_ENTRY';self.arc_index=1;self.circle=CircleTracker([self.entry['x'],self.entry['y']],-1,self.radius,self.circle_minimum);self.circle.update(self.pose)
        elif self.stage in ('CIRCLE_ENTRY','CIRCLE_EXIT'):
            if self.circle.complete:
                if self.stage=='CIRCLE_ENTRY':
                    self.stage='TRANSIT';self.circle=None
                    self.scene_pub.publish(String(data=json.dumps({'op':'transit'})))
                else:
                    self.stage='COMPLETE';self.motion='COMPLETE';self.reason='task completed'
                    self.permission(False);self.event('completed',target=self.target,ekf_pose=self.pose);return
            else:
                self.arc_index+=1
                if self.arc_index>10:self.abort('full circle closure was not achieved');return
        elif self.stage=='TRANSIT':
            if self.bits!=3:self.abort('transit finished without both valid colors');return
            self.stage='CIRCLE_EXIT';self.arc_index=1;self.circle=CircleTracker([self.exit['x'],self.exit['y']],1,self.radius,self.circle_minimum);self.circle.update(self.pose)
        self.select_target();self.motion='STOPPING';self.still_since=None;self.reason='stage transition; wait for vessel stop'
        self.event('stage_target',target=self.target)

    @staticmethod
    def path_points(path):
        return [[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in path.poses]

    def next_plan_target(self):
        self.plan_is_observation=False
        if self.stage=='TRANSIT' and self.bits==3 and self.full_plan_failures>=2 and self.corridor_path:
            point=observation_goal(self.path_points(self.corridor_path),self.pose,self.target,self.observation_distance)
            if point is not None:
                self.plan_is_observation=True;return point
        return self.target

    def plan_failed(self,status,reason='no legal path'):
        self.last_retry=self.now()
        if self.stage=='TRANSIT':
            if self.plan_is_observation:
                self.observation_distance-=1.
                if self.observation_distance<1.:self.full_plan_failures=0;self.observation_distance=3.
            else:self.full_plan_failures+=1
        self.motion='WAIT_CLEAR';self.reason=reason
        self.event('plan_failed',status=status,purpose='observation' if self.plan_is_observation else 'mission',reason=reason)

    def observation_completed(self):
        self.executing_observation=False;self.full_plan_failures=0;self.observation_distance=3.
        self.motion='STOPPING';self.still_since=None;self.yield_until=None;self.route_wait_until=None
        self.route_wait_started=None
        self.reason='observation point reached; wait for fresh map and replan mission goal'
        self.permission(False)
        self.event('observation_point_reached',observation_target=self.planned_target,mission_target=self.target,ekf_pose=self.pose)

    def leave_station_for_replan(self):
        if self.state.mode=='LOITER':
            self.station_hold_pending=False
            self.motion='RESUMING';self.reason='leave LOITER, confirm GUIDED, then replan current pose'
            self.event('station_resume_requested')
        else:
            self.motion='ENTERING_HOLD';self.reason='await in-flight LOITER acknowledgement before release'

    def request_plan(self):
        if not self.plan_client.server_is_ready() or not self.follow_client.server_is_ready():return
        self.route_wait_until=None
        self.planned_target=self.next_plan_target();target=self.planned_target
        self.publish_context();goal=ComputePathToPose.Goal();goal.planner_id='Task1';goal.use_start=False
        goal.goal=PoseStamped();goal.goal.header.frame_id='map';goal.goal.header.stamp=self.get_clock().now().to_msg()
        goal.goal.pose.position.x=float(target[0]);goal.goal.pose.position.y=float(target[1])
        goal.goal.pose.orientation.z=math.sin(target[2]/2);goal.goal.pose.orientation.w=math.cos(target[2]/2)
        self.plan_future=self.plan_client.send_goal_async(goal);self.plan_handle=None;self.plan_result=None
        self.request_generation=self.generation;self.motion='PLANNING';self.last_retry=self.now()
        self.event('planning',target=self.target,planned_target=target,purpose='observation' if self.plan_is_observation else 'mission')

    def poll_actions(self):
        if self.plan_future and self.plan_future.done():
            handle=self.plan_future.result();self.plan_future=None
            if handle.accepted:self.plan_handle=handle;self.plan_result=handle.get_result_async()
            elif self.motion=='PLANNING':self.motion='WAIT_CLEAR';self.reason='planner goal rejected'
        if getattr(self,'plan_result',None) and self.plan_result.done():
            result=self.plan_result.result();self.plan_result=None;self.plan_handle=None
            if self.request_generation==self.generation and self.motion=='PLANNING':
                if result.status!=4 or not result.result.path.poses:
                    # Back off from completion, not request time. A ten-second
                    # failed search otherwise starts another in this same tick,
                    # repeatedly holding the costmap lock before it can update.
                    self.plan_failed(result.status)
                elif (self.plan_is_observation and not bounded_observation_path(self.path_points(result.result.path),
                        self.path_points(self.corridor_path),self.planned_target)):
                    self.plan_failed(6,'observation route exceeds short same-corridor bound')
                else:
                    if not self.plan_is_observation:self.full_plan_failures=0;self.observation_distance=3.
                    self.last_path=result.result.path;self.candidate_pub.publish(self.last_path)
                    self.motion='CHECKING_TRAFFIC';self.reason='waiting for candidate path traffic assessment'
                    self.event('path_candidate',poses=len(self.last_path.poses),target=self.target,
                               planned_target=self.planned_target,purpose='observation' if self.plan_is_observation else 'mission')
        if self.send_future and self.send_future.done():
            self.follow_handle=self.send_future.result();self.send_future=None
            if not self.follow_handle.accepted:
                self.follow_handle=None;self.motion='WAIT_CLEAR';self.reason='controller goal rejected'
            else:
                self.result_future=self.follow_handle.get_result_async();self.cancel_requested=False
                if self.request_generation!=self.generation or self.motion!='STARTING':
                    self.follow_handle.cancel_goal_async();self.cancel_requested=True
                else:self.motion='EXECUTING';self.reason='fresh path executing';self.event('execution_started')
        if self.result_future and self.result_future.done():
            result=self.result_future.result();was_cancel=self.cancel_requested
            self.follow_handle=None;self.result_future=None;self.cancel_requested=False
            self.event('controller_result',status=result.status,cancelled=was_cancel)
            if not was_cancel and self.motion=='EXECUTING':
                if result.status==4:
                    if self.executing_observation:self.observation_completed()
                    else:self.advance()
                else:self.interrupt('controller failed; wait and replan')

    def tick(self):
        speed_limit=self.publish_speed_limit()
        self.publish_context();self.poll_actions()
        if getattr(self,'enable_stationkeeping',False) and self.started and self.stage not in ('COMPLETE','ABORTED'):
            if self.state and self.state.mode not in ('GUIDED','LOITER'):
                self.abort('FCU left task modes; operator or failsafe owns motion')
            if self.motion=='ENTERING_HOLD' and self.state and self.state.mode=='LOITER':self.leave_station_for_replan()
            if self.motion=='RESUMING' and self.state and self.state.mode=='GUIDED' and time.monotonic()-self.state_wall<2.:
                self.motion='STOPPING';self.still_since=None;self.last_retry=self.now()
                self.station_resume_until=self.now()+5.
                self.event('station_guided_confirmed')
        if self.started and self.stage not in ('COMPLETE','ABORTED'):
            if self.now()-self.started_at>1200:self.abort('1200 sim-second task timeout')
            elif not self.inputs_valid() or not self.traffic.valid(self.now(),time.monotonic()):
                if self.motion not in ('STOPPING','IDLE'):
                    self.event('stale_inputs',ages_s=self.input_ages())
                    self.interrupt('observation, EKF or LiDAR traffic assessment stale')
                self.still_since=None
            else:
                if not self.traffic.clear and self.motion in ('EXECUTING','STARTING'):
                    self.interrupt('LiDAR traffic: '+self.traffic.reason)
                    if self.route_wait_started is None:self.route_wait_started=self.now()
                    self.yield_until=self.route_wait_started+self.traffic_yield_wait
                    self.event('traffic_yield',source_stamp=self.traffic.stamp,
                               track_id=self.traffic_message.track_id,
                               conflict_after_s=self.traffic_message.conflict_after_s)
                if self.stage=='WAIT_SURVEY':
                    self.entry=next(b for b in self.buoys if b['state']==3);self.exit=next(b for b in self.buoys if b['state']==4)
                    self.event('observations_reported',source_stamp=self.last_source_stamp,buoys=self.buoys,report_sink='local_only')
                    self.stage='APPROACH_ENTRY';self.select_target();self.event('survey_accepted',target=self.target)
                if (self.motion=='CHECKING_TRAFFIC' and self.traffic.path_stamp==seconds(self.last_path.header.stamp)
                        and self.candidate_route_ready()):
                    if self.traffic_permits_path() and self.traffic.clear_for(self.now(),time.monotonic()):
                        if (getattr(self,'enable_stationkeeping',False) and self.state
                                and (self.state.mode=='LOITER' or self.station_hold_pending)):
                            # Keep a fixed native target while evaluating blocked
                            # alternatives. A usable candidate only triggers
                            # release; its path is discarded and recomputed once
                            # GUIDED is acknowledged and the vessel has settled.
                            self.leave_station_for_replan()
                        else:
                            if self.stage=='TRANSIT':
                                d=self.route_assessment
                                self.event('route_selected',source_stamp=self.route_stamp,path_stamp=seconds(d.path_stamp),
                                           eta_s=d.eta_s,reference_eta_s=d.reference_eta_s,switch_cost_s=d.switch_cost_s,
                                           purpose='observation' if self.plan_is_observation else 'mission',
                                           preference_budget_expired=self.yield_until is not None and self.now()>=self.yield_until)
                            self.route_wait_started=None;self.yield_until=None
                            self.executing_observation=self.plan_is_observation
                            if self.stage=='TRANSIT' and not self.plan_is_observation:self.corridor_path=self.last_path
                            self.preferred_path_stamp=self.last_path.header.stamp
                            self.publish_context()
                            self.path_pub.publish(self.last_path)
                            goal=FollowPath.Goal();goal.path=self.last_path;goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
                            self.send_future=self.follow_client.send_goal_async(goal);self.motion='STARTING'
                            self.event('path_ready',poses=len(goal.path.poses),target=self.target,
                                       planned_target=self.planned_target,purpose='observation' if self.plan_is_observation else 'mission')
                    elif not self.traffic.clear:
                        self.motion='WAIT_CLEAR';self.reason='candidate path blocked by traffic';self.last_retry=self.now()
                        self.event('path_blocked_traffic',source_stamp=self.traffic.stamp)
                if self.motion in ('STOPPING','WAIT_CLEAR','WAIT_ROUTE'):
                    v=self.odom.twist.twist.linear;w=self.odom.twist.twist.angular
                    if math.hypot(v.x,v.y)<self.stop_speed and abs(w.z)<self.stop_yaw:
                        if self.still_since is None:self.still_since=self.now()
                    else:self.still_since=None
                    free=not any((self.follow_handle,self.send_future,self.plan_future,getattr(self,'plan_result',None)))
                    retry_delay=2. if self.traffic.clear else 5.
                    hold_ready=(getattr(self,'enable_stationkeeping',False) and self.state and self.state.mode=='LOITER'
                                and math.hypot(v.x,v.y)<.15)
                    if (free and ((self.still_since is not None and self.now()-self.still_since>=self.stop_dwell) or hold_ready)
                            and self.now()-self.last_retry>=retry_delay):
                        # Compute alternatives while stopped, then compare wait
                        # and detour using fresh arrival estimates. A timeout
                        # only expires route preference, never safety gates.
                        if self.route_wait_until is None or self.now()>=self.route_wait_until:
                            self.request_plan()
        allow=(self.motion=='EXECUTING' and self.inputs_valid() and self.traffic_permits_path()
               and self.stage not in ('COMPLETE','ABORTED'))
        self.permission(allow)
        status=dict(sim=self.now(),stage=self.stage,motion=self.motion,revision=self.last_revision,
                    speed_limit_percent=speed_limit,
                    target=self.target,pose=self.pose,reason=self.reason,passed_colors=self.bits,
                    route_purpose='observation' if self.executing_observation else 'mission',
                    circle_angle=self.circle.angle if self.circle else None)
        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    rclpy.init();node=Mission()
    try:rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():node.permission(False)
    finally:
        if node.log:node.log.close()
        node.destroy_node();rclpy.try_shutdown()
