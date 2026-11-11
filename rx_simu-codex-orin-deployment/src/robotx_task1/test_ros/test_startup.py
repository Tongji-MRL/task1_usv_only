"""Run on a separate ROS_DOMAIN_ID; catches Node reserved-name and API errors."""
import unittest
import copy
import json
import time
import rclpy
from robotx_task1.mission import Mission
from robotx_task1.observer import KnownObserver
from robotx_task1.traffic_guard import TrafficGuard
from robotx_task1_interfaces.msg import Buoy, BuoyArray, MissionContext, RouteAssessment
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from rclpy.time import Time


class RosStartup(unittest.TestCase):
    def test_terminal_speed_limit_reduces_and_restores_dwb_caps(self):
        rclpy.init();mission=Mission()
        try:
            messages=[];mission.speed_limit_pub=type('Publisher',(),dict(publish=lambda _,msg:messages.append(msg)))()
            mission.stage='TRANSIT';mission.pose=[0.,0.,0.];mission.planned_target=[3.,0.,0.]
            self.assertEqual(mission.publish_speed_limit(),50.)
            self.assertTrue(messages[-1].percentage)
            mission.planned_target=[5.,0.,0.]
            self.assertEqual(mission.publish_speed_limit(),100.)
            mission.stage='CIRCLE_EXIT';mission.planned_target=[1.,0.,0.]
            self.assertEqual(mission.publish_speed_limit(),100.)
            self.assertFalse(mission.started)
        finally:mission.destroy_node();rclpy.try_shutdown()

    def test_latest_context_is_valid_with_older_but_fresh_sensor_source(self):
        rclpy.init();guard=None
        try:
            guard=TrafficGuard(parameter_overrides=[Parameter('footprint',value=json.dumps([[-2.,-1.],[2.,-1.],[2.,1.],[-2.,1.]]))])
            guard.marker_revision=1;now=guard.get_clock().now().nanoseconds
            reference=Path();reference.header.frame_id='map';reference.header.stamp=Time(nanoseconds=now-5_000_000_000).to_msg()
            for x in (0.,20.):
                p=PoseStamped();p.pose.position.x=x;p.pose.orientation.w=1.;reference.poses.append(p)
            guard.on_reference(reference)
            candidate=copy.deepcopy(reference);candidate.header.stamp=Time(nanoseconds=now-100_000_000).to_msg()
            guard.on_path(candidate)
            context=MissionContext();context.header.frame_id='map';context.header.stamp=Time(nanoseconds=now).to_msg()
            context.revision=1;context.enforce_passage=True;context.preferred_path_stamp=reference.header.stamp
            guard.on_context(context)
            sensor=copy.deepcopy(candidate.header);sensor.stamp=Time(nanoseconds=now-250_000_000).to_msg()
            received=[];guard.route_pub=type('Publisher',(),dict(publish=lambda _,msg:received.append(msg)))()
            guard.assess_route(sensor,[],[0.,0.,0.])
            self.assertEqual(received[-1].reference_stamp,reference.header.stamp)
            self.assertEqual(received[-1].header.stamp,sensor.stamp)
            guard.mission_context.header.stamp=Time(nanoseconds=now-2_000_000_000).to_msg()
            guard.last_route_stamp=-float('inf');guard.assess_route(sensor,[],[0.,0.,0.])
            self.assertEqual(received[-1].reference_stamp,Time().to_msg())
        finally:
            if guard:guard.destroy_node()
            rclpy.try_shutdown()

    def test_guard_publishes_source_timed_route_assessment(self):
        rclpy.init();guard=None
        try:
            guard=TrafficGuard(parameter_overrides=[Parameter('footprint',value=json.dumps([[-2.,-1.],[2.,-1.],[2.,1.],[-2.,1.]]))])
            path=Path();path.header.frame_id='map';path.header.stamp=guard.get_clock().now().to_msg()
            for x in (0.,20.):
                p=PoseStamped();p.pose.position.x=x;p.pose.orientation.w=1.;path.poses.append(p)
            guard.on_path(path);guard.marker_revision=1
            context=MissionContext();context.header=copy.deepcopy(path.header);context.revision=1;context.enforce_passage=True
            guard.on_context(context)
            self.assertIsNotNone(guard.context)  # rclpy's context was not overwritten.
            received=[]
            sub=guard.create_subscription(RouteAssessment,'/task1/route_assessment',received.append,10)
            guard.assess_route(path.header,[],[0.,0.,0.])
            deadline=time.monotonic()+2.
            while not received and time.monotonic()<deadline:rclpy.spin_once(guard,timeout_sec=.05)
            self.assertTrue(received)
            self.assertEqual(received[0].header.stamp,path.header.stamp)
            self.assertEqual(received[0].path_stamp,path.header.stamp)
            self.assertEqual(received[0].revision,1)
            self.assertEqual(received[0].wait_s,0.)
            self.assertGreater(received[0].eta_s,20/.3)
            guard.destroy_subscription(sub)
        finally:
            if guard:guard.destroy_node()
            rclpy.try_shutdown()

    def test_stale_revision_and_semantic_cancel(self):
        rclpy.init();mission=Mission()
        try:
            msg=BuoyArray();msg.header.frame_id='map';msg.header.stamp=mission.get_clock().now().to_msg();msg.revision=1;msg.survey_complete=True
            for i in range(10):
                b=Buoy();b.id=str(i);b.position.x=float(i);b.collision_radius=.36;b.state=3 if i==0 else 4 if i==1 else 0
                msg.buoys.append(b)
            delayed=copy.deepcopy(msg)
            t=mission.get_clock().now().nanoseconds-400_000_000
            delayed.header.stamp.sec=t//1_000_000_000;delayed.header.stamp.nanosec=t%1_000_000_000
            mission.on_observation(delayed);self.assertIsNone(mission.obs)
            mission.on_observation(msg);self.assertEqual(mission.last_revision,1)
            newer=copy.deepcopy(msg);newer.header.stamp=mission.get_clock().now().to_msg();newer.revision=2;newer.buoys[2].state=1
            mission.started=True;mission.stage='TRANSIT';mission.motion='EXECUTING'
            mission.corridor_path=Path();mission.full_plan_failures=2
            mission.on_observation(newer)
            self.assertEqual(mission.motion,'STOPPING');self.assertEqual(mission.generation,1)
            self.assertIsNone(mission.corridor_path);self.assertEqual(mission.full_plan_failures,0)
            mission.on_observation(msg);self.assertEqual(mission.last_revision,2);self.assertEqual(mission.generation,1)
            future=copy.deepcopy(newer);future.header.stamp.sec+=2;future.revision=3
            mission.on_observation(future);self.assertEqual(mission.last_revision,2)
        finally:mission.destroy_node();rclpy.try_shutdown()

    def test_nodes_start_with_closed_permission(self):
        rclpy.init();mission=None;observer=None
        try:
            mission=Mission();observer=KnownObserver()
            self.assertFalse(mission.started);self.assertEqual(mission.stage,'WAIT_SURVEY')
            self.assertFalse(mission.inputs_valid())
            mission.tick()  # Includes actual generated context and permission messages.
        finally:
            if observer:observer.destroy_node()
            if mission:mission.destroy_node()
            rclpy.try_shutdown()


if __name__=='__main__':unittest.main()
