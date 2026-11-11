"""Exercise permission and action transitions; no ROS node or flight controller."""
import time
from types import SimpleNamespace as NS
import unittest
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from robotx_task1.mission import Mission
from robotx_task1.traffic import TrafficLease


class TrafficMissionTests(unittest.TestCase):
    def mission(self,motion='STOPPING',clear=False):
        m=NS(stage='TRANSIT',motion=motion,started=True,started_at=0.,stamp=10.,last_retry=0.,
             still_since=8.,stop_dwell=1.5,stop_speed=.04,stop_yaw=.04,
             follow_handle=None,send_future=None,plan_future=None,plan_result=None,result_future=None,
             reason='',last_revision=1,bits=0,pose=[0,0,0],circle=None,target=[20,0,0],
             generation=1,request_generation=1,yield_until=None,traffic_yield_wait=60.,
             route_wait_until=None,route_wait_started=None,preferred_path_stamp=None,route_stamp=10.,route_wall=time.monotonic(),
             corridor_path=None,full_plan_failures=0,observation_distance=3.,
             plan_is_observation=False,executing_observation=False,planned_target=[20,0,0],
             traffic=TrafficLease(),last_path=Path(),events=[],permissions=[],plans=0,follows=0)
        m.last_path.header.stamp.sec=8
        m.odom=NS(twist=NS(twist=NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))))
        m.now=lambda:m.stamp;m.publish_context=lambda:None;m.poll_actions=lambda:None
        m.publish_speed_limit=lambda:100.
        m.inputs_valid=lambda:True;m.permission=lambda value:m.permissions.append(value)
        m.input_ages=lambda:dict(traffic_source=m.now()-m.traffic.stamp)
        m.status_pub=NS(publish=lambda msg:None);m.path_pub=NS(publish=lambda msg:None)
        m.event=lambda event,**kw:m.events.append((event,kw))
        m.request_plan=lambda:setattr(m,'plans',m.plans+1)
        m.traffic_permits_path=lambda:Mission.traffic_permits_path(m)
        def interrupt(reason):m.motion='STOPPING';m.reason=reason;m.events.append(('stop_requested',reason))
        m.interrupt=interrupt
        def follow(goal):m.follows+=1;return NS()
        m.follow_client=NS(send_goal_async=follow)
        m.traffic.update(10.,8.,clear,'test traffic',10.,time.monotonic())
        m.traffic_message=NS(track_id=1,conflict_after_s=3.)
        m.route_assessment=NS(path_stamp=m.last_path.header.stamp,reference_stamp=Path().header.stamp,
             revision=1,eta_s=60.,wait_s=0.,reference_eta_s=-1.,reference_wait_s=-1.,
             switch_cost_s=0.,observation_budget_s=0.,prefer_wait=False,reason='test')
        m.candidate_route_ready=lambda:Mission.candidate_route_ready(m)
        m.plan_failed=lambda status,reason='no legal path':Mission.plan_failed(m,status,reason)
        m.path_points=Mission.path_points
        m.observation_completed=lambda:Mission.observation_completed(m)
        return m

    def test_stopped_vessel_can_replan_while_old_path_is_blocked(self):
        m=self.mission();Mission.tick(m)
        self.assertEqual(m.plans,1)
        self.assertFalse(m.permissions[-1])

    def test_crossing_yield_can_compute_alternatives_while_motion_remains_disabled(self):
        m=self.mission();m.yield_until=65.;Mission.tick(m)
        self.assertEqual(m.plans,1)
        self.assertFalse(m.permissions[-1])

    def test_clear_crossing_replans_against_latest_map_before_moving(self):
        m=self.mission(clear=True);m.yield_until=65.
        for i in range(1,12):
            m.stamp=10+i/10
            m.traffic.update(m.stamp,8.,True,'clear',m.stamp,time.monotonic())
        Mission.tick(m)
        self.assertEqual(m.last_path.header.stamp.sec,8)
        self.assertEqual(m.plans,1)
        self.assertEqual(m.follows,0)
        self.assertFalse(m.permissions[-1])

    def test_persistent_blockage_replans_after_yield_budget(self):
        m=self.mission();m.yield_until=9.;Mission.tick(m)
        self.assertEqual(m.plans,1)
        self.assertFalse(m.permissions[-1])

    def test_short_clear_gap_can_plan_but_cannot_resume_motion(self):
        m=self.mission(clear=True);m.yield_until=65.;Mission.tick(m)
        self.assertEqual(m.motion,'STOPPING')
        self.assertEqual(m.follows,0)
        self.assertEqual(m.plans,1)

    def test_semantic_interrupt_invalidates_waiting_route_resume(self):
        m=self.mission();m.yield_until=65.;m.cancel_requested=False
        Mission.interrupt(m,'semantic revision changed')
        self.assertIsNone(m.yield_until)
        self.assertEqual(m.generation,2)
        self.assertFalse(m.permissions[-1])

    def test_old_assessment_cannot_start_new_candidate(self):
        m=self.mission('CHECKING_TRAFFIC',True);m.last_path.header.stamp.sec=9
        Mission.tick(m)
        self.assertEqual(m.follows,0)
        self.assertFalse(m.permissions[-1])

    def test_blocked_candidate_is_never_sent_to_controller(self):
        m=self.mission('CHECKING_TRAFFIC');Mission.tick(m)
        self.assertEqual(m.motion,'WAIT_CLEAR')
        self.assertEqual(m.follows,0)
        self.assertFalse(m.permissions[-1])

    def test_clear_candidate_waits_for_continuous_clearance_then_starts(self):
        m=self.mission('CHECKING_TRAFFIC',True);Mission.tick(m)
        self.assertEqual(m.follows,0)
        for i in range(1,12):
            m.stamp=10+i/10
            m.traffic.update(m.stamp,8.,True,'clear',m.stamp,time.monotonic())
            m.route_stamp=m.stamp;m.route_wall=time.monotonic()
        Mission.tick(m)
        self.assertEqual(m.follows,1)
        self.assertEqual(m.motion,'STARTING')
        self.assertFalse(m.permissions[-1])

    def test_preferred_wait_keeps_control_closed_and_replans_after_timer(self):
        m=self.mission('CHECKING_TRAFFIC',True);m.route_assessment.prefer_wait=True
        m.route_assessment.reference_wait_s=8.
        Mission.tick(m)
        self.assertEqual(m.motion,'WAIT_ROUTE');self.assertEqual(m.follows,0);self.assertEqual(m.plans,0)
        self.assertEqual(m.route_wait_until,18.)
        m.stamp=18.;m.traffic.update(18.,8.,True,'clear',18.,time.monotonic())
        Mission.tick(m);self.assertEqual(m.plans,1);self.assertFalse(m.permissions[-1])

    def test_expired_preference_budget_does_not_override_future_conflict(self):
        m=self.mission('CHECKING_TRAFFIC',True);m.yield_until=9.
        m.route_assessment.prefer_wait=True;m.route_assessment.wait_s=-1.
        Mission.tick(m);self.assertEqual(m.motion,'WAIT_ROUTE');self.assertEqual(m.follows,0)

    def test_expired_preference_allows_safe_detour_without_reusing_old_path(self):
        m=self.mission('CHECKING_TRAFFIC',True);m.yield_until=9.;m.route_assessment.prefer_wait=True
        self.assertTrue(Mission.candidate_route_ready(m))
        self.assertEqual(m.follows,0)

    def test_wait_budget_does_not_start_during_clear_travel(self):
        m=self.mission('CHECKING_TRAFFIC',True)
        self.assertTrue(Mission.candidate_route_ready(m))
        self.assertIsNone(m.yield_until);self.assertIsNone(m.route_wait_started)

    def test_unknown_departure_only_spends_finite_observation_budget(self):
        m=self.mission('CHECKING_TRAFFIC',True);m.route_assessment.observation_budget_s=30.
        self.assertFalse(Mission.candidate_route_ready(m))
        self.assertEqual(m.route_wait_started,10.);self.assertEqual(m.route_wait_until,25.)
        self.assertTrue(m.events[-1][1]['observation_only'])
        m.motion='CHECKING_TRAFFIC';m.stamp=35.;m.route_stamp=35.
        self.assertFalse(Mission.candidate_route_ready(m));self.assertEqual(m.route_wait_until,40.)
        m.motion='CHECKING_TRAFFIC';m.stamp=40.;m.route_stamp=40.
        self.assertTrue(Mission.candidate_route_ready(m));self.assertEqual(m.follows,0)

    def test_sensor_interrupt_cannot_renew_observation_budget(self):
        m=self.mission('CHECKING_TRAFFIC',True);m.route_assessment.observation_budget_s=60.
        m.route_wait_started=-55.;m.cancel_requested=False
        Mission.interrupt(m,'observation, EKF or LiDAR traffic assessment stale')
        self.assertEqual(m.route_wait_started,-55.)
        self.assertTrue(Mission.candidate_route_ready(m));self.assertEqual(m.follows,0)

    def test_stale_arrival_assessment_cannot_start_candidate(self):
        m=self.mission('CHECKING_TRAFFIC',True);m.route_stamp=8.
        m.traffic.clear_since=8.
        Mission.tick(m);self.assertEqual(m.follows,0);self.assertFalse(m.permissions[-1])

    def test_wrong_reference_or_revision_cannot_authorize_switch(self):
        m=self.mission('CHECKING_TRAFFIC',True)
        m.route_assessment.reference_stamp.sec=7
        self.assertFalse(Mission.candidate_route_ready(m))
        m.route_assessment.reference_stamp.sec=0;m.route_assessment.revision=2
        self.assertFalse(Mission.candidate_route_ready(m))

    def test_expired_traffic_assessment_interrupts_execution(self):
        m=self.mission('EXECUTING',True);m.stamp=10.5;Mission.tick(m)
        self.assertEqual(m.motion,'STOPPING')
        self.assertFalse(m.permissions[-1])
        self.assertIn(('stale_inputs',dict(ages_s=dict(traffic_source=.5))),m.events)

    def test_failed_search_backoff_starts_when_search_finishes(self):
        m=self.mission('PLANNING',True)
        m.plan_result=NS(done=lambda:True,result=lambda:NS(status=6,result=NS(path=Path())))
        Mission.poll_actions(m);self.assertEqual(m.motion,'WAIT_CLEAR')
        self.assertEqual(m.last_retry,10.)
        Mission.tick(m);self.assertEqual(m.plans,0);self.assertFalse(m.permissions[-1])
        m.stamp=12.;m.traffic.update(12.,8.,True,'clear',12.,time.monotonic())
        Mission.tick(m);self.assertEqual(m.plans,1);self.assertFalse(m.permissions[-1])

    def test_observation_goal_requires_prior_corridor_and_two_failed_full_searches(self):
        m=self.mission();m.bits=3;m.corridor_path=Path()
        for x in (0.,20.):
            p=PoseStamped();p.pose.position.x=x;p.pose.orientation.w=1.;m.corridor_path.poses.append(p)
        m.full_plan_failures=1;self.assertEqual(Mission.next_plan_target(m),m.target)
        m.full_plan_failures=2;goal=Mission.next_plan_target(m)
        self.assertTrue(m.plan_is_observation);self.assertGreaterEqual(goal[0],3.);self.assertLess(goal[0],3.5)
        self.assertEqual(m.target,[20,0,0])
        m.bits=1;self.assertEqual(Mission.next_plan_target(m),m.target)
        self.assertFalse(m.plan_is_observation)

    def test_observation_controller_success_does_not_complete_the_transit(self):
        m=self.mission('EXECUTING',True);m.executing_observation=True;m.cancel_requested=False
        m.result_future=NS(done=lambda:True,result=lambda:NS(status=4))
        m.advance=lambda:self.fail('An observation goal is not the mission goal')
        Mission.poll_actions(m)
        self.assertEqual(m.stage,'TRANSIT');self.assertEqual(m.target,[20,0,0])
        self.assertEqual(m.motion,'STOPPING');self.assertFalse(m.permissions[-1])
        self.assertEqual(m.full_plan_failures,0);self.assertIsNone(m.still_since)

    def test_failed_observation_goal_tries_shorter_bound_without_granting_motion(self):
        m=self.mission('PLANNING',True);m.plan_is_observation=True;m.full_plan_failures=2
        Mission.plan_failed(m,6)
        self.assertEqual(m.observation_distance,2.);self.assertEqual(m.motion,'WAIT_CLEAR')
        self.assertEqual(m.last_retry,10.);self.assertEqual(m.follows,0)


if __name__=='__main__':unittest.main()
