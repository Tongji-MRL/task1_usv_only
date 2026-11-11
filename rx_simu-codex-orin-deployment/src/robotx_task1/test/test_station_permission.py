"""Mode handoff must not overlap a navigation action or reuse stale GUIDED."""
import time
import unittest
from types import SimpleNamespace as NS
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from robotx_task1.mission import Mission


class StationPermissionTests(unittest.TestCase):
    def mission(self):
        self.messages=[]
        return NS(enable_stationkeeping=True,station_resume_until=0.,station_hold_pending=False,
            last_revision=1,reason='wait',started=True,stage='TRANSIT',motion='WAIT_CLEAR',
            follow_handle=None,send_future=None,result_future=None,state=NS(mode='GUIDED'),
            odom=Odometry(),traffic_message=NS(hold_clear=True),inputs_valid=lambda:True,
            traffic=NS(valid=lambda *_:True),now=lambda:10.,
            get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time(sec=10))),
            permission_pub=NS(publish=self.messages.append),event=lambda *_:None)

    def test_wait_entry_requires_cancelled_action_and_clear_rotation_area(self):
        m=self.mission();m.result_future=object();Mission.permission(m,False)
        self.assertEqual(self.messages[-1].station_action,'resume')
        m.result_future=None;m.traffic_message.hold_clear=False;Mission.permission(m,False)
        self.assertEqual(self.messages[-1].station_action,'resume')
        m.traffic_message.hold_clear=True;Mission.permission(m,False)
        self.assertEqual(self.messages[-1].station_action,'hold')
        self.assertTrue(m.station_hold_pending)

    def test_inflight_entry_cannot_plan_against_old_guided_confirmation(self):
        m=self.mission();Mission.permission(m,False);Mission.leave_station_for_replan(m)
        self.assertEqual(m.motion,'ENTERING_HOLD')
        Mission.permission(m,False);self.assertEqual(self.messages[-1].station_action,'hold')
        m.state.mode='LOITER';Mission.leave_station_for_replan(m)
        self.assertEqual(m.motion,'RESUMING')
        self.assertFalse(m.station_hold_pending)
        Mission.permission(m,False);self.assertEqual(self.messages[-1].station_action,'resume')

    def test_candidate_computation_keeps_same_hold_until_release_is_requested(self):
        m=self.mission();Mission.permission(m,False);m.motion='PLANNING';Mission.permission(m,False)
        self.assertEqual(self.messages[-1].station_action,'hold')
        m.state.mode='LOITER'
        for phase in ('PLANNING','CHECKING_TRAFFIC','WAIT_ROUTE'):
            m.motion=phase;Mission.permission(m,False)
            self.assertEqual(self.messages[-1].station_action,'hold')
        Mission.leave_station_for_replan(m);Mission.permission(m,False)
        self.assertEqual(self.messages[-1].station_action,'resume')

    def test_fault_or_new_occupied_hold_area_revokes_pending_entry(self):
        m=self.mission();Mission.permission(m,False)
        m.traffic_message.hold_clear=False;Mission.permission(m,False)
        self.assertEqual(self.messages[-1].station_action,'stop')

    def test_obstacle_during_candidate_planning_cannot_silently_resume(self):
        m=self.mission();m.state.mode='LOITER'
        for phase in ('PLANNING','CHECKING_TRAFFIC','WAIT_ROUTE'):
            m.motion=phase;m.traffic_message.hold_clear=False;Mission.permission(m,False)
            self.assertEqual(self.messages[-1].station_action,'stop')

    def test_native_correction_does_not_recapture_target_or_resume_early(self):
        m=self.mission();m.odom.twist.twist.linear.x=.2
        Mission.permission(m,False)
        self.assertEqual(self.messages[-1].station_action,'resume')
        m.state.mode='LOITER';m.motion='PLANNING';Mission.permission(m,False)
        self.assertEqual(self.messages[-1].station_action,'hold')
        m.stage='ABORTED';Mission.permission(m,False)
        self.assertEqual(self.messages[-1].station_action,'stop')


if __name__=='__main__':unittest.main()
