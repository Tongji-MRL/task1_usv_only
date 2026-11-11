"""RoboCommand report contents against the official RobotX 2026 schema names.

The field and enum names below are transcribed from robonation/robocommand
RobotX_2026/proto (rx_reports.proto, rx_common.proto, common.proto). Encoding
to protobuf happens on the OCS and is not exercised here.
"""
import json
import math
import sys
import unittest
from pathlib import Path

from robotx_task1 import robocommand_core as rc

sys.path.insert(0, str(Path(__file__).parents[3]/'scripts'))
import ocs_relay  # noqa: E402

HEARTBEAT_FIELDS = {'state', 'position', 'spd_mps', 'heading_deg', 'roll_deg', 'pitch_deg', 'altitude_hae_m',
                    'depth_m', 'current_task', 'vehicle_type', 'flight_phase', 'uav_registration'}
ROBOT_STATES = {'STATE_UNKNOWN', 'STATE_KILLED', 'STATE_MANUAL', 'STATE_AUTO'}
BEACON_STATES = {'BEACON_STATE_UNKNOWN', 'BEACON_STATE_OFF', 'BEACON_STATE_FLASHING_RED',
                 'BEACON_STATE_FLASHING_GREEN', 'BEACON_STATE_FLASHING_BLUE', 'BEACON_STATE_STEADY_BLUE'}


def buoy(name, x, y, state):
    return dict(id=name, x=x, y=y, state=state, radius=0.2)


class ContentTests(unittest.TestCase):
    def test_heartbeat_uses_schema_names_and_compass_heading(self):
        body = rc.heartbeat(1.2810, 103.8550, 0.25, math.pi/2, 0.02, -0.01, 12.3, 'STATE_AUTO', 'TASK_SAFE_PASSAGE')
        self.assertLessEqual(set(body), HEARTBEAT_FIELDS)
        self.assertEqual(body['position'], dict(latitude=1.2810, longitude=103.8550))
        self.assertEqual(body['heading_deg'], 0.0)       # ENU yaw 90 deg is north.
        self.assertEqual(rc.heartbeat(0, 0, 0, 0.0, 0, 0, 0, 'STATE_AUTO', 'TASK_NONE')['heading_deg'], 90.0)
        self.assertEqual(rc.heartbeat(0, 0, 0, math.pi, 0, 0, 0, 'STATE_AUTO', 'TASK_NONE')['heading_deg'], 270.0)
        self.assertEqual(body['vehicle_type'], 'TYPE_USV')
        self.assertAlmostEqual(body['roll_deg'], 1.15, places=2)
        json.dumps(body)

    def test_robot_state_and_task_follow_the_fcu_and_the_mission(self):
        self.assertEqual(rc.robot_state(True, True, 'GUIDED'), 'STATE_AUTO')
        self.assertEqual(rc.robot_state(True, True, 'MANUAL'), 'STATE_MANUAL')
        self.assertEqual(rc.robot_state(True, False, 'GUIDED'), 'STATE_KILLED')
        self.assertEqual(rc.robot_state(False, True, 'GUIDED'), 'STATE_UNKNOWN')
        self.assertLessEqual({rc.robot_state(c, a, m) for c in (0, 1) for a in (0, 1) for m in ('GUIDED', 'HOLD')},
                             ROBOT_STATES)
        for state in ('WAIT_START', 'COMPLETE', 'ABORT', ''):
            self.assertEqual(rc.current_task(state), 'TASK_NONE')
        for state in ('SEARCH_ENTRY', 'THREAD', 'CIRCLE_EXIT', 'HOLD'):
            self.assertEqual(rc.current_task(state), 'TASK_SAFE_PASSAGE')

    def test_safe_passage_reports_classified_buoys_georeferenced_from_the_vessel(self):
        buoys = [buoy('B1', 10.0, 0.0, rc.FLASH_BLUE), buoy('B2', 10.0, 100.0, rc.RED), buoy('B3', 5.0, 5.0, rc.UNKNOWN),
                 buoy('B4', -90.0, 0.0, rc.OFF), buoy('B5', 0.0, 0.0, rc.GREEN)]
        report = rc.safe_passage(buoys, boat_xy=(10.0, 0.0), boat_latlon=(1.0, 103.0))
        self.assertEqual(set(report), {'buoys', 'entry_position'})   # No exit known yet.
        self.assertEqual(report['entry_position'], dict(latitude=1.0, longitude=103.0))
        self.assertEqual(len(report['buoys']), 4)                     # The unclassified one is left out.
        self.assertLessEqual({b['state'] for b in report['buoys']}, BEACON_STATES)
        north = report['buoys'][1]
        self.assertEqual(north['state'], 'BEACON_STATE_FLASHING_RED')
        self.assertAlmostEqual((north['position']['latitude']-1.0)*111320.0, 100.0, delta=0.5)
        self.assertEqual(north['position']['longitude'], 103.0)
        west = report['buoys'][2]
        self.assertAlmostEqual((103.0-west['position']['longitude'])*111320.0*math.cos(math.radians(1.0)), 100.0, delta=0.5)
        with_exit = rc.safe_passage(buoys+[buoy('B6', 20.0, 0.0, rc.STEADY_BLUE)], (10.0, 0.0), (1.0, 103.0))
        self.assertIn('exit_position', with_exit)

    def test_report_is_sent_on_change_rate_limited_and_refreshed(self):
        schedule = rc.ReportSchedule(min_interval=1.0, refresh=10.0)
        unknown = [buoy('B1', 0.0, 0.0, rc.UNKNOWN)]
        self.assertFalse(schedule.due(0.0, unknown))            # Nothing to report yet.
        first = [buoy('B1', 0.0, 0.0, rc.RED)]
        self.assertTrue(schedule.due(1.0, first))
        self.assertFalse(schedule.due(2.5, [buoy('B1', 0.05, 0.0, rc.RED)]))  # Centimetre jitter.
        more = first+[buoy('B2', 3.0, 0.0, rc.GREEN)]
        self.assertFalse(schedule.due(1.5, more))               # Too soon after the last one.
        self.assertTrue(schedule.due(2.6, more))
        self.assertTrue(schedule.due(4.0, [buoy('B1', 0.0, 0.0, rc.GREEN), more[1]]))  # Corrected state.
        self.assertFalse(schedule.due(9.0, [buoy('B1', 0.0, 0.0, rc.GREEN), more[1]]))
        self.assertTrue(schedule.due(14.5, [buoy('B1', 0.0, 0.0, rc.GREEN), more[1]]))  # Refresh.


class RelayTests(unittest.TestCase):
    def relay(self):
        published = []

        def build(schema, team_id, vehicle_id, seq, kind, body, now):
            if kind not in ocs_relay.BODIES or not isinstance(body, dict):
                raise ValueError('bad packet')
            return type('Report', (), dict(seq=seq, kind=kind, SerializeToString=lambda self: json.dumps(
                dict(team_id=team_id, vehicle_id=vehicle_id, seq=seq, kind=kind, body=body)).encode()))()
        return ocs_relay.Relay(None, 'TEAM', 'USV1', lambda topic, payload: published.append((topic, payload)),
                               build=build), published

    def packet(self, **changes):
        return json.dumps(dict(dict(vehicle_id='USV1', kind='heartbeat', body=dict(state='STATE_AUTO')), **changes))

    def test_topic_sequence_and_rejection_of_foreign_or_broken_datagrams(self):
        relay, published = self.relay()
        self.assertEqual(relay.handle(self.packet(), 0.0).seq, 1)
        self.assertEqual(published[0][0], 'robocommand/robotx/TEAM/USV1/report')
        self.assertIsNone(relay.handle(self.packet(vehicle_id='UAV1'), 0.3))
        self.assertIsNone(relay.handle(self.packet(kind='docking'), 0.4))
        self.assertIsNone(relay.handle(b'not json', 0.5))
        self.assertIsNone(relay.handle(self.packet(body=None), 0.6))
        self.assertEqual(relay.handle(self.packet(kind='safe_passage', body=dict(buoys=[])), 0.7).seq, 2)
        self.assertEqual([json.loads(p)['seq'] for _, p in published], [1, 2])  # Refusals use no number.

    def test_at_most_five_messages_per_second(self):
        relay, published = self.relay()
        for i in range(12):
            relay.handle(self.packet(), 0.05*i)
        self.assertEqual((len(published), relay.dropped), (5, 7))
        self.assertIsNotNone(relay.handle(self.packet(), 1.01))
        self.assertEqual(relay.seq, 6)

    def test_declaration_tiers_and_topic_names_match_the_schema(self):
        self.assertEqual(ocs_relay.TIERS, dict(none='TIER_NONE', core='TIER_CORE', advanced='TIER_ADVANCED',
                                               disruptive='TIER_DISRUPTIVE'))
        self.assertEqual(ocs_relay.report_topic('RNRX', 'USV1'), 'robocommand/robotx/RNRX/USV1/report')
        self.assertEqual(set(ocs_relay.BODIES.values()), {'heartbeat', 'safe_passage'})


if __name__ == '__main__':
    unittest.main()
