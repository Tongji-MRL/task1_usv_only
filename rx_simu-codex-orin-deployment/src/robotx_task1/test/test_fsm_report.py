"""Messages to the team's USV state machine (usv_orin_state_machine version2); no ROS."""
import json
import unittest

from robotx_task1 import robocommand_core as rc
from robotx_task1.fsm_report import (IncidentLink, ReportEnvelope, StateMachineLink, StateMachineWatch,
                                     course_corners, local_xy, progress, task4_command)

BODY = rc.safe_passage([dict(id='B1', x=1.0, y=0.0, state=rc.RED), dict(id='B2', x=0.0, y=1.0, state=rc.GREEN),
                        dict(id='B3', x=2.0, y=2.0, state=rc.FLASH_BLUE),
                        dict(id='B4', x=3.0, y=3.0, state=rc.UNKNOWN)], (0.0, 0.0), (1.28, 103.85))


def link(**options):
    return StateMachineLink(ReportEnvelope('TONG', 'usv', 0), **options)


class ProgressTests(unittest.TestCase):
    def test_fields_match_the_state_machine_and_task3(self):
        self.assertEqual(progress('THREAD'), dict(vehicle='usv', task='TASK1', progress='TASK1_THREAD', error=None))
        # version2 ends Task 1 on exactly this message.
        finished = progress('COMPLETE')
        self.assertEqual((finished['vehicle'], finished['task'], finished['progress']), ('usv', 'TASK1', 'FINISHED'))
        # An abort also ends Task 1 for the state machine; the reason goes in `error`.
        self.assertEqual(progress('ABORT', 'total time limit reached'),
                         dict(vehicle='usv', task='TASK1', progress='FINISHED', error='total time limit reached'))

    def test_progress_on_change_and_refresh_only(self):
        bridge = link(refresh=2.0)
        self.assertEqual(bridge.update(0.0, {}, None, 0.0), [])
        self.assertEqual([p['progress'] for _, p in bridge.update(0.0, dict(state='WAIT_START'), None, 0.0)],
                         ['TASK1_WAIT_START'])
        self.assertEqual(bridge.update(0.5, dict(state='WAIT_START'), None, 0.5), [])
        self.assertEqual(len(bridge.update(2.0, dict(state='WAIT_START'), None, 2.0)), 1)  # Refresh.
        self.assertEqual([p['progress'] for _, p in bridge.update(2.1, dict(state='SEARCH_ENTRY'), BODY, 2.1)],
                         ['TASK1_SEARCH_ENTRY'])  # No report before the end.


class EndTests(unittest.TestCase):
    def test_complete_sends_one_report_then_finished(self):
        bridge = link(repeat=1.0)
        bridge.update(0.0, dict(state='CIRCLE_EXIT'), BODY, 1794450000.0)
        out = bridge.update(1.0, dict(state='COMPLETE', detail='exit circle complete'), BODY, 1794450001.0)
        self.assertEqual([kind for kind, _ in out], ['result', 'progress'])  # Report before FINISHED.
        report = out[0][1]
        self.assertEqual({k: report[k] for k in ('team_id', 'vehicle_id', 'seq')},
                         dict(team_id='TONG', vehicle_id='usv', seq=1))
        self.assertEqual(report['sent_at'], '2026-11-12T02:20:01Z')
        self.assertEqual(sorted(b['state'] for b in report['safe_passage']['buoys']),
                         ['BEACON_STATE_FLASHING_BLUE', 'BEACON_STATE_FLASHING_GREEN', 'BEACON_STATE_FLASHING_RED'])
        self.assertIn('entry_position', report['safe_passage'])
        self.assertNotIn('exit_position', report['safe_passage'])
        json.dumps(report)
        self.assertEqual(out[1][1]['progress'], 'FINISHED')
        self.assertEqual(bridge.update(1.5, dict(state='COMPLETE'), BODY, 0.0), [])
        again = bridge.update(2.0, dict(state='COMPLETE'), BODY, 0.0)
        self.assertEqual([(k, p['progress']) for k, p in again], [('progress', 'FINISHED')])  # Never a 2nd report.

    def test_abort_reports_the_buoys_then_finishes_with_the_reason(self):
        bridge = link()
        bridge.update(0.0, dict(state='THREAD'), BODY, 0.0)
        self.assertIsNone(bridge.finished_at)
        out = bridge.update(3.0, dict(state='ABORT', detail='total time limit reached'), BODY, 0.0)
        self.assertEqual([kind for kind, _ in out], ['result', 'progress'])
        self.assertEqual(out[1][1], dict(vehicle='usv', task='TASK1', progress='FINISHED',
                                         error='total time limit reached'))
        self.assertEqual(bridge.finished_at, 3.0)  # When the end was first reported (exit timer).
        bridge.update(4.0, dict(state='ABORT'), BODY, 0.0)
        self.assertEqual(bridge.finished_at, 3.0)

    def test_final_progress_waits_for_a_georeferenced_report(self):
        bridge = link(report_wait=5.0)
        bridge.update(0.0, dict(state='CIRCLE_EXIT'), None, 0.0)
        self.assertEqual(bridge.update(1.0, dict(state='COMPLETE'), None, 1.0), [])
        out = bridge.update(2.0, dict(state='COMPLETE'), BODY, 2.0)  # Fix is back in time.
        self.assertEqual([kind for kind, _ in out], ['result', 'progress'])
        bridge = link(report_wait=5.0)
        bridge.update(0.0, dict(state='COMPLETE'), None, 0.0)
        out = bridge.update(5.0, dict(state='COMPLETE'), None, 5.0)  # Gave up: finish without it.
        self.assertEqual([(k, p['progress']) for k, p in out], [('progress', 'FINISHED')])
        self.assertTrue(bridge.missing_report)

    def test_lost_state_machine_ends_task1_only_after_it_was_seen(self):
        watch = StateMachineWatch(5.0)
        self.assertFalse(watch.update(0.0, False))
        self.assertFalse(watch.update(100.0, False))  # Never seen (run on its own): never expires.
        self.assertFalse(watch.update(101.0, True))
        self.assertFalse(watch.update(105.0, False))
        self.assertFalse(watch.update(106.0, True))   # Back in time.
        self.assertTrue(watch.update(111.5, False))   # Gone for more than 5 s.
        self.assertFalse(StateMachineWatch(0.0).update(1e6, False))  # Disabled.

    def test_sequence_continues_from_the_configured_start(self):
        envelope = ReportEnvelope('TONG', 'usv', 7)
        self.assertEqual(envelope.safe_passage({}, 0.0)['seq'], 8)
        with self.assertRaises(ValueError):
            ReportEnvelope('', 'usv')


CORNERS = [{'latitude': 1.2810, 'longitude': 103.8540}, {'latitude': 1.2810, 'longitude': 103.8543},
           {'latitude': 1.2813, 'longitude': 103.8543}, {'latitude': 1.2813, 'longitude': 103.8540}]
# The two Task 4 messages of the main state machine (interface sheet of 2026-11-13).
BEGIN = {'vehicle': 'usv', 'task': 'TASK4', 'progress': None, 'error': None,
         'command': {'type': 'TASK4_BEGIN', 'command_seq': 100,
                     'center': {'latitude': 31.000001, 'longitude': 121.000001},
                     'radius_m': 500.0, 'vehicle_type': 'USV'}}
RESUME = {'vehicle': 'usv', 'task': 'TASK4', 'progress': None, 'error': None, 'command': 'resume'}


class CourseConfigTests(unittest.TestCase):
    def test_rxcourse_corners_in_any_envelope(self):
        expected = [(c['latitude'], c['longitude']) for c in CORNERS]
        for payload in ({'course_id': 'A', 'pinger_freq_hz': 0, 'corners': CORNERS},
                        {'payload': {'course': {'corners': CORNERS}}},
                        {'corners': [[c['latitude'], c['longitude']] for c in CORNERS]},
                        {'corners': [{'lat': c['latitude'], 'lon': c['longitude']} for c in CORNERS]}):
            with self.subTest(payload=payload):
                self.assertEqual(course_corners(json.dumps(payload)), expected)
                self.assertEqual(course_corners(payload), expected)

    def test_unusable_course_config(self):
        for payload in ('not json', '{}', json.dumps({'corners': CORNERS[:2]}),
                        json.dumps({'corners': CORNERS[:3]+[{'latitude': 'x', 'longitude': 0}]}),
                        json.dumps({'corners': CORNERS[:3]+[{'latitude': 95.0, 'longitude': 0}]})):
            with self.subTest(payload=payload):
                self.assertIsNone(course_corners(payload))

    def test_wgs84_to_local_is_the_inverse_of_the_report_model(self):
        ref, xy = (1.2811, 103.8541), (12.0, -3.0)
        for east, north in ((0.0, 0.0), (25.0, -40.0), (-7.5, 3.25)):
            lat, lon = rc.offset_latlon(ref[0], ref[1], east, north)
            x, y = local_xy(lat, lon, ref[0], ref[1], xy)
            self.assertAlmostEqual(x, xy[0]+east, places=3)
            self.assertAlmostEqual(y, xy[1]+north, places=3)


class Task4Tests(unittest.TestCase):
    def test_the_state_machine_commands(self):
        self.assertEqual(task4_command(json.dumps(BEGIN)), ('begin', 100, (31.000001, 121.000001), 500.0))
        self.assertEqual(task4_command(json.dumps(RESUME)), ('resume',))
        self.assertEqual(task4_command(dict(RESUME, command={'type': 'ALL_CLEAR'})), ('resume',))
        for other in (dict(BEGIN, task='TASK3'), dict(BEGIN, vehicle='uav'), dict(RESUME, command='end'),
                      'not json', json.dumps([1, 2])):
            with self.subTest(other=other):
                self.assertIsNone(task4_command(json.dumps(other) if isinstance(other, dict) else other))
        no_centre = task4_command(dict(BEGIN, command=dict(BEGIN['command'], center=None, radius_m='x')))
        self.assertEqual(no_centre, ('begin', 100, None, None))

    def test_begin_and_resume_are_both_acknowledged_with_the_begin_seq(self):
        link = IncidentLink()
        self.assertEqual(link.handle(task4_command(BEGIN)), (100, True))
        self.assertTrue(link.active)
        self.assertEqual(link.handle(task4_command(BEGIN)), (100, False))  # Repeated: acked, no change.
        self.assertEqual(link.handle(task4_command(RESUME)), (100, True))  # "resume" carries no seq.
        self.assertFalse(link.active)
        self.assertEqual(link.handle(task4_command(RESUME)), (100, False))
        self.assertEqual(IncidentLink().handle(task4_command(RESUME)), (None, False))  # Nothing to ack.
        self.assertEqual(link.handle(None), (None, False))

    def test_incident_ack_body_and_shared_sequence(self):
        envelope = ReportEnvelope('TONG', 'usv', 0)
        envelope.safe_passage({}, 0.0)
        ack = envelope.incident_ack(100, 1794450001.0)
        self.assertEqual(ack['incident_ack'], {'command_seq': 100})  # As in the interface sheet.
        self.assertEqual((ack['team_id'], ack['vehicle_id'], ack['seq']), ('TONG', 'usv', 2))
        self.assertEqual(ack['sent_at'], '2026-11-12T02:20:01Z')


if __name__ == '__main__':
    unittest.main()
