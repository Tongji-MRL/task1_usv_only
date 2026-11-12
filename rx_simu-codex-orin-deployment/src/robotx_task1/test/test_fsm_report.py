"""Messages to the team's USV state machine (usv_orin_state_machine version2); no ROS."""
import json
import unittest

from robotx_task1 import robocommand_core as rc
from robotx_task1.fsm_report import ReportEnvelope, StateMachineLink, progress

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
        self.assertNotEqual(progress('ABORT', 'total time limit reached')['progress'], 'FINISHED')

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

    def test_abort_reports_the_buoys_and_does_not_finish(self):
        bridge = link()
        out = bridge.update(0.0, dict(state='ABORT', detail='total time limit reached'), BODY, 0.0)
        self.assertEqual([kind for kind, _ in out], ['result', 'progress'])
        self.assertEqual(out[1][1], dict(vehicle='usv', task='TASK1', progress='TASK1_ABORT',
                                         error='total time limit reached'))

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

    def test_sequence_continues_from_the_configured_start(self):
        envelope = ReportEnvelope('TONG', 'usv', 7)
        self.assertEqual(envelope.safe_passage({}, 0.0)['seq'], 8)
        with self.assertRaises(ValueError):
            ReportEnvelope('', 'usv')


if __name__ == '__main__':
    unittest.main()
