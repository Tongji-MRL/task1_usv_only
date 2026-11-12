"""Closed-loop Safe Passage runs judged by the independent rule oracle; no ROS."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import mission_sim as ms  # noqa: E402
from robotx_task1 import gap_planner as gp  # noqa: E402
from robotx_task1.mission_core import Mission  # noqa: E402


def finished(sim, out):
    return out['state'] in ('COMPLETE', 'ABORT')


def mirrored_field():
    """Same field mirrored in y; colors swapped so the transit stays solvable."""
    swap = {gp.RED: gp.GREEN, gp.GREEN: gp.RED}
    return [dict(b, y=-b['y'], state=swap.get(b['state'], b['state'])) for b in ms.example_field()]


class FullRunTests(unittest.TestCase):
    def check(self, sim, out):
        verdict = sim.judge()
        self.assertEqual(out['state'], 'COMPLETE', (out, sim.events[-5:]))
        self.assertTrue(verdict['passages'])
        self.assertTrue(all(e['legal'] for e in verdict['passages']), verdict['passages'])
        self.assertEqual(verdict['blind'], [])  # Nothing came abeam before it was classified.
        self.assertGreater(verdict['clearance'], 0.1)
        entry = next(b['id'] for b in sim.truth if b['state'] == gp.ENTRY)
        leave = next(b['id'] for b in sim.truth if b['state'] == gp.EXIT)
        self.assertLessEqual(verdict['winding'][entry][0], -2*math.pi)  # Clockwise.
        self.assertGreaterEqual(verdict['winding'][leave][1], 2*math.pi)  # Counter-clockwise.
        self.assertLessEqual(sim.max_command[0], 0.3+1e-9)
        self.assertLessEqual(sim.max_command[1], 0.3+1e-9)
        order = [e['type'] for e in sim.events]
        for earlier, later in (('started', 'entry_found'), ('entry_found', 'circle_entry_done'),
                               ('circle_entry_done', 'gap_passed'), ('gap_passed', 'circle_exit_done')):
            self.assertLess(order.index(earlier), order.index(later), order)
        self.assertEqual(order[-2:], ['circle_exit_done', 'complete'])
        return verdict

    def test_example_field_is_completed_with_every_colored_buoy_on_its_side(self):
        sim = ms.Simulation()
        sim.mission.start()
        verdict = self.check(sim, sim.run(900, until=finished))
        self.assertEqual({e['id']: e['side'] for e in verdict['passages']},
                         dict(R1='starboard', G1='port', R2='starboard', G2='port'))
        self.assertGreaterEqual(len(sim.mission.passed), 1)
        self.assertEqual(sim.mission.credit, dict(green=True, red=True))
        self.assertLess(sim.time, 400)

    def test_other_start_poses_noise_seeds_and_the_mirrored_field(self):
        cases = [dict(pose=(-3.0, 4.0, -1.0), seed=1), dict(pose=(-6.0, -1.0, 2.5), seed=2),
                 dict(buoys=mirrored_field(), pose=(-4.0, 3.0, 0.0), seed=3),
                 dict(buoys=mirrored_field(), pose=(-2.0, -5.0, 1.5), seed=4)]
        for case in cases:
            with self.subTest(pose=case['pose'], mirrored='buoys' in case):
                sim = ms.Simulation(**case)
                sim.mission.start()
                self.check(sim, sim.run(1100, until=finished))

    def test_shorter_color_range_still_never_passes_an_unclassified_buoy(self):
        original = ms.COLOR_RANGE
        ms.COLOR_RANGE = 3.0
        try:
            sim = ms.Simulation(look_range=2.8)
            sim.mission.start()
            self.check(sim, sim.run(1100, until=finished))
        finally:
            ms.COLOR_RANGE = original


class GatingTests(unittest.TestCase):
    def test_no_motion_before_the_start_request_or_without_guided(self):
        sim = ms.Simulation()
        out = sim.run(5)
        self.assertEqual((out['state'], sim.max_command), ('WAIT_START', (0.0, 0.0)))
        sim.mission.start()
        sim.mode = 'MANUAL'
        self.assertEqual(sim.run(5)['state'], 'WAIT_START')
        sim.mode, sim.armed = 'GUIDED', False
        self.assertEqual(sim.run(5)['state'], 'WAIT_START')
        sim.armed, sim.inputs_ok = True, False
        self.assertEqual(sim.run(5)['state'], 'WAIT_START')
        self.assertEqual(sim.max_command, (0.0, 0.0))
        sim.inputs_ok = True
        self.assertNotEqual(sim.run(1)['state'], 'WAIT_START')

    def test_stale_input_holds_with_zero_command_and_the_run_resumes(self):
        sim = ms.Simulation()
        sim.mission.start()
        sim.run(900, until=lambda s, o: o['state'] == 'THREAD')
        sim.inputs_ok = False
        start = len(sim.commands)
        out = sim.run(10)
        self.assertEqual(out['state'], 'HOLD')
        self.assertEqual(set(sim.commands[start:]), {(0.0, 0.0)})
        sim.inputs_ok = True
        out = sim.run(900, until=finished)
        self.assertEqual(out['state'], 'COMPLETE')
        verdict = sim.judge()
        self.assertTrue(all(e['legal'] for e in verdict['passages']), verdict['passages'])
        self.assertEqual(verdict['blind'], [])
        self.assertEqual([e['type'] for e in sim.events].count('hold'), 1)

    def test_leaving_guided_aborts_for_good(self):
        for change in (dict(mode='MANUAL'), dict(mode='HOLD'), dict(armed=False)):
            with self.subTest(change=change):
                sim = ms.Simulation()
                sim.mission.start()
                sim.run(900, until=lambda s, o: o['state'] == 'CIRCLE_ENTRY')
                for name, value in change.items():
                    setattr(sim, name, value)
                start = len(sim.commands)
                self.assertEqual(sim.run(3)['state'], 'ABORT')
                sim.mode, sim.armed = 'GUIDED', True  # Switching back must not restart motion.
                self.assertEqual(sim.run(20)['state'], 'ABORT')
                self.assertEqual(set(sim.commands[start:]), {(0.0, 0.0)})

    def test_total_time_limit_aborts(self):
        sim = ms.Simulation(total_timeout=30.0)
        sim.mission.start()
        out = sim.run(60, until=finished)
        self.assertEqual(out['state'], 'ABORT')
        self.assertLess(sim.time, 31.0)

    def test_occupied_safety_box_stops_forward_motion_only(self):
        sim = ms.Simulation()
        sim.mission.start()
        sim.run(900, until=lambda s, o: o['state'] == 'APPROACH_ENTRY' and o['v'] > 0.1)
        sim.blocked = True
        start = len(sim.commands)
        sim.run(4)
        self.assertEqual({v for v, _ in sim.commands[start:]}, {0.0})
        sim.blocked = False
        self.assertEqual(sim.run(900, until=finished)['state'], 'COMPLETE')

    def test_threading_waits_for_rtk_fixed(self):
        sim = ms.Simulation()
        sim.mission.start()
        sim.rtk_ok = False
        out = sim.run(300, until=lambda s, o: 'RTK' in o['detail'])
        self.assertEqual(out['state'], 'ALIGN')
        sim.run(20)
        self.assertNotIn('THREAD', sim.states)
        self.assertEqual(sim.mission.passed, [])
        sim.rtk_ok = True
        self.assertEqual(sim.run(900, until=finished)['state'], 'COMPLETE')

    def test_unknown_options_are_rejected(self):
        with self.assertRaises(ValueError):
            Mission(crusie_speed=0.3)


class Task4Tests(unittest.TestCase):
    """Task 4 from the state machine: a keep-out zone pauses Task 1, "resume" continues it."""

    def test_keep_out_zone_around_the_vessel_is_left_then_task1_completes(self):
        sim = ms.Simulation()
        sim.mission.start()
        sim.run(900, until=lambda s, out: 'circle_entry_done' in [e['type'] for e in s.events])
        here = tuple(sim.pose[:2])
        zone = dict(x=here[0], y=here[1], radius=2.0)
        sim.mission.set_incident(zone)
        out = sim.run(120, until=lambda s, o: o['state'] == 'TASK4_PAUSE' and s.speed < 0.01 and
                      math.dist(s.pose[:2], here) > 2.0+1.5)
        self.assertEqual(out['state'], 'TASK4_PAUSE')
        self.assertGreater(math.dist(sim.pose[:2], here), zone['radius'], 'still inside the keep-out zone')
        waited = sim.time
        out = sim.run(30)  # Outside: waits, zero commands.
        self.assertEqual(out['state'], 'TASK4_PAUSE')
        self.assertTrue(all(v == 0.0 and w == 0.0 for v, w in sim.commands[-200:]))
        self.assertGreater(sim.time-waited, 25.0)
        sim.mission.set_incident(None)
        out = sim.run(1200, until=finished)
        self.assertEqual(out['state'], 'COMPLETE', sim.events[-5:])
        types = [e['type'] for e in sim.events]
        self.assertLess(types.index('task4_begin'), types.index('task4_resume'))
        verdict = sim.judge()
        self.assertTrue(all(e['legal'] for e in verdict['passages']), verdict['passages'])
        self.assertGreater(verdict['clearance'], 0.1)

    def test_zone_elsewhere_stops_the_vessel_and_pause_time_is_not_task1_time(self):
        sim = ms.Simulation(total_timeout=120.0)
        sim.mission.start()
        sim.run(20)
        sim.mission.set_incident(dict(x=sim.pose[0]+50.0, y=sim.pose[1], radius=5.0))
        sim.run(5)  # The hull slows down.
        out = sim.run(100)  # Together with the next 100 s longer than the Task 1 time limit.
        sim.inputs_ok = False  # An input stalls during Task 4: HOLD, then back to the pause.
        self.assertEqual(sim.run(5)['state'], 'HOLD')
        sim.inputs_ok = True
        out = sim.run(100)
        self.assertEqual(out['state'], 'TASK4_PAUSE')
        self.assertTrue(all(v == 0.0 and w == 0.0 for v, w in sim.commands[-1000:]))
        sim.mission.set_incident(None)
        out = sim.run(1.0)
        self.assertNotIn(out['state'], ('TASK4_PAUSE', 'ABORT', 'COMPLETE'))  # Resumed, not timed out.
        self.assertLess(sim.mission.elapsed(sim.time), 40.0)

    def test_unplaced_zone_centre_only_stops(self):
        sim = ms.Simulation()
        sim.mission.start()
        sim.run(20)
        sim.mission.set_incident(dict(x=None, y=None, radius=10.0))  # No GNSS fix for the centre.
        sim.run(5)
        out = sim.run(10)
        self.assertEqual(out['state'], 'TASK4_PAUSE')
        self.assertTrue(all(v == 0.0 and w == 0.0 for v, w in sim.commands[-100:]))


if __name__ == '__main__':
    unittest.main()
