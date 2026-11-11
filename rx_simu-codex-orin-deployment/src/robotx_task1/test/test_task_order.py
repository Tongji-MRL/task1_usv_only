"""Task order from the 2026 handbook (Task 1 Core), checked in closed loop; no ROS.

- The ENTRY buoy is circled clockwise *before entering the field*; passages before
  that do not score.
- After the entry circle, the vessel passes GREEN buoys only to port and RED buoys
  only to starboard (25 points each, so at least one of each is needed), and the
  reported buoy states earn 10 points each, so every red and green is classified.
- Then the EXIT buoy is circled counter-clockwise.
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial import Delaunay

sys.path.insert(0, str(Path(__file__).parent))
import mission_sim as ms  # noqa: E402
import test_mission as tm  # noqa: E402
from robotx_task1 import gap_planner as gp  # noqa: E402
from robotx_task1.core import passage_events  # noqa: E402
from robotx_task1.mission_core import Mission  # noqa: E402


def buoy(name, x, y, state):
    return dict(id=name, x=x, y=y, state=state, radius=0.2)


def inside_hull(points, buoys, margin=0.0):
    """For each point: inside the convex hull of the buoy centres (shrunk by margin)?"""
    centres = np.array([[b['x'], b['y']] for b in buoys])
    centre = centres.mean(axis=0)
    shrunk = centre+(centres-centre)*np.maximum(0.0, 1.0-margin/np.linalg.norm(centres-centre, axis=1))[:, None]
    return Delaunay(shrunk).find_simplex(np.asarray(points)[:, :2]) >= 0


class OutsidePlanningTests(unittest.TestCase):
    FIELD = [buoy('A', 0.0, 0.0, gp.OFF), buoy('B', 3.0, 0.0, gp.OFF), buoy('C', 6.0, 0.0, gp.OFF),
             buoy('D', 0.0, 3.0, gp.OFF), buoy('E', 3.0, 3.0, gp.OFF), buoy('F', 6.0, 3.0, gp.OFF)]

    def test_outside_mode_goes_around_the_field(self):
        start, goal = (3.0, -3.0), (3.0, 6.0)
        direct = gp.plan(self.FIELD, start, goal)
        through = [(start[0], start[1])]+[(w['x'], w['y']) for w in direct['waypoints']]
        self.assertTrue(any(inside_hull(np.array(through), self.FIELD, margin=0.3)))  # Straight through.
        around = gp.plan(self.FIELD, start, goal, outside=True)
        path = np.array([start]+[(w['x'], w['y']) for w in around['waypoints']])
        dense = np.vstack([a+(b-a)*t for a, b in zip(path, path[1:]) for t in np.linspace(0, 1, 30)])
        self.assertFalse(any(inside_hull(dense, self.FIELD, margin=0.3)))
        self.assertFalse(around['entered_field'])

    def test_outside_mode_still_reaches_a_goal_inside_the_field(self):
        route = gp.plan(self.FIELD, (3.0, -3.0), (4.5, 1.5), outside=True)
        self.assertIsNotNone(route)
        self.assertTrue(route['entered_field'])  # Reported, so the mission can log it.


class GateChoiceTests(unittest.TestCase):
    def test_scoring_gate_puts_green_to_port_and_red_to_starboard(self):
        mission = Mission()
        mission.buoys = [buoy('R', 0.0, 0.0, gp.RED), buoy('G', 0.0, 2.6, gp.GREEN), buoy('O', 6.0, 1.3, gp.OFF)]
        mission.obstacles, mission.walls, mission.solid = [], [], []
        gate = mission.scoring_gate((-4.0, 1.3, 0.0))
        self.assertEqual((gate['port'], gate['starboard'], gate['gives']), ('G', 'R', 2))
        a, mid, b = np.array(gate['align']), np.array(gate['mid']), np.array(gate['through'])
        yaw = math.atan2(*(b-a)[::-1])
        events = passage_events([a[0], a[1], yaw], [b[0], b[1], yaw], mission.buoys)
        self.assertEqual({e['id']: (e['side'], e['legal']) for e in events},
                         dict(G=('port', True), R=('starboard', True)))
        mission.credit = dict(green=True, red=True)
        self.assertIsNone(mission.scoring_gate((-4.0, 1.3, 0.0)))  # Nothing left to earn.

    def test_two_reds_never_form_a_gate(self):
        mission = Mission()
        mission.buoys = [buoy('R1', 0.0, 0.0, gp.RED), buoy('R2', 0.0, 2.6, gp.RED)]
        mission.obstacles, mission.walls, mission.solid = [], [], []
        gate = mission.scoring_gate((-4.0, 1.3, 0.0))
        self.assertTrue(gate is None or gate['port'] not in ('R1', 'R2'))


class ClosedLoopOrderTests(unittest.TestCase):
    def run_field(self, **kwargs):
        sim = ms.Simulation(**kwargs)
        sim.mission.start()
        out = sim.run(1200, until=tm.finished)
        return sim, out

    def test_example_field_follows_the_handbook_order(self):
        sim, out = self.run_field()
        self.assertEqual(out['state'], 'COMPLETE', sim.events[-5:])
        times = {e['type']: e['time'] for e in reversed(sim.events)}
        circle_done = times['circle_entry_done']
        steps = int(round(circle_done/ms.DT))
        # 1. Not inside the buoy field before the entry circle.
        entry = next(b for b in sim.truth if b['state'] == gp.ENTRY)
        others = [b for b in sim.truth if b['id'] != entry['id']]
        early = np.array([p for p in sim.track[:steps] if math.dist(p[:2], (entry['x'], entry['y'])) > 2.0])
        self.assertFalse(any(inside_hull(early, others, margin=0.5)) if len(early) else False)
        # 2. After it: at least one green to port and one red to starboard, never a wrong side.
        late = [e for i, (a, b) in enumerate(zip(sim.track[steps:], sim.track[steps+1:]))
                if not sim.phase[steps+i].startswith('CIRCLE') and math.dist(a[:2], b[:2]) >= ms.PIVOT_STEP
                for e in passage_events(list(a), list(b), sim.truth) if not e['ambiguous']]
        self.assertTrue(all(e['legal'] for e in late), late)
        self.assertIn(('port', gp.GREEN), {(e['side'], e['state']) for e in late})
        self.assertIn(('starboard', gp.RED), {(e['side'], e['state']) for e in late})
        self.assertEqual(sim.mission.credit, dict(green=True, red=True))
        # 3. Every red and green classified before the exit circle starts.
        exit_start = sim.phase.index('CIRCLE_EXIT')
        coloured = {b['id'] for b in sim.truth if b['state'] in (gp.RED, gp.GREEN)}
        self.assertLessEqual(coloured, sim.classified[exit_start])
        # 4. Exit circle last.
        self.assertEqual([e['type'] for e in sim.events][-2:], ['circle_exit_done', 'complete'])


if __name__ == '__main__':
    unittest.main()
