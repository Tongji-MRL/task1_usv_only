"""Gap planner on the handbook's example mix: 2 red, 2 green, 4 off, 2 blue."""
import math
import unittest

import numpy as np

from robotx_task1 import gap_planner as gp
from robotx_task1.core import passage_events

START, GOAL = (1.0, 1.0), (11.0, 7.2)


def field(**changes):
    """Irregular field, entry and exit on opposite corners; +x is the transit direction."""
    layout = dict(E=(0.0, 0.0, gp.ENTRY), X=(12.0, 8.0, gp.EXIT),
                  R1=(4.0, 3.0, gp.RED), G1=(4.0, 4.9, gp.GREEN),
                  R2=(8.0, 4.5, gp.RED), G2=(8.0, 6.5, gp.GREEN),
                  O1=(2.0, 6.5, gp.OFF), O2=(6.0, 2.0, gp.OFF),
                  O3=(6.0, 7.5, gp.OFF), O4=(10.0, 3.0, gp.OFF))
    layout.update(changes)
    return [dict(id=name, x=x, y=y, state=state, radius=0.2) for name, (x, y, state) in layout.items()]


def judge(route, buoys, start=START):
    """Independent dense replay: every abeam event and the closest approach."""
    points = [np.array(start)]+[np.array([w['x'], w['y']]) for w in route['waypoints']]
    events, closest = [], float('inf')
    for a, b in zip(points, points[1:]):
        yaw = math.atan2(b[1]-a[1], b[0]-a[0])
        steps = max(1, int(np.linalg.norm(b-a)/0.05))
        for i in range(steps):
            p, q = a+(b-a)*i/steps, a+(b-a)*(i+1)/steps
            events += passage_events([p[0], p[1], yaw], [q[0], q[1], yaw], buoys)
            closest = min([closest]+[math.dist(q, (c['x'], c['y']))-c['radius'] for c in buoys])
    return events, closest


class GapPlannerTests(unittest.TestCase):
    def test_example_field_is_threaded_with_red_to_starboard_and_green_to_port(self):
        buoys = field()
        route = gp.plan(buoys, START, GOAL)
        self.assertIsNotNone(route)
        self.assertEqual(route['look'], [])
        pairs = {(g['port'], g['starboard']) for g in route['gaps']}
        self.assertIn(('G1', 'R1'), pairs)
        self.assertIn(('G2', 'R2'), pairs)
        events, closest = judge(route, buoys)
        self.assertTrue(events and all(e['legal'] for e in events), events)
        self.assertEqual({e['id']: e['side'] for e in events},
                         dict(R1='starboard', G1='port', R2='starboard', G2='port'))
        self.assertGreaterEqual(closest, 0.55)
        self.assertEqual(route['waypoints'][-1], dict(x=11.0, y=7.2, kind='goal', gap=None))
        self.assertLess(route['length'], 1.6*math.dist(START, GOAL))

    def test_narrow_gaps_get_alignment_and_pass_through_points_on_the_gap_normal(self):
        route = gp.plan(field(), START, GOAL)
        index = next(i for i, g in enumerate(route['gaps']) if (g['port'], g['starboard']) == ('G1', 'R1'))
        gap = route['gaps'][index]
        self.assertTrue(gap['narrow'])
        self.assertAlmostEqual(gap['width'], 1.5)
        align, mid, through = [w for w in route['waypoints'] if w['gap'] == index]
        self.assertEqual([align['kind'], mid['kind'], through['kind']], ['align', 'mid', 'through'])
        self.assertTrue(np.allclose([mid['x'], mid['y']], [4.0, 3.95]))
        self.assertTrue(np.allclose(gap['normal'], [1.0, 0.0]))
        self.assertAlmostEqual(align['y'], 3.95)
        self.assertLess(align['x'], 4.0-1.0)
        self.assertGreater(through['x'], 4.0+0.5)

    def test_gap_with_the_colors_on_the_wrong_sides_is_never_crossed(self):
        buoys = field(R1=(4.0, 4.9, gp.RED), G1=(4.0, 3.0, gp.GREEN))
        route = gp.plan(buoys, START, GOAL)
        if route is not None:
            self.assertNotIn(('R1', 'G1'), {(g['port'], g['starboard']) for g in route['gaps']})
            self.assertTrue(all(e['legal'] for e in judge(route, buoys)[0]))

    def test_gap_narrower_than_the_vessel_margin_is_not_used(self):
        buoys = field(G1=(4.0, 4.0, gp.GREEN))  # 0.6 m of clear water.
        route = gp.plan(buoys, START, GOAL)
        if route is not None:
            self.assertNotIn(('G1', 'R1'), {(g['port'], g['starboard']) for g in route['gaps']})
            events, closest = judge(route, buoys)
            self.assertTrue(all(e['legal'] for e in events))
            self.assertGreaterEqual(closest, 0.55)

    def test_unclassified_buoy_must_be_looked_at_before_it_comes_abeam(self):
        buoys = field(G1=(4.0, 4.9, gp.UNKNOWN))
        route = gp.plan(buoys, START, GOAL)
        self.assertIsNotNone(route)
        self.assertEqual(route['look'], ['G1'])
        # Everything before the reported leg is free of unclassified buoys abeam.
        before = dict(route, waypoints=route['waypoints'][:route['look_leg']])
        for assumed in (gp.RED, gp.GREEN):
            judged = [dict(b, state=assumed if b['id'] == 'G1' else b['state']) for b in buoys]
            self.assertNotIn('G1', {e['id'] for e in judge(before, judged)[0]})
        # Once it turns out to be red, the optimistic gap is abandoned.
        buoys = field(G1=(4.0, 4.9, gp.RED))
        route = gp.plan(buoys, START, GOAL)
        if route is not None:
            self.assertEqual(route['look'], [])
            self.assertTrue(all(e['legal'] for e in judge(route, buoys)[0]))
            self.assertNotIn(('G1', 'R1'), {(g['port'], g['starboard']) for g in route['gaps']})

    def test_partial_map_and_trivial_cases(self):
        self.assertEqual(gp.plan([], START, GOAL)['gaps'], [])
        near = gp.plan(field(), (1.0, 1.0), (1.5, 1.2))
        self.assertEqual((near['gaps'], len(near['waypoints'])), ([], 1))
        known = [b for b in field() if b['id'] in ('E', 'R1', 'G1')]
        route = gp.plan(known, START, (6.0, 4.0))
        self.assertIn(('G1', 'R1'), {(g['port'], g['starboard']) for g in route['gaps']})

    def test_path_check_reports_wrong_side_and_tight_segments(self):
        buoys = field()
        wrong = gp.check_path([(1.0, 2.0), (7.0, 2.6)], buoys)  # R1 would be to port.
        self.assertEqual(wrong['illegal'], 0)
        tight = gp.check_path([(3.0, 3.6), (5.0, 3.6)], buoys)   # 0.4 m from R1's surface.
        self.assertEqual((tight['illegal'], tight['tight']), (None, 0))

    def test_exploration_goal_is_beyond_the_field_seen_from_the_entry(self):
        buoys = [b for b in field() if b['id'] != 'X']
        goal = gp.exploration_goal(buoys, buoys[0])
        self.assertGreater(goal[0], 10.0)
        self.assertGreater(goal[1], 3.0)
        self.assertIsNone(gp.exploration_goal(buoys[:1], buoys[0]))


if __name__ == '__main__':
    unittest.main()
