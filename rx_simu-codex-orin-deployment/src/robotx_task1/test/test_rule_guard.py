"""Side-rule guard details found in the field replays (Delta course, 43 px/m); no ROS.

- Right after a turn on the spot the hull is still turning while the speed only builds
  up: a green buoy just behind the starboard beam swings abeam although the commanded
  (straight-ish) motion would leave it behind. Task1Mission.motion_risk must stop that.
- Two buoys on the same side at almost the same distance: the judge's "closest buoy
  when abeam" is decided by centimetres. The planner keeps a margin (`tie`), the
  guard uses the judge's own 1 cm, so a planned leg is not stopped by the guard.
- Next to the platform every gap crossing out of a pocket may be illegal, while a short
  leg at an angle is legal: Task1Mission.detour finds it (photo field, west side).
- Known non-task buoys (start gate) never become landmarks or obstacle cells.
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from robotx_task1 import gap_planner as gp  # noqa: E402
from robotx_task1 import landmark_map as lm  # noqa: E402
from robotx_task1.core import passage_events  # noqa: E402
from robotx_task1.mission_core import Mission  # noqa: E402
from robotx_task1.survey_core import Cluster  # noqa: E402


def buoy(name, x, y, state):
    return dict(id=name, x=x, y=y, state=state, radius=0.2)


class MotionRiskTests(unittest.TestCase):
    def mission(self):
        mission = Mission()
        mission.buoys = [buoy('G6', 5.63, 16.85, gp.GREEN), buoy('R2', 8.23, 15.19, gp.RED),
                         buoy('G1', 11.95, 16.51, gp.GREEN)]
        mission.entry_done = True
        return mission

    def test_turn_after_pivot_sweeping_a_green_buoy_is_stopped(self):
        mission = self.mission()
        pose = (8.74, 18.91, -0.96)  # G6 1.5 deg behind the starboard beam, 3.7 m away.
        self.assertTrue(mission.motion_risk(pose, 0.29, -0.3, -0.3))
        # Turning on the spot stays allowed: the judge ignores a stationary pivot.
        self.assertEqual(passage_events(list(pose), [pose[0], pose[1], -1.3], mission.buoys), [])

    def test_aligned_leg_shielded_by_the_red_buoy_may_be_driven(self):
        # Same place after the pivot: G6 comes abeam with R2 0.15 m nearer on that side.
        self.assertFalse(self.mission().motion_risk((8.72, 18.51, -1.1), 0.3, 0.0, 0.0))


class TieTests(unittest.TestCase):
    leg = [(-3.0, 0.0), (3.0, 0.0)]

    def test_planner_keeps_a_margin_over_the_judges_tie(self):
        # G lies 2.0 m to starboard (wrong side); R, 3.5 cm nearer on that side, shields it.
        near_tie = [buoy('G', 0.0, -2.0, gp.GREEN), buoy('R', 0.5, -1.9, gp.RED)]
        self.assertTrue(all(e['legal'] for e in passage_events([-3, 0, 0], [3, 0, 0], near_tie)))
        self.assertIsNone(gp.check_path(self.leg, near_tie, heading_slack=0.0, tie=0.01)['illegal'])
        self.assertEqual(gp.check_path(self.leg, near_tie, heading_slack=0.0)['illegal'], 0)
        # A clear shield (R 0.37 m nearer) is fine with the margin too.
        shielded = [buoy('G', 0.0, -2.0, gp.GREEN), buoy('R', 0.3, -1.6, gp.RED)]
        self.assertIsNone(gp.check_path(self.leg, shielded)['illegal'])


class PocketTests(unittest.TestCase):
    """Photo field, west side: a pocket between green 2, red 4, unlit 5 and the platform."""

    def mission(self):
        mission = Mission()
        mission.buoys = [buoy('B2', -1.26, 10.08, gp.GREEN), buoy('B4', -1.57, 13.40, gp.RED),
                         buoy('B5', 0.98, 11.25, gp.OFF), buoy('B7', -2.22, 15.35, gp.RED),
                         buoy('B8', 1.15, 16.53, gp.EXIT), buoy('B3', 3.63, 10.36, gp.OFF),
                         buoy('B6', 4.41, 12.14, gp.OFF), buoy('B1', 1.50, 7.52, gp.OFF)]
        platform = [(x+0.25, y+0.25) for x in (-4.5, -4.0, -3.5, -3.0) for y in (10.5, 11.0, 11.5, 12.0, 12.5)]
        mission.obstacles, mission.walls = platform, []
        mission.solid = list(platform)
        mission.entry_done = True
        return mission

    def test_detour_leaves_the_pocket_where_gap_routes_cannot(self):
        mission = self.mission()
        pose, goal = (-1.0, 12.45, 0.5), (0.54, 15.38)
        self.assertIsNone(mission.plan_route(pose[:2], goal))
        route = mission.detour(pose, goal)
        self.assertIsNotNone(route)
        first = route['waypoints'][0]
        self.assertEqual(first['kind'], 'escape')
        path = [pose[:2]]+[(w['x'], w['y']) for w in route['waypoints']]
        verdict = mission.check(path)
        self.assertIsNone(verdict['illegal'])
        self.assertIsNone(verdict['tight'])


class ContinueTests(unittest.TestCase):
    def test_rest_of_route_is_kept_only_while_map_and_goal_are_unchanged(self):
        mission = Mission()
        mission.buoys = [buoy('R', 0.0, 0.0, gp.RED), buoy('G', 0.0, 2.6, gp.GREEN)]
        mission.obstacles, mission.walls, mission.solid = [], [], []
        mission.entry_done = True
        mission.remember_map()
        route = dict(waypoints=[dict(x=-1.0, y=1.3, kind='mid', gap=None), dict(x=3.0, y=1.3, kind='goal', gap=None)],
                     gaps=[], look=[], look_leg=None)
        mission.previous = dict(route=route, next=1, goal=(3.0, 1.3), signature=dict(mission.signature),
                                obstacles=mission.obstacle_count)
        mission.reached = (-1.0, 1.3)
        kept = mission.continued((-1.1, 1.32, 0.0), (3.0, 1.3))
        self.assertEqual([(w['x'], w['y']) for w in kept['waypoints']], [(3.0, 1.3)])
        self.assertIsNone(mission.continued((-1.1, 1.32, 0.0), (3.0, 5.0)))  # Another goal.
        mission.buoys.append(buoy('N', 6.0, 6.0, gp.UNKNOWN))
        self.assertIsNone(mission.continued((-1.1, 1.32, 0.0), (3.0, 1.3)))  # The map changed.


class MarkerTests(unittest.TestCase):
    def test_start_gate_buoys_are_not_landmarks_or_obstacle_cells(self):
        course = [(0.0, 0.0), (25.0, 0.0), (25.0, 30.0), (0.0, 30.0)]
        markers = [(2.2, 1.2), (11.2, 1.7)]
        world = lm.LandmarkMap(boundary=course)
        world.markers = markers
        found = [Cluster(list(p), 12, 0.4, 0.0, 0.9) for p in markers+[(6.0, 19.0)]]
        for i in range(6):
            world.update_lidar(0.5*i, (6.0, 2.0), found)
        self.assertEqual([(round(b['x'], 1), round(b['y'], 1)) for b in world.snapshot()], [(6.0, 19.0)])
        grid = lm.ObstacleGrid(boundary=course)
        grid.markers = markers
        for _ in range(4):
            grid.update([[2.2, 1.3], [11.1, 1.7], [20.0, 25.0]])
        self.assertEqual(grid.points(), [(20.25, 25.25)])
        self.assertFalse(lm.inside_course((2.5, 1.5), course, markers=markers))
        self.assertTrue(lm.inside_course((2.5, 1.5), course))


if __name__ == '__main__':
    unittest.main()
