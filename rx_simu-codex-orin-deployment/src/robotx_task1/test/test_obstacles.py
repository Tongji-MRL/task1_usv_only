"""Non-buoy obstacles (Task 3 platform, dock): grid, planner and closed loop; no ROS."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import mission_sim as ms  # noqa: E402
import test_mission as tm  # noqa: E402
from robotx_task1 import gap_planner as gp  # noqa: E402
from robotx_task1.core import passage_events  # noqa: E402
from robotx_task1.landmark_map import ObstacleGrid, inside_polygon, shrink_polygon  # noqa: E402
from robotx_task1.survey_core import StaticTree, candidates, cluster, polygon_in_frame  # noqa: E402


def square(center, size, spacing=0.5):
    """Obstacle cell centres covering a square, as ObstacleGrid.points() returns them."""
    steps = np.arange(-size/2+spacing/2, size/2, spacing)
    return [(center[0]+x, center[1]+y) for x in steps for y in steps]


def buoy(name, x, y, state):
    return dict(id=name, x=x, y=y, state=state, radius=0.2)


def platform_gap(x, y, center, size):
    return math.hypot(max(abs(x-center[0])-size/2, 0.0), max(abs(y-center[1])-size/2, 0.0))


class GridTests(unittest.TestCase):
    def test_large_cluster_points_are_separated_from_buoys(self):
        outline = ms.platform_outline((5.0, 0.0), 2.5)
        platform = np.column_stack([outline, np.full(len(outline), 0.4)])
        rng = np.random.default_rng(0)
        buoy_points = np.column_stack([np.array([1.0, 0.0])+0.1*rng.normal(size=(12, 2)), np.full(12, 0.5)])
        found, groups = cluster(np.vstack([platform, buoy_points]), members=True)
        self.assertEqual(len(found), 2)
        self.assertEqual(sum(len(g) for g in groups), len(platform)+len(buoy_points))
        self.assertEqual(len(candidates(found)), 1)  # Only the buoy is a landmark candidate.
        big = [g for c, g in zip(found, groups) if c.extent > 1.0]
        self.assertEqual([len(g) for g in big], [len(platform)])
        self.assertEqual(cluster(np.empty((0, 3)), members=True), ([], []))

    def test_cells_need_repeated_hits_respect_the_boundary_and_persist(self):
        grid = ObstacleGrid(boundary=[(-10, -10), (10, -10), (10, 10), (-10, 10)])
        wall = ms.platform_outline((0.0, 0.0), 2.0)
        grid.update(np.vstack([wall, [[15.0, 0.0], [15.2, 0.0]]]))
        grid.update(np.vstack([wall, [[3.0, 3.0]]]))  # A single transient return.
        self.assertEqual(grid.points(), [])
        grid.update(wall)
        cells = grid.points()
        self.assertTrue(cells)
        self.assertTrue(all(abs(x) <= 1.5 and abs(y) <= 1.5 for x, y in cells))  # No clutter, nothing outside.
        revision = grid.revision
        for _ in range(20):
            grid.update(np.empty((0, 2)))  # Out of view: kept.
        self.assertEqual((grid.points(), grid.revision), (cells, revision))


class PlannerTests(unittest.TestCase):
    def test_route_goes_around_a_platform_on_the_direct_line(self):
        buoys = [buoy('O1', 0.0, 4.0, gp.OFF), buoy('O2', 10.0, 4.0, gp.OFF)]
        self.assertEqual(len(gp.plan(buoys, (0.0, 0.0), (10.0, 0.0))['waypoints']), 1)  # Straight without it.
        route = gp.plan(buoys, (0.0, 0.0), (10.0, 0.0), obstacles=square((5.0, 0.0), 2.5))
        self.assertIsNotNone(route)
        path = [(0.0, 0.0)]+[(w['x'], w['y']) for w in route['waypoints']]
        for a, b in zip(path, path[1:]):
            for t in np.linspace(0, 1, 50):
                x, y = a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])
                self.assertGreater(platform_gap(x, y, (5.0, 0.0), 2.5), 0.5)  # Half beam plus margin.
        self.assertTrue(all(g['port'] in (None, 'O1', 'O2') and g['starboard'] in (None, 'O1', 'O2')
                            for g in route['gaps']))  # Obstacle cells are never reported as gap ends.

    def test_cells_of_one_object_cannot_be_threaded(self):
        wall = [(5.0, y) for y in np.arange(-6.0, 6.01, 0.5)]  # A 12 m dock across the way.
        route = gp.plan([], (0.0, 0.0), (10.0, 0.0), obstacles=wall)
        self.assertIsNotNone(route)
        self.assertTrue(any(abs(w['y']) > 6.0 for w in route['waypoints']))  # Around the end.

    def test_obstacles_never_shield_a_wrong_side_buoy(self):
        # Green buoy to starboard of the line; an obstacle nearer on that side must not make
        # the passage legal, since the judges count buoys only.
        green = [buoy('G', 5.0, -1.6, gp.GREEN)]
        check = gp.check_path([(0.0, 0.0), (10.0, 0.0)], green, obstacles=[(5.0, -0.9)], obstacle_radius=0.0)
        self.assertEqual(check['illegal'], 0)
        self.assertFalse(passage_events([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], green)[0]['legal'])

    def test_path_check_flags_obstacle_clearance(self):
        self.assertEqual(gp.check_path([(0.0, 0.0), (10.0, 0.0)], [], obstacles=[(5.0, 0.6)])['tight'], 0)
        self.assertIsNone(gp.check_path([(0.0, 0.0), (10.0, 0.0)], [], obstacles=[(5.0, 1.0)])['tight'])


class BoundaryTests(unittest.TestCase):
    SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]

    def test_shrink_and_inside_in_either_vertex_order(self):
        for polygon in (self.SQUARE, self.SQUARE[::-1]):
            inner = shrink_polygon(polygon, 1.0)
            self.assertEqual(sorted((round(x, 6), round(y, 6)) for x, y in inner),
                             [(1.0, 1.0), (1.0, 9.0), (9.0, 1.0), (9.0, 9.0)])
        self.assertTrue(inside_polygon((5.0, 5.0), self.SQUARE))
        self.assertFalse(inside_polygon((0.5, 5.0), shrink_polygon(self.SQUARE, 1.0)))
        self.assertTrue(inside_polygon((99.0, 99.0), None))

    def test_map_boundary_is_moved_into_the_odometry_frame(self):
        tree = StaticTree()
        self.assertIsNone(polygon_in_frame(tree, self.SQUARE, 'odom', 'map'))  # Not georeferenced yet.
        tree.add('map', 'odom', [10.0, -5.0, 0.0], [0.0, 0.0, 0.0, 1.0])  # map_origin: translation only.
        moved = polygon_in_frame(tree, self.SQUARE, 'odom', 'map')
        self.assertEqual([tuple(round(v, 6) for v in q) for q in moved],
                         [(-10.0, 5.0), (0.0, 5.0), (0.0, 15.0), (-10.0, 15.0)])

    def test_route_stays_inside_the_boundary_wall(self):
        wall = gp.wall_points(self.SQUARE)
        self.assertEqual(len(wall), 80)
        route = gp.plan([buoy('O', 5.0, 5.0, gp.OFF)], (5.0, 1.5), (5.0, 8.5), obstacles=wall)
        self.assertIsNotNone(route)
        for w in route['waypoints']:
            self.assertTrue(inside_polygon((w['x'], w['y']), shrink_polygon(self.SQUARE, 0.79)), w)
        # A goal outside the course cannot be reached.
        self.assertIsNone(gp.plan([], (5.0, 5.0), (12.0, 5.0), obstacles=wall))


class ClosedLoopTests(unittest.TestCase):
    def test_example_field_with_a_platform_on_the_unobstructed_route(self):
        sim = ms.Simulation()
        sim.mission.start()
        sim.run(900, until=tm.finished)
        # Put a 2 m platform where the unobstructed run went, away from every buoy.
        start = sim.track[0][:2]
        free = [p for p in sim.track if min(math.dist(p[:2], (b['x'], b['y'])) for b in sim.truth) > 2.6
                and math.dist(p[:2], start) > 3.0]
        center = free[len(free)//2][:2]
        sim = ms.Simulation(platforms=[(center, 2.0)])
        sim.mission.start()
        out = sim.run(1200, until=tm.finished)
        verdict = sim.judge()
        self.assertEqual(out['state'], 'COMPLETE', (center, out))
        self.assertTrue(sim.grid.points())
        self.assertGreater(verdict['platform_clearance'], 0.1, center)
        self.assertTrue(all(e['legal'] for e in verdict['passages']), verdict['passages'])
        self.assertEqual(verdict['blind'], [])
        self.assertGreater(verdict['clearance'], 0.1)


if __name__ == '__main__':
    unittest.main()
