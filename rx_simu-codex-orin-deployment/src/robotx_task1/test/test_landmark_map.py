"""Lamp-state decision and online landmark map; no ROS, no hardware."""
import math
import unittest

import numpy as np

from robotx_task1 import landmark_map as lm
from robotx_task1.survey_core import Cluster


def feed(lamp, label_at, start=0.0, duration=6.0, rate=10.0):
    """Feed labels and return [(time, state)] at every change."""
    changes = []
    for i in range(int(duration*rate)+1):
        t = start+i/rate
        if lamp.add(t, label_at(t)):
            changes.append((round(t, 2), lamp.state))
    return changes


def flashing(color, phase=0.0):
    return lambda t: color if int(math.floor(t+phase)) % 2 == 0 else 'none'


def cluster(x, y, extent=0.4, top=0.9):
    return Cluster(np.array([x, y]), 12, extent, 0.0, top)


class LampTests(unittest.TestCase):
    def test_flashing_red_and_green_after_two_seconds(self):
        for color, state in (('red', lm.RED), ('green', lm.GREEN)):
            for phase in (0.0, 0.5, 1.0):
                with self.subTest(color=color, phase=phase):
                    changes = feed(lm.LampClassifier(), flashing(color, phase))
                    self.assertEqual(changes, [(2.0, state)])

    def test_blue_needs_two_periods_to_separate_entry_from_exit(self):
        for phase in (0.0, 0.3, 1.0):
            self.assertEqual(feed(lm.LampClassifier(), flashing('blue', phase)), [(4.0, lm.FLASH_BLUE)])
        self.assertEqual(feed(lm.LampClassifier(), lambda t: 'blue'), [(4.0, lm.STEADY_BLUE)])
        # A steady lamp with scattered missed frames is not mistaken for the entry.
        flicker = feed(lm.LampClassifier(), lambda t: 'none' if round(t*10) % 4 == 0 else 'blue')
        self.assertNotIn(lm.FLASH_BLUE, [state for _, state in flicker])

    def test_off_needs_three_quiet_seconds_and_unknown_otherwise(self):
        self.assertEqual(feed(lm.LampClassifier(), lambda t: 'none'), [(3.0, lm.OFF)])
        lamp = lm.LampClassifier()
        self.assertEqual(feed(lamp, lambda t: 'none', duration=2.5), [])
        self.assertEqual(lamp.state, lm.UNKNOWN)
        # Red and green in the same series: nothing is decided.
        mixed = lm.LampClassifier()
        feed(mixed, lambda t: ('red', 'green', 'none', 'none')[int(t*10) % 4])
        self.assertEqual(mixed.state, lm.UNKNOWN)

    def test_interrupted_observation_does_not_accumulate(self):
        lamp = lm.LampClassifier()
        for start in (0.0, 5.0, 10.0):
            feed(lamp, flashing('red'), start=start, duration=1.5)
        self.assertEqual(lamp.state, lm.UNKNOWN)
        feed(lamp, flashing('red'), start=20.0, duration=2.0)
        self.assertEqual(lamp.state, lm.RED)

    def test_state_is_latched_and_only_persistent_evidence_corrects_it(self):
        lamp = lm.LampClassifier()
        feed(lamp, flashing('red'), duration=3.0)
        feed(lamp, lambda t: 'none', start=3.1, duration=10.0)  # Now hidden or looked away.
        self.assertEqual(lamp.state, lm.RED)
        self.assertEqual(feed(lamp, flashing('green'), start=20.0, duration=4.0), [])
        self.assertEqual(lamp.state, lm.RED)
        self.assertEqual(feed(lamp, flashing('green'), start=30.0, duration=8.0)[-1][1], lm.GREEN)
        # OFF is corrected immediately when the lamp is seen lit.
        dark = lm.LampClassifier()
        feed(dark, lambda t: 'none', duration=3.5)
        self.assertEqual(dark.state, lm.OFF)
        self.assertEqual(feed(dark, flashing('green'), start=3.6, duration=4.0)[0][1], lm.GREEN)


class MapTests(unittest.TestCase):
    def test_confirmation_ids_and_transient_clutter(self):
        world = lm.LandmarkMap()
        for i in range(16):
            found = [cluster(6.0, 0.0), cluster(6.0, 1.9)]+([cluster(3.0, 4.0)] if i in (2, 3) else [])
            world.update_lidar(0.5*i, (0.0, 0.0), found)
            if i == 2:
                self.assertEqual(world.snapshot(), [])  # Three hits are not enough.
        snapshot = world.snapshot()
        self.assertEqual([b['id'] for b in snapshot], ['B1', 'B2'])  # 1.5 m gap kept apart.
        self.assertEqual({b['state'] for b in snapshot}, {lm.UNKNOWN})
        self.assertEqual(snapshot[0]['radius'], 0.2)
        self.assertEqual(world.revision, 2)
        self.assertEqual(len(world.tracks), 2)  # The clutter track expired.

    def test_near_observations_dominate_and_confirmed_landmarks_persist(self):
        world = lm.LandmarkMap()
        for i in range(20):
            boat = (0.6*i, 0.0)
            bias = 0.3 if i < 10 else 0.0  # Far observations shifted, as by a heading error.
            world.update_lidar(0.5*i, boat, [cluster(13.0+bias, 0.5+bias)])
        for i in range(20, 60):
            world.update_lidar(0.5*i, (0.0, 0.0), [])  # Out of view for 20 s.
        only = world.snapshot()[0]
        self.assertLess(math.dist((only['x'], only['y']), (13.0, 0.5)), 0.05)

    def test_color_labels_reach_the_right_landmark_and_bump_the_revision(self):
        world = lm.LandmarkMap()
        for i in range(5):
            world.update_lidar(0.5*i, (0.0, 0.0), [cluster(6.0, 0.0), cluster(6.0, 1.9)])
        revision = world.revision
        for i in range(25):
            world.observe_color('B2', i/10, 'red' if i % 20 < 10 else 'none')
        self.assertFalse(world.observe_color('B9', 3.0, 'red'))
        self.assertEqual([b['state'] for b in world.snapshot()], [lm.UNKNOWN, lm.RED])
        self.assertEqual(world.revision, revision+1)

    def test_course_filter_drops_only_the_corner_buoys(self):
        # Delta course: buoy 3 lies 0.8 m inside the boundary line; corner buoys sit on the vertices.
        course = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]
        world = lm.LandmarkMap(boundary=course)
        found = [cluster(0.8, 11.0), cluster(-0.2, 15.0), cluster(-0.6, 5.0), cluster(0.1, 0.1),
                 cluster(19.6, 19.8), cluster(10.0, 10.0)]
        for i in range(6):
            world.update_lidar(0.5*i, (10.0, 5.0), found)
        kept = sorted((round(b['x'], 1), round(b['y'], 1)) for b in world.snapshot())
        self.assertEqual(kept, [(-0.2, 15.0), (0.8, 11.0), (10.0, 10.0)])  # 0.2 m outside: position noise.
        self.assertTrue(lm.inside_course((0.8, 11.0), course))
        self.assertFalse(lm.inside_course((0.5, 0.6), course))  # Within 1 m of a corner buoy.
        self.assertTrue(lm.inside_course((5.0, 5.0), None))

    def test_boundary_and_landmark_cap(self):
        world = lm.LandmarkMap(boundary=[(0, -5), (20, -5), (20, 5), (0, 5)], max_landmarks=3)
        for i in range(6):
            world.update_lidar(0.5*i, (1.0, 0.0), [cluster(5.0+2*k, 0.0) for k in range(5)]+[cluster(5.0, 9.0)])
        self.assertEqual(len(world.snapshot()), 3)
        self.assertGreater(world.rejected, 0)
        self.assertTrue(all(abs(b['y']) < 5 for b in world.snapshot()))


if __name__ == '__main__':
    unittest.main()
