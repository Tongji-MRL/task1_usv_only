"""Randomized closed-loop runs: jittered fields, start poses and color ranges.

The fixed seed makes this a regression test, not a proof. It records the
current, imperfect state honestly:
  - after the entry circle (passages before it do not score, handbook Task 1), no run
    may pass a red or green buoy on the wrong side; no run may touch a buoy;
  - at least 90 % of the runs must finish both circles (2026-11-11: 30 of 33; the
    misses have a 3 m colour range and run into the 1200 s mission limit);
  - runs that do not finish must have stopped (ABORT) rather than wander;
  - at most one run may bring an unclassified buoy abeam after the entry circle (a
    near-tie between two buoys at almost the same distance, decided by centimetres).
"""
import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import mission_sim as ms  # noqa: E402
import test_mission as tm  # noqa: E402
from robotx_task1 import gap_planner as gp  # noqa: E402
from robotx_task1.core import passage_events  # noqa: E402


def scored_part(sim):
    """(wrong-side events, unclassified buoys abeam) after the entry circle, outside circles."""
    done = next((e['time'] for e in sim.events if e['type'] == 'circle_entry_done'), None)
    first = int(round(done/ms.DT)) if done is not None else len(sim.track)
    everything = [dict(b, state=gp.RED) for b in sim.truth]
    wrong, blind = [], set()
    for i in range(first, len(sim.track)-1):
        a, b = sim.track[i], sim.track[i+1]
        if sim.phase[i].startswith('CIRCLE') or math.dist(a[:2], b[:2]) < ms.PIVOT_STEP:
            continue
        wrong += [e for e in passage_events(list(a), list(b), sim.truth) if not e['ambiguous'] and not e['legal']]
        blind |= {e['id'] for e in passage_events(list(a), list(b), everything) if e['id'] not in sim.classified[i]}
    return wrong, sorted(blind)


def cases(count=70, seed=5):
    rng = random.Random(seed)
    for index in range(count):
        angle, distance = rng.uniform(0, 2*math.pi), rng.uniform(3, 7)
        color_range = rng.choice([3.0, 4.0, 5.0, 6.0])
        field = tm.mirrored_field() if index % 2 else ms.example_field()
        for buoy in field:
            buoy['x'] += rng.uniform(-0.3, 0.3)
            buoy['y'] += rng.uniform(-0.3, 0.3)
        pose = (distance*math.cos(angle)-1, distance*math.sin(angle), rng.uniform(-3, 3))
        # Start outside the field, on the entry side, clear of every buoy.
        if pose[0] <= -1.5 and min(math.dist(pose[:2], (b['x'], b['y'])) for b in field) >= 1.5:
            yield index, field, pose, color_range


class SweepTests(unittest.TestCase):
    def test_randomized_runs(self):
        original = ms.COLOR_RANGE
        outcomes, wrong, blind, touched = {}, [], [], []
        try:
            for index, field, pose, color_range in cases():
                ms.COLOR_RANGE = color_range
                sim = ms.Simulation(buoys=field, pose=pose, seed=index, look_range=color_range-0.2)
                sim.mission.start()
                # Slightly beyond the mission's own 1200 s limit, so a run that cannot finish
                # is seen stopping itself (ABORT) rather than being cut off by the test.
                outcomes[index] = sim.run(1210, until=tm.finished)['state']
                verdict = sim.judge()
                scored_wrong, scored_blind = scored_part(sim)
                wrong += [(index, e['id'], e['side']) for e in scored_wrong]
                blind += [(index, name) for name in scored_blind]
                touched += [index] if verdict['clearance'] <= 0.0 else []
        finally:
            ms.COLOR_RANGE = original
        self.assertGreaterEqual(len(outcomes), 25)
        self.assertEqual(wrong, [])
        self.assertEqual(touched, [])
        unfinished = {k: v for k, v in outcomes.items() if v != 'COMPLETE'}
        self.assertLessEqual(len(unfinished), 0.1*len(outcomes), unfinished)
        self.assertEqual(set(unfinished.values())-{'ABORT'}, set(), unfinished)
        self.assertLessEqual(len({index for index, _ in blind}), 1, blind)


if __name__ == '__main__':
    unittest.main()
