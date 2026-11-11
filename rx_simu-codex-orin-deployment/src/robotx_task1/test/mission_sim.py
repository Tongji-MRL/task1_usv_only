"""Closed-loop stand-in for the vessel and its sensors; no ROS.

A unicycle with first-order speed and yaw-rate response. Buoys enter the
landmark map through the real LandmarkMap from simulated LiDAR clusters (inside
LIDAR_RANGE) and simulated camera labels (inside COLOR_RANGE and the forward
field of view), so the mission only ever sees what the vessel could have seen.
"""
import math

import numpy as np

from robotx_task1 import gap_planner as gp
from robotx_task1.core import passage_events
from robotx_task1.landmark_map import LandmarkMap, ObstacleGrid
from robotx_task1.mission_core import Mission
from robotx_task1.survey_core import Cluster

LIDAR_RANGE, COLOR_RANGE, HALF_FOV = 10.0, 5.0, math.radians(30.0)
DT, HULL = 0.1, 0.35
# The rule oracle exempts a stationary pivot. A real hull still drifts a little while it
# turns, so the judge here treats motion below 5 cm/s as a pivot as well.
PIVOT_STEP = 0.05*DT
LABEL = {gp.OFF: None, gp.RED: 'red', gp.GREEN: 'green', gp.ENTRY: 'blue', gp.EXIT: 'blue'}


def example_field():
    """Handbook mix: 2 red, 2 green, 4 off, entry and exit on opposite corners."""
    layout = dict(E=(0.0, 0.0, gp.ENTRY), X=(12.0, 8.0, gp.EXIT),
                  R1=(4.0, 3.0, gp.RED), G1=(4.0, 4.9, gp.GREEN),
                  R2=(8.0, 4.5, gp.RED), G2=(8.0, 6.5, gp.GREEN),
                  O1=(2.0, 6.5, gp.OFF), O2=(6.0, 2.0, gp.OFF),
                  O3=(6.0, 7.5, gp.OFF), O4=(10.0, 3.0, gp.OFF))
    return [dict(id=name, x=x, y=y, state=state, radius=0.2) for name, (x, y, state) in layout.items()]


def platform_outline(center, size, spacing=0.1):
    """Points along the edge of a square platform (what the LiDAR returns)."""
    half, (cx, cy) = size/2, center
    edge = np.arange(-half, half, spacing)
    return np.vstack([np.column_stack([cx+edge, np.full_like(edge, cy-half)]),
                      np.column_stack([np.full_like(edge, cx+half), cy+edge]),
                      np.column_stack([cx-edge, np.full_like(edge, cy+half)]),
                      np.column_stack([np.full_like(edge, cx-half), cy-edge])])


def square(center, size):
    half, (cx, cy) = size/2, center
    return [(cx-half, cy-half), (cx+half, cy-half), (cx+half, cy+half), (cx-half, cy+half)]


def polygon_outline(polygon, spacing=0.1):
    """Points along the edges of a polygon (what the LiDAR returns from a structure)."""
    points = []
    for a, b in zip(polygon, list(polygon[1:])+list(polygon[:1])):
        a, b = np.asarray(a, float), np.asarray(b, float)
        steps = max(1, int(np.ceil(np.linalg.norm(b-a)/spacing)))
        points += [a+(b-a)*i/steps for i in range(steps)]
    return np.array(points)


def polygon_distance(point, polygon):
    """Distance from a point to a polygon; negative inside."""
    x, y = point
    inside, best = False, float('inf')
    for (x0, y0), (x1, y1) in zip(polygon, list(polygon[1:])+list(polygon[:1])):
        if (y0 > y) != (y1 > y) and x < x0+(y-y0)*(x1-x0)/(y1-y0):
            inside = not inside
        dx, dy = x1-x0, y1-y0
        t = 0.0 if dx == dy == 0 else min(1.0, max(0.0, ((x-x0)*dx+(y-y0)*dy)/(dx*dx+dy*dy)))
        best = min(best, math.hypot(x-x0-t*dx, y-y0-t*dy))
    return -best if inside else best


class Simulation:
    def __init__(self, buoys=None, pose=(-4.0, -3.0, 0.3), seed=0, platforms=(), structures=(), **mission):
        """Non-buoy objects seen through the ObstacleGrid: platforms are (center, size) squares,
        structures arbitrary polygons (a dock with legs is one polygon per beam)."""
        self.truth = buoys or example_field()
        self.platforms = [(tuple(c), float(size)) for c, size in platforms]
        self.structures = [square(c, size) for c, size in self.platforms]+[list(map(tuple, q)) for q in structures]
        self.grid = ObstacleGrid()
        self.pose, self.speed, self.rate, self.time = list(pose), 0.0, 0.0, 0.0
        self.rng = np.random.default_rng(seed)
        self.map = LandmarkMap()
        self.mission = Mission(**mission)
        self.track, self.events, self.states = [tuple(pose)], [], []
        self.phase, self.classified, self.commands = [], [], []
        self.inputs_ok, self.mode, self.armed, self.blocked, self.rtk_ok = True, 'GUIDED', True, False, True
        self.max_command = (0.0, 0.0)

    def lamp(self, buoy):
        if buoy['state'] == gp.EXIT:
            return 'blue'
        lit = int(math.floor(self.time)) % 2 == 0
        return LABEL[buoy['state']] if lit and LABEL[buoy['state']] else 'none'

    def sense(self, step):
        here = self.pose[:2]
        if step % 5 == 0:
            outline = [polygon_outline(q) for q in self.structures]
            outline = np.vstack(outline) if outline else np.empty((0, 2))
            seen = outline[np.hypot(outline[:, 0]-here[0], outline[:, 1]-here[1]) <= LIDAR_RANGE]
            self.grid.update(seen+self.rng.normal(0, 0.03, seen.shape))
            self.map.update_lidar(self.time, here, [
                Cluster(np.array([b['x'], b['y']])+self.rng.normal(0, 0.03, 2), 12, 0.4, 0.0, 0.9)
                for b in self.truth if math.dist(here, (b['x'], b['y'])) <= LIDAR_RANGE])
        for landmark in self.map.snapshot():
            real = min(self.truth, key=lambda b: math.dist((b['x'], b['y']), (landmark['x'], landmark['y'])))
            distance = math.dist(here, (landmark['x'], landmark['y']))
            bearing = math.atan2(landmark['y']-here[1], landmark['x']-here[0])
            off_axis = abs(math.atan2(math.sin(bearing-self.pose[2]), math.cos(bearing-self.pose[2])))
            if distance <= COLOR_RANGE and off_axis <= HALF_FOV:
                self.map.observe_color(landmark['id'], self.time, self.lamp(real))

    def run(self, duration, until=None):
        for step in range(int(duration/DT)):
            self.sense(step)
            out = self.mission.step(self.time, tuple(self.pose), self.map.snapshot(), self.inputs_ok,
                                    self.mode, self.armed, self.rtk_ok, self.blocked, self.grid.points())
            self.events += [dict(e, time=round(self.time, 1)) for e in out['events']]
            if not self.states or self.states[-1] != out['state']:
                self.states.append(out['state'])
            self.max_command = (max(self.max_command[0], out['v']), max(self.max_command[1], abs(out['w'])))
            assert out['v'] >= 0.0
            self.phase.append(out['state'])
            self.commands.append((out['v'], out['w']))
            self.classified.append({min(self.truth, key=lambda b: math.dist((b['x'], b['y']), (m['x'], m['y'])))['id']
                                    for m in self.map.snapshot() if m['state'] != gp.UNKNOWN})
            self.speed += (out['v']-self.speed)*DT/0.5
            self.rate += (out['w']-self.rate)*DT/0.4
            self.pose[0] += self.speed*math.cos(self.pose[2])*DT
            self.pose[1] += self.speed*math.sin(self.pose[2])*DT
            self.pose[2] += self.rate*DT
            self.time += DT
            self.track.append(tuple(self.pose))
            if until is not None and until(self, out):
                break
        return out

    def judge(self):
        """Independent verdict from the true buoys and the driven track.

        passages: rule-oracle events outside the two circles. circle_passages:
        events while circling, where outside buoys inevitably come abeam.
        blind: buoys that came abeam (outside the circles) before the map had
        classified them. winding: signed angle driven around each blue buoy
        within 2.5 m of it, as (largest clockwise turn, largest counter-clockwise turn).
        """
        result = dict(passages=[], circle_passages=[], blind=[], clearance=float('inf'), winding={},
                      platform_clearance=float('inf'))
        everything = [dict(b, state=gp.RED) for b in self.truth]
        blue = {b['id']: b for b in self.truth if b['state'] in (gp.ENTRY, gp.EXIT)}
        last, totals = {}, {}
        for index, (before, after) in enumerate(zip(self.track, self.track[1:])):
            circling = self.phase[index].startswith('CIRCLE')
            if math.dist(before[:2], after[:2]) < PIVOT_STEP:
                continue  # Turning on the spot: the beam sweeps past buoys, the vessel does not.
            # A tie for the nearest buoy is never credited by the oracle, but it is not a wrong side.
            events = [e for e in passage_events(list(before), list(after), self.truth) if not e['ambiguous']]
            result['circle_passages' if circling else 'passages'] += events
            if not circling:
                result['blind'] += [e['id'] for e in passage_events(list(before), list(after), everything)
                                    if e['id'] not in self.classified[index]]
            for polygon in self.structures:
                gap = polygon_distance(after[:2], polygon)-HULL
                result['platform_clearance'] = min(result['platform_clearance'], gap)
            result['clearance'] = min([result['clearance']]+[
                math.dist(after[:2], (b['x'], b['y']))-b['radius']-HULL for b in self.truth])
            for name, buoy in blue.items():
                if math.dist(after[:2], (buoy['x'], buoy['y'])) > 2.5:
                    last.pop(name, None)
                    continue
                angle = math.atan2(after[1]-buoy['y'], after[0]-buoy['x'])
                if name in last:
                    step = math.atan2(math.sin(angle-last[name]), math.cos(angle-last[name]))
                    total, low, high = totals.get(name, (0.0, 0.0, 0.0))
                    total += step
                    low, high = min(low, total), max(high, total)
                    totals[name] = (total, low, high)
                    # Largest turn in one sense, wherever it started: an approach that first
                    # swings the other way must not be subtracted from the full circle.
                    clockwise, counter = result['winding'].get(name, (0.0, 0.0))
                    result['winding'][name] = (min(clockwise, total-high), max(counter, total-low))
                last[name] = angle
        return result
