"""Online buoy landmark map and lamp-state decision (no ROS imports).

The LiDAR gives position and identity, the camera only the lamp color. A lamp
state is decided from an uninterrupted series of color labels and then latched:
a buoy that is no longer in view keeps its state.
"""
import math

import numpy as np

OFF, RED, GREEN, FLASH_BLUE, STEADY_BLUE, UNKNOWN = 0, 1, 2, 3, 4, 5
NAMES = {OFF: 'off', RED: 'red', GREEN: 'green', FLASH_BLUE: 'blue_flash',
         STEADY_BLUE: 'blue_steady', UNKNOWN: 'unknown'}


class LampClassifier:
    """Decide one buoy's lamp state from timed labels red / green / blue / none.

    Lamps flash 1 s on, 1 s off. Red and green need red_green_time of
    observation; telling flashing from steady blue needs blue_time (two
    periods); OFF needs off_time without any lit frame. Labels must only be
    added while the buoy is inside the reliable color range and unobstructed.
    """

    def __init__(self, red_green_time=2.0, blue_time=4.0, off_time=3.0, max_gap=0.5,
                 correction_time=3.0):
        self.red_green_time, self.blue_time, self.off_time = red_green_time, blue_time, off_time
        self.max_gap, self.correction_time = max_gap, correction_time
        self.samples = []
        self.state = UNKNOWN
        self.pending = None  # (different decision, time it was first made)

    def add(self, time, label):
        """Add one label; returns True if the latched state changed."""
        if self.samples and (time-self.samples[-1][0] > self.max_gap or time < self.samples[-1][0]):
            self.samples = []  # Interrupted: durations may not be bridged.
            self.pending = None
        self.samples.append((time, label))
        horizon = time-max(self.red_green_time, self.blue_time, self.off_time)-self.max_gap
        while self.samples[0][0] < horizon:
            self.samples.pop(0)
        decision = self.decide()
        if decision is None or decision == self.state:
            self.pending = None
            return False
        if self.state == UNKNOWN or self.state == OFF:
            self.state, self.pending = decision, None
            return True
        if decision == OFF:
            return False  # A lamp that was seen lit is not downgraded to OFF.
        if self.pending is None or self.pending[0] != decision:
            self.pending = (decision, time)
        if time-self.pending[1] >= self.correction_time:
            self.state, self.pending = decision, None
            return True
        return False

    def window(self, duration):
        """Labels of the last `duration` seconds, or None if not covered yet."""
        end = self.samples[-1][0]
        if end-self.samples[0][0] < duration-1e-9:
            return None
        return [s for s in self.samples if s[0] >= end-duration-1e-9]

    def decide(self):
        def share(samples, label):
            return sum(1 for _, found in samples if found == label)/len(samples)
        recent = self.window(self.red_green_time)
        if recent:
            for label, state in (('red', RED), ('green', GREEN)):
                others = sum(share(recent, k) for k in ('red', 'green', 'blue') if k != label)
                if share(recent, label) >= 0.25 and others <= 0.05:
                    return state
        long = self.window(self.blue_time)
        if long and share(long, 'red')+share(long, 'green') <= 0.05:
            blue = share(long, 'blue')
            if blue >= 0.85:
                return STEADY_BLUE
            if 0.25 <= blue <= 0.75 and self.flashing(long):
                return FLASH_BLUE
        quiet = self.window(self.off_time)
        if quiet and share(quiet, 'none') >= 0.97:
            return OFF
        return None

    @staticmethod
    def flashing(samples):
        """True if the complete on / off phases inside the window last about 1 s."""
        runs, start = [], 0
        for i in range(1, len(samples)+1):
            if i == len(samples) or (samples[i][1] == 'blue') != (samples[start][1] == 'blue'):
                end = samples[i][0] if i < len(samples) else samples[-1][0]
                runs.append(end-samples[start][0])
                start = i
        inner = runs[1:-1]  # The first and last phase are cut by the window.
        return len(inner) >= 2 and all(0.6 <= r <= 1.4 for r in inner)


def inside_polygon(point, polygon):
    """Even-odd test; no polygon means everything is inside."""
    if polygon is None:
        return True
    x, y, inside = point[0], point[1], False
    for (x0, y0), (x1, y1) in zip(polygon, np.roll(polygon, -1, axis=0)):
        if (y0 > y) != (y1 > y) and x < x0+(y-y0)*(x1-x0)/(y1-y0):
            inside = not inside
    return inside


def shrink_polygon(polygon, margin):
    """Convex polygon moved inward by `margin` (either vertex order)."""
    polygon = [tuple(map(float, p)) for p in polygon]
    area = sum(x0*y1-x1*y0 for (x0, y0), (x1, y1) in zip(polygon, polygon[1:]+polygon[:1]))
    sign = 1.0 if area > 0 else -1.0
    lines = []
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:]+polygon[:1]):
        dx, dy = x1-x0, y1-y0
        length = math.hypot(dx, dy)
        nx, ny = -sign*dy/length, sign*dx/length  # Inward normal.
        lines.append(((x0+nx*margin, y0+ny*margin), (dx, dy)))
    out = []
    for (p, d), (q, e) in zip(lines[-1:]+lines[:-1], lines):
        det = d[0]*e[1]-d[1]*e[0]
        t = ((q[0]-p[0])*e[1]-(q[1]-p[1])*e[0])/det
        out.append((p[0]+t*d[0], p[1]+t*d[1]))
    return out


def inside_course(point, polygon, corner_radius=1.0, tolerance=0.3, markers=()):
    """True for objects that belong to the course: inside the boundary polygon (lettered
    buoys at its vertices), `tolerance` metres of position error allowed, and farther
    than `corner_radius` from every vertex, i.e. not one of the boundary buoys.

    A buoy may lie close to a boundary line (0.8 m on the Delta course), so the polygon
    is not shrunk; only the corner buoys themselves are excluded.

    markers: known buoys that are not Task 1 buoys (e.g. a start gate), excluded like the
    corner buoys. The mission avoids them as obstacle points (BuoyMapper publishes them).
    """
    if any(math.dist(point[:2], q) <= corner_radius for q in markers):
        return False
    if polygon is None:
        return True
    polygon = [tuple(map(float, q)) for q in polygon]
    if any(math.dist(point[:2], q) <= corner_radius for q in polygon):
        return False
    return inside_polygon(point, shrink_polygon(polygon, -tolerance))


class ObstacleGrid:
    """Non-buoy objects (platform, dock) as occupied cells in the odometry frame.

    Fed with the points of LiDAR clusters that are too large to be a buoy. A
    cell is confirmed when it was hit in `confirm_hits` of the last
    `confirm_windows` LiDAR windows, and then kept: the Task 1 objects are
    moored, and a cell that disappears may only be occluded. A moving boat
    inside the boundary therefore leaves cells behind; the boundary limits this.
    """

    def __init__(self, cell=0.5, confirm_hits=3, confirm_windows=8, boundary=None, max_cells=4000,
                 corner_radius=1.0):
        """boundary: the course polygon through the lettered corner buoys (see inside_course)."""
        self.cell, self.confirm_hits, self.confirm_windows = cell, confirm_hits, confirm_windows
        self.boundary = None if boundary is None else np.asarray(boundary, float)
        self.max_cells, self.corner_radius = max_cells, corner_radius
        self.markers = []  # Known non-task buoys (see inside_course).
        self.hits, self.confirmed, self.windows, self.revision = {}, set(), 0, 0

    def update(self, points):
        """One LiDAR window: (n, 2+) points of all oversized clusters."""
        self.windows += 1
        points = np.asarray(points, float)
        if not points.size:
            return
        points = points.reshape(len(points), -1)
        for key in {tuple(k) for k in np.floor(points[:, :2]/self.cell).astype(np.int64)}:
            if key in self.confirmed:
                continue
            center = ((key[0]+0.5)*self.cell, (key[1]+0.5)*self.cell)
            if not inside_course(center, self.boundary, self.corner_radius, markers=self.markers):
                continue
            recent = [w for w in self.hits.get(key, []) if w > self.windows-self.confirm_windows]+[self.windows]
            self.hits[key] = recent
            if len(recent) >= self.confirm_hits and len(self.confirmed) < self.max_cells:
                self.confirmed.add(key)
                self.revision += 1
                del self.hits[key]
        if len(self.hits) > 4*self.max_cells:
            self.hits = {k: v for k, v in self.hits.items() if v[-1] > self.windows-self.confirm_windows}

    def points(self):
        """Centers of confirmed cells, sorted."""
        return [((k[0]+0.5)*self.cell, (k[1]+0.5)*self.cell) for k in sorted(self.confirmed)]


class LandmarkMap:
    """At most max_landmarks confirmed buoys, tracked by nearest-neighbor association."""

    def __init__(self, gate=0.5, confirm_hits=4, confirm_windows=8, tentative_timeout=5.0,
                 max_landmarks=10, boundary=None, weight_cap=20.0, corner_radius=1.0, **lamp):
        self.gate, self.confirm_hits, self.confirm_windows = gate, confirm_hits, confirm_windows
        self.tentative_timeout, self.max_landmarks, self.weight_cap = tentative_timeout, max_landmarks, weight_cap
        self.boundary = None if boundary is None else np.asarray(boundary, float)
        self.corner_radius = corner_radius
        self.markers = []  # Known non-task buoys (see inside_course).
        self.lamp = lamp
        self.tracks, self.windows, self.revision, self.rejected = [], 0, 0, 0

    def inside(self, point):
        return inside_course(point, self.boundary, self.corner_radius, markers=self.markers)

    def update_lidar(self, time, boat, clusters):
        """Associate one window of buoy-sized clusters (survey_core.Cluster)."""
        self.windows += 1
        clusters = [c for c in clusters if self.inside(c.center)]
        pairs = sorted((math.dist(t['xy'], c.center), i, j) for i, t in enumerate(self.tracks)
                       for j, c in enumerate(clusters) if math.dist(t['xy'], c.center) <= self.gate)
        used_tracks, used_clusters = set(), set()
        for _, i, j in pairs:
            if i in used_tracks or j in used_clusters:
                continue
            used_tracks.add(i)
            used_clusters.add(j)
            self.absorb(self.tracks[i], time, boat, clusters[j])
        for j, found in enumerate(clusters):
            if j not in used_clusters:
                track = dict(id=None, xy=np.array(found.center, float), weight=0., hits=[],
                             extent=found.extent, z_top=found.z_max, seen=time, lamp=LampClassifier(**self.lamp))
                self.absorb(track, time, boat, found)
                self.tracks.append(track)
        confirmed = sum(t['id'] is not None for t in self.tracks)
        for track in self.tracks:
            track['hits'] = [w for w in track['hits'] if w > self.windows-self.confirm_windows]
            if track['id'] is None and len(track['hits']) >= self.confirm_hits:
                if confirmed >= self.max_landmarks:
                    self.rejected += 1  # More objects than buoys: needs the operator's attention.
                    continue
                confirmed += 1
                track['id'] = f'B{confirmed}'
                self.revision += 1
        self.tracks = [t for t in self.tracks
                       if t['id'] is not None or time-t['seen'] <= self.tentative_timeout]

    def absorb(self, track, time, boat, found):
        distance = max(math.dist(boat, found.center), 1.)
        weight = 1./distance**2  # Near observations dominate the position.
        track['xy'] = (track['xy']*track['weight']+np.asarray(found.center)*weight)/(track['weight']+weight)
        track['weight'] = min(track['weight']+weight, self.weight_cap)
        track['hits'].append(self.windows)
        track['seen'] = time
        if distance <= 5.0 or track['weight'] <= weight:
            track['extent'], track['z_top'] = found.extent, max(found.z_max, track['z_top'])

    def observe_color(self, name, time, label):
        """Add a camera label for a confirmed landmark; True if its state changed."""
        track = next((t for t in self.tracks if t['id'] == name), None)
        if track is None:
            return False
        changed = track['lamp'].add(time, label)
        self.revision += changed
        return changed

    def snapshot(self):
        """Confirmed landmarks as dicts(id, x, y, state, radius, z_top)."""
        return [dict(id=t['id'], x=float(t['xy'][0]), y=float(t['xy'][1]), state=t['lamp'].state,
                     radius=float(min(0.5, max(0.1, t['extent']/2))), z_top=float(t['z_top']))
                for t in sorted((t for t in self.tracks if t['id'] is not None),
                                key=lambda t: int(t['id'][1:]))]
