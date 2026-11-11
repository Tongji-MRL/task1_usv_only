"""Offline measurement helpers for the USV-only Task 1 survey (no ROS imports).

Step 1 of docs/task1/usv_only_plan.md: from recorded data, measure how far the
MID-360 forms a stable buoy cluster and how far the forward color camera
separates the lamp colors. Times are float seconds on one common clock.
"""
import math
from collections import namedtuple

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

# Optical axes (z forward, x right, y down) expressed in a camera body frame
# (x forward, y left, z up); REP-103 rpy (-pi/2, 0, -pi/2).
BODY_FROM_OPTICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])

Cluster = namedtuple('Cluster', 'center count extent z_min z_max')
Window = namedtuple('Window', 'time boat clusters')

# Hue intervals in degrees. Yellow / orange and cyan stay unclassified.
HUES = dict(red=((340., 360.), (0., 20.)), green=((90., 170.),), blue=((200., 265.),))
# Operator truth label -> (expected camera label, lamp flashes at 50 % duty).
TRUTH = dict(red=('red', True), green=('green', True), blue_flash=('blue', True),
             blue_steady=('blue', False), off=('none', False))


class PoseTrack:
    """Interpolated FCU odometry; never bridges gaps longer than max_gap."""

    def __init__(self, times, positions, quaternions, max_gap=0.5):
        times = np.asarray(times, float)
        if len(times) < 2:
            raise ValueError('At least two odometry samples are required')
        # Keep strictly newer source stamps only; repeated or reordered samples
        # must not create zero-length interpolation intervals.
        keep = np.concatenate(([True], times[1:] > np.maximum.accumulate(times)[:-1]))
        if keep.sum() < 2:
            raise ValueError('Odometry stamps do not advance')
        self.times = times[keep]
        self.positions = np.asarray(positions, float)[keep]
        self.slerp = Slerp(self.times, Rotation.from_quat(np.asarray(quaternions, float)[keep]))
        self.max_gap = max_gap

    def at(self, times):
        """Return positions (n,3), Rotation (n) and a validity mask for times."""
        times = np.atleast_1d(np.asarray(times, float))
        clipped = np.clip(times, self.times[0], self.times[-1])
        upper = np.clip(np.searchsorted(self.times, clipped, side='right'), 1, len(self.times)-1)
        valid = ((times >= self.times[0]) & (times <= self.times[-1]) &
                 (self.times[upper]-self.times[upper-1] <= self.max_gap))
        positions = np.column_stack([np.interp(clipped, self.times, self.positions[:, i])
                                     for i in range(3)])
        return positions, self.slerp(clipped), valid


def to_world(points, times, track, mount_rotation, mount_translation, bin_s=0.005):
    """Sensor points -> x, y in the odometry frame and z above the base_link origin.

    Each point uses the vessel pose at its own measurement time (5 ms groups),
    so a turning vessel does not smear a buoy. z is gravity-aligned and relative
    to base_link, independent of the EKF altitude. Returns (points, valid).
    """
    points = np.asarray(points, float).reshape(-1, 3)
    if not len(points):
        return np.empty((0, 3)), np.empty(0, bool)
    body = mount_rotation.apply(points)+mount_translation
    unique, inverse = np.unique(np.floor(np.asarray(times, float)/bin_s).astype(np.int64),
                                return_inverse=True)
    positions, rotations, valid = track.at((unique+0.5)*bin_s)
    level = rotations[inverse].apply(body)
    return np.column_stack([positions[inverse, :2]+level[:, :2], level[:, 2]]), valid[inverse]


class StaticTree:
    """Mounting transforms from /tf_static; lookups may chain and invert edges."""

    def __init__(self):
        self.edges = {}

    def add(self, parent, child, translation, quaternion):
        parent, child = parent.lstrip('/'), child.lstrip('/')
        rotation, translation = Rotation.from_quat(quaternion), np.asarray(translation, float)
        self.edges.setdefault(parent, {})[child] = (rotation, translation)
        self.edges.setdefault(child, {})[parent] = (rotation.inv(), -rotation.inv().apply(translation))

    def lookup(self, parent, child):
        """Pose of child in parent as (Rotation, translation), or None."""
        parent, child = parent.lstrip('/'), child.lstrip('/')
        poses, queue = {parent: (Rotation.identity(), np.zeros(3))}, [parent]
        while queue:
            frame = queue.pop(0)
            rotation, translation = poses[frame]
            if frame == child:
                return rotation, translation
            for other, (r, t) in self.edges.get(frame, {}).items():
                if other not in poses:
                    poses[other] = (rotation*r, rotation.apply(t)+translation)
                    queue.append(other)
        return None


def polygon_in_frame(tree, polygon, target, source):
    """Polygon given in `source` (e.g. map) expressed in `target` (e.g. odom), or None."""
    pose = tree.lookup(target, source)
    if pose is None:
        return None
    rotation, translation = pose
    points = np.column_stack([np.asarray(polygon, float), np.zeros(len(polygon))])
    return [tuple(map(float, p[:2])) for p in rotation.apply(points)+translation]


def cluster(points, tolerance=0.35, cell=0.1, members=False):
    """Euclidean clusters in the horizontal plane; points are (x, y, z).

    Points are first reduced to occupied cells so that a dense shoreline cannot
    produce a quadratic number of neighbor pairs. With members=True the points
    of every cluster are returned as well: (clusters, list of point arrays).
    """
    points = np.asarray(points, float).reshape(-1, 3)
    if not len(points):
        return ([], []) if members else []
    cells, inverse = np.unique(np.floor(points[:, :2]/cell).astype(np.int64), axis=0,
                               return_inverse=True)
    inverse = inverse.reshape(-1)
    pairs = cKDTree((cells+0.5)*cell).query_pairs(tolerance, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs), bool), (pairs[:, 0], pairs[:, 1])),
                       shape=(len(cells), len(cells)))
    labels = connected_components(graph, directed=False)[1][inverse]
    order = np.argsort(labels, kind='stable')
    points, labels = points[order], labels[order]
    starts = np.flatnonzero(np.concatenate(([True], labels[1:] != labels[:-1])))
    counts = np.diff(np.concatenate((starts, [len(points)])))
    low, high = np.minimum.reduceat(points, starts), np.maximum.reduceat(points, starts)
    centers = np.add.reduceat(points[:, :2], starts)/counts[:, None]
    found = [Cluster(centers[i], int(counts[i]), float((high[i, :2]-low[i, :2]).max()),
                     float(low[i, 2]), float(high[i, 2])) for i in range(len(starts))]
    if not members:
        return found
    return found, np.split(points, starts[1:])


def candidates(clusters, min_points=5, max_extent=1.0):
    """Buoy-sized clusters; walls, docks and sparse clutter are excluded."""
    return [c for c in clusters if c.count >= min_points and c.extent <= max_extent]


def build_landmarks(windows, gate=0.5, min_windows=10):
    """Nearest-neighbor tracks of static clusters, weighting near observations.

    The gate must stay below the field gap (1.5-2 m clear water) so adjacent
    buoys are never merged. Only tracks seen in min_windows windows are kept.
    """
    tracks = []
    for window in windows:
        if not window.clusters:
            continue
        centers = np.array([c.center for c in window.clusters])
        pairs = []
        if tracks:
            distance = np.linalg.norm(np.array([t['xy'] for t in tracks])[:, None]-centers[None], axis=2)
            pairs = sorted((distance[i, j], i, j) for i, j in zip(*np.nonzero(distance <= gate)))
        used_tracks, used_clusters = set(), set()
        for _, i, j in pairs:
            if i in used_tracks or j in used_clusters:
                continue
            used_tracks.add(i)
            used_clusters.add(j)
            track = tracks[i]
            weight = 1./max(math.dist(window.boat, centers[j]), 1.)**2
            track['xy'] = (track['xy']*track['weight']+centers[j]*weight)/(track['weight']+weight)
            track['weight'] += weight
            track['hits'].append((window.time, window.boat, window.clusters[j]))
        for j, found in enumerate(window.clusters):
            if j not in used_clusters:
                weight = 1./max(math.dist(window.boat, centers[j]), 1.)**2
                tracks.append(dict(xy=centers[j].copy(), weight=weight,
                                   hits=[(window.time, window.boat, found)]))
    kept = sorted((t for t in tracks if len(t['hits']) >= min_windows),
                  key=lambda t: (t['hits'][0][0], t['xy'][0], t['xy'][1]))
    landmarks = []
    for index, track in enumerate(kept, 1):
        ranges = np.array([math.dist(boat, track['xy']) for _, boat, _ in track['hits']])
        near = np.argsort(ranges)[:max(3, len(ranges)//4)]
        found = [track['hits'][i][2] for i in near]
        landmarks.append(dict(
            id=f'L{index:02d}', x=float(track['xy'][0]), y=float(track['xy'][1]),
            hits=len(track['hits']), first_time=float(track['hits'][0][0]),
            nearest_range=float(ranges.min()), farthest_range=float(ranges.max()),
            extent=float(np.median([c.extent for c in found])),
            # Height above base_link from the closest quarter of the observations,
            # where the vertical field of view covers most of the buoy.
            z_top=float(np.percentile([c.z_max for c in found], 90)),
            z_bottom=float(np.percentile([c.z_min for c in found], 10))))
    return landmarks


def detection_table(windows, landmark_xy, gate=0.5, bin_width=1.0, max_range=30.0):
    """Per range bin: windows spent there and windows with a cluster at the landmark."""
    count = int(math.ceil(max_range/bin_width))
    rows = [dict(lo=i*bin_width, hi=(i+1)*bin_width, windows=0, detected=0, points=[],
                 errors=[], farthest=None) for i in range(count)]
    for window in windows:
        distance = math.dist(window.boat, landmark_xy)
        if distance >= count*bin_width:
            continue
        row = rows[int(distance/bin_width)]
        row['windows'] += 1
        errors = [math.dist(c.center, landmark_xy) for c in window.clusters]
        if errors and min(errors) <= gate:
            best = int(np.argmin(errors))
            row['detected'] += 1
            row['points'].append(window.clusters[best].count)
            row['errors'].append(errors[best])
            row['farthest'] = max(row['farthest'] or 0., distance)
    for row in rows:
        points, errors = row.pop('points'), row.pop('errors')
        row['rate'] = row['detected']/row['windows'] if row['windows'] else None
        row['median_points'] = int(np.median(points)) if points else 0
        # Scatter of the cluster about the landmark; grows with range when the
        # heading used for the transform is wrong.
        row['rms_error'] = float(np.sqrt(np.mean(np.square(errors)))) if errors else None
    return [row for row in rows if row['windows']]


def farthest_detection(table):
    found = [row['farthest'] for row in table if row.get('farthest') is not None]
    return max(found) if found else None


def detection_ok(min_rate=0.9, min_windows=6):
    """Bin verdict: True / False, or None when too few windows were recorded."""
    def verdict(row):
        if row['windows'] < min_windows:
            return None
        return row['rate'] >= min_rate
    return verdict


def stable_band(table, verdict):
    """Longest run of adjacent passing bins as dict(near, far), or None.

    A failing or under-sampled bin ends the run: range is not claimed across
    distances that were not actually demonstrated.
    """
    best, run = None, None
    for row in sorted(table, key=lambda r: r['lo']):
        if verdict(row) and (run is None or abs(row['lo']-run['far']) < 1e-9):
            run = dict(near=run['near'] if run else row['lo'], far=row['hi'])
        elif verdict(row):
            run = dict(near=row['lo'], far=row['hi'])
        else:
            run = None
        if run and (best is None or run['far']-run['near'] > best['far']-best['near']):
            best = dict(run)
    return best


class Camera:
    """Pinhole color camera; rotation/translation are the optical frame in base_link."""

    def __init__(self, fx, fy, cx, cy, width, height, rotation, translation):
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)
        self.width, self.height = int(width), int(height)
        self.rotation, self.translation = rotation, np.asarray(translation, float)

    def project(self, body_points):
        optical = self.rotation.inv().apply(np.atleast_2d(np.asarray(body_points, float))-self.translation)
        depth = optical[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            uv = np.column_stack([self.fx*optical[:, 0]/depth+self.cx,
                                  self.fy*optical[:, 1]/depth+self.cy])
        return uv, depth

    def roi(self, body_point, width_m, height_m, min_px=4, min_depth=0.5):
        """Pixel box (u0, v0, u1, v1) of a physical box at body_point, or None."""
        uv, depth = self.project(body_point)
        (u, v), depth = uv[0], depth[0]
        if not depth >= min_depth or not (0 <= u < self.width and 0 <= v < self.height):
            return None
        size_u = max(min_px, int(round(self.fx*width_m/depth)))
        size_v = max(min_px, int(round(self.fy*height_m/depth)))
        u0, v0 = int(round(u-size_u/2)), int(round(v-size_v/2))
        return (max(0, u0), max(0, v0), min(self.width, u0+size_u), min(self.height, v0+size_v))


def level_to_body(landmark_xy, z_above_base, boat_position, boat_rotation):
    """Landmark (odometry x, y; z above base_link) expressed in base_link."""
    return boat_rotation.inv().apply([landmark_xy[0]-boat_position[0],
                                      landmark_xy[1]-boat_position[1], z_above_base])


def rois_overlap(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def overlapping(rois):
    """Ids whose boxes overlap another box; neither lamp is judged in that frame."""
    names = list(rois)
    return {n for n in names for m in names if n != m and rois_overlap(rois[n], rois[m])}


def hue_saturation_value(rgb):
    rgb = np.asarray(rgb, np.float32)/255.
    high, low = rgb.max(-1), rgb.min(-1)
    delta = high-low
    safe = np.where(delta > 0, delta, 1.)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    hue = np.where(high == r, ((g-b)/safe) % 6, np.where(high == g, (b-r)/safe+2, (r-g)/safe+4))*60.
    return np.where(delta > 0, hue, 0.), np.where(high > 0, delta/np.where(high > 0, high, 1.), 0.), high


def classify_roi(rgb, s_min=0.45, v_min=0.45, min_pixels=4, dominance=0.6):
    """Count bright saturated pixels per lamp color in one RGB region.

    A lamp is reported only if one color has min_pixels and dominates the other
    lamp colors. Clipped white pixels are counted separately: many of them with
    label 'none' means the exposure is too high to see the LED color.
    """
    rgb = np.asarray(rgb, np.uint8).reshape(-1, 3)
    hue, saturation, value = hue_saturation_value(rgb)
    lit = (saturation >= s_min) & (value >= v_min)
    counts = {name: int(sum(np.count_nonzero(lit & (hue >= lo) & (hue <= hi)) for lo, hi in spans))
              for name, spans in HUES.items()}
    best = max(counts, key=counts.get)
    total = sum(counts.values())
    label = best if counts[best] >= min_pixels and counts[best] >= dominance*total else 'none'
    return dict(label=label, pixels=len(rgb),
                white=int(np.count_nonzero((value >= 0.95) & (saturation < 0.2))), **counts)


def color_table(samples, bin_width=1.0, max_range=30.0):
    """Per range bin label counts from (range, label) samples of one landmark."""
    rows = {}
    for distance, label in samples:
        if distance >= max_range:
            continue
        index = int(distance/bin_width)
        row = rows.setdefault(index, dict(lo=index*bin_width, hi=(index+1)*bin_width, frames=0,
                                          red=0, green=0, blue=0, none=0))
        row['frames'] += 1
        row[label] += 1
    return [rows[i] for i in sorted(rows)]


def color_ok(truth, min_frames=10, flash_min=0.3, steady_min=0.8, wrong_max=0.05):
    """Bin verdict against the operator's truth label.

    A flashing lamp is on for half of the frames, so flash_min=0.3 requires
    roughly 60 % of the lit frames to show the right color. Any other lamp
    color in more than wrong_max of the frames fails the bin.
    """
    if truth not in TRUTH:
        raise ValueError(f'Unknown truth label {truth!r}; use one of {sorted(TRUTH)}')
    expected, flashing = TRUTH[truth]

    def verdict(row):
        if row['frames'] < min_frames:
            return None
        wrong = sum(row[k] for k in ('red', 'green', 'blue') if k != expected)/row['frames']
        if wrong > wrong_max:
            return False
        if expected == 'none':
            return True
        return row[expected]/row['frames'] >= (flash_min if flashing else steady_min)
    return verdict


def find_color_topics(topics):
    """(image topic, camera_info topic) of the color stream in a name->type map."""
    def matching(kind, tail):
        return sorted(n for n, t in topics.items() if t == kind and tail(n))
    images = (matching('sensor_msgs/msg/Image', lambda n: n.endswith('/color/image_raw')) or
              matching('sensor_msgs/msg/CompressedImage', lambda n: '/color/image_raw' in n))
    if not images:
        return None, None
    info = images[0].split('/image_raw')[0]+'/camera_info'
    return images[0], info if topics.get(info) == 'sensor_msgs/msg/CameraInfo' else None
