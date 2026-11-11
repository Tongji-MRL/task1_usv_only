"""LiDAR-only moving-object envelopes and conservative short-horizon yielding.

Inputs are measured points in map, the FCU pose and an accepted spatial path.
No scene identities, traffic dimensions, truth poses or scheduled motion enter
this module. A corridor test deliberately over-approximates timing uncertainty:
an object crossing any reachable part of the corridor triggers a yield.
"""
from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree, ConvexHull
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.ndimage import maximum_filter


class TrafficLease:
    """Both source and receipt age matter; a new path needs its own assessment."""
    def __init__(self, timeout=.4):
        self.timeout = timeout
        self.stamp = -math.inf
        self.received = -math.inf
        self.path_stamp = 0.
        self.clear = False
        self.clear_since = None
        self.reason = 'traffic assessment missing'

    def update(self, stamp, path_stamp, clear, reason, sim, wall):
        if (not all(math.isfinite(t) for t in (stamp, path_stamp, sim, wall)) or
                stamp <= self.stamp or not -.05 <= sim - stamp <= self.timeout+1e-9):
            return False
        if not clear:self.clear_since=None
        elif not self.clear or not self.valid(sim,wall) or path_stamp!=self.path_stamp:self.clear_since=stamp
        self.stamp, self.received, self.path_stamp = stamp, wall, path_stamp
        self.clear, self.reason = clear, reason
        return True

    def valid(self, sim, wall):
        return -.05 <= sim - self.stamp <= self.timeout+1e-9 and 0 <= wall - self.received <= self.timeout+1e-9

    def permits(self, sim, wall, path_stamp):
        return self.valid(sim, wall) and self.clear and self.path_stamp == path_stamp

    def clear_for(self, sim, wall, duration=1.):
        return (self.valid(sim,wall) and self.clear and self.clear_since is not None and
                self.stamp-self.clear_since >= duration)


class StaticMap:
    """Exclude surveyed fixed surfaces; sliding views of a quay are not motion."""
    def __init__(self, grid, resolution, origin, padding=.4):
        self.resolution = resolution
        self.origin = np.asarray(origin)
        cells = int(math.ceil(padding/resolution))
        self.occupied = maximum_filter((grid >= 65) | (grid < 0), size=2*cells+1)

    def unmapped(self, points):
        delta = np.asarray(points)[:, :2] - self.origin[:2]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        indices = np.floor(delta @ [[c,-s],[s,c]] / self.resolution).astype(int)
        height, width = self.occupied.shape
        valid = (indices[:,0]>=0)&(indices[:,0]<width)&(indices[:,1]>=0)&(indices[:,1]<height)
        keep = np.zeros(len(points),dtype=bool)
        keep[valid] = ~self.occupied[indices[valid,1],indices[valid,0]]
        return np.asarray(points)[keep]

    def disk_clear(self,center,radius):
        delta=np.asarray(center)-self.origin[:2];c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        cell=delta@np.array([[c,-s],[s,c]])/self.resolution
        extent=radius/self.resolution+math.sqrt(.5)
        lo=np.floor(cell-extent).astype(int);hi=np.ceil(cell+extent).astype(int)
        h,w=self.occupied.shape
        if np.any(lo<0) or hi[0]>=w or hi[1]>=h:return False
        yy,xx=np.mgrid[lo[1]:hi[1]+1,lo[0]:hi[0]+1]
        disk=(xx+.5-cell[0])**2+(yy+.5-cell[1])**2<=extent**2
        return not np.any(self.occupied[lo[1]:hi[1]+1,lo[0]:hi[0]+1]&disk)


def exclude_surveyed_markers(points, markers):
    """Fixed surveyed buoys remain in KnownBuoys/collision costs, not traffic tracks."""
    keep = np.ones(len(points),dtype=bool)
    for x,y,radius in markers:
        keep &= np.linalg.norm(points[:,:2]-[x,y],axis=1) > radius+.4
    return points[keep]


def clusters(points, radius=.55, minimum=5):
    points = np.asarray(points, dtype=float)[:, :2]
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return []
    # Uniform spatial support, instead of overweighting densely sampled faces.
    _, indices = np.unique(np.floor(points / .12), axis=0, return_index=True)
    points = points[indices]
    pairs = cKDTree(points).query_pairs(radius, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
                       shape=(len(points), len(points)))
    count, labels = connected_components(graph, directed=False)
    return [np.array([p.min(axis=0), p.max(axis=0)])
            for label in range(count) if len(p := points[labels == label]) >= minimum]


def tracking_points(points, sensor_position, tracks, stamp, acquisition_range=20., continuation_range=40.):
    """Use distant measured surfaces only to continue an established object.

    LaserScan's navigation range is not the raw LiDAR's range. Cutting an
    established object at that boundary creates a needless loss, while using
    all distant partial quay faces to acquire tracks creates false motion.
    Keep the original acquisition window and admit actual farther returns
    only inside the association envelope of a confirmed moving track.
    """
    points=np.asarray(points)
    ranges=np.linalg.norm(points[:,:2]-np.asarray(sensor_position)[:2],axis=1)
    keep=ranges<=acquisition_range
    for track in tracks:
        if not track.moving:continue
        padding=.6+.15*min(max(stamp-track.stamp,0.),3.)
        box=track.at(stamp)
        keep |= ((ranges<=continuation_range)&
                 np.all(points[:,:2]>=box[0]-padding,axis=1)&
                 np.all(points[:,:2]<=box[1]+padding,axis=1))
    return points[keep]


@dataclass
class Track:
    id: int
    stamp: float
    bounds: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))
    moving: bool = False
    history: deque = field(default_factory=deque)
    extent: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def observe(self, bounds, stamp):
        self.extent = np.maximum(self.extent,bounds[1]-bounds[0])
        shrinking = bounds[1]-bounds[0] < .8*self.extent
        self.history.append((stamp, bounds.copy()))
        while self.history and stamp - self.history[0][0] > 3.:
            self.history.popleft()
        times = np.array([t for t, _ in self.history])
        boxes = np.array([b for _, b in self.history])
        if len(times) >= 6 and times[-1] - times[0] >= 1.8:
            centers = boxes.mean(axis=1)
            dt = times - times.mean()
            velocity = dt @ (centers - centers.mean(axis=0)) / (dt @ dt)
            residual = np.linalg.norm(centers - centers.mean(axis=0) - dt[:, None] * velocity, axis=1)
            speed = np.linalg.norm(velocity)
            edge_velocity = (dt @ boxes.reshape(len(times),4) / (dt @ dt)).reshape(2,2)
            consistent_edges = np.abs(edge_velocity[1]-edge_velocity[0]) < .10
            # A changing visible face is not a reliable acceleration estimate.
            reliable = np.quantile(residual, .9) < .16 and speed < 2.
            stable_size = np.ptp(boxes[:,1]-boxes[:,0],axis=0)<.2
            if reliable:
                for axis in range(2):
                    if consistent_edges[axis] and (not self.moving or not shrinking[axis] or stable_size[axis]):
                        self.velocity[axis] = .25 * velocity[axis] + .75 * self.velocity[axis]
                if np.all(consistent_edges) and speed >= .10 and np.linalg.norm(centers[-1] - centers[0]) >= .25:
                    self.moving = True
        # Retain the observed extent through a partial view, translating old
        # surfaces by measured motion. Never shrink to just the visible cabin.
        projected = boxes + (stamp - times)[:, None, None] * self.velocity
        self.bounds = np.array([projected[:, 0].min(axis=0), projected[:, 1].max(axis=0)])
        if self.moving:
            for axis in range(2):
                if shrinking[axis]:
                    # Bound all placements of the previously observed extent
                    # containing today's visible fragment. Anchor to fresh
                    # surfaces, not an indefinitely extrapolated old corner.
                    self.bounds[0,axis]=min(self.bounds[0,axis],bounds[1,axis]-self.extent[axis])
                    self.bounds[1,axis]=max(self.bounds[1,axis],bounds[0,axis]+self.extent[axis])
        self.stamp = stamp

    def at(self, stamp):
        return self.bounds + max(0., stamp - self.stamp) * self.velocity


class Tracker:
    def __init__(self, horizon=20., observation_range=20.):
        self.tracks = []
        self.stamp = -math.inf
        self.next_id = 1
        self.merges = []
        self.retired = []
        self.horizon = horizon
        self.observation_range = observation_range

    def update(self, points, stamp, position=None):
        if not math.isfinite(stamp) or stamp <= self.stamp:
            raise ValueError('Non-monotonic source cloud')
        self.stamp = stamp
        self.merges = []
        self.retired = []
        observations = clusters(points)
        # Lost moving objects are retained, not silently declared free water.
        self.tracks = [t for t in self.tracks if t.moving or stamp - t.stamp <= 3.]
        def contains(outer,inner,padding=.25):
            return np.all(inner[0]>=outer[0]-padding) and np.all(inner[1]<=outer[1]+padding)
        # A roof and hull can be disconnected in the projected point cloud.
        # Associate fragments overlapping an already measured envelope. Full
        # containment fails when a newly visible bow extends beyond an older
        # partial view, especially while the velocity estimate is warming up.
        for track in sorted(self.tracks,key=lambda t:np.prod(t.bounds[1]-t.bounds[0]),reverse=True):
            if stamp-track.stamp>2.:
                continue
            box=track.at(stamp)
            enclosed=[i for i,b in enumerate(observations)
                      if np.all(b[0]<=box[1]+.15) and np.all(b[1]>=box[0]-.15)]
            if len(enclosed)>1:
                boxes=np.array([observations[i] for i in enclosed])
                observations=[b for i,b in enumerate(observations) if i not in enclosed]
                observations.append(np.array([boxes[:,0].min(axis=0),boxes[:,1].max(axis=0)]))
        used = set()
        if observations and self.tracks:
            predicted = np.array([t.at(stamp) for t in self.tracks])
            observed = np.array(observations)
            distance = np.linalg.norm(predicted.mean(axis=1)[:,None]-observed.mean(axis=1)[None,:],axis=2)
            gap=np.linalg.norm(np.maximum(np.maximum(predicted[:,None,0]-observed[None,:,1],
                                observed[None,:,0]-predicted[:,None,1]),0.),axis=2)
            age=np.array([min(stamp-t.stamp,3.) for t in self.tracks])[:,None]
            size_limit=(np.linalg.norm(predicted[:,1]-predicted[:,0],axis=1)[:,None]+
                        np.linalg.norm(observed[:,1]-observed[:,0],axis=1)[None,:])*.5+.5+.15*age
            valid=(gap<=.6+.15*age)&(distance<=size_limit)
            cost=np.where(valid,3*gap+.1*distance,1e6)
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                if valid[i,j]:
                    self.tracks[i].observe(observations[j], stamp)
                    used.add(j)
        for i, bounds in enumerate(observations):
            if i in used:
                continue
            track = Track(self.next_id, stamp, bounds)
            self.next_id += 1
            track.observe(bounds, stamp)
            self.tracks.append(track)
        removed=set()
        for small in self.tracks:
            for large in self.tracks:
                if (small.id==large.id or large.id in removed or not large.moving or large.stamp!=stamp or
                        stamp-small.stamp>3. or
                        np.prod(large.bounds[1]-large.bounds[0]) < 2*np.prod(small.bounds[1]-small.bounds[0])):
                    continue
                box=small.at(stamp)
                if contains(large.bounds,box) and np.linalg.norm(small.velocity-large.velocity)<.3:
                    large.bounds=np.array([np.minimum(large.bounds[0],box[0]),np.maximum(large.bounds[1],box[1])])
                    removed.add(small.id)
                    self.merges.append([small.id,large.id])
                    break
        self.tracks=[t for t in self.tracks if t.id not in removed]
        if position is not None:
            kept=[]
            for track in self.tracks:
                future=track.at(stamp+self.horizon)
                window=np.array([np.minimum(track.bounds[0],future[0]),np.maximum(track.bounds[1],future[1])])
                distance=np.linalg.norm(np.maximum(np.maximum(window[0]-position,position-window[1]),0.))
                # The guard has a 20 m observation range. Retire an unseen
                # track only after its last and predicted envelopes are both
                # outside that range plus 2 m. Near-field loss never expires.
                if stamp-track.stamp>2. and distance>self.observation_range+2.:
                    self.retired.append(track.id)
                else:kept.append(track)
            self.tracks=kept
        return self.tracks


def corridor(footprint, pose, path, travel):
    """Full hull swept along the next reachable spatial path segment."""
    poses = [np.asarray(pose)]
    if path is not None and len(path):
        path = np.asarray(path)
        index = int(np.argmin(np.linalg.norm(path[:, :2] - pose[:2], axis=1)))
        distance = 0.
        last = np.asarray(pose[:2])
        for p in path[index:]:
            distance += np.linalg.norm(p[:2] - last)
            last = p[:2]
            if distance > travel:
                break
            if np.linalg.norm(p[:2] - poses[-1][:2]) >= .25:
                poses.append(p)
    polygons = []
    for x, y, a in poses:
        c, s = math.cos(a), math.sin(a)
        polygons.append(np.asarray(footprint) @ [[c, s], [-s, c]] + [x, y])
    points = np.concatenate(polygons)
    return points[ConvexHull(points).vertices]


def intersects_box(polygon, bounds, padding):
    lo, hi = bounds[0] - padding, bounds[1] + padding
    box = np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]])
    edges = np.roll(polygon, -1, axis=0) - polygon
    axes = np.concatenate([np.column_stack([-edges[:, 1], edges[:, 0]]), np.eye(2)])
    p, b = polygon @ axes.T, box @ axes.T
    return not np.any((p.max(axis=0) < b.min(axis=0)) | (b.max(axis=0) < p.min(axis=0)))


def predicted_regions(track,stamp,horizon=20.,margin=.6,velocity_uncertainty=.04):
    age=max(0.,stamp-track.stamp)
    for future in np.arange(0.,horizon+.01,.5):
        box=track.at(stamp+future)
        padding=margin+.15+velocity_uncertainty*future+.2*age
        if age>2.:
            box=np.array([np.minimum(box[0],track.bounds[0]),np.maximum(box[1],track.bounds[1])])
        yield float(future),np.array([box[0]-padding,box[1]+padding])


def traffic_grid(tracks,stamp,shape,resolution,origin,horizon=20.,margin=.6):
    """Separate inferred occupancy, never a rewrite of measured point clouds."""
    grid=np.zeros(shape,dtype=np.int8)
    c,s=math.cos(origin[2]),math.sin(origin[2]);rotation=np.array([[c,-s],[s,c]])
    for track in tracks:
        if not track.moving:continue
        for _,(lo,hi) in predicted_regions(track,stamp,horizon,margin):
            corners=np.array([lo,[lo[0],hi[1]],hi,[hi[0],lo[1]]])
            cells=(corners-origin[:2])@rotation/resolution
            begin=np.floor(cells.min(axis=0)).astype(int);end=np.ceil(cells.max(axis=0)).astype(int)+1
            x0,y0=np.maximum(begin,0);x1,y1=np.minimum(end,[shape[1],shape[0]])
            if x0<x1 and y0<y1:grid[y0:y1,x0:x1]=100
    return grid


def conflict(tracks, stamp, swept, horizon=20., margin=.6, velocity_uncertainty=.04):
    """Return the earliest conservative corridor conflict, or None.

    After 2 s without observations a moving track's extrapolation is no longer
    trusted: keep its last/predicted envelope and grow uncertainty. This may
    require operator intervention after prolonged occlusion, but cannot grant
    clearance just because a track timed out.
    """
    risks = []
    for track in tracks:
        if not track.moving:
            continue
        age = max(0., stamp - track.stamp)
        for future,box in predicted_regions(track,stamp,horizon,margin,velocity_uncertainty):
            if intersects_box(swept, box, 0.):
                risks.append(dict(track_id=track.id, future_s=float(future), age_s=age,
                                  bounds=track.bounds.tolist(), velocity=track.velocity.tolist(),
                                  reason='lost moving target' if age > 2. else 'predicted traffic conflict'))
                break
    return min(risks, key=lambda x: x['future_s']) if risks else None
