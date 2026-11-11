"""Incremental gap planner for the USV-only Safe Passage (no ROS imports).

Buoys are landmarks with a lamp state; every Delaunay edge between two of them
is a candidate gap. A gap may be crossed only with a non-red buoy to port and a
non-green buoy to starboard. The resulting waypoints are then checked segment
by segment: a path that would put a wrong buoy abeam is rejected, one that
would put an unclassified buoy abeam is reported as needing a look first. The
tests judge the driven track with the independent oracle core.passage_events.

Obstacles that are not buoys (the Task 3 platform, a dock) are given as points,
for example 0.5 m occupancy cells from the LiDAR. They take part in the
triangulation and in the clearance check, so the path keeps away from them and
cannot pass between two neighbouring cells, but they never take part in the
port/starboard rule: the judges count buoys only.
"""
import heapq
import math

import numpy as np
from scipy.spatial import Delaunay

from .core import passage_events

OFF, RED, GREEN, ENTRY, EXIT, UNKNOWN = 0, 1, 2, 3, 4, 5

DEFAULTS = dict(
    min_width=1.2,        # Clear water needed between two buoy surfaces [m].
    wide_width=2.4,       # Wider gaps need no alignment / straight threading.
    align_distance=2.0,   # Alignment point before a narrow gap [m].
    through_distance=1.0,  # Pass-through point after a narrow gap [m].
    ring_margin=3.0,      # Virtual outside points this far beyond the field [m].
    ring_spacing=3.0,
    ring_penalty=4.0,     # Extra cost per crossing that uses an outside point [m].
    unknown_penalty=3.0,  # Extra cost per unclassified gap end [m].
    colour_penalty=8.0,   # Extra cost for crossing an edge with a red end to port or green to starboard [m].
    clearance=0.5,        # Path to buoy surface: half beam 0.35 m plus 0.15 m; below min_width/2.
    default_radius=0.2,
    avoid=(),             # Pairs of buoy ids whose gap must not be crossed (again).
    obstacles=(),         # (x, y) points of non-buoy obstacles.
    obstacle_radius=0.3,  # Half a 0.5 m cell plus position noise [m].
    max_replans=60,
    forbid=(),            # Points an escape step must not return to.
    outside=False,        # Before the entry circle: stay out of the buoy field if at all possible.
    outside_penalty=100.0,  # Extra cost per gap between two buoys crossed in outside mode [m].
    _escaping=False)       # Internal: no second escape step.


def _ring(points, margin, spacing):
    low, high = points.min(axis=0)-margin, points.max(axis=0)+margin
    corners = [low, (high[0], low[1]), high, (low[0], high[1])]
    ring = []
    for a, b in zip(corners, corners[1:]+corners[:1]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        steps = max(1, int(math.ceil(np.linalg.norm(b-a)/spacing)))
        ring += [a+(b-a)*i/steps for i in range(steps)]
    return np.array(ring)


def _segment_distance(p, a, b):
    ab, ap = b-a, p-a
    t = 0. if not ab.any() else min(1., max(0., float(ap @ ab)/float(ab @ ab)))
    return float(np.linalg.norm(ap-t*ab))


def field_hull(centers, margin=0.3):
    """Delaunay of the buoy centres moved `margin` towards their middle: the buoy field.

    None when there is no inside (fewer than three buoys, or all in a line)."""
    centers = np.asarray(centers, float).reshape(-1, 2)
    if len(centers) < 3:
        return None
    centre = centers.mean(axis=0)
    distance = np.linalg.norm(centers-centre, axis=1)
    shrunk = centre+(centers-centre)*np.maximum(0.0, 1.0-margin/np.maximum(distance, 1e-9))[:, None]
    try:
        return Delaunay(shrunk)
    except Exception:
        return None


def in_field(hull, point):
    return hull is not None and bool(hull.find_simplex(np.asarray(point, float).reshape(1, 2))[0] >= 0)


def _enters(path, centers, margin=0.3, step=0.2, leaving_ok=True):
    """True if the polyline passes inside the buoy field (see field_hull).

    With leaving_ok, a path that starts inside (the field grew when a buoy came into LiDAR
    range) only counts if it comes back in after having left."""
    hull = field_hull(centers, margin)
    if hull is None:
        return False
    points = [np.asarray(a, float)+(np.asarray(b, float)-a)*t for a, b in zip(path, path[1:])
              for t in np.linspace(0.0, 1.0, max(2, int(np.linalg.norm(np.asarray(b, float)-a)/step)+1))]
    inside = hull.find_simplex(np.array(points)) >= 0
    if leaving_ok:
        outside = np.flatnonzero(~inside)
        inside = inside[outside[0]:] if len(outside) else inside[:0]
    return bool(np.any(inside))


def check_path(points, buoys, clearance=0.5, heading_slack=0.1, obstacles=(), obstacle_radius=0.3, tie=0.05,
               beam_margin=0.15):
    """Judge a polyline before it is driven.

    Returns dict(illegal, look, tight): index of the first segment with a
    wrong-side event of the rule oracle (or None), {segment index: ids of
    unclassified buoys that would come abeam}, and index of the first segment
    closer than `clearance` to a buoy surface or an obstacle point (or None).

    A wrong-side buoy counts even when another buoy on that side is up to `tie` metres
    nearer when it comes abeam: the vessel's guard (Task1Mission.rule_risk) uses a smaller
    tie, so a leg that passes here is not stopped there, and the judge's 1 cm tie never
    decides a passage that was planned.

    Each leg is judged from `beam_margin` metres behind its start, like the guard does: a
    buoy lying only centimetres behind the beam where a leg starts counts as coming abeam.
    """
    things = [(q['x'], q['y'], q['radius']) for q in buoys]+[(o[0], o[1], obstacle_radius) for o in obstacles]
    centers = np.array([t[:2] for t in things], float).reshape(-1, 2)
    radii = np.array([t[2] for t in things], float)
    result = dict(illegal=None, look={}, tight=None)
    points = [np.asarray(q, float) for q in points]
    points = [q for i, q in enumerate(points) if i == 0 or np.linalg.norm(q-points[i-1]) >= 1e-6]
    for index, (a, b) in enumerate(zip(points, points[1:])):
        yaw = math.atan2(b[1]-a[1], b[0]-a[0])
        # A vessel that already is inside the margin of a buoy may leave it, but not close in further.
        if result['tight'] is None and len(centers):
            ab = b-a
            t = np.clip(((centers-a) @ ab)/float(ab @ ab), 0.0, 1.0)
            gap = np.linalg.norm(centers-(a+t[:, None]*ab), axis=1)
            limit = radii+clearance
            if index == 0:
                limit = np.minimum(limit, np.linalg.norm(centers-a, axis=1)-0.05)
            if np.any(gap < limit):
                result['tight'] = index
        # The driven heading differs from the leg by a few degrees. A buoy almost dead ahead
        # then switches sides and stops shielding a wrong-side buoy behind it, so the leg
        # is also judged with the heading turned by +-heading_slack.
        back = a-beam_margin*(b-a)/max(float(np.linalg.norm(b-a)), 1e-9)
        for assumed, slack in ((s, d) for s in (RED, GREEN) for d in (0.0, -heading_slack, heading_slack)):
            judged = [dict(q, state=assumed if q['state'] == UNKNOWN else q['state']) for q in buoys]
            for event in passage_events([back[0], back[1], yaw+slack], [b[0], b[1], yaw+slack], judged, tie=tie):
                original = next(q for q in buoys if q['id'] == event['id'])
                right = 'starboard' if event['state'] == RED else 'port'
                if original['state'] == UNKNOWN:
                    if not event['ambiguous'] or event['side'] != right:
                        result['look'].setdefault(index, set()).add(event['id'])
                elif event['side'] != right and result['illegal'] is None:
                    result['illegal'] = index
    return result


def plan(buoys, start, goal, **options):
    """Plan from start to goal through legal gaps.

    buoys: dicts with id, x, y, state and optional radius. Returns None if no
    legal route exists, else dict(waypoints, gaps, look, length):
      waypoints: dicts(x, y, kind in escape|align|mid|through|goal, gap index or None)
      gaps: dicts(port, starboard, width, narrow, mid, normal)
      look: ids of unclassified buoys that must be classified before the first
            leg that would bring them abeam (empty: the next leg may be driven)
      look_leg: waypoint index of that leg, or None.
    """
    p = dict(DEFAULTS, **options)
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    real = [dict(b, radius=b.get('radius') or p['default_radius']) for b in buoys]
    solid = np.array([[float(o[0]), float(o[1])] for o in p['obstacles']]).reshape(-1, 2)
    if len(real)+len(solid) < 1:
        return dict(waypoints=[dict(x=float(goal[0]), y=float(goal[1]), kind='goal', gap=None)],
                    gaps=[], look=[], look_leg=None, length=float(np.linalg.norm(goal-start)), entered_field=False)
    centers = np.array([[b['x'], b['y']] for b in real]).reshape(-1, 2)
    ring = _ring(np.vstack([centers, solid, start, goal]), p['ring_margin'], p['ring_spacing'])
    nodes = np.vstack([centers, solid, ring])
    count, inner = len(real), len(real)+len(solid)  # Buoys, then obstacle points, then the ring.
    radius = [b['radius'] for b in real]+[p['obstacle_radius']]*len(solid)
    mesh = Delaunay(nodes)
    hull = field_hull(centers) if p['outside'] else None
    first, last = int(mesh.find_simplex(start)), int(mesh.find_simplex(goal))
    if first < 0 or last < 0:
        return None

    def crossing(triangle, k):
        """Leave `triangle` through the edge opposite its vertex k."""
        other = int(mesh.neighbors[triangle][k])
        if other < 0:
            return None
        i, j = [int(v) for n, v in enumerate(mesh.simplices[triangle]) if n != k]
        radii = sum(radius[v] for v in (i, j) if v < inner)
        width = float(np.linalg.norm(nodes[i]-nodes[j]))-radii
        outside = int(i >= inner)+int(j >= inner)
        mid = (nodes[i]+nodes[j])/2
        normal = np.array([-(nodes[j]-nodes[i])[1], (nodes[j]-nodes[i])[0]])
        normal /= np.linalg.norm(normal)
        if normal @ (nodes[mesh.simplices[triangle][k]]-mid) > 0:
            normal = -normal  # Points away from the triangle being left.
        left = np.array([-normal[1], normal[0]])
        port, starboard = (i, j) if (nodes[i]-mid) @ left > 0 else (j, i)
        states = [real[v]['state'] if v < count else OFF for v in (port, starboard)]
        # A red end to port or a green end to starboard forbids the crossing. Next to the start
        # or the goal, however, the route may cross an edge far from its ends (a goal just
        # beyond the line between two buoys): there it is only made expensive, and check_path
        # decides whether the buoy really comes abeam.
        wrong_colour = states[0] == RED or states[1] == GREEN
        legal = (width >= p['min_width'] and (not wrong_colour or triangle == first or other == last)
                 and not (port < count and starboard < count and
                          frozenset((real[port]['id'], real[starboard]['id'])) in avoided))
        # Into the field: between two buoys, or into a triangle lying inside the buoy hull
        # (near a boundary the wall points replace some of the buoy-buoy edges).
        field = (i < count and j < count) or in_field(hull, nodes[mesh.simplices[other]].mean(axis=0))
        return dict(key=(triangle, k), to=other, mid=mid, normal=normal, width=width, legal=legal, field=field,
                    port=real[port]['id'] if port < count else None,
                    starboard=real[starboard]['id'] if starboard < count else None,
                    extra=p['ring_penalty']*outside+p['unknown_penalty']*states.count(UNKNOWN)
                    + p['colour_penalty']*wrong_colour + (p['outside_penalty'] if p['outside'] and field else 0.0))

    banned, avoided = set(), {frozenset(pair) for pair in p['avoid']}
    for _ in range(p['max_replans']):
        # Dijkstra over directed edge crossings; cost is the distance between gap middles.
        queue, best, parent, final = [(0., 0, None, first, start)], {}, {}, None
        serial = 1
        while queue:
            cost, _, key, triangle, point = heapq.heappop(queue)
            if key is not None and best.get(key, float('inf')) < cost:
                continue
            if triangle == last:
                final = key
                break
            for k in range(3):
                step = crossing(triangle, k)
                if step is None or not step['legal'] or step['key'] in banned:
                    continue
                total = cost+float(np.linalg.norm(step['mid']-point))+step['extra']
                if total < best.get(step['key'], float('inf')):
                    best[step['key']] = total
                    parent[step['key']] = (key, step)
                    heapq.heappush(queue, (total, serial, step['key'], step['to'], step['mid']))
                    serial += 1
        else:
            return None
        steps = []
        while final is not None:
            final, step = parent[final]
            steps.append(step)
        steps.reverse()
        for step in steps:
            step['narrow'] = step['width'] < p['wide_width']

        def layout(keep_wide, scale=1.0):
            """Waypoints for the gap sequence; wide gaps only add a steering point."""
            waypoints, owner, previous = [], [], start
            kept = [i for i, s in enumerate(steps) if s['narrow'] or keep_wide]
            for n, index in enumerate(kept):
                step = steps[index]
                following = steps[kept[n+1]]['mid'] if n+1 < len(kept) else goal
                points = [('mid', step['mid'])]
                if step['narrow']:
                    before = min(scale*p['align_distance'], 0.45*float(np.linalg.norm(step['mid']-previous)))
                    after = min(scale*p['through_distance'], 0.45*float(np.linalg.norm(following-step['mid'])))
                    points = [('align', step['mid']-before*step['normal']), ('mid', step['mid']),
                              ('through', step['mid']+after*step['normal'])]
                for kind, point in points:
                    waypoints.append(dict(x=float(point[0]), y=float(point[1]), kind=kind, gap=index))
                    owner.append(index)
                previous = np.array([waypoints[-1]['x'], waypoints[-1]['y']])
            waypoints.append(dict(x=float(goal[0]), y=float(goal[1]), kind='goal', gap=None))
            owner.append(len(steps)-1)
            path = [start]+[np.array([w['x'], w['y']]) for w in waypoints]
            verdict = check_path(path, real, p['clearance'], obstacles=solid,
                                 obstacle_radius=p['obstacle_radius'])
            bad = [v for v in (verdict['illegal'], verdict['tight']) if v is not None]
            return waypoints, owner, path, verdict, bad

        # Prefer the direct line between narrow gaps: the middles of wide gaps are
        # only artifacts of the triangulation. Keep them if the direct line is not legal.
        # Shorter alignment and pass-through distances are tried before a gap is given up:
        # in a dense field the full distances can run into the next buoy.
        tried = []
        # In outside mode a smooth path (wide gap middles dropped) must not cut across the buoy
        # field between two distant waypoints; then every gap middle is kept, so that the path
        # stays in the triangles it crosses.
        stays_out = p['outside'] and not any(s['field'] for s in steps)
        for keep_wide, scale in ((False, 1.0), (True, 1.0), (False, 0.5), (True, 0.5)):
            waypoints, owner, path, verdict, bad = layout(keep_wide, scale)
            if not bad and stays_out and not keep_wide and _enters(path, centers):
                bad = [0]
            tried.append((waypoints, owner, path, verdict, bad))
            if not bad:
                break
        if bad and all(min(t[4]) <= 1 for t in tried) and not p['_escaping']:
            # Only the first legs fail (the first waypoint may lie right next to the start): the vessel sits where any move towards the first waypoint
            # brings a buoy abeam on the wrong side (e.g. level with a red buoy to port). A short
            # legal move in another direction is tried, and the route planned again from there.
            candidates = [start+d*np.array([math.cos(k*math.pi/4), math.sin(k*math.pi/4)])
                          for d in (1.0, 2.0) for k in range(8)]
            # Towards the goal first, and never back to where an earlier escape started from
            # (that would let the vessel shuttle between two escape points for ever).
            candidates = [e for e in sorted(candidates, key=lambda e: float(np.linalg.norm(goal-e)))
                          if all(math.dist(e, f) > 0.75 for f in p['forbid'])
                          and not (p['outside'] and in_field(hull, e))]  # Outside mode: not into the field.
            for escape in candidates:
                distance = float(np.linalg.norm(escape-start))
                first_leg = check_path([start, escape], real, p['clearance'], obstacles=solid,
                                       obstacle_radius=p['obstacle_radius'])
                if first_leg['illegal'] is not None or first_leg['tight'] is not None or first_leg['look']:
                    continue
                rest = plan(buoys, escape, goal, **dict(options, _escaping=True))
                if rest is not None:
                    rest['waypoints'].insert(0, dict(x=float(escape[0]), y=float(escape[1]), kind='escape', gap=None))
                    rest['length'] += distance
                    if rest['look_leg'] is not None:
                        rest['look_leg'] += 1
                    return rest
        if bad:
            if not steps:
                return None  # The direct line itself is illegal and there is no gap to change.
            banned.add(steps[max(0, owner[min(bad)])]['key'])
            continue
        leg = min(verdict['look']) if verdict['look'] else None
        length = sum(float(np.linalg.norm(b-a)) for a, b in zip(path, path[1:]))
        return dict(
            waypoints=waypoints, length=length, look_leg=leg,
            look=sorted(verdict['look'][leg]) if leg is not None else [],
            entered_field=any(s['field'] for s in steps),
            gaps=[dict(port=s['port'], starboard=s['starboard'], width=round(s['width'], 3),
                       narrow=s['narrow'], mid=[float(v) for v in s['mid']],
                       normal=[float(v) for v in s['normal']]) for s in steps])
    return None


def wall_points(polygon, spacing=0.5):
    """Points every `spacing` metres along a closed polygon (the course boundary).

    Given to plan() and check_path() as obstacles, they form a wall that the
    route cannot cross: neighbouring points are closer than the minimum width.
    """
    points = []
    for a, b in zip(polygon, list(polygon[1:])+list(polygon[:1])):
        a, b = np.asarray(a, float), np.asarray(b, float)
        steps = max(1, int(math.ceil(np.linalg.norm(b-a)/spacing)))
        points += [tuple(a+(b-a)*i/steps) for i in range(steps)]
    return points


def exploration_goal(buoys, entry, margin=3.0):
    """Goal while the exit is unknown: the far side of the field seen from the entry."""
    others = np.array([[b['x'], b['y']] for b in buoys if b['id'] != entry['id']])
    origin = np.array([entry['x'], entry['y']])
    if not len(others):
        return None
    direction = others.mean(axis=0)-origin
    if np.linalg.norm(direction) < 1e-6:
        return None
    direction /= np.linalg.norm(direction)
    return origin+direction*(float(((others-origin) @ direction).max())+margin)
