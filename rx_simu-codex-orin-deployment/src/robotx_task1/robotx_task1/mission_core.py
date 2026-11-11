"""USV-only Safe Passage mission logic (no ROS imports).

One step() per control cycle: vessel pose, the current landmark list and input
health in; forward speed, yaw rate, state and events out. The vessel discovers
buoys as it goes, so every decision is made on the partial map and repeated
when the map changes. An unclassified buoy is never brought abeam: the vessel
stops and looks at it first.

Order, from the 2026 handbook (Task 1 Core):
1. Find the flashing blue ENTRY buoy without entering the buoy field (outside
   mode of the planner; passages before the entry circle do not score anyway).
2. Circle it clockwise. Only now does the attempt score.
3. Earn "green only to port" and "red only to starboard": at least one of each,
   preferably through one red/green gate, never a wrong side.
4. Classify every remaining buoy, so that all red and green ones can be reported.
5. Circle the steady blue EXIT buoy counter-clockwise.
"""
import math

import numpy as np

from . import gap_planner as gp
from . import track_control as tc
from .core import CircleTracker, passage_events
from .landmark_map import inside_polygon, shrink_polygon

DEFAULTS = dict(
    cruise_speed=0.3, thread_speed=0.25, circle_speed=0.25, max_yaw_rate=0.3, search_yaw_rate=0.15,
    circle_radii=(1.3, 1.5, 1.1, 0.95),  # Tried in this order; from the buoy center [m]. The
                                         # smallest is above the 0.83 m turning radius at circle_speed.
    circle_margin=0.15,            # Extra angle beyond one full turn [rad].
    arrive_tolerance=0.4, turn_point_tolerance=0.12, align_tolerance=0.15, align_heading=math.radians(5.0),
    look_range=4.0,                # Must not exceed the measured reliable color range [m].
    look_min_range=2.0, look_timeout=10.0, look_attempts=3, heading_tolerance=0.1,
    search_step=3.0, total_timeout=1200.0, blocked_timeout=8.0, replan_interval=2.0, planner={},
    unreachable_retry=60.0,  # A buoy that could not be approached is tried again after this [s]
    unreachable_moved=6.0,   # and once the vessel is this far from where it failed [m].
    stand_off_tries=4,  # Stand-off points tried around a buoy before it is reported unreachable.
    gate_max_spacing=6.0,  # Red/green pairs farther apart are not used as a scoring gate [m].
    gate_align=2.0, gate_through=1.5,  # Alignment and pass-through distances of a scoring gate [m].
    survey_until=0.75,  # Share of total_timeout after which classifying the rest is skipped.
    patrol_inset=2.0,   # Distance of the entry-search patrol from the boundary [m].
    boundary=None)  # Course boundary polygon (the lettered buoys) in the mission frame.

TERMINAL = ('COMPLETE', 'ABORT')


class Mission:
    def __init__(self, **options):
        unknown = set(options)-set(DEFAULTS)
        if unknown:
            raise ValueError(f'Unknown mission options: {sorted(unknown)}')
        self.p = dict(DEFAULTS, **options)
        self.state, self.detail = 'WAIT_START', 'waiting for the start request'
        self.requested, self.started_at = False, None
        self.entry_done, self.entry_id = False, None
        self.events, self.assumed_off, self.look_failures = [], set(), {}
        self.passed, self.announced, self.crossed, self.explore = [], set(), set(), None
        self.guarded, self.obstacles, self.obstacle_count = 0, [], 0
        self.set_boundary(self.p['boundary'])
        self.retry_at, self.unreachable, self.clock = -float('inf'), {}, 0.0
        self.yaw_fix = None
        self.leg_next = None  # (leg, the waypoint after it) while following a planned route.
        self.reached = None  # The last waypoint arrived at.
        self.previous = None  # dict(route, next waypoint index, goal) of the route being followed.
        self.escapes = []  # Where the last escape steps started.
        self.last_fix, self.centre_reached = None, False
        self.far_tries = {}
        self.credit = dict(green=False, red=False)  # Earned after the entry circle.
        self.tried_gates, self.no_gate_signature, self.prev_pose = set(), None, None
        self.patrol, self.patrolled, self.field_entered = None, set(), False
        self.patrol_retry_at = -float('inf')
        self.reset_motion()

    def reset_motion(self):
        self.leg = self.gap = self.look = self.ring = self.look_target = self.gate_target = None
        self.look_chain = []
        self.spin, self.last_yaw, self.signature, self.blocked_since = 0.0, None, None, None

    def start(self):
        """Operator request. Motion still needs fresh inputs and an armed GUIDED FCU."""
        self.requested = True

    def emit(self, kind, **data):
        self.events.append(dict(type=kind, **data))

    # ------------------------------------------------------------------ step
    def step(self, now, pose, buoys, inputs_ok=True, mode='GUIDED', armed=True, rtk_ok=True, blocked=False,
             obstacles=()):
        """Returns dict(v, w, state, detail, events). Zero command unless actively driving.

        obstacles: (x, y) points of non-buoy objects (ObstacleGrid.points()).
        """
        self.events, self.clock = [], now
        v = w = 0.0
        guided = mode.upper() == 'GUIDED' and armed
        if self.state == 'WAIT_START':
            if self.requested and inputs_ok and guided:
                self.started_at = now
                self.set('SEARCH_ENTRY', 'looking for the flashing blue entry buoy')
                self.emit('started')
        elif self.state not in TERMINAL:
            if not guided:
                self.abort(f'FCU mode {mode}, armed={armed}: operator has control')
            elif now-self.started_at > self.p['total_timeout']:
                self.abort('total time limit reached')
            elif not inputs_ok:
                if self.state != 'HOLD':
                    self.set('HOLD', 'an input is stale')
                    self.emit('hold')
            else:
                if self.state == 'HOLD':
                    self.reset_motion()
                    self.set('PLAN' if self.entry_done else 'SEARCH_ENTRY', 'inputs recovered')
                    self.emit('resumed')
                self.buoys = [dict(b, state=gp.OFF if b['id'] in self.assumed_off and
                                   b['state'] == gp.UNKNOWN else b['state']) for b in buoys]
                self.rtk_ok = rtk_ok
                self.obstacles = list(obstacles)
                self.solid = self.obstacles+self.walls
                for state, event in ((gp.ENTRY, 'entry_found'), (gp.EXIT, 'exit_found')):
                    seen = self.find(state=state)
                    if seen is not None and event not in self.announced:
                        self.announced.add(event)
                        self.emit(event, id=seen['id'])
                self.track_credit(pose)
                rate = self.yaw_rate(now, pose)
                v, w = getattr(self, 'do_'+self.state.lower())(now, pose)
                if self.last_yaw is not None:
                    self.spin += abs(tc.wrap(pose[2]-self.last_yaw))
                self.last_yaw = pose[2]
                if v > 0.0 and not self.state.startswith('CIRCLE') and self.motion_risk(pose, v, w, rate):
                    # The motion actually commanded (turning while moving) differs from the
                    # checked straight leg: never sweep the beam over an unclassified or
                    # wrong-side buoy. Turning on the spot stays allowed.
                    v = 0.0
                    self.guarded += 1
                    if self.guarded*0.1 > self.p['blocked_timeout']:
                        self.guarded, self.leg, self.gap = 0, None, None
                        self.emit('rule_guard', state=self.state)
                        if self.state in ('ALIGN', 'TRANSIT', 'APPROACH_ENTRY', 'APPROACH_EXIT', 'THREAD'):
                            self.set('PLAN', 'motion would bring a buoy abeam on the wrong side; planning again')
                else:
                    self.guarded = 0
                # The hull keeps sliding for a moment after the command drops to zero. Turning
                # during that slide sweeps the beam as well, so the turn waits for the stop.
                drift = self.drift_speed(now, pose)
                if (w != 0.0 and drift > 0.05 and not self.state.startswith('CIRCLE')
                        and self.rule_risk(pose, max(v, drift), w) and not self.rule_risk(pose, max(v, drift), 0.0)):
                    w = 0.0
                if blocked and v > 0.0:
                    v = 0.0  # Safety box ahead is occupied: turning stays allowed.
                    self.blocked_since = self.blocked_since if self.blocked_since is not None else now
                    if now-self.blocked_since > self.p['blocked_timeout'] and self.state != 'THREAD':
                        self.emit('blocked', state=self.state)
                        self.blocked_since, self.leg, self.gap = None, None, None
                        if self.state in ('ALIGN', 'TRANSIT', 'APPROACH_ENTRY', 'APPROACH_EXIT'):
                            self.set('PLAN', 'path ahead blocked; planning again')
                else:
                    self.blocked_since = None
        if self.state in TERMINAL or self.state in ('HOLD', 'WAIT_START'):
            v = w = 0.0
        return dict(v=float(v), w=float(w), state=self.state, detail=self.detail, events=self.events)

    def track_credit(self, pose):
        """Score items earned by the vessel's own motion since the last cycle (map buoys)."""
        before, self.prev_pose = self.prev_pose, tuple(pose)
        if (before is None or not self.entry_done or self.state.startswith('CIRCLE')
                or math.dist(before[:2], pose[:2]) < 0.005):
            return
        known = [b for b in self.buoys if b['state'] != gp.UNKNOWN]
        for event in passage_events(list(before), list(pose), known):
            if event['ambiguous']:
                continue
            if not event['legal']:
                self.emit('wrong_side', id=event['id'], side=event['side'])
            elif event['state'] == gp.GREEN and not self.credit['green']:
                self.credit['green'] = True
                self.emit('credit', item='green_to_port', id=event['id'])
            elif event['state'] == gp.RED and not self.credit['red']:
                self.credit['red'] = True
                self.emit('credit', item='red_to_starboard', id=event['id'])

    def drift_speed(self, now, pose):
        """Speed over ground estimated from the last two poses [m/s]."""
        last, self.last_fix = self.last_fix, (now, pose[0], pose[1])
        if last is None or now-last[0] <= 0.0 or now-last[0] > 1.0:
            return 0.0
        return math.dist(pose[:2], last[1:])/(now-last[0])

    def yaw_rate(self, now, pose):
        """Turn rate over the last cycle [rad/s] (0 when unknown)."""
        last, self.yaw_fix = self.yaw_fix, (now, pose[2])
        if last is None or now-last[0] <= 0.0 or now-last[0] > 1.0:
            return 0.0
        return tc.wrap(pose[2]-last[1])/(now-last[0])

    def motion_risk(self, pose, v, w, rate):
        """rule_risk for the command as the hull will really answer it: right after a turn on
        the spot the hull is still turning (at `rate`) while the speed only builds up, so a
        buoy just behind the beam can swing abeam before the vessel moves on."""
        return any(self.rule_risk(pose, speed, turn) for speed, turn in
                   ((v, w), (0.2*v, w), (0.2*v, rate)) if speed > 0.0)

    def rule_risk(self, pose, v, w, horizon=1.0, steps=5, margin=0.15, tie=0.01):
        """True if the next `horizon` seconds of this command bring a buoy abeam that is
        unclassified (either colour assumed) or on the wrong side. A wrong-side buoy counts
        even when another buoy on that side is up to `tie` metres nearer: which of two
        almost equidistant buoys the judge sees as "the closest" must not be left to chance."""
        view = self.rule_view()
        judged = [[dict(b, state=assumed if b['state'] == gp.UNKNOWN else b['state']) for b in view]
                  for assumed in (gp.RED, gp.GREEN)]
        unknown = {b['id'] for b in view if b['state'] == gp.UNKNOWN}
        dt = horizon/steps
        # The prediction starts at, slightly behind and slightly ahead of the vessel: a buoy lying
        # right on the beam line (within the landmark position error) must count as about to
        # come abeam, both when moving past it and when a turn swings the beam back over it.
        for offset in (-margin, 0.0, margin):
            x, y, yaw = pose[0]+offset*math.cos(pose[2]), pose[1]+offset*math.sin(pose[2]), pose[2]
            for _ in range(steps):
                after = (x+v*dt*math.cos(yaw+w*dt/2), y+v*dt*math.sin(yaw+w*dt/2), yaw+w*dt)
                for buoys in judged:
                    for event in passage_events([x, y, yaw], list(after), buoys, tie=tie):
                        right = 'starboard' if event['state'] == gp.RED else 'port'
                        if event['id'] in unknown or event['side'] != right:
                            return True
                x, y, yaw = after
        return False

    def set(self, state, detail=''):
        self.state, self.detail = state, detail

    def abort(self, reason):
        self.set('ABORT', reason)
        self.emit('abort', reason=reason)

    def find(self, name=None, state=None):
        for buoy in self.buoys:
            if (name is not None and buoy['id'] == name) or (state is not None and buoy['state'] == state):
                return buoy
        return None

    def nearest_unknown(self, pose):
        # Not tried again from near the place where it could not be approached, nor soon.
        recent = {k for k, (t, where) in self.unreachable.items()
                  if self.clock-t < self.p['unreachable_retry'] or math.dist(pose[:2], where) < self.p['unreachable_moved']}
        unknown = [b for b in self.buoys if b['state'] == gp.UNKNOWN and b['id'] not in recent]
        # Buoys that could not be approached go to the back of the queue, so that every
        # unclassified buoy gets its turn instead of the two nearest ones alternating.
        return min(unknown, key=lambda b: (self.far_tries.get(b['id'], 0), math.dist(pose[:2], (b['x'], b['y']))),
                   default=None)

    # ---------------------------------------------------------------- search
    def do_search_entry(self, now, pose):
        return self.search(now, pose, gp.ENTRY, 'entry_found', 'entry')

    def do_search_exit(self, now, pose):
        return self.search(now, pose, gp.EXIT, 'exit_found', 'exit')

    def search(self, now, pose, state, event, phase):
        target = self.find(state=state)
        if target is not None:
            self.reset_motion()
            self.set('PLAN', f'{phase} buoy {target["id"]} identified')
            return 0.0, 0.0
        if self.leg is not None and self.map_changed(positions=True):
            # A buoy or obstacle found on the way: the rest of the leg is checked again.
            verdict = self.check([pose[:2], self.leg[1]])
            if verdict['illegal'] is not None or verdict['tight'] is not None or verdict['look']:
                self.leg, self.spin = None, 0.0
                self.detail = 'search leg no longer clear'
                return 0.0, 0.0
            self.remember_map()
        if self.leg is not None:  # Move to a new vantage point.
            v, w, _, _, remaining = tc.follow(pose, self.leg[0], self.leg[1], self.p['cruise_speed'],
                                              self.p['max_yaw_rate'])
            if remaining <= self.p['arrive_tolerance']:
                self.leg, self.spin = None, 0.0
            return v, w
        # The LiDAR sees all round; turning on the spot only helps the camera, so it is done
        # only when an unclassified buoy is within the colour range from here.
        nearby = any(b['state'] == gp.UNKNOWN and math.dist(pose[:2], (b['x'], b['y'])) <= self.look_distance(b['id'])
                     for b in self.buoys)
        if self.spin < 2*math.pi+0.5 and nearby:
            self.detail = f'turning to find the {phase} buoy'
            return 0.0, self.p['search_yaw_rate']
        unknown = self.nearest_unknown(pose)
        self.spin = 0.0
        if unknown is not None:
            return self.want_look([unknown['id']], pose)
        # Nothing to look at within reach. Before the entry circle the field may not be entered:
        # patrol inside the boundary, around the buoys. Afterwards head for the course middle.
        if self.boundary and not self.entry_done and (now >= self.patrol_retry_at or self.map_changed(False)):
            self.remember_map()
            self.patrol_retry_at = now+self.p['replan_interval']  # Until a route is found.
            for point in self.patrol_points(pose):
                route = self.plan_route(pose[:2], point, avoid=False)
                if route is not None and not route['entered_field'] and not (route['look'] and route['look_leg'] == 0):
                    first = route['waypoints'][0]
                    self.leg = (tuple(pose[:2]), (first['x'], first['y']))
                    self.remember_map()
                    self.detail = f'patrolling outside the field to find the {phase} buoy'
                    return 0.0, 0.0
        if self.boundary and not self.centre_reached and self.entry_done:
            centre = tuple(np.mean(np.array(self.boundary), axis=0))
            if math.dist(pose[:2], centre) <= 2.0:
                self.centre_reached = True
            else:
                route = self.plan_route(pose[:2], centre, avoid=False)
                if route is not None and not (route['look'] and route['look_leg'] == 0):
                    first = route['waypoints'][0]
                    self.leg = (tuple(pose[:2]), (first['x'], first['y']))
                    self.remember_map()
                    self.detail = f'heading for the course centre to find the {phase} buoy'
                    return 0.0, 0.0
        # Otherwise advance along the current heading, if that is legal.
        ahead = (pose[0]+self.p['search_step']*math.cos(pose[2]), pose[1]+self.p['search_step']*math.sin(pose[2]))
        verdict = self.check([pose[:2], ahead])
        into_field = not self.entry_done and gp._enters([pose[:2], ahead], np.array(
            [(b['x'], b['y']) for b in self.buoys]).reshape(-1, 2))
        if verdict['illegal'] is None and verdict['tight'] is None and not verdict['look'] and not into_field:
            self.leg = (tuple(pose[:2]), ahead)
            self.remember_map()
        self.emit(f'{phase}_not_found')
        return 0.0, 0.0

    def patrol_points(self, pose):
        """Patrol points inside the boundary, next one first; visited ones are skipped."""
        if self.patrol is None:
            inner = shrink_polygon(self.boundary, self.p['patrol_inset'])
            self.patrol = [q for a, b in zip(inner, inner[1:]+inner[:1]) for q in (a, tuple((np.array(a)+b)/2))]
        for i, q in enumerate(self.patrol):
            if math.dist(pose[:2], q) <= 2.0:
                self.patrolled.add(i)
        if len(self.patrolled) == len(self.patrol):
            self.patrolled = set()  # A second round: buoys seen late may have moved the map.
        left = [i for i in range(len(self.patrol)) if i not in self.patrolled]
        return [self.patrol[i] for i in sorted(left, key=lambda i: math.dist(pose[:2], self.patrol[i]))]

    # ------------------------------------------------------------------ look
    def want_look(self, names, pose):
        """Make the nearest of these unclassified buoys the next thing to look at."""
        found = [self.find(name=n) for n in names]
        nearest = min((b for b in found if b is not None), key=lambda b: math.dist(pose[:2], (b['x'], b['y'])))
        self.look_target, self.leg, self.gap = nearest['id'], None, None
        self.look_chain.append(nearest['id'])
        self.set('PLAN', f'{nearest["id"]} must be classified first')
        return 0.0, 0.0

    def stand_offs(self, buoy, distance, pose, bearings=12):
        """Clear points `distance` from a buoy to look at it from, nearest to the vessel first."""
        points = []
        for k in range(bearings):
            angle = 2*math.pi*k/bearings
            point = (buoy['x']+distance*math.cos(angle), buoy['y']+distance*math.sin(angle))
            clear = all(math.dist(point, (b['x'], b['y'])) >= b.get('radius', 0.2)+0.6
                        for b in self.buoys if b['id'] != buoy['id'])
            clear = clear and all(math.dist(point, o) >= 0.9 for o in self.solid)
            if not self.entry_done:
                # Before the entry circle only from outside the buoy field.
                clear = clear and not gp.in_field(gp.field_hull([(b['x'], b['y']) for b in self.buoys]), point)
            if clear and self.inside(point, margin=0.8):
                points.append(point)
        return sorted(points, key=lambda q: math.dist(pose[:2], q))

    def look_distance(self, name):
        """Stand-off for a look; one metre closer after every failed attempt."""
        return max(self.p['look_min_range'], self.p['look_range']-self.look_failures.get(name, 0))

    def do_look(self, now, pose):
        """Face the buoy and wait for the map to classify it."""
        buoy = self.find(name=self.look_target)
        if buoy is None or buoy['state'] != gp.UNKNOWN:
            self.look_target, self.look_chain, self.spin = None, [], 0.0
            self.set('PLAN', 'classification finished')
            return 0.0, 0.0
        self.detail = f'classifying {buoy["id"]}'
        error = tc.wrap(math.atan2(buoy['y']-pose[1], buoy['x']-pose[0])-pose[2])
        if self.look['deadline'] is None and abs(error) < self.p['heading_tolerance']:
            self.look['deadline'] = now+self.p['look_timeout']
        if self.look['deadline'] is not None and now > self.look['deadline']:
            if math.dist(pose[:2], (buoy['x'], buoy['y'])) > self.look_distance(buoy['id'])+0.5:
                # Seen from too far away: no evidence that the lamp is off. Try other buoys
                # first and come back when the map or the position has changed.
                self.unreachable[buoy['id']] = (now, tuple(pose[:2]))
                self.far_tries[buoy['id']] = self.far_tries.get(buoy['id'], 0)+1
                self.look_target, self.look_chain = None, []
                self.emit('look_too_far', id=buoy['id'])
                self.set('PLAN', f'{buoy["id"]} not reachable for a close look yet')
                return 0.0, 0.0
            failures = self.look_failures[buoy['id']] = self.look_failures.get(buoy['id'], 0)+1
            if failures >= self.p['look_attempts']:
                # Never seen lit from close range: handled as an unlit obstacle from now on.
                self.assumed_off.add(buoy['id'])
                self.emit('assumed_off', id=buoy['id'])
                self.look_target, self.look_chain = None, []
            self.set('PLAN', 'not classified in time; trying again')
            return 0.0, 0.0
        return tc.turn(error, self.p['max_yaw_rate'])

    # ------------------------------------------------------------------ plan
    def circle_start(self, pose, buoy, direction):
        """Nearest point of the first legal, clear circle around a blue buoy."""
        center = (buoy['x'], buoy['y'])
        angle = math.atan2(pose[1]-center[1], pose[0]-center[0])
        others = [b for b in self.buoys if b['id'] != buoy['id']]
        # Only clearance is checked here. While circling, every buoy outside the circle comes
        # abeam on the outer side once, whatever the radius; whether the judges count that as a
        # passage is an open rule question that cannot be solved by choosing a radius.
        choice = next((radius for radius in self.p['circle_radii'] if gp.check_path(
            tc.circle_polygon(center, radius, direction, angle), others, obstacles=self.solid)['tight'] is None), None)
        if choice is None:
            # No radius keeps the full margin: take the one that keeps the most.
            things = [((b['x'], b['y']), b.get('radius', 0.2)) for b in others]+[(o, 0.3) for o in self.solid]

            def margin(radius):
                ring = tc.circle_polygon(center, radius, direction, angle, count=36)
                return min((math.dist(q, c)-r for q in ring for c, r in things), default=float('inf'))
            choice = max(self.p['circle_radii'], key=margin)
            self.emit('circle_not_clear', id=buoy['id'], radius=choice)
        return dict(id=buoy['id'], center=center, radius=choice, direction=direction,
                    point=(center[0]+choice*math.cos(angle), center[1]+choice*math.sin(angle)))

    def scoring_gate(self, pose):
        """A red/green gate that earns a missing score item: green to port, red to starboard.

        Both ends are checked (two reds or two greens never form a gate); the best gate earns
        both items, then the nearest one. A gate that leads into a dead end (no legal way
        from its far side back to its near side without crossing it again) is skipped.
        Returns dict(port, starboard, align, mid, through, gives) or None.
        """
        known = [b for b in self.buoys if b['state'] != gp.UNKNOWN]
        candidates = []
        for port in known:
            for starboard in known:
                if port is starboard or port['state'] == gp.RED or starboard['state'] == gp.GREEN:
                    continue
                gives = ((port['state'] == gp.GREEN and not self.credit['green'])
                         + (starboard['state'] == gp.RED and not self.credit['red']))
                pair = (port['id'], starboard['id'])
                if not gives or pair in self.tried_gates:
                    continue
                a, b = np.array([port['x'], port['y']]), np.array([starboard['x'], starboard['y']])
                spacing = float(np.linalg.norm(a-b))
                if (spacing > self.p['gate_max_spacing'] or
                        spacing-port.get('radius', 0.2)-starboard.get('radius', 0.2) < gp.DEFAULTS['min_width']):
                    continue
                normal = np.array([(a-b)[1], -(a-b)[0]])/spacing  # Port buoy on the left.
                mid = (a+b)/2
                align, through = mid-self.p['gate_align']*normal, mid+self.p['gate_through']*normal
                if not (self.inside(tuple(align), 0.8) and self.inside(tuple(through), 0.8)):
                    continue
                verdict = self.check([align, mid, through])
                if verdict['illegal'] is not None or verdict['tight'] is not None or verdict['look']:
                    continue
                candidates.append(((-gives, math.dist(pose[:2], align)), dict(
                    port=port['id'], starboard=starboard['id'], align=tuple(align), mid=tuple(mid),
                    through=tuple(through), normal=normal, gives=gives)))
        for _, gate in sorted(candidates, key=lambda c: c[0]):
            if self.way_back(gate):
                return gate
        return None

    def way_back(self, gate):
        """True if a legal route leads from the far side of `gate` back to its near side
        without crossing it again (e.g. not into a pocket between buoys and the platform)."""
        avoid = set(self.crossed) | {frozenset((gate['port'], gate['starboard']))}
        if gp.plan(self.rule_view(), gate['through'], gate['align'], **dict(
                self.p['planner'], avoid=avoid, obstacles=self.solid, outside=not self.entry_done)) is not None:
            return True
        crossed, self.crossed = self.crossed, avoid  # The short detour legs count as well.
        try:
            return self.detour((*gate['through'], 0.0), gate['align']) is not None
        finally:
            self.crossed = crossed

    def wants_survey(self):
        """After the score items: classify the rest, unless time is running short."""
        return (self.entry_done and (all(self.credit.values()) or self.no_gate_signature is not None)
                and self.clock-self.started_at < self.p['survey_until']*self.p['total_timeout'])

    def goal(self, pose):
        """(goal point, circle to start on arrival or None, state name while approaching)."""
        if not self.entry_done:
            ring = self.circle_start(pose, self.find(state=gp.ENTRY), -1)
            return ring['point'], ring, 'APPROACH_ENTRY'
        if not all(self.credit.values()):
            signature = {(b['id'], b['state']) for b in self.buoys}
            if self.no_gate_signature != signature:
                gate = self.scoring_gate(pose)
                if gate is not None:
                    self.gate_target, self.no_gate_signature = gate, None
                    return gate['align'], None, 'GATE'
                # No usable gate with this map; asked again when the map changes.
                self.no_gate_signature = signature
                self.emit('no_scoring_gate')
        exit_buoy = self.find(state=gp.EXIT)
        if exit_buoy is not None:
            ring = self.circle_start(pose, exit_buoy, +1)
            return ring['point'], ring, 'APPROACH_EXIT'
        if self.explore is None:
            entry = self.find(name=self.entry_id) or dict(id=self.entry_id, x=pose[0], y=pose[1])
            far = gp.exploration_goal(self.buoys, entry)
            # Pulled back towards the entry until it lies inside the course.
            while far is not None and not self.inside(far):
                back = np.array([entry['x'], entry['y']])-far
                if np.linalg.norm(back) < 0.5:
                    far = None
                    break
                far = far+0.25*back/np.linalg.norm(back)
            # Fixed once chosen: a goal that follows every map update never gets reached.
            self.explore = None if far is None else tuple(far)
        return self.explore, None, 'TRANSIT'

    def set_boundary(self, polygon):
        """Course boundary; the route keeps clearance from it like from an obstacle."""
        self.boundary = None if polygon is None else [tuple(map(float, p)) for p in polygon]
        self.walls = gp.wall_points(self.boundary) if self.boundary else []
        self.solid = self.obstacles+self.walls

    def plan_route(self, start, goal, avoid=True, may_enter=False):
        """gap_planner.plan with this mission's obstacles, memory and phase.

        Before the entry circle a route into the buoy field is refused, except towards the
        entry buoy itself (may_enter) when no way round exists.
        """
        route = gp.plan(self.rule_view(), start, goal, **dict(
            self.p['planner'], avoid=self.crossed if avoid else (), obstacles=self.solid, forbid=self.escapes,
            outside=not self.entry_done))
        if route is not None and not self.entry_done:
            # Geometric check of the polyline actually driven: the planner's own flag can miss a
            # field entry near the boundary, where wall points replace buoy-to-buoy edges.
            path = [tuple(start)]+[(w['x'], w['y']) for w in route['waypoints']]
            # Decided on the driven polyline alone; leaving the field is always allowed.
            route['entered_field'] = gp._enters(path, np.array([(b['x'], b['y']) for b in self.buoys]).reshape(-1, 2))
        if route is not None and route.get('entered_field') and not self.entry_done and not may_enter:
            return None
        if route is not None and route.get('entered_field') and not self.entry_done and not self.field_entered:
            # Only when no way around exists, e.g. an entry buoy inside the field.
            self.field_entered = True
            self.emit('field_entered_before_entry')
        return route

    def continued(self, pose, goal):
        """The rest of the route being followed, if the vessel has just reached its waypoint,
        the goal and the map are unchanged and the rest is clear from here; else None."""
        previous = self.previous
        if (previous is None or not self.entry_done or self.reached is None
                or math.dist(pose[:2], self.reached) > self.p['arrive_tolerance']
                or math.dist(goal, previous['goal']) > 1.0):
            return None
        saved = self.signature, self.obstacle_count
        self.signature, self.obstacle_count = previous['signature'], previous['obstacles']
        changed = self.map_changed(positions=True)
        self.signature, self.obstacle_count = saved
        if changed:
            return None
        rest = previous['route']['waypoints'][previous['next']:]
        if not rest:
            return None
        verdict = self.check([pose[:2]]+[(w['x'], w['y']) for w in rest])
        if verdict['illegal'] is not None or verdict['tight'] is not None or 0 in verdict['look']:
            return None
        # Unclassified buoys further on are looked at when the vessel gets there (do_plan).
        leg = min(verdict['look'], default=None)
        return dict(previous['route'], waypoints=rest, look=sorted(verdict['look'].get(leg, ())), look_leg=leg)

    def detour(self, pose, goal, lengths=(1.5, 2.5), directions=16):
        """A short straight leg that is clear by itself, from whose end a route leads on.

        The gap planner only steers through the middle of triangle edges. Next to a wide
        object (the platform) every such crossing may bring a buoy abeam on the wrong side,
        while a leg at an angle (keeping that buoy behind the beam) is legal.
        """
        bearing = math.atan2(goal[1]-pose[1], goal[0]-pose[0])
        turns = sorted((2*math.pi*k/directions for k in range(directions)),
                       key=lambda a: abs(tc.wrap(a-bearing)))
        for length in lengths:
            for angle in turns:
                end = (pose[0]+length*math.cos(angle), pose[1]+length*math.sin(angle))
                if not self.inside(end, 0.8):
                    continue
                verdict = self.check([pose[:2], end])
                if verdict['illegal'] is not None or verdict['tight'] is not None or verdict['look']:
                    continue
                route = self.plan_route(end, goal)
                if route is None:
                    continue
                route['waypoints'] = [dict(x=end[0], y=end[1], kind='escape', gap=None)]+route['waypoints']
                if route.get('look_leg') is not None:
                    route['look_leg'] += 1
                return route
        return None

    def check(self, points):
        return gp.check_path(points, self.rule_view(), obstacles=self.solid)

    def rule_view(self):
        """Buoys as the side rules see them. Passages before the entry circle do not score
        (handbook: "after circling the ENTRY ..."), so until then every buoy is treated as
        unlit: only clearance and staying out of the field matter."""
        if self.entry_done:
            return self.buoys
        return [dict(b, state=gp.OFF) for b in self.buoys]

    def inside(self, point, margin=1.0):
        return self.boundary is None or inside_polygon(point, shrink_polygon(self.boundary, margin))

    def remember_map(self):
        self.signature = {b['id']: (b['state'], b['x'], b['y']) for b in self.buoys}
        self.obstacle_count = len(self.obstacles)

    def map_changed(self, positions):
        """New landmark or obstacle cell, changed lamp state or (optionally) a landmark moved by 0.3 m."""
        if set(self.signature) != {b['id'] for b in self.buoys} or len(self.obstacles) != self.obstacle_count:
            return True
        return any(self.signature[b['id']][0] != b['state'] or
                   (positions and math.dist(self.signature[b['id']][1:], (b['x'], b['y'])) > 0.3)
                   for b in self.buoys)

    def do_plan(self, now, pose):
        if now < self.retry_at and not self.map_changed(positions=False):
            return 0.0, 0.0
        self.remember_map()
        target = self.find(name=self.look_target) if self.look_target is not None else None
        if target is None or target['state'] != gp.UNKNOWN:
            self.look_target, target = None, None
        if target is not None:
            # Navigate to a stand-off point in front of the buoy, then dwell (LOOK).
            wanted = self.look_distance(target['id'])
            distance = math.dist(pose[:2], (target['x'], target['y']))
            if distance <= wanted:
                self.look = dict(deadline=None)
                self.set('LOOK', f'classifying {target["id"]}')
                return 0.0, 0.0
            stands = self.stand_offs(target, wanted-self.p['arrive_tolerance'], pose)
            if not stands and not self.entry_done:
                # No place to look from outside the field here; the patrol will come round.
                self.unreachable[target['id']] = (now, tuple(pose[:2]))
                self.far_tries[target['id']] = self.far_tries.get(target['id'], 0)+1
                self.look_target, self.look_chain = None, []
                self.set('SEARCH_ENTRY', f'{target["id"]} cannot be seen from outside the field here')
                return 0.0, 0.0
            goal = stands[0] if stands else (target['x']+(wanted-self.p['arrive_tolerance'])*(pose[0]-target['x'])/distance,
                                             target['y']+(wanted-self.p['arrive_tolerance'])*(pose[1]-target['y'])/distance)
            ring, approach = None, 'TRANSIT'
        else:
            self.look_chain = []
            if self.wants_survey():
                unknown = self.nearest_unknown(pose)
                if unknown is not None:
                    return self.want_look([unknown['id']], pose)
            if not self.entry_done and self.find(state=gp.ENTRY) is None:
                self.set('SEARCH_ENTRY', 'entry buoy not in the map')
                return 0.0, 0.0
            goal, ring, approach = self.goal(pose)
            if goal is None:
                self.spin = 0.0
                self.set('SEARCH_EXIT', 'no exit and nothing ahead; searching')
                return 0.0, 0.0
            if approach == 'GATE' and math.dist(pose[:2], goal) <= self.p['arrive_tolerance']:
                return self.start_gate(pose)
            if math.dist(pose[:2], goal) <= self.p['arrive_tolerance']:
                return self.arrived(ring, pose)
        # Gaps already crossed are not crossed again: planning afresh after every gap has no
        # memory, and without this the vessel can weave back and forth between two routes.
        entering = target is None and approach == 'APPROACH_ENTRY'
        route = self.plan_route(pose[:2], goal, may_enter=entering)
        if (route is None or route['waypoints'][0]['kind'] == 'escape') and target is None:
            # Stopped a little short of a gap's middle, the vessel may lie in the neighbouring
            # triangle: a fresh plan then steps out and comes back to the same point, again and
            # again. The rest of the route being followed is kept instead, while it is clear.
            route = self.continued(pose, goal) or route
        if route is None and target is not None:
            # Other sides of the buoy: the stand-off point facing the vessel may be blocked.
            for point in stands[1:self.p['stand_off_tries']]:
                route = self.plan_route(pose[:2], point)
                if route is not None:
                    goal = point
                    break
        if route is None and ring is not None:
            # The nearest point of the circle can lie between two buoys that may only be passed
            # from the other side; any point of the circle is a valid place to start it.
            center, radius = ring['center'], ring['radius']
            base = math.atan2(goal[1]-center[1], goal[0]-center[0])
            for turn in (0.25, -0.25, 0.5, -0.5, 0.75, -0.75, 1.0):
                angle = base+turn*math.pi
                point = (center[0]+radius*math.cos(angle), center[1]+radius*math.sin(angle))
                route = self.plan_route(pose[:2], point, may_enter=entering)
                if route is not None:
                    goal, ring = point, dict(ring, point=point)
                    break
        if route is None and approach == 'GATE':
            # This gate cannot be reached now: try the next one at the next cycle.
            self.tried_gates.add((self.gate_target['port'], self.gate_target['starboard']))
            self.gate_target = None
            self.emit('gate_unreachable')
            return 0.0, 0.0
        if route is None and self.crossed and target is None:
            route = self.plan_route(pose[:2], goal, avoid=False, may_enter=entering)
            if route is not None:
                self.emit('recrossing_allowed')
        if route is None and self.entry_done:
            route = self.detour(pose, goal)
            if route is not None:
                self.emit('detour')
        index, line_start = 0, tuple(pose[:2])
        if route is not None:
            # A steering point of a wide gap that is already reached is skipped, otherwise the
            # vessel would stop on the edge of a triangle and plan the same point again.
            # Skipping changes the leg actually driven, so the direct leg is checked again.
            # A point reached within turn_point_tolerance is skipped anyway: the next leg is
            # then followed as planned, from that point (the line follower takes out the offset).
            def point(i):
                return route['waypoints'][i]['x'], route['waypoints'][i]['y']
            while (route['waypoints'][index]['kind'] == 'mid'
                   and not route['gaps'][route['waypoints'][index]['gap']]['narrow']
                   and math.dist(pose[:2], point(index)) <= 2*self.p['arrive_tolerance']):
                verdict = self.check([pose[:2], point(index+1)])
                if verdict['illegal'] is not None or verdict['tight'] is not None:
                    if math.dist(pose[:2], point(index)) > self.p['turn_point_tolerance']:
                        break  # The turn must be made at the point itself (see do_transit).
                    line_start = point(index)
                else:
                    line_start = tuple(pose[:2])
                index += 1
        needed = [] if route is None or not route['look'] or route['look_leg'] > index else route['look']
        if route is None or needed:
            fresh = [n for n in needed if n not in self.look_chain]
            if route is None and target is None:
                unknown = self.nearest_unknown(pose)
                fresh = [unknown['id']] if unknown is not None and unknown['id'] not in self.look_chain else []
            if fresh:
                return self.want_look(fresh, pose)
            if target is not None:
                # No legal way to get closer: look from here. Only a look from within the
                # colour range counts towards treating the buoy as unlit.
                self.emit('look_from_afar', id=target['id'])
                self.look = dict(deadline=None)
                self.set('LOOK', f'classifying {target["id"]} from a distance')
                return 0.0, 0.0
            self.detail = 'no legal route to the goal with the current map'
            if now >= self.retry_at:
                self.emit('no_route')
            self.retry_at = now+self.p['replan_interval']  # Planning again every cycle changes nothing.
            return 0.0, 0.0
        first = route['waypoints'][index]
        if first['kind'] == 'escape':
            self.escapes = (self.escapes+[tuple(pose[:2])])[-4:]
        self.leg = (line_start, (first['x'], first['y']))
        self.leg_next = (self.leg, point(index+1)) if index+1 < len(route['waypoints']) else None
        self.ring = ring if first['kind'] == 'goal' else None
        self.remember_map()
        self.previous = None if target is not None else dict(
            route=route, next=index+1, goal=tuple(goal), signature=dict(self.signature), obstacles=self.obstacle_count)
        if first['kind'] == 'align':
            gap = route['gaps'][first['gap']]
            mid, through = [w for w in route['waypoints'] if w['gap'] == first['gap']][1:]
            self.gap = dict(port=gap['port'], starboard=gap['starboard'], normal=np.array(gap['normal']),
                            before=math.dist((first['x'], first['y']), (mid['x'], mid['y'])),
                            after=math.dist((through['x'], through['y']), (mid['x'], mid['y'])),
                            mid=np.array([mid['x'], mid['y']]), frozen=False, turning=False)
            self.set('ALIGN', f'aligning with gap {gap["port"]} | {gap["starboard"]}')
        else:
            self.gap = None
            state = approach if first['kind'] == 'goal' and approach != 'GATE' else 'TRANSIT'
            self.set(state, f'heading to {first["kind"]} point')
        return 0.0, 0.0

    def start_gate(self, pose):
        """At the alignment point of a scoring gate: align with it and thread it."""
        gate, self.gate_target = self.gate_target, None
        self.tried_gates.add((gate['port'], gate['starboard']))
        mid = np.array(gate['mid'])
        self.gap = dict(port=gate['port'], starboard=gate['starboard'], normal=np.array(gate['normal']),
                        before=float(np.linalg.norm(mid-gate['align'])), after=float(np.linalg.norm(gate['through']-mid)),
                        mid=mid, frozen=False, turning=False)
        self.leg = (tuple(pose[:2]), tuple(gate['align']))
        self.remember_map()
        self.set('ALIGN', f'aligning with scoring gate {gate["port"]} | {gate["starboard"]}')
        return 0.0, 0.0

    def arrived(self, ring, pose):
        self.leg = self.gap = None
        if ring is None:
            self.spin = 0.0
            self.set('SEARCH_EXIT', 'far side reached without an exit; searching')
            return 0.0, 0.0
        self.ring = dict(ring, tracker=CircleTracker(list(ring['center']), ring['direction'], radius=ring['radius'],
                                                     minimum_angle=2*math.pi+self.p['circle_margin']))
        entry = ring['direction'] < 0
        if entry:
            self.entry_id = ring['id']
        self.set('CIRCLE_ENTRY' if entry else 'CIRCLE_EXIT',
                 f'circling {ring["id"]} {"clockwise" if entry else "counter-clockwise"}')
        return 0.0, 0.0

    # --------------------------------------------------------------- driving
    def do_transit(self, now, pose):
        if self.map_changed(positions=True):
            # Keep the leg while it is still legal: changing route on every map update makes the
            # vessel weave. A new plan is made at the next waypoint anyway.
            verdict = self.check([pose[:2], self.leg[1]])
            if verdict['illegal'] is not None or verdict['tight'] is not None or verdict['look']:
                self.leg = None
                self.set('PLAN', 'map changed and the current leg is no longer clear')
                return 0.0, 0.0
            self.remember_map()
        v, w, _, _, remaining = tc.follow(pose, self.leg[0], self.leg[1], self.p['cruise_speed'],
                                          self.p['max_yaw_rate'])
        distance = math.dist(pose[:2], self.leg[1])
        if remaining <= self.p['arrive_tolerance'] or distance <= self.p['arrive_tolerance']:
            # A route may turn at a waypoint so that a buoy swings behind the beam while the
            # vessel turns on the spot. Turning short of the point would sweep the beam over
            # that buoy while moving, so the vessel closes in until the next leg is clear.
            following = self.leg_next is not None and self.leg_next[0] is self.leg
            if (following and distance > self.p['turn_point_tolerance'] and remaining > 0.02
                    and self.check([pose[:2], self.leg_next[1]])['illegal'] is not None):
                return v, w
            self.reached = self.leg[1]
            ring, self.leg = self.ring, None
            if ring is not None and self.state != 'TRANSIT':
                return self.arrived(ring, pose)
            self.set('PLAN', 'waypoint reached')
            return 0.0, 0.0
        return v, w

    do_approach_entry = do_transit
    do_approach_exit = do_transit

    def gap_line(self):
        """Alignment, middle and pass-through points from the current buoy positions."""
        gap = self.gap
        ends = [self.find(name=gap[side]) for side in ('port', 'starboard')]
        if not gap['frozen'] and all(ends):
            a, b = (np.array([e['x'], e['y']]) for e in ends)
            normal = np.array([(a-b)[1], -(a-b)[0]])/np.linalg.norm(a-b)  # Port buoy on the left.
            gap['mid'], gap['normal'] = (a+b)/2, normal
        return gap['mid']-gap['before']*gap['normal'], gap['mid'], gap['mid']+gap['after']*gap['normal']

    def do_align(self, now, pose):
        gap = self.gap
        align, mid, through = self.gap_line()
        if self.map_changed(positions=False):
            ends = [self.find(name=gap[side]) for side in ('port', 'starboard')]
            wrong = ((ends[0] is not None and ends[0]['state'] == gp.RED) or
                     (ends[1] is not None and ends[1]['state'] == gp.GREEN))
            verdict = self.check([pose[:2], align] if not gap['turning'] else [pose[:2], through])
            if wrong or verdict['illegal'] is not None or (verdict['look'] and not gap['turning']):
                self.leg = self.gap = None
                self.set('PLAN', 'a lamp state changed and this gap is no longer usable')
                return 0.0, 0.0
            self.remember_map()
        heading = math.atan2(gap['normal'][1], gap['normal'][0])
        offset = math.dist(pose[:2], align)
        if not gap['turning']:
            v, w, _, _, remaining = tc.follow(pose, self.leg[0], align, self.p['cruise_speed'], self.p['max_yaw_rate'])
            if offset <= self.p['align_tolerance']:
                gap['turning'] = True
            elif remaining <= 0.0:
                if offset <= 2*self.p['align_tolerance']:
                    gap['turning'] = True
                else:
                    self.leg = (tuple(pose[:2]), tuple(align))  # Missed the point: approach it again.
            return (0.0, 0.0) if gap['turning'] else (v, w)
        error = tc.wrap(heading-pose[2])
        if abs(error) > self.p['align_heading']:
            return tc.turn(error, self.p['max_yaw_rate'])
        # Aligned. The lamps at both ends must be known before the vessel moves between them.
        # Checked along the very line that THREAD will hold: from here to the pass-through point.
        verdict = self.check([pose[:2], through])
        pending = sorted(set().union(*verdict['look'].values())) if verdict['look'] else []
        if pending:
            return self.want_look(pending, pose)
        if verdict['illegal'] is not None or verdict['tight'] is not None:
            self.leg = self.gap = None
            self.set('PLAN', 'gap no longer legal from here')
            return 0.0, 0.0
        if not self.rtk_ok:
            self.detail = 'aligned; waiting for an RTK fixed solution before threading'
            return 0.0, 0.0
        gap['frozen'] = True  # From here the line is held on the FCU pose alone.
        gap['line'] = (tuple(pose[:2]), tuple(through))
        self.set('THREAD', f'threading gap {gap["port"]} | {gap["starboard"]}')
        return 0.0, 0.0

    def do_thread(self, now, pose):
        gap = self.gap
        v, w, cross, _, remaining = tc.follow(pose, gap['line'][0], gap['line'][1], self.p['thread_speed'],
                                              self.p['max_yaw_rate'], lookahead=0.8)
        self.detail = f'threading gap {gap["port"]} | {gap["starboard"]}, offset {cross:+.2f} m'
        if remaining <= 0.0:
            # Only buoy pairs identify a gap; obstacle cells and outside points are reported as None.
            pair = (gap['port'], gap['starboard'])
            # Only buoy pairs identify a gap; obstacle cells and outside points are reported as None.
            # Going out to circle a buoy and coming back can legitimately need the same gap twice in
            # the same direction; a third time means the planner is cycling.
            again = None not in pair and self.passed.count(pair) >= 2
            if None not in pair:
                self.crossed.add(frozenset(pair))
            self.passed.append(pair)
            self.emit('gap_passed', port=gap['port'], starboard=gap['starboard'])
            if again:
                self.abort(f'gap {gap["port"]} | {gap["starboard"]} threaded three times: route is cycling')
                return 0.0, 0.0
            self.leg = self.gap = None
            self.set('PLAN', 'gap passed')
            return 0.0, 0.0
        return v, w

    def do_circle_entry(self, now, pose):
        return self.circle(pose, 'circle_entry_done')

    def do_circle_exit(self, now, pose):
        return self.circle(pose, 'circle_exit_done')

    def circle(self, pose, event):
        ring = self.ring
        if ring['tracker'].update(list(pose[:2])):
            self.emit(event, id=ring['id'])
            self.ring = None
            if event == 'circle_entry_done':
                self.entry_done = True
                self.set('PLAN', 'entry circle complete')
            else:
                self.set('COMPLETE', 'exit circle complete')
                self.emit('complete')
            return 0.0, 0.0
        v, w, radial = tc.circle(pose, ring['center'], ring['radius'], ring['direction'],
                                 self.p['circle_speed'], self.p['max_yaw_rate'])
        self.detail = (f'circling {ring["id"]}: {math.degrees(ring["tracker"].angle):.0f} deg, '
                       f'radius error {radial:+.2f} m')
        return v, w
