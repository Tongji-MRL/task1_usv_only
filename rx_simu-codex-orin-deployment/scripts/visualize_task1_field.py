#!/usr/bin/env python3
"""Replay the USV-only Task 1 mission on the field measured from the 2026-11-10 photos.

Builds a top-down map from pixel positions in the wide shore photo (pinhole
camera, calibrated with the buoy base width), runs the real mission code
(gap_planner, landmark_map, mission_core) in the closed-loop stand-in from
src/robotx_task1/test/mission_sim.py, and writes a self-contained HTML replay.

Usage (repository root):
    python3 scripts/visualize_task1_field.py [--field photo|delta|alpha] [--out DIR] [--video]

--field delta replays the published Delta course layout (four lettered boundary
buoys, ten lamp buoys, the Task 3 dock), scaled from the map image.
--field alpha replays Course Alpha from the RoboCommand screenshot, scaled with the
course corners of the RoboNation test server (25 m x 30 m).
--video also renders every run as an MP4 (needs matplotlib and imageio-ffmpeg).

Not a validation of control or perception: the vessel is a kinematic model and
the sensors see everything inside fixed ranges. See the page header.
"""
import argparse
import itertools
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/robotx_task1'), str(ROOT/'src/robotx_task1/test')]

import numpy as np  # noqa: E402

import mission_sim as ms  # noqa: E402
from robotx_task1 import gap_planner as gp  # noqa: E402
from robotx_task1.landmark_map import LandmarkMap  # noqa: E402

# ------------------------------------------------------------------ the field
# Photo 1 (2000x1500): base of each buoy (front edge), in pixels.
CAMERA = dict(focal_px=1500.0, cx=1000.0, cy=750.0, horizon_y=580.0,
              # Height from the 0.51 m buoy base: h = W * (y - horizon) / width_px,
              # 3.43-3.49 m from buoys 0, 1 and A; independent of the focal length.
              height_m=3.45, base_back_m=0.25)
PIXELS = {'0': (899, 899), '1': (1423, 875), '2': (693, 856), '3': (1081, 852), '4': (717, 815),
          '5': (879, 840), '6': (1130, 829), '7': (700, 796), '8': (914, 786), '9': (1105, 796),
          'A': (560, 1168), 'B': (290, 786), 'platform_front': (540, 842)}
# Lamps seen in photo 2. The other blue buoy is not visible: it is one of 0, 1, 3, 5, 6.
SEEN = {'7': gp.RED, '4': gp.RED, '2': gp.GREEN, '9': gp.GREEN}
BLUE_SEEN, UNSEEN = '8', ['0', '1', '3', '5', '6']
# C and D are not in the photos: placed to enclose the field with about 3 m of water.
ASSUMED_CORNERS = {'C': (4.0, 29.5), 'D': (9.0, 13.0)}
# Task 3 delivery platform, square [m]: its front edge spans 122 px at about 20 m in photo 1.
PLATFORM_SIZE = 1.6
STARTS = {'near A': ((-1.0, 11.0), math.pi/2), 'west side': ((-5.0, 15.0), math.radians(40))}


# Delta course map (1292x1218 px image). Buoy body centres and lamp colours read from the image.
DELTA_PIXELS = {'1': (250, 684), '2': (469, 680), '3': (164, 514), '4': (284, 564), '5': (208, 383),
                '6': (383, 497), '7': (452, 579), '8': (365, 371), '9': (477, 443), '10': (637, 386)}
DELTA_COLOURS = {'1': gp.RED, '6': gp.RED, '9': gp.RED, '7': gp.GREEN, '8': gp.GREEN, '10': gp.GREEN,
                 '3': gp.OFF, '4': gp.OFF}  # 2 and 5 are blue; which one flashes is not shown.
DELTA_CORNERS = {'A': (123, 1096), 'B': (111, 54), 'C': (1147, 179), 'D': (1034, 1143)}  # SW, NW, NE, SE.
# Task 3 dock: back beam and four legs, as centrelines with their width in pixels.
DELTA_DOCK = [((728, 838), (1008, 983), 45), ((718, 850), (684, 946), 25), ((805, 895), (770, 991), 25),
              ((893, 940), (857, 1036), 25), ((980, 985), (943, 1081), 25)]
DELTA_STARTS = {'东北角入场': ((1080, 245), -135.0), '东侧中部': ((960, 560), 180.0), '西南侧': ((330, 990), 70.0)}
# No scale is printed. Neighbouring buoy centres are 106-110 px apart. The team's on-site
# estimate (2026-11-11) is 2-3 m between buoys, two to three boat lengths: 40 px/m gives
# 2.7 m between centres (2.2 m of clear water). 32 and 53 px/m bracket it.
DELTA_SCALE = 40.0


# Course Alpha, RoboCommand screenshot (1920 px wide): buoy body centres in pixels. The
# course corners of the RoboNation test server (lat/lon) give a 25 m x 30 m rectangle,
# 646 x 778 px here: 25.9 px/m. The panel says "entry buoy-10, exit buoy-1"; the team
# reports buoy 1 (top) flashing. Both are replayed.
ALPHA_PIXELS = {'1': (888, 502), '2': (1048, 483), '3': (1184, 579), '4': (840, 646), '5': (981, 579),
                '6': (1086, 632), '7': (1158, 726), '8': (898, 788), '9': (990, 722), '10': (1057, 823)}
ALPHA_COLOURS = {'4': gp.RED, '8': gp.RED, '9': gp.RED, '2': gp.GREEN, '6': gp.GREEN, '7': gp.GREEN,
                 '3': gp.OFF, '5': gp.OFF}
ALPHA_CORNERS = {'A': (733, 1015), 'B': (733, 238), 'C': (1379, 237), 'D': (1380, 1015)}  # SW, NW, NE, SE.
ALPHA_SCALE = 25.9
# Two unlit buoys of the start gate near the south edge: not Task 1 buoys (mapper markers).
ALPHA_MARKERS = {'S1': (790, 984), 'S2': (1024, 970)}
# Task 3 dock: back beam and four legs, centrelines with their width in pixels.
ALPHA_DOCK = [((1176, 318), (1318, 392), 23), ((1166, 336), (1144, 372), 12), ((1213, 360), (1190, 398), 12),
              ((1258, 384), (1235, 422), 12), ((1304, 408), (1282, 447), 12)]
ALPHA_STARTS = {'起点门': ((907, 975), 90.0)}


def pixel_field(pixels, corners, scale, dock, starts, markers=None):
    """Field in metres from map pixels: origin at corner A (south-west), x east, y north."""
    ox, oy = corners['A']

    def metres(u, v):
        return round((u-ox)/scale, 3), round((oy-v)/scale, 3)

    def beam(a, b, width):
        (x0, y0), (x1, y1) = metres(*a), metres(*b)
        dx, dy = x1-x0, y1-y0
        length = math.hypot(dx, dy)
        nx, ny = -dy/length*width/scale/2, dx/length*width/scale/2
        return [(x0+nx, y0+ny), (x1+nx, y1+ny), (x1-nx, y1-ny), (x0-nx, y0-ny)]

    return dict(buoys={k: metres(*v) for k, v in pixels.items()},
                corners={k: metres(*v) for k, v in corners.items()},
                structures=[beam(a, b, w) for a, b, w in dock],
                starts={k: (metres(*uv), math.radians(yaw)) for k, (uv, yaw) in starts.items()},
                markers={k: metres(*v) for k, v in (markers or {}).items()},
                source={k: 'map' for k in 'ABCD'}, scale=scale)


def build_alpha_field(scale=ALPHA_SCALE):
    return pixel_field(ALPHA_PIXELS, ALPHA_CORNERS, scale, ALPHA_DOCK, ALPHA_STARTS, ALPHA_MARKERS)


def alpha_scenarios(field, colour_ranges=(5.0, 3.5)):
    for entry, leave, note in (('1', '10', '现场：上方 1 闪蓝'), ('10', '1', '面板：入口 buoy-10')):
        states = dict(ALPHA_COLOURS, **{entry: gp.ENTRY, leave: gp.EXIT})
        for colour_range in colour_ranges:
            for start in field['starts']:
                yield dict(name=f'{entry} 闪蓝入口，{leave} 常蓝出口（{note}）｜识色 {colour_range:g} m｜起点 {start}',
                           states=states, start=start, colour_range=colour_range)


def build_delta_field(scale=DELTA_SCALE):
    """Delta course in metres, origin at corner A (south-west), x east, y north (image up)."""
    ox, oy = DELTA_CORNERS['A']

    def metres(u, v):
        return round((u-ox)/scale, 3), round((oy-v)/scale, 3)

    def beam(a, b, width):
        (x0, y0), (x1, y1) = metres(*a), metres(*b)
        dx, dy = x1-x0, y1-y0
        length = math.hypot(dx, dy)
        nx, ny = -dy/length*width/scale/2, dx/length*width/scale/2
        return [(x0+nx, y0+ny), (x1+nx, y1+ny), (x1-nx, y1-ny), (x0-nx, y0-ny)]

    return dict(buoys={k: metres(*v) for k, v in DELTA_PIXELS.items()},
                corners={k: metres(*v) for k, v in DELTA_CORNERS.items()},
                structures=[beam(a, b, w) for a, b, w in DELTA_DOCK],
                starts={k: (metres(*uv), math.radians(yaw)) for k, (uv, yaw) in DELTA_STARTS.items()},
                source={k: 'map' for k in 'ABCD'}, scale=scale)


def delta_scenarios(field, colour_ranges=(5.0, 3.5)):
    for entry, leave in (('2', '5'), ('5', '2')):
        states = dict(DELTA_COLOURS, **{entry: gp.ENTRY, leave: gp.EXIT})
        for colour_range in colour_ranges:
            for start in field['starts']:
                yield dict(name=f'{entry} 闪蓝入口，{leave} 常蓝出口｜识色 {colour_range:g} m｜起点 {start}',
                           states=states, start=start, colour_range=colour_range)


def ground(u, v):
    """Pixel on the water -> (right, ahead) in metres from the photo position."""
    c = CAMERA
    pitch = math.atan((c['cy']-c['horizon_y'])/c['focal_px'])
    x, y = (u-c['cx'])/c['focal_px'], (v-c['cy'])/c['focal_px']
    down = y*math.cos(pitch)+math.sin(pitch)
    forward = -y*math.sin(pitch)+math.cos(pitch)
    t = c['height_m']/down
    return x*t, forward*t+c['base_back_m']


def build_field():
    """All positions in a frame with its origin at buoy A, x to the right, y away from the shore."""
    raw = {k: ground(*v) for k, v in PIXELS.items()}
    raw.update(ASSUMED_CORNERS)
    ox, oy = raw['A']
    shift = {k: (round(x-ox, 2), round(y-oy, 2)) for k, (x, y) in raw.items()}
    buoys = {str(i): shift[str(i)] for i in range(10)}
    corners = {k: shift[k] for k in 'ABCD'}
    px, py = shift['platform_front']
    platform = (px, py+PLATFORM_SIZE/2)
    starts = {name: ((x-ox, y-oy), yaw) for name, ((x, y), yaw) in STARTS.items()}
    return dict(buoys=buoys, corners=corners, platform=platform, starts=starts,
                structures=[ms.square(platform, PLATFORM_SIZE)],
                source={k: 'photo 1' for k in ('A', 'B')} | {k: 'assumed' for k in ASSUMED_CORNERS})


def inside(point, polygon):
    x, y, result = point[0], point[1], False
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:]+polygon[:1]):
        if (y0 > y) != (y1 > y) and x < x0+(y-y0)*(x1-x0)/(y1-y0):
            result = not result
    return result


def shrink(polygon, margin):  # Same as landmark_map.shrink_polygon; kept for the page.
    """Convex polygon moved inward by `margin` (corner buoys stay outside the mapper boundary)."""
    area = sum(x0*y1-x1*y0 for (x0, y0), (x1, y1) in zip(polygon, polygon[1:]+polygon[:1]))
    sign = 1 if area > 0 else -1
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


def scenarios(field):
    """Every lamp assignment consistent with photo 2, from every start."""
    for blue_role, other in itertools.product(('entry', 'exit'), UNSEEN):
        states = {n: gp.OFF for n in field['buoys']}
        states.update(SEEN)
        states[BLUE_SEEN] = gp.ENTRY if blue_role == 'entry' else gp.EXIT
        states[other] = gp.EXIT if blue_role == 'entry' else gp.ENTRY
        for start in field['starts']:
            label = f'8 {"闪蓝入口" if blue_role == "entry" else "常蓝出口"}，{other} {"常蓝出口" if blue_role == "entry" else "闪蓝入口"}'
            yield dict(name=f'{label}｜起点 {start}', states=states, start=start)


class FieldSimulation(ms.Simulation):
    """mission_sim with the four boundary buoys visible to the LiDAR."""

    def __init__(self, field, mapper_boundary, **kwargs):
        super().__init__(**kwargs)
        markers = list(field.get('markers', {}).values())
        # Corner buoys and known non-task buoys (start gate) are LiDAR clusters like any buoy.
        self.extra = [np.array(p) for p in list(field['corners'].values())+markers]
        self.map = LandmarkMap(boundary=mapper_boundary)
        self.map.markers = markers

    def sense(self, step):
        if step % 5 == 0:
            here = self.pose[:2]
            corners = [ms.Cluster(p+self.rng.normal(0, 0.03, 2), 12, 0.4, 0.0, 0.9)
                       for p in self.extra if math.dist(here, p) <= ms.LIDAR_RANGE]
            update = self.map.update_lidar
            self.map.update_lidar = lambda t, boat, clusters: update(t, boat, clusters+corners)
            super().sense(step)
            self.map.update_lidar = update
        else:
            super().sense(step)


def run(field, case, color_range, duration=1200.0):
    color_range = case.get('colour_range', color_range)
    corners = [field['corners'][k] for k in 'ADCB']
    boundary = corners  # Mapper course filter: inside it and not at a corner buoy.
    truth = [dict(id=n, x=x, y=y, state=case['states'][n], radius=0.2) for n, (x, y) in field['buoys'].items()]
    (sx, sy), yaw = field['starts'][case['start']]
    ms.COLOR_RANGE = color_range
    sim = FieldSimulation(field, boundary, buoys=truth, pose=(sx, sy, yaw), seed=1,
                          look_range=color_range-0.2, structures=field['structures'],
                          boundary=corners)
    sim.grid = ms.ObstacleGrid(boundary=boundary)
    markers = [tuple(q) for q in field.get('markers', {}).values()]
    if markers:
        # BuoyMapper publishes the markers with the obstacle cells (see its `markers` parameter).
        sim.grid.markers = markers
        grid_points = sim.grid.points
        sim.grid.points = lambda: grid_points()+markers
    routes, frames, maps, last_map, details, cells = [], [], [], None, [], []
    original_plan = gp.plan

    def capture(*args, **kwargs):
        route = original_plan(*args, **kwargs)
        if route is not None:
            routes.append([round(sim.time, 1), [[round(w['x'], 2), round(w['y'], 2), w['kind']]
                                                for w in route['waypoints']]])
        return route

    def record(s, out):
        step = len(s.track)-2
        if step % 2 == 0:
            if out['detail'] not in details:
                details.append(out['detail'])
            leg = s.mission.leg
            frames.append([round(s.time-ms.DT, 1), *[round(v, 3) for v in s.track[-2]], round(out['v'], 2),
                           round(out['w'], 2), out['state'], details.index(out['detail']),
                           [round(v, 2) for v in leg[1]] if leg else None])
        if not cells or cells[-1][2] != s.grid.revision:
            cells.append([round(s.time, 1), [[round(x, 2), round(y, 2)] for x, y in s.grid.points()], s.grid.revision])
        snapshot = [[m['id'], round(m['x'], 2), round(m['y'], 2), m['state']] for m in s.map.snapshot()]
        key = [(m[0], m[3], round(m[1], 1), round(m[2], 1)) for m in snapshot]
        if key != last_map[0]:
            last_map[0] = key
            maps.append([round(s.time, 1), snapshot])
        return out['state'] in ('COMPLETE', 'ABORT')

    last_map = [None]
    gp.plan = capture
    try:
        sim.mission.start()
        out = sim.run(duration, until=record)
    finally:
        gp.plan = original_plan
    verdict = sim.judge()
    track = np.array(sim.track)
    outside = [i for i, p in enumerate(track[::5]) if not inside(p[:2], corners)]
    platform_gap = verdict['platform_clearance']
    corner_gap = min(math.dist((x, y), c) for x, y, _ in track for c in corners)-0.25-ms.HULL
    names = {m['id']: min(truth, key=lambda b: math.dist((b['x'], b['y']), (m['x'], m['y'])))['id']
             for m in sim.map.snapshot()}
    names.update({m[0]: min(truth, key=lambda b: math.dist((b['x'], b['y']), (m[1], m[2])))['id']
                  for _, snap in maps for m in snap})
    result = dict(
        name=case['name'], truth=truth, final=out['state'], final_detail=out['detail'], time=round(sim.time, 1),
        frames=frames, details=details, maps=maps, routes=routes, names=names, cells=[c[:2] for c in cells],
        events=[dict(e, time=e['time']) for e in sim.events],
        verdict=dict(
            # Before the entry circle nothing scores; wrong / blind count the scored part only.
            wrong_before=len([e for e in verdict['passages'] if not e['legal']]),
            wrong=[[e['id'], e['side']] for e in verdict['passages'] if not e['legal']],
            legal=[[e['id'], e['side']] for e in verdict['passages'] if e['legal']],
            circle_abeam=[[e['id'], e['side'], e['legal']] for e in verdict['circle_passages'] if e['state'] in (1, 2)],
            blind=sorted(set(verdict['blind'])), clearance=round(verdict['clearance'], 2),
            winding={k: [round(math.degrees(v), 0) for v in w] for k, w in verdict['winding'].items()},
            outside_boundary_s=round(len(outside)*5*ms.DT, 1), platform_gap=round(platform_gap, 2),
            corner_gap=round(corner_gap, 2), assumed_off=sorted(sim.mission.assumed_off),
            score=handbook_score(sim, verdict, names)))
    result['verdict']['wrong'] = result['verdict']['score']['wrong']
    result['verdict']['wrong_before'] -= len(result['verdict']['wrong'])
    result['verdict']['blind'] = result['verdict']['score']['blind']
    return result


def handbook_score(sim, verdict, track_truth_names):
    """Task 1 Core items of the 2026 handbook, judged from the true buoys and the driven track.

    The heartbeat (25) is assumed; the reports are judged by the final onboard map. The
    "only to port / only to starboard" items count passages after the entry circle and
    before the exit circle, outside the circles themselves, and need at least one passage.
    """
    from robotx_task1.core import passage_events
    events = sim.events
    done = next((e['time'] for e in events if e['type'] == 'circle_entry_done'), None)
    first = int(round(done/ms.DT)) if done is not None else len(sim.track)
    last = sim.phase.index('CIRCLE_EXIT') if 'CIRCLE_EXIT' in sim.phase else len(sim.phase)
    scored = []
    for i in range(first, min(last, len(sim.track)-1)):
        a, b = sim.track[i], sim.track[i+1]
        if sim.phase[i].startswith('CIRCLE') or math.dist(a[:2], b[:2]) < ms.PIVOT_STEP:
            continue
        scored += [e for e in passage_events(list(a), list(b), sim.truth) if not e['ambiguous']]
    greens = [e for e in scored if e['state'] == gp.GREEN]
    reds = [e for e in scored if e['state'] == gp.RED]
    truth = {b['id']: b['state'] for b in sim.truth}
    final = {track_truth_names.get(m['id']): m['state'] for m in sim.map.snapshot()}
    reported = {k: v for k, v in final.items() if k is not None and v == truth.get(k)}
    entry = next(k for k, v in truth.items() if v == gp.ENTRY)
    leave = next(k for k, v in truth.items() if v == gp.EXIT)
    winding = verdict['winding']
    items = dict(
        heartbeat=25,
        report_entry=10*(entry in reported), report_exit=10*(leave in reported),
        report_green=10*min(4, sum(1 for k, v in reported.items() if v == gp.GREEN)),
        report_red=10*min(4, sum(1 for k, v in reported.items() if v == gp.RED)),
        circle_entry=25*(winding.get(entry, (0, 0))[0] <= -2*math.pi),
        circle_exit=25*(winding.get(leave, (0, 0))[1] >= 2*math.pi),
        green_only_port=25*bool(greens and all(e['legal'] for e in greens)),
        red_only_starboard=25*bool(reds and all(e['legal'] for e in reds)))
    coloured = {k for k, v in truth.items() if v in (gp.RED, gp.GREEN)}
    complete = (coloured <= set(reported) and entry in reported and leave in reported
                and all(items[k] for k in ('circle_entry', 'circle_exit', 'green_only_port', 'red_only_starboard')))
    items['bonus'] = 100*complete
    collisions = int(verdict['clearance'] <= 0.0)+int(verdict['platform_clearance'] <= 0.0)
    items['collisions'] = -10*collisions
    possible = (25+20+10*min(4, sum(v == gp.GREEN for v in truth.values()))
                + 10*min(4, sum(v == gp.RED for v in truth.values()))+100+100)
    # Time inside the buoy field before the entry circle (the entry buoy's own surroundings excluded).
    others = np.array([[b['x'], b['y']] for b in sim.truth if b['id'] != entry])
    centre = others.mean(axis=0)
    shrunk = centre+(others-centre)*np.maximum(0.0, 1.0-0.5/np.linalg.norm(others-centre, axis=1))[:, None]
    from scipy.spatial import Delaunay
    hull = Delaunay(shrunk)
    ex, ey = next((b['x'], b['y']) for b in sim.truth if b['id'] == entry)
    early = [q for q in sim.track[:first] if math.dist(q[:2], (ex, ey)) > 2.0]
    inside_field = int(np.sum(hull.find_simplex(np.array(early)[:, :2]) >= 0)) if early else 0
    # Unclassified buoys that came abeam in the scored part (as the closest buoy on their side).
    everything = [dict(b, state=gp.RED) for b in sim.truth]
    blind = set()
    for i in range(first, min(last, len(sim.track)-1)):
        a, b = sim.track[i], sim.track[i+1]
        if sim.phase[i].startswith('CIRCLE') or math.dist(a[:2], b[:2]) < ms.PIVOT_STEP:
            continue
        blind |= {e['id'] for e in passage_events(list(a), list(b), everything) if e['id'] not in sim.classified[i]}
    return dict(items=items, total=sum(items.values()), possible=possible, blind=sorted(blind),
                wrong=[[e['id'], e['side']] for e in scored if not e['legal']],
                field_before_entry_s=round(inside_field*ms.DT, 1),
                scored_passages=[[e['id'], e['side'], e['legal']] for e in scored])


def nearest_gaps(field):
    pairs = sorted((math.dist(a[1], b[1]), a[0], b[0]) for a, b in itertools.combinations(field['buoys'].items(), 2))
    return [[a, b, round(d-0.51, 2)] for d, a, b in pairs[:4]]


PHOTO_META = dict(
    title='Task 1 纯 USV 方案 · 现场地图回放',
    subtitle='浮标位置由 11-10 现场照片透视换算（相机高度按浮标底座宽度标定为 3.45 m）；灯色取自第二张照片：7、4 红，2、9 绿，8 蓝。另一个蓝灯在照片里看不到，只能是 0、1、3、5、6 之一，8 是闪蓝还是常蓝也无法从静态照片判断，所以列出全部 10 种组合 × 2 个起点。',
    notes=['坐标原点在浮标 A，x 向右、y 背离岸边（以拍摄位置为准），单位米。',
           'A、B 来自照片；C、D 照片里看不到，按包住场地、留约 3 m 水面假定。',
           '带红绿贴纸的 UUV 浮标已按要求忽略。'],
    structure='Task 3 平台')
ALPHA_META = dict(
    title='Task 1 纯 USV 方案 · Alpha 场地回放',
    subtitle='按 RoboCommand 的 Course Alpha 截图建图，比例由 RoboNation 测试服务器的场地角点换算（25 m × 30 m，25.9 像素/米）：红 4、8、9，绿 2、6、7，灭 3、5，蓝 1、10。现场确认上方 1 号是闪蓝入口；面板上写的是入口 buoy-10、出口 buoy-1，两种都跑。',
    notes=['坐标原点在西南角 A，x 向东、y 向北（图的上方），单位米。',
           '南边两个无灯浮标是起点门，不是 Task 1 浮标：建图节点按 markers 参数把它们排除出地标，只当障碍点发布。',
           '右上是 Task 3 码头（横梁加四条腿），船只能从雷达障碍栅格知道它。'],
    structure='Task 3 码头')
DELTA_META = dict(
    title='Task 1 纯 USV 方案 · Delta 场地回放',
    subtitle='按官方 Delta 场地分布图建图：红 1、6、9，绿 7、8、10，灭 3、4，蓝 2、5（图上分不出哪个闪烁，两种都跑），右下是 Task 3 码头。图上没有比例尺，按相邻浮标中心距约 2 m（与现场照片一致）换算。',
    notes=['坐标原点在西南角边界浮标 A，x 向东、y 向北（图的上方），单位米。',
           '四个角浮标是边界；码头按一根横梁加四条腿建模，船只能从雷达建出的障碍栅格知道它。',
           '起点：东北角（图上黄色虚线进场处）、东侧中部、西南侧。'],
    structure='Task 3 码头')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--field', choices=('photo', 'delta', 'alpha'), default='photo')
    parser.add_argument('--scale', type=float, default=DELTA_SCALE, help='delta map pixels per metre')
    parser.add_argument('--out', default='')
    parser.add_argument('--color-range', type=float, default=5.0,
                        help='reliable color range [m] for the photo field; not measured yet (survey step 1)')
    parser.add_argument('--video', action='store_true', help='also render every run as MP4 (videos/ in --out)')
    parser.add_argument('--only', type=int, default=None, help='replay only this scenario index')
    args = parser.parse_args()
    if args.field == 'alpha':
        field, meta = build_alpha_field(), dict(ALPHA_META)
        cases, default_out = list(alpha_scenarios(field)), ROOT/'runs/task1_alpha_viz'
    elif args.field == 'delta':
        field, cases, meta = build_delta_field(args.scale), None, dict(DELTA_META)
        cases = list(delta_scenarios(field))
        meta['subtitle'] += f' 比例尺 {args.scale:g} 像素/米。'
        default_out = ROOT/f'runs/task1_delta_viz_{args.scale:g}'
    else:
        field, meta = build_field(), dict(PHOTO_META)
        cases, default_out = list(scenarios(field)), ROOT/'runs/task1_field_viz'
    out = Path(args.out or default_out)
    out.mkdir(parents=True, exist_ok=True)
    runs = []
    if args.only is not None:
        cases = [cases[args.only]]
    for case in cases:
        result = run(field, case, args.color_range)
        v = result['verdict']
        print(f"{result['final']:9s} {result['time']:6.1f}s wrong={len(v['wrong'])} blind={v['blind']} "
              f"clear={v['clearance']} out={v['outside_boundary_s']}s structure={v['platform_gap']} "
              f"off={v['assumed_off']} score={v['score']['total']}/{v['score']['possible']} "
              f"field_before_entry={v['score']['field_before_entry_s']}s | {case['name']}", flush=True)
        runs.append(result)
    data = dict(meta=meta,
                field=dict(buoys=field['buoys'], corners=field['corners'], source=field['source'],
                           structures=field['structures'],
                           boundary=shrink([field['corners'][k] for k in 'ADCB'], 1.0),
                           markers=field.get('markers', {}),
                           nearest=nearest_gaps(field)),
                params=dict(lidar_range=ms.LIDAR_RANGE, color_range=args.color_range,
                            half_fov=math.degrees(ms.HALF_FOV), hull=ms.HULL),
                runs=runs)
    page = (Path(__file__).with_name('task1_field_viz.html').read_text(encoding='utf-8')
            .replace('/*DATA*/null', json.dumps(data, ensure_ascii=False, separators=(',', ':'))))
    (out/'index.html').write_text(page, encoding='utf-8')
    field_record = dict(buoys=field['buoys'], corners=field['corners'], structures=field['structures'])
    if args.field == 'alpha':
        field_record.update(frame='origin at corner A (south-west), x east, y north; metres',
                            scale_px_per_m=ALPHA_SCALE, pixels=ALPHA_PIXELS, corner_pixels=ALPHA_CORNERS,
                            markers=field['markers'], marker_pixels=ALPHA_MARKERS)
    elif args.field == 'delta':
        field_record.update(frame='origin at corner A (south-west), x east, y north; metres',
                            scale_px_per_m=args.scale, pixels=DELTA_PIXELS, corner_pixels=DELTA_CORNERS)
    else:
        field_record.update(frame='origin at buoy A; x to the right and y away from the shore as seen from the '
                            'photo position; metres', corner_source=field['source'], camera_model=CAMERA,
                            pixels=PIXELS)
    (out/'field.json').write_text(json.dumps(field_record, indent=1, ensure_ascii=False), encoding='utf-8')
    print(f'Wrote {out/"index.html"}')
    if args.video:
        import task1_video
        for number, result in enumerate(runs):
            index = args.only if args.only is not None else number
            path = out/'videos'/f'run_{index:02d}.mp4'
            task1_video.render(data, result, path)
            print(f'Wrote {path}')


if __name__ == '__main__':
    main()
