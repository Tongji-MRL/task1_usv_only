"""Task 1 messages for the team's USV state machine (no ROS imports).

The USV state machine (usv_orin_state_machine, version2) starts Task 1 and is
the only party Task 1 talks to; the vessel never reports to the OCS itself.
Its interface, as used by the Task 3 executor:

* /tongji_mrl/twave/progress, String JSON with vehicle/task/progress/error.
  The state machine forwards every message to /tongji_mrl/mission/status, and
  `progress == "FINISHED"` ends Task 1 (it then stops this launch).
* /tongji_mrl/twave/result, String JSON: one report body in the official
  RxReport field names, in the envelope of the Task 3 executor
  (team_id, vehicle_id, seq, sent_at, <body>). The state machine forwards it
  to /tongji_mrl/mission/task_report.

Task 1 sends one SafePassageReport with every classified buoy when the
mission ends, COMPLETE or ABORT (buoys reported after an abort still score),
and only then progress FINISHED, so the state machine moves on to the next
task either way; after an ABORT the reason is in `error` (null after COMPLETE).

From the state machine Task 1 takes:

* /tongji_mrl/twave/course_config (published when Task 1 starts): the course
  corners, as in the official RxCourse (`corners`: latitude/longitude), used as
  the course boundary;
* /tongji_mrl/twave/task_cmd, Task 4 (the state machine keeps Task 1 running):
  {"task": "TASK4", "command": {"type": "TASK4_BEGIN", "command_seq": n,
  "center": {latitude, longitude}, "radius_m": r, ...}} is a keep-out zone and
  {"task": "TASK4", "command": "resume"} ends it. Both are acknowledged with an
  IncidentAck {"incident_ack": {"command_seq": n}} (the resume with the seq of
  the TASK4_BEGIN it ends) on the result topic, in the same envelope.
"""
import json
import math
from datetime import datetime, timezone

TERMINAL = ('COMPLETE', 'ABORT')
EARTH_RADIUS = 6378137.0  # Same small-offset model as robocommand_core.offset_latlon.


def progress(state, error=None):
    """Progress message for a mission_core state name; both end states are FINISHED."""
    name = 'FINISHED' if state in TERMINAL else 'TASK1_'+state
    return dict(vehicle='usv', task='TASK1', progress=name, error=error)


def timestamp(unix_seconds):
    return datetime.fromtimestamp(unix_seconds, tz=timezone.utc).isoformat().replace('+00:00', 'Z')


class ReportEnvelope:
    """Sequenced result envelope, the same layout as the Task 3 executor's."""

    def __init__(self, team_id='TONG', vehicle_id='usv', initial_seq=0):
        if not str(team_id).strip() or not str(vehicle_id).strip():
            raise ValueError('team_id and vehicle_id must be non-empty')
        if isinstance(initial_seq, bool) or not isinstance(initial_seq, int) or initial_seq < 0:
            raise ValueError('initial_seq must be a nonnegative integer')
        self.team_id, self.vehicle_id, self.seq = str(team_id).strip(), str(vehicle_id).strip(), initial_seq

    def envelope(self, name, body, unix_seconds):
        self.seq += 1
        return {'team_id': self.team_id, 'vehicle_id': self.vehicle_id, 'seq': self.seq,
                'sent_at': timestamp(unix_seconds), name: body}

    def safe_passage(self, body, unix_seconds):
        return self.envelope('safe_passage', body, unix_seconds)

    def incident_ack(self, command_seq, unix_seconds):
        return self.envelope('incident_ack', {'command_seq': command_seq}, unix_seconds)


def local_xy(latitude, longitude, ref_latitude, ref_longitude, ref_xy=(0.0, 0.0)):
    """East/north position [m] of a WGS84 point, given a reference point's WGS84 and local
    position (e.g. the vessel's fused fix and its odometry position): the inverse of
    robocommand_core.offset_latlon, so reports and inputs use one model."""
    east = math.radians(longitude-ref_longitude)*EARTH_RADIUS*math.cos(math.radians(ref_latitude))
    north = math.radians(latitude-ref_latitude)*EARTH_RADIUS
    return ref_xy[0]+east, ref_xy[1]+north


def _latlon(value):
    """(latitude, longitude) from {latitude, longitude} / {lat, lon} / {latitude_deg, ...} / [lat, lon]."""
    if isinstance(value, dict):
        for la, lo in (('latitude', 'longitude'), ('lat', 'lon'), ('latitude_deg', 'longitude_deg')):
            if la in value and lo in value:
                value = (value[la], value[lo])
                break
        else:
            return None
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            latitude, longitude = float(value[0]), float(value[1])
        except (TypeError, ValueError):
            return None
        if math.isfinite(latitude) and math.isfinite(longitude) and abs(latitude) <= 90 and abs(longitude) <= 180:
            return latitude, longitude
    return None


def course_corners(payload):
    """Course corners [(lat, lon), ...] from a course_config payload (dict or JSON text), or None.

    The `corners` list of the official RxCourse is searched for at any depth, so an
    envelope around it ({"course_config": {...}}, {"payload": {...}}) does not matter.
    At least three valid corners are needed.
    """
    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except ValueError:
            return None
    stack = [payload]
    while stack:
        item = stack.pop()
        if isinstance(item, dict):
            if isinstance(item.get('corners'), list):
                corners = [_latlon(c) for c in item['corners']]
                if len(corners) >= 3 and all(c is not None for c in corners):
                    return corners
            stack.extend(item.values())
        elif isinstance(item, list):
            stack.extend(item)
    return None


def task4_command(payload):
    """('begin', seq, (lat, lon) or None, radius) | ('resume',) | None for a task_cmd payload."""
    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except ValueError:
            return None
    if not isinstance(payload, dict) or payload.get('task') != 'TASK4' or payload.get('vehicle', 'usv') != 'usv':
        return None
    command = payload.get('command')
    if isinstance(command, str) and command.strip().lower() == 'resume':
        return ('resume',)
    if isinstance(command, dict):
        kind = str(command.get('type', '')).upper()
        if kind in ('RESUME', 'ALL_CLEAR'):
            return ('resume',)
        if kind in ('TASK4_BEGIN', 'KEEP_OUT_ZONE'):
            try:
                radius = float(command.get('radius_m'))
            except (TypeError, ValueError):
                radius = float('nan')
            return ('begin', command.get('command_seq'), _latlon(command.get('center')),
                    radius if math.isfinite(radius) and radius >= 0.0 else None)
    return None


class IncidentLink:
    """Task 4 seen from Task 1: which IncidentAck to send and whether Task 1 is paused.

    handle() returns (ack command_seq or None, changed). A repeated TASK4_BEGIN with the
    same seq is acknowledged again but changes nothing; a resume without an active Task 4
    is acknowledged with the last seq (or not at all if there never was one).
    """

    def __init__(self):
        self.active, self.seq, self.center, self.radius = False, None, None, None

    def handle(self, command):
        if command is None:
            return None, False
        if command[0] == 'begin':
            _, seq, center, radius = command
            changed = not self.active or seq != self.seq or center != self.center or radius != self.radius
            self.active, self.seq, self.center, self.radius = True, seq, center, radius
            return seq, changed
        changed = self.active
        self.active = False
        return self.seq, changed


class StateMachineWatch:
    """True once the state machine was seen and has then been absent for `timeout` seconds.

    The state machine starts Task 1 in its own session, so if the state machine dies
    without cleaning up (killed, crashed, terminal closed), nothing else would stop
    Task 1, which may hold the controls. Never expires before the first sighting, so
    Task 1 can still be run on its own for tests; timeout <= 0 disables the watch.
    """

    def __init__(self, timeout):
        self.timeout, self.seen_at = timeout, None

    def update(self, now, present):
        if present:
            self.seen_at = now
        return self.timeout > 0.0 and self.seen_at is not None and now-self.seen_at > self.timeout


class StateMachineLink:
    """Decides what to send to the state machine for each mission status.

    update() returns a list of ('progress' | 'result', payload) in send order.
    Progress is sent on every state change and refreshed every `refresh`
    seconds; the final progress is repeated every `repeat` seconds until the
    launch is stopped. At the end the report goes first: FINISHED makes the
    state machine stop this launch. Without a georeferenced report (no fresh
    GNSS fix) the final progress waits up to `report_wait` seconds, then goes
    out without one, and `missing_report` is set.
    """

    def __init__(self, envelope, refresh=2.0, repeat=1.0, report_wait=5.0):
        self.envelope, self.refresh, self.repeat, self.report_wait = envelope, refresh, repeat, report_wait
        self.state = self.sent_at = self.ended_at = self.finished_at = None
        self.reported = self.missing_report = False
        self.error = None

    def update(self, now, status, report_body, unix_seconds):
        state = status.get('state', '') if isinstance(status, dict) else ''
        if not state:
            return []
        out = []
        if state in TERMINAL:
            if self.ended_at is None:
                self.ended_at = now
                self.error = (status.get('detail') or 'aborted') if state == 'ABORT' else None
            if not self.reported:
                if report_body is not None:
                    out.append(('result', self.envelope.safe_passage(report_body, unix_seconds)))
                    self.reported = True
                elif now-self.ended_at < self.report_wait:
                    return out
                else:
                    self.reported = self.missing_report = True
            due = self.state != state or self.sent_at is None or now-self.sent_at >= self.repeat
            error = self.error
        else:
            due = self.state != state or self.sent_at is None or now-self.sent_at >= self.refresh
            error = None
        if due:
            out.append(('progress', progress(state, error)))
            self.state, self.sent_at = state, now
            if state in TERMINAL and self.finished_at is None:
                self.finished_at = now
        return out
