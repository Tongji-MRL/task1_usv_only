"""Contents of the RoboCommand reports for Task 1 (no ROS, no network imports).

Field and enum names follow the official schema, robonation/robocommand
RobotX_2026/proto (rx_reports.proto, rx_common.proto, common.proto). Per the
2026 handbook section 3.4 the vessel never connects to RoboCommand: these
dictionaries are sent to the team's Operator Control Station, and only the OCS
publishes them as protobuf over MQTT (scripts/ocs_relay.py).

Two points are not defined by the schema and are assumptions to confirm with
the technical director: heading_deg is taken as degrees clockwise from true
north, and a disarmed vehicle is reported as STATE_KILLED.
"""
import math

EARTH_RADIUS = 6378137.0
OFF, RED, GREEN, FLASH_BLUE, STEADY_BLUE, UNKNOWN = 0, 1, 2, 3, 4, 5
BEACON = {OFF: 'BEACON_STATE_OFF', RED: 'BEACON_STATE_FLASHING_RED', GREEN: 'BEACON_STATE_FLASHING_GREEN',
          FLASH_BLUE: 'BEACON_STATE_FLASHING_BLUE', STEADY_BLUE: 'BEACON_STATE_STEADY_BLUE'}
ACTIVE_STATES = ('SEARCH_ENTRY', 'SEARCH_EXIT', 'PLAN', 'LOOK', 'ALIGN', 'THREAD', 'TRANSIT',
                 'APPROACH_ENTRY', 'APPROACH_EXIT', 'CIRCLE_ENTRY', 'CIRCLE_EXIT', 'HOLD')


def offset_latlon(latitude, longitude, east, north):
    """WGS84 position `east`, `north` metres from a reference (small offsets)."""
    return (latitude+math.degrees(north/EARTH_RADIUS),
            longitude+math.degrees(east/(EARTH_RADIUS*math.cos(math.radians(latitude)))))


def robot_state(connected, armed, mode):
    if not connected:
        return 'STATE_UNKNOWN'
    if not armed:
        return 'STATE_KILLED'
    return 'STATE_AUTO' if mode.upper() == 'GUIDED' else 'STATE_MANUAL'


def current_task(mission_state):
    """A transition into TASK_SAFE_PASSAGE starts the attempt, TASK_NONE ends it."""
    return 'TASK_SAFE_PASSAGE' if mission_state in ACTIVE_STATES else 'TASK_NONE'


def heartbeat(latitude, longitude, speed, yaw, roll, pitch, altitude, state, task):
    """Heartbeat body; yaw is ENU (0 east, counter-clockwise), radians."""
    return dict(state=state, position=dict(latitude=latitude, longitude=longitude),
                spd_mps=round(speed, 3), heading_deg=round((90.0-math.degrees(yaw)) % 360.0, 2),
                roll_deg=round(math.degrees(roll), 2), pitch_deg=round(math.degrees(pitch), 2),
                altitude_hae_m=round(altitude, 2), current_task=task, vehicle_type='TYPE_USV')


def safe_passage(buoys, boat_xy, boat_latlon):
    """SafePassageReport body from the landmark list.

    Landmarks are in the odometry frame; they are georeferenced through the
    vessel's own position, so no map origin is needed. Unclassified buoys are
    not reported: a wrong state earns nothing and may be changed later anyway.
    """
    def position(buoy):
        latitude, longitude = offset_latlon(boat_latlon[0], boat_latlon[1],
                                            buoy['x']-boat_xy[0], buoy['y']-boat_xy[1])
        return dict(latitude=round(latitude, 7), longitude=round(longitude, 7))
    report = dict(buoys=[dict(position=position(b), state=BEACON[b['state']])
                         for b in buoys if b['state'] in BEACON])
    for state, key in ((FLASH_BLUE, 'entry_position'), (STEADY_BLUE, 'exit_position')):
        found = [b for b in buoys if b['state'] == state]
        if found:
            report[key] = position(found[0])
    return report


class ReportSchedule:
    """Send a SafePassageReport when the mapped states change, and refresh it slowly.

    Positions move by centimetres all the time; only a change of the set of
    reported states, or a landmark that moved by more than `moved` metres,
    counts as new. RoboCommand allows 5 messages per second per system.
    """

    def __init__(self, min_interval=1.0, refresh=10.0, moved=0.5):
        self.min_interval, self.refresh, self.moved = min_interval, refresh, moved
        self.sent_at, self.sent = None, None

    def due(self, now, buoys):
        current = {b['id']: (b['state'], b['x'], b['y']) for b in buoys if b['state'] in BEACON}
        if not current and self.sent is None:
            return False
        if self.sent_at is not None and now-self.sent_at < self.min_interval:
            return False
        changed = (self.sent is None or set(current) != set(self.sent) or any(
            current[k][0] != self.sent[k][0] or math.dist(current[k][1:], self.sent[k][1:]) > self.moved
            for k in current))
        if changed or now-self.sent_at >= self.refresh:
            self.sent_at, self.sent = now, current
            return True
        return False
