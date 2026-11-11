"""ROS-independent geometry and command validity rules."""

import math


def scan_ranges_valid(ranges, range_min, range_max):
    """NaNs are unknown; +inf denotes a valid no-hit ray, -inf is invalid."""
    return any(value == math.inf or (math.isfinite(value) and range_min <= value <= range_max)
               for value in ranges)


# SET_POSITION_TARGET_LOCAL_NED: ignore position, vz, acceleration and yaw.
# Keep vx, vy and yaw_rate. Do not set FORCE (512).
VELOCITY_YAW_RATE_MASK = 1 | 2 | 4 | 32 | 64 | 128 | 256 | 1024
BODY_NED_FRAME = 8


def normalise_quaternion(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('Quaternion must contain four finite values')
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-6:
        raise ValueError('Quaternion is zero')
    return tuple(v / norm for v in q)


def multiply_quaternions(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def split_yaw_attitude(q):
    """Return yaw-only rotation and residual; their product equals q."""
    x, y, z, w = full = normalise_quaternion(q)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    level = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    inverse = (-level[0], -level[1], -level[2], level[3])
    return level, multiply_quaternions(inverse, full)


def stamp_is_fresh(stamp_seconds, now_seconds, timeout, future_tolerance=0.1):
    return (math.isfinite(stamp_seconds) and stamp_seconds > 0.0 and
            -future_tolerance <= now_seconds - stamp_seconds <= timeout)


def bounded_command(values, max_speed, max_yaw_rate, lateral_tolerance=0.001):
    """Validate a ROS FLU Twist; return forward speed and CCW yaw rate.

    A rejected command must cause zero output, not leave an older command active.
    MAVROS, rather than this function, converts FLU/ENU signs to MAVLink FRD/NED.
    """
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite or incomplete Twist')
    vx, vy, vz, wx, wy, wz = values
    if any(abs(v) > lateral_tolerance for v in (vy, vz, wx, wy)):
        raise ValueError('Differential boat accepts only linear.x and angular.z')
    if vx < -lateral_tolerance:
        raise ValueError('Reverse is disabled in this navigation profile')
    return min(max(vx, 0.0), max_speed), max(-max_yaw_rate, min(wz, max_yaw_rate))


class CommandGate:
    """Require fresh command, odometry, scan and state using a monotonic clock."""

    def __init__(self, command_timeout=0.5, odom_timeout=0.5,
                 scan_timeout=0.5, state_timeout=2.0):
        self.timeouts = dict(command=command_timeout, odom=odom_timeout,
                             scan=scan_timeout, state=state_timeout)
        if not all(math.isfinite(t) and t > 0.0 for t in self.timeouts.values()):
            raise ValueError('All timeouts must be positive')
        self.received = {key: None for key in self.timeouts}
        self.command = (0.0, 0.0)
        self.eligible = False

    def update_state(self, connected, armed, mode, now):
        eligible = connected and armed and mode.upper() == 'GUIDED'
        # Do not replay a command cached before arming / changing mode.
        if eligible != self.eligible:
            self.received['command'] = None
        self.eligible = eligible
        self.received['state'] = now

    def update_command(self, command, now):
        self.command = command
        self.received['command'] = now

    def invalidate(self, source):
        self.received[source] = None

    def mark(self, source, now):
        self.received[source] = now

    def evaluate(self, now):
        if not self.eligible:
            return (0.0, 0.0), 'FCU must be connected, armed and in GUIDED'
        for source, timeout in self.timeouts.items():
            then = self.received[source]
            if then is None or not 0.0 <= now - then <= timeout:
                return (0.0, 0.0), source + ' missing or stale'
        return self.command, 'ready'
