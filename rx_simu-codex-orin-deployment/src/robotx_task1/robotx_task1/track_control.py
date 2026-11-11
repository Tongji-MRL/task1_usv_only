"""Pivot, straight-segment and circle tracking for the small vessel (no ROS imports).

Outputs are forward speed (never negative: the control bridge rejects reverse)
and counter-clockwise yaw rate, both limited. ArduRover closes the inner loops.
"""
import math

import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


def turn(yaw_error, max_yaw_rate, gain=1.2):
    """Pivot on the spot towards a heading."""
    return 0.0, clamp(gain*yaw_error, max_yaw_rate)


def follow(pose, a, b, speed, max_yaw_rate, gain=1.2, lookahead=1.0, pivot_angle=0.3, taper=0.5):
    """Track the line a -> b. Returns (v, w, cross, heading_error, remaining).

    cross is the lateral offset (positive to port of the line). Beyond
    pivot_angle of heading error the vessel stops and turns first, and it slows
    down before the end of the segment: large heading changes are made on the
    spot, so the beam does not sweep across buoys while the vessel is moving.
    """
    a, b = np.asarray(a, float), np.asarray(b, float)
    length = float(np.linalg.norm(b-a))
    if length < 1e-6:
        return 0.0, 0.0, 0.0, 0.0, 0.0
    unit = (b-a)/length
    relative = np.array([pose[0], pose[1]])-a
    along, cross = float(relative @ unit), float(unit[0]*relative[1]-unit[1]*relative[0])
    error = wrap(math.atan2(unit[1], unit[0])+math.atan2(-cross, lookahead)-pose[2])
    v = min(speed, 0.08+taper*max(0.0, length-along))*math.cos(error) if abs(error) < pivot_angle else 0.0
    return v, clamp(gain*error, max_yaw_rate), cross, error, length-along


def circle(pose, center, radius, direction, speed, max_yaw_rate, gain=1.2, lookahead=0.8, pivot_angle=0.6):
    """Track a circle; direction +1 counter-clockwise, -1 clockwise. Returns (v, w, radial_error)."""
    dx, dy = pose[0]-center[0], pose[1]-center[1]
    distance = math.hypot(dx, dy)
    radial = distance-radius
    tangent = math.atan2(dy, dx)+direction*math.pi/2
    # Outside the circle: steer towards the center, which is to port when counter-clockwise.
    error = wrap(tangent+direction*math.atan2(radial, lookahead)-pose[2])
    if abs(error) >= pivot_angle:
        return 0.0, clamp(gain*error, max_yaw_rate), radial
    v = speed*math.cos(error)
    return v, clamp(direction*v/max(radius, 0.1)+gain*error, max_yaw_rate), radial


def circle_polygon(center, radius, direction, start_angle, count=16):
    """Closed polygon along the circle, for rule and clearance checks."""
    return [(center[0]+radius*math.cos(start_angle+direction*2*math.pi*i/count),
             center[1]+radius*math.sin(start_angle+direction*2*math.pi*i/count)) for i in range(count+1)]


def scan_blocked(ranges, angle_min, angle_increment, length=1.0, half_width=0.45, min_range=0.3):
    """True if a valid scan return lies in the box ahead of the vessel.

    The scan is expected in a base-aligned frame (x forward). Non-finite ranges
    are not obstacles here; the control bridge separately requires a fresh scan.
    """
    ranges = np.asarray(ranges, float)
    angles = angle_min+angle_increment*np.arange(len(ranges))
    valid = np.isfinite(ranges) & (ranges >= min_range)
    x, y = ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid])
    return bool(np.any((x > 0.0) & (x <= length) & (np.abs(y) <= half_width)))
