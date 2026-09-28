"""2D rigid transforms as (x, y, theta) tuples, theta in radians CCW."""

import math


def wrap(theta):
    return math.atan2(math.sin(theta), math.cos(theta))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def from_json(p):
    if p is None:
        return None
    return (float(p["x"]), float(p["y"]), float(p["theta"]))
