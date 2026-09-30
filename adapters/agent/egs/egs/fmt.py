"""Text rendering shared by the commands: ages, poses, distances."""

import math
import time


def now_ns():
    return time.time_ns()


def age_s(seconds):
    if seconds is None:
        return "-"
    s = max(0, int(seconds))
    if s < 60:
        return "%ds" % s
    if s < 3600:
        return "%dm" % (s // 60)
    if s < 86400:
        return "%dh" % (s // 3600)
    return "%dd" % (s // 86400)


def age_ns(stamp_ns, now=None):
    if stamp_ns is None:
        return "-"
    return age_s(((now if now is not None else now_ns()) - int(stamp_ns)) / 1e9)


def xy(p):
    return "%.2f,%.2f" % (p[0], p[1])


def pose(p):
    if p is None:
        return "-"
    return "%.2f,%.2f,%.2f" % (p[0], p[1], p[2])


def dist(a, b):
    if a is None or b is None:
        return None
    return math.hypot(a[0] - b[0], a[1] - b[1])


def meters(d):
    return "-" if d is None else "%.1fm" % d


def clock_hour(robot, p):
    """Bearing of p seen from the robot as a clock hour: 12 ahead, 3 right, 9 left."""
    if robot is None or p is None:
        return None
    if abs(p[0] - robot[0]) < 1e-9 and abs(p[1] - robot[1]) < 1e-9:
        return None
    rel = math.atan2(p[1] - robot[1], p[0] - robot[0]) - robot[2]
    hour = int(round(-rel / (math.pi / 6))) % 12
    return 12 if hour == 0 else hour


def clock(robot, p):
    hour = clock_hour(robot, p)
    return "-" if hour is None else "%do'clock" % hour


def ids(items):
    return ", ".join(str(i) for i in items) if items else "-"


def submap(s):
    return "[%s,%s]" % (s[0], s[1]) if s else "-"


def scan_match(m):
    """Agreement of the scan with the local map only; a wrong global pose can still score high."""
    if not m:
        return "match -"
    return "match %.2f (avg %.2f)" % (m["score"], m["avg"])
