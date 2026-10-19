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


def correction(r):
    m, rad = r.get("correction_m"), r.get("correction_rad")
    if m is None or rad is None:
        return "-"
    return "%.2f m %d°" % (m, round(math.degrees(rad)))


def relocalization(r):
    head = "relocalization %s %s%s %s" % (
        r.get("id"),
        r.get("kind"),
        "" if r.get("anchor") is None else " (anchor %s)" % r["anchor"],
        r.get("state"),
    )
    return head + _outcome_tail(r)


def _outcome_tail(r):
    state = r.get("state")
    if state == "matched":
        return ": corrected " + correction(r)
    if state == "no_match":
        return " (%s candidates)" % r.get("candidates")
    if state == "superseded":
        by = r.get("superseded_by")
        return " (fed session dropped)" if by is None else " by %s" % by
    return ""


def event(e):
    kind = e.get("kind")
    head = "%s %s" % (e.get("seq"), kind)
    if kind == "relocalization":
        return "%s request %s %s%s" % (head, e.get("request_id"), e.get("state"), _outcome_tail(e))
    if kind == "pose_correction":
        return "%s session %s moved %s" % (head, e.get("session"), correction(e))
    if kind == "fed_session_dropped":
        return "%s session %s, fed now %s" % (head, e.get("session"), e.get("fed_now"))
    if kind == "kidnap_suspected":
        return "%s session %s: %s; searching as request %s" % (
            head,
            e.get("session"),
            _kidnap_evidence(e),
            e.get("request_id"),
        )
    return "%s session %s" % (head, e.get("session"))


def _kidnap_evidence(k):
    lowest = k.get("lowest_match_score")
    return "base %.2f (cover %d%%), %s without base match, %s" % (
        k["base_score"],
        round(100 * k["base_coverage"]),
        meters(k.get("since_base_match_m")),
        "never matched the base" if lowest is None else "match dipped to %.2f" % lowest,
    )


def kidnap(k, r=None):
    """The newest kidnap the process suspected on its own, and where its search stands."""
    head = "kidnap suspected: %s; request %s" % (_kidnap_evidence(k), k.get("request_id"))
    if r and r.get("id") == k.get("request_id") and r.get("state") == "pending":
        return head + ", searching"
    if not k.get("continuity_broken"):
        return head + ", found again"
    return head + (
        ", not confirmed by the base since: if egs here --sample 20 stays below 0.45, "
        "egs session rm %s (plan, then --yes), then egs relocalize" % k.get("session")
    )


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


def localization(loc):
    loc = loc or {}
    base = loc.get("base")
    if not base:
        base_text = "base -"
    else:
        score = base.get("score")
        base_text = "base %s (cover %d%%)" % (
            "-" if score is None else "%.2f" % score,
            round(100 * base["coverage"]),
        )
    m = loc.get("scan_match")
    match_text = "match -" if not m else "match %.2f (avg %.2f)" % (m["score"], m["avg"])
    return "%s  since base match %s  %s" % (
        base_text,
        meters(loc.get("since_base_match_m")),
        match_text,
    )


def base_sample(b):
    if not b["covered_reads"]:
        text = "base over %gs: no read at cover >= 50%% (%d reads)" % (b["seconds"], b["reads"])
    else:
        text = "base over %gs: median %.2f  min %.2f  (%d of %d reads at cover >= 50%%)" % (
            b["seconds"],
            b["median"],
            b["min"],
            b["covered_reads"],
            b["reads"],
        )
    if b.get("restarted"):
        text += (
            "  restarted %d: a correction landed, only reads after it count; sample again "
            "until the median settles" % b["restarted"]
        )
    return text
