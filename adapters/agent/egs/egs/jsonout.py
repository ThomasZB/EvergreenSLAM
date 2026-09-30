"""--json shapes: poses as [x, y, theta], ages as seconds; service fields keep their names."""

from . import fmt


def pose(p):
    return None if p is None else [p[0], p[1], p[2]]


def age_s(stamp_ns):
    if stamp_ns is None:
        return None
    return round(max(0, fmt.now_ns() - int(stamp_ns)) / 1e9, 3)


def resolution(r):
    """Everything `egs where` knows; theta is null where the text prints `-`."""
    position = None
    if r.pose is not None:
        theta = r.pose[2] if r.precision in ("own", "offset") else None
        position = [r.pose[0], r.pose[1], theta]
    return {
        "path": r.path,
        "precision": r.precision,
        "place": r.place,
        "anchor": r.anchor,
        "state": r.state,
        "orphan_reason": r.orphan_reason,
        "approach": pose(r.approach),
        "pose": position,
        "resolved_via": r.resolved_via,
        "notes": list(r.notes),
        "bound_children": list(r.bound_children),
        "saved_age_s": age_s(r.saved_at_ns),
    }
