"""egs zone: keep-out polygons stored in the anchor frame of their nearest place (zone.yaml)."""

import math
import os
import posixpath

from . import se2
from .client import EXIT_OK, EXIT_REFUSED, EXIT_USAGE, EgsError, explain
from .store import node_path
from .yamlmini import dumps

ZONE_FILE = "zone.yaml"
KEEPOUT = "keepout"
MIN_AREA_M2 = 0.01
MAX_EXTENT_M = 100.0  # the service's ZoneStore::kMaxExtentM
DEFAULT_OFFSET_MAX_M = 10.0
ZONE_KEY_ORDER = ("kind", "frame", "polygon")

REASONS = {
    "frame_mismatch": "the directory moved under another place: `egs zone rm` it and add it again "
    "while standing near its new place",
    "orphan": "the place's binding is orphaned: go there and `egs place save` it again",
    "unknown_anchor": "the place's place.yaml names an anchor the process does not know",
    "no_place": "no directory at or above it has a place.yaml",
    "holds_place": "zone.yaml and place.yaml share a directory: put the zone one level below",
    "unreadable_place": "the nearest place.yaml does not parse",
    "bad_zone_file": "zone.yaml is not kind + polygon of at least 3 [x, y] points",
    "too_large": "the polygon is over %gm wide or tall in its anchor frame: `egs zone rm` it"
    % MAX_EXTENT_M,
    "unknown_kind": "not a keepout zone: ignored",
}


def area(points):
    """Absolute shoelace area; rigid transforms keep it, so either frame gives the same number."""
    n = len(points)
    return (
        abs(
            sum(
                points[i][0] * points[(i + 1) % n][1] - points[(i + 1) % n][0] * points[i][1]
                for i in range(n)
            )
        )
        / 2.0
    )


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _segments_touch(p1, p2, q1, q2):
    d1, d2 = _cross(q1, q2, p1), _cross(q1, q2, p2)
    d3, d4 = _cross(p1, p2, q1), _cross(p1, p2, q2)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and 0 not in (d1, d2, d3, d4):
        return True

    def on(a, b, c):
        return (
            _cross(a, b, c) == 0
            and min(a[0], b[0]) <= c[0] <= max(a[0], b[0])
            and min(a[1], b[1]) <= c[1] <= max(a[1], b[1])
        )

    return on(q1, q2, p1) or on(q1, q2, p2) or on(p1, p2, q1) or on(p1, p2, q2)


def self_intersects(points):
    n = len(points)
    if len(set(points)) != n:
        return True
    for i in range(n):
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue
            if _segments_touch(points[i], points[(i + 1) % n], points[j], points[(j + 1) % n]):
                return True
    return False


def _parse_points(args):
    if args.rect is not None:
        x0, y0, x1, y1 = args.rect
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    points = []
    for token in args.polygon.split():
        parts = token.split(",")
        try:
            if len(parts) != 2:
                raise ValueError(token)
            points.append((float(parts[0]), float(parts[1])))
        except ValueError:
            raise EgsError('--polygon takes "X,Y X,Y X,Y ..." (bad point %r)' % token, EXIT_USAGE)
    return points


def _check_shape(points):
    if len(points) < 3:
        raise EgsError("a zone needs at least 3 points", EXIT_USAGE)
    if not all(math.isfinite(v) for p in points for v in p):
        raise EgsError("zone points must be finite numbers", EXIT_USAGE)
    if self_intersects(points):
        raise EgsError(
            "the polygon crosses itself or repeats a point: list the corners in order around it",
            EXIT_USAGE,
        )
    a = area(points)
    if a < MIN_AREA_M2:
        raise EgsError(
            "the polygon is degenerate (area %.4fm2 < %.2fm2)" % (a, MIN_AREA_M2), EXIT_USAGE
        )
    for axis in (0, 1):
        values = [p[axis] for p in points]
        if max(values) - min(values) > MAX_EXTENT_M:
            raise EgsError(
                "the polygon is over %gm wide or tall; a keep-out zone is a local area"
                % MAX_EXTENT_M,
                EXIT_USAGE,
            )


def cmd_add(ctx, args):
    path = node_path(args.path, ctx.memory_dir() if os.path.isabs(args.path.strip()) else None)
    if path == "places":
        raise EgsError(
            "a zone lives in a directory below a place: places/<place>/<name>", EXIT_USAGE
        )
    points = _parse_points(args)
    _check_shape(points)
    store, mem = ctx.store(), ctx.memory()
    kind = store.kind(path)
    if kind not in (None, "dir"):
        raise EgsError("%s exists and is not a directory" % path, EXIT_USAGE)
    names = {n: t for n, t, _ in (store.listdir(path) or [])} if kind == "dir" else {}
    if ZONE_FILE in names:
        ctx.print(
            "refused: %s/%s exists; add is not an update: `egs zone rm %s` first"
            % (path, ZONE_FILE, path)
        )
        return EXIT_REFUSED
    if "place.yaml" in names:
        ctx.print(
            "refused: %s holds a place.yaml; a zone lives in its own directory below the place, "
            "e.g. %s/<name>" % (path, path)
        )
        return EXIT_REFUSED
    found = mem.nearest_place(path)
    if found is None:
        ctx.print(
            "refused: no directory above %s has a place.yaml; a zone moves with its place's "
            "anchor, so stand there and `egs place save <ancestor>` first" % path
        )
        return EXIT_REFUSED
    a_path, aid, _ = found

    robot_frame = args.frame == "robot"
    j = ctx.client.get("/anchors/%d" % aid, {"robot": 1} if robot_frame else None)
    if not j.get("ok"):
        return ctx.refused(j)
    a_pose = se2.from_json(j.get("pose"))
    if a_pose is None:
        ctx.print("refused: %s" % explain("unresolvable", "%s (anchor %d)" % (a_path, aid)))
        return EXIT_REFUSED
    if robot_frame:
        robot = se2.from_json(j.get("robot"))
        if robot is None:
            ctx.print("refused: %s" % explain("no_robot_pose"))
            return EXIT_REFUSED
        # Unaligned, the robot pose is in the fed session's own frame and a frozen anchor is not:
        # the conversion would persist a polygon in the wrong place. --force does not cover this.
        if j.get("has_frozen_base") and j.get("aligned_to_base") is False:
            ctx.print(
                "refused: not aligned to the base map yet, so the robot pose and %s's anchor are in "
                "different frames; drive where the maps overlap until `egs status` says aligned to "
                "base yes, then retry (or give the points with --frame anchor)" % a_path
            )
            return EXIT_REFUSED
        limit = float(ctx.env.get("EGS_OFFSET_MAX_M") or DEFAULT_OFFSET_MAX_M)
        dist = math.hypot(robot[0] - a_pose[0], robot[1] - a_pose[1])
        if dist > limit and not args.force:
            ctx.print(
                "refused: the robot is %.1fm from %s (anchor %d), over the %.1fm limit "
                "(EGS_OFFSET_MAX_M); save a place nearer the zone with `egs place save`, "
                "or pass --force" % (dist, a_path, aid, limit)
            )
            return EXIT_REFUSED
        to_anchor = se2.compose(se2.inverse(a_pose), robot)
        points = [se2.compose(to_anchor, (x, y, 0.0))[:2] for x, y in points]

    polygon = [[round(x, 3), round(y, 3)] for x, y in points]
    _check_shape([tuple(p) for p in polygon])
    store.mkdir(path)
    text = dumps({"kind": KEEPOUT, "frame": a_path, "polygon": polygon}, ZONE_KEY_ORDER)
    store.write(posixpath.join(path, ZONE_FILE), text.encode("utf-8"))
    ctx.print(
        "zone %s keepout: %d points, %.2fm2, in the frame of %s (anchor %d)  @solve %s"
        % (path, len(polygon), area(polygon), a_path, aid, j.get("at_num_solves", "-"))
    )
    return EXIT_OK


def _zone_line(z):
    head = "%s %s" % (z.get("path"), z.get("kind") or "-")
    place = z.get("frame_path")
    anchor = z.get("anchor")
    where = "%s (anchor %s)" % (place, anchor) if place and anchor is not None else place or "-"
    reason = z.get("reason")
    if z.get("polygon_xy") is None:
        return "%s  %s  %s: %s" % (
            head,
            where,
            "ignored" if reason == "unknown_kind" else "unresolvable",
            explain_zone(reason),
        )
    line = "%s  %s %s  area %.2fm2" % (head, where, z.get("state"), area(z.get("polygon") or []))
    if reason:
        line += "  %s: %s (zone.yaml says frame %s)" % (
            reason,
            REASONS.get(reason, ""),
            z.get("frame"),
        )
    return line


def explain_zone(reason):
    return "%s (%s)" % (reason, REASONS[reason]) if reason in REASONS else str(reason)


def cmd_ls(ctx, args):
    j = ctx.client.get("/zones")
    if not j.get("ok"):
        return ctx.refused(j)
    zones = j.get("zones") or []
    rows = []
    for z in zones:
        row = dict(z)
        row["area_m2"] = round(area(z.get("polygon") or []), 4)
        if not args.xy:
            row.pop("polygon_xy", None)
        rows.append(row)
    ctx.result(dict(j, zones=rows))
    if not zones:
        ctx.print("no zones")
    for z in zones:
        ctx.print(_zone_line(z))
        if args.xy and z.get("polygon_xy") is not None:
            ctx.print("  xy " + " ".join("%.2f,%.2f" % (p[0], p[1]) for p in z["polygon_xy"]))
    ctx.print("@solve %s" % j.get("at_num_solves", "-"))
    return EXIT_OK


def cmd_rm(ctx, args):
    path = node_path(args.path, ctx.memory_dir() if os.path.isabs(args.path.strip()) else None)
    store = ctx.store()
    rel = posixpath.join(path, ZONE_FILE)
    if store.kind(rel) != "file":
        ctx.print("refused: %s has no %s" % (path, ZONE_FILE))
        return EXIT_REFUSED
    store.rm(rel)
    removed_dir = store.listdir(path) == []
    if removed_dir:
        store.rm(path)
    ctx.print("removed %s%s" % (rel, " and the empty directory" if removed_dir else ""))
    return EXIT_OK
