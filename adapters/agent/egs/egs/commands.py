"""Usage-layer commands: here, status, place save, where, find, observe, snapshot, init-pose,
relocalize, checkpoint, root."""

import json
import math
import os
import posixpath
import re
import shutil

from . import fmt, se2
from .client import EXIT_OK, EXIT_REFUSED, EXIT_USAGE, EgsError, explain
from .memory import Resolution, UnreadablePlace, match_rank
from .store import node_path
from .yamlmini import dumps

NODE_KEY_ORDER = ("kind", "aliases", "labels", "offset", "offset_from")
DEFAULT_OFFSET_MAX_M = 10.0


def solve(j):
    return "@solve %s" % j.get("at_num_solves", "-")


def cmd_root(ctx, args):
    d = ctx.memory_dir()
    ctx.print(d)
    if not ctx.local_ok():
        ctx.warn("note: %s is not readable here; use `egs fs ...` (paths relative to memory/)" % d)
    return EXIT_OK


def cmd_here(ctx, args):
    j = ctx.client.get("/here")
    if not j.get("ok"):
        return ctx.refused(j)
    if j.get("has_frozen_base") and j.get("aligned_to_base") is False:
        ctx.print("aligned to base: no (positions below are unreliable until aligned)")
    cur = j.get("current")
    ctx.print("current " + ("%s %s" % (cur["path"], fmt.meters(cur["dist_m"])) if cur else "-"))
    closest = j.get("closest") or []
    ctx.print(
        "closest "
        + (
            "; ".join(
                "%s %s %s" % (c["path"], fmt.meters(c["dist_m"]), c["state"]) for c in closest
            )
            or "-"
        )
    )
    robot = se2.from_json(j.get("robot"))
    ctx.print(
        "robot %s  keyframe %s  %s"
        % (fmt.pose(robot), fmt.age_s(j.get("keyframe_age_s")), solve(j))
    )
    return EXIT_OK


def cmd_status(ctx, args):
    j = ctx.client.get("/status")
    if not j.get("ok"):
        return ctx.refused(j)
    yn = lambda v: "-" if v is None else ("yes" if v else "no")  # noqa: E731
    ctx.print(
        ("map %s  " % j["map"] if j.get("map") else "")
        + "boot %s  fed session %s  phase %s"
        % (j.get("boot_count"), j.get("fed_session"), j.get("phase")),
        "frozen base %s  aligned to base %s"
        % (yn(j.get("has_frozen_base")), yn(j.get("aligned_to_base"))),
        "closures +%s since last status" % j.get("closures_delta"),
        "anchors %s (orphans %s)  place.yaml files %s"
        % (j.get("num_anchors"), j.get("num_orphans"), j.get("num_place_files")),
    )
    dups = j.get("duplicate_anchor_paths") or []
    if dups:
        ctx.print("duplicate anchor (cp -r?), ignored by here: " + ", ".join(dups))
    stale = _index_mismatch(ctx, j.get("num_place_files"))
    if stale:
        ctx.print(stale)
    ctx.print(solve(j))
    return EXIT_OK


def _index_mismatch(ctx, num_place_files):
    try:
        data = ctx.store().read("index.tsv")
    except EgsError:
        return None
    if data is None:
        return "index.tsv missing: run `egs snapshot`" if num_place_files else None
    rows = [l for l in data.decode("utf-8", "replace").splitlines() if l and not l.startswith("#")]
    if num_place_files is not None and len(rows) != num_place_files:
        return "index.tsv stale: %d rows, %s place.yaml files: run `egs snapshot`" % (
            len(rows),
            num_place_files,
        )
    return None


def cmd_place_save(ctx, args):
    path = node_path(args.path, _mem_dir_or_none(ctx, args.path))
    j = ctx.client.post("/place/save", {"path": path, "scan": not args.no_scan})
    if not j.get("ok"):
        code = ctx.refused(j)
        if j.get("anchor"):
            ctx.print("anchor %s was saved but %s/place.yaml was not written" % (j["anchor"], path))
        return code
    ctx.print(
        "saved %s anchor %s (%s)  state %s  submap %s  keyframe %s  %s"
        % (
            path,
            j.get("anchor"),
            "re-bound existing" if j.get("rebound_existing") else "new",
            j.get("state"),
            fmt.submap(j.get("submap_id")),
            fmt.age_s(j.get("keyframe_age_s")),
            solve(j),
        )
    )
    return EXIT_OK


def _mem_dir_or_none(ctx, raw):
    """Only an absolute path needs memory/'s location; spare relative ones the /root call."""
    return ctx.memory_dir() if os.path.isabs(raw.strip()) else None


def _existing_node(ctx, raw):
    path = node_path(raw, ctx.memory_dir())
    mem = ctx.memory()
    if not mem.is_dir(path):
        raise EgsError("no such node: %s (paths are relative to memory/)" % path, EXIT_USAGE)
    return path, mem


def cmd_where(ctx, args):
    path, mem = _existing_node(ctx, args.path)
    found = mem.nearest_place(path)
    j = {}
    if found:
        j = ctx.client.get("/anchors/%d" % found[1])
        if not j.get("ok") and j.get("reason") != "unknown_anchor":
            return ctx.refused(j)
    r = mem.resolve(path, lambda aid: j if j.get("ok") else None)
    if found is None:
        ctx.print(
            "%s  precision none  reason no bound ancestor  bound children: %s"
            % (path, ", ".join(r.bound_children) or "-")
        )
        return EXIT_OK
    for n in r.notes:
        ctx.print("note: " + n)
    if r.state == "unknown":
        ctx.print(
            "%s  unresolvable: anchor %d in %s/place.yaml is unknown" % (path, r.anchor, r.place)
        )
        return EXIT_REFUSED
    if not r.resolved:
        ctx.print(
            "%s  unresolvable: %s (anchor %d) is orphan (%s); go there and `egs place save %s`  %s"
            % (path, r.place, r.anchor, r.orphan_reason or "-", r.place, solve(j))
        )
        return EXIT_REFUSED
    theta = "%.2f" % r.pose[2] if r.precision in ("own", "offset") else "-"
    ctx.print(
        "%s  xy %s  theta %s  precision %s  resolved_via %s"
        % (path, fmt.xy(r.pose), theta, r.precision, r.resolved_via),
        "approach %s %s (%s)  saved %s  %s"
        % (r.place, fmt.pose(r.approach), r.state, fmt.age_ns(r.saved_at_ns), solve(j)),
    )
    return EXIT_OK


def cmd_find(ctx, args):
    query = " ".join(args.query)
    mem = ctx.memory()
    hits = []
    for rel in mem.nodes():
        ny, _ = mem.node_yaml(rel)
        rank = match_rank(query, posixpath.basename(rel), ny)
        if rank is not None:
            hits.append((rank, rel, ny or {}))
    if not hits:
        ctx.print("no match for %r" % query)
        return EXIT_OK
    j = ctx.client.get("/anchors", {"robot": 1})
    if not j.get("ok"):
        return ctx.refused(j)
    table = {a["anchor"]: a for a in j.get("anchors", [])}
    robot = se2.from_json(j.get("robot"))
    rows = []
    for rank, rel, ny in hits:
        try:
            r = mem.resolve(rel, table.get)
        except UnreadablePlace as e:
            # One broken place.yaml flags its own rows; the service skips it the same way.
            r = Resolution(rel)
            r.place, r.state = e.place, "unreadable"
        last = mem.last_observation(rel)
        d = fmt.dist(robot, r.pose)
        key = (
            bool(last) and last.get("result") == "absent",
            rank,
            math.inf if d is None else d,
            rel,
        )
        rows.append((key, _find_row(rel, ny, r, last, robot, d)))
    rows.sort(key=lambda x: x[0])
    limit = len(rows) if args.limit <= 0 else args.limit
    for _, line in rows[:limit]:
        ctx.print(line)
    if len(rows) > limit:
        ctx.print("... %d more (--limit 0 for all)" % (len(rows) - limit))
    ctx.print(solve(j))
    return EXIT_OK


def _find_row(rel, ny, r, last, robot, d):
    if r.resolved:
        prec = "%s:%s(%s)" % (r.precision, r.resolved_via, r.state)
    elif r.place:
        prec = "%s:%s" % (r.state if r.state in ("unknown", "unreadable") else "orphan", r.place)
    else:
        prec = "none"
    last_s = "last=%s %s" % (last["result"], fmt.age_ns(last["recorded_at"])) if last else "last=-"
    return "%s [%s] %s %s %s prec=%s" % (
        rel,
        ny.get("kind") or "-",
        fmt.meters(d),
        fmt.clock(robot, r.pose),
        last_s,
        prec,
    )


def cmd_observe(ctx, args):
    path, mem = _existing_node(ctx, args.path)
    result = args.result or (None if args.invalidates is not None else "seen")
    obs_rel = posixpath.join(path, "observations.jsonl")
    if args.offset is not None:
        code = _write_offset(ctx, mem, path, args)
        if code != EXIT_OK:
            return code
    last = max((o["recorded_at"] for o in mem.observations(path)), default=0)
    line = {"recorded_at": max(fmt.now_ns(), last + 1), "source": args.source}
    if result:
        line["result"] = result
    if args.invalidates is not None:
        line["invalidates"] = args.invalidates
    if args.note:
        line["note"] = args.note
    ctx.store().append(obs_rel, (json.dumps(line, ensure_ascii=False) + "\n").encode("utf-8"))
    ctx.print(
        "observed %s %s  id=%d%s"
        % (
            path,
            result or "-",
            line["recorded_at"],
            "  invalidates %d" % args.invalidates if args.invalidates is not None else "",
        )
    )
    return EXIT_OK


def _write_offset(ctx, mem, path, args):
    found = mem.nearest_place(path)
    if found is None:
        ctx.print(
            "refused: no ancestor of %s has a place.yaml; stand at one and `egs place save <ancestor>`"
            % path
        )
        return EXIT_REFUSED
    a_path, aid, _ = found
    if a_path == path:
        ctx.print("refused: %s has its own place.yaml; an offset from itself means nothing" % path)
        return EXIT_REFUSED
    ny_rel = posixpath.join(path, "node.yaml")
    ny, err = mem.node_yaml(path)
    if err:
        ctx.print("refused: %s; node.yaml must stay in the flat subset of memory/README.md" % err)
        return EXIT_REFUSED
    j = ctx.client.get("/anchors/%d" % aid, {"robot": 1})
    if not j.get("ok"):
        return ctx.refused(j)
    a_pose, robot = se2.from_json(j.get("pose")), se2.from_json(j.get("robot"))
    if a_pose is None:
        ctx.print("refused: %s" % explain("unresolvable", "%s (anchor %d)" % (a_path, aid)))
        return EXIT_REFUSED
    if robot is None:
        ctx.print("refused: %s" % explain("no_robot_pose"))
        return EXIT_REFUSED
    limit = float(ctx.env.get("EGS_OFFSET_MAX_M") or DEFAULT_OFFSET_MAX_M)
    dist = math.hypot(robot[0] - a_pose[0], robot[1] - a_pose[1])
    if dist > limit and not args.force:
        ctx.print(
            "refused: the robot is %.1fm from %s (anchor %d), over the %.1fm limit "
            "(EGS_OFFSET_MAX_M); stand at the object and `egs place save` a place there, "
            "or pass --force" % (dist, a_path, aid, limit)
        )
        return EXIT_REFUSED
    off = se2.compose(se2.inverse(a_pose), se2.compose(robot, tuple(args.offset)))
    data = dict(ny or {})
    if args.kind:
        data["kind"] = args.kind
    elif not data.get("kind"):
        data["kind"] = _default_kind(path)
    data["offset"] = [round(off[0], 3), round(off[1], 3), round(off[2], 4)]
    data["offset_from"] = aid
    ctx.store().write(ny_rel, dumps(data, NODE_KEY_ORDER).encode("utf-8"))
    ctx.print(
        "offset %s from %s (anchor %d) written to %s  %s"
        % (fmt.pose(off), a_path, aid, ny_rel, solve(j))
    )
    return EXIT_OK


def _default_kind(path):
    words = [w for w in re.split(r"[-_]", posixpath.basename(path)) if w]
    return words[-1] if words else "thing"


def cmd_snapshot(ctx, args):
    if args.output:
        ctx.refuse_inside_memory(args.output)
    j = ctx.client.post("/snapshot")
    if not j.get("ok"):
        return ctx.refused(j)
    files = j.get("files") or []
    src = _local_snapshot_dir(ctx, j.get("dir") or "")
    ctx.print("snapshot %s  %s" % (j.get("seq"), src), "files: " + ", ".join(files), solve(j))
    if args.output:
        if not os.path.isdir(src):
            ctx.print(
                "refused: %s is not readable here; -o needs the map directory on this machine "
                "(the fs passthrough covers memory/ only)" % src
            )
            return EXIT_REFUSED
        os.makedirs(args.output, exist_ok=True)
        for name in files:
            shutil.copy2(os.path.join(src, name), os.path.join(args.output, name))
        ctx.print("copied %d files to %s" % (len(files), os.path.abspath(args.output)))
    return EXIT_OK


def _local_snapshot_dir(ctx, served):
    """The service's dir may be container-absolute or relative to its cwd; the layout under
    map_dir is fixed, so rebuild it on the local map_dir when EGS_MEMORY gives one."""
    local = ctx.local_map_dir()
    if not served or local is None:
        return served
    return os.path.join(local, "snapshots", posixpath.basename(served.rstrip("/")))


def cmd_init_pose(ctx, args):
    if args.place:
        if args.pose:
            raise EgsError("give either x y theta or --place, not both", EXIT_USAGE)
        path = node_path(args.place, ctx.memory_dir())
        aid = ctx.memory().place_anchor(path)
        if aid is None:
            ctx.print("refused: %s has no place.yaml (init-pose --place does not walk up)" % path)
            return EXIT_REFUSED
        params = {"anchor": aid}
    elif args.pose and len(args.pose) == 3:
        params = {"x": args.pose[0], "y": args.pose[1], "theta": args.pose[2]}
    else:
        raise EgsError("init-pose takes x y theta, or --place <path>", EXIT_USAGE)
    j = ctx.client.post("/init-pose", params)
    if not j.get("ok"):
        return ctx.refused(j)
    ctx.print(
        "initial pose set to %s; not a promise: check with `egs here`  %s"
        % (fmt.pose(se2.from_json(j.get("pose"))), solve(j))
    )
    return EXIT_OK


def cmd_relocalize(ctx, args):
    j = ctx.client.post("/relocalize")
    if not j.get("ok"):
        return ctx.refused(j)
    ctx.print("global relocalization queued; check with `egs here`  %s" % solve(j))
    return EXIT_OK


def cmd_checkpoint(ctx, args):
    j = ctx.client.post("/checkpoint")
    if not j.get("ok"):
        return ctx.refused(j)
    ctx.print("checkpoint written (%s this run)  %s" % (j.get("num_checkpoints_written"), solve(j)))
    return EXIT_OK
