"""Usage-layer commands: here, status, place save, where, find, observe, snapshot, init-pose,
relocalize, events, checkpoint, root."""

import json
import math
import os
import posixpath
import re
import shutil
import statistics
import time

from . import fmt, jsonout, se2
from .client import EXIT_OK, EXIT_REFUSED, EXIT_TRANSPORT, EXIT_USAGE, EgsError, explain
from .memory import ATTACHMENTS, RANK_NAME, Resolution, UnreadablePlace, match
from .store import node_path
from .yamlmini import dumps

NODE_KEY_ORDER = ("kind", "aliases", "labels", "offset", "offset_from")
DEFAULT_OFFSET_MAX_M = 10.0
# The service's request body limit; local copies keep it so both modes accept the same files.
MAX_ATTACHMENT_BYTES = 8 << 20
BASE_MIN_COVERAGE = 0.5
RANK_NAMES = ("name", "alias", "kind", "word", "fuzzy")
RELOCALIZATION_POLL_S = 0.5
EVENTS_POLL_S = 1.0


def solve(j):
    return "@solve %s" % j.get("at_num_solves", "-")


def cmd_root(ctx, args):
    root = ctx.root()
    local = ctx.is_local()
    url = ctx.client.base_url
    name = root.get("map") or os.path.basename(os.path.normpath(root.get("map_dir") or "-"))
    local_dir = ctx.memory_dir() if local else None
    if local:
        ctx.print(local_dir)
    else:
        ctx.print(
            "remote %s  map %s" % (url, name),
            "memory/ is on the robot: work in it with egs fs ls|tree|cat|grep|write|append|"
            "mkdir|mv|rm (paths relative to memory/)",
        )
    ctx.result(
        {
            "ok": True,
            "reason": None,
            "mode": "local" if local else "remote",
            "url": url,
            "map": name,
            "instance_id": root.get("instance_id"),
            "memory_dir": root.get("memory_dir"),
            "local_dir": local_dir,
            "readable": local,
        }
    )
    return EXIT_OK


def cmd_here(ctx, args):
    seconds = getattr(args, "sample", None)
    if seconds is not None and not 0 < seconds <= 60:
        raise EgsError("--sample takes 0 < S <= 60 seconds", EXIT_USAGE, "bad_param")
    events = _Corrections(ctx) if seconds else None
    j = ctx.client.get("/here")
    if not j.get("ok"):
        return ctx.refused(j)
    sample = None
    if seconds:
        j, sample = _sample_base(ctx, j, seconds, events)
    if j.get("has_frozen_base") and j.get("aligned_to_base") is False:
        ctx.print("linked to base: no (positions below are unreliable until linked)")
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
    ctx.print(fmt.localization(j.get("localization")))
    if sample is not None:
        ctx.print(fmt.base_sample(sample))
    if j.get("relocalization"):
        ctx.print(fmt.relocalization(j["relocalization"]))
    ctx.result(dict(j, robot=jsonout.pose(robot), base_sample=sample))
    return EXIT_OK


class _Corrections:
    def __init__(self, ctx):
        self.ctx = ctx
        self.seq = None
        self.poll()

    def poll(self):
        try:
            j = self.ctx.client.get("/events", {"since": self.seq or 0})
        except EgsError:
            return 0
        if not j.get("ok"):
            return 0
        first = self.seq is None
        self.seq = j.get("next_seq", self.seq)
        if first:
            return 0
        if j.get("lost"):
            return 1
        return sum(1 for e in j.get("events") or [] if _is_correction(e))


def _is_correction(e):
    kind = e.get("kind")
    return kind == "pose_correction" or (kind == "relocalization" and e.get("state") == "matched")


def _sample_base(ctx, j, seconds, events):
    """Reads before a correction describe the old pose: a correction restarts the sample."""
    seen, reads, restarted = set(), [], 0
    deadline = time.monotonic() + seconds
    while True:
        base = (j.get("localization") or {}).get("base")
        if base and base.get("score") is not None:
            read = (base["score"], base["coverage"])
            if read not in seen:
                seen.add(read)
                reads.append(read)
        if time.monotonic() >= deadline:
            break
        time.sleep(0.5)
        corrections = events.poll()
        if corrections:
            restarted += corrections
            reads = []
        nxt = ctx.client.get("/here")
        if nxt.get("ok"):
            j = nxt
    covered = sorted(score for score, coverage in reads if coverage >= BASE_MIN_COVERAGE)
    return j, {
        "seconds": seconds,
        "reads": len(reads),
        "covered_reads": len(covered),
        "median": statistics.median(covered) if covered else None,
        "min": covered[0] if covered else None,
        "restarted": restarted,
    }


def cmd_status(ctx, args):
    j = ctx.client.get("/status")
    if not j.get("ok"):
        return ctx.refused(j)
    yn = lambda v: "-" if v is None else ("yes" if v else "no")  # noqa: E731
    ctx.print(
        ("map %s  " % j["map"] if j.get("map") else "")
        + "boot %s  fed session %s" % (j.get("boot_count"), j.get("fed_session")),
        "frozen base %s  linked to base %s"
        % (yn(j.get("has_frozen_base")), yn(j.get("aligned_to_base"))),
        fmt.localization(j.get("localization")),
        "closures +%s since last status" % j.get("closures_delta"),
        "anchors %s (orphans %s)  place.yaml files %s"
        % (j.get("num_anchors"), j.get("num_orphans"), j.get("num_place_files")),
    )
    if j.get("relocalization"):
        ctx.print(fmt.relocalization(j["relocalization"]))
    dups = j.get("duplicate_anchor_paths") or []
    if dups:
        ctx.print("duplicate anchor (cp -r?), ignored by here: " + ", ".join(dups))
    stale = _index_mismatch(ctx, j.get("num_place_files"))
    if stale:
        ctx.print(stale)
    ctx.print(solve(j))
    ctx.result(dict(j, index_warning=stale))
    return EXIT_OK


def _index_mismatch(ctx, num_place_files):
    try:
        data = ctx.store().read("index.tsv")
    except EgsError:
        return None
    if data is None:
        return None
    rows = [l for l in data.decode("utf-8", "replace").splitlines() if l and not l.startswith("#")]
    if num_place_files is not None and len(rows) != num_place_files:
        return "index.tsv stale: %d rows, %s place.yaml files: run `egs snapshot`" % (
            len(rows),
            num_place_files,
        )
    return None


def cmd_place_save(ctx, args):
    path = node_path(args.path, _mem_dir_or_none(ctx, args.path))
    params = {"path": path, "scan": not args.no_scan}
    if args.offset is not None:
        params["offset"] = ",".join(repr(v) for v in args.offset)
    j = ctx.client.post("/place/save", params)
    if not j.get("ok"):
        code = ctx.refused(dict(j, path=path))
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
        + ("  offset %s" % fmt.pose(j["offset"]) if j.get("offset") else "")
    )
    ctx.result(dict(j, path=path))
    return EXIT_OK


def _mem_dir_or_none(ctx, raw):
    """Only an absolute path needs memory/'s location; spare relative ones the /root call."""
    return ctx.memory_dir() if os.path.isabs(raw.strip()) else None


def _existing_node(ctx, raw, create_hint=False):
    path = node_path(raw, ctx.memory_dir())
    mem = ctx.memory()
    if not mem.is_dir(path):
        hint = "paths are relative to memory/"
        if create_hint:
            hint = (
                "to remember a new thing, create it first: `egs fs mkdir %s`, then `egs fs "
                "write %s/node.yaml` with `kind: <word>` and `aliases: [...]`" % (path, path)
            )
        raise EgsError("no such node: %s (%s)" % (path, hint), EXIT_USAGE, "no_such_node")
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
    now = ctx.robot_now_ns() if r.saved_at_ns is not None else None
    obj = jsonout.resolution(r, now)
    read_first = mem.read_first(path)
    obj.update(ok=True, reason=None, at_num_solves=j.get("at_num_solves"), read_first=read_first)
    code = _print_where(ctx, path, r, found, j, now)
    if read_first:
        ctx.print("read first: " + ", ".join(read_first))
    if code == EXIT_OK:
        ctx.result(obj)
    else:
        reason = "unknown_anchor" if r.state == "unknown" else "unresolvable"
        ctx.result(dict(obj, ok=False, reason=reason))
    return code


def _print_where(ctx, path, r, found, j, now):
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
        % (r.place, fmt.pose(r.approach), r.state, fmt.age_ns(r.saved_at_ns, now), solve(j)),
    )
    return EXIT_OK


def cmd_find(ctx, args):
    query = " ".join(args.query)
    mem = ctx.memory()
    hits = []
    for rel in mem.nodes():
        ny, _ = mem.node_yaml(rel)
        m = match(query, posixpath.basename(rel), ny)
        if m is not None:
            hits.append((m, rel, ny or {}))
    if not hits:
        ctx.print("no match for %r" % query)
        ctx.result({"ok": True, "reason": None, "query": query, "rows": [], "more": 0})
        return EXIT_OK
    j = ctx.client.get("/anchors", {"robot": 1})
    if not j.get("ok"):
        return ctx.refused(j)
    table = {a["anchor"]: a for a in j.get("anchors", [])}
    robot = se2.from_json(j.get("robot"))
    rows = []
    for (rank, matched), rel, ny in hits:
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
        line = _find_row(rel, ny, r, last, robot, d, rank, matched)
        rows.append((key, line, _find_obj(rank, matched, ny, r, last, robot, d)))
    rows.sort(key=lambda x: x[0])
    limit = len(rows) if args.limit <= 0 else args.limit
    for _, line, _ in rows[:limit]:
        ctx.print(line)
    more = max(0, len(rows) - limit)
    if more:
        ctx.print("... %d more (--limit 0 for all)" % more)
    ctx.print(solve(j))
    ctx.result(
        {
            "ok": True,
            "reason": None,
            "at_num_solves": j.get("at_num_solves"),
            "query": query,
            "robot": jsonout.pose(robot),
            "rows": [obj for _, _, obj in rows[:limit]],
            "more": more,
        }
    )
    return EXIT_OK


def _find_row(rel, ny, r, last, robot, d, rank, matched):
    if r.resolved:
        prec = "%s:%s(%s)" % (r.precision, r.resolved_via, r.state)
    elif r.place:
        prec = "%s:%s" % (r.state if r.state in ("unknown", "unreadable") else "orphan", r.place)
    else:
        prec = "none"
    last_s = "last=%s %s" % (last["result"], fmt.age_ns(last["recorded_at"])) if last else "last=-"
    why = ""
    if rank != RANK_NAME:
        why = "  (%s %s)" % (RANK_NAMES[rank], json.dumps(matched, ensure_ascii=False))
    return "%s%s [%s] %s %s %s prec=%s" % (
        rel,
        why,
        ny.get("kind") or "-",
        fmt.meters(d),
        fmt.clock(robot, r.pose),
        last_s,
        prec,
    )


def _find_obj(rank, matched, ny, r, last, robot, d):
    obj = jsonout.resolution(r)
    del obj["notes"], obj["bound_children"]
    obj.update(
        kind=ny.get("kind"),
        rank=rank,
        match=RANK_NAMES[rank],
        matched={"rank": RANK_NAMES[rank], "text": matched},
        dist_m=d,
        clock=fmt.clock_hour(robot, r.pose),
        result=last["result"] if last else None,
        last_seen_age_s=jsonout.age_s(last["recorded_at"]) if last else None,
    )
    return obj


def _read_attachment(path):
    try:
        with open(path, "rb") as f:
            data = f.read(MAX_ATTACHMENT_BYTES + 1)
    except OSError as e:
        raise EgsError(
            "cannot read attachment %s: %s" % (path, e.strerror or e), EXIT_USAGE, "bad_attachment"
        )
    if len(data) > MAX_ATTACHMENT_BYTES:
        raise EgsError("attachment %s is over 8 MiB" % path, EXIT_USAGE, "too_large")
    ext = os.path.splitext(path)[1][1:].lower()
    return data, ext if re.fullmatch(r"[a-z0-9]+", ext) else "bin"


def _parse_stamp(text):
    """Unix ns, or Unix seconds with a decimal point; exact either way (no float rounding)."""
    if re.fullmatch(r"\d+", text):
        return int(text)
    if re.fullmatch(r"\d+\.\d*", text):
        whole, frac = text.split(".")
        return int(whole) * 10**9 + int((frac + "0" * 9)[:9])
    raise EgsError(
        "--at takes Unix ns (e.g. 1785000000123456789) or Unix seconds with a decimal point: %s"
        % text,
        EXIT_USAGE,
        "bad_param",
    )


def cmd_observe(ctx, args):
    if args.at is not None and args.offset is None:
        raise EgsError("--at only goes with --offset", EXIT_USAGE)
    observed_at = _parse_stamp(args.at) if args.at is not None else None
    path, mem = _existing_node(ctx, args.path, create_hint=True)
    attachment = _read_attachment(args.attach) if args.attach else None
    result = args.result or (None if args.invalidates is not None else "seen")
    obs_rel = posixpath.join(path, "observations.jsonl")
    offset = None
    if args.offset is not None:
        code, offset = _write_offset(ctx, mem, path, args, observed_at)
        if code != EXIT_OK:
            return code
    last = max((o["recorded_at"] for o in mem.observations(path)), default=0)
    line = {"recorded_at": max(fmt.now_ns(), last + 1)}
    if observed_at is not None:
        line["observed_at"] = observed_at
    line["source"] = args.source
    if result:
        line["result"] = result
    if args.invalidates is not None:
        line["invalidates"] = args.invalidates
    if args.note:
        line["note"] = args.note
    if attachment is not None:
        data, ext = attachment
        rel = "%s/%d.%s" % (ATTACHMENTS, line["recorded_at"], ext)
        ctx.store().mkdir(posixpath.join(path, ATTACHMENTS))
        ctx.store().write(posixpath.join(path, rel), data)
        line["attachment"] = rel
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
    ctx.result(
        {
            "ok": True,
            "reason": None,
            "path": path,
            "id": line["recorded_at"],
            "line": line,
            "attachment": line.get("attachment"),
            "offset": offset,
        }
    )
    return EXIT_OK


def _write_offset(ctx, mem, path, args, observed_at):
    found = mem.nearest_place(path)
    if found is None:
        return (
            ctx.refuse(
                "no_bound_ancestor",
                "no ancestor of %s has a place.yaml; %s" % (path, _save_spot_first(path)),
            ),
            None,
        )
    a_path, aid, _ = found
    if a_path == path:
        return (
            ctx.refuse(
                "own_place", "%s has its own place.yaml; an offset from itself means nothing" % path
            ),
            None,
        )
    ny_rel = posixpath.join(path, "node.yaml")
    ny, err = mem.node_yaml(path)
    if err:
        return (
            ctx.refuse(
                "bad_node_yaml",
                "%s; node.yaml must stay in the flat subset of memory/README.md" % err,
            ),
            None,
        )
    params = {"robot": 1}
    if observed_at is not None:
        params["at"] = observed_at
    j = ctx.client.get("/anchors/%d" % aid, params)
    if not j.get("ok"):
        return ctx.refused(j), None
    a_pose, robot = se2.from_json(j.get("pose")), se2.from_json(j.get("robot"))
    if a_pose is None:
        text = explain("unresolvable", "%s (anchor %d)" % (a_path, aid))
        return ctx.refuse("unresolvable", text), None
    if robot is None:
        return ctx.refuse("no_robot_pose", explain("no_robot_pose")), None
    # Unaligned, the robot pose is in the fed session's own frame and a frozen anchor is not:
    # the offset would be measured between two frames. --force does not cover this.
    if j.get("has_frozen_base") and j.get("aligned_to_base") is False:
        return (
            ctx.refuse(
                "not_aligned",
                "not linked to the base map yet, so the robot pose and %s's anchor are in "
                "different frames; drive where the maps overlap until `egs status` says linked "
                "to base yes, then retry" % a_path,
            ),
            None,
        )
    limit = float(ctx.env.get("EGS_OFFSET_MAX_M") or DEFAULT_OFFSET_MAX_M)
    dist = math.hypot(robot[0] - a_pose[0], robot[1] - a_pose[1])
    if dist > limit and not args.force:
        text = (
            "the robot is %.1fm from %s (anchor %d), over the %.1fm limit (EGS_OFFSET_MAX_M); "
            "%s, or pass --force" % (dist, a_path, aid, limit, _save_spot_first(path))
        )
        return ctx.refuse("too_far", text, dist_m=dist, limit_m=limit), None
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
    written = {
        "offset": data["offset"],
        "offset_from": aid,
        "place": a_path,
        "node_yaml": ny_rel,
        "at_num_solves": j.get("at_num_solves"),
    }
    return EXIT_OK, written


def _save_spot_first(path):
    parent, name = posixpath.dirname(path), posixpath.basename(path)
    return (
        "save the spot you stand at as a place first (`egs place save %s/<spot>`), "
        "move %s under it (`egs fs mv %s %s/<spot>/%s`), then observe it there"
        % (parent, name, path, parent, name)
    )


def _default_kind(path):
    words = [w for w in re.split(r"[-_]", posixpath.basename(path)) if w]
    return words[-1] if words else "thing"


def cmd_snapshot(ctx, args):
    if args.output:
        ctx.refuse_inside_memory(args.output)
    j = ctx.client.post("/snapshot")
    if not j.get("ok"):
        return ctx.refused(j)
    files, seq = j.get("files") or [], j.get("seq")
    local = _local_snapshot_dir(ctx, j.get("dir") or "")
    ctx.print(
        "snapshot %s  %s" % (seq, local or j.get("dir")), "files: " + ", ".join(files), solve(j)
    )
    if local is None and not args.output:
        ctx.print("the files are on the robot: -o DIR downloads them")
    copied = None
    if args.output:
        os.makedirs(args.output, exist_ok=True)
        for name in files:
            dest = os.path.join(args.output, name)
            if local is not None and os.path.isdir(local):
                shutil.copy2(os.path.join(local, name), dest)
            else:
                with open(dest, "wb") as f:
                    f.write(_snapshot_file(ctx, seq, name))
        copied = os.path.abspath(args.output)
        ctx.print("copied %d files to %s" % (len(files), copied))
    ctx.result(dict(j, local_dir=local, copied_to=copied))
    return EXIT_OK


def _local_snapshot_dir(ctx, served):
    """The service's dir may be container-absolute or relative to its cwd; the layout under
    map_dir is fixed, so rebuild it on the local map_dir. None: the files are remote."""
    local = ctx.local_map_dir()
    if not served or local is None:
        return None
    return os.path.join(local, "snapshots", posixpath.basename(served.rstrip("/")))


def _snapshot_file(ctx, seq, name):
    resp = ctx.client.request("GET", "/snapshots/file", {"seq": seq, "name": name})
    if resp.status == 200 and not resp.is_json:
        return resp.body
    ctx.client.check(resp)
    raise EgsError("unexpected answer for snapshot file %s" % name, EXIT_REFUSED)


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
    queued = "initial pose %s sent" % fmt.pose(se2.from_json(j.get("pose")))
    return _await_relocalization(ctx, args, j, queued)


def cmd_relocalize(ctx, args):
    j = ctx.client.post("/relocalize")
    if not j.get("ok"):
        return ctx.refused(j)
    return _await_relocalization(ctx, args, j, "global relocalization sent")


def _await_relocalization(ctx, args, j, queued):
    rid = j.get("request_id")
    if args.no_wait or rid is None:
        ctx.print(
            "%s%s; not a promise: check with `egs here`  %s"
            % (queued, "" if rid is None else " as request %s" % rid, solve(j))
        )
        ctx.result(j)
        return EXIT_OK
    deadline = time.monotonic() + max(0.0, args.wait)
    while True:
        h = ctx.client.get("/here", {"relocalization": rid})
        r = h.get("relocalization") if h.get("ok") else None
        if r is None or r.get("id") != rid:
            raise EgsError(
                "outcome unknown: the process no longer tracks request %s (restarted?); "
                "check with `egs here`" % rid,
                EXIT_TRANSPORT,
                "outcome_unknown",
            )
        if r.get("state") != "pending":
            break
        left = deadline - time.monotonic()
        if left <= 0:
            raise EgsError(
                "outcome unknown: request %s still pending after %gs and may yet apply; "
                "check with `egs here`" % (rid, args.wait),
                EXIT_TRANSPORT,
                "outcome_unknown",
            )
        time.sleep(min(RELOCALIZATION_POLL_S, left))
    ctx.print(_relocalization_outcome(r, h.get("aligned_to_base")))
    if r["state"] == "matched":
        ctx.result(dict(j, relocalization=r))
        return EXIT_OK
    ctx.result(dict(j, ok=False, reason=r["state"], relocalization=r))
    return EXIT_REFUSED


def _relocalization_outcome(r, linked):
    state = r.get("state")
    if state == "matched":
        return "matched: corrected %s; check egs here" % fmt.correction(r)
    if state == "superseded":
        if r.get("superseded_by") is None:
            return "superseded: the fed session was dropped; send it again"
        return "superseded by request %s" % r["superseded_by"]
    n = r.get("candidates") or 0
    if n == 0:
        return (
            "no_match (0 candidates): nothing saved lies around that pose; the place or pose is "
            "outside the saved map"
        )
    if not linked:
        return (
            "no_match (%d candidates): the scan fits none of them; move or turn the robot to face "
            "mapped structure and send it again, or egs init-pose --place <path> if it stands at a "
            "saved place" % n
        )
    return (
        "no_match (%d candidates): the scan fits none of them; if the robot really is there, the "
        "fed session holds a wrong constraint: egs session rm <fed id> (plan, then --yes), then "
        "send it again" % n
    )


def cmd_events(ctx, args):
    since = args.since
    while True:
        j = ctx.client.get("/events", {"since": since})
        if not j.get("ok"):
            return ctx.refused(j)
        if j.get("lost"):
            ctx.warn(
                "events lost: the log no longer holds %s (overrun or restarted); from %s on"
                % (since, j.get("first_seq"))
            )
        if not args.follow:
            for e in j.get("events") or []:
                ctx.print(fmt.event(e))
            ctx.print("next_seq %s" % j.get("next_seq"))
            ctx.result(j)
            return EXIT_OK
        for e in j.get("events") or []:
            line = json.dumps(e, ensure_ascii=False) if ctx.json else fmt.event(e)
            ctx.out.write(line + "\n")
            ctx.out.flush()
        since = j.get("next_seq", since)
        time.sleep(EVENTS_POLL_S)


def cmd_checkpoint(ctx, args):
    j = ctx.client.post("/checkpoint")
    if not j.get("ok"):
        return ctx.refused(j)
    ctx.print("checkpoint written (%s this run)  %s" % (j.get("num_checkpoints_written"), solve(j)))
    ctx.result(j)
    return EXIT_OK
