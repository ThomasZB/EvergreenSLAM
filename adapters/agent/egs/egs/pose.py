"""egs pose: the robot's pose from the newest scan (GET /pose), once or streamed."""

import http.client
import json
import time
import urllib.parse

from . import fmt, jsonout, se2
from .client import EXIT_OK, EXIT_USAGE, EgsError

DEFAULT_HZ = 10.0
MAX_HZ = 30.0
MAX_BACKOFF_S = 5.0


def _sleep(seconds):
    time.sleep(seconds)


def _obj(j):
    return dict(j, pose=jsonout.pose(se2.from_json(j.get("pose"))))


def _line(j):
    v = j.get("velocity")
    vel = "-" if v is None else "%.2f,%.2f,%.2f" % (v["vx"], v["vy"], v["omega"])
    age = j.get("age_s")
    return "pose %s  vel %s  seq %s  stamp_ns %s  age %s" % (
        fmt.pose(se2.from_json(j.get("pose"))),
        vel,
        "-" if j.get("seq") is None else j["seq"],
        "-" if j.get("stamp_ns") is None else j["stamp_ns"],
        "-" if age is None else "%.2fs" % age,
    )


def cmd_pose(ctx, args):
    if args.watch is None:
        j = ctx.client.get("/pose")
        if not j.get("ok"):
            return ctx.refused(j)
        ctx.print(_line(j))
        ctx.result(_obj(j))
        return EXIT_OK
    if not 0 < args.watch <= MAX_HZ:
        raise EgsError("--watch takes 0 < HZ <= %g" % MAX_HZ, EXIT_USAGE)
    return _watch(ctx, args.watch)


def _get(conn):
    conn.request("GET", "/pose")
    r = conn.getresponse()
    body = r.read()
    if r.status != 200:
        raise http.client.HTTPException("HTTP %d" % r.status)
    return json.loads(body.decode("utf-8"))


def _watch(ctx, hz):
    url = urllib.parse.urlsplit(ctx.client.base_url)
    period = 1.0 / hz
    conn = None
    last_seq = None
    failures = 0
    waiting_noted = False
    try:
        while True:
            started = time.monotonic()
            try:
                if conn is None:
                    conn = http.client.HTTPConnection(
                        url.hostname, url.port or 80, timeout=ctx.client.timeout
                    )
                j = _get(conn)
            except (OSError, ValueError, http.client.HTTPException) as e:
                if conn is not None:
                    conn.close()
                conn = None
                delay = min(MAX_BACKOFF_S, period * 2**failures)
                failures += 1
                ctx.warn(
                    "egs pose: %s (%s); reconnecting in %.1fs" % (ctx.client.base_url, e, delay)
                )
                _sleep(delay)
                continue
            if failures:
                ctx.warn("egs pose: reconnected")
                failures = 0
            seq = j.get("seq")
            if seq is None and not waiting_noted:
                ctx.warn("egs pose: no scan yet")
                waiting_noted = True
            if seq is not None and seq != last_seq:
                last_seq = seq
                if ctx.json:
                    ctx.out.write(json.dumps(_obj(j)) + "\n")
                else:
                    ctx.out.write(_line(j) + "\n")
                ctx.out.flush()
            _sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        ctx.emitted = True
        return EXIT_OK
    finally:
        if conn is not None:
            conn.close()
