#!/usr/bin/env python3
"""Samples graph size and process cost of a running EvergreenSLAM into a CSV, for the bounded-growth
curve. Python 3 standard library only; run it next to the node and leave it running across reboots.

  python3 tools/log_growth.py --map_dir ~/maps/default --out growth.csv [--interval 60]

Rows are appended; a sample taken while the service or the process is down is skipped. Only
/sessions is read: /status would consume the closures_delta that `egs status` reports. A restart
shows as a new pid.
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import time
import urllib.request

COLUMNS = [
    "unix_s", "at_num_solves", "sessions", "frozen_sessions", "fed",
    "nodes", "submaps", "fed_nodes", "fed_submaps", "anchors", "pid", "rss_mb", "cpu_s",
    "map_dir_mb",
]


def get(url, path):
    with urllib.request.urlopen(url + path, timeout=10) as r:
        return json.load(r)


def find_process(pattern):
    """(pid, rss_mb, cpu_s) of the largest match: `ros2 run` leaves a Python wrapper with the
    same command line."""
    out = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True).stdout.split()
    best = None
    for pid in (int(p) for p in out if int(p) != os.getpid()):
        cost = process_cost(pid)
        if cost is not None and (best is None or cost[0] > best[1]):
            best = (pid,) + cost
    return best


def cpu_seconds(text):
    # ps time: [[dd-]hh:]mm:ss[.cc]
    days, _, rest = text.rpartition("-")
    parts = [float(p) for p in rest.split(":")]
    seconds = 0.0
    for p in parts:
        seconds = seconds * 60 + p
    return seconds + (int(days) * 86400 if days else 0)


def process_cost(pid):
    out = subprocess.run(["ps", "-o", "rss=", "-o", "time=", "-p", str(pid)],
                         capture_output=True, text=True).stdout.split()
    if len(out) != 2:
        return None
    return int(out[0]) / 1024.0, cpu_seconds(out[1])


def dir_mb(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.lstat(os.path.join(root, f)).st_size
            except OSError:
                pass
    return total / 1e6


def sample(args):
    sessions = get(args.url, "/sessions")
    if not sessions.get("ok"):
        return None
    proc = find_process(args.process)
    if proc is None:
        return None
    rows = sessions.get("sessions") or []
    fed = [s for s in rows if s.get("role") == "fed"]
    return {
        "unix_s": int(time.time()),
        "at_num_solves": sessions.get("at_num_solves"),
        "sessions": len(rows),
        "frozen_sessions": sum(1 for s in rows if s.get("role") == "frozen"),
        "fed": sessions.get("fed"),
        "nodes": sum(s.get("nodes", 0) for s in rows),
        "submaps": sum(s.get("submaps", 0) for s in rows),
        "fed_nodes": fed[0].get("nodes") if fed else 0,
        "fed_submaps": fed[0].get("submaps") if fed else 0,
        "anchors": sum(s.get("anchors", 0) for s in rows),
        "pid": proc[0],
        "rss_mb": round(proc[1], 1),
        "cpu_s": round(proc[2], 1),
        "map_dir_mb": round(dir_mb(args.map_dir), 2) if args.map_dir else "",
    }


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", required=True, help="CSV file, appended to")
    p.add_argument("--map_dir", help="the node's map directory, for its size on disk")
    p.add_argument("--interval", type=float, default=60.0, help="seconds between samples")
    p.add_argument("--url", default=os.environ.get("EGS_URL", "http://127.0.0.1:8643"))
    p.add_argument("--process", default="evergreenslam_(node|bag)",
                   help="pgrep -f pattern of the SLAM process")
    args = p.parse_args()

    new_file = not os.path.exists(args.out) or os.path.getsize(args.out) == 0
    with open(args.out, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        if new_file:
            writer.writeheader()
        while True:
            try:
                row = sample(args)
            except (OSError, ValueError) as e:
                row = None
                print("skip: %s" % e, file=sys.stderr)
            if row is not None:
                writer.writerow(row)
                f.flush()
            time.sleep(args.interval)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
