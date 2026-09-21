#!/usr/bin/env python3
"""Score a trajectory against a Freiburg .relations ground-truth file.

Relations: t1 t2 dx dy dz droll dpitch dyaw, the true pose(t1)->pose(t2) transform in
pose(t1)'s frame. Error per relation = estimated relative transform minus the true one
(Burgard et al. 2009 metric). Prints translational and angular RMS.

usage: eval_relations.py <relations> <csv> <kind>
  kind = odom  -> csv is *_trajectory.csv (stamp_ns, est_x, est_y, est_theta, ...)
  kind = nodes -> csv is *_nodes.csv (stamp_ns, x, y, theta)
"""

import bisect
import math
import sys


def load_csv(path, kind):
    stamps, xs, ys, ts = [], [], [], []
    with open(path) as f:
        next(f)
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 4:
                continue
            stamps.append(int(parts[0]) / 1e9)
            xs.append(float(parts[1]))
            ys.append(float(parts[2]))
            ts.append(float(parts[3]))
    return stamps, xs, ys, ts


def interpolate(stamps, xs, ys, ts, t):
    i = bisect.bisect_left(stamps, t)
    if i == 0:
        return (xs[0], ys[0], ts[0]) if t > stamps[0] - 0.5 else None
    if i >= len(stamps):
        return (xs[-1], ys[-1], ts[-1]) if t < stamps[-1] + 0.5 else None
    t0, t1 = stamps[i - 1], stamps[i]
    if t1 - t0 > 2.0:  # keyframes can be sparse when stationary; do not bridge real gaps
        return None
    a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    dyaw = math.atan2(math.sin(ts[i] - ts[i - 1]), math.cos(ts[i] - ts[i - 1]))
    return (
        xs[i - 1] + a * (xs[i] - xs[i - 1]),
        ys[i - 1] + a * (ys[i] - ys[i - 1]),
        ts[i - 1] + a * dyaw,
    )


def relative(p1, p2):
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    c, s = math.cos(-p1[2]), math.sin(-p1[2])
    return (c * dx - s * dy, s * dx + c * dy,
            math.atan2(math.sin(p2[2] - p1[2]), math.cos(p2[2] - p1[2])))


def main():
    relations_path, csv_path, kind = sys.argv[1], sys.argv[2], sys.argv[3]
    stamps, xs, ys, ts = load_csv(csv_path, kind)
    sum2_t, sum2_r, n, skipped = 0.0, 0.0, 0, 0
    with open(relations_path) as f:
        for line in f:
            parts = line.split()
            if len(parts) < 8:
                continue
            t1, t2 = float(parts[0]), float(parts[1])
            true_rel = (float(parts[2]), float(parts[3]), float(parts[7]))
            p1 = interpolate(stamps, xs, ys, ts, t1)
            p2 = interpolate(stamps, xs, ys, ts, t2)
            if p1 is None or p2 is None:
                skipped += 1
                continue
            est_rel = relative(p1, p2)
            et = math.hypot(est_rel[0] - true_rel[0], est_rel[1] - true_rel[1])
            er = math.atan2(math.sin(est_rel[2] - true_rel[2]), math.cos(est_rel[2] - true_rel[2]))
            sum2_t += et * et
            sum2_r += er * er
            n += 1
    if n == 0:
        sys.exit("no relation could be evaluated")
    print(f"{csv_path} vs {relations_path}: {n} relations ({skipped} skipped)")
    print(f"translational rms {math.sqrt(sum2_t / n):.4f} m")
    print(f"angular rms {math.sqrt(sum2_r / n):.4f} rad")


if __name__ == "__main__":
    main()
