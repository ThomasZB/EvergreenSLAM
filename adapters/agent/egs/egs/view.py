"""egs view: one PNG rendered by the service, saved under <map_dir>/views/."""

import os
import tempfile

from .client import EXIT_OK, EXIT_USAGE, EgsError, explain
from .store import node_path

PRESETS = ("map", "here", "route", "trail", "session", "custom")
MAX_LAYERS = 4
DEFAULT_EGO_M = 7.5


def cmd_view(ctx, args):
    if args.output:
        ctx.refuse_inside_memory(args.output)
    params = {"preset": args.preset}
    target, session = args.target, args.session
    if args.arg is not None:
        if args.preset == "route" and target is None:
            target = args.arg
        elif args.preset == "session" and session is None:
            session = args.arg
        else:
            raise EgsError(
                "unexpected argument %r for view %s" % (args.arg, args.preset), EXIT_USAGE
            )
    if args.preset == "route" and not target:
        raise EgsError("view route needs a place: egs view route places/<path>", EXIT_USAGE)
    if target:
        base = ctx.memory_dir() if os.path.isabs(target.strip()) else None
        params["target"] = node_path(target, base)
    if session is not None:
        params["session"] = session
    if args.preset == "custom":
        if not args.layers:
            raise EgsError("view custom needs --layers a,b,... (at most 4)", EXIT_USAGE)
        layers = [l.strip() for l in args.layers.split(",") if l.strip()]
        if len(layers) > MAX_LAYERS:
            raise EgsError(explain("too_many_layers", ",".join(layers)), EXIT_USAGE)
        params["layers"] = ",".join(layers)
    elif args.layers:
        raise EgsError("--layers only goes with view custom", EXIT_USAGE)
    ego = args.ego
    if ego is not None and ego < 0:
        raise EgsError(
            explain("bad_param", "--ego must be >= 0 (0 = no crop)"), EXIT_USAGE, "bad_param"
        )
    if ego is not None and args.full:
        raise EgsError("give --ego R or --full, not both", EXIT_USAGE)
    implicit_ego = ego is None and args.preset == "here"
    if implicit_ego:
        ego = DEFAULT_EGO_M
    if ego is not None and ego > 0 and not args.full:
        params["ego"] = ego

    resp = ctx.client.request("GET", "/view", params)
    if resp.is_json:
        j = ctx.client.check(resp)
        if not (implicit_ego and "ego" in params and j.get("reason") == "no_robot_pose"):
            return ctx.refused(j)
        # Without a pose the default crop has no centre: show the whole map, robot and scan
        # dropped as the legend says, like before the crop became the default.
        del params["ego"]
        resp = ctx.client.request("GET", "/view", params)
        if resp.is_json:
            return ctx.refused(ctx.client.check(resp))
    if resp.status != 200:
        ctx.client.check(resp)
    h = resp.headers
    ctx.print("layers=%s  @solve %s" % (h.get("X-EGS-Layers", "-"), h.get("X-EGS-Solve", "-")))
    legend, dropped = [], []
    for item in (h.get("X-EGS-Legend") or "").split(";"):
        item = item.strip()
        # The header line above already names the layers; `dropped=` still prints.
        if item and not item.startswith("layers="):
            ctx.print(item)
            if item.startswith("dropped="):
                dropped = item[len("dropped=") :].split(",")
            else:
                legend.append(item)
    png = _local_copy(ctx, args, h.get("X-EGS-View") or "", resp.body)
    ctx.print(png)
    solve = h.get("X-EGS-Solve")
    ctx.result(
        {
            "ok": True,
            "reason": None,
            "at_num_solves": int(solve) if solve and solve.isdigit() else None,
            "layers": [l for l in (h.get("X-EGS-Layers") or "").split(",") if l],
            "legend": legend,
            "dropped": dropped,
            "ego_m": params.get("ego"),
            "png": png,
        }
    )
    return EXIT_OK


def _local_copy(ctx, args, view, png):
    """The server's file when it is visible here, else a local copy of the bytes."""
    if args.output:
        _write(args.output, png)
        return os.path.abspath(args.output)
    path = view
    if view and not os.path.isabs(view):
        try:
            base = ctx.map_dir()
        except EgsError:
            base = None
        if base:
            path = os.path.join(base, view if "/" in view else os.path.join("views", view))
    if path and os.path.isfile(path):
        return path
    name = os.path.basename(view) or "view_%s.png" % args.preset
    local = os.path.join(tempfile.gettempdir(), "egs-views", name)
    _write(local, png)
    return local


def _write(path, data):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "wb") as f:
        f.write(data)
