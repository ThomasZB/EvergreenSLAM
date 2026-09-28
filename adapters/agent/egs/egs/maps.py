"""Management layer: egs map ls|new|open. Nothing here deletes a map."""

import time

from .client import EXIT_OK, EXIT_TRANSPORT, Client, EgsError

# The swap finishes the old graph and boots the new one; the port is closed meanwhile.
SWITCH_MIN_WAIT_S = 30.0
POLL_INTERVAL_S = 0.5


def cmd_ls(ctx, args):
    j = ctx.client.get("/maps")
    if not j.get("ok"):
        return ctx.refused(j)
    for m in j.get("maps") or []:
        ctx.print("%s %s" % ("*" if m.get("current") else " ", m.get("name")))
    ctx.print("root %s" % j.get("map_root"))
    return EXIT_OK


def cmd_new(ctx, args):
    return _switch(ctx, "/maps/new", args.name)


def cmd_open(ctx, args):
    return _switch(ctx, "/maps/open", args.name, opened=True)


def _switch(ctx, path, name, opened=False):
    j = ctx.client.post(path, {"name": name})
    if not j.get("ok"):
        return ctx.refused(j)
    if not j.get("pending"):
        ctx.print("map %s is already open" % name)
        return EXIT_OK
    wait_s = max(ctx.poll_s, SWITCH_MIN_WAIT_S)
    root = _wait_for(ctx, name, wait_s)
    if root is None:
        raise EgsError(
            "outcome unknown: map %s not open after %gs; check `egs root`" % (name, wait_s),
            EXIT_TRANSPORT,
        )
    memory = root.get("memory_dir")
    ctx.print(
        "map %s open; memory/ is now %s" % (name, memory),
        "the workspace changed: cd %s again; the other map's places are not visible here" % memory,
    )
    if opened:
        # The map boots from its own last_pose.pb, not from where the robot is now.
        ctx.print(
            "if the robot moved since this map was last open, run "
            "`egs init-pose --place <p>` or `egs relocalize`"
        )
    return EXIT_OK


def _wait_for(ctx, name, wait_s):
    """Transport errors and 503 mean the process is still switching, not that it failed."""
    client = Client(ctx.client.base_url, min(ctx.client.timeout, 5.0))
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        try:
            root = client.get("/root")
            if root.get("map") == name:
                return root
        except EgsError as e:
            if e.code != EXIT_TRANSPORT:
                raise
        time.sleep(POLL_INTERVAL_S)
    return None
