"""Management layer: egs session ls|freeze|new|rm. Destructive operations are plan -> apply."""

import time

from . import fmt
from .client import EXIT_OK, EXIT_REFUSED, EgsError, explain

PLAN_ONLY = "plan only: add --yes to apply"


def cmd_ls(ctx, args):
    j = ctx.client.get("/sessions")
    if not j.get("ok"):
        return ctx.refused(j)
    ctx.print("fed %s  %s" % (j.get("fed"), "@solve %s" % j.get("at_num_solves", "-")))
    for s in j.get("sessions") or []:
        anchored = s.get("anchored")
        ctx.print(
            "%s %s  nodes %s  submaps %s  anchors %s  phase %s  anchored %s  started %s  last node %s"
            % (
                s.get("id"),
                s.get("role"),
                s.get("nodes"),
                s.get("submaps"),
                s.get("anchors"),
                s.get("phase") or "-",
                "-" if anchored is None else ("yes" if anchored else "no"),
                fmt.age_ns(s.get("start_ns")),
                fmt.age_ns(s.get("last_node_ns")),
            )
        )
    return EXIT_OK


def _anchor_paths(ctx, anchor_ids):
    """anchor id -> [place paths], best effort: the plan must print even if memory/ is not."""
    if not anchor_ids:
        return {}
    try:
        mem = ctx.memory()
        out = {}
        for rel in mem.nodes():
            try:
                aid = mem.place_anchor(rel)
            except EgsError:
                continue
            if aid in anchor_ids:
                out.setdefault(aid, []).append(rel)
        return out
    except EgsError:
        return {}


def _print_plan(ctx, title, plan):
    orphaned = plan.get("anchors_orphaned") or []
    paths = _anchor_paths(ctx, set(orphaned))
    deletes = plan.get("would_delete") or []
    shown = " ".join(fmt.submap(s) for s in deletes[:8]) + (" ..." if len(deletes) > 8 else "")
    ctx.print(
        "plan: " + title,
        "  sessions_affected: " + fmt.ids(plan.get("sessions_affected")),
        "  would_delete: %d submaps %s" % (len(deletes), shown) if deletes else "  would_delete: -",
        "  anchors_orphaned: "
        + (
            ", ".join(
                "%s %s" % (a, ",".join(paths.get(a, [])) or "(no place.yaml)") for a in orphaned
            )
            or "-"
        ),
        "  @solve %s" % plan.get("at_num_solves", "-"),
    )


def _plan_ok(ctx, plan):
    if plan.get("rejection"):
        ctx.print("rejected: " + explain(plan["rejection"]))
        return False
    if not plan.get("ok"):
        ctx.refused(plan)
        return False
    return True


def cmd_freeze(ctx, args):
    params = {"force": bool(args.force)}
    plan = ctx.client.post("/sessions/freeze/plan", params)
    fed = (plan.get("sessions_affected") or ["?"])[0]
    what = "freeze fed session %s, then feed a fresh one" % fed
    _print_plan(ctx, what, plan)
    if not _plan_ok(ctx, plan):
        return EXIT_REFUSED
    if not args.yes:
        ctx.print(PLAN_ONLY)
        return EXIT_OK
    params["plan_token"] = plan.get("plan_token")
    j = ctx.client.post("/sessions/freeze/apply", params)
    if not j.get("ok"):
        return ctx.refused(j, "not applied")
    sid = j.get("frozen_session")
    deadline = time.monotonic() + ctx.poll_s
    while True:
        s = ctx.client.get("/sessions")
        role = {x.get("id"): x.get("role") for x in s.get("sessions") or []}.get(sid)
        if role == "frozen":
            ctx.print(
                "session %s frozen; fed session now %s  @solve %s"
                % (sid, s.get("fed"), s.get("at_num_solves"))
            )
            return EXIT_OK
        if time.monotonic() >= deadline:
            break
        time.sleep(0.25)
    ctx.print(
        "freeze of session %s queued but not visible after %gs: check `egs session ls`"
        % (sid, ctx.poll_s)
    )
    return EXIT_REFUSED


def cmd_new(ctx, args):
    return cmd_freeze(ctx, args)


def cmd_rm(ctx, args):
    plan = ctx.client.post("/sessions/rm/plan", {"id": args.id})
    _print_plan(ctx, "remove floating session %s" % args.id, plan)
    if not _plan_ok(ctx, plan):
        return EXIT_REFUSED
    if not args.yes:
        ctx.print(PLAN_ONLY)
        return EXIT_OK
    j = ctx.client.post("/sessions/rm/apply", {"id": args.id, "plan_token": plan.get("plan_token")})
    if not j.get("ok"):
        return ctx.refused(j, "not applied")
    orphaned = j.get("anchors_orphaned") or []
    ctx.print(
        "removed session %s; anchors orphaned: %s  @solve %s"
        % (j.get("removed", args.id), fmt.ids(orphaned), j.get("at_num_solves"))
    )
    if orphaned:
        ctx.print("their places resolve to nothing now: stand there and `egs place save` again")
    return EXIT_OK
