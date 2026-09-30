"""egs: the bridge from an agent's shell to the running EvergreenSLAM process."""

import argparse
import sys

from . import commands, fscmd, maps, sessions, view, zones
from .client import EXIT_OK, EXIT_REFUSED, EXIT_TRANSPORT, EXIT_USAGE, EgsError
from .context import Ctx

DESCRIPTION = """\
egs is the bridge from your shell to the running SLAM process. memory/ is a plain directory
tree you work in with ls/find/grep/cat/mv; egs only answers what the live process alone knows:
coordinates, saving a place, maps and sessions. Read memory/README.md before changing memory/.
All paths are relative to memory/ and start with places/."""

EPILOG = """\
environment:
  EGS_URL      service address (default http://127.0.0.1:8643)
  EGS_MEMORY   memory/ directory; default: ask the process (GET /root). When readable, its
               parent is the map directory view and snapshot print paths in
  EGS_TIMEOUT  seconds per request (default 30)
  EGS_OFFSET_MAX_M  observe --offset and zone add refuse beyond this robot-to-place distance
               (default 10)

memory/ access: when the memory directory is not readable on this machine, every tree read and
write egs does (where, find, observe, fs, ...) goes through the process's /fs endpoints instead,
transparently and with the same rules. `egs fs` is the shell stand-in for that case.

node.yaml must stay in the flat YAML subset of memory/README.md (key: value, flow lists [a, b],
numbers, strings); egs refuses to rewrite anything else, and rewriting drops comments.

Timestamps print as ages (35s, 20m, 6d). A mutating command that gets no answer within the
timeout prints 'outcome unknown': check with egs status / here / session ls before retrying.

--json (usage commands, map ls, session ls): one JSON object instead of text; prefer it to parsing.

exit codes: 0 done (a session plan printed without --yes is done too), 1 refused or plan
rejected (nothing changed unless stated), 2 usage or bad path, 3 process unreachable, not
started, or outcome unknown.
There is no goto: egs never moves the robot; hand `approach` from `egs where` to your navigator."""

AGENT_HELP = """\
1. no map-frame XY in any file you write; ask `egs where` for a current position
2. place.yaml belongs to the process: never write, edit or cp -r it; stand there, `egs place save`
3. things you can stand in front of are places; small movable things: node.yaml, no place.yaml
4. grep -r always takes --include=node.yaml; to find an object use `egs find`
5. egs paths are relative to memory/ and start with places/
6. skills and attachments are reserved names, never a place or a thing; no symbolic links
7. observations.jsonl is append-only; retract a line with `egs observe <path> --invalidates <id>`
8. there is no egs goto: egs never moves the robot

where am I -> egs here
where is a place or a thing -> egs where places/<path>
find a thing by name, alias or kind -> egs find <words>
remember the spot the robot stands at -> egs place save places/<path> [--offset DX DY 0, <= 3 m]
I saw it / it is not there -> egs observe places/<path> [--result absent] [--note ..] [--attach F]
it is DX ahead, DY left of the robot -> egs observe places/<path> --offset DX DY 0 (within 10 m)
go somewhere -> egs where places/<path>, give its approach pose to your navigator
the navigator must avoid an area -> egs zone add places/<place>/<name> --rect X0 Y0 X1 Y1 (ls | rm)
is the map healthy, aligned to the base -> egs status
a picture -> egs view map | here | route places/<p> | trail | session [id]
export the map as files -> egs snapshot [-o DIR] (DIR outside memory/)
the robot's pose is wrong -> egs init-pose --place places/<p>, then egs here; else egs relocalize
make sure it is on disk now -> egs checkpoint
a broken session (a fall) -> egs session rm <fed id> --yes: drops it in process; pose lost
new container, rename, re-parent, delete -> mkdir, mv, rm -r (bindings move with directories)
notes and skills -> cat > f.tmp then mv f.tmp notes.md (or SKILL.md)

exit 0 done, 1 refused or plan rejected, 2 bad usage or path, 3 unreachable or outcome unknown
outcome unknown: a change may have happened; check egs status / here / session ls, then retry
"""

JSON_HELP = "print one JSON object instead of text"
EXIT_REASONS = {EXIT_REFUSED: "refused", EXIT_USAGE: "usage", EXIT_TRANSPORT: "transport"}


class _Parser(argparse.ArgumentParser):
    def error(self, message):
        raise EgsError("%s: %s (see %s --help)" % (self.prog, message, self.prog), EXIT_USAGE)


def _json_flag(sp):
    # SUPPRESS: an unset subcommand flag must not overwrite `egs --json <cmd>`.
    sp.add_argument("--json", action="store_true", default=argparse.SUPPRESS, help=JSON_HELP)
    sp.set_defaults(json_ok=True)
    return sp


def _cmd_help(parser):
    def run(ctx, args):
        if args.topic == "agent":
            ctx.out.write(AGENT_HELP)
        else:
            parser.print_help(file=ctx.out)
        return EXIT_OK

    return run


def build_parser():
    p = _Parser(
        prog="egs",
        description=DESCRIPTION,
        epilog=EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--json", action="store_true", default=False, help=JSON_HELP)
    sub = p.add_subparsers(dest="cmd", metavar="<command>", parser_class=_Parser)
    sub.required = True

    def add(name, fn, help_, **kw):
        sp = sub.add_parser(name, help=help_, description=help_, **kw)
        sp.set_defaults(fn=fn)
        return sp

    _json_flag(add("root", commands.cmd_root, "print the absolute path of memory/"))
    _json_flag(add("here", commands.cmd_here, "where the robot is: current place, two closest, pose"))
    _json_flag(
        add("status", commands.cmd_status, "map health: frozen base, alignment, phase, closures")
    )

    place = sub.add_parser("place", help="save the place the robot stands at")
    psub = place.add_subparsers(dest="sub", metavar="<save>", parser_class=_Parser)
    psub.required = True
    ps = psub.add_parser("save", help="bind <path> to the robot's current pose (stand there first)")
    ps.add_argument("path", help="places/...; re-saving keeps the anchor id and updates the pose")
    ps.add_argument("--no-scan", action="store_true", help="do not keep the keyframe scan")
    ps.add_argument(
        "--offset",
        nargs=3,
        type=float,
        metavar=("DX", "DY", "DTHETA"),
        help="bind a pose this far from the robot instead (x forward, y left; m, rad; "
        "within 3 m, in free space the robot can see)",
    )
    ps.set_defaults(fn=commands.cmd_place_save)
    _json_flag(ps)

    w = add(
        "where", commands.cmd_where, "position of a place or thing, precision and approach pose"
    )
    w.add_argument("path")
    _json_flag(w)

    f = add(
        "find", commands.cmd_find, "find things: name > alias > kind > word > fuzzy; absent last"
    )
    f.add_argument("query", nargs="+")
    f.add_argument("--limit", type=int, default=10, help="rows to print (0 = all; default 10)")
    _json_flag(f)

    o = add(
        "observe", commands.cmd_observe, "append an observation; --offset also writes node.yaml"
    )
    o.add_argument("path")
    o.add_argument("--result", choices=("seen", "absent", "not_observed"), help="default seen")
    o.add_argument("--note")
    o.add_argument("--source", choices=("agent", "human"), default="agent")
    o.add_argument("--invalidates", type=int, metavar="ID", help="retract an earlier line by id")
    o.add_argument(
        "--offset",
        nargs=3,
        type=float,
        metavar=("DX", "DY", "DTHETA"),
        help="where the thing is in the robot's current frame (x forward, y left; m, rad)",
    )
    o.add_argument("--kind", help="node.yaml kind when writing an offset (default: from the name)")
    o.add_argument(
        "--force",
        action="store_true",
        help="write the offset even when the robot is farther than EGS_OFFSET_MAX_M from the place",
    )
    o.add_argument(
        "--attach",
        metavar="FILE",
        help="copy FILE (at most 8 MiB) to <path>/attachments/<id>.<ext> and name it in the line",
    )
    _json_flag(o)

    zone = sub.add_parser("zone", help="keep-out zones the navigator must avoid")
    zsub = zone.add_subparsers(dest="sub", metavar="<add|ls|rm>", parser_class=_Parser)
    zsub.required = True
    za = zsub.add_parser(
        "add",
        help="write <path>/zone.yaml: a keep-out polygon that moves with its nearest place",
    )
    za.add_argument("path", help="places/<place>/<name>: a new directory below the place")
    shape = za.add_mutually_exclusive_group(required=True)
    shape.add_argument(
        "--polygon",
        metavar='"X,Y X,Y X,Y ..."',
        help='at least 3 corners in order; --polygon="-1,0 ..." when the first starts with -',
    )
    shape.add_argument(
        "--rect", nargs=4, type=float, metavar=("X0", "Y0", "X1", "Y1"), help="two opposite corners"
    )
    za.add_argument(
        "--frame",
        choices=("robot", "anchor"),
        default="robot",
        help="robot (default): metres in the robot's current frame, x forward, y left; "
        "anchor: already in the place's anchor frame",
    )
    za.add_argument(
        "--force",
        action="store_true",
        help="add even when the robot is farther than EGS_OFFSET_MAX_M from the place",
    )
    za.set_defaults(fn=zones.cmd_add)
    zl = zsub.add_parser(
        "ls", help="every zone: its place, state and area, or why it is unresolvable"
    )
    _json_flag(zl)
    zl.add_argument("--xy", action="store_true", help="also the map-frame polygon (never store it)")
    zl.set_defaults(fn=zones.cmd_ls)
    zr = zsub.add_parser("rm", help="delete <path>/zone.yaml (and the directory once empty)")
    zr.add_argument("path")
    zr.set_defaults(fn=zones.cmd_rm)

    s = add("snapshot", commands.cmd_snapshot, "export map, trajectory, places; refresh index.tsv")
    s.add_argument("-o", "--output", metavar="DIR", help="also copy the files to DIR")
    _json_flag(s)

    v = add(
        "view",
        view.cmd_view,
        "render one picture: map | here | route <place> | trail | session [id] | custom --layers",
    )
    v.add_argument("preset", choices=view.PRESETS)
    v.add_argument("arg", nargs="?", help="route: the target place; session: the session id")
    v.add_argument(
        "--layers",
        help="custom only: map,robot,scan,trail,places,submaps,zones,"
        "target=<path>,session=<id> (at most 4)",
    )
    v.add_argument(
        "--ego",
        nargs="?",
        type=float,
        const=view.DEFAULT_EGO_M,
        metavar="R",
        help="crop to R m around the robot (default 7.5); `view here` crops by default, "
        "--ego 0 shows the whole map",
    )
    v.add_argument("--full", action="store_true", help="no crop (same as --ego 0)")
    v.add_argument("--target", help="route target (same as the positional)")
    v.add_argument("--session", help="session id (same as the positional)")
    v.add_argument("-o", "--output", metavar="FILE", help="write the PNG to FILE")
    _json_flag(v)

    ip = add("init-pose", commands.cmd_init_pose, "tell the process where the robot is")
    ip.add_argument("pose", nargs="*", type=float, metavar="X Y THETA")
    ip.add_argument("--place", help="use the pose saved in <path>/place.yaml (no walk-up)")

    add("relocalize", commands.cmd_relocalize, "search the whole map for the robot")
    add("checkpoint", commands.cmd_checkpoint, "write the session and anchors to disk now")

    ses = sub.add_parser("session", help="management: list, freeze, rotate, remove sessions")
    ssub = ses.add_subparsers(dest="sub", metavar="<ls|freeze|new|rm>", parser_class=_Parser)
    ssub.required = True
    _json_flag(ssub.add_parser("ls", help="list sessions")).set_defaults(fn=sessions.cmd_ls)
    for name, fn, help_ in (
        ("freeze", sessions.cmd_freeze, "freeze the fed session (plan, then --yes applies)"),
        ("new", sessions.cmd_new, "same as freeze: a fed freeze always starts a fresh session"),
    ):
        sp = ssub.add_parser(name, help=help_, description=help_)
        sp.add_argument("--yes", action="store_true", help="apply the plan")
        sp.add_argument("--force", action="store_true", help=argparse.SUPPRESS)
        sp.set_defaults(fn=fn)
    rm = ssub.add_parser(
        "rm",
        help="remove a floating session, or drop the fed one in process: a fresh one is fed and "
        "the robot's pose is lost (plan, then --yes applies)",
    )
    rm.add_argument("id", type=int)
    rm.add_argument("--yes", action="store_true", help="apply the plan")
    _json_flag(rm)
    rm.set_defaults(fn=sessions.cmd_rm)

    mp = sub.add_parser("map", help="management: list maps, start a fresh one, open a saved one")
    msub = mp.add_subparsers(dest="sub", metavar="<ls|new|open>", parser_class=_Parser)
    msub.required = True
    _json_flag(msub.add_parser("ls", help="list the maps; * marks the open one")).set_defaults(
        fn=maps.cmd_ls
    )
    for name, fn, help_ in (
        ("new", maps.cmd_new, "start a fresh map <name>; the open one stays on disk"),
        ("open", maps.cmd_open, "switch to the saved map <name>"),
    ):
        sp = msub.add_parser(name, help=help_, description=help_)
        sp.add_argument("name", help="[a-z0-9][a-z0-9_-]*")
        sp.set_defaults(fn=fn)

    fs = sub.add_parser("fs", help="memory/ file operations when you cannot reach the directory")
    fsub = fs.add_subparsers(
        dest="sub", metavar="<ls|tree|cat|write|append|mkdir|mv|rm>", parser_class=_Parser
    )
    fsub.required = True
    for name, fn, help_ in (
        ("ls", fscmd.cmd_ls, "list a directory"),
        ("tree", fscmd.cmd_tree, "print a directory tree; a cut-off directory shows …"),
    ):
        sp = fsub.add_parser(name, help=help_)
        sp.add_argument("path", nargs="?", default=".")
        if name == "tree":
            sp.add_argument("-L", dest="depth", type=int, default=3, help="depth (1..6, default 3)")
        sp.set_defaults(fn=fn)
    sp = fsub.add_parser("cat", help="print a file")
    sp.add_argument("path")
    sp.add_argument(
        "-o", "--output", metavar="FILE", help="write the raw bytes to FILE, print its path"
    )
    sp.set_defaults(fn=fscmd.cmd_cat)
    for name, fn, help_ in (
        ("write", fscmd.cmd_write, "replace a file atomically with stdin"),
        ("append", fscmd.cmd_append, "append stdin to a file in one write"),
    ):
        sp = fsub.add_parser(name, help=help_)
        sp.add_argument("path")
        sp.add_argument("--file", help="read the content from FILE instead of stdin")
        sp.set_defaults(fn=fn)
    sp = fsub.add_parser("mkdir", help="mkdir -p")
    sp.add_argument("path")
    sp.set_defaults(fn=fscmd.cmd_mkdir)
    sp = fsub.add_parser("mv", help="rename; the destination must not exist")
    sp.add_argument("src")
    sp.add_argument("dst")
    sp.set_defaults(fn=fscmd.cmd_mv)
    sp = fsub.add_parser("rm", help="remove a file or an empty directory; -r for a subtree")
    sp.add_argument("path")
    sp.add_argument("-r", "--recursive", action="store_true")
    sp.set_defaults(fn=fscmd.cmd_rm)

    h = add("help", None, "`egs help agent`: the rules and command table in 30 lines")
    h.add_argument("topic", nargs="?", choices=("agent",))
    h.set_defaults(fn=_cmd_help(p))
    return p


def main(argv=None, env=None, out=None, err=None):
    ctx = Ctx(env, out, err)
    argv = sys.argv[1:] if argv is None else list(argv)
    # Until parsing succeeds, a usage error still answers in the mode that was asked for.
    ctx.json = "--json" in argv
    try:
        args = build_parser().parse_args(argv)
        ctx.json = bool(args.json)
        if ctx.json and not getattr(args, "json_ok", False):
            raise EgsError("--json is not supported by this command", EXIT_USAGE)
        return args.fn(ctx, args)
    except EgsError as e:
        ctx.err.write("egs: %s\n" % e)
        reason = e.reason or EXIT_REASONS.get(e.code, "error")
        ctx.result({"ok": False, "reason": reason, "detail": str(e)})
        return e.code
    except SystemExit as e:
        return e.code if isinstance(e.code, int) else EXIT_USAGE
    except KeyboardInterrupt:
        return 130
    except BrokenPipeError:
        return 0
