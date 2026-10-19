"""egs: the bridge from an agent's shell to the running EvergreenSLAM process."""

import argparse
import sys

from . import commands, fscmd, maps, pose, sessions, skill, view, zones
from .client import EXIT_OK, EXIT_REFUSED, EXIT_TRANSPORT, EXIT_USAGE, EgsError
from .context import Ctx

DESCRIPTION = """\
egs is the bridge from your shell to the running SLAM process, usually on a robot reached through
an SSH tunnel. Start with `egs root`: it checks the connection and says where memory/ lives.
memory/ is a directory tree of places and things; work in it with `egs fs` (ls, tree, cat, grep,
write, append, mkdir, mv, rm). egs also answers what the live process alone knows: coordinates,
saving a place, maps and sessions. Read memory/README.md before changing memory/.
All paths are relative to memory/ and start with places/."""

EPILOG = """\
connection (command line > environment > config file > defaults):
  --robot NAME / EGS_ROBOT  use [robot.NAME] of ~/.config/egs/config ($XDG_CONFIG_HOME/egs/config):
               url = http://127.0.0.1:8643, timeout = 30, memory = <dir>; [default] fills gaps
  EGS_URL      service address (default http://127.0.0.1:8643)
  EGS_TIMEOUT  seconds per request (default 30)
  EGS_MEMORY   a memory/ directory on this machine to work in directly (a mounted map directory)
  EGS_USE_PROXY=1  go through the system/environment HTTP proxy (default: never)
  EGS_OFFSET_MAX_M  observe --offset and zone add refuse beyond this robot-to-place distance
               (default 10)
  The robot's service listens on 127.0.0.1:8643. From another machine open a tunnel first:
    ssh -NT -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 \\
        -L 127.0.0.1:8643:127.0.0.1:8643 user@robot

memory/ access: egs reads and writes files directly only when EGS_MEMORY (or config memory) names
them, or when this machine runs the SLAM process itself (`egs root` prints a path). Otherwise every
tree access (where, find, observe, fs, ...) goes through the process's /fs endpoints with the same
rules, and `egs root` prints `remote <url>  map <name>`.

node.yaml must stay in the flat YAML subset of memory/README.md (key: value, flow lists [a, b],
numbers, strings); egs refuses to rewrite anything else, and rewriting drops comments.

Timestamps print as ages (35s, 20m, 6d); the robot's own stamps age by its clock (GET /pose).
A mutating command that gets no answer within the timeout prints 'outcome unknown': check with
egs status / here / session ls before retrying.

--json (every command but fs cat, skill, help, mcp): one JSON object instead of text; prefer it.

exit codes: 0 done (a session plan printed without --yes is done too), 1 refused or plan
rejected (nothing changed unless stated), or a relocalization ended no_match or superseded,
2 usage or bad path, 3 process unreachable, not started, or outcome unknown.
There is no goto: egs never moves the robot; hand `approach` from `egs where` to your navigator."""

AGENT_HELP = """\
1. no map-frame XY in any file you write; ask `egs where` for a current position
2. place.yaml belongs to the process: never write, edit or copy it; stand there, `egs place save`
3. things you can stand in front of are places; small movable things: node.yaml, no place.yaml
4. to find a thing use `egs find`; to search notes and skills `egs fs grep <words> [--path <p>]`
5. egs paths are relative to memory/ and start with places/
6. skills and attachments are reserved names, never a place or a thing; no symbolic links
7. observations.jsonl is append-only; retract a line with `egs observe <path> --invalidates <id>`
8. there is no egs goto: egs never moves the robot

first -> egs root: checks the connection; memory/ is on the robot, so work in it with egs fs
where am I / is a place or a thing / find it by name -> egs here / where places/<p> / find <words>
remember the spot the robot stands at -> egs place save places/<path> [--offset DX DY 0, <= 3 m]
I saw it / it is not there -> egs observe places/<path> [--result absent] [--note ..] [--attach F]
a new thing -> egs fs mkdir places/<place>/<thing>, egs fs write <that>/node.yaml (kind, aliases)
it is DX ahead, DY left -> egs observe <thing> --offset DX DY 0 (no place within 10 m: save one)
...in a photo taken a moment ago -> add --at <stamp_ns> (robot sensor clock, see egs pose --json)
go somewhere -> egs where places/<path>, give its approach pose to your navigator
the navigator must avoid an area -> egs zone add places/<place>/<name> --rect X0 Y0 X1 Y1 (ls | rm)
after a start or a correction -> egs here --sample 20 until the base median settles (>= 0.5 ok)
a picture -> egs view map | here | route places/<p> | trail | session [id]
pose wrong, robot at a saved place now (<= 2 m) -> egs init-pose --place places/<p> (waits)
pose wrong, robot moved since or unsure where -> egs relocalize (a kidnap line in here: it searched already)
no_match, or base low a minute after matched -> egs session rm <fed id> (plan, --yes), then again
on disk now -> egs checkpoint; when did corrections land -> egs events; map health -> egs status
look around memory/ -> egs fs tree places | ls <path> | cat <file> | grep <words> [--path <p>]
new container, rename, re-parent, delete -> egs fs mkdir | mv | rm -r (bindings move along)
notes and skills -> egs fs write places/<p>/notes.md --file F (or stdin); egs fs append ...

exit 0 done, 1 refused, rejected or no_match, 2 bad usage or path, 3 unreachable or outcome unknown
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


def _cmd_mcp(ctx, args):
    from . import mcp

    return mcp.cmd_mcp(ctx, args)


def build_parser():
    p = _Parser(
        prog="egs",
        description=DESCRIPTION,
        epilog=EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--json", action="store_true", default=False, help=JSON_HELP)
    p.add_argument("--robot", metavar="NAME", help="connect as [robot.NAME] of the config file")
    sub = p.add_subparsers(dest="cmd", metavar="<command>", parser_class=_Parser)
    sub.required = True

    def add(name, fn, help_, **kw):
        sp = sub.add_parser(name, help=help_, description=help_, **kw)
        sp.set_defaults(fn=fn)
        return sp

    _json_flag(
        add(
            "root",
            commands.cmd_root,
            "check the connection; print memory/'s path, or `remote <url>  map <name>`",
        )
    )
    h = add("here", commands.cmd_here, "where the robot is: current place, two closest, pose")
    h.add_argument(
        "--sample",
        type=float,
        metavar="S",
        help="keep reading for S seconds (<= 60) and add the median and minimum base score of the "
        "keyframes seen at cover >= 50%%; a pose correction or matched relocalization during the "
        "window drops the reads before it (restarted N)",
    )
    _json_flag(h)
    _json_flag(
        add("status", commands.cmd_status, "map health: frozen base, base link, closures")
    )
    pw = add("pose", pose.cmd_pose, "the robot's pose from the newest scan, for programs")
    pw.add_argument(
        "--watch",
        nargs="?",
        type=float,
        const=pose.DEFAULT_HZ,
        metavar="HZ",
        help="poll at HZ (default 10, at most 30) and print one line per new scan "
        "(one JSON object per line with --json) until Ctrl-C",
    )
    _json_flag(pw)

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
        help="where the thing is in the robot's frame, now or at --at (x forward, y left; m, rad)",
    )
    o.add_argument(
        "--at",
        metavar="STAMP",
        help="with --offset: the offset was measured at this robot sensor time (Unix ns, as "
        "`egs pose --json` stamp_ns, or Unix seconds with a decimal point), not now",
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
    _json_flag(za)
    zl = zsub.add_parser(
        "ls", help="every zone: its place, state and area, or why it is unresolvable"
    )
    _json_flag(zl)
    zl.add_argument("--xy", action="store_true", help="also the map-frame polygon (never store it)")
    zl.set_defaults(fn=zones.cmd_ls)
    zr = zsub.add_parser("rm", help="delete <path>/zone.yaml (and the directory once empty)")
    zr.add_argument("path")
    zr.set_defaults(fn=zones.cmd_rm)
    _json_flag(zr)

    s = add("snapshot", commands.cmd_snapshot, "export map, trajectory, places; refresh index.tsv")
    s.add_argument(
        "-o", "--output", metavar="DIR", help="also copy the files to DIR (downloads when remote)"
    )
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
    v.add_argument(
        "-o", "--output", metavar="FILE", help="write the PNG to FILE; - writes it to stdout"
    )
    _json_flag(v)

    ip = add(
        "init-pose",
        commands.cmd_init_pose,
        "tell the process where the robot is; waits for matched / no_match",
    )
    ip.add_argument("pose", nargs="*", type=float, metavar="X Y THETA")
    ip.add_argument("--place", help="use the pose saved in <path>/place.yaml (no walk-up)")
    rl = add("relocalize", commands.cmd_relocalize, "search the whole map for the robot; waits")
    for sp in (ip, rl):
        sp.add_argument(
            "--wait",
            type=float,
            default=30.0,
            metavar="S",
            help="seconds to wait for the outcome (default 30), then 'outcome unknown'",
        )
        sp.add_argument("--no-wait", action="store_true", help="return once the request is queued")
        _json_flag(sp)

    ev = add("events", commands.cmd_events, "what the process did to the pose and the sessions")
    ev.add_argument("--since", type=int, default=0, metavar="N", help="from sequence N (default 0)")
    ev.add_argument("--follow", action="store_true", help="poll every second, one line per event")
    _json_flag(ev)
    _json_flag(
        add("checkpoint", commands.cmd_checkpoint, "write the session and anchors to disk now")
    )

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
        _json_flag(sp).set_defaults(fn=fn)
    rm = ssub.add_parser(
        "rm",
        help="remove an idle session, or drop the fed one in process: a fresh one is fed and "
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
        _json_flag(sp).set_defaults(fn=fn)

    fs = sub.add_parser("fs", help="memory/ file operations, on the robot or here")
    fsub = fs.add_subparsers(
        dest="sub", metavar="<ls|tree|cat|grep|write|append|mkdir|mv|rm>", parser_class=_Parser
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
        _json_flag(sp).set_defaults(fn=fn)
    sp = fsub.add_parser("cat", help="print a file")
    sp.add_argument("path")
    sp.add_argument(
        "-o",
        "--output",
        metavar="FILE",
        help="write the raw bytes to FILE, print its path (- is stdout)",
    )
    sp.set_defaults(fn=fscmd.cmd_cat)
    sp = fsub.add_parser(
        "grep",
        help="print path:line:text for lines holding WORDS joined by single spaces (one fixed "
        "string, any case); skips observations.jsonl and attachments/; at most 200",
    )
    sp.add_argument(
        "words", nargs="+", metavar="WORD", help="the phrase to find, one or more words (hand truck)"
    )
    sp.add_argument("--path", default=".", help="a directory or file to search (default memory/)")
    sp.add_argument(
        "--include",
        metavar="GLOBS",
        help="comma-separated file-name globs (default node.yaml,notes.md,SKILL.md,*.md,*.yaml)",
    )
    _json_flag(sp).set_defaults(fn=fscmd.cmd_grep)
    for name, fn, help_ in (
        ("write", fscmd.cmd_write, "replace a file atomically with stdin"),
        ("append", fscmd.cmd_append, "append stdin to a file in one write"),
    ):
        sp = fsub.add_parser(name, help=help_)
        sp.add_argument("path")
        sp.add_argument("--file", help="read the content from FILE instead of stdin")
        _json_flag(sp).set_defaults(fn=fn)
    sp = fsub.add_parser("mkdir", help="mkdir -p")
    sp.add_argument("path")
    _json_flag(sp).set_defaults(fn=fscmd.cmd_mkdir)
    sp = fsub.add_parser("mv", help="rename; the destination must not exist")
    sp.add_argument("src")
    sp.add_argument("dst")
    _json_flag(sp).set_defaults(fn=fscmd.cmd_mv)
    sp = fsub.add_parser("rm", help="remove a file or an empty directory; -r for a subtree")
    sp.add_argument("path")
    sp.add_argument("-r", "--recursive", action="store_true")
    _json_flag(sp).set_defaults(fn=fscmd.cmd_rm)

    sk = sub.add_parser("skill", help="the Agent Skill shipped with egs")
    sksub = sk.add_subparsers(dest="sub", metavar="<show|install>", parser_class=_Parser)
    sksub.required = True
    sksub.add_parser("show", help="print SKILL.md").set_defaults(fn=skill.cmd_show)
    si = sksub.add_parser("install", help="write <dir>/evergreenslam/SKILL.md")
    si.add_argument("dir", help="the agent's skills directory, e.g. ~/.claude/skills")
    si.set_defaults(fn=skill.cmd_install)

    mc = add(
        "mcp",
        _cmd_mcp,
        "serve the egs commands as MCP tools on stdin/stdout, for one robot (see README)",
    )
    mc.add_argument(
        "--management", action="store_true", help="also serve map and session freeze|new"
    )

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
        ctx.robot = args.robot
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
