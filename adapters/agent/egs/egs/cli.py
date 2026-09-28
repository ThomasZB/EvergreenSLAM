"""egs: the bridge from an agent's shell to the running EvergreenSLAM process."""

import argparse
import sys

from . import commands, fscmd, maps, sessions, view
from .client import EXIT_USAGE, EgsError
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
  EGS_OFFSET_MAX_M  observe --offset refuses beyond this robot-to-place distance (default 10)

memory/ access: when the memory directory is not readable on this machine, every tree read and
write egs does (where, find, observe, fs, ...) goes through the process's /fs endpoints instead,
transparently and with the same rules. `egs fs` is the shell stand-in for that case.

node.yaml must stay in the flat YAML subset of memory/README.md (key: value, flow lists [a, b],
numbers, strings); egs refuses to rewrite anything else, and rewriting drops comments.

Timestamps print as ages (35s, 20m, 6d). A mutating command that gets no answer within the
timeout prints 'outcome unknown': check with egs status / here / session ls before retrying.

exit codes: 0 done (a session plan printed without --yes is done too), 1 refused or plan
rejected (nothing changed unless stated), 2 usage or bad path, 3 process unreachable, not
started, or outcome unknown.
There is no goto: egs never moves the robot; hand `approach` from `egs where` to your navigator."""


class _Parser(argparse.ArgumentParser):
    def error(self, message):
        raise EgsError("%s: %s (see %s --help)" % (self.prog, message, self.prog), EXIT_USAGE)


def build_parser():
    p = _Parser(
        prog="egs",
        description=DESCRIPTION,
        epilog=EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = p.add_subparsers(dest="cmd", metavar="<command>", parser_class=_Parser)
    sub.required = True

    def add(name, fn, help_, **kw):
        sp = sub.add_parser(name, help=help_, description=help_, **kw)
        sp.set_defaults(fn=fn)
        return sp

    add("root", commands.cmd_root, "print the absolute path of memory/")
    add("here", commands.cmd_here, "where the robot is: current place, two closest, pose")
    add("status", commands.cmd_status, "map health: frozen base, alignment, phase, closures")

    place = sub.add_parser("place", help="save the place the robot stands at")
    psub = place.add_subparsers(dest="sub", metavar="<save>", parser_class=_Parser)
    psub.required = True
    ps = psub.add_parser("save", help="bind <path> to the robot's current pose (stand there first)")
    ps.add_argument("path", help="places/...; re-saving keeps the anchor id and updates the pose")
    ps.add_argument("--no-scan", action="store_true", help="do not keep the keyframe scan")
    ps.set_defaults(fn=commands.cmd_place_save)

    w = add(
        "where", commands.cmd_where, "position of a place or thing, precision and approach pose"
    )
    w.add_argument("path")

    f = add(
        "find", commands.cmd_find, "find things: name > alias > kind > word > fuzzy; absent last"
    )
    f.add_argument("query", nargs="+")
    f.add_argument("--limit", type=int, default=10, help="rows to print (0 = all; default 10)")

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

    s = add("snapshot", commands.cmd_snapshot, "export map, trajectory, places; refresh index.tsv")
    s.add_argument("-o", "--output", metavar="DIR", help="also copy the files to DIR")

    v = add(
        "view",
        view.cmd_view,
        "render one picture: map | here | route <place> | trail | session [id] | custom --layers",
    )
    v.add_argument("preset", choices=view.PRESETS)
    v.add_argument("arg", nargs="?", help="route: the target place; session: the session id")
    v.add_argument(
        "--layers",
        help="custom only: map,robot,scan,trail,places,submaps,"
        "target=<path>,session=<id> (at most 4)",
    )
    v.add_argument(
        "--ego",
        nargs="?",
        type=float,
        const=view.DEFAULT_EGO_M,
        metavar="R",
        help="crop to R m around the robot (default 7.5)",
    )
    v.add_argument("--target", help="route target (same as the positional)")
    v.add_argument("--session", help="session id (same as the positional)")
    v.add_argument("-o", "--output", metavar="FILE", help="write the PNG to FILE")

    ip = add("init-pose", commands.cmd_init_pose, "tell the process where the robot is")
    ip.add_argument("pose", nargs="*", type=float, metavar="X Y THETA")
    ip.add_argument("--place", help="use the pose saved in <path>/place.yaml (no walk-up)")

    add("relocalize", commands.cmd_relocalize, "search the whole map for the robot")
    add("checkpoint", commands.cmd_checkpoint, "write the session and anchors to disk now")

    ses = sub.add_parser("session", help="management: list, freeze, rotate, remove sessions")
    ssub = ses.add_subparsers(dest="sub", metavar="<ls|freeze|new|rm>", parser_class=_Parser)
    ssub.required = True
    ssub.add_parser("ls", help="list sessions").set_defaults(fn=sessions.cmd_ls)
    for name, fn, help_ in (
        ("freeze", sessions.cmd_freeze, "freeze the fed session (plan, then --yes applies)"),
        ("new", sessions.cmd_new, "same as freeze: a fed freeze always starts a fresh session"),
    ):
        sp = ssub.add_parser(name, help=help_, description=help_)
        sp.add_argument("--yes", action="store_true", help="apply the plan")
        sp.add_argument("--force", action="store_true", help=argparse.SUPPRESS)
        sp.set_defaults(fn=fn)
    rm = ssub.add_parser("rm", help="remove a floating session (plan, then --yes applies)")
    rm.add_argument("id", type=int)
    rm.add_argument("--yes", action="store_true", help="apply the plan")
    rm.set_defaults(fn=sessions.cmd_rm)

    mp = sub.add_parser("map", help="management: list maps, start a fresh one, open a saved one")
    msub = mp.add_subparsers(dest="sub", metavar="<ls|new|open>", parser_class=_Parser)
    msub.required = True
    msub.add_parser("ls", help="list the maps; * marks the open one").set_defaults(fn=maps.cmd_ls)
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
        ("tree", fscmd.cmd_tree, "print a directory tree"),
    ):
        sp = fsub.add_parser(name, help=help_)
        sp.add_argument("path", nargs="?", default=".")
        if name == "tree":
            sp.add_argument("-L", dest="depth", type=int, default=2, help="depth (1..6, default 2)")
        sp.set_defaults(fn=fn)
    sp = fsub.add_parser("cat", help="print a file")
    sp.add_argument("path")
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
    return p


def main(argv=None, env=None, out=None, err=None):
    ctx = Ctx(env, out, err)
    try:
        args = build_parser().parse_args(sys.argv[1:] if argv is None else argv)
        return args.fn(ctx, args)
    except EgsError as e:
        ctx.err.write("egs: %s\n" % e)
        return e.code
    except SystemExit as e:
        return e.code if isinstance(e.code, int) else EXIT_USAGE
    except KeyboardInterrupt:
        return 130
    except BrokenPipeError:
        return 0
