"""egs skill: the Agent Skill shipped with egs, printed or installed into a skills directory."""

import os
import tempfile

from .client import EXIT_OK, EXIT_USAGE, EgsError

NAME = "evergreenslam"
SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "SKILL.md")


def text():
    with open(SOURCE, encoding="utf-8") as f:
        return f.read()


def cmd_show(ctx, args):
    ctx.out.write(text())
    return EXIT_OK


def cmd_install(ctx, args):
    dest_dir = os.path.join(os.path.abspath(os.path.expanduser(args.dir)), NAME)
    dest = os.path.join(dest_dir, "SKILL.md")
    if os.path.realpath(dest) == os.path.realpath(SOURCE):
        ctx.print(dest + " (already the shipped file)")
        return EXIT_OK
    data = text()
    try:
        os.makedirs(dest_dir, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=dest_dir, prefix=".SKILL.", suffix=".tmp")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(data)
        os.chmod(tmp, 0o644)
        os.replace(tmp, dest)
    except OSError as e:
        raise EgsError("cannot write %s: %s" % (dest, e.strerror or e), EXIT_USAGE)
    ctx.print(dest)
    return EXIT_OK
