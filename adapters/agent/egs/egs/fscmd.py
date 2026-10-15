"""egs fs: shell-alike file operations on memory/, local or through the /fs passthrough."""

import os
import sys

from .client import EXIT_OK, EXIT_REFUSED, EXIT_USAGE, EgsError
from .store import norm


def _rel(ctx, path):
    return norm(path, ctx.memory_dir())


def _stdin_bytes(args):
    if args.file:
        with open(args.file, "rb") as f:
            return f.read()
    stdin = sys.stdin
    return stdin.buffer.read() if hasattr(stdin, "buffer") else stdin.read().encode("utf-8")


def cmd_ls(ctx, args):
    rel = _rel(ctx, args.path)
    entries = ctx.store().listdir(rel)
    if entries is None:
        ctx.print("fs_error: not a directory: " + rel)
        return EXIT_REFUSED
    for name, t, size in entries:
        if t == "dir":
            ctx.print(name + "/")
        elif t == "file":
            ctx.print("%s  %d" % (name, size))
        else:
            ctx.print(name + "@")
    return EXIT_OK


def cmd_tree(ctx, args):
    depth = max(1, min(6, args.depth))
    ctx.write_bytes(ctx.store().tree(_rel(ctx, args.path), depth).encode("utf-8"))
    return EXIT_OK


def cmd_cat(ctx, args):
    rel = _rel(ctx, args.path)
    if args.output == "-":
        args.output = None
    if args.output:
        ctx.refuse_inside_memory(args.output)
    data = ctx.store().read(rel)
    if data is None:
        ctx.print("cannot read %s: no such file, or not a file" % rel)
        return EXIT_REFUSED
    if args.output:
        out = os.path.abspath(args.output)
        try:
            with open(out, "wb") as f:
                f.write(data)
        except OSError as e:
            raise EgsError("cannot write %s: %s" % (out, e.strerror or e), EXIT_USAGE)
        ctx.print(out)
        return EXIT_OK
    ctx.write_bytes(data)
    return EXIT_OK


def cmd_grep(ctx, args):
    if not args.pattern:
        raise EgsError("fs grep needs a non-empty PATTERN", EXIT_USAGE, "bad_param")
    matches, truncated = ctx.store().grep(_rel(ctx, args.path), args.pattern, args.include)
    for m in matches:
        ctx.print("%s:%d:%s" % (m["path"], m["line"], m["text"]))
    if truncated:
        ctx.warn("stopped at %d matches: narrow PATH, PATTERN or --include" % len(matches))
    ctx.result({"ok": True, "reason": None, "matches": matches, "truncated": truncated})
    return EXIT_OK


def cmd_write(ctx, args):
    ctx.store().write(_rel(ctx, args.path), _stdin_bytes(args))
    return EXIT_OK


def cmd_append(ctx, args):
    ctx.store().append(_rel(ctx, args.path), _stdin_bytes(args))
    return EXIT_OK


def cmd_mkdir(ctx, args):
    ctx.store().mkdir(_rel(ctx, args.path))
    return EXIT_OK


def cmd_mv(ctx, args):
    ctx.store().mv(_rel(ctx, args.src), _rel(ctx, args.dst))
    return EXIT_OK


def cmd_rm(ctx, args):
    ctx.store().rm(_rel(ctx, args.path), recursive=args.recursive)
    return EXIT_OK
