"""egs fs: shell-alike file operations on memory/, local or through the /fs passthrough."""

import sys

from .client import EXIT_OK, EXIT_REFUSED
from .store import norm


def _rel(ctx, path):
    return norm(path, ctx.memory_dir())


def _write_bytes(ctx, data):
    buf = getattr(ctx.out, "buffer", None)
    if buf is not None:
        ctx.out.flush()
        buf.write(data)
        buf.flush()
    else:
        ctx.out.write(data.decode("utf-8", "replace"))


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
    _write_bytes(ctx, ctx.store().tree(_rel(ctx, args.path), depth).encode("utf-8"))
    return EXIT_OK


def cmd_cat(ctx, args):
    rel = _rel(ctx, args.path)
    data = ctx.store().read(rel)
    if data is None:
        ctx.print("cannot read %s: no such file, or not a file" % rel)
        return EXIT_REFUSED
    _write_bytes(ctx, data)
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
