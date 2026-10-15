"""MCP tools generated from the egs argparse tree: one tool per leaf command, run by cli.main."""

import argparse
import base64
import io
import json
import os
import tempfile
import traceback

EXCLUDED = {("help",), ("mcp",), ("skill", "install")}
MANAGEMENT_ALLOWED = {("session", "ls"), ("session", "rm")}
RENAMED = {("skill", "show"): "egs_help"}
DESCRIPTIONS = {
    ("skill", "show"): "The manual for these tools (SKILL.md): rules, problem -> command tables, "
    "how to read answers. Call it once before anything else. `egs <cmd> <sub>` in it is the "
    "tool egs_<cmd>_<sub>; its flags are this tool's parameters."
}
# These options run until Ctrl-C; a tool call must return.
STREAMING = {"watch", "follow"}
DROPPED = {("fs", "cat"): {"output"}}
TEXT_OUTPUT = {("fs", "cat"), ("skill", "show")}
FORCED = {("fs", "cat"): ["--output=-"]}
IMAGE_TYPES = {
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".gif": "image/gif",
    ".webp": "image/webp",
}


class ToolInputError(Exception):
    pass


class Param:
    def __init__(self, action, schema, required):
        self.action = action
        self.schema = schema
        self.required = required
        self.positional = action is not None and not action.option_strings
        strings = action.option_strings if action is not None else []
        self.option = max(strings, key=len) if strings else None


class Tool:
    def __init__(self, path, parser, help_):
        self.path = path
        self.name = RENAMED.get(path) or "egs_" + "_".join(p.replace("-", "_") for p in path)
        self.json_ok = path not in TEXT_OUTPUT
        self.params = {}
        dropped = DROPPED.get(path, set()) | STREAMING
        kept = []
        for a in parser._actions:
            if a.dest in ("help", "json") or a.dest in dropped or a.help == argparse.SUPPRESS:
                continue
            kept.append(a)
            self.params[a.dest] = Param(a, _schema(a), _required(a))
        self.stdin = any(a.dest == "file" and "stdin" in (a.help or "") for a in kept)
        if self.stdin:
            self.params["content"] = Param(
                None,
                {"type": "string", "description": "the text to write (instead of file)"},
                False,
            )
        self.description = _describe(path, parser, help_, kept)

    def spec(self):
        props = {name: p.schema for name, p in self.params.items()}
        schema = {"type": "object", "properties": props, "additionalProperties": False}
        required = [name for name, p in self.params.items() if p.required]
        if required:
            schema["required"] = required
        return {"name": self.name, "description": self.description, "inputSchema": schema}


def _item_type(action):
    if action.type is float:
        return {"type": "number"}
    if action.type is int:
        return {"type": "integer"}
    return {"type": "string"}


def _schema(a):
    if isinstance(a, argparse._StoreConstAction):
        s = {"type": "boolean"}
    elif isinstance(a, argparse._CountAction):
        s = {"type": "integer"}
    else:
        item = _item_type(a)
        if a.choices is not None:
            item["enum"] = list(a.choices)
        if isinstance(a.nargs, int):
            s = {"type": "array", "items": item, "minItems": a.nargs, "maxItems": a.nargs}
        elif a.nargs in ("+", "*") or isinstance(a, argparse._AppendAction):
            s = {"type": "array", "items": item}
            if a.nargs == "+":
                s["minItems"] = 1
        else:
            s = item
    text = [] if a.help in (None, argparse.SUPPRESS) else [a.help]
    if a.metavar:
        text.insert(0, " ".join(a.metavar) if isinstance(a.metavar, tuple) else a.metavar)
    if text:
        s["description"] = ": ".join(text)
    if a.default not in (False, argparse.SUPPRESS) and isinstance(a.default, (str, int, float)):
        s["default"] = a.default
    return s


def _required(a):
    if a.option_strings:
        return bool(a.required)
    return a.nargs not in ("?", "*")


def _describe(path, parser, help_, kept):
    if path in DESCRIPTIONS:
        return DESCRIPTIONS[path]
    fmt = parser._get_formatter()
    fmt.add_usage(None, kept, parser._mutually_exclusive_groups, prefix="")
    usage = " ".join(fmt.format_help().split())
    text = (help_ or parser.description or "").strip()
    lines = [text if text.endswith((".", "…")) else text + ".", "CLI: " + usage]
    for g in parser._mutually_exclusive_groups:
        if g.required:
            lines.append("Give exactly one of: %s." % ", ".join(a.dest for a in g._group_actions))
    return "\n".join(l for l in lines if l)


def _subparsers(parser):
    for a in parser._actions:
        if isinstance(a, argparse._SubParsersAction):
            return a
    return None


def _leaves(parser, path=(), help_=None, management=False):
    sub = _subparsers(parser)
    if sub is None:
        yield path, parser, help_, management
        return
    helps = {a.dest: a.help for a in sub._choices_actions}
    seen = set()
    for name, child in sub.choices.items():
        if id(child) in seen:
            continue
        seen.add(id(child))
        h = helps.get(name)
        mgmt = management or (h or "").startswith("management")
        for leaf in _leaves(child, path + (name,), h, mgmt):
            yield leaf


def build_tools(parser, management=False):
    tools = []
    for path, sub, help_, mgmt in _leaves(parser):
        if any(path[: len(e)] == e for e in EXCLUDED):
            continue
        if mgmt and not management and path not in MANAGEMENT_ALLOWED:
            continue
        tools.append(Tool(path, sub, help_))
    return sorted(tools, key=lambda t: t.name)


def _number(v, integer):
    if integer and float(v).is_integer():
        return str(int(v))
    text = repr(v)
    if "e" in text or "E" in text:
        from decimal import Decimal

        text = format(Decimal(text), "f")
    return text


def _scalar(name, p, v):
    integer = p.action is not None and p.action.type is int
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return _number(v, integer)
    if isinstance(v, str):
        return v
    raise ToolInputError("%s: expected a single value, got %s" % (name, json.dumps(v)))


def _values(name, p, v):
    items = v if isinstance(v, list) else [v]
    return [_scalar(name, p, x) for x in items]


def build_argv(tool, arguments):
    unknown = sorted(set(arguments) - set(tool.params))
    if unknown:
        raise ToolInputError(
            "unknown parameter %s; %s takes: %s"
            % (", ".join(unknown), tool.name, ", ".join(tool.params) or "nothing")
        )
    opts, pos = [], []
    for name, p in tool.params.items():
        v = arguments.get(name)
        if v is None or p.action is None:
            continue
        a = p.action
        if p.positional:
            pos += _values(name, p, v)
        elif isinstance(a, argparse._StoreConstAction):
            if v is True:
                opts.append(p.option)
            elif v is not False:
                raise ToolInputError("%s: expected true or false" % name)
        elif isinstance(a, argparse._CountAction):
            opts += [p.option] * int(v)
        elif isinstance(a, argparse._AppendAction):
            opts += ["%s=%s" % (p.option, x) for x in _values(name, p, v)]
        elif isinstance(a.nargs, int) or a.nargs in ("+", "*"):
            opts += [p.option] + _values(name, p, v)
        elif a.nargs == "?" and isinstance(v, bool):
            if v:
                opts.append(p.option)
        else:
            opts.append("%s=%s" % (p.option, _scalar(name, p, v)))
    return opts, pos


def call(tool, arguments, env=None, base_argv=(), log=None):
    """Run one tool; returns an MCP CallToolResult."""
    from . import cli

    tmp = None
    try:
        opts, pos = build_argv(tool, arguments)
        if tool.stdin:
            if arguments.get("content") is not None:
                fd, tmp = tempfile.mkstemp(prefix="egs-mcp-", suffix=".txt")
                with os.fdopen(fd, "w", encoding="utf-8", newline="") as f:
                    f.write(str(arguments["content"]))
                opts.append("--file=" + tmp)
            elif arguments.get("file") is None:
                raise ToolInputError("give content (the text) or file (a path on this machine)")
        argv = list(base_argv) + list(tool.path) + opts + FORCED.get(tool.path, [])
        if tool.json_ok:
            argv.append("--json")
        if pos:
            argv += ["--"] + pos
        out = io.TextIOWrapper(io.BytesIO(), encoding="utf-8", newline="")
        err = io.StringIO()
        try:
            code = cli.main(argv, env=env, out=out, err=err)
        except Exception as e:
            if log is not None:
                log.write(traceback.format_exc())
            return _result(["egs crashed: %s: %s" % (type(e).__name__, e)], True)
        out.flush()
        data = out.buffer.getvalue()
    except ToolInputError as e:
        return _result([str(e)], True)
    finally:
        if tmp:
            os.unlink(tmp)
    code = 0 if code is None else code
    stderr = err.getvalue().strip()
    if tool.path == ("fs", "cat") and code == 0:
        blocks = _file_blocks(arguments.get("path") or "", data)
    else:
        text = data.decode("utf-8", "replace").strip()
        blocks = [_text(text or "(no output, exit %d)" % code)]
        if tool.path == ("view",) and code == 0:
            blocks += _view_image(text)
    if stderr:
        blocks.append(_text(stderr))
    return {"content": blocks, "isError": code != 0}


def _text(s):
    return {"type": "text", "text": s}


def _image(data, mime):
    return {"type": "image", "data": base64.b64encode(data).decode("ascii"), "mimeType": mime}


def _result(lines, error):
    return {"content": [_text("\n".join(lines))], "isError": error}


def _view_image(text):
    try:
        png = json.loads(text).get("png")
        with open(png, "rb") as f:
            return [_image(f.read(), "image/png")]
    except (ValueError, AttributeError, TypeError, OSError) as e:
        return [_text("no image: %s" % e)]


def _file_blocks(path, data):
    mime = IMAGE_TYPES.get(os.path.splitext(path)[1].lower())
    if mime:
        return [_text("%s: %s, %d bytes" % (path, mime, len(data))), _image(data, mime)]
    try:
        return [_text(data.decode("utf-8") or "(empty file)")]
    except UnicodeDecodeError:
        pass
    d = tempfile.mkdtemp(prefix="egs-mcp-")
    local = os.path.join(d, os.path.basename(path.rstrip("/")) or "file")
    with open(local, "wb") as f:
        f.write(data)
    return [_text("%s: binary, %d bytes, saved on this machine as %s" % (path, len(data), local))]
