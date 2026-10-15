"""egs mcp: the egs commands as MCP tools, newline-delimited JSON-RPC 2.0 on stdin/stdout."""

import json
import os
import sys
import traceback

from . import __version__, mcptools
from .client import EXIT_OK

PROTOCOL_VERSIONS = ("2025-11-25", "2025-06-18", "2025-03-26", "2024-11-05")
INSTRUCTIONS = (
    "Call egs_help once first: it is the manual these tools follow (`egs <cmd> <sub>` there is "
    "the tool egs_<cmd>_<sub>, flags are parameters).\n"
    "Then call egs_root to check the connection to the robot and see where memory/ lives.\n"
    "Results are the egs CLI's --json objects; isError means a refusal or failure, its reason "
    "inside."
)

PARSE_ERROR = -32700
INVALID_REQUEST = -32600
METHOD_NOT_FOUND = -32601
INVALID_PARAMS = -32602
INTERNAL_ERROR = -32603


class RpcError(Exception):
    def __init__(self, code, message):
        Exception.__init__(self, message)
        self.code = code


def _error(id_, code, message):
    return {"jsonrpc": "2.0", "id": id_, "error": {"code": code, "message": message}}


class Server:
    def __init__(self, management=False, env=None, base_argv=(), log=None):
        from . import cli

        self.tools = mcptools.build_tools(cli.build_parser(), management)
        self.by_name = {t.name: t for t in self.tools}
        self.env = env
        self.base_argv = list(base_argv)
        self.log = log or sys.stderr
        self.methods = {
            "initialize": self.initialize,
            "ping": lambda params: {},
            "tools/list": self.tools_list,
            "tools/call": self.tools_call,
        }

    def initialize(self, params):
        asked = params.get("protocolVersion")
        version = asked if asked in PROTOCOL_VERSIONS else PROTOCOL_VERSIONS[0]
        return {
            "protocolVersion": version,
            "capabilities": {"tools": {"listChanged": False}},
            "serverInfo": {"name": "egs", "version": __version__},
            "instructions": INSTRUCTIONS,
        }

    def tools_list(self, params):
        return {"tools": [t.spec() for t in self.tools]}

    def tools_call(self, params):
        name = params.get("name")
        tool = self.by_name.get(name)
        if tool is None:
            raise RpcError(INVALID_PARAMS, "unknown tool: %s" % name)
        arguments = params.get("arguments")
        if arguments is None:
            arguments = {}
        if not isinstance(arguments, dict):
            raise RpcError(INVALID_PARAMS, "arguments must be an object")
        result = mcptools.call(tool, arguments, self.env, self.base_argv, self.log)
        self.log.write("egs mcp: %s -> %s\n" % (name, "error" if result["isError"] else "ok"))
        return result

    def handle(self, msg):
        """One parsed message; returns the response, or None for notifications and responses."""
        if not isinstance(msg, dict) or msg.get("jsonrpc") != "2.0":
            return _error(None, INVALID_REQUEST, "not a JSON-RPC 2.0 message")
        method = msg.get("method")
        is_request = "id" in msg
        id_ = msg.get("id")
        if not isinstance(method, str):
            if "result" in msg or "error" in msg:
                return None
            return _error(id_, INVALID_REQUEST, "no method")
        if not is_request:
            return None
        fn = self.methods.get(method)
        if fn is None:
            return _error(id_, METHOD_NOT_FOUND, "method not found: %s" % method)
        params = msg.get("params")
        if params is None:
            params = {}
        if not isinstance(params, dict):
            return _error(id_, INVALID_PARAMS, "params must be an object")
        try:
            return {"jsonrpc": "2.0", "id": id_, "result": fn(params)}
        except RpcError as e:
            return _error(id_, e.code, str(e))
        except Exception as e:
            self.log.write(traceback.format_exc())
            return _error(id_, INTERNAL_ERROR, "%s: %s" % (type(e).__name__, e))

    def handle_line(self, line):
        try:
            msg = json.loads(line)
        except ValueError as e:
            return _error(None, PARSE_ERROR, "parse error: %s" % e)
        if isinstance(msg, list):
            if not msg:
                return _error(None, INVALID_REQUEST, "empty batch")
            answers = [r for r in (self.handle(m) for m in msg) if r is not None]
            return answers or None
        return self.handle(msg)


def serve(inp, outp, management=False, env=None, base_argv=(), log=None):
    """Answers requests one at a time until EOF. inp and outp are binary streams."""
    server = Server(management, env, base_argv, log)
    server.log.write(
        "egs mcp %s: %d tools, protocol %s\n"
        % (__version__, len(server.tools), " ".join(PROTOCOL_VERSIONS))
    )
    saved = sys.stdin, sys.stdout
    # Nothing a command does may read the protocol stream or write into it.
    sys.stdin, sys.stdout = open(os.devnull), sys.stderr
    try:
        for raw in iter(inp.readline, b""):
            line = raw.decode("utf-8", "replace").strip()
            if not line:
                continue
            answer = server.handle_line(line)
            if answer is not None:
                outp.write((json.dumps(answer, ensure_ascii=False) + "\n").encode("utf-8"))
                outp.flush()
    finally:
        sys.stdin.close()
        sys.stdin, sys.stdout = saved
    return EXIT_OK


def cmd_mcp(ctx, args):
    ctx.settings()
    base = ["--robot", ctx.robot] if ctx.robot else []
    return serve(sys.stdin.buffer, sys.stdout.buffer, args.management, ctx.env, base, ctx.err)
