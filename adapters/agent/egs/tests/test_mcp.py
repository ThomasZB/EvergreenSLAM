import argparse
import base64
import io
import json
import os
import subprocess
import sys
import unittest

from support import EgsTestCase
from fake_service import PNG

from egs import cli, mcptools
from egs.mcp import PROTOCOL_VERSIONS, serve

EGS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def req(id_, method, params=None):
    m = {"jsonrpc": "2.0", "id": id_, "method": method}
    if params is not None:
        m["params"] = params
    return m


def tool_call(id_, name, **arguments):
    return req(id_, "tools/call", {"name": name, "arguments": arguments})


def leaf_commands(parser, path=(), top_help=""):
    """Independent of mcptools: (path, help of its top-level command) of every leaf command."""
    subs = [a for a in parser._actions if isinstance(a, argparse._SubParsersAction)]
    if not subs:
        return [(path, top_help)]
    helps = {a.dest: a.help or "" for a in subs[0]._choices_actions}
    out = []
    for name, child in subs[0].choices.items():
        out += leaf_commands(child, path + (name,), top_help or helps.get(name, ""))
    return out


class McpTestCase(EgsTestCase):
    def rpc(self, *msgs, management=False):
        lines = [m if isinstance(m, str) else json.dumps(m) for m in msgs]
        outp = io.BytesIO()
        self.log = io.StringIO()
        inp = io.BytesIO(("\n".join(lines) + "\n").encode("utf-8"))
        serve(inp, outp, management, env=self.env, log=self.log)
        return [json.loads(l) for l in outp.getvalue().decode("utf-8").splitlines()]

    def call(self, name, **arguments):
        [r] = self.rpc(tool_call(1, name, **arguments))
        return r["result"]

    def tools(self, management=False):
        [r] = self.rpc(req(1, "tools/list"), management=management)
        return {t["name"]: t for t in r["result"]["tools"]}

    def json_of(self, result):
        return json.loads(result["content"][0]["text"])


class ProtocolTest(McpTestCase):
    def test_initialize_negotiates_the_version(self):
        known, unknown, missing = self.rpc(
            req(1, "initialize", {"protocolVersion": "2025-03-26", "capabilities": {}}),
            req(2, "initialize", {"protocolVersion": "1999-01-01"}),
            req(3, "initialize", {}),
        )
        self.assertEqual(known["result"]["protocolVersion"], "2025-03-26")
        self.assertEqual(unknown["result"]["protocolVersion"], PROTOCOL_VERSIONS[0])
        self.assertEqual(missing["result"]["protocolVersion"], PROTOCOL_VERSIONS[0])
        for v in ("2024-11-05", "2025-03-26", "2025-06-18"):
            self.assertIn(v, PROTOCOL_VERSIONS)
        r = known["result"]
        self.assertEqual(r["serverInfo"]["name"], "egs")
        self.assertEqual(r["capabilities"], {"tools": {"listChanged": False}})
        self.assertIn("egs_help", r["instructions"])
        self.assertIn("egs_root", r["instructions"])

    def test_ping_unknown_method_bad_json_notifications_batches(self):
        out = self.rpc(
            {"jsonrpc": "2.0", "method": "notifications/initialized"},
            req(1, "ping"),
            req(2, "resources/list"),
            "{not json",
            {"jsonrpc": "2.0", "method": "no/such/notification"},
            [req(3, "ping"), {"jsonrpc": "2.0", "method": "notifications/initialized"}],
            req("four", "tools/call", {"name": "egs_nothing"}),
            req(5, "tools/call", {"name": "egs_here", "arguments": [1]}),
        )
        self.assertEqual(len(out), 6, out)
        self.assertEqual(out[0], {"jsonrpc": "2.0", "id": 1, "result": {}})
        self.assertEqual(out[1]["error"]["code"], -32601)
        self.assertEqual(out[2]["error"]["code"], -32700)
        self.assertIsNone(out[2]["id"])
        self.assertEqual(out[3], [{"jsonrpc": "2.0", "id": 3, "result": {}}])
        self.assertEqual((out[4]["id"], out[4]["error"]["code"]), ("four", -32602))
        self.assertEqual(out[5]["error"]["code"], -32602)


class ToolListTest(McpTestCase):
    def test_names_and_schemas(self):
        tools = self.tools()
        names = list(tools)
        self.assertEqual(names, sorted(names))
        for name in (
            "egs_help", "egs_root", "egs_here", "egs_where", "egs_find", "egs_place_save",
            "egs_observe", "egs_view", "egs_fs_grep", "egs_fs_cat", "egs_fs_write",
            "egs_zone_add", "egs_init_pose", "egs_session_ls", "egs_session_rm",
        ):
            self.assertIn(name, tools)
        for name in (
            "egs_map_ls", "egs_map_new", "egs_session_freeze", "egs_session_new",
            "egs_mcp", "egs_help_agent", "egs_skill_install", "egs_skill_show",
        ):
            self.assertNotIn(name, tools)
        for t in tools.values():
            s = t["inputSchema"]
            self.assertEqual(s["type"], "object")
            self.assertLessEqual(set(s.get("required", [])), set(s["properties"]))
            self.assertNotIn("json", s["properties"])
            self.assertNotIn("robot", s["properties"])
            self.assertTrue(t["description"])
        save = tools["egs_place_save"]["inputSchema"]
        self.assertEqual(
            save["properties"]["offset"],
            dict(
                save["properties"]["offset"],
                type="array",
                items={"type": "number"},
                minItems=3,
                maxItems=3,
            ),
        )
        self.assertEqual(save["properties"]["no_scan"]["type"], "boolean")
        self.assertEqual(save["required"], ["path"])
        observe = tools["egs_observe"]["inputSchema"]["properties"]
        self.assertEqual(observe["result"]["enum"], ["seen", "absent", "not_observed"])
        self.assertEqual(observe["invalidates"]["type"], "integer")
        self.assertEqual(tools["egs_find"]["inputSchema"]["properties"]["query"]["type"], "array")
        self.assertNotIn("watch", tools["egs_pose"]["inputSchema"]["properties"])
        self.assertNotIn("output", tools["egs_fs_cat"]["inputSchema"]["properties"])
        self.assertIn("content", tools["egs_fs_write"]["inputSchema"]["properties"])
        self.assertIn("exactly one of: polygon, rect", tools["egs_zone_add"]["description"])
        self.assertIn("CLI: egs place save", tools["egs_place_save"]["description"])

    def test_management_tools_only_on_request(self):
        tools = self.tools(management=True)
        for name in ("egs_map_ls", "egs_map_new", "egs_map_open", "egs_session_freeze"):
            self.assertIn(name, tools)
        self.assertNotIn("force", tools["egs_session_freeze"]["inputSchema"]["properties"])

    def test_every_usage_command_has_a_tool(self):
        tools = self.tools()
        excluded = {("help",), ("mcp",), ("skill", "install"), ("skill", "show")}
        for path, group_help in leaf_commands(cli.build_parser()):
            if path in excluded or group_help.startswith("management"):
                continue
            name = "egs_" + "_".join(p.replace("-", "_") for p in path)
            self.assertTrue(name in tools, path)


class ToolCallTest(McpTestCase):
    def test_help_is_the_skill(self):
        r = self.call("egs_help")
        self.assertFalse(r["isError"])
        self.assertTrue(r["content"][0]["text"].startswith("---\nname: evergreenslam"))

    def test_here_answers_json(self):
        r = self.call("egs_here")
        self.assertFalse(r["isError"], r)
        self.assertTrue(self.json_of(r)["ok"])
        self.assertEqual(self.json_of(r)["robot"], [0.0, 0.0, 0.0])

    def test_refusal_keeps_the_reason(self):
        self.state.keyframe = False
        r = self.call("egs_place_save", path="places/hall")
        self.assertTrue(r["isError"])
        self.assertEqual(self.json_of(r)["reason"], "no_keyframe")

    def test_numbers_and_negative_positionals_reach_argparse_exactly(self):
        tool = {t.name: t for t in mcptools.build_tools(cli.build_parser())}["egs_place_save"]
        opts, pos = mcptools.build_argv(tool, {"path": "-odd", "offset": [1e-05, -0.5, 3]})
        args = cli.build_parser().parse_args(["place", "save"] + opts + ["--"] + pos)
        self.assertEqual(args.offset, [1e-05, -0.5, 3.0])
        self.assertEqual(args.path, "-odd")
        r = self.call("egs_init_pose", pose=[-1.5, 2, 0.25])
        self.assertFalse(r["isError"], r)
        q = self.requests("POST", "/init-pose")[-1][2]
        self.assertEqual((q["x"], q["y"], q["theta"]), ("-1.5", "2.0", "0.25"))

    def test_view_returns_the_image(self):
        r = self.call("egs_view", preset="map")
        self.assertFalse(r["isError"], r)
        self.assertTrue(self.json_of(r)["png"].endswith(".png"))
        image = r["content"][1]
        self.assertEqual((image["type"], image["mimeType"]), ("image", "image/png"))
        self.assertEqual(base64.b64decode(image["data"]), PNG)

    def test_fs_cat_text_image_and_binary(self):
        with open(os.path.join(self.mem, "places/kitchen/photo.png"), "wb") as f:
            f.write(PNG)
        with open(os.path.join(self.mem, "places/kitchen/blob.bin"), "wb") as f:
            f.write(b"\xff\xfe\x00\x01")
        for remote in (False, True):
            if remote:
                self.remote_only()
            r = self.call("egs_fs_cat", path="README.md")
            self.assertEqual(r["content"], [{"type": "text", "text": "# memory\n"}])
            r = self.call("egs_fs_cat", path="places/kitchen/photo.png")
            self.assertEqual(base64.b64decode(r["content"][-1]["data"]), PNG)
            r = self.call("egs_fs_cat", path="places/kitchen/blob.bin")
            local = r["content"][0]["text"].rsplit(" ", 1)[1]
            with open(local, "rb") as f:
                self.assertEqual(f.read(), b"\xff\xfe\x00\x01")
            r = self.call("egs_fs_cat", path="places/kitchen/none.md")
            self.assertTrue(r["isError"])

    def test_fs_write_and_append_take_content(self):
        for remote in (False, True):
            if remote:
                self.remote_only()
            r = self.call("egs_fs_write", path="places/kitchen/notes.md", content="ramp 是 left\n")
            self.assertFalse(r["isError"], r)
            r = self.call("egs_fs_append", path="places/kitchen/notes.md", content="two\n")
            self.assertFalse(r["isError"], r)
            self.assertEqual(self.read("places/kitchen/notes.md"), "ramp 是 left\ntwo\n")
        r = self.call("egs_fs_write", path="places/kitchen/notes.md")
        self.assertTrue(r["isError"])
        self.assertIn("content", r["content"][0]["text"])

    def test_bad_arguments_are_tool_errors_and_the_server_keeps_going(self):
        out = self.rpc(
            tool_call(1, "egs_where"),
            tool_call(2, "egs_where", path="places/kitchen", depth=3),
            tool_call(3, "egs_place_save", path="places/hall", offset=[1, 2]),
            tool_call(4, "egs_observe", path="places/kitchen", result="maybe"),
            tool_call(5, "egs_find", query={"a": 1}),
            req(6, "ping"),
        )
        for r in out[:5]:
            self.assertTrue(r["result"]["isError"], r)
        self.assertEqual(self.json_of(out[0]["result"])["reason"], "usage")
        self.assertIn("unknown parameter depth", out[1]["result"]["content"][0]["text"])
        self.assertIn("invalid choice", out[3]["result"]["content"][0]["text"])
        self.assertEqual(out[5]["result"], {})

    def test_fs_zone_and_checkpoint_answer_json(self):
        for remote in (False, True):
            if remote:
                self.remote_only()
            r = self.call("egs_fs_ls", path="places")
            self.assertFalse(r["isError"], r)
            self.assertIn({"name": "kitchen", "type": "dir", "size": 0}, self.json_of(r)["entries"])
            r = self.call("egs_fs_tree", path="places", depth=1)
            self.assertNotIn("apple", self.json_of(r)["tree"])
            r = self.call("egs_fs_mkdir", path="places/kitchen/shelf")
            self.assertEqual(self.json_of(r)["path"], "places/kitchen/shelf")
            r = self.call("egs_fs_mv", src="places/kitchen/shelf", dst="places/kitchen/rack")
            self.assertEqual(self.json_of(r)["to"], "places/kitchen/rack")
            r = self.call("egs_fs_rm", path="places/kitchen/rack")
            self.assertEqual(self.json_of(r)["path"], "places/kitchen/rack")
            r = self.call("egs_fs_write", path="places/kitchen/notes.md", content="x\n")
            self.assertEqual(self.json_of(r)["path"], "places/kitchen/notes.md")
            r = self.call("egs_zone_add", path="places/kitchen/wet", rect=[1, -1, 3, 1])
            self.assertFalse(r["isError"], r)
            self.assertEqual(self.json_of(r)["frame"], "places/kitchen")
            r = self.call("egs_zone_rm", path="places/kitchen/wet")
            self.assertTrue(self.json_of(r)["removed_dir"])
        r = self.call("egs_checkpoint")
        self.assertEqual(self.json_of(r)["num_checkpoints_written"], 4)
        r = self.call("egs_fs_rm", path="places/kitchen")
        self.assertTrue(r["isError"])
        self.assertEqual(self.json_of(r)["reason"], "not_empty")


class StdioTest(McpTestCase):
    def test_python_m_egs_mcp_over_pipes(self):
        config = os.path.join(self.tmp, "config", "egs")
        os.makedirs(config)
        with open(os.path.join(config, "config"), "w") as f:
            f.write("[robot.home]\nurl = %s\n" % self.service.url)
        env = dict(os.environ, XDG_CONFIG_HOME=os.path.join(self.tmp, "config"))
        env.pop("EGS_URL", None)
        env.pop("EGS_ROBOT", None)
        msgs = [
            req(1, "initialize", {"protocolVersion": "2025-06-18", "capabilities": {}}),
            {"jsonrpc": "2.0", "method": "notifications/initialized"},
            req(2, "tools/list"),
            tool_call(3, "egs_here"),
            tool_call(4, "egs_where", nope=1),
            "garbage",
            req(5, "ping"),
        ]
        p = subprocess.run(
            [sys.executable, "-m", "egs", "--robot", "home", "mcp"],
            input="".join((m if isinstance(m, str) else json.dumps(m)) + "\n" for m in msgs),
            capture_output=True,
            text=True,
            cwd=EGS_DIR,
            env=env,
            timeout=60,
        )
        self.assertEqual(p.returncode, 0, p.stderr)
        out = [json.loads(l) for l in p.stdout.splitlines()]
        self.assertTrue(all(m["jsonrpc"] == "2.0" for m in out))
        self.assertEqual([m["id"] for m in out], [1, 2, 3, 4, None, 5])
        self.assertEqual(out[0]["result"]["protocolVersion"], "2025-06-18")
        self.assertFalse(out[2]["result"]["isError"], out[2])
        self.assertTrue(self.json_of(out[2]["result"])["ok"])
        self.assertTrue(out[3]["result"]["isError"])
        self.assertIn("egs mcp", p.stderr)


if __name__ == "__main__":
    unittest.main()
