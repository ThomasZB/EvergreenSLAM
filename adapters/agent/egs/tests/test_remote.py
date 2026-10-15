import io
import json
import os
import socket
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest import mock

from support import EgsTestCase
from fake_service import PNG

from egs import config
from egs.cli import main
from egs.client import EgsError

SKILL = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "skills",
    "evergreenslam",
    "SKILL.md",
)


def closed_port_url():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return "http://127.0.0.1:%d" % s.getsockname()[1]


class _BadGateway(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        self.send_response(502)
        self.send_header("Content-Length", "0")
        self.end_headers()


class RemoteCase(EgsTestCase):
    def egs_bytes(self, *argv, **env):
        out, err = io.TextIOWrapper(io.BytesIO(), encoding="utf-8"), io.StringIO()
        e = dict(self.env)
        e.update(env)
        code = main(list(argv), env=e, out=out, err=err)
        out.flush()
        return code, out.buffer.getvalue(), err.getvalue()

    def mismatch(self):
        with open(os.path.join(self.tmp, "map", "agent_instance"), "w") as f:
            f.write("0" * 32 + "\n")

    def mode(self, **env):
        code, out, err = self.egs("root", "--json", **env)
        self.assertEqual(code, 0, err)
        return json.loads(out)


class ConfigTest(RemoteCase):
    def setUp(self):
        super().setUp()
        self.dead = closed_port_url()
        path = os.path.join(self.tmp, "config", "egs", "config")
        os.makedirs(os.path.dirname(path))
        with open(path, "w") as f:
            f.write(
                "[default]\ntimeout = 7\n\n[robot.lab]\nurl = %s\n\n[robot.dead]\nurl = %s\n"
                "\n[robot.mounted]\nurl = %s\nmemory = %s\n"
                % (self.service.url, self.dead, self.service.url, self.mem)
            )

    def load(self, robot=None, **env):
        e = {"XDG_CONFIG_HOME": self.env["XDG_CONFIG_HOME"]}
        e.update(env)
        return config.load(e, robot)

    def test_precedence(self):
        s = self.load()
        self.assertEqual((s.url, s.timeout, s.memory), ("http://127.0.0.1:8643", 7.0, None))
        self.assertEqual(self.load(EGS_TIMEOUT="3").timeout, 3.0)
        self.assertEqual(self.load(EGS_ROBOT="lab").url, self.service.url)
        self.assertEqual(self.load(EGS_ROBOT="lab", EGS_URL="http://x:1").url, "http://x:1")
        self.assertEqual(self.load("lab", EGS_URL="http://x:1").url, self.service.url)
        self.assertEqual(self.load("lab", EGS_ROBOT="dead").url, self.service.url)
        self.assertEqual(self.load("lab", EGS_TIMEOUT="3").timeout, 3.0)
        with self.assertRaises(EgsError) as e:
            self.load("nope")
        self.assertEqual(e.exception.reason, "unknown_robot")

    def test_robot_option_and_env_reach_the_service(self):
        code, out, err = self.egs("--robot", "lab", "here", EGS_URL=self.dead)
        self.assertEqual(code, 0, err)
        self.assertTrue(out.startswith("current places/dock"))
        code, _, err = self.egs("here", EGS_URL=self.dead)
        self.assertEqual(code, 3)
        self.assertIn(self.dead, err)
        self.assertIn("ssh -NT", err)
        self.assertEqual(self.egs("here", EGS_URL=self.dead, EGS_ROBOT="lab")[0], 3)
        self.assertEqual(self.egs("here", EGS_URL="", EGS_ROBOT="lab")[0], 0)
        code, _, err = self.egs("--robot", "nope", "here")
        self.assertEqual(code, 2)
        self.assertIn("[robot.nope]", err)

    def test_config_memory_is_an_explicit_local_dir(self):
        self.remote_only()
        root = self.mode(EGS_ROBOT="mounted")
        self.assertEqual((root["mode"], root["local_dir"]), ("local", self.mem))
        code, _, err = self.egs("--robot", "mounted", "where", "places/kitchen/table")
        self.assertEqual(code, 0, err)
        self.assertEqual(self.requests("GET", "/fs/ls"), [])


class ProxyTest(RemoteCase):
    def test_environment_proxies_are_ignored_unless_asked(self):
        proxy = ThreadingHTTPServer(("127.0.0.1", 0), _BadGateway)
        threading.Thread(target=proxy.serve_forever, daemon=True).start()
        url = "http://127.0.0.1:%d" % proxy.server_address[1]
        environ = {k: v for k, v in os.environ.items() if k.lower() not in ("no_proxy",)}
        environ.update(http_proxy=url, HTTP_PROXY=url)
        try:
            with mock.patch.dict(os.environ, environ, clear=True):
                code, out, err = self.egs("here")
                self.assertEqual(code, 0, err)
                code, _, err = self.egs("here", EGS_USE_PROXY="1")
                self.assertEqual(code, 3)
                self.assertIn("HTTP 502", err)
        finally:
            proxy.shutdown()
            proxy.server_close()


class InstanceModeTest(RemoteCase):
    def test_matching_instance_works_on_the_files(self):
        root = self.mode()
        self.assertEqual(root["mode"], "local")
        self.assertEqual(root["instance_id"], self.state.instance_id)
        self.assertEqual(root["local_dir"], self.mem)
        self.assertTrue(root["readable"])
        self.assertEqual(self.egs("root")[1], self.mem + "\n")
        self.assertEqual(self.egs("where", "places/kitchen/table")[0], 0)
        self.assertEqual(self.requests("GET", "/fs/ls"), [])

    def test_mismatching_or_missing_instance_goes_through_fs(self):
        marker = os.path.join(self.tmp, "map", "agent_instance")
        for setup in (self.mismatch, lambda: os.remove(marker)):
            setup()
            del self.state.requests[:]
            root = self.mode()
            self.assertEqual(
                (root["mode"], root["local_dir"], root["readable"]), ("remote", None, False)
            )
            self.assertEqual(root["memory_dir"], self.mem)
            self.assertEqual(root["url"], self.service.url)
            self.assertEqual(root["map"], "map")
            code, out, err = self.egs("observe", "places/kitchen/fridge/milk", "--note", "cold")
            self.assertEqual(code, 0, out + err)
            self.assertTrue(self.requests("GET", "/fs/ls"))
            self.assertTrue(self.requests("POST", "/fs/append"))

    def test_unreachable_root_names_the_url_and_the_tunnel(self):
        dead = closed_port_url()
        code, out, err = self.egs("root", EGS_URL=dead)
        self.assertEqual(code, 3)
        self.assertIn(dead, err)
        self.assertIn("ssh -NT", err)
        self.assertIn("EGS_URL", err)


class GrepTest(RemoteCase):
    def setUp(self):
        super().setUp()
        self.write("places/kitchen/notes.md", "The Galley door sticks\r\nnothing\nGALLEY light\n")
        self.write("places/kitchen/observations.jsonl", '{"note": "galley"}\n')
        self.write("places/kitchen/fridge/attachments/1.md", "galley\n")
        self.write("places/kitchen/data.txt", "galley\n")
        self.write("places/kitchen/big.md", "galley\n" + "x" * (1 << 20))
        self.write("places/office/SKILL.md", "axb\n")
        os.symlink(
            os.path.join(self.mem, "places/kitchen/notes.md"),
            os.path.join(self.mem, "places/office/link.md"),
        )

    def both(self, *argv):
        local = self.egs("fs", "grep", *argv)
        self.assertEqual(self.requests("GET", "/fs/grep"), [])
        self.mismatch()
        remote = self.egs("fs", "grep", *argv)
        self.assertTrue(self.requests("GET", "/fs/grep"))
        self.assertEqual(local, remote)
        with open(os.path.join(self.tmp, "map", "agent_instance"), "w") as f:
            f.write(self.state.instance_id)
        del self.state.requests[:]
        return local

    def test_fixed_string_any_case_with_skips(self):
        code, out, err = self.both("galley")
        self.assertEqual(code, 0, err)
        self.assertEqual(
            out.splitlines(),
            [
                "places/kitchen/node.yaml:2:aliases: [厨房, galley]",
                "places/kitchen/notes.md:1:The Galley door sticks",
                "places/kitchen/notes.md:3:GALLEY light",
            ],
        )
        _, out, _ = self.both("GALLEY", "--path", "places/kitchen", "--include", "*.txt, node.yaml")
        self.assertEqual(
            out.splitlines(),
            [
                "places/kitchen/data.txt:1:galley",
                "places/kitchen/node.yaml:2:aliases: [厨房, galley]",
            ],
        )
        self.assertEqual(self.both("a.b")[1], "")
        node = self.both("厨房", "--path", "places/kitchen/node.yaml")[1]
        self.assertEqual(node.count("\n"), 1)
        code, out, _ = self.both("--json", "light")
        self.assertEqual(
            json.loads(out),
            {
                "ok": True,
                "reason": None,
                "pattern": "light",
                "matches": [{"path": "places/kitchen/notes.md", "line": 3, "text": "GALLEY light"}],
                "truncated": False,
            },
        )

    def test_truncates_at_two_hundred(self):
        self.write("places/hall/notes.md", "".join("galley %d\n" % i for i in range(250)))
        code, out, err = self.both("--json", "galley", "--path", "places/hall")
        j = json.loads(out)
        self.assertEqual((len(j["matches"]), j["truncated"]), (200, True))
        code, out, err = self.egs("fs", "grep", "galley", "--path", "places/hall")
        self.assertEqual(len(out.splitlines()), 200)
        self.assertIn("stopped at 200", err)

    def test_refusals(self):
        self.assertEqual(self.egs("fs", "grep", "x", "--path", "places/office/link.md")[0], 2)
        self.assertEqual(self.egs("fs", "grep", "x", "--path", "../")[0], 2)
        self.assertEqual(self.egs("fs", "grep", "", "--path", "places")[0], 2)
        self.assertEqual(self.egs("fs", "grep", "x", "--path", "places/none")[0], 1)
        self.assertEqual(self.egs("fs", "grep")[0], 2)

    def test_several_words_are_one_phrase(self):
        self.write("places/kitchen/table/node.yaml", "kind: table\naliases: [hand truck]\n")
        self.write("places/kitchen/table/notes.md", "a truck, by hand\n")
        code, out, err = self.both("hand", "truck")
        self.assertEqual(code, 0, err)
        self.assertEqual(out, "places/kitchen/table/node.yaml:2:aliases: [hand truck]\n")
        self.assertEqual(self.both("hand   truck", "--path", "places/kitchen")[1], out)
        code, out, err = self.egs("fs", "grep", "galley", "places/kitchen")
        self.assertEqual((code, out), (0, ""))
        self.assertIn("use --path places/kitchen", err)


class BytesToStdoutTest(RemoteCase):
    def test_view_png_to_stdout(self):
        code, out, err = self.egs_bytes("view", "map", "-o", "-")
        self.assertEqual(code, 0, err)
        self.assertEqual(out, PNG)
        self.assertTrue(err.startswith("layers=map,places,zones"))
        code, out, err = self.egs_bytes("view", "map", "-o", "-", "--json")
        self.assertEqual(code, 2)
        self.assertEqual(json.loads(out)["ok"], False)
        code, out, err = self.egs_bytes("view", "route", "places/office", "-o", "-")
        self.assertEqual((code, out), (1, b""))
        self.assertIn("no_binding", err)

    def test_fs_cat_to_stdout(self):
        blob = bytes(range(256))
        with open(os.path.join(self.mem, "places/dock/photo.jpg"), "wb") as f:
            f.write(blob)
        code, out, err = self.egs_bytes("fs", "cat", "places/dock/photo.jpg", "-o", "-")
        self.assertEqual((code, out), (0, blob), err)


class RemoteFilesTest(RemoteCase):
    def test_view_never_trusts_the_server_path_when_remote(self):
        self.mismatch()
        code, out, err = self.egs("view", "map")
        self.assertEqual(code, 0, err)
        png = out.splitlines()[-1]
        views = os.path.join(tempfile.gettempdir(), "egs-views", self.state.instance_id)
        self.assertEqual(png, os.path.join(views, "000007_map.png"))
        with open(png, "rb") as f:
            self.assertEqual(f.read(), PNG)

    def test_snapshot_downloads_when_remote(self):
        self.remote_only()
        code, out, err = self.egs("snapshot")
        self.assertEqual(code, 0, err)
        self.assertIn("-o DIR downloads them", out)
        dest = os.path.join(self.tmp, "export")
        code, out, err = self.egs("snapshot", "-o", dest, "--json")
        self.assertEqual(code, 0, err)
        j = json.loads(out)
        self.assertEqual((j["local_dir"], j["copied_to"]), (None, dest))
        self.assertEqual(sorted(os.listdir(dest)), ["map.png", "summary.txt"])
        self.assertEqual(
            sorted(q["name"] for _, _, q in self.requests("GET", "/snapshots/file")),
            ["map.png", "summary.txt"],
        )


class SkillTest(RemoteCase):
    def test_show_and_install_the_one_skill(self):
        with open(SKILL, encoding="utf-8") as f:
            text = f.read()
        self.assertIn("\nname: evergreenslam\n", text)
        code, out, _ = self.egs("skill", "show")
        self.assertEqual((code, out), (0, text))
        skills = os.path.join(self.tmp, "skills")
        code, out, err = self.egs("skill", "install", skills)
        self.assertEqual(code, 0, err)
        dest = os.path.join(skills, "evergreenslam", "SKILL.md")
        self.assertEqual(out, dest + "\n")
        with open(dest, encoding="utf-8") as f:
            self.assertEqual(f.read(), text)
        self.assertEqual(self.state.requests, [])


if __name__ == "__main__":
    unittest.main()
