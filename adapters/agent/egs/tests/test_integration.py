"""egs against the real AgentService: a fake host (service/test/fake_host_main.cc) drives a
PoseGraph through simulated laps and serves it. Skipped unless EGS_FAKE_HOST names the binary."""

import io
import os
import re
import select
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from egs.cli import main  # noqa: E402

FAKE_HOST = os.environ.get("EGS_FAKE_HOST")
FAKE_HOST = os.path.abspath(FAKE_HOST) if FAKE_HOST else None
READY_TIMEOUT_S = 60
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


@unittest.skipUnless(FAKE_HOST, "set EGS_FAKE_HOST to the fake_host binary")
class IntegrationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="egs-integration-")
        self.map_dir = os.path.join(self.tmp, "map")
        self.log = open(os.path.join(self.tmp, "fake_host.log"), "w+")
        self.host = subprocess.Popen(
            [FAKE_HOST, "--port", "0", "--map_dir", self.map_dir],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=self.log,
            text=True,
        )
        ready, _, _ = select.select([self.host.stdout], [], [], READY_TIMEOUT_S)
        line = self.host.stdout.readline() if ready else ""
        m = re.match(r"READY (\d+)", line)
        if not m:
            self._stop_host()
            self.fail("fake host did not start: %r\n%s" % (line, self._host_log()))
        self.env = {"EGS_URL": "http://127.0.0.1:%s" % m.group(1), "EGS_POLL_S": "2"}

    def tearDown(self):
        self._stop_host()
        self.log.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _stop_host(self):
        if self.host.poll() is None:
            self.host.stdin.close()
            try:
                self.host.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.host.kill()
                self.host.wait()
        self.host.stdout.close()

    def _host_log(self):
        self.log.flush()
        self.log.seek(0)
        return self.log.read()[-4000:]

    def egs(self, *argv, **env):
        out, err = io.StringIO(), io.StringIO()
        e = dict(self.env)
        e.update(env)
        code = main(list(argv), env=e, out=out, err=err)
        return code, out.getvalue(), err.getvalue()

    def ok(self, *argv):
        code, out, err = self.egs(*argv)
        self.assertEqual(code, 0, "egs %s -> %d\n%s%s" % (" ".join(argv), code, out, err))
        return out

    def test_local_memory(self):
        self._scenario()

    def test_remote_memory(self):
        # A memory dir this machine cannot see: every tree access goes through /fs.
        self.env["EGS_MEMORY"] = os.path.join(self.tmp, "not-mounted", "memory")
        self._scenario()

    def _scenario(self):
        remote = "EGS_MEMORY" in self.env
        memory = os.path.realpath(os.path.join(self.map_dir, "memory"))
        code, root, err = self.egs("root")
        self.assertEqual(code, 0, err)
        self.assertEqual(root.strip(), self.env["EGS_MEMORY"] if remote else memory)
        self.assertEqual("not readable here" in err, remote, err)
        self.assertTrue(self.ok("status").startswith("map map  boot"))
        self.assertIn("* map", self.ok("map", "ls").splitlines())
        code, out, _ = self.egs("map", "new", "other")
        self.assertEqual(code, 1, out)
        self.assertIn("not_supported", out)

        first = self.ok("place", "save", "places/dock")
        m = re.search(r"saved places/dock anchor (\d+) \(new\)", first)
        self.assertIsNotNone(m, first)
        anchor = m.group(1)
        second = self.ok("place", "save", "places/dock", "--no-scan")
        self.assertIn("anchor %s (re-bound existing)" % anchor, second)
        with open(os.path.join(memory, "places/dock/place.yaml")) as f:
            self.assertEqual(f.read(), "anchor: %s\n" % anchor)

        self.ok("fs", "mkdir", "places/dock/shelf/cup")
        self.assertIn("precision own", self.ok("where", "places/dock"))
        shelf = self.ok("where", "places/dock/shelf")
        self.assertIn("precision inherited", shelf)
        self.assertIn("resolved_via places/dock", shelf)
        self.assertIn("current places/dock", self.ok("here"))

        observed = self.ok("observe", "places/dock/shelf/cup", "--offset", "0.5", "0", "0")
        self.assertIn("written to places/dock/shelf/cup/node.yaml", observed)
        with open(os.path.join(memory, "places/dock/shelf/cup/node.yaml")) as f:
            self.assertIn("offset_from: %s" % anchor, f.read())
        cup = self.ok("where", "places/dock/shelf/cup")
        self.assertIn("precision offset", cup)
        self.assertTrue(self.ok("find", "cup").startswith("places/dock/shelf/cup [cup]"))

        snapshot = self.ok("snapshot")
        m = re.search(r"^snapshot \d+  (\S+)$", snapshot, re.M)
        self.assertIsNotNone(m, snapshot)
        self.assertTrue(os.path.isfile(os.path.join(m.group(1), "map.pgm")))
        self.assertTrue(os.path.isfile(os.path.join(memory, "index.tsv")))

        lines = self.ok("view", "here").splitlines()
        self.assertTrue(lines[0].startswith("layers=map,robot,scan"), lines)
        self.assertEqual([l for l in lines if l.startswith("layers=")], [lines[0]], lines)
        png = lines[-1]
        self.assertEqual(
            os.path.realpath(os.path.dirname(png)),
            os.path.join(os.path.realpath(self.map_dir), "views"),
        )
        with open(png, "rb") as f:
            self.assertEqual(f.read(len(PNG_SIGNATURE)), PNG_SIGNATURE)
        self.assertRegex(self.ok("view", "map"), r"(?m)^1 places/dock \d")

        self._fs()

        ls = self.ok("session", "ls")
        fed = re.match(r"fed (\d+)", ls).group(1)
        self.assertRegex(ls, r"(?m)^%s fed " % fed)
        code, out, _ = self.egs("session", "rm", fed)
        self.assertEqual(code, 1, out)
        self.assertIn("fed_session", out)
        code, out, _ = self.egs("session", "freeze")
        rejected = "rejected" in out
        self.assertEqual(code, 1 if rejected else 0, out)
        self.assertEqual("plan only: add --yes to apply" in out, not rejected, out)
        self.assertRegex(self.ok("session", "ls"), r"(?m)^%s fed " % fed)

    def _fs(self):
        note = os.path.join(self.tmp, "note.md")
        with open(note, "w") as f:
            f.write("blue mug\n")
        self.ok("fs", "write", "places/dock/notes.md", "--file", note)
        self.ok("fs", "append", "places/dock/notes.md", "--file", note)
        self.assertEqual(self.ok("fs", "cat", "places/dock/notes.md"), "blue mug\nblue mug\n")
        code, out, _ = self.egs("fs", "cat", "places/dock/missing.md")
        self.assertEqual(code, 1, out)
        self.assertIn("cannot read places/dock/missing.md", out)
        self.assertIn("places/", self.ok("fs", "ls", ".").splitlines())
        self.assertIn("dock", self.ok("fs", "tree", ".", "-L", "3"))
        self.ok("fs", "mv", "places/dock/notes.md", "places/dock/shelf/notes.md")
        self.assertIn("notes.md  18", self.ok("fs", "ls", "places/dock/shelf"))
        code, _, err = self.egs("fs", "rm", "places/dock/place.yaml")
        self.assertEqual(code, 2, err)
        self.assertIn("owned_by_process", err)
        code, _, err = self.egs("fs", "rm", "places/dock/shelf")
        self.assertEqual(code, 1, err)
        self.assertIn("not_empty", err)
        self.ok("fs", "rm", "places/dock/shelf/notes.md")
        self.assertNotIn("notes.md", self.ok("fs", "ls", "places/dock/shelf"))


if __name__ == "__main__":
    unittest.main()
