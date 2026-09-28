"""Shared fixture: a temp memory tree, a fake service, and an in-process egs runner."""

import io
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from egs.cli import main  # noqa: E402
from fake_service import FakeService, FakeState, pose  # noqa: E402

NS_MIN = 60 * 10**9
NS_DAY = 86400 * 10**9

TREE = {
    "README.md": "# memory\n",
    "places/SKILL.md": "---\nname: home\n---\n",
    "places/skills/return-to-dock/SKILL.md": "---\nname: return-to-dock\n---\n",
    "places/dock/place.yaml": "anchor: 1\n",
    "places/kitchen/place.yaml": "anchor: 2\n",
    "places/kitchen/node.yaml": "kind: room\naliases: [厨房, galley]\n",
    "places/kitchen/skills/clean/SKILL.md": "---\nname: clean\n---\n",
    "places/kitchen/table/node.yaml": "kind: table\noffset: [1.0, 0.0, 0.0]\noffset_from: 2\n",
    "places/kitchen/table/apple/node.yaml": "kind: apple  # red\naliases: [苹果]\n",
    "places/kitchen/fridge/place.yaml": "anchor: 3\n",
    "places/kitchen/fridge/node.yaml": "kind: fridge\naliases: [icebox]\n",
    "places/kitchen/fridge/milk/node.yaml": "kind: milk\nlabels: [food]\n",
    "places/office/node.yaml": "kind: room\n",
    "places/office/desk/place.yaml": "anchor: 4\n",
    "places/office/desk/apple/node.yaml": "kind: apple\noffset: [0.5, 0.0, 0.0]\noffset_from: 4\n",
    "places/office/desk/pineapple/node.yaml": "kind: fruit\n",
    "places/office/desk/green_apple/node.yaml": "kind: fruit\n",
    "places/garage/node.yaml": "kind: room\n",
    "places/garage/shelf/place.yaml": "anchor: 5\n",
}


def obs(recorded_at, result, **kw):
    import json

    d = {"recorded_at": recorded_at, "source": "agent", "result": result}
    d.update(kw)
    return json.dumps(d) + "\n"


class EgsTestCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="egs-test-")
        self.mem = os.path.join(self.tmp, "map", "memory")
        for rel, text in TREE.items():
            p = os.path.join(self.mem, rel)
            os.makedirs(os.path.dirname(p), exist_ok=True)
            with open(p, "w", encoding="utf-8") as f:
                f.write(text)
        now = time.time_ns()
        self.write(
            "places/kitchen/table/apple/observations.jsonl", obs(now - 20 * NS_MIN, "absent")
        )
        self.write("places/office/desk/apple/observations.jsonl", obs(now - 6 * NS_DAY, "seen"))
        self.state = FakeState(self.mem, os.path.join(self.tmp, "map"))
        self.state.anchor(1, pose(0.0, 0.0, 0.0))
        self.state.anchor(2, pose(5.0, 0.0, 0.0))
        self.state.anchor(3, pose(6.0, -2.0, 0.0), state="pending")
        self.state.anchor(4, pose(-3.0, 4.0, math.pi / 2))
        self.state.anchor(5, None, state="orphan", orphan_reason="session_removed")
        self.service = FakeService(self.state)
        self.env = {"EGS_URL": self.service.url, "EGS_POLL_S": "2"}

    def tearDown(self):
        self.service.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, rel, text):
        p = os.path.join(self.mem, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            f.write(text)

    def read(self, rel):
        with open(os.path.join(self.mem, rel), encoding="utf-8") as f:
            return f.read()

    def remote_only(self):
        """The process reports a memory dir this machine cannot see: egs must use /fs."""
        self.state.root_memory_dir = os.path.join(self.tmp, "not-mounted", "memory")

    def egs(self, *argv, **env):
        out, err = io.StringIO(), io.StringIO()
        e = dict(self.env)
        e.update(env)
        code = main(list(argv), env=e, out=out, err=err)
        return code, out.getvalue(), err.getvalue()

    def requests(self, method, path):
        return self.state.paths_of(method, path)
