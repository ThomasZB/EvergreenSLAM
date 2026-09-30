import json
import math
import os
import unittest

from fake_service import pose
from support import EgsTestCase

from egs import se2
from egs.yamlmini import loads


class ObserveTest(EgsTestCase):
    def lines(self, rel):
        return [json.loads(l) for l in self.read(rel).splitlines()]

    def test_appends_strictly_increasing_lines(self):
        rel = "places/kitchen/fridge/milk/observations.jsonl"
        self.write(
            rel, '{"recorded_at": 9999999999999999999, "source": "agent", "result": "seen"}\n'
        )
        code, out, _ = self.egs(
            "observe", "places/kitchen/fridge/milk", "--result", "absent", "--note", "shelf empty"
        )
        self.assertEqual(code, 0)
        code, _, _ = self.egs("observe", "places/kitchen/fridge/milk")
        lines = self.lines(rel)
        self.assertEqual(len(lines), 3)
        self.assertEqual(lines[1]["result"], "absent")
        self.assertEqual(lines[1]["note"], "shelf empty")
        self.assertEqual(lines[1]["source"], "agent")
        self.assertEqual(lines[2]["result"], "seen")
        self.assertGreater(lines[1]["recorded_at"], lines[0]["recorded_at"])
        self.assertGreater(lines[2]["recorded_at"], lines[1]["recorded_at"])
        self.assertIn("id=%d" % lines[1]["recorded_at"], out)
        self.assertFalse(self.requests("GET", "/anchors/3"))

    def test_invalidates(self):
        rel = "places/office/desk/apple/observations.jsonl"
        first = self.lines(rel)[0]["recorded_at"]
        code, _, _ = self.egs(
            "observe", "places/office/desk/apple", "--invalidates", str(first), "--source", "human"
        )
        self.assertEqual(code, 0)
        last = self.lines(rel)[-1]
        self.assertEqual(last["invalidates"], first)
        self.assertNotIn("result", last)
        self.assertEqual(last["source"], "human")

    def test_offset_writes_node_yaml(self):
        # A = (1, 0, pi/2), robot = (1, 1, 0), thing 0.5 m ahead of the robot -> world (1.5, 1).
        self.state.anchor(3, pose(1.0, 0.0, math.pi / 2))
        self.state.robot = pose(1.0, 1.0, 0.0)
        code, out, _ = self.egs(
            "observe", "places/kitchen/fridge/milk", "--offset", "0.5", "0", "0"
        )
        self.assertEqual(code, 0, out)
        self.assertEqual(self.requests("GET", "/anchors/3")[0][2], {"robot": "1"})
        node = loads(self.read("places/kitchen/fridge/milk/node.yaml"))
        self.assertEqual(node["kind"], "milk")
        self.assertEqual(node["labels"], ["food"])
        self.assertEqual(node["offset_from"], 3)
        dx, dy, dth = node["offset"]
        self.assertAlmostEqual(dx, 1.0, places=3)
        self.assertAlmostEqual(dy, -0.5, places=3)
        self.assertAlmostEqual(dth, -math.pi / 2, places=3)
        world = se2.compose((1.0, 0.0, math.pi / 2), (dx, dy, dth))
        self.assertAlmostEqual(world[0], 1.5, places=3)
        self.assertAlmostEqual(world[1], 1.0, places=3)
        self.assertEqual(len(self.lines("places/kitchen/fridge/milk/observations.jsonl")), 1)
        code, out, _ = self.egs("where", "places/kitchen/fridge/milk")
        self.assertIn("xy 1.50,1.00", out)
        self.assertIn("precision offset", out)

    def test_offset_on_a_new_node_needs_a_kind(self):
        self.write("places/kitchen/table/red_mug/.keep", "")
        code, _, _ = self.egs("observe", "places/kitchen/table/red_mug", "--offset", "0", "0", "0")
        self.assertEqual(code, 0)
        self.assertEqual(loads(self.read("places/kitchen/table/red_mug/node.yaml"))["kind"], "mug")

    def test_offset_refusals(self):
        self.write("places/garage/box/.keep", "")
        code, out, _ = self.egs("observe", "places/garage/box", "--offset", "0", "0", "0")
        self.assertEqual(code, 1)
        self.assertIn("egs place save", out)
        code, out, _ = self.egs("observe", "places/kitchen", "--offset", "0", "0", "0")
        self.assertEqual(code, 1)
        self.state.robot = None
        code, out, _ = self.egs("observe", "places/kitchen/table", "--offset", "0", "0", "0")
        self.assertEqual(code, 1)
        self.assertIn("no_robot_pose", out)
        self.write("places/kitchen/table/node.yaml", "kind: table\npos:\n  x: 1\n")
        code, out, _ = self.egs("observe", "places/kitchen/table", "--offset", "0", "0", "0")
        self.assertEqual(code, 1)
        self.assertIn("flat subset", out)
        self.assertIn("pos:\n  x: 1", self.read("places/kitchen/table/node.yaml"))

    def test_offset_refused_until_aligned_to_the_base(self):
        self.state.aligned_to_base = False
        before = self.read("places/kitchen/fridge/milk/node.yaml")
        for extra in ((), ("--force",)):
            code, out, _ = self.egs(
                "observe", "places/kitchen/fridge/milk", "--offset", "1", "0", "0", *extra
            )
            self.assertEqual(code, 1, out)
            self.assertIn("not aligned to the base map", out)
        self.assertEqual(self.read("places/kitchen/fridge/milk/node.yaml"), before)
        code, out, _ = self.egs(
            "observe", "places/kitchen/fridge/milk", "--offset", "1", "0", "0", "--json"
        )
        self.assertEqual(json.loads(out)["reason"], "not_aligned")
        self.state.has_frozen_base = False
        code, _, _ = self.egs("observe", "places/kitchen/fridge/milk", "--offset", "1", "0", "0")
        self.assertEqual(code, 0)

    def test_offset_far_from_the_place_needs_force(self):
        # Anchor 3 (the fridge) is at (6, -2); the robot is 100 m away.
        self.state.robot = pose(106.0, -2.0, 0.0)
        before = self.read("places/kitchen/fridge/milk/node.yaml")
        code, out, _ = self.egs("observe", "places/kitchen/fridge/milk", "--offset", "1", "0", "0")
        self.assertEqual(code, 1)
        self.assertIn("100.0m from places/kitchen/fridge (anchor 3)", out)
        self.assertIn("10.0m limit", out)
        self.assertIn("egs place save", out)
        self.assertIn("--force", out)
        self.assertEqual(self.read("places/kitchen/fridge/milk/node.yaml"), before)
        self.assertFalse(
            os.path.exists(os.path.join(self.mem, "places/kitchen/fridge/milk/observations.jsonl"))
        )
        code, out, _ = self.egs(
            "observe",
            "places/kitchen/fridge/milk",
            "--offset",
            "1",
            "0",
            "0",
            EGS_OFFSET_MAX_M="200",
        )
        self.assertEqual(code, 0, out)
        code, out, _ = self.egs(
            "observe", "places/kitchen/fridge/milk", "--offset", "1", "0", "0", "--force"
        )
        self.assertEqual(code, 0, out)
        node = loads(self.read("places/kitchen/fridge/milk/node.yaml"))
        self.assertAlmostEqual(node["offset"][0], 101.0, places=3)

    def test_missing_node(self):
        code, _, err = self.egs("observe", "places/kitchen/toaster")
        self.assertEqual(code, 2)
        self.assertIn("no such node", err)


if __name__ == "__main__":
    unittest.main()
