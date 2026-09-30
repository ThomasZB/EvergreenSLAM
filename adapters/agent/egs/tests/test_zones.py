import json
import math
import os
import unittest

from fake_service import pose
from support import EgsTestCase

from egs import zones
from egs.yamlmini import loads


class ZoneTest(EgsTestCase):
    def zone(self, rel):
        return loads(self.read(rel + "/zone.yaml"), nested_keys=("polygon",))

    def test_add_converts_the_robot_frame_to_the_anchor_frame(self):
        # Kitchen anchor at (5, 0, 0); the robot stands on it facing +y.
        self.state.robot = pose(5.0, 0.0, math.pi / 2)
        code, out, err = self.egs(
            "zone", "add", "places/kitchen/slope", "--rect", "1", "-1", "3", "1"
        )
        self.assertEqual(code, 0, out + err)
        self.assertEqual(self.requests("GET", "/anchors/2")[0][2], {"robot": "1"})
        z = self.zone("places/kitchen/slope")
        self.assertEqual(z["kind"], "keepout")
        self.assertEqual(z["frame"], "places/kitchen")
        expected = [[1.0, 1.0], [1.0, 3.0], [-1.0, 3.0], [-1.0, 1.0]]
        for got, want in zip(z["polygon"], expected):
            self.assertAlmostEqual(got[0], want[0], places=3)
            self.assertAlmostEqual(got[1], want[1], places=3)
        self.assertIn("4.00m2", out)
        self.assertIn("places/kitchen (anchor 2)", out)

    def test_add_in_the_anchor_frame_needs_no_robot(self):
        self.state.robot = None
        code, out, _ = self.egs(
            "zone",
            "add",
            "places/kitchen/table/spill",
            "--polygon",
            "0,0 1,0 0,1",
            "--frame",
            "anchor",
        )
        self.assertEqual(code, 0, out)
        self.assertEqual(self.requests("GET", "/anchors/2")[0][2], {})
        self.assertEqual(
            self.zone("places/kitchen/table/spill")["polygon"], [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]
        )

    def test_add_refusals(self):
        self.assertEqual(
            self.egs("zone", "add", "places/kitchen/a", "--rect", "0", "0", "1", "1")[0], 0
        )
        code, out, _ = self.egs("zone", "add", "places/kitchen/a", "--rect", "0", "0", "2", "2")
        self.assertEqual(code, 1)
        self.assertIn("egs zone rm places/kitchen/a", out)
        code, out, _ = self.egs("zone", "add", "places/kitchen", "--rect", "0", "0", "1", "1")
        self.assertEqual(code, 1)
        self.assertIn("holds a place.yaml", out)
        code, out, _ = self.egs("zone", "add", "places/office/rug", "--rect", "0", "0", "1", "1")
        self.assertEqual(code, 1)
        self.assertIn("egs place save", out)
        self.assertFalse(os.path.exists(os.path.join(self.mem, "places/office/rug")))
        code, out, _ = self.egs(
            "zone", "add", "places/garage/shelf/oil", "--rect", "0", "0", "1", "1"
        )
        self.assertEqual(code, 1)
        self.assertIn("unresolvable", out)
        for polygon in (
            "0,0 1,1 1,0 0,1",
            "0,0 1,0 2,0",
            "0,0 1,0",
            "0,0 1,0 1,1 1,0",
            "0,0 1 1,1",
        ):
            code, _, err = self.egs("zone", "add", "places/kitchen/b", "--polygon", polygon)
            self.assertEqual(code, 2, polygon + ": " + err)
        self.assertEqual(
            self.egs("zone", "add", "places/kitchen/b", "--rect", "0", "0", "0.05", "0.05")[0], 2
        )
        self.assertEqual(
            self.egs("zone", "add", "places/kitchen/B", "--rect", "0", "0", "1", "1")[0], 2
        )
        self.assertFalse(os.path.exists(os.path.join(self.mem, "places/kitchen/b")))

    def test_far_from_the_place_needs_force(self):
        self.state.robot = pose(105.0, 0.0, 0.0)
        code, out, _ = self.egs(
            "zone", "add", "places/kitchen/slope", "--rect", "1", "-1", "3", "1"
        )
        self.assertEqual(code, 1)
        self.assertIn("100.0m from places/kitchen (anchor 2)", out)
        self.assertIn("--force", out)
        self.assertFalse(os.path.exists(os.path.join(self.mem, "places/kitchen/slope")))
        code, out, _ = self.egs(
            "zone", "add", "places/kitchen/slope", "--rect", "1", "-1", "3", "1", "--force"
        )
        self.assertEqual(code, 0, out)
        self.assertAlmostEqual(self.zone("places/kitchen/slope")["polygon"][0][0], 101.0, places=3)

    def test_robot_frame_refused_until_aligned_to_the_base(self):
        self.state.robot = pose(5.0, 0.0, 0.0)
        self.state.aligned_to_base = False
        for extra in ((), ("--force",)):
            code, out, _ = self.egs(
                "zone", "add", "places/kitchen/slope", "--rect", "1", "-1", "3", "1", *extra
            )
            self.assertEqual(code, 1, out)
            self.assertIn("not aligned to the base map", out)
            self.assertFalse(os.path.exists(os.path.join(self.mem, "places/kitchen/slope")))
        code, out, _ = self.egs(
            "zone", "add", "places/kitchen/slope", "--rect", "1", "-1", "3", "1", "--frame", "anchor"
        )
        self.assertEqual(code, 0, out)

    def test_shape_is_checked_as_written_and_bounded(self):
        # 0.012 m2 as typed, but millimetre rounding collapses both x to 0.
        code, _, err = self.egs(
            "zone", "add", "places/kitchen/thin", "--frame", "anchor", "--rect", "0", "0", "0.0004",
            "30",
        )
        self.assertEqual(code, 2, err)
        self.assertRegex(err, "degenerate|repeats a point")
        self.assertFalse(os.path.exists(os.path.join(self.mem, "places/kitchen/thin")))
        code, _, err = self.egs(
            "zone", "add", "places/kitchen/huge", "--frame", "anchor", "--rect", "0", "0", "1000",
            "1",
        )
        self.assertEqual(code, 2, err)
        self.assertIn("over 100m", err)
        self.write(
            "places/kitchen/typo/zone.yaml",
            "kind: keepout\nframe: places/kitchen\npolygon: [[0, 0], [1000, 0], [0, 1]]\n",
        )
        rows = {z["path"]: z for z in json.loads(self.egs("zone", "ls", "--json")[1])["zones"]}
        self.assertEqual(rows["places/kitchen/typo"]["reason"], "too_large")

    def test_ls_text_and_json(self):
        self.state.robot = pose(5.0, 0.0, 0.0)
        self.egs("zone", "add", "places/kitchen/slope", "--rect", "1", "-1", "3", "1")
        self.write(
            "places/garage/shelf/oil/zone.yaml",
            "kind: keepout\nframe: places/garage/shelf\n" "polygon: [[0, 0], [1, 0], [0, 1]]\n",
        )
        self.write("places/nowhere/zone.yaml", "kind: keepout\npolygon: [[0, 0], [1, 0], [0, 1]]\n")
        code, out, _ = self.egs("zone", "ls")
        self.assertEqual(code, 0, out)
        lines = out.splitlines()
        self.assertIn(
            "places/kitchen/slope keepout  places/kitchen (anchor 2) frozen  area 4.00m2", lines
        )
        self.assertTrue(
            any(
                l.startswith("places/garage/shelf/oil") and "unresolvable: orphan" in l
                for l in lines
            ),
            out,
        )
        self.assertTrue(any("no_place" in l for l in lines), out)
        self.assertEqual(lines[-1], "@solve 100")
        self.assertNotIn("6.00,-1.00", out)

        code, out, _ = self.egs("zone", "ls", "--json")
        self.assertEqual(code, 0)
        j = json.loads(out)
        self.assertTrue(j["ok"])
        self.assertEqual(j["at_num_solves"], 100)
        rows = {z["path"]: z for z in j["zones"]}
        self.assertNotIn("polygon_xy", rows["places/kitchen/slope"])
        self.assertAlmostEqual(rows["places/kitchen/slope"]["area_m2"], 4.0)
        self.assertEqual(rows["places/garage/shelf/oil"]["reason"], "orphan")
        j = json.loads(self.egs("zone", "ls", "--json", "--xy")[1])
        xy = {z["path"]: z for z in j["zones"]}["places/kitchen/slope"]["polygon_xy"]
        self.assertAlmostEqual(xy[0][0], 6.0)
        self.assertAlmostEqual(xy[0][1], -1.0)
        self.assertIn("xy 6.00,-1.00", self.egs("zone", "ls", "--xy")[1])

    def test_a_moved_zone_reports_frame_mismatch(self):
        self.state.robot = pose(5.0, 0.0, 0.0)
        self.egs("zone", "add", "places/kitchen/slope", "--rect", "1", "-1", "3", "1")
        os.rename(
            os.path.join(self.mem, "places/kitchen/slope"),
            os.path.join(self.mem, "places/office/desk/slope"),
        )
        code, out, _ = self.egs("zone", "ls")
        self.assertEqual(code, 0)
        self.assertIn("frame_mismatch", out)
        self.assertIn("egs zone rm", out)
        self.assertIn("zone.yaml says frame places/kitchen", out)

    def test_rm(self):
        self.egs("zone", "add", "places/kitchen/slope", "--rect", "1", "-1", "3", "1")
        code, out, _ = self.egs("zone", "rm", "places/kitchen/slope")
        self.assertEqual(code, 0, out)
        self.assertIn("and the empty directory", out)
        self.assertFalse(os.path.exists(os.path.join(self.mem, "places/kitchen/slope")))
        self.egs("zone", "add", "places/kitchen/wet", "--rect", "1", "-1", "3", "1")
        self.write("places/kitchen/wet/notes.md", "mopped daily\n")
        self.assertEqual(self.egs("zone", "rm", "places/kitchen/wet")[0], 0)
        self.assertTrue(os.path.exists(os.path.join(self.mem, "places/kitchen/wet/notes.md")))
        self.assertEqual(self.egs("zone", "rm", "places/kitchen/wet")[0], 1)

    def test_remote_memory(self):
        self.remote_only()
        code, out, err = self.egs(
            "zone", "add", "places/kitchen/slope", "--rect", "1", "-1", "3", "1"
        )
        self.assertEqual(code, 0, out + err)
        self.assertTrue(self.requests("POST", "/fs/write"))
        self.assertEqual(self.zone("places/kitchen/slope")["frame"], "places/kitchen")
        self.assertEqual(self.egs("zone", "rm", "places/kitchen/slope")[0], 0)
        self.assertFalse(os.path.exists(os.path.join(self.mem, "places/kitchen/slope")))


class ShapeTest(unittest.TestCase):
    def test_area_and_self_intersection(self):
        self.assertAlmostEqual(zones.area([(0, 0), (2, 0), (2, 1), (0, 1)]), 2.0)
        self.assertTrue(zones.self_intersects([(0, 0), (1, 1), (1, 0), (0, 1)]))
        self.assertFalse(zones.self_intersects([(0, 0), (2, 0), (2, 2), (1, 1), (0, 2)]))
        self.assertTrue(zones.self_intersects([(0, 0), (2, 0), (1, 0), (1, 1)]))


if __name__ == "__main__":
    unittest.main()
