import json
import os
import time
import unittest

from support import NS_DAY, NS_MIN, EgsTestCase


class WhereTest(EgsTestCase):
    def test_own(self):
        code, out, _ = self.egs("where", "places/kitchen")
        self.assertEqual(code, 0)
        self.assertIn("xy 5.00,0.00", out)
        self.assertIn("precision own", out)
        self.assertIn("approach places/kitchen 5.00,0.00,0.00 (frozen)", out)
        self.assertIn("saved 6d", out)
        self.assertIn("@solve 100", out)

    def test_offset_on_the_path_itself(self):
        code, out, _ = self.egs("where", "places/kitchen/table")
        self.assertEqual(code, 0)
        self.assertIn("xy 6.00,0.00", out)
        self.assertIn("precision offset", out)
        self.assertIn("resolved_via places/kitchen/table", out)

    def test_offset_rotates_with_the_anchor(self):
        code, out, _ = self.egs("where", "places/office/desk/apple")
        self.assertEqual(code, 0)
        self.assertIn("xy -3.00,4.50", out)
        self.assertIn("precision offset", out)
        self.assertIn("approach places/office/desk -3.00,4.00,1.57", out)

    def test_inherited_from_an_offset_between(self):
        code, out, _ = self.egs("where", "places/kitchen/table/apple")
        self.assertEqual(code, 0)
        self.assertIn("xy 6.00,0.00", out)
        self.assertIn("theta -", out)
        self.assertIn("precision inherited", out)
        self.assertIn("resolved_via places/kitchen/table", out)
        self.assertIn("approach places/kitchen ", out)

    def test_inherited_from_the_place(self):
        code, out, _ = self.egs("where", "places/kitchen/fridge/milk")
        self.assertEqual(code, 0)
        self.assertIn("xy 6.00,-2.00", out)
        self.assertIn("precision inherited  resolved_via places/kitchen/fridge", out)
        self.assertIn("(pending)", out)

    def test_none_lists_bound_children(self):
        code, out, _ = self.egs("where", "places/office")
        self.assertEqual(code, 0)
        self.assertIn("precision none", out)
        self.assertIn("bound children: places/office/desk", out)
        self.assertNotIn("xy", out)

    def test_offset_from_mismatch_after_mv_falls_back_to_inherited(self):
        os.rename(
            os.path.join(self.mem, "places/kitchen/table"),
            os.path.join(self.mem, "places/office/desk/table"),
        )
        code, out, _ = self.egs("where", "places/office/desk/table")
        self.assertEqual(code, 0)
        self.assertIn("precision inherited", out)
        self.assertIn("resolved_via places/office/desk", out)
        self.assertIn("xy -3.00,4.00", out)
        self.assertIn("offset_from 2", out)
        code, out, _ = self.egs("where", "places/office/desk/table/apple")
        self.assertIn("precision inherited  resolved_via places/office/desk\n", out)

    def test_orphan_resolves_to_nothing(self):
        code, out, _ = self.egs("where", "places/garage/shelf")
        self.assertEqual(code, 1)
        self.assertIn("orphan (session_removed)", out)
        self.assertIn("egs place save places/garage/shelf", out)
        self.assertNotIn("xy", out)

    def test_unknown_anchor(self):
        self.write("places/dock/place.yaml", "anchor: 99\n")
        code, out, _ = self.egs("where", "places/dock")
        self.assertEqual(code, 1)
        self.assertIn("anchor 99", out)

    def test_path_rules(self):
        code, _, err = self.egs("where", "kitchen")
        self.assertEqual(code, 2)
        self.assertIn("start with places/", err)
        code, _, err = self.egs("where", "places/nowhere")
        self.assertEqual(code, 2)
        self.assertIn("no such node", err)
        code, _, err = self.egs("where", "places/kitchen/skills/clean")
        self.assertEqual(code, 2)
        self.assertIn("reserved_name", err)
        code, out, _ = self.egs("where", os.path.join(self.mem, "places/kitchen") + "/")
        self.assertEqual(code, 0)
        self.assertIn("places/kitchen  xy 5.00,0.00", out)

    def test_read_first_lists_skills_and_notes_from_places_down(self):
        self.write("places/kitchen/notes.md", "mind the step\n")
        self.write("places/kitchen/table/SKILL.md", "---\nname: table\n---\n")
        self.write("places/kitchen/table/apple/notes.md", "bruised\n")
        self.write("places/kitchen/fridge/notes.md", "not on the way\n")
        want = [
            "places/SKILL.md",
            "places/kitchen/notes.md",
            "places/kitchen/table/SKILL.md",
            "places/kitchen/table/apple/notes.md",
        ]
        for remote in (False, True):
            if remote:
                self.remote_only()
            self.state.requests.clear()
            code, out, _ = self.egs("where", "places/kitchen/table/apple")
            self.assertEqual(code, 0, out)
            self.assertEqual(out.splitlines()[-1], "read first: " + ", ".join(want))
            listed = [q["path"] for _, _, q in self.requests("GET", "/fs/ls")]
            self.assertEqual(len(listed), len(set(listed)), listed)
            obj = json.loads(self.egs("where", "places/kitchen/table/apple", "--json")[1])
            self.assertEqual(obj["read_first"], want)
        code, out, _ = self.egs("where", "places/office")
        self.assertEqual(out.splitlines()[-1], "read first: places/SKILL.md")
        code, out, _ = self.egs("where", "places/garage/shelf")
        self.assertEqual(code, 1)
        self.assertEqual(out.splitlines()[-1], "read first: places/SKILL.md")
        os.remove(os.path.join(self.mem, "places/SKILL.md"))
        code, out, _ = self.egs("where", "places/dock")
        self.assertNotIn("read first", out)
        obj = json.loads(self.egs("where", "places/dock", "--json")[1])
        self.assertEqual(obj["read_first"], [])

    def test_saved_age_is_on_the_robot_clock(self):
        robot_now = self.state.sensor_base_ns
        self.state.anchors[1]["saved_at_ns"] = robot_now - 2 * NS_MIN
        code, out, _ = self.egs("where", "places/dock")
        self.assertIn("saved 2m", out)
        obj = json.loads(self.egs("where", "places/dock", "--json")[1])
        self.assertAlmostEqual(obj["saved_age_s"], 120, delta=5)
        self.assertEqual(len(self.requests("GET", "/pose")), 2)
        self.state.pose_sample = False
        self.state.anchors[1]["saved_at_ns"] = time.time_ns() - 3 * NS_DAY
        code, out, _ = self.egs("where", "places/dock")
        self.assertIn("saved 3d", out)
        self.state.requests.clear()
        self.egs("where", "places/office")
        self.assertEqual(self.requests("GET", "/pose"), [])

    def test_memory_from_env_skips_root(self):
        code, out, _ = self.egs("where", "places/dock", EGS_MEMORY=self.mem)
        self.assertEqual(code, 0)
        self.assertEqual(self.requests("GET", "/root"), [])
        self.assertEqual(self.requests("GET", "/fs/ls"), [])


if __name__ == "__main__":
    unittest.main()
