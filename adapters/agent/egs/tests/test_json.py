import json
import os
import unittest

from fake_service import pose
from support import EgsTestCase

RESOLUTION_KEYS = {
    "path",
    "precision",
    "place",
    "anchor",
    "state",
    "orphan_reason",
    "approach",
    "pose",
    "resolved_via",
    "notes",
    "bound_children",
    "saved_age_s",
}


class JsonTest(EgsTestCase):
    """--json: exactly one object on stdout, same exit code, documented keys."""

    def js(self, *argv, code=0, **env):
        rc, out, err = self.egs(*argv, **env)
        self.assertEqual(rc, code, out + err)
        lines = out.splitlines()
        self.assertEqual(len(lines), 1, out)
        obj = json.loads(lines[0])
        self.assertIsInstance(obj, dict)
        self.assertIn("ok", obj)
        self.assertIn("reason", obj)
        self.assertEqual(obj["ok"], code == 0, obj)
        return obj

    def test_flag_before_or_after_the_command(self):
        for argv in (("--json", "here"), ("here", "--json")):
            obj = self.js(*argv)
            self.assertEqual(obj["at_num_solves"], 100)
        self.assertFalse(self.egs("here")[1].startswith("{"))

    def test_here_keeps_service_keys_and_lists_the_pose(self):
        self.state.robot = pose(1.0, 2.0, 0.5)
        obj = self.js("here", "--json")
        self.assertEqual(obj["robot"], [1.0, 2.0, 0.5])
        self.assertEqual(obj["current"], {"path": "places/dock", "anchor": 1, "dist_m": 0.4})
        self.assertEqual([c["path"] for c in obj["closest"]], ["places/dock", "places/kitchen"])
        self.assertEqual(obj["keyframe_age_s"], 1.5)
        self.assertTrue(obj["aligned_to_base"])

    def test_status(self):
        obj = self.js("status", "--json")
        self.assertEqual(obj["num_place_files"], 5)
        self.assertEqual(obj["index_warning"], "index.tsv missing: run `egs snapshot`")
        self.assertEqual(obj["phase"], "LOCKED")

    def test_root(self):
        obj = self.js("--json", "root")
        self.assertEqual(obj["memory_dir"], self.mem)
        self.assertTrue(obj["readable"])

    def test_where_serializes_the_whole_resolution(self):
        obj = self.js("where", "places/kitchen/table/apple", "--json")
        self.assertLessEqual(RESOLUTION_KEYS, set(obj))
        self.assertEqual(obj["precision"], "inherited")
        self.assertEqual(obj["resolved_via"], "places/kitchen/table")
        self.assertEqual(obj["pose"], [6.0, 0.0, None])
        self.assertEqual(obj["approach"], [5.0, 0.0, 0.0])
        self.assertEqual(obj["place"], "places/kitchen")
        self.assertEqual(obj["anchor"], 2)
        self.assertEqual(obj["state"], "frozen")
        self.assertAlmostEqual(obj["saved_age_s"], 6 * 86400, delta=60)
        self.assertEqual(obj["at_num_solves"], 100)

        own = self.js("where", "places/office/desk/apple", "--json")
        self.assertEqual(own["precision"], "offset")
        self.assertEqual(len(own["pose"]), 3)
        self.assertIsNotNone(own["pose"][2])

        none = self.js("where", "places/office", "--json")
        self.assertEqual(none["precision"], "none")
        self.assertEqual(none["bound_children"], ["places/office/desk"])
        self.assertIsNone(none["pose"])

        orphan = self.js("where", "places/garage/shelf", "--json", code=1)
        self.assertEqual(orphan["reason"], "unresolvable")
        self.assertEqual(orphan["orphan_reason"], "session_removed")
        self.assertLessEqual(RESOLUTION_KEYS, set(orphan))

    def test_find_rows(self):
        obj = self.js("find", "apple", "--json")
        rows = obj["rows"]
        self.assertEqual(rows[0]["path"], "places/office/desk/apple")
        for row in rows:
            self.assertLessEqual(
                {"path", "kind", "rank", "last_seen_age_s", "result", "dist_m", "precision"},
                set(row),
            )
        by_path = {r["path"]: r for r in rows}
        self.assertEqual(by_path["places/kitchen/table/apple"]["result"], "absent")
        self.assertAlmostEqual(
            by_path["places/kitchen/table/apple"]["last_seen_age_s"], 20 * 60, delta=60
        )
        self.assertEqual(by_path["places/office/desk/apple"]["match"], "name")
        self.assertEqual(obj["more"], 0)
        limited = self.js("find", "apple", "--limit", "1", "--json")
        self.assertEqual(len(limited["rows"]), 1)
        self.assertEqual(limited["more"], len(rows) - 1)
        self.assertEqual(self.js("find", "zebra", "--json")["rows"], [])

    def test_place_save_and_its_refusal(self):
        obj = self.js("place", "save", "places/hall", "--json")
        self.assertEqual(obj["anchor"], 50)
        self.assertEqual(obj["path"], "places/hall")
        self.assertEqual(obj["state"], "pending")
        self.state.keyframe = False
        refused = self.js("--json", "place", "save", "places/hall", code=1)
        self.assertEqual(refused["reason"], "no_keyframe")
        self.assertIn("detail", refused)

    def test_observe_with_offset_and_its_refusal(self):
        obj = self.js("observe", "places/kitchen/fridge/milk", "--offset", "0.5", "0", "0", "--json")
        self.assertEqual(obj["path"], "places/kitchen/fridge/milk")
        self.assertEqual(obj["line"]["recorded_at"], obj["id"])
        self.assertEqual(obj["line"]["result"], "seen")
        self.assertEqual(obj["offset"]["offset_from"], 3)
        self.assertEqual(len(obj["offset"]["offset"]), 3)
        self.assertIsNone(obj["attachment"])
        self.state.robot = pose(106.0, -2.0, 0.0)
        refused = self.js(
            "observe", "places/kitchen/fridge/milk", "--offset", "1", "0", "0", "--json", code=1
        )
        self.assertEqual(refused["reason"], "too_far")
        self.assertIn("--force", refused["detail"])

    def test_snapshot(self):
        obj = self.js("snapshot", "--json")
        self.assertEqual(obj["seq"], 1)
        self.assertEqual(obj["files"], ["map.png", "summary.txt"])
        self.assertTrue(os.path.isdir(obj["local_dir"]))
        self.assertIsNone(obj["copied_to"])

    def test_view_metadata(self):
        self.state.view_dropped = "scan"
        obj = self.js("view", "route", "places/kitchen", "--json")
        self.assertEqual(
            obj["layers"], ["map", "robot", "target=places/kitchen", "places", "zones"]
        )
        self.assertEqual(obj["legend"], ["1 places/dock 0.4m", "2 places/kitchen 5.0m"])
        self.assertEqual(obj["dropped"], ["scan"])
        self.assertEqual(obj["at_num_solves"], 100)
        self.assertTrue(os.path.isfile(obj["png"]))
        refused = self.js("view", "route", "places/office", "--json", code=1)
        self.assertEqual(refused["reason"], "no_binding")

    def test_map_and_session_ls(self):
        maps = self.js("map", "ls", "--json")
        self.assertEqual(maps["maps"], [{"name": "map", "current": True}])
        sessions = self.js("--json", "session", "ls")
        self.assertEqual(sessions["fed"], 2)
        self.assertEqual([s["id"] for s in sessions["sessions"]], [0, 1, 2])
        self.assertLess(sessions["sessions"][2]["last_node_age_s"], 60)
        self.assertIn("started_age_s", sessions["sessions"][0])

    def test_errors_are_objects_too(self):
        obj = self.js("--json", "where", "places/nowhere", code=2)
        self.assertEqual(obj["reason"], "no_such_node")
        self.assertIn("no such node", obj["detail"])
        obj = self.js("--json", "observe", "places/nope", code=2)
        self.assertEqual(obj["reason"], "no_such_node")
        obj = self.js("--json", "where", "places/kitchen/skills", code=2)
        self.assertEqual(obj["reason"], "reserved_name")
        obj = self.js("here", "--json", code=3, EGS_URL="http://127.0.0.1:9")
        self.assertEqual(obj["reason"], "unreachable")
        obj = self.js("--json", "fs", "ls", "places", code=2)
        self.assertIn("--json", obj["detail"])
        obj = self.js("where", "--json", code=2)
        self.assertEqual(obj["reason"], "usage")

    def test_refusals_keep_the_machine_reason(self):
        # fs_error from mkdir, local and through /fs: a file sits where attachments/ goes.
        self.write("places/kitchen/fridge/milk/attachments", "not a directory")
        photo = os.path.join(self.tmp, "a.png")
        with open(photo, "wb") as f:
            f.write(b"png")
        argv = ("--json", "observe", "places/kitchen/fridge/milk", "--attach", photo)
        for remote in (False, True):
            if remote:
                self.remote_only()
            obj = self.js(*argv, code=1)
            self.assertEqual(obj["reason"], "fs_error", obj)
        obj = self.js("--json", "view", "here", "--ego", "-1", code=2)
        self.assertEqual(obj["reason"], "bad_param")

if __name__ == "__main__":
    unittest.main()
