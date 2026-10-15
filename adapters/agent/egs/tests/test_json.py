import argparse
import io
import json
import math
import os
import unittest

from fake_service import pose
from support import EgsTestCase

from egs import cli

NO_JSON = {("fs", "cat"), ("skill", "show"), ("skill", "install"), ("help",), ("mcp",)}

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
        self.assertIsNone(obj["index_warning"])
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
        self.assertEqual(
            by_path["places/office/desk/green_apple"]["matched"],
            {"rank": "word", "text": "green_apple"},
        )
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
        obj = self.js("--json", "fs", "cat", "README.md", code=2)
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
    def test_fs_commands_local_and_remote(self):
        content = os.path.join(self.tmp, "notes.md")
        with open(content, "w", encoding="utf-8") as f:
            f.write("cold\n")
        for remote in (False, True):
            if remote:
                self.remote_only()
            ls = self.js("fs", "ls", "places/kitchen", "--json")
            self.assertEqual(ls["path"], "places/kitchen")
            by_name = {e["name"]: e for e in ls["entries"]}
            self.assertEqual(by_name["fridge"]["type"], "dir")
            place = by_name["place.yaml"]
            self.assertEqual((place["type"], place["size"]), ("file", 10))
            self.assertEqual([e["name"] for e in ls["entries"]], sorted(by_name))
            bad = self.js("fs", "ls", "places/kitchen/place.yaml", "--json", code=1)
            self.assertEqual(bad["reason"], "fs_error")

            tree = self.js("--json", "fs", "tree", "places/kitchen", "-L", "1")
            self.assertEqual((tree["path"], tree["depth"]), ("places/kitchen", 1))
            self.assertIn("fridge", tree["tree"])
            self.assertEqual(tree["tree"], self.egs("fs", "tree", "places/kitchen", "-L", "1")[1])

            d = "places/kitchen/shelf%d" % remote
            self.assertEqual(self.js("fs", "mkdir", d, "--json")["path"], d)
            notes = d + "/notes.md"
            for verb in ("write", "append"):
                obj = self.js("fs", verb, notes, "--file", content, "--json")
                self.assertEqual(obj["path"], notes)
            self.assertEqual(self.read(notes), "cold\ncold\n")
            mv = self.js("fs", "mv", d, d + "x", "--json")
            self.assertEqual((mv["from"], mv["to"]), (d, d + "x"))
            self.assertEqual(self.js("fs", "rm", "-r", d + "x", "--json")["path"], d + "x")
            self.assertFalse(os.path.exists(os.path.join(self.mem, d + "x")))

            refused = self.js("fs", "rm", "places/kitchen", "--json", code=1)
            self.assertEqual(refused["reason"], "not_empty")
            bad = self.js("fs", "rm", "places/kitchen/place.yaml", "--json", code=2)
            self.assertEqual(bad["reason"], "owned_by_process")

    def test_zone_add_and_rm(self):
        self.state.robot = pose(5.0, 0.0, math.pi / 2)
        for remote in (False, True):
            if remote:
                self.remote_only()
            argv = ("zone", "add", "places/kitchen/slope", "--rect", "1", "-1", "3", "1", "--json")
            obj = self.js(*argv)
            self.assertEqual(obj["path"], "places/kitchen/slope")
            self.assertEqual((obj["kind"], obj["frame"]), ("keepout", "places/kitchen"))
            self.assertEqual(obj["anchor"], 2)
            for got, want in zip(obj["polygon"], [[1, 1], [1, 3], [-1, 3], [-1, 1]]):
                self.assertAlmostEqual(got[0], want[0], places=3)
                self.assertAlmostEqual(got[1], want[1], places=3)
            self.assertAlmostEqual(obj["area_m2"], 4.0, places=3)
            self.assertEqual(obj["at_num_solves"], 100)
            rect = ("--rect", "0", "0", "1", "1", "--json")
            again = self.js("zone", "add", "places/kitchen/slope", *rect, code=1)
            self.assertEqual(again["reason"], "exists")
            rm = self.js("zone", "rm", "places/kitchen/slope", "--json")
            self.assertEqual((rm["path"], rm["removed_dir"]), ("places/kitchen/slope", True))
            gone = self.js("zone", "rm", "places/kitchen/slope", "--json", code=1)
            self.assertEqual(gone["reason"], "no_zone")
        self.state.robot = pose(105.0, 0.0, 0.0)
        far = self.js("zone", "add", "places/kitchen/far", *rect, code=1)
        self.assertEqual(far["reason"], "too_far")
        self.assertIn("--force", far["detail"])

    def test_checkpoint(self):
        obj = self.js("checkpoint", "--json")
        self.assertEqual(obj["num_checkpoints_written"], 4)
        self.assertIn("at_num_solves", obj)

    def test_session_freeze_plan_apply_and_rejection(self):
        plan = self.js("session", "freeze", "--json")
        self.assertFalse(plan["applied"])
        self.assertEqual(plan["sessions_affected"], [2])
        self.assertTrue(plan["plan_token"])
        done = self.js("session", "new", "--yes", "--json")
        self.assertEqual((done["applied"], done["outcome"]), (True, "frozen"))
        self.assertEqual((done["frozen_session"], done["fed"]), (2, 3))
        self.state.freeze_removes = True
        self.assertEqual(self.js("session", "freeze", "--yes", "--json")["outcome"], "removed")
        self.state.freeze_removes = False
        self.state.freeze_visible_after = 10**6
        late = self.js("session", "freeze", "--yes", "--json", code=1, EGS_POLL_S="0.3")
        self.assertEqual(late["reason"], "not_visible")
        self.assertTrue(late["applied"])
        self.state.freeze_rejection = "not_anchored"
        rejected = self.js("session", "freeze", "--yes", "--json", code=1)
        self.assertEqual((rejected["reason"], rejected["applied"]), ("not_anchored", False))

    def test_map_new_and_open(self):
        self.state.maps.append("lab")
        same = self.js("map", "open", "map", "--json")
        self.assertEqual((same["map"], same["switched"]), ("map", False))
        opened = self.js("map", "open", "lab", "--json")
        self.assertEqual((opened["map"], opened["switched"]), ("lab", True))
        self.assertEqual(opened["root"]["map"], "lab")
        self.assertEqual(self.js("map", "new", "lab", "--json", code=1)["reason"], "exists")
        self.assertTrue(self.js("--json", "map", "new", "fresh")["switched"])

    def test_every_command_but_the_exempt_takes_json(self):
        def leaves(parser, path=()):
            subs = [a for a in parser._actions if isinstance(a, argparse._SubParsersAction)]
            if not subs:
                yield path, parser
                return
            for name, child in subs[0].choices.items():
                yield from leaves(child, path + (name,))

        seen = set()
        for path, parser in leaves(cli.build_parser()):
            seen.add(path)
            has = any("--json" in a.option_strings for a in parser._actions)
            self.assertEqual(has, path not in NO_JSON, path)
        self.assertLessEqual(NO_JSON, seen)


if __name__ == "__main__":
    unittest.main()
