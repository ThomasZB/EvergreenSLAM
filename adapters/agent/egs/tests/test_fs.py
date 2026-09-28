import io
import json
import os
import sys
import unittest
from unittest import mock

from support import EgsTestCase


class FallbackTest(EgsTestCase):
    """The process's memory dir is not visible here: every tree access must go through /fs."""

    def setUp(self):
        super().setUp()
        self.remote_only()

    def test_root_warns(self):
        code, out, err = self.egs("root")
        self.assertEqual(code, 0)
        self.assertEqual(out.strip(), self.state.root_memory_dir)
        self.assertIn("egs fs", err)

    def test_where_and_find_through_fs(self):
        code, out, _ = self.egs("where", "places/kitchen/table/apple")
        self.assertEqual(code, 0, out)
        self.assertIn("xy 6.00,0.00", out)
        self.assertIn("precision inherited", out)
        self.assertTrue(self.requests("GET", "/fs/ls"))
        self.assertTrue(self.requests("GET", "/fs/cat"))
        code, out, _ = self.egs("find", "apple")
        self.assertEqual(code, 0, out)
        self.assertIn("places/kitchen/table/apple", out.splitlines()[2])

    def test_observe_through_fs(self):
        code, out, _ = self.egs("observe", "places/kitchen/fridge/milk", "--offset", "0", "0", "0")
        self.assertEqual(code, 0, out)
        self.assertIn("offset_from: 3", self.read("places/kitchen/fridge/milk/node.yaml"))
        line = json.loads(self.read("places/kitchen/fridge/milk/observations.jsonl"))
        self.assertEqual(line["result"], "seen")
        self.assertTrue(self.requests("POST", "/fs/write"))
        self.assertTrue(self.requests("POST", "/fs/append"))

    def test_fs_commands(self):
        code, out, _ = self.egs("fs", "ls", "places/kitchen")
        self.assertEqual(code, 0)
        self.assertIn("fridge/", out)
        self.assertIn("place.yaml  10", out)
        with mock.patch.object(sys, "stdin", io.TextIOWrapper(io.BytesIO(b"cold here\n"))):
            code, _, _ = self.egs("fs", "write", "places/kitchen/fridge/notes.md")
        self.assertEqual(code, 0)
        self.assertEqual(self.read("places/kitchen/fridge/notes.md"), "cold here\n")
        code, out, _ = self.egs("fs", "cat", "places/kitchen/fridge/notes.md")
        self.assertEqual(out, "cold here\n")
        code, _, err = self.egs("fs", "rm", "places/kitchen/place.yaml")
        self.assertEqual(code, 2)
        self.assertIn("owned_by_process", err)
        code, _, err = self.egs("fs", "rm", "places/kitchen")
        self.assertEqual(code, 1)
        self.assertIn("not_empty", err)
        code, _, err = self.egs("fs", "mv", "places/kitchen", "places/dock")
        self.assertIn("exists", err)
        code, _, _ = self.egs("fs", "mv", "places/kitchen", "places/galley")
        self.assertEqual(code, 0)
        self.assertTrue(os.path.isfile(os.path.join(self.mem, "places/galley/place.yaml")))
        code, _, err = self.egs("fs", "cat", "../etc/passwd")
        self.assertEqual(code, 2)
        code, _, err = self.egs("fs", "mv", "places/galley", "places/dock/skills")
        self.assertEqual(code, 2, err)
        self.assertIn("reserved_name", err)


class LocalFsTest(EgsTestCase):
    """Same verbs on a readable directory: no /fs traffic, same rules."""

    def test_local_rules(self):
        code, out, _ = self.egs("fs", "ls", "places")
        self.assertEqual(code, 0)
        self.assertIn("kitchen/", out)
        self.assertEqual(self.requests("GET", "/fs/ls"), [])
        code, _, err = self.egs("fs", "rm", "places/dock/place.yaml")
        self.assertEqual(code, 2)
        self.assertIn("owned_by_process", err)
        code, _, err = self.egs("fs", "mkdir", "places/Küche")
        self.assertEqual(code, 2)
        self.assertIn("not_slug", err)
        os.symlink(os.path.join(self.mem, "places/kitchen"), os.path.join(self.mem, "places/k"))
        code, _, err = self.egs("fs", "cat", "places/k/place.yaml")
        self.assertEqual(code, 2)
        self.assertIn("symlink", err)
        code, out, _ = self.egs("fs", "tree", "places/kitchen", "-L", "1")
        self.assertIn("├── fridge", out)
        code, _, _ = self.egs("fs", "mkdir", "places/hall/bench")
        self.assertEqual(code, 0)
        self.assertEqual(self.requests("POST", "/fs/mkdir"), [])

    def test_local_owned_names_fold_case(self):
        with mock.patch.object(sys, "stdin", io.TextIOWrapper(io.BytesIO(b"anchor: 99\n"))):
            code, _, err = self.egs("fs", "write", "places/dock/Place.yaml")
        self.assertEqual(code, 2, err)
        self.assertIn("owned_by_process", err)
        self.assertEqual(self.read("places/dock/place.yaml"), "anchor: 1\n")
        with mock.patch.object(sys, "stdin", io.TextIOWrapper(io.BytesIO(b"x"))):
            code, _, err = self.egs("fs", "append", "Index.TSV")
        self.assertEqual(code, 2, err)
        self.assertIn("owned_by_process", err)

    def test_local_places_never_move_under_skills(self):
        for dst in ("places/dock/skills", "places/dock/skills/kitchen", "places/skills/kitchen"):
            code, _, err = self.egs("fs", "mv", "places/kitchen", dst)
            self.assertEqual(code, 2, (dst, err))
            self.assertIn("reserved_name", err)
        self.assertTrue(os.path.isfile(os.path.join(self.mem, "places/kitchen/place.yaml")))
        code, _, err = self.egs("fs", "mv", "places/office", "places/dock/skills/office")
        self.assertEqual(code, 2, err)
        self.assertIn("reserved_name", err)
        self.assertEqual(self.egs("fs", "mkdir", "places/dock/skills/park")[0], 0)
        code, _, err = self.egs(
            "fs", "mv", "places/kitchen/skills/clean", "places/dock/skills/clean"
        )
        self.assertEqual(code, 0, err)
        self.assertTrue(os.path.isfile(os.path.join(self.mem, "places/dock/skills/clean/SKILL.md")))


if __name__ == "__main__":
    unittest.main()
