import json
import os
import unittest

from support import EgsTestCase

from egs import cli


class TreeTest(EgsTestCase):
    def test_default_depth_is_three_and_cut_offs_are_marked(self):
        code, out, _ = self.egs("fs", "tree", "places")
        self.assertEqual(code, 0, out)
        lines = out.splitlines()
        # places/kitchen/table/apple is depth 3; its node.yaml (depth 4) is cut off.
        i = lines.index("│       ├── apple")
        self.assertEqual(lines[i + 1], "│       │   └── …")
        self.assertEqual(lines[-1], "17 directories, 12 files")
        self.assertNotIn("observations.jsonl", out)
        # An empty directory at the limit has nothing cut off.
        self.write("places/hall/empty/.keep", "")
        os.remove(os.path.join(self.mem, "places/hall/empty/.keep"))
        code, out, _ = self.egs("fs", "tree", "places/hall", "-L", "1")
        self.assertEqual(out.splitlines()[:2], ["places/hall", "└── empty"])
        self.assertNotIn("…", out)
        code, out, _ = self.egs("fs", "tree", "places", "-L", "6")
        self.assertNotIn("…", out)
        self.assertIn("observations.jsonl", out)
        counted = self.egs("fs", "tree", "places", "-L", "1")[1].splitlines()[-1]
        self.assertEqual(counted, "6 directories, 1 files")


class CatOutputTest(EgsTestCase):
    def test_raw_bytes_to_a_file(self):
        blob = bytes(range(256)) * 4
        with open(os.path.join(self.mem, "places/dock/photo.jpg"), "wb") as f:
            f.write(blob)
        dest = os.path.join(self.tmp, "photo.jpg")
        for remote in (False, True):
            if remote:
                self.remote_only()
                os.remove(dest)
            code, out, err = self.egs("fs", "cat", "places/dock/photo.jpg", "-o", dest)
            self.assertEqual(code, 0, err)
            self.assertEqual(out, dest + "\n")
            with open(dest, "rb") as f:
                self.assertEqual(f.read(), blob)
        code, out, _ = self.egs("fs", "cat", "places/dock/none.jpg", "-o", dest)
        self.assertEqual(code, 1)

    def test_output_inside_memory_is_refused(self):
        before = self.read("places/dock/place.yaml")
        for target in ("places/dock/place.yaml", "places/dock/copy.jpg"):
            code, out, err = self.egs(
                "fs", "cat", "places/kitchen/place.yaml", "-o", os.path.join(self.mem, target)
            )
            self.assertEqual(code, 2, out + err)
            self.assertIn("inside memory/", err)
        self.assertEqual(self.read("places/dock/place.yaml"), before)
        self.assertFalse(os.path.exists(os.path.join(self.mem, "places/dock/copy.jpg")))


class AttachTest(EgsTestCase):
    def attach_file(self, name, data):
        path = os.path.join(self.tmp, name)
        with open(path, "wb") as f:
            f.write(data)
        return path

    def check_attach(self):
        before = json.loads(self.egs("where", "places/kitchen/fridge/milk", "--json")[1])
        photo = self.attach_file("Shelf.JPG", b"\xff\xd8jpeg")
        code, out, err = self.egs(
            "observe", "places/kitchen/fridge/milk", "--attach", photo, "--note", "top shelf"
        )
        self.assertEqual(code, 0, out + err)
        line = json.loads(self.read("places/kitchen/fridge/milk/observations.jsonl"))
        rel = "attachments/%d.jpg" % line["recorded_at"]
        self.assertEqual(line["attachment"], rel)
        self.assertEqual(line["note"], "top shelf")
        with open(os.path.join(self.mem, "places/kitchen/fridge/milk", rel), "rb") as f:
            self.assertEqual(f.read(), b"\xff\xd8jpeg")
        blob = self.attach_file("scan", b"\0\1")
        code, out, _ = self.egs("observe", "places/kitchen/fridge/milk", "--attach", blob, "--json")
        self.assertEqual(code, 0, out)
        self.assertTrue(json.loads(out)["attachment"].endswith(".bin"))
        after = json.loads(self.egs("where", "places/kitchen/fridge/milk", "--json")[1])
        for key in ("precision", "resolved_via", "place", "pose", "bound_children", "notes"):
            self.assertEqual(after[key], before[key], key)
        code, out, _ = self.egs("where", "places/kitchen/fridge", "--json")
        self.assertEqual(code, 0, out)
        self.assertEqual(json.loads(out)["bound_children"], [])

    def test_local(self):
        self.check_attach()

    def test_through_fs(self):
        self.remote_only()
        self.check_attach()
        self.assertTrue(self.requests("POST", "/fs/mkdir"))
        self.assertTrue(self.requests("POST", "/fs/write"))

    def test_too_big_or_missing_writes_nothing(self):
        big = self.attach_file("big.png", b"\0" * ((8 << 20) + 1))
        code, _, err = self.egs("observe", "places/kitchen/fridge/milk", "--attach", big)
        self.assertEqual(code, 2)
        self.assertIn("8 MiB", err)
        code, _, err = self.egs(
            "observe", "places/kitchen/fridge/milk", "--attach", os.path.join(self.tmp, "none")
        )
        self.assertEqual(code, 2)
        milk = os.path.join(self.mem, "places/kitchen/fridge/milk")
        self.assertFalse(os.path.exists(os.path.join(milk, "observations.jsonl")))
        self.assertFalse(os.path.exists(os.path.join(milk, "attachments")))

    def test_attachments_is_never_a_node(self):
        self.write("places/kitchen/attachments/node.yaml", "kind: attachments\n")
        code, out, _ = self.egs("find", "attachments")
        self.assertIn("no match", out)
        self.assertNotIn("places/kitchen/attachments", out)
        code, out, _ = self.egs("find", "room")
        self.assertIn("places/kitchen", out)
        for argv in (
            ("where", "places/kitchen/attachments"),
            ("observe", "places/kitchen/attachments"),
            ("place", "save", "places/kitchen/attachments"),
        ):
            code, _, err = self.egs(*argv)
            self.assertEqual(code, 2, argv)
            self.assertIn("reserved_name", err)
        self.assertFalse(self.requests("POST", "/place/save"))

    def test_failed_copy_means_no_line(self):
        photo = self.attach_file("a.png", b"png")
        # A file where the attachments directory should go makes the copy fail.
        self.write("places/kitchen/fridge/milk/attachments", "not a directory")
        code, _, _ = self.egs("observe", "places/kitchen/fridge/milk", "--attach", photo)
        self.assertNotEqual(code, 0)
        self.assertFalse(
            os.path.exists(os.path.join(self.mem, "places/kitchen/fridge/milk/observations.jsonl"))
        )


class HelpTest(EgsTestCase):
    def test_help_agent_fits_in_thirty_lines(self):
        code, out, _ = self.egs("help", "agent")
        self.assertEqual(code, 0)
        lines = out.splitlines()
        self.assertLessEqual(len(lines), 30)
        for keyword in (
            "XY",
            "place.yaml",
            "node.yaml",
            "--include=node.yaml",
            "places/",
            "skills",
            "symbolic links",
            "append-only",
            "goto",
            "exit 0",
            "outcome unknown",
        ):
            self.assertIn(keyword, out)
        self.assertEqual(sum(1 for l in lines if l[:2] in {"%d." % i for i in range(1, 9)}), 8)

    def test_help_alone_is_dash_dash_help(self):
        code, out, _ = self.egs("help")
        self.assertEqual(code, 0)
        self.assertEqual(out, cli.build_parser().format_help())


class ViewEgoTest(EgsTestCase):
    def test_here_crops_by_default(self):
        for argv, ego in (
            (("view", "here"), "7.5"),
            (("view", "here", "--ego", "3"), "3.0"),
            (("view", "here", "--ego", "0"), None),
            (("view", "here", "--full"), None),
            (("view", "map"), None),
            (("view", "map", "--ego"), "7.5"),
        ):
            self.state.requests.clear()
            code, out, _ = self.egs(*argv)
            self.assertEqual(code, 0, out)
            self.assertEqual(self.requests("GET", "/view")[0][2].get("ego"), ego, argv)

    def test_here_without_a_pose_falls_back_to_the_whole_map(self):
        self.state.robot = None
        self.state.view_dropped = "robot,scan"
        code, out, _ = self.egs("view", "here")
        self.assertEqual(code, 0, out)
        self.assertIn("dropped=robot,scan", out)
        self.assertEqual([q.get("ego") for _, _, q in self.requests("GET", "/view")], ["7.5", None])
        # An explicit crop is the caller's choice: its refusal stands.
        code, out, _ = self.egs("view", "here", "--ego", "3")
        self.assertEqual(code, 1)
        self.assertIn("no_robot_pose", out)


if __name__ == "__main__":
    unittest.main()
