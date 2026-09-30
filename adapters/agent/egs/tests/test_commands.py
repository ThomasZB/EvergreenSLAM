import os
import unittest

from support import EgsTestCase

from egs import fmt


class FormatTest(unittest.TestCase):
    def test_ages(self):
        self.assertEqual(fmt.age_s(35), "35s")
        self.assertEqual(fmt.age_s(20 * 60 + 5), "20m")
        self.assertEqual(fmt.age_s(3 * 3600), "3h")
        self.assertEqual(fmt.age_s(6 * 86400 + 10), "6d")
        self.assertEqual(fmt.age_ns(1000 * 10**9, now=1035 * 10**9), "35s")
        self.assertEqual(fmt.age_ns(None), "-")

    def test_clock(self):
        robot = (0.0, 0.0, 0.0)
        self.assertEqual(fmt.clock(robot, (2.0, 0.0, 0.0)), "12o'clock")
        self.assertEqual(fmt.clock(robot, (0.0, -2.0, 0.0)), "3o'clock")
        self.assertEqual(fmt.clock(robot, (0.0, 2.0, 0.0)), "9o'clock")
        self.assertEqual(fmt.clock(robot, (-2.0, 0.0, 0.0)), "6o'clock")


class CommandTest(EgsTestCase):
    def test_here(self):
        code, out, _ = self.egs("here")
        self.assertEqual(code, 0)
        self.assertEqual(
            out.splitlines(),
            [
                "current places/dock 0.4m",
                "closest places/dock 0.4m frozen; places/kitchen 5.0m frozen",
                "robot 0.00,0.00,0.00  keyframe 1s  @solve 100",
                "match 0.82 (avg 0.80)",
            ],
        )
        self.assertEqual(self.requests("GET", "/root"), [])

    def test_here_and_status_without_a_host_frame(self):
        self.state.scan_match = None
        code, out, _ = self.egs("here")
        self.assertEqual(code, 0)
        self.assertEqual(out.splitlines()[-1], "match -")
        code, out, _ = self.egs("status")
        self.assertIn("match -", out.splitlines())

    def test_here_flags_unaligned_positions(self):
        self.state.aligned_to_base = False
        code, out, _ = self.egs("here")
        self.assertEqual(code, 0)
        lines = out.splitlines()
        self.assertEqual(
            lines[0], "aligned to base: no (positions below are unreliable until aligned)"
        )
        self.assertEqual(lines[1], "current places/dock 0.4m")
        self.assertTrue(lines[2].startswith("closest places/dock"))
        # Without a frozen base there is nothing to be aligned to.
        self.state.has_frozen_base = False
        code, out, _ = self.egs("here")
        self.assertTrue(out.startswith("current "), out)

    def test_status_flags_stale_index(self):
        self.write("index.tsv", "#path anchor state x y theta num_solves\nplaces/dock\t1\n")
        code, out, _ = self.egs("status")
        self.assertEqual(code, 0)
        self.assertIn("frozen base yes  aligned to base yes", out)
        self.assertIn("closures +2", out)
        self.assertIn("match 0.82 (avg 0.80)", out.splitlines())
        self.assertIn("index.tsv stale: 1 rows, 5 place.yaml files", out)

    def test_place_save_new_and_rebind(self):
        code, out, _ = self.egs("place", "save", "places/hall")
        self.assertEqual(code, 0, out)
        self.assertIn("saved places/hall anchor 50 (new)", out)
        self.assertEqual(self.read("places/hall/place.yaml"), "anchor: 50\n")
        self.assertEqual(
            self.requests("POST", "/place/save")[0][2], {"path": "places/hall", "scan": "true"}
        )
        code, out, _ = self.egs("place", "save", "places/hall", "--no-scan")
        self.assertIn("anchor 50 (re-bound existing)", out)
        self.state.keyframe = False
        code, out, _ = self.egs("place", "save", "places/hall")
        self.assertEqual(code, 1)
        self.assertIn("refused: no_keyframe", out)

    def test_place_save_with_offset(self):
        code, out, _ = self.egs("place", "save", "places/shelf", "--offset", "1", "0", "0")
        self.assertEqual(code, 0, out)
        self.assertTrue(out.rstrip().endswith("offset 1.00,0.00,0.00"), out)
        self.assertEqual(self.requests("POST", "/place/save")[0][2]["offset"], "1.0,0.0,0.0")
        code, out, _ = self.egs("place", "save", "places/far", "--offset", "3", "1", "0")
        self.assertEqual(code, 1)
        self.assertIn("refused: offset_too_far", out)
        self.state.offset_free = False
        code, out, _ = self.egs("place", "save", "places/wall", "--offset", "1", "0", "0")
        self.assertEqual(code, 1)
        self.assertIn("refused: offset_not_free", out)
        self.assertFalse(os.path.exists(os.path.join(self.mem, "places/wall")))
        code, _, _ = self.egs("place", "save", "places/x", "--offset", "1", "0")
        self.assertEqual(code, 2)

    def test_view_preset_prints_layers_legend_path(self):
        code, out, _ = self.egs("view", "route", "places/kitchen", "--ego")
        self.assertEqual(code, 0, out)
        lines = out.splitlines()
        self.assertEqual(lines[0], "layers=map,robot,target=places/kitchen,places,zones  @solve 100")
        self.assertEqual(lines[1:3], ["1 places/dock 0.4m", "2 places/kitchen 5.0m"])
        self.assertTrue(lines[3].endswith("views/000007_route.png"))
        self.assertTrue(os.path.isfile(lines[3]))
        q = self.requests("GET", "/view")[0][2]
        self.assertEqual(q, {"preset": "route", "target": "places/kitchen", "ego": "7.5"})

    def test_view_prints_layers_once_and_dropped(self):
        self.state.view_dropped = "robot,scan"
        code, out, _ = self.egs("view", "here")
        self.assertEqual(code, 0, out)
        lines = out.splitlines()
        self.assertEqual([l for l in lines if l.startswith("layers=")], [lines[0]])
        self.assertEqual(lines[1], "dropped=robot,scan")

    def test_view_and_snapshot_use_egs_memory_map_dir(self):
        # The process runs in a container: /root paths mean nothing on this machine.
        self.state.root_map_dir = "/root/runs/map"
        self.state.root_memory_dir = "/root/runs/map/memory"
        local_map = os.path.join(self.tmp, "map")
        code, out, _ = self.egs("view", "here", EGS_MEMORY=self.mem + "/")
        self.assertEqual(code, 0, out)
        self.assertEqual(out.splitlines()[-1], os.path.join(local_map, "views", "000007_here.png"))
        code, out, _ = self.egs("snapshot", EGS_MEMORY=self.mem)
        self.assertEqual(code, 0, out)
        snap = os.path.join(local_map, "snapshots", "000001")
        self.assertEqual(out.splitlines()[0], "snapshot 1  " + snap)
        dest = os.path.join(self.tmp, "out")
        code, out, _ = self.egs("snapshot", "-o", dest, EGS_MEMORY=self.mem)
        self.assertEqual(code, 0, out)
        self.assertTrue(os.path.isfile(os.path.join(dest, "summary.txt")))
        # The view is not in the local map_dir: fall back to a temp copy of the bytes.
        self.state.map_dir = os.path.join(self.tmp, "elsewhere")
        code, out, _ = self.egs("view", "map", EGS_MEMORY=self.mem)
        self.assertEqual(code, 0, out)
        png = out.splitlines()[-1]
        self.assertFalse(png.startswith(local_map), png)
        self.assertIn("egs-views", png)
        self.assertTrue(os.path.isfile(png))

    def test_view_refusals(self):
        code, _, err = self.egs("view", "custom", "--layers", "map,robot,scan,trail,places")
        self.assertEqual(code, 2)
        self.assertIn("too_many_layers", err)
        self.assertEqual(self.requests("GET", "/view"), [])
        code, out, _ = self.egs("view", "route", "places/office")
        self.assertEqual(code, 1)
        self.assertIn("no_binding", out)
        code, _, err = self.egs("view", "route")
        self.assertEqual(code, 2)

    def test_view_output_file(self):
        target = os.path.join(self.tmp, "here.png")
        code, out, _ = self.egs("view", "here", "-o", target)
        self.assertEqual(out.splitlines()[-1], target)
        with open(target, "rb") as f:
            self.assertTrue(f.read().startswith(b"\x89PNG"))

    def test_init_pose(self):
        code, out, _ = self.egs("init-pose", "1.5", "-2", "0.3")
        self.assertEqual(code, 0)
        self.assertIn("initial pose set to 1.50,-2.00,0.30", out)
        code, out, _ = self.egs("init-pose", "--place", "places/dock")
        self.assertEqual(code, 0)
        self.assertEqual(self.requests("POST", "/init-pose")[1][2], {"anchor": "1"})
        code, out, _ = self.egs("init-pose", "--place", "places/kitchen/table")
        self.assertEqual(code, 1)
        self.assertIn("does not walk up", out)
        code, out, _ = self.egs("init-pose", "--place", "places/garage/shelf")
        self.assertIn("unresolvable", out)

    def test_snapshot_copy(self):
        dest = os.path.join(self.tmp, "out")
        code, out, _ = self.egs("snapshot", "-o", dest)
        self.assertEqual(code, 0, out)
        self.assertTrue(os.path.isfile(os.path.join(dest, "summary.txt")))

    def test_exports_never_land_in_memory(self):
        cwd = os.getcwd()
        os.chdir(self.mem)
        try:
            code, _, err = self.egs("snapshot", "-o", "export")
            self.assertEqual(code, 2, err)
            self.assertIn("inside memory/", err)
            code, _, err = self.egs("view", "here", "-o", "places/here.png")
            self.assertEqual(code, 2, err)
        finally:
            os.chdir(cwd)
        self.assertFalse(os.path.exists(os.path.join(self.mem, "export")))
        self.assertFalse(os.path.exists(os.path.join(self.mem, "places/here.png")))
        self.assertEqual(self.requests("POST", "/snapshot"), [])
        self.assertEqual(self.egs("snapshot", "-o", self.mem)[0], 2)

    def test_relocalize_checkpoint(self):
        self.assertEqual(self.egs("relocalize")[0], 0)
        code, out, _ = self.egs("checkpoint")
        self.assertIn("checkpoint written (4 this run)", out)

    def test_timeout_on_mutating_command_is_outcome_unknown(self):
        self.state.delays["/checkpoint"] = 1.0
        code, _, err = self.egs("checkpoint", EGS_TIMEOUT="0.2")
        self.assertEqual(code, 3)
        self.assertIn("outcome unknown", err)
        self.state.delays["/here"] = 1.0
        code, _, err = self.egs("here", EGS_TIMEOUT="0.2")
        self.assertEqual(code, 3)
        self.assertNotIn("outcome unknown", err)

    def test_unreachable(self):
        code, _, err = self.egs("here", EGS_URL="http://127.0.0.1:9")
        self.assertEqual(code, 3)
        self.assertIn("cannot reach", err)


if __name__ == "__main__":
    unittest.main()
