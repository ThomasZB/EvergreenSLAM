import unittest

import egs.maps as maps_mod
from support import EgsTestCase


class MapsTest(EgsTestCase):
    def setUp(self):
        super().setUp()
        self.state.maps.append("lab")

    def test_ls_marks_the_open_map_then_prints_the_root(self):
        code, out, _ = self.egs("map", "ls")
        self.assertEqual(code, 0, out)
        self.assertEqual(out.splitlines(), ["  lab", "* map", "root %s" % self.state.map_root])

    def test_status_names_the_map(self):
        code, out, _ = self.egs("status")
        self.assertEqual(code, 0, out)
        self.assertTrue(out.startswith("map map  boot 3"), out)

    def test_new_existing_name_is_refused(self):
        code, out, _ = self.egs("map", "new", "lab")
        self.assertEqual(code, 1)
        self.assertIn("exists", out)
        self.assertIsNone(self.state.switching)

    def test_new_bad_name_is_a_usage_error(self):
        code, _, err = self.egs("map", "new", "Bad")
        self.assertEqual(code, 2)
        self.assertIn("not_slug", err)

    def test_new_polls_through_the_swap(self):
        self.state.switch_down_polls = 2
        self.state.switch_after_polls = 4
        code, out, err = self.egs("map", "new", "fresh")
        self.assertEqual(code, 0, out + err)
        memory = "%s/fresh/memory" % self.state.map_root
        self.assertIn("map fresh open; memory/ is now %s" % memory, out)
        self.assertIn("cd %s again" % memory, out)
        self.assertNotIn("init-pose", out)
        self.assertEqual(self.requests("POST", "/maps/new")[0][2], {"name": "fresh"})
        self.assertEqual(len(self.requests("GET", "/root")), 5)
        code, out, _ = self.egs("map", "ls")
        self.assertIn("* fresh", out)

    def test_new_that_never_lands_is_outcome_unknown(self):
        self.state.switch_after_polls = 10**6
        orig = maps_mod.SWITCH_MIN_WAIT_S, maps_mod.POLL_INTERVAL_S
        maps_mod.SWITCH_MIN_WAIT_S, maps_mod.POLL_INTERVAL_S = 0.0, 0.05
        try:
            code, out, err = self.egs("map", "new", "fresh", EGS_POLL_S="0.3")
        finally:
            maps_mod.SWITCH_MIN_WAIT_S, maps_mod.POLL_INTERVAL_S = orig
        self.assertEqual(code, 3, out + err)
        self.assertIn("outcome unknown", err)
        self.assertIn("egs root", err)

    def test_second_request_while_switching_is_refused(self):
        self.state.switching = "fresh"
        for argv in (("map", "new", "other"), ("map", "open", "map")):
            code, out, _ = self.egs(*argv)
            self.assertEqual(code, 1, argv)
            self.assertIn("switching (a map switch is in progress", out)
        self.assertNotIn("other", self.state.maps)
        self.assertEqual(self.requests("GET", "/root"), [])

    def test_open_current_is_not_pending(self):
        code, out, _ = self.egs("map", "open", "map")
        self.assertEqual(code, 0, out)
        self.assertIn("map map is already open", out)
        self.assertEqual(self.requests("GET", "/root"), [])

    def test_open_switches(self):
        code, out, _ = self.egs("map", "open", "lab")
        self.assertEqual(code, 0, out)
        self.assertIn("map lab open", out)
        self.assertIn("egs init-pose --place <p>` or `egs relocalize`", out)

    def test_open_missing_is_unknown_map(self):
        code, out, _ = self.egs("map", "open", "nowhere")
        self.assertEqual(code, 1)
        self.assertIn("unknown_map (no such map: see `egs map ls`)", out)

    def test_host_without_root_is_not_supported(self):
        self.state.map_root = None
        for argv in (("map", "ls"), ("map", "new", "fresh"), ("map", "open", "lab")):
            code, out, _ = self.egs(*argv)
            self.assertEqual(code, 1, argv)
            self.assertIn("not_supported (this host cannot switch maps", out)
        code, out, _ = self.egs("status")
        self.assertTrue(out.startswith("boot 3"), out)


if __name__ == "__main__":
    unittest.main()
