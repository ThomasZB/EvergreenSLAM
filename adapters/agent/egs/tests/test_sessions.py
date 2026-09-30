import json
import unittest

from fake_service import pose
from support import EgsTestCase


class SessionTest(EgsTestCase):
    def test_ls(self):
        code, out, _ = self.egs("session", "ls")
        self.assertEqual(code, 0)
        self.assertIn("fed 2", out)
        self.assertIn("1 floating  nodes 40", out)
        self.assertIn("2 fed", out)
        self.assertIn("last node 0s", out)

    def test_freeze_without_yes_prints_plan_and_exits_0(self):
        code, out, _ = self.egs("session", "freeze")
        self.assertEqual(code, 0)
        self.assertIn("plan: freeze fed session 2", out)
        self.assertIn("sessions_affected: 2", out)
        self.assertEqual(out.splitlines()[-1], "plan only: add --yes to apply")
        self.assertEqual(len(self.requests("POST", "/sessions/freeze/plan")), 1)
        self.assertEqual(self.requests("POST", "/sessions/freeze/apply"), [])
        self.assertEqual(self.requests("POST", "/sessions/freeze/plan")[0][2], {"force": "false"})

    def test_freeze_applies_and_polls(self):
        self.state.freeze_visible_after = 2
        code, out, _ = self.egs("session", "freeze", "--yes")
        self.assertEqual(code, 0, out)
        self.assertIn("session 2 frozen; fed session now 3", out)
        apply = self.requests("POST", "/sessions/freeze/apply")
        self.assertEqual(len(apply), 1)
        self.assertTrue(apply[0][2]["plan_token"])
        self.assertGreaterEqual(len(self.requests("GET", "/sessions")), 2)

    def test_freeze_not_visible_in_time(self):
        self.state.freeze_visible_after = 10**6
        code, out, _ = self.egs("session", "new", "--yes", EGS_POLL_S="0.3")
        self.assertEqual(code, 1)
        self.assertIn("not visible", out)

    def test_freeze_rejected_plan_exits_1(self):
        self.state.freeze_rejection = "covariance too large"
        code, out, _ = self.egs("session", "freeze")
        self.assertEqual(code, 1)
        self.assertIn("rejected: covariance too large", out)
        self.assertNotIn("plan only", out)

    def test_freeze_rejection(self):
        self.state.freeze_rejection = "not anchored"
        code, out, _ = self.egs("session", "freeze", "--yes")
        self.assertEqual(code, 1)
        self.assertIn("rejected: not anchored", out)
        self.assertEqual(self.requests("POST", "/sessions/freeze/apply"), [])

    def test_freeze_plan_changed(self):
        self.state.mutate_before_apply = True
        code, out, _ = self.egs("session", "freeze", "--yes")
        self.assertEqual(code, 1)
        self.assertIn("not applied: plan_changed", out)
        self.assertEqual(self.requests("GET", "/sessions"), [])

    def test_rm_plan_lists_orphaned_places(self):
        self.state.anchor(7, pose(1.0, 1.0, 0.0), state="pending")
        self.state.anchors[7]["submap_id"] = [1, 0]
        self.write("places/hall/place.yaml", "anchor: 7\n")
        code, out, _ = self.egs("session", "rm", "1")
        self.assertEqual(code, 0)
        self.assertIn("plan only: add --yes to apply", out)
        self.assertIn("would_delete: 3 submaps [1,0] [1,1] [1,2]", out)
        self.assertIn("anchors_orphaned: 7 places/hall", out)
        self.assertEqual(self.requests("POST", "/sessions/rm/apply"), [])
        code, out, _ = self.egs("session", "rm", "1", "--yes")
        self.assertEqual(code, 0, out)
        self.assertIn("removed session 1; anchors orphaned: 7", out)
        code, out, _ = self.egs("where", "places/hall")
        self.assertEqual(code, 1)
        self.assertIn("orphan", out)

    def test_rm_refusals(self):
        code, out, _ = self.egs("session", "rm", "2", "--yes")
        self.assertEqual(code, 1)
        self.assertIn("rejected: not_supported", out)
        code, out, _ = self.egs("session", "rm", "0", "--yes")
        self.assertIn("rejected: frozen_session", out)
        self.assertEqual(self.requests("POST", "/sessions/rm/apply"), [])

    def test_rm_fed_prints_the_drop_plan(self):
        self.state.can_drop = True
        code, out, _ = self.egs("session", "rm", "2")
        self.assertEqual(code, 0, out)
        self.assertIn("plan: drop fed session 2 and feed a fresh one", out)
        self.assertIn("would_delete: 4 submaps", out)
        self.assertIn("this is the session being mapped now: it is replaced by a fresh one", out)
        self.assertIn("the robot's pose is lost: run egs init-pose --place", out)
        self.assertIn("plan only: add --yes to apply", out)
        self.assertEqual(self.requests("POST", "/sessions/rm/apply"), [])

    def test_rm_fed_drops_it_in_process(self):
        self.state.can_drop = True
        self.state.anchor(7, pose(1.0, 1.0, 0.0), state="pending")
        self.state.anchors[7]["submap_id"] = [2, 0]
        code, out, err = self.egs("session", "rm", "2", "--yes")
        self.assertEqual(code, 0, out + err)
        self.assertIn("session 2 dropped; fed session now 3  @solve", out)
        self.assertIn("anchors orphaned: 7", out)
        self.assertIn("stand there and `egs place save` again", out)
        self.assertIn("the robot's pose is lost: run egs init-pose --place <p> or egs relocalize", out)
        self.assertEqual(len(self.requests("POST", "/sessions/rm/apply")), 1)
        self.assertEqual(self.requests("GET", "/status"), [])
        self.assertEqual(self.state.anchors[7]["state"], "orphan")
        code, out, _ = self.egs("session", "ls")
        self.assertIn("3 fed  nodes 0", out)
        self.assertNotIn("\n2 ", "\n" + out)

    def test_rm_json(self):
        code, out, _ = self.egs("session", "rm", "1", "--json")
        self.assertEqual(code, 0, out)
        j = json.loads(out)
        self.assertTrue(j["ok"])
        self.assertFalse(j["applied"])
        self.assertIn("plan_token", j)
        code, out, _ = self.egs("session", "rm", "1", "--yes", "--json")
        self.assertEqual(code, 0, out)
        j = json.loads(out)
        self.assertEqual((j["ok"], j["applied"], j["removed"]), (True, True, 1))
        self.assertFalse(j["pose_lost"])
        self.state.can_drop = True
        code, out, _ = self.egs("session", "rm", "2", "--yes", "--json")
        self.assertEqual(code, 0, out)
        j = json.loads(out)
        self.assertEqual(
            (j["applied"], j["removed"], j["fed_session"], j["pose_lost"]), (True, 2, 3, True)
        )
        self.assertEqual(j["anchors_orphaned"], [])
        self.assertIn("at_num_solves", j)

    def test_rm_fed_refused_by_the_host_is_not_applied(self):
        self.state.can_drop = True
        self.state.drop_refusal = "session_changed"
        code, out, err = self.egs("session", "rm", "2", "--yes")
        self.assertEqual(code, 1, out + err)
        self.assertIn("not applied: session_changed", out)
        self.assertIn("run egs session ls and plan again", out)
        self.assertNotIn("dropped;", out)
        code, out, _ = self.egs("session", "ls")
        self.assertIn("2 fed", out)

    def test_rm_fed_while_a_map_switch_is_pending_is_refused(self):
        self.state.can_drop = True
        self.state.switching = "fresh"
        self.state.switch_after_polls = 10**6
        code, out, _ = self.egs("session", "rm", "2", "--yes")
        self.assertEqual(code, 1)
        self.assertIn("rejected: switching", out)
        self.assertEqual(self.requests("POST", "/sessions/rm/apply"), [])

    def test_rm_plan_changed(self):
        import egs.sessions as sessions_mod

        orig = sessions_mod._plan_ok

        def plan_ok_then_mutate(ctx, plan):
            # A place gets saved in session 1 between plan and apply.
            self.state.anchor(9, pose(0.0, 0.0, 0.0))
            self.state.anchors[9]["submap_id"] = [1, 1]
            return orig(ctx, plan)

        sessions_mod._plan_ok = plan_ok_then_mutate
        try:
            code, out, _ = self.egs("session", "rm", "1", "--yes")
        finally:
            sessions_mod._plan_ok = orig
        self.assertEqual(code, 1)
        self.assertIn("not applied: plan_changed", out)
        self.assertEqual(len(self.state.sessions), 3)
        self.assertEqual(self.state.anchors[9]["state"], "frozen")


if __name__ == "__main__":
    unittest.main()
