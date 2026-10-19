"""init-pose / relocalize waiting for their outcome, the relocalization line, and egs events."""

import json
import unittest
from unittest import mock

from support import EgsTestCase

from egs import commands


class RelocalizationTest(EgsTestCase):
    def test_init_pose_waits_for_a_match(self):
        self.state.relocalization_pending_polls = 1
        code, out, _ = self.egs("init-pose", "1.5", "-2", "0.3")
        self.assertEqual(code, 0, out)
        self.assertEqual(out, "matched: corrected 1.10 m 164°; check egs here\n")
        polls = self.requests("GET", "/here")
        self.assertEqual([q for _, _, q in polls], [{"relocalization": "1"}] * 2)

    def test_init_pose_place_reports_kind_and_anchor_in_json(self):
        code, out, _ = self.egs("init-pose", "--place", "places/dock", "--json")
        self.assertEqual(code, 0, out)
        obj = json.loads(out)
        self.assertTrue(obj["ok"])
        self.assertEqual(obj["request_id"], 1)
        r = obj["relocalization"]
        self.assertEqual((r["kind"], r["anchor"], r["state"]), ("place", 1, "matched"))

    def test_no_match_with_nothing_to_match_says_outside_the_map(self):
        self.state.relocalization_outcome = {"state": "no_match", "candidates": 0}
        code, out, _ = self.egs("init-pose", "--place", "places/dock")
        self.assertEqual(code, 1)
        self.assertTrue(out.startswith("no_match (0 candidates): "), out)
        self.assertIn("outside the saved map", out)

    def test_no_match_with_candidates_points_at_the_recovery(self):
        self.state.relocalization_outcome = {"state": "no_match", "candidates": 4}
        code, out, _ = self.egs("relocalize", "--json")
        self.assertEqual(code, 1)
        obj = json.loads(out)
        self.assertFalse(obj["ok"])
        self.assertEqual(obj["reason"], "no_match")
        self.assertEqual(obj["relocalization"]["kind"], "global")
        code, out, _ = self.egs("relocalize")
        self.assertIn("no_match (4 candidates)", out)
        self.assertIn("egs session rm <fed id>", out)

    def test_superseded(self):
        self.state.relocalization_outcome = {"state": "superseded", "superseded_by": 7}
        code, out, _ = self.egs("init-pose", "1", "2", "0")
        self.assertEqual((code, out), (1, "superseded by request 7\n"))
        self.state.relocalization_outcome = {"state": "superseded", "superseded_by": None}
        code, out, _ = self.egs("init-pose", "1", "2", "0")
        self.assertEqual(code, 1)
        self.assertIn("fed session was dropped", out)

    def test_still_pending_at_the_deadline_is_outcome_unknown(self):
        self.state.relocalization_pending_polls = 1000
        with mock.patch.object(commands, "RELOCALIZATION_POLL_S", 0.01):
            code, out, err = self.egs("init-pose", "1", "2", "0", "--wait", "0.05")
        self.assertEqual(code, 3)
        self.assertIn("outcome unknown: request 1 still pending", err)

    def test_a_request_the_process_forgot_is_outcome_unknown(self):
        self.state.relocalizations.clear()
        with mock.patch.object(self.state, "relocalization_request", return_value=42):
            code, _, err = self.egs("relocalize")
        self.assertEqual(code, 3)
        self.assertIn("no longer tracks request 42", err)

    def test_no_wait_returns_at_once(self):
        code, out, _ = self.egs("relocalize", "--no-wait")
        self.assertEqual(code, 0)
        self.assertIn("global relocalization sent as request 1; not a promise", out)
        self.assertEqual(self.requests("GET", "/here"), [])

    def test_here_and_status_show_the_latest_request(self):
        self.assertNotIn("relocalization", self.egs("here")[1])
        self.state.relocalization_outcome = {"state": "no_match", "candidates": 0}
        self.egs("init-pose", "--place", "places/dock")
        line = "relocalization 1 place (anchor 1) no_match (0 candidates)"
        self.assertIn(line, self.egs("here")[1].splitlines())
        self.assertIn(line, self.egs("status")[1].splitlines())


KIDNAP = {
    "time_ns": 5,
    "request_id": 4,
    "session": 2,
    "keyframes": 8,
    "lowest_match_score": 0.1,
    "base_score": 0.31,
    "base_coverage": 0.78,
    "since_base_match_m": 4.9,
}


class KidnapTest(EgsTestCase):
    def kidnap(self, request_state, continuity_broken):
        self.state.localization["kidnap"] = dict(KIDNAP, continuity_broken=continuity_broken)
        self.state.relocalizations[4] = {
            "id": 4,
            "kind": "kidnap",
            "anchor": None,
            "state": request_state,
            "candidates": 12,
            "constraints": 0,
            "correction_m": None,
            "correction_rad": None,
            "requested_ns": None,
            "resolved_ns": None,
            "superseded_by": None,
        }

    def kidnap_line(self, command="here"):
        lines = [l for l in self.egs(command)[1].splitlines() if l.startswith("kidnap")]
        self.assertEqual(len(lines), 1)
        return lines[0]

    def test_no_line_without_a_kidnap(self):
        self.assertNotIn("kidnap", self.egs("here")[1])
        self.assertNotIn("kidnap", self.egs("status")[1])

    def test_a_running_search_says_so(self):
        self.kidnap("pending", True)
        self.assertEqual(
            self.kidnap_line(),
            "kidnap suspected: base 0.31 (cover 78%), 4.9m without base match, match dipped to "
            "0.10; request 4, searching",
        )
        self.assertIn("relocalization 4 kidnap pending", self.egs("here")[1].splitlines())

    def test_found_again(self):
        self.kidnap("matched", False)
        self.assertTrue(self.kidnap_line("status").endswith("; request 4, found again"))

    def test_still_unconfirmed_points_at_the_recovery(self):
        self.kidnap("matched", True)
        self.assertTrue(
            self.kidnap_line().endswith(
                "not confirmed by the base since: if egs here --sample 20 stays below 0.45, "
                "egs session rm 2 (plan, then --yes), then egs relocalize"
            )
        )

    def test_a_boot_that_never_matched_the_base(self):
        self.kidnap("no_match", True)
        self.state.localization["kidnap"]["lowest_match_score"] = None
        self.assertIn("never matched the base", self.kidnap_line())

    def test_json_carries_the_object(self):
        self.kidnap("matched", False)
        obj = json.loads(self.egs("here", "--json")[1])
        self.assertEqual(obj["localization"]["kidnap"]["request_id"], 4)


class EventsTest(EgsTestCase):
    def setUp(self):
        super().setUp()
        matched = {"state": "matched", "candidates": 2, "constraints": 1, "superseded_by": None}
        self.state.events = [
            {"time_ns": 1, "kind": "session_frozen", "session": 1},
            {"time_ns": 2, "kind": "pose_correction", "session": 2, "correction_m": 0.45},
            dict(matched, time_ns=3, kind="relocalization", request_id=3, correction_m=1.1),
            {"time_ns": 4, "kind": "fed_session_dropped", "session": 2, "fed_now": 3},
        ]
        self.state.events[1]["correction_rad"] = 0.0
        self.state.events[2]["correction_rad"] = 2.86

    def test_lists_and_pages(self):
        code, out, _ = self.egs("events")
        self.assertEqual(code, 0)
        self.assertEqual(
            out.splitlines(),
            [
                "0 session_frozen session 1",
                "1 pose_correction session 2 moved 0.45 m 0°",
                "2 relocalization request 3 matched: corrected 1.10 m 164°",
                "3 fed_session_dropped session 2, fed now 3",
                "next_seq 4",
            ],
        )
        code, out, _ = self.egs("events", "--since", "3")
        self.assertEqual(out.splitlines()[0], "3 fed_session_dropped session 2, fed now 3")
        obj = json.loads(self.egs("events", "--since", "4", "--json")[1])
        self.assertEqual((obj["events"], obj["next_seq"], obj["lost"]), ([], 4, False))

    def test_a_kidnap_names_its_evidence_and_search(self):
        self.state.events.append(dict(KIDNAP, kind="kidnap_suspected"))
        self.assertEqual(
            self.egs("events", "--since", "4")[1].splitlines()[0],
            "4 kidnap_suspected session 2: base 0.31 (cover 78%), 4.9m without base match, "
            "match dipped to 0.10; searching as request 4",
        )

    def test_lost_events_are_reported(self):
        self.state.events_first_seq = 10
        code, out, err = self.egs("events", "--since", "2")
        self.assertEqual(code, 0)
        self.assertIn("events lost", err)
        self.assertTrue(out.startswith("10 session_frozen"))

    def test_follow_prints_one_json_object_per_event(self):
        sleeps = []

        def sleep(seconds):
            sleeps.append(seconds)
            if len(sleeps) == 1:
                self.state.events.append({"time_ns": 5, "kind": "session_removed", "session": 1})
            else:
                raise KeyboardInterrupt

        with mock.patch.object(commands.time, "sleep", sleep):
            code, out, _ = self.egs("events", "--follow", "--json", "--since", "3")
        self.assertEqual(code, 130)
        lines = [json.loads(l) for l in out.splitlines()]
        self.assertEqual([e["seq"] for e in lines], [3, 4])
        self.assertEqual(lines[1]["kind"], "session_removed")
        since = [q["since"] for _, _, q in self.requests("GET", "/events")]
        self.assertEqual(since, ["3", "4"])


if __name__ == "__main__":
    unittest.main()
