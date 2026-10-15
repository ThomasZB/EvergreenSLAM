import json
import unittest
from unittest import mock

from support import EgsTestCase

from egs import pose as pose_cmd


class PoseTest(EgsTestCase):
    def test_once(self):
        code, out, _ = self.egs("pose")
        self.assertEqual(code, 0)
        self.assertEqual(
            out.splitlines(),
            [
                "pose 1.00,2.00,0.50  vel 0.30,0.00,0.10  seq 1  stamp_ns 1785000000100000000  "
                "age 0.04s"
            ],
        )
        code, out, _ = self.egs("pose", "--json")
        obj = json.loads(out)
        self.assertEqual(obj["pose"], [1.0, 2.0, 0.5])
        self.assertEqual(obj["velocity"], {"vx": 0.3, "vy": 0.0, "omega": 0.1})
        self.assertEqual(obj["stamp_ns"], 1785000000200000000)
        self.assertEqual(obj["seq"], 2)

    def test_once_without_a_global_pose(self):
        self.state.pose = None
        code, out, _ = self.egs("pose")
        self.assertEqual(code, 0)
        self.assertTrue(out.startswith("pose -  vel 0.30,0.00,0.10  seq 1"), out)
        self.assertIsNone(json.loads(self.egs("pose", "--json")[1])["pose"])

    def watch(self, *argv, polls):
        """Runs `egs pose --watch` until the fake sleep has been called `polls` times."""
        calls = []

        def fake_sleep(seconds):
            calls.append(seconds)
            if len(calls) >= polls:
                raise KeyboardInterrupt

        with mock.patch.object(pose_cmd, "_sleep", fake_sleep):
            code, out, err = self.egs("pose", "--watch", *argv)
        return code, out, err, calls

    def test_watch_prints_each_new_sample_once_as_ndjson(self):
        self.state.pose_seq_every = 2
        self.state.drop_pose_polls = (4,)
        code, out, err, calls = self.watch("10", "--json", polls=7)
        self.assertEqual(code, 0)
        lines = [json.loads(line) for line in out.splitlines()]
        # Polls 1..7, the 4th dropped: seqs 1, 1, 2, -, 3, 3, 4.
        self.assertEqual([obj["seq"] for obj in lines], [1, 2, 3, 4])
        self.assertEqual(lines[0]["pose"], [1.0, 2.0, 0.5])
        self.assertIn("reconnecting in 0.1s", err)
        self.assertIn("reconnected", err)
        self.assertTrue(all(0 <= s <= 0.1 for s in calls), calls)

    def test_watch_text_and_rate_limits(self):
        code, out, _, _ = self.watch(polls=2)
        self.assertEqual(code, 0)
        self.assertEqual(len(out.splitlines()), 2)
        self.assertTrue(out.startswith("pose 1.00,2.00,0.50"), out)
        for hz in ("0", "31"):
            code, _, err = self.egs("pose", "--watch", hz)
            self.assertEqual(code, 2)
            self.assertIn("--watch", err)


if __name__ == "__main__":
    unittest.main()
