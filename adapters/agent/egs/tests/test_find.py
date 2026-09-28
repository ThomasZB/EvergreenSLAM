import time
import unittest

from support import NS_MIN, EgsTestCase, obs

from egs.memory import RANK_ALIAS, RANK_FUZZY, RANK_KIND, RANK_NAME, RANK_WORD, match_rank


def rows(out):
    return [l.split(" ")[0] for l in out.splitlines() if l.startswith("places/")]


class MatchRankTest(unittest.TestCase):
    def test_tiers(self):
        self.assertEqual(match_rank("apple", "apple", {}), RANK_NAME)
        self.assertEqual(match_rank("Coffee Table", "coffee_table", {}), RANK_NAME)
        self.assertEqual(match_rank("冰箱", "fridge", {"aliases": ["冰箱"]}), RANK_ALIAS)
        self.assertEqual(match_rank("fridge", "icebox", {"kind": "fridge"}), RANK_KIND)
        self.assertEqual(match_rank("food", "milk", {"labels": ["food"]}), RANK_KIND)
        self.assertEqual(match_rank("apple", "green_apple", {}), RANK_WORD)
        self.assertEqual(match_rank("apple", "apple-2", {}), RANK_WORD)
        self.assertEqual(match_rank("aple", "apple", {}), RANK_FUZZY)

    def test_word_boundary(self):
        self.assertIsNone(match_rank("apple", "pineapple", {"kind": "fruit"}))
        self.assertIsNone(match_rank("cup", "hiccup", {}))
        self.assertIsNone(match_rank("cup", "cupboard", {}))
        self.assertEqual(match_rank("cup", "cup_holder", {}), RANK_WORD)


class FindTest(EgsTestCase):
    def test_absent_sinks_below_weaker_matches(self):
        code, out, _ = self.egs("find", "apple")
        self.assertEqual(code, 0)
        self.assertEqual(
            rows(out),
            [
                "places/office/desk/apple",
                "places/office/desk/green_apple",
                "places/kitchen/table/apple",
            ],
        )
        self.assertNotIn("pineapple", out)
        first = out.splitlines()[0]
        self.assertIn("[apple] 5.4m", first)
        self.assertIn("last=seen 6d", first)
        self.assertIn("prec=offset:places/office/desk/apple(frozen)", first)
        self.assertIn("last=absent 20m", out.splitlines()[2])
        self.assertIn("prec=inherited:places/kitchen/table(frozen)", out.splitlines()[2])
        self.assertTrue(out.rstrip().endswith("@solve 100"))

    def test_invalidated_absent_does_not_sink(self):
        rel = "places/kitchen/table/apple/observations.jsonl"
        absent_at = int(self.read(rel).split(":")[1].split(",")[0])
        self.write(
            rel,
            self.read(rel)
            + '{"recorded_at": %d, "source": "human", "invalidates": %d}\n'
            % (time.time_ns(), absent_at),
        )
        code, out, _ = self.egs("find", "apple")
        self.assertEqual(rows(out)[1], "places/kitchen/table/apple")
        self.assertIn("last=-", out)

    def test_rank_then_distance(self):
        code, out, _ = self.egs("find", "room")
        self.assertEqual(code, 0)
        # all three are kind matches; kitchen resolves at 5 m, office and garage have no position
        self.assertEqual(rows(out), ["places/kitchen", "places/garage", "places/office"])
        self.assertIn("prec=none", out)

    def test_alias_and_label(self):
        code, out, _ = self.egs("find", "苹果")
        self.assertEqual(rows(out), ["places/kitchen/table/apple"])
        code, out, _ = self.egs("find", "food")
        self.assertEqual(rows(out), ["places/kitchen/fridge/milk"])

    def test_skills_are_not_nodes(self):
        code, out, _ = self.egs("find", "clean")
        self.assertIn("no match", out)
        self.assertEqual(self.requests("GET", "/anchors"), [])

    def test_unreadable_place_flags_only_its_rows(self):
        self.write("places/broken/place.yaml", "anchor: [1\n")
        self.write("places/broken/cup/node.yaml", "kind: cup\n")
        self.write("places/dock/cup/node.yaml", "kind: cup\n")
        code, out, _ = self.egs("find", "cup")
        self.assertEqual(code, 0, out)
        by_path = {l.split(" ")[0]: l for l in out.splitlines() if l.startswith("places/")}
        self.assertIn("prec=unreadable:places/broken", by_path["places/broken/cup"])
        self.assertIn("prec=inherited", by_path["places/dock/cup"])
        code, out, err = self.egs("where", "places/broken")
        self.assertNotEqual(code, 0)
        self.assertIn("places/broken/place.yaml", out + err)

    def test_limit_and_more(self):
        for i in range(12):
            self.write("places/dock/cup_%02d/node.yaml" % i, "kind: cup\n")
        now = time.time_ns()
        self.write("places/dock/cup_00/observations.jsonl", obs(now - 5 * NS_MIN, "absent"))
        code, out, _ = self.egs("find", "cup")
        lines = rows(out)
        self.assertEqual(len(lines), 10)
        self.assertNotIn("places/dock/cup_00", lines)
        self.assertIn("... 2 more", out)
        code, out, _ = self.egs("find", "cup", "--limit", "0")
        self.assertEqual(rows(out)[-1], "places/dock/cup_00")


if __name__ == "__main__":
    unittest.main()
