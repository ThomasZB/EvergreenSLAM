import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from egs.yamlmini import YamlSubsetError, dumps, loads  # noqa: E402


class YamlMiniTest(unittest.TestCase):
    def test_readme_example(self):
        text = (
            "kind: fridge               # required\n"
            "aliases: [冰箱, icebox]     # optional\n"
            "labels: [food]\n"
            "offset: [0.8, 0.3, 0.0]    # optional\n"
            "offset_from: 7\n"
        )
        self.assertEqual(
            loads(text),
            {
                "kind": "fridge",
                "aliases": ["冰箱", "icebox"],
                "labels": ["food"],
                "offset": [0.8, 0.3, 0.0],
                "offset_from": 7,
            },
        )

    def test_scalars(self):
        d = loads("a: 7\nb: -1.5e2\nc: 'it''s'\nd: \"x # y\"\ne: ~\nf: true\ng: []\nh: a#b\n")
        self.assertEqual(
            d,
            {
                "a": 7,
                "b": -150.0,
                "c": "it's",
                "d": "x # y",
                "e": None,
                "f": True,
                "g": [],
                "h": "a#b",
            },
        )

    def test_rejects_outside_the_subset(self):
        for bad in (
            "pos:\n  x: 1\n",
            "aliases:\n- a\n",
            "a: [1, [2]]\n",
            "a: 1\na: 2\n",
            "just text\n",
            "a: {x: 1}\n",
            "a: [1, 2\n",
        ):
            with self.assertRaises(YamlSubsetError, msg=bad):
                loads(bad)

    def test_round_trip(self):
        d = {
            "offset_from": 3,
            "kind": "mug",
            "aliases": ["杯子", "red mug", "a: b", "12"],
            "offset": [1.0, -0.5, -1.5708],
            "note": "true",
        }
        text = dumps(d, ("kind", "aliases", "labels", "offset", "offset_from"))
        self.assertTrue(text.startswith('kind: mug\naliases: [杯子, red mug, "a: b", "12"]\n'))
        self.assertEqual(loads(text), d)


if __name__ == "__main__":
    unittest.main()
