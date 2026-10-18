# -*- coding: utf-8 -*-
"""python -m unittest tools/visiond/test_tofwatch.py"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tofwatch import Approach, centre_m  # noqa: E402

DT = 1 / 15


def feed(det, t0, values):
    out = []
    for i, v in enumerate(values):
        r = det.step(t0 + i * DT, v)
        if r is not None:
            out.append((round(t0 + i * DT, 2), r))
    return out


class ApproachTest(unittest.TestCase):
    def test_someone_walking_up_is_seen_once(self):
        det = Approach()
        hits = feed(det, 0.0, [1.6] * 45 + [1.2, 0.9, 0.6, 0.55, 0.5, 0.5, 0.5] + [0.5] * 60)
        self.assertEqual(len(hits), 1)
        self.assertAlmostEqual(hits[0][1], 0.5)

    def test_a_near_wall_all_along_is_nobody(self):
        self.assertEqual(feed(Approach(), 0.0, [0.5] * 120), [])

    def test_one_noisy_frame_is_nobody(self):
        self.assertEqual(feed(Approach(), 0.0, [1.6] * 45 + [0.4] + [1.6] * 45), [])

    def test_open_space_then_a_hand(self):
        det = Approach()
        self.assertEqual(len(feed(det, 0.0, [None] * 45 + [0.3] * 10)), 1)

    def test_not_again_before_the_cooldown(self):
        det = Approach()
        first = feed(det, 0.0, [1.6] * 45 + [0.5] * 10 + [1.6] * 45 + [0.5] * 10)
        self.assertEqual(len(first), 1)
        later = feed(det, 20.0, [1.6] * 45 + [0.5] * 10)
        self.assertEqual(len(later), 1, "after 15 s it may greet again")

    def test_centre_reads_only_valid_middle_zones(self):
        d, st = [3000] * 64, [0] * 64
        for i in (18, 19, 20, 21, 26, 27):
            d[i], st[i] = 450, 5
        self.assertAlmostEqual(centre_m({"distance_mm": d, "status": st}), 0.45)
        self.assertIsNone(centre_m({"distance_mm": d, "status": [0] * 64}))


if __name__ == "__main__":
    unittest.main()
