# -*- coding: utf-8 -*-
"""python -m pytest tools/visiond/test_motion.py — what moves catches the eye."""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from motion import SMALL, moving_blob, small_grey  # noqa: E402


def room(seed=0):
    """A textured still scene, SMALL and blurred like the real thing."""
    rng = np.random.default_rng(seed)
    big = (rng.integers(60, 200, (SMALL[1] // 6 + 1, SMALL[0] // 6 + 1), dtype=np.uint8)
           .repeat(6, 0).repeat(6, 1)[: SMALL[1], : SMALL[0]])
    return small_grey(np.ascontiguousarray(big))


class MotionTest(unittest.TestCase):
    def test_a_still_scene_has_no_motion(self):
        a = room()
        noisy = np.clip(a.astype(int) + np.random.default_rng(1).integers(-6, 7, a.shape), 0, 255).astype(np.uint8)
        self.assertIsNone(moving_blob(a, noisy))

    def test_a_hand_moving_on_the_left_is_found_there(self):
        a = room()
        b = a.copy()
        b[30:50, 20:35] = 250                    # something bright came in, left of centre
        cx, cy, frac, blob = moving_blob(a, b)
        self.assertTrue(15 < cx < 40 and 25 < cy < 55, (cx, cy))
        self.assertLess(frac, 0.15)

    def test_the_largest_patch_wins(self):
        a = room()
        b = a.copy()
        b[10:14, 140:144] = 255                  # a small flicker on the right
        b[50:75, 60:90] = 255                    # a ball in the middle
        cx, cy, _, _ = moving_blob(a, b)
        self.assertTrue(55 < cx < 95 and 45 < cy < 80, (cx, cy))

    def test_the_whole_picture_shifting_is_the_head_not_the_room(self):
        a = room()
        b = np.roll(a, 8, axis=1)                 # the head turned a little
        self.assertIsNone(moving_blob(a, b))

    def test_the_first_frame_has_nothing_to_compare(self):
        self.assertIsNone(moving_blob(None, room()))


if __name__ == "__main__":
    unittest.main()
