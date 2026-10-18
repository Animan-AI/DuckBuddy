# -*- coding: utf-8 -*-
"""python -m pytest tools/visiond/test_faces.py — where a face is, as angles from the camera."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from faces import DETECT_SIZE, HFOV_DEG, MODEL, face_angles, small_bgr  # noqa: E402


class AnglesTest(unittest.TestCase):
    def test_the_centre_is_straight_ahead(self):
        self.assertEqual(face_angles(320, 180, 640, 360), (0.0, -0.0))

    def test_the_right_edge_is_half_the_field_to_the_right(self):
        yaw, pitch = face_angles(640, 180, 640, 360)
        self.assertAlmostEqual(math.degrees(yaw), -HFOV_DEG / 2, places=6)

    def test_left_is_positive_and_up_is_positive(self):
        yaw, pitch = face_angles(100, 50, 640, 360)
        self.assertGreater(yaw, 0)
        self.assertGreater(pitch, 0)


@unittest.skipUnless(os.path.exists(MODEL), "the YuNet model is not here")
class FinderTest(unittest.TestCase):
    def test_a_blank_frame_has_no_face(self):
        import numpy as np
        from faces import FaceFinder
        self.assertEqual(FaceFinder().find(np.full((DETECT_SIZE[1], DETECT_SIZE[0], 3), 128, np.uint8)), [])


class SmallFrameTest(unittest.TestCase):
    def test_duckcams_frame_comes_out_small_and_in_colour(self):
        import numpy as np
        w, h = 1920, 1088
        nv12 = np.zeros((h * 3 // 2, w), np.uint8)
        nv12[:h] = 120                                  # mid grey luma
        nv12[h:] = 128                                  # no chroma
        nv12[h:, 0:w // 2:2] = 90                       # U low on the left half: towards yellow (B low)
        img = small_bgr(nv12.ravel(), w, h)
        self.assertEqual(img.shape, (DETECT_SIZE[1], DETECT_SIZE[0], 3))
        left, right = img[:, : DETECT_SIZE[0] // 4].mean(axis=(0, 1)), img[:, -DETECT_SIZE[0] // 4:].mean(axis=(0, 1))
        self.assertLess(left[0], right[0] - 20, (left, right))
        self.assertLess(abs(right[0] - right[2]), 3, "no chroma: grey")


if __name__ == "__main__":
    unittest.main()
