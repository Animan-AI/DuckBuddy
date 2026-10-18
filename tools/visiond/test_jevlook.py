# -*- coding: utf-8 -*-
"""python -m unittest tools/visiond/test_jevlook.py"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from jevlook import nv12_to_jpeg, person_event  # noqa: E402


class PersonEventTest(unittest.TestCase):
    def test_no_answer_is_the_bare_tof_event(self):
        self.assertEqual(person_event(None, 0.5), {"type": "person", "yaw": 0.0, "dist": 0.5, "via": "tof"})

    def test_someone_waving_on_the_right(self):
        a = {"person": {"noul": 0.95}, "activity": {"choice": "waving", "probabilities": {"waving": 0.8}},
             "facing": {"noul": 0.9}, "face": {"left": 0.7, "right": 0.8, "top": 0.1, "bottom": 0.3, "confidence": 0.8}}
        ev = person_event(a, 0.6)
        self.assertEqual((ev["present"], ev["activity"], ev["via"]), (True, "waving", "jev"))
        self.assertAlmostEqual(ev["yaw"], math.radians(-0.25 * 62), places=3)

    def test_a_wall_is_nobody_and_gets_no_direction(self):
        a = {"person": {"noul": 0.1}, "activity": {"choice": "unclear", "probabilities": {"unclear": 0.9}},
             "face": {"left": 0.4, "right": 0.6}}
        ev = person_event(a, 0.4)
        self.assertFalse(ev["present"])
        self.assertEqual(ev["yaw"], 0.0)

    def test_an_unsure_activity_is_unclear(self):
        a = {"person": {"noul": 0.9}, "activity": {"choice": "waving", "probabilities": {"waving": 0.3}}}
        self.assertEqual(person_event(a, 0.6)["activity"], "unclear")


class FrameTest(unittest.TestCase):
    def test_nv12_becomes_a_jpeg_jev_accepts(self):
        try:
            import cv2
        except ImportError:
            self.skipTest("no OpenCV here")
        w, h = 1920, 1080
        nv12 = np.full(w * h * 3 // 2, 128, np.uint8)
        jpeg = nv12_to_jpeg(nv12, w, h)
        self.assertEqual(jpeg[:2], b"\xff\xd8")
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(img.shape[1], 960)
        self.assertTrue(280 <= img.shape[0] <= 1568)


if __name__ == "__main__":
    unittest.main()
