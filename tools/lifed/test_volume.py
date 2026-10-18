# -*- coding: utf-8 -*-
"""python -m unittest tools/lifed/test_volume.py — the speaker's loudness (lifed.Volume), no amixer."""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lifed  # noqa: E402


class FakeClock:
    def __init__(self, hm):
        self.hm = hm

    def now(self):
        return self.hm


class VolumeTest(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(tempfile.mkdtemp(), "state.json")

    def vol(self, hm=(15, 0)):
        return lifed.Volume(self.path, FakeClock(hm), card=None)

    def test_steps_clamp_and_are_kept(self):
        v = self.vol()
        self.assertEqual(v.change(step=-15), 85)
        self.assertEqual(v.change(set_to=5), lifed.Volume.MIN)
        self.assertEqual(v.change(set_to=150), lifed.Volume.MAX)
        v.change(set_to=60)
        self.assertEqual(self.vol().state(), {"user": 60, "now": 60, "night_cap": False}, "after a restart")

    def test_night_caps_unless_turned_up_that_night(self):
        self.vol().change(set_to=80)
        night = self.vol((23, 10))
        self.assertEqual(night.effective(), lifed.Volume.NIGHT_CAP)
        self.assertTrue(night.state()["night_cap"])
        self.assertEqual(night.change(step=15), 55, "louder at night: from what it plays at")
        self.assertEqual(night.change(set_to=90), 90, "turned up past the cap at night: honoured")
        self.assertEqual(self.vol((6, 0)).effective(), 90, "... until the morning")
        self.assertEqual(self.vol((8, 0)).effective(), 90)

    def test_no_clock_no_night(self):
        self.vol().change(set_to=80)
        self.assertEqual(self.vol(None).effective(), 80)


if __name__ == "__main__":
    unittest.main()
