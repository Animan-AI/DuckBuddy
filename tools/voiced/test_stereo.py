# -*- coding: utf-8 -*-
"""python -m pytest tools/voiced/test_stereo.py — the 48 kHz stereo stream: ring buffer and decimator."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stereo import Decimator, Ring  # noqa: E402

FS = 48000


def tone(hz, seconds=0.5):
    t = np.arange(int(seconds * FS)) / FS
    return np.sin(2 * math.pi * hz * t).astype(np.float32)


class DecimatorTest(unittest.TestCase):
    def test_speech_passes_and_what_would_alias_does_not(self):
        for hz, lo, hi in ((300, 0.97, 1.02), (1000, 0.97, 1.02), (4000, 0.95, 1.02), (10000, 0, 0.01), (20000, 0, 0.01)):
            y = Decimator().process(tone(hz))[200:]
            gain = np.sqrt(2 * np.mean(y ** 2))
            self.assertTrue(lo <= gain <= hi, (hz, gain))

    def test_chunks_give_the_same_as_one_go(self):
        x = np.random.default_rng(1).standard_normal(48000).astype(np.float32)
        whole = Decimator().process(x)
        d = Decimator()
        parts = np.concatenate([d.process(x[i:i + n]) for i, n in ((0, 1536), (1536, 1000), (2536, 7), (2543, 45457))])
        self.assertEqual(len(parts), len(whole))
        self.assertTrue(np.allclose(parts, whole, atol=1e-5))
        self.assertEqual(len(whole), 16000)


class RingTest(unittest.TestCase):
    def test_cut_by_absolute_index_across_the_wrap(self):
        r = Ring(1000)
        x = np.stack([np.arange(2500), -np.arange(2500)], 1).astype(np.float32)
        for i in range(0, 2500, 300):
            r.add(x[i:i + 300])
        got = r.cut(1600, 1700)
        self.assertTrue(np.array_equal(got[:, 0], np.arange(1600, 1700)))
        self.assertTrue(np.array_equal(got[:, 1], -np.arange(1600, 1700)))

    def test_what_is_gone_is_not_returned(self):
        r = Ring(1000)
        r.add(np.ones((2500, 2), dtype=np.float32))
        self.assertEqual(len(r.cut(0, 2500)), 1000)
        self.assertEqual(len(r.cut(2400, 3000)), 100)
        self.assertEqual(len(r.cut(100, 200)), 0)


if __name__ == "__main__":
    unittest.main()
