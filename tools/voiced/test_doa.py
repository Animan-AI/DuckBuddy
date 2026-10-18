# -*- coding: utf-8 -*-
"""python -m pytest tools/voiced/test_doa.py — the direction of a sound from two mics 6 cm apart, on
synthetic speech-like sound: delayed by a fraction of a sample, with noise, the head fan's hum and
some room echo."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from doa import MIC_SPACING_M, SPEED_OF_SOUND, direction, gcc_phat, onset_direction  # noqa: E402

FS = 48000


def speechlike(seconds, rng):
    """Band-limited noise (200-3500 Hz) under a 4 Hz syllable envelope."""
    n = int(seconds * FS)
    x = np.fft.irfft(np.fft.rfft(rng.standard_normal(n)) * _band(n, 200, 3500), n)
    t = np.arange(n) / FS
    return x * (0.3 + 0.7 * np.clip(np.sin(2 * math.pi * 4 * t), 0, None))


def _band(n, lo, hi):
    f = np.fft.rfftfreq(n, 1 / FS)
    return ((f >= lo) & (f <= hi)).astype(float)


def delayed(x, seconds):
    """x delayed by `seconds` (fractional), through the FFT."""
    n = len(x)
    f = np.fft.rfftfreq(n, 1 / FS)
    return np.fft.irfft(np.fft.rfft(x) * np.exp(-2j * math.pi * f * seconds), n)


def pair(yaw, rng, snr_db=15.0, fan=0.0, echo=False, seconds=1.0):
    """What the two mics hear of a talker at `yaw` (positive left)."""
    s = speechlike(seconds, rng)
    tau = MIC_SPACING_M * math.sin(yaw) / SPEED_OF_SOUND     # the right mic lags by tau
    left, right = s.copy(), delayed(s, tau)
    if echo:
        for ch in (0, 1):
            ir = np.zeros(int(0.15 * FS))
            ir[0] = 1.0
            taps = rng.integers(int(0.004 * FS), len(ir), 40)
            ir[taps] += rng.standard_normal(40) * 0.25 * np.exp(-taps / (0.05 * FS))
            if ch == 0:
                left = np.convolve(left, ir)[:len(s)]
            else:
                right = np.convolve(right, ir)[:len(s)]
    p = np.sqrt(np.mean(s ** 2)) / 10 ** (snr_db / 20)
    left = left + rng.standard_normal(len(s)) * p
    right = right + rng.standard_normal(len(s)) * p
    if fan:
        t = np.arange(len(s)) / FS
        hum = sum(math.cos(k) * np.sin(2 * math.pi * 73 * k * t + k) for k in range(1, 8)) * fan
        left, right = left + hum, right + hum
    return left, right


def fan_only(n, rng, fan=3.0):
    t = np.arange(n) / FS
    hum = sum(math.cos(k) * np.sin(2 * math.pi * 73 * k * t + k) for k in range(1, 8)) * fan
    return hum + rng.standard_normal(n) * 0.01, hum + rng.standard_normal(n) * 0.01


class DoaTest(unittest.TestCase):
    def test_the_sign_left_is_positive(self):
        rng = np.random.default_rng(1)
        d = direction(*pair(math.radians(40), rng), FS)
        self.assertGreater(d.yaw, 0)
        d = direction(*pair(math.radians(-40), rng), FS)
        self.assertLess(d.yaw, 0)

    def test_angles_in_front(self):
        rng = np.random.default_rng(2)
        for deg in (0, 15, 30, 45, 60, -20, -50):
            d = direction(*pair(math.radians(deg), rng), FS)
            self.assertAlmostEqual(math.degrees(d.yaw), deg, delta=4, msg=deg)
            self.assertGreater(d.conf, 0.7, deg)

    def test_the_fan_hum_does_not_pull_it_to_the_front(self):
        # the hum at 3x the speech's rms, identical at both mics (alone it would read as 0 deg); the
        # half second before the speech (voiced keeps it) tells which bins are the fan's
        rng = np.random.default_rng(3)
        for deg in (35, -35):
            left, right = pair(math.radians(deg), rng, fan=3.0, seconds=1.5)
            cut = int(0.5 * FS)
            quiet_l, quiet_r = fan_only(cut, rng)
            left[:cut], right[:cut] = quiet_l, quiet_r
            d = direction(left[cut:], right[cut:], FS, noise=(left[:cut], right[:cut]))
            self.assertAlmostEqual(math.degrees(d.yaw), deg, delta=6, msg=deg)

    def test_the_fan_s_higher_harmonics_too(self):
        # 10-12's fan: 260 Hz, harmonics up to ~1.9 kHz, all inside the band
        rng = np.random.default_rng(8)
        left, right = pair(math.radians(-40), rng, seconds=1.5)
        t = np.arange(len(left)) / FS
        hum = sum(np.sin(2 * math.pi * 260 * k * t + k) * (2.0 if k == 7 else 0.6) for k in range(1, 9))
        left, right = left + hum, right + hum
        cut = int(0.5 * FS)
        noise = (hum[:cut] + rng.standard_normal(cut) * 0.01, hum[:cut] + rng.standard_normal(cut) * 0.01)
        d = direction(left[cut:], right[cut:], FS, noise=noise)
        self.assertAlmostEqual(math.degrees(d.yaw), -40, delta=6)

    def test_with_room_echo_and_noise(self):
        rng = np.random.default_rng(4)
        errs = []
        for deg in (-60, -30, 0, 30, 60):
            d = direction(*pair(math.radians(deg), rng, snr_db=8, echo=True), FS)
            errs.append(abs(math.degrees(d.yaw) - deg))
        self.assertLess(np.mean(errs), 8, errs)

    def test_front_and_back_are_both_given(self):
        rng = np.random.default_rng(5)
        d = direction(*pair(math.radians(30), rng), FS)
        self.assertAlmostEqual(math.degrees(d.yaw_back), 150, delta=5)
        d = direction(*pair(math.radians(-30), rng), FS)
        self.assertAlmostEqual(math.degrees(d.yaw_back), -150, delta=5)

    def test_silence_has_no_direction_and_a_short_click_still_does(self):
        self.assertIsNone(direction(np.zeros(4800), np.zeros(4800), FS, min_rms=1e-3))
        rng = np.random.default_rng(6)
        click = np.zeros(4800)
        click[1000:1100] = rng.standard_normal(100)
        tau = MIC_SPACING_M * math.sin(math.radians(-45)) / SPEED_OF_SOUND
        d = direction(click, delayed(click, tau), FS)
        self.assertAlmostEqual(math.degrees(d.yaw), -45, delta=6)

    def test_a_clap_in_an_echoing_room_is_placed_by_its_onset(self):
        # the direct sound, then a room's reflections from all sides: walls 1-7 m away, 6-40 ms late,
        # each 30-60 % of the direct sound
        rng = np.random.default_rng(10)
        for deg in (70, -70, 20):
            n = int(0.3 * FS)
            clap = np.zeros(n)
            clap[2000:2200] = rng.standard_normal(200) * np.exp(-np.arange(200) / 40)
            tau = MIC_SPACING_M * math.sin(math.radians(deg)) / SPEED_OF_SOUND
            left, right = clap.copy(), delayed(clap, tau)
            for k in range(12):                       # echoes, each from a random side, 3-40 ms late
                lag = int(rng.uniform(0.006, 0.04) * FS)
                side = MIC_SPACING_M * math.sin(rng.uniform(-1.5, 1.5)) / SPEED_OF_SOUND
                e = np.roll(clap, lag) * rng.uniform(0.3, 0.6)
                left, right = left + e, right + delayed(e, side)
            d = onset_direction(left, right, FS)
            self.assertAlmostEqual(math.degrees(d.yaw), deg, delta=8, msg=deg)

    def test_gcc_phat_resolves_a_fraction_of_a_sample(self):
        rng = np.random.default_rng(7)
        s = speechlike(0.2, rng)
        tau, _ = gcc_phat(s, delayed(s, 2.3 / FS), FS, MIC_SPACING_M / SPEED_OF_SOUND)
        self.assertAlmostEqual(tau * FS, 2.3, delta=0.15)


if __name__ == "__main__":
    unittest.main()
