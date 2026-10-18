# -*- coding: utf-8 -*-
"""The two head mics as voiced reads them since 2026-10-18: 48 kHz stereo straight from the codec (the
dsnoop), kept for a few seconds so a VAD segment's span can be cut out of it for its direction (doa.py),
and turned into the 16 kHz mono that VAD and SenseVoice want here, instead of by ALSA's plug.
numpy only (no scipy on the robot); tested in test_stereo.py.
"""
from __future__ import annotations

import numpy as np

FS_IN, FS_OUT = 48000, 16000
FACTOR = FS_IN // FS_OUT


def lowpass_taps(n: int = 95, cutoff_hz: float = 7000.0, fs: float = FS_IN) -> np.ndarray:
    """A windowed-sinc (Blackman) low-pass: flat to ~5.5 kHz, -70 dB past ~8.5 kHz at 95 taps."""
    m = np.arange(n) - (n - 1) / 2
    h = np.sinc(2 * cutoff_hz / fs * m) * np.blackman(n)
    return (h / h.sum()).astype(np.float32)


class Decimator:
    """48 kHz mono in chunks -> 16 kHz mono, continuous across chunks (filter state and phase kept).
    Output k is input 3k, delayed by the filter's (n - 1) / 2 = 47 samples (~1 ms)."""

    def __init__(self, taps: np.ndarray | None = None):
        self.h = lowpass_taps() if taps is None else taps
        self.state = np.zeros(len(self.h) - 1, dtype=np.float32)
        self.phase = 0

    def process(self, x: np.ndarray) -> np.ndarray:
        buf = np.concatenate([self.state, np.asarray(x, dtype=np.float32)])
        y = np.convolve(buf, self.h, mode="valid")
        out = y[self.phase::FACTOR]
        self.phase = (self.phase - len(y)) % FACTOR
        self.state = buf[len(buf) - (len(self.h) - 1):]
        return out.astype(np.float32)


class Ring:
    """The last `cap` frames of a (frames, channels) stream, addressed by absolute frame index. Written
    in place - no copying the history every chunk."""

    def __init__(self, cap: int, channels: int = 2):
        self.buf = np.zeros((cap, channels), dtype=np.float32)
        self.cap, self.total = cap, 0

    def add(self, x: np.ndarray):
        x = np.asarray(x, dtype=np.float32)
        n = len(x)
        if n >= self.cap:
            self.buf[:] = x[-self.cap:]
            self.total += n
            return
        i = self.total % self.cap
        k = min(n, self.cap - i)
        self.buf[i:i + k] = x[:k]
        self.buf[:n - k] = x[k:]
        self.total += n

    def cut(self, a: int, b: int) -> np.ndarray:
        """Frames [a, b) - as many of them as are still held."""
        first = max(self.total - self.cap, 0)
        a, b = max(a, first), min(b, self.total)
        if b <= a:
            return np.zeros((0, self.buf.shape[1]), dtype=np.float32)
        idx = np.arange(a, b) % self.cap
        return self.buf[idx]
