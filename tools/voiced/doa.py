# -*- coding: utf-8 -*-
"""Where a sound came from, from the two microphones on Xiaobai's head (docs/project/attention-gaze-design-
20261018-CN.md §1.1). Pure: numpy only, tested in test_doa.py.

The two mics sit left and right on the head, ~6 cm apart (the user, 2026-10-18), 8.4 cm round the head
for sound (the clap test, MIC_SPACING_M). A sound from the left reaches the left mic first; the right
channel lags it by tau = d * sin(theta) / c, |tau| <= 245 us. GCC-PHAT finds tau from the whitened
cross-spectrum over 300-8000 Hz, the peak searched within the physical +-tau only. The head fan hums at both mics alike - harmonics of 73 Hz (2026-10-17) or of
250-271 Hz up to 1.9 kHz (10-12) - and would pull every estimate to the front: whitening gives its
bins a full vote. So frames are Hann-windowed (no leakage from the strong lines) and, given a stretch of
noise from just before the speech, the bins the noise rules (under 4x its power) are dropped. Frames
are scored one by one and the loud ones vote.

A pair in a line cannot tell front from back: theta and 180 - theta give the same delay. The result
carries both; the life layer settles it (a prior, a head turn and a second sentence, or the camera).

Angles are the robot's: radians, positive to the left, 0 straight ahead.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

SPEED_OF_SOUND = 343.0       # m/s at ~20 C
# The mics are ~6 cm apart, but sound bends round the head to the far one: claps from the robot's left /
# right on 2026-10-18 arrived 221-253 us apart (front and back -18..-3 us), so the acoustic spacing is
# 245 us x 343 m/s = 8.4 cm. Channel 0 (J7, "MIC-L") is the robot's left.
MIC_SPACING_M = 0.084
# Broad: on claps 300-2800 Hz scattered (a few cycles in a short frame); with the search held to the
# physical +-245 us, the aliases a pair has above c / 2d (~2 kHz) do not win. The fan is masked by noise.
BAND_HZ = (300.0, 8000.0)
FRAME_S = 0.064
HOP_S = 0.032
INTERP = 16                  # sub-sample resolution of the correlation peak


@dataclass
class Direction:
    yaw: float               # the front answer, radians, positive left
    yaw_back: float          # the mirror answer behind (pi - yaw, wrapped)
    sigma: float             # spread of the frame votes, radians
    conf: float              # 0..1: share of loud frames agreeing within 10 deg
    frames: int              # loud frames that voted
    tau_s: float             # the delay of the right channel behind the left, seconds


NOISE_MARGIN = 4.0          # a bin votes only where the frame's power is this many times the noise's


def _nfft(frame: int) -> int:
    return 1 << int(math.ceil(math.log2(2 * frame)))


def noise_psd(left: np.ndarray, right: np.ndarray, frame: int) -> np.ndarray:
    """Mean power per bin of the two channels over frames of `frame` samples (Hann), for gcc_phat."""
    n, win = _nfft(frame), np.hanning(frame)
    acc, k = 0.0, 0
    for s in range(0, len(left) - frame + 1, frame // 2):
        L = np.fft.rfft(left[s:s + frame] * win, n)
        R = np.fft.rfft(right[s:s + frame] * win, n)
        acc = acc + (np.abs(L) ** 2 + np.abs(R) ** 2) / 2
        k += 1
    return acc / max(k, 1)


def gcc_phat(left: np.ndarray, right: np.ndarray, fs: float, max_tau: float,
             band=BAND_HZ, interp: int = INTERP, noise: np.ndarray | None = None) -> tuple[float, float]:
    """The delay of `right` behind `left` (seconds, > 0: the sound reached the left mic first) and the
    peak's height over the mean of the correlation (sharpness). `noise`: noise_psd() for this frame
    length - bins the noise rules do not vote."""
    frame = len(left)
    n, win = _nfft(frame), np.hanning(frame)
    L = np.fft.rfft(left * win, n)
    R = np.fft.rfft(right * win, n)
    cross = L * np.conj(R)
    power = (np.abs(L) ** 2 + np.abs(R) ** 2) / 2
    cross /= np.abs(cross) + 1e-12
    f = np.fft.rfftfreq(n, 1.0 / fs)
    cross[(f < band[0]) | (f > band[1])] = 0.0
    if noise is not None:
        cross[power < NOISE_MARGIN * noise] = 0.0
    cc = np.fft.irfft(cross, n * interp)
    m = max(1, int(round(interp * fs * max_tau)))
    cc = np.concatenate((cc[-m:], cc[:m + 1]))       # lags -m .. +m
    k = int(np.argmax(np.abs(cc)))
    lag = (k - m) / (interp * fs)                      # > 0: left leads (cc peaks where right = left shifted)
    sharp = float(np.abs(cc[k]) / (np.mean(np.abs(cc)) + 1e-12))
    return -lag, sharp


def tau_to_yaw(tau: float, spacing: float = MIC_SPACING_M, c: float = SPEED_OF_SOUND) -> float:
    return math.asin(max(-1.0, min(1.0, tau * c / spacing)))


def direction(left: np.ndarray, right: np.ndarray, fs: float, spacing: float = MIC_SPACING_M,
              c: float = SPEED_OF_SOUND, min_rms: float | None = None,
              noise: tuple[np.ndarray, np.ndarray] | None = None) -> Direction | None:
    """The direction of the sound in a segment (e.g. one VAD segment, or the word that named the duck).
    `noise`: the two channels of a stretch with no speech in it, from just before (voiced keeps it).
    None when no frame is loud enough to vote."""
    left = np.asarray(left, dtype=np.float64)
    right = np.asarray(right, dtype=np.float64)
    max_tau = spacing / c
    frame, hop = int(FRAME_S * fs), int(HOP_S * fs)
    if len(left) < frame:
        frame = len(left)
        hop = frame
    starts = range(0, len(left) - frame + 1, hop)
    rms = np.array([np.sqrt(np.mean(left[s:s + frame] ** 2 + right[s:s + frame] ** 2) / 2) for s in starts])
    if not len(rms):
        return None
    gate = min_rms if min_rms is not None else 0.5 * float(np.max(rms))
    npsd = None
    if noise is not None and len(noise[0]) >= frame:
        npsd = noise_psd(np.asarray(noise[0], dtype=np.float64), np.asarray(noise[1], dtype=np.float64), frame)
    votes, weights = [], []
    for s, r in zip(starts, rms):
        if r < gate:
            continue
        tau, sharp = gcc_phat(left[s:s + frame], right[s:s + frame], fs, max_tau, noise=npsd)
        votes.append(tau)
        weights.append(r * sharp)
    if not votes:
        return None
    votes, weights = np.array(votes), np.array(weights)
    order = np.argsort(votes)
    cum = np.cumsum(weights[order])
    tau = float(votes[order][np.searchsorted(cum, cum[-1] / 2)])     # weighted median
    yaws = np.array([tau_to_yaw(v, spacing, c) for v in votes])
    yaw = tau_to_yaw(tau, spacing, c)
    agree = np.abs(yaws - yaw) < math.radians(10)
    sigma = float(np.sqrt(np.average((yaws - yaw) ** 2, weights=weights)))
    back = math.copysign(math.pi, yaw) - yaw if yaw != 0 else math.pi
    return Direction(yaw=yaw, yaw_back=back, sigma=max(sigma, math.radians(3)),
                     conf=float(weights[agree].sum() / weights.sum()), frames=len(votes), tau_s=tau)


ONSET_S = (0.002, 0.008)     # a transient: 2 ms before its onset to 8 ms after - the direct sound, before a
                             # wall 1 m off echoes back (6 ms); the floor's comes within the first ms with it


def onset_direction(left: np.ndarray, right: np.ndarray, fs: float, spacing: float = MIC_SPACING_M,
                    c: float = SPEED_OF_SOUND) -> Direction | None:
    """The direction of a clap, a knock, a bang: from its first ~12 ms only. The room's echoes arrive
    within milliseconds and, over a speech-length frame, outvote the direct sound (2026-10-18: claps
    over 60 ms read anywhere from -73 to +59 deg; their first 8-14 ms gave +221..+253 us from the left
    and -237..-253 us from the right). The onset is where the level first passes half its peak."""
    left = np.asarray(left, dtype=np.float64)
    right = np.asarray(right, dtype=np.float64)
    env = np.maximum(np.abs(left), np.abs(right))
    if not len(env) or env.max() <= 0:
        return None
    on = int(np.argmax(env > 0.5 * env.max()))
    a, b = max(0, on - int(ONSET_S[0] * fs)), min(len(left), on + int(ONSET_S[1] * fs))
    tau, sharp = gcc_phat(left[a:b], right[a:b], fs, spacing / c)
    yaw = tau_to_yaw(tau, spacing, c)
    back = math.copysign(math.pi, yaw) - yaw if yaw != 0 else math.pi
    return Direction(yaw=yaw, yaw_back=back, sigma=math.radians(10), conf=min(1.0, sharp / 10.0), frames=1,
                     tau_s=tau)

