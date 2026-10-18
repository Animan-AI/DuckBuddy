# -*- coding: utf-8 -*-
"""A talking duck: speech re-voiced the way a "duck voice" (buccal speech, the cheek squeezing air
instead of the larynx) sounds — high and unsteady pitch, a raspy source, a fast amplitude flutter,
no low end, everything pinched into a narrow nasal band, and a gravelly period-doubling growl. Built
from scratch with the WORLD vocoder rather than cloned from any character's recordings, and with this
robot's own quack traits (`sounds-gen show`: the 37.8 Hz buzz) so the talking and the quacks are one
creature.

The articulation (WORLD's spectral envelope) is kept, so the words stay readable; the source under it
changes. 2026-10-15, the user on levels 1-3 (formants raised, no growl): "the heavy-metal feel is
right, but it is still a woman's voice" — raising the formants is what makes a voice read as female
and small, so levels 4-5 leave them where they are and pinch and growl instead.

    python ducktalk.py in.wav out.wav --level 4      # 1 mild ... 3 strong (bright), 4-5 metal
"""
import argparse

import numpy as np
import soundfile as sf

QUACK_AM_HZ = 37.8   # this robot's quack buzz rate (seed 3764158588)

LEVELS = {
    # f0: median Hz; wobble: pitch unsteadiness, st; fscale: formant scale; ap: aperiodicity floor
    # (rasp); flutter: quack-buzz depth; nasal: dB at 1.8 / 3.2 kHz; pinch: dB of a narrow resonance
    # at 2.2 kHz; growl: depth of the f0/2 period-doubling.
    1: dict(f0=300.0, wobble=0.6, fscale=1.12, ap=0.35, flutter=0.35, nasal=6.0, pinch=0.0, growl=0.0),
    2: dict(f0=360.0, wobble=1.0, fscale=1.20, ap=0.55, flutter=0.55, nasal=8.0, pinch=0.0, growl=0.0),
    3: dict(f0=420.0, wobble=1.5, fscale=1.28, ap=0.70, flutter=0.75, nasal=10.0, pinch=0.0, growl=0.0),
    4: dict(f0=360.0, wobble=1.5, fscale=1.00, ap=0.60, flutter=0.70, nasal=6.0, pinch=12.0, growl=0.55),
    5: dict(f0=420.0, wobble=2.0, fscale=0.96, ap=0.70, flutter=0.80, nasal=6.0, pinch=15.0, growl=0.85),
}


def _peak(x, sr, f0, gain_db, q):
    from scipy.signal import lfilter
    a = 10 ** (gain_db / 40)
    w = 2 * np.pi * f0 / sr
    alpha = np.sin(w) / (2 * q)
    b = np.array([1 + alpha * a, -2 * np.cos(w), 1 - alpha * a])
    d = np.array([1 + alpha / a, -2 * np.cos(w), 1 - alpha / a])
    return lfilter(b / d[0], d / d[0], x)


def _warp_envelope(sp, scale):
    """Formants moved by `scale` (> 1 up, a smaller head): sample the envelope at f / scale."""
    n = sp.shape[1]
    src = np.clip(np.arange(n) / scale, 0, n - 1)
    lo = np.floor(src).astype(int)
    hi = np.minimum(lo + 1, n - 1)
    frac = src - lo
    return sp[:, lo] * (1 - frac) + sp[:, hi] * frac


def duck_talk(x, sr, level=2, seed=0):
    import pyworld as pw
    from scipy.signal import butter, sosfilt
    p = LEVELS[level]
    rng = np.random.default_rng(seed)
    x = x.astype(np.float64)
    f0, t = pw.harvest(x, sr, frame_period=5.0)
    sp = pw.cheaptrick(x, f0, t, sr)
    ap = pw.d4c(x, f0, t, sr)
    voiced = f0 > 0
    med = np.median(f0[voiced]) if voiced.any() else p["f0"]
    # pitch: up to the duck's, its contour exaggerated a little, and an unsteady wobble on top
    contour = np.where(voiced, (f0 / med) ** 1.3, 1.0)
    walk = np.cumsum(rng.normal(0, 1, len(t)))
    walk = (walk - np.convolve(walk, np.ones(40) / 40, "same")) / 6.0          # slow drift removed
    wobble = 2 ** (p["wobble"] / 12 * np.clip(walk, -1.5, 1.5))
    f0n = np.where(voiced, p["f0"] * contour * wobble, 0.0)
    ap = np.maximum(ap, p["ap"])
    if p["fscale"] != 1.0:
        sp = _warp_envelope(sp, p["fscale"])
    y = pw.synthesize(np.ascontiguousarray(f0n), np.ascontiguousarray(sp), np.ascontiguousarray(ap), sr,
                      frame_period=5.0)
    # the growl: every other period louder — sidebands at f0 / 2, the gravel in a squeezed voice
    if p["growl"] > 0:
        f0s = np.interp(np.arange(len(y)) / sr, t, f0n)
        phase = np.cumsum(2 * np.pi * f0s / sr)
        y = y * (1 + p["growl"] * np.sin(phase / 2) * (f0s > 0))
    # squeezed into the nasal band, no chest
    y = sosfilt(butter(3, 380, "highpass", fs=sr, output="sos"), y)
    y = _peak(y, sr, 1800.0, p["nasal"], 1.6)
    y = _peak(y, sr, 3200.0, p["nasal"] * 0.5, 2.0)
    if p["pinch"] > 0:
        y = _peak(y, sr, 2200.0, p["pinch"], 3.0)
        y = sosfilt(butter(2, 5500, "lowpass", fs=sr, output="sos"), y)   # no airy top
    # the quack's flutter
    n = np.arange(len(y)) / sr
    fl = p["flutter"]
    s = np.sin(2 * np.pi * QUACK_AM_HZ * n)
    y = y * (1 - fl / 2 + fl / 2 * (0.6 * np.sign(s) + 0.4 * s))
    return y / (np.abs(y).max() + 1e-9) * 10 ** (-1 / 20)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inp")
    ap.add_argument("out")
    ap.add_argument("--level", type=int, default=4, choices=sorted(LEVELS))
    a = ap.parse_args()
    x, sr = sf.read(a.inp)
    if x.ndim > 1:
        x = x.mean(axis=1)
    sf.write(a.out, duck_talk(x, sr, a.level).astype(np.float32), sr, subtype="PCM_16")


if __name__ == "__main__":
    main()
