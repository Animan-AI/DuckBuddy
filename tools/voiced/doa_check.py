# -*- coding: utf-8 -*-
"""The clap test for the two head mics (docs/project/attention-gaze-design-20261018-CN.md §9): record the
48 kHz stereo stream while someone claps or says "小白" from a known direction, find each event, and print
the direction doa.py makes of it.

    python3 doa_check.py record left90 8        # 8 s to ~/robot/recs/doa/left90.wav (clap 3x, say 小白)
    python3 doa_check.py analyze ~/robot/recs/doa/left90.wav [--swap] [--spacing 0.06]

Channel 0 is the codec's left input (J7, "MIC-L"); whether that mic sits on the robot's left is what
the first recording from a side tells (a clap at the robot's left must read positive; --swap if not).
Recordings hold voices: they stay on the robot and are deleted once the mics are calibrated.
"""
import argparse
import math
import os
import subprocess
import sys
import wave

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from doa import MIC_SPACING_M, direction, onset_direction  # noqa: E402

FS = 48000
OUT = os.path.expanduser("~/robot/recs/doa")


def record(label: str, seconds: float):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, label + ".wav")
    subprocess.run(["arecord", "-q", "-D", "duck_in", "-f", "S16_LE", "-r", str(FS), "-c", "2",
                    "-d", str(int(math.ceil(seconds))), path], check=True)
    print(path)


def load(path):
    with wave.open(path) as w:
        assert w.getnchannels() == 2 and w.getframerate() == FS, "48 kHz stereo expected"
        x = np.frombuffer(w.readframes(w.getnframes()), np.int16).reshape(-1, 2).astype(np.float64) / 32768
    return x[:, 0], x[:, 1]


def events(left, right, min_gap_s=0.15):
    """(start, end) samples of the loud stretches: 20 ms frames over 4x the median level, merged across
    gaps under min_gap_s, padded 50 ms."""
    hop = int(0.02 * FS)
    rms = np.array([np.sqrt(np.mean((left[i:i + hop] ** 2 + right[i:i + hop] ** 2) / 2))
                    for i in range(0, len(left) - hop, hop)])
    loud = rms > 2.5 * np.median(rms)
    spans, start, last = [], None, None
    for i, on in enumerate(loud):
        if on:
            start = i if start is None else start
            last = i
        elif start is not None and (i - last) * hop > min_gap_s * FS:
            spans.append((start * hop, (last + 1) * hop))
            start = None
    if start is not None:
        spans.append((start * hop, (last + 1) * hop))
    pad = int(0.05 * FS)
    return [(max(0, a - pad), min(len(left), b + pad)) for a, b in spans]


def analyze(path, swap=False, spacing=MIC_SPACING_M):
    left, right = load(path)
    if swap:
        left, right = right, left
    print(f"{os.path.basename(path)}: {len(left) / FS:.1f} s, rms L {np.sqrt(np.mean(left ** 2)):.4f} "
          f"R {np.sqrt(np.mean(right ** 2)):.4f}, corr {np.corrcoef(left, right)[0, 1]:.2f}")
    for a, b in events(left, right):
        q0 = max(0, a - int(0.5 * FS))
        noise = (left[q0:a], right[q0:a]) if a - q0 > int(0.1 * FS) else None
        transient = b - a < int(0.25 * FS)          # a clap, a knock: its onset; speech: frame votes
        d = (onset_direction(left[a:b], right[a:b], FS, spacing=spacing) if transient
             else direction(left[a:b], right[a:b], FS, spacing=spacing, noise=noise))
        if d is None:
            continue
        print(f"  {'clap ' if transient else 'voice'} {a / FS:5.2f}-{b / FS:5.2f} s  yaw {math.degrees(d.yaw):+6.1f} deg (or {math.degrees(d.yaw_back):+6.1f} "
              f"behind)  sigma {math.degrees(d.sigma):4.1f}  conf {d.conf:.2f}  frames {d.frames}  "
              f"tau {d.tau_s * 1e6:+6.1f} us")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("label")
    r.add_argument("seconds", type=float)
    z = sub.add_parser("analyze")
    z.add_argument("wav", nargs="+")
    z.add_argument("--swap", action="store_true")
    z.add_argument("--spacing", type=float, default=MIC_SPACING_M)
    a = ap.parse_args()
    if a.cmd == "record":
        record(a.label, a.seconds)
    else:
        for p in a.wav:
            analyze(p, a.swap, a.spacing)


if __name__ == "__main__":
    main()
