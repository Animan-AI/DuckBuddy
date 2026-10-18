# -*- coding: utf-8 -*-
"""Stress-test walking policies in the BAM simulation against what the robot showed.

Runs each policy through conditions built from the 2026-10-05 robot test: leg servos at a
fraction of the fitted gain, a head heavier and further forward than the model, more
delay. Reports fall time, peak forward lean, distance walked and how far the neck dived
(the robot's failure threw it to -80°; the home pose is +20°).

    python training/bam/stress_hl2915.py [--lowpass] training/policies/hl2915_walk_v1_3999.onnx ...
"""
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
LEGS = ["hip_roll", "hip_pitch", "knee", "ankle"]


def legs(scale):
    return ",".join(f"{side}_{j}={scale}" for side in ("left", "right") for j in LEGS)


HEAVY = ["--head-mass-scale", "1.3", "--head-com-forward-mm", "10"]
CONDITIONS = [
    ("nominal", [], 20),
    ("legs 0.5 + heavy head", ["--kp-scale", legs(0.5)] + HEAVY, 20),
    ("legs 0.4 + heavy head", ["--kp-scale", legs(0.4)] + HEAVY, 20),
    ("legs 0.5 + heavy head, 40 ms", ["--kp-scale", legs(0.5)] + HEAVY, 40),
    ("hips L0.3/R0.5 + heavy head", ["--kp-scale", "left_hip_pitch=0.3,right_hip_pitch=0.5"] + HEAVY, 20),
]


LOWPASS = []  # "--lowpass": robotd's default target low-pass, as the v3 policies train with


def run(policy, extra, delay, vx=0.2, duration=8.0):
    with tempfile.TemporaryDirectory() as tmp:
        npz = os.path.join(tmp, "log.npz")
        cmd = [sys.executable, os.path.join(HERE, "sim_hl2915.py"),
               "--params", os.path.join(ROOT, "training/bam/params/hl2915/m4.json"),
               "--policy", policy, "--kp", "16", "--kp-head", "16", "--action-scale", "1.0",
               "--duration", str(duration), "--vx", str(vx), "--ramp-s", "0.3", "--vin", "10.7",
               "--delay-ms", str(delay), "--npz", npz] + LOWPASS + extra
        out = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
        if out.returncode != 0:
            return None, out.stderr[-400:]
        with np.load(npz) as z:  # closed before the temp dir goes (Windows holds it open)
            pitch = np.degrees(z["pitch"])
            height, x, neck = z["z"], z["x"], np.degrees(z["q"][:, 5])
        down = (height < 0.05) | (np.abs(pitch) > 45)
        fell = np.argmax(down) / 50.0 if down.any() else None
        return dict(fell=fell, lean_max=pitch.max(), dx=float(x[-1] - x[0]), neck_min=neck.min()), None


def main():
    policies = [p for p in sys.argv[1:] if p != "--lowpass"]
    if "--lowpass" in sys.argv:
        LOWPASS[:] = ["--head-lowpass", "0.5", "--legs-lowpass", "0.7"]
    print(f"{'condition':<32}" + "".join(f"{os.path.basename(p)[:22]:>34}" for p in policies))
    for label, extra, delay in CONDITIONS:
        row = f"{label:<32}"
        for p in policies:
            r, err = run(p, extra, delay)
            if r is None:
                row += f"{'error':>34}"
                print(err, file=sys.stderr)
                continue
            fell = "walks" if r["fell"] is None else f"falls {r['fell']:.1f}s"
            row += f"{fell:>11} lean{r['lean_max']:5.1f} dx{r['dx']:+5.2f} neck{r['neck_min']:+4.0f}"
        print(row, flush=True)


if __name__ == "__main__":
    main()
