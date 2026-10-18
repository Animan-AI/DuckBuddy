# -*- coding: utf-8 -*-
"""Standing still and head gestures, in the robot-matched BAM simulation (2026-10-13).

On the robot the stand-up network (standing slot, scale 0.6) self-oscillates at 4-5 Hz and its
head reached only 20-45% of duck-mcp's gestures. A walk-and-stand network (VelStand) would stand
under robotd's `walk_stands` at walking scale and gain. Per policy, after the stand network has
held the robot for 1 s as robotd starts it:

- still: 6 s at zero command — trunk pitch-rate RMS, its main frequency;
- shake / nod / tilt: duck-mcp's sine gestures on the head command (yaw 0.95 rad, pitch 0.55 rad
  around -0.15, roll 0.8 rad) — the head joint's span over the command's span, and the pitch-rate
  RMS while it moves.

    python training/bam/stand_head_eval_hl2915.py standup=training/policies/hl2915_standup_v3bl_5999.onnx@0.6#13 \
        velstand=path.onnx@0.9 --out results.md

`@scale` is the action scale, `#kp` the leg register-50 value (stand slot: 16 x 0.8 = 13).
"""
import argparse
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)

STAND = os.path.join(ROOT, "training/policies/hl2915_standup_v3bl_5999.onnx")
BACKLASH = [("0.35", 0), ("0.3:1.0", 1), ("0.3:1.0", 2)]
LAGS = (20.0, 25.0)
PRE_S, RUN_S = 1.0, 6.0
HEAD_IDX = {"neck_pitch": 5, "head_pitch": 6, "head_yaw": 7, "head_roll": 8}  # policy order


def gesture(kind):
    """head command (neck_pitch, head_pitch, head_yaw, head_roll) at time t into the run."""
    def f(t):
        if kind == "still":
            return [0.0] * 4
        if kind == "shake":
            return [0.0, 0.0, 0.95 * math.sin(2 * math.pi * t / 0.9), 0.0]
        if kind == "nod":
            return [0.0, -0.15 + 0.55 * math.sin(2 * math.pi * t / 0.8), 0.0, 0.0]
        return [0.0, 0.0, 0.0, 0.8 * (0.5 - 0.5 * math.cos(2 * math.pi * t / 2.0))]  # tilt
    return f


def one(job):
    import sim_hl2915 as sh
    import videos_hl2915 as v
    label, path, scale, kp, kind, lag, (bl, seed) = job
    a = v.sh_args(path, 0.0, PRE_S + RUN_S, "640x480", None, None, 1.0, v.SIDE)
    a.video = None
    a.pre_policy, a.pre_s, a.pre_kp_ratio = STAND, PRE_S, 0.8
    a.delay_ms, a.imu_lag_ms, a.vin, a.action_scale, a.kp = 32.0, lag, 11.6, scale, kp
    a.head_lowpass, a.legs_lowpass, a.backlash_deg, a.backlash_seed = 0.5, 0.7, bl, seed
    a.zero_error_deg = ""
    g = gesture(kind)
    a.cmd_fn = lambda t: {"twist": [0.0, 0.0, 0.0], "head": g(t - PRE_S) if t >= PRE_S else [0.0] * 4,
                          "body": [0.0] * 3}
    log = sh.run(a, a.scene, path, {"twist": [0, 0, 0], "head": [0] * 4, "body": [0] * 3})
    w0 = int(PRE_S * 50) + 25
    p, r = np.degrees(log["pitch"]), np.degrees(log["roll"])
    fell = any(np.hypot(p[w0:], r[w0:]) > 30)
    gy = np.degrees(log["gyro"][w0:, 1])
    fr = np.fft.rfftfreq(len(gy), 0.02)
    peak = float(fr[1:][np.abs(np.fft.rfft(gy - gy.mean()))[1:].argmax()])
    out = dict(key=(label, kind), fell=fell, rms=float(np.sqrt((gy ** 2).mean())), peak=peak)
    if kind != "still":
        j = {"shake": "head_yaw", "nod": "head_pitch", "tilt": "head_roll"}[kind]
        cmd = np.array([g(t)[["neck_pitch", "head_pitch", "head_yaw", "head_roll"].index(j)]
                        for t in np.arange(len(gy)) / 50.0 + 0.5])
        q = log["q"][w0:, HEAD_IDX[j]]
        out["ratio"] = float(np.ptp(q) / max(np.ptp(cmd), 1e-6))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("policies", nargs="+", help="label=path[@scale][#kp]")
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--out")
    a = ap.parse_args()
    jobs = []
    for spec in a.policies:
        label, rest = spec.split("=", 1)
        rest, _, kp = rest.partition("#")
        path, _, scale = rest.partition("@")
        jobs += [(label, path, float(scale or 1.0), int(kp or 16), kind, lag, bl)
                 for kind in ("still", "shake", "nod", "tilt") for lag in LAGS for bl in BACKLASH]
    with ProcessPoolExecutor(a.workers) as ex:
        res = list(ex.map(one, jobs))
    n = len(LAGS) * len(BACKLASH)
    lines = [f"Standing at zero twist; mean of {n} runs. ratio = head joint span / command span.", "",
             "| policy | falls | still: pitch-rate RMS / peak Hz | shake: ratio / RMS | nod: ratio / RMS | tilt: ratio / RMS |",
             "|---|---|---|---|---|---|"]
    for label in dict.fromkeys(j[0] for j in jobs):
        def m(kind, k):
            return np.nanmean([x.get(k, np.nan) for x in res if x["key"] == (label, kind)])
        falls = sum(x["fell"] for x in res if x["key"][0] == label)
        lines.append(f"| {label} | {falls} | {m('still', 'rms'):.1f} / {m('still', 'peak'):.1f} | "
                     + " | ".join(f"{m(k, 'ratio'):.2f} / {m(k, 'rms'):.1f}" for k in ("shake", "nod", "tilt")) + " |")
    table = "\n".join(lines)
    print(table)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(table + "\n")


if __name__ == "__main__":
    main()
