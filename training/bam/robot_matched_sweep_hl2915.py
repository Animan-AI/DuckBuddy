# -*- coding: utf-8 -*-
"""Rank walking checkpoints under the conditions the HL-2915 robot actually runs (2026-10-10).

`checkpoint_sweep_hl2915.py` ranked checkpoints with the IMU read exactly and 20 ms of command
delay, and its picks did not survive the robot. Here every run is set up as robotd runs it:
- IMU observed 20 or 25 ms late (median filter off): the delay the robot's rocking matches;
- command delay 32 ms (measured in the air), robotd's target low-pass 0.5 / 0.7;
- gear play +-0.35 deg and random 0.3-1.0 deg draws;
- the stand-up policy holding the robot at gain x0.8 for 1 s before the walk, as robotd starts it;
- 0.2 m/s for 8 s.

Metrics per policy (mean over runs): falls (tilt > 30 deg), trunk pitch-rate RMS and neck
target standard deviation (robot, config G: 34-43 deg/s and 3.8-5 deg), |yaw| after 1 s and
after 8 s, distance walked.

    python training/bam/robot_matched_sweep_hl2915.py \
        v4lf=training/policies/hl2915_walk_v4bl_lowfric_3999.onnx@0.8 \
        v6_3999=sweep/v6/3999.onnx v6lp_3999=sweep/v6lp/3999.onnx --out results.md

A policy is `label=path` with an optional `@scale` (robotd's walking action_scale, 1.0) and `#kp`
(the legs' [bus] position_kp it was trained for, 16; the head stays at 16). `--friction`
runs every policy on that floor friction too (a dusty tile floor sits around 0.25-0.35).
"""
import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)

STAND = os.path.join(ROOT, "training/policies/hl2915_standup_v3bl_5999.onnx")
IMU_LAGS = (20.0, 25.0)  # --imu-lags overrides
BACKLASH = [("0.35", 0)] + [("0.3:1.0", s) for s in range(1, 6)]
WALK_S, PRE_S = 8.0, 1.0


def one(job):
    import sim_hl2915 as sh
    import videos_hl2915 as v
    label, path, scale, kp, friction, vx, lag, (bl, seed) = job
    a = v.sh_args(path, vx, PRE_S + WALK_S, "640x480", None, None, 1.0, v.SIDE)
    a.video = None
    a.pre_policy, a.pre_s, a.pre_kp_ratio = STAND, PRE_S, 0.8
    a.delay_ms, a.imu_lag_ms, a.vin, a.action_scale = 32.0, lag, 11.6, scale
    a.kp, a.friction = kp, friction
    a.backlash_deg, a.backlash_seed = bl, seed
    log = sh.run(a, a.scene, path, {"twist": [vx, 0, 0], "head": [0] * 4, "body": [0] * 3})
    w0 = int(PRE_S * 50)
    p, r = np.degrees(log["pitch"]), np.degrees(log["roll"])
    fall = next((i for i in range(w0, len(p)) if np.hypot(p[i], r[i]) > 30), None)
    end = fall if fall is not None else len(p)
    gy = np.degrees(log["gyro"][w0:end, 1])
    neck = np.degrees(log["target"][w0 + 10:end, 5])
    yaw = np.degrees(np.unwrap(log["yaw"]))
    yaw -= yaw[w0]
    y1 = yaw[min(w0 + 50, end - 1)]
    dist = float(np.hypot(log["x"][end - 1] - log["x"][w0], log["y"][end - 1] - log["y"][w0]))
    return dict(label=label, lag=lag, fell=fall is not None, rms=float(np.sqrt((gy ** 2).mean())),
                neck=float(neck.std()) if len(neck) > 2 else float("nan"), yaw1=abs(float(y1)),
                yaw8=abs(float(yaw[end - 1])), dist=dist)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("policies", nargs="+", help="label=path[@action_scale]")
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--imu-lags", default="20,25", help="IMU delays to run, ms")
    ap.add_argument("--friction", type=float, help="floor friction (default: the MJCF's 1.0)")
    ap.add_argument("--vx", type=float, default=0.2, help="walk command, m/s")
    ap.add_argument("--out", help="write the table here too")
    a = ap.parse_args()
    lags = [float(x) for x in a.imu_lags.split(",")]
    jobs = []
    for spec in a.policies:
        label, rest = spec.split("=", 1)
        rest, _, kp = rest.partition("#")
        path, _, scale = rest.partition("@")
        scale = scale or "1.0"
        jobs += [(label, path, float(scale), int(kp or 16), a.friction, a.vx, lag, bl) for lag in lags for bl in BACKLASH]
    with ProcessPoolExecutor(a.workers) as ex:
        res = list(ex.map(one, jobs))
    labels = list(dict.fromkeys(j[0] for j in jobs))
    n = len(lags) * len(BACKLASH)
    lines = [f"| policy | falls (of {n}) | pitch-rate RMS deg/s | neck std deg | yaw 1 s | yaw 8 s | distance m |",
             "|---|---|---|---|---|---|---|"]
    for label in labels:
        r = [x for x in res if x["label"] == label]
        m = lambda k: np.nanmean([x[k] for x in r])
        lines.append(f"| {label} | {sum(x['fell'] for x in r)} | {m('rms'):.0f} | {m('neck'):.1f} | "
                     f"{m('yaw1'):.0f} | {m('yaw8'):.0f} | {m('dist'):.2f} |")
    table = "\n".join(lines)
    print(table)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(table + "\n")


if __name__ == "__main__":
    main()
