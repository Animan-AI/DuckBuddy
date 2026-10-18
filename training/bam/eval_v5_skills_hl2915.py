# -*- coding: utf-8 -*-
"""Two checks the skill eval (`eval_skills_hl2915.py`) does not make, in the same BAM simulation and
stress conditions (2026-10-18):

  stand    a stand-up policy standing still (from the standing pose, zero command): left / right
           symmetry of its leg targets over the last 3 s - left + right per pair (the model mirrors
           every leg joint with a sign flip, so 0 is symmetric) - and the trunk's roll. On the robot
           standup_v3bl (no mirror loss) stood with the right foot ~1 deg forward and the right hip
           roll loaded (2026-10-17).
  pick     a GroundPick policy (the cloth pick) through two 4 s cycles from standing, the phase clock
           in the twist slot as robotd's ground_pick plays it: per cycle the mouth tip's lowest
           height and how long it stays within 1.5 cm of the floor in the bottom hold, the head's
           floor force, the trunk's tilt, a fall, and whether it is standing again at the end.
           No cloth and no payload in this sim.

    python training/bam/eval_v5_skills_hl2915.py --stand A.onnx B.onnx --pick C.onnx [--workers 12]
"""
import argparse
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim_hl2915 as sh  # noqa: E402
from eval_skills_hl2915 import ROBOTD_LOWPASS, SCENE, ZERO, args_for, standing  # noqa: E402
from stress_hl2915 import CONDITIONS  # noqa: E402

PAIRS = ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle"]   # policy order: left 0-4, right 9-13
PERIOD = 4.0
HOLD = (0.375, 0.525)   # the -Cloth- bottom, as a fraction of the period
NEAR_M = 0.015


def stand_job(policy, cond, backlash):
    label, extra, delay = cond
    log = sh.run(args_for(policy, extra, delay, "stand", 8.0, None, ROBOTD_LOWPASS, backlash), SCENE, policy, ZERO)
    tg = np.degrees(np.asarray(log["target"])[-150:])
    q = np.degrees(np.asarray(log["q"])[-150:])
    g = np.asarray(log["g"])[-150:].mean(0)
    roll = math.degrees(math.atan2(-g[1], -g[2]))
    sums = {p: (tg[:, i].mean() + tg[:, i + 9].mean(), q[:, i].mean() + q[:, i + 9].mean()) for i, p in enumerate(PAIRS)}
    return os.path.basename(os.path.dirname(policy)) + "/" + os.path.basename(policy), label, sums, roll


def pick_cmd(t):
    ph = (t % PERIOD) / PERIOD
    return dict(ZERO, twist=[math.cos(2 * math.pi * ph), math.sin(2 * math.pi * ph), 0.0])


def pick_job(policy, cond, backlash):
    label, extra, delay = cond
    log = sh.run(args_for(policy, extra, delay, "stand", 2 * PERIOD, pick_cmd, ROBOTD_LOWPASS, backlash),
                 SCENE, policy, ZERO)
    t = np.asarray(log["t"])
    mz = np.asarray(log.get("mouth_z", [np.nan] * len(t)))
    up = standing(log)
    gz = np.asarray(log["gz"])
    head_f = np.asarray(log["head_f"])
    out = []
    for c in range(2):
        ph = (t - c * PERIOD) / PERIOD
        cyc = (ph >= 0) & (ph < 1)
        hold = cyc & (ph >= HOLD[0]) & (ph < HOLD[1])
        lowest = float(np.nanmin(mz[cyc])) if cyc.any() else float("nan")
        near_s = float(((mz < NEAR_M) & hold).sum()) / 50.0
        tilt = float(np.degrees(np.arccos(np.clip(-gz[cyc], -1, 1))).max())
        back = bool(up[cyc & (ph >= 0.9)].all())
        fell = bool((gz[cyc] > -0.5).any())
        out.append((lowest, near_s, float(head_f[cyc].max()), tilt, back, fell))
    return os.path.basename(os.path.dirname(policy)) + "/" + os.path.basename(policy), label, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stand", nargs="*", default=[])
    ap.add_argument("--pick", nargs="*", default=[])
    ap.add_argument("--backlash-deg", default="0.35")
    ap.add_argument("--workers", type=int, default=os.cpu_count())
    a = ap.parse_args()
    with ProcessPoolExecutor(a.workers) as ex:
        stands = [ex.submit(stand_job, p, c, a.backlash_deg) for p in a.stand for c in CONDITIONS]
        picks = [ex.submit(pick_job, p, c, a.backlash_deg) for p in a.pick for c in CONDITIONS]
        if stands:
            print("## Standing: left + right of the leg targets (deg, 0 = symmetric), last 3 s; trunk roll\n")
            print("| policy | condition | hip yaw | hip roll | hip pitch | knee | ankle | roll deg |")
            print("|---|---|---|---|---|---|---|---|")
            for f in stands:
                name, label, sums, roll = f.result()
                print(f"| {name} | {label} | " + " | ".join(f"{sums[p][0]:+.2f}" for p in PAIRS) + f" | {roll:+.2f} |")
        if picks:
            print("\n## Cloth pick: per 4 s cycle (two cycles)\n")
            print("| policy | condition | lowest mouth tip cm | s within 1.5 cm in the hold | head force N | "
                  "max tilt deg | standing at the end | fell |")
            print("|---|---|---|---|---|---|---|---|")
            for f in picks:
                name, label, out = f.result()
                cols = [" / ".join(fmt(o[k]) for o in out) for k, fmt in
                        ((0, lambda v: f"{100 * v:.1f}"), (1, lambda v: f"{v:.2f}"), (2, lambda v: f"{v:.1f}"),
                         (3, lambda v: f"{v:.0f}"), (4, lambda v: "yes" if v else "NO"),
                         (5, lambda v: "FELL" if v else "no"))]
                print(f"| {name} | {label} | " + " | ".join(cols) + " |")


if __name__ == "__main__":
    main()
