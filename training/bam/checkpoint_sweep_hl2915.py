# -*- coding: utf-8 -*-
"""Score every checkpoint of a walking run in the BAM simulation, to choose which one goes on
the robot instead of taking the last.

Upstream (microduck_rl docs/velstand_policy.md): checkpoints of one run differ a lot in yaw
although the training metrics do not — 500 turned right, 1499 left, 1250 walked straight —
so they pick the checkpoint by a walk sweep. Ours had always been the last one, and v2bl
turned 60-130° in 4 s on the robot.

Per checkpoint (ONNX, e.g. from `training/hl2915/ckpt_onnx.py`):
  straight-walk yaw rate and distance, nominal, backlash +-0.35° (measured), 0.2 and 0.3 m/s,
      both floors. A checkpoint must cover half the commanded distance to count: some treat
      0.2 m/s as "stand" and score zero yaw for the wrong reason (v2bl 2500 / 2750)
  falls / distance over stress_hl2915's conditions x backlash (0.35, 0.3-1.0) x floor (1.0, 0.5)

    python training/bam/checkpoint_sweep_hl2915.py sweep/v4bl/*.onnx --out training/data/sim-hl2915/sweep-v4bl
"""
import argparse
import os
import re
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import compare_walk_hl2915 as cw  # noqa: E402
from stress_hl2915 import CONDITIONS  # noqa: E402

NOMINAL = CONDITIONS[0]
WALKS = 0.5  # nominal distance / commanded distance, worst of both speeds and floors
BACKLASH = [("0.35", "0.35", 0), ("0.3-1.0", "0.3:1.0", 0)]
FLOORS = cw.FLOORS


def iteration(path):
    m = re.findall(r"(\d+)", os.path.basename(path))
    return int(m[-1]) if m else -1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("policies", nargs="+")
    ap.add_argument("--workers", type=int, default=os.cpu_count())
    ap.add_argument("--out", help="directory for summary.md and results.csv")
    a = ap.parse_args()
    policies = sorted(a.policies, key=iteration)

    yaw_jobs = [(p, NOMINAL, BACKLASH[0], f, True, vx) for p in policies for f in FLOORS for vx in (0.2, 0.3)]
    stress_jobs = [(p, c, b, f, True, 0.2) for p in policies for c in CONDITIONS for b in BACKLASH for f in FLOORS]
    jobs = yaw_jobs + stress_jobs
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(cw.job, *zip(*jobs)))
    yaw_rows, stress_rows = rows[:len(yaw_jobs)], rows[len(yaw_jobs):]

    lines = ["# Checkpoint sweep (BAM sim, robotd low-pass 0.5/0.7)", "",
             "yaw rate = yaw after 8 s / 8 s, nominal condition, backlash ±0.35°; falls over "
             f"{len(CONDITIONS)} conditions × {len(BACKLASH)} backlash × {len(FLOORS)} floors at 0.2 m/s.", "",
             "| checkpoint | yaw °/s 0.2 m/s (f1.0 / f0.5) | yaw °/s 0.3 m/s (f1.0 / f0.5) | max \\|yaw\\| °/s "
             "| walked / commanded (min) | falls | median dx m (stress) | worst condition |",
             "|---|---|---|---|---|---|---|---|"]
    summary = []
    for p in policies:
        n = os.path.basename(p)
        y = {(r["vx"], r["floor"]): r["yaw8"] / cw.DURATION for r in yaw_rows if r["policy"] == n}
        fell_yaw = sum(r["fell"] is not None for r in yaw_rows if r["policy"] == n)
        rs = [r for r in stress_rows if r["policy"] == n]
        falls = sum(r["fell"] is not None for r in rs)
        by_cond = {c[0]: sum(r["fell"] is not None for r in rs if r["cond"] == c[0]) for c in CONDITIONS}
        worst = max(by_cond, key=by_cond.get)
        ymax = max(abs(v) for v in y.values())
        # commanded distance: vx over the run, less half the 0.3 s command ramp
        walked = min(r["dx"] / (r["vx"] * (cw.DURATION - 0.15)) for r in yaw_rows if r["policy"] == n)
        summary.append((n, ymax, falls + fell_yaw, walked))
        lines.append(f"| {n} | {y[(0.2, 'f1.0')]:+.1f} / {y[(0.2, 'f0.5')]:+.1f} | {y[(0.3, 'f1.0')]:+.1f} / {y[(0.3, 'f0.5')]:+.1f} "
                     f"| {ymax:.1f} | {walked:.2f} | {falls}/{len(rs)}" + (f" (+{fell_yaw} in yaw runs)" if fell_yaw else "") +
                     f" | {np.median([r['dx'] for r in rs]):+.2f} | {worst + f' {by_cond[worst]}' if by_cond[worst] else '—'} |")
    ok = [s for s in summary if s[2] == 0 and s[3] >= WALKS]
    lines.append("")
    if ok:
        pick = min(ok, key=lambda s: s[1])
        lines.append(f"Least yaw among checkpoints that walk (≥ {WALKS:.0%} of the commanded distance) and never fall: "
                     f"**{pick[0]}** (max |yaw| {pick[1]:.1f} °/s).")
    else:
        lines.append(f"No checkpoint both walks (≥ {WALKS:.0%}) and never falls; pick by hand from the table.")
    text = "\n".join(lines) + "\n"
    print(text)
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        with open(os.path.join(a.out, "summary.md"), "w", encoding="utf-8") as f:
            f.write(text)
        import csv
        with open(os.path.join(a.out, "results.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
