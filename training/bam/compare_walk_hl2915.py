# -*- coding: utf-8 -*-
"""Compare walking policies in the BAM simulation with gear backlash and a slippery floor.

`stress_hl2915.py` runs without backlash, and there v2, v2bl and v3bl all walked while the
robot told them apart (2026-10-08: only v2bl walked its 4 s). This adds the backlash the
training models (`sim_hl2915.py --backlash-deg`: a passive hinge per servo, the encoder
reading through it) and a glazed-tile floor, over the same leg-gain / head / delay stress:

  backlash  none | +-0.35° (measured on the robot, 2026-10-08) | +-1.0° (v2bl's training)
            | per joint 0.3-1.0° (v4's training), three seeds
  floor     friction 1.0 (MJCF default) | 0.5

Reports, per run, the fall time, distance and the yaw after 4 s of walking (the robot test
length; v2bl turned +68° and +131° there), and per policy the fall count by backlash.
Every run uses robotd's deployed target low-pass (head 0.5 / legs 0.7) unless --no-lowpass.

    python training/bam/compare_walk_hl2915.py training/policies/hl2915_walk_v2bl_3999.onnx \
        training/policies/hl2915_walk_v4bl_3999.onnx --out training/data/sim-hl2915/v4-compare
"""
import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)

import sim_hl2915 as sh  # noqa: E402
from stress_hl2915 import CONDITIONS  # noqa: E402

BACKLASH = [("none", "", 0), ("0.35", "0.35", 0), ("1.0", "1.0", 0)] + \
    [(f"0.3-1.0 s{s}", "0.3:1.0", s) for s in range(3)]
FLOORS = [("f1.0", None), ("f0.5", 0.5)]
VX, DURATION, YAW_AT = 0.2, 8.0, 4.0


def args_for(policy, extra, delay, backlash, seed, friction, lowpass, vx=VX):
    a = argparse.Namespace(
        scene=os.path.join(ROOT, "hardware/rev-a/mjcf/scene_walk.xml"), policy=policy,
        params=os.path.join(ROOT, "training/bam/params/hl2915/m4.json"), position=False,
        kp=16, kp_head=16, vin=10.7, current_ma=1300.0, delay_ms=delay, timestep=0.001,
        settle=1.0, settle_kp=32, action_scale=1.0, vx=vx, duration=DURATION, video=None,
        kp_scale="", head_mass_scale=1.0, head_com_forward_mm=0.0, ramp_s=0.3,
        head_lowpass=0.5 if lowpass else 1.0, legs_lowpass=0.7 if lowpass else 1.0,
        backlash_deg=backlash, backlash_seed=seed, friction=friction)
    it = iter(extra)  # stress_hl2915's CONDITIONS are CLI flags
    for flag in it:
        key = flag.lstrip("-").replace("-", "_")
        setattr(a, key, type(getattr(a, key))(next(it)))
    return a


def job(policy, cond, bl, floor, lowpass, vx=VX):
    (label, extra, delay), (bl_label, bl_spec, seed), (floor_label, friction) = cond, bl, floor
    cmd = {"twist": [vx, 0.0, 0.0], "head": [0.0] * 4, "body": [0.0] * 3}
    log = sh.run(args_for(policy, extra, delay, bl_spec, seed, friction, lowpass, vx),
                 os.path.join(ROOT, "hardware/rev-a/mjcf/scene_walk.xml"), policy, cmd)
    pitch, z = np.degrees(log["pitch"]), log["z"]
    down = (z < 0.05) | (np.abs(pitch) > 45)  # same as stress_hl2915
    fell = float(np.argmax(down) / 50.0) if down.any() else None
    yaw = np.degrees(np.unwrap(log["yaw"]) - log["yaw"][0])
    i4 = min(int(YAW_AT * 50), len(yaw) - 1)
    return dict(policy=os.path.basename(policy), cond=label, backlash=bl_label, floor=floor_label, vx=vx,
                fell=fell, dx=float(log["x"][-1] - log["x"][0]), yaw4=float(yaw[i4]),
                yaw8=float(yaw[-1]), lean_max=float(pitch.max()),
                neck_min=float(np.degrees(log["q"][:, 5]).min()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("policies", nargs="+")
    ap.add_argument("--no-lowpass", action="store_true")
    ap.add_argument("--workers", type=int, default=os.cpu_count())
    ap.add_argument("--out", help="directory for results.csv and summary.md")
    a = ap.parse_args()
    jobs = [(p, c, b, f, not a.no_lowpass) for p in a.policies for c in CONDITIONS for b in BACKLASH for f in FLOORS]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(job, *zip(*jobs)))

    lines = [f"# BAM walk comparison, vx {VX} m/s, {DURATION:.0f} s, low-pass {'off' if a.no_lowpass else '0.5/0.7'}", ""]
    names = [os.path.basename(p) for p in a.policies]
    # Summary: falls and yaw per backlash setting, over conditions x floors.
    lines += ["## Falls / runs and median |yaw @4 s|, by backlash", "",
              "| backlash | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for bl_label, _, _ in BACKLASH:
        cells = []
        for n in names:
            rs = [r for r in rows if r["policy"] == n and r["backlash"] == bl_label]
            falls = sum(r["fell"] is not None for r in rs)
            yaw = np.median([abs(r["yaw4"]) for r in rs if r["fell"] is None] or [np.nan])
            cells.append(f"{falls}/{len(rs)} falls, yaw {yaw:.0f}°")
        lines.append(f"| {bl_label} | " + " | ".join(cells) + " |")
    lines += ["", "## Falls / runs by condition (all backlash, both floors)", "",
              "| condition | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for label, _, _ in CONDITIONS:
        cells = []
        for n in names:
            rs = [r for r in rows if r["policy"] == n and r["cond"] == label]
            cells.append(f"{sum(r['fell'] is not None for r in rs)}/{len(rs)}, dx {np.median([r['dx'] for r in rs]):+.2f}")
        lines.append(f"| {label} | " + " | ".join(cells) + " |")
    lines += ["", "## Every run", "", "| policy | condition | backlash | floor | result | dx m | yaw@4s | yaw@8s | lean max | neck min |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        res = "walks" if r["fell"] is None else f"falls {r['fell']:.1f} s"
        lines.append(f"| {r['policy']} | {r['cond']} | {r['backlash']} | {r['floor']} | {res} | {r['dx']:+.2f} | "
                     f"{r['yaw4']:+.0f} | {r['yaw8']:+.0f} | {r['lean_max']:.1f} | {r['neck_min']:+.0f} |")
    text = "\n".join(lines) + "\n"
    print("\n".join(lines[:len(BACKLASH) + len(CONDITIONS) + 12]))
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
