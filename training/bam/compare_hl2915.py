# -*- coding: utf-8 -*-
"""Score fitted HL-2915 BAM models against the bench logs and plot sim vs real.

    uv run python compare_hl2915.py --logdir all --params params/hl2915/m4.json [more.json] \
        --plot compare.png

Prints the mean absolute position error per model, per Kp and per trajectory, and
plots a grid (rows: Kp, columns: trajectory) for the heaviest load with each model overlaid.
"""
import argparse
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hl2915  # noqa: E402,F401

from bam import simulate  # noqa: E402
from bam.logs import Logs  # noqa: E402
from bam.model import load_model  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--logdir", required=True)
ap.add_argument("--params", nargs="+", required=True)
ap.add_argument("--plot")
a = ap.parse_args()

logs = Logs(a.logdir).logs
results = {}
for path in a.params:
    model = load_model(path)
    sim = simulate.Simulator(model)
    rows = []
    for log in logs:
        q_sim = np.array(sim.rollout_log(log, simulate_control=True)[0])
        q_real = np.array([e["position"] for e in log["entries"]])
        rows.append((log, q_sim, q_real, float(np.mean(np.abs(q_sim - q_real)))))
    results[path] = rows

    name = os.path.basename(path)
    print(f"\n== {name}: MAE {np.degrees(np.mean([r[3] for r in rows])):.2f}°")
    by = defaultdict(list)
    for log, _, _, err in rows:
        by[("kp", log["kp"])].append(err)
        by[("traj", log["trajectory"])].append(err)
        by[("mass", log["mass"])].append(err)
    for key in sorted(by, key=str):
        print(f"   {key[0]:>4} {str(key[1]):<16} {np.degrees(np.mean(by[key])):5.2f}°")
    # Linearised stiffness, as the base class's to_mujoco would give it.
    act = model.actuator
    gain = act.error_gain * model.error_gain_ratio.value * act.vin * model.kt.value / model.R.value
    print(f"   kt {model.kt.value:.3f} N·m/A, R {model.R.value:.2f} Ω, armature {model.armature.value:.4f}, "
          f"stall @12.4 V {12.4 * model.kt.value / model.R.value:.2f} N·m, "
          f"no-load {12.4 / model.kt.value * 60 / (2 * np.pi):.0f} rpm")
    print("   stiffness N·m/rad: " + ", ".join(f"Kp{k} {gain * k:.2f}" for k in (8, 16, 24, 32)))

if a.plot:
    first = results[a.params[0]]
    heaviest = max(log["mass"] for log, *_ in first)
    kps = sorted({log["kp"] for log, *_ in first})
    trajs = sorted({log["trajectory"] for log, *_ in first})
    fig, axes = plt.subplots(len(kps), len(trajs), figsize=(4.2 * len(trajs), 2.6 * len(kps)),
                             sharex=True, squeeze=False)
    for i, kp in enumerate(kps):
        for j, tr in enumerate(trajs):
            ax = axes[i][j]
            plotted_real = False
            for path, rows in results.items():
                for log, q_sim, q_real, err in rows:
                    if log["kp"] != kp or log["trajectory"] != tr or log["mass"] != heaviest:
                        continue
                    t = np.arange(len(q_real)) * log["dt"]
                    if not plotted_real:
                        ax.plot(t, np.degrees([e["goal_position"] for e in log["entries"]]),
                                color="0.7", lw=0.8, label="goal")
                        ax.plot(t, np.degrees(q_real), color="k", lw=1.2, label="real")
                        plotted_real = True
                    ax.plot(t, np.degrees(q_sim), lw=1.0,
                            label=f"{os.path.basename(path)} ({np.degrees(err):.1f}°)")
            ax.set_title(f"Kp {kp} · {tr}", fontsize=9)
            ax.grid(alpha=0.3)
            ax.legend(fontsize=6, loc="best")
            if j == 0:
                ax.set_ylabel("deg")
    for ax in axes[-1]:
        ax.set_xlabel("s")
    fig.suptitle(f"HL-2915 bench, {heaviest * 1000:.0f} g load: real vs BAM sim", fontsize=11)
    fig.tight_layout()
    fig.savefig(a.plot, dpi=110)
    print(f"\nplot -> {a.plot}")
