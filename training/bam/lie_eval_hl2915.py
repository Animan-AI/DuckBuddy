# -*- coding: utf-8 -*-
"""LieDown acceptance in the robot-matched BAM simulation (2026-10-14, training/LIEDOWN-DESIGN-20261013-CN.md).

One run, as robotd's deep rest does it: stand 2 s (flag 0) -> lie (flag 1) 6 s -> torque off 3 s ->
torque on, targets blended back from where the joints lie (0.8 s) and held at flag 1 for 1 s ->
get up 5 s — on the stand-up network, as robotd gets up from the lie (`--rise lie` tries the lie
network's own flag 0 instead). Under the robot's conditions (IMU 20 / 25 ms late, 32 ms command delay, robotd
low-pass 0.5 / 0.7, gear play), on a normal floor and a grippy one (silicone pads), full-collision
rev-a model.

Reported per (floor): time to lie (legs+head within 0.1 rad of LIE on average and the trunk within 5 deg
of lying, held 0.5 s), the head's peak floor force, the servos' torque^2 lying at rest (limp-ready),
how far the trunk turns in the 3 s limp, and whether it stands again (upright, tall) at the end.
Acceptance: lies in <= 4 s, head peak < 5 N, limp change < 5 deg, stands up.

    python training/bam/lie_eval_hl2915.py training/policies/sweep/lie/3999.onnx --friction 1.0,2.0 --out lie.md
"""
import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)

# 14-joint (policy) LIE pose and the trunk's gravity lying, as the LieDown task trains them.
LIE_14 = np.array([0, 0, -0.782, 1.485, 0.227, -0.940, 0.466, 0, 0, 0, 0, 0.784, -1.484, -0.229])
LIE_G = np.array([0.442, 0.0, -0.897])
STAND_S, LIE_S, LIMP_S, BRACE_S, RISE_S = 2.0, 6.0, 3.0, 1.0, 5.0
BACKLASH = [("0.35", 0)] + [("0.3:1.0", s) for s in range(1, 6)]
LAGS = (20.0, 25.0)
STAND = os.path.join(ROOT, "training/policies/hl2915_standup_v3bl_5999.onnx")


def one(job):
    import sim_hl2915 as sh
    import videos_hl2915 as v
    path, friction, lag, (bl, seed), video, rise = job
    t_lie, t_limp = STAND_S, STAND_S + LIE_S
    t_unlimp, t_rise = t_limp + LIMP_S, t_limp + LIMP_S + BRACE_S
    total = t_rise + RISE_S
    a = v.sh_args(path, 0.0, total, "640x480", video, None, 1.0, v.SIDE)
    a.scene = os.path.join(ROOT, "hardware/rev-a/mjcf/scene.xml")
    a.delay_ms, a.imu_lag_ms, a.vin, a.action_scale = 32.0, lag, 11.6, 1.0
    a.kp, a.friction, a.ramp_s = 16, friction, 0.0
    a.head_lowpass, a.legs_lowpass = 0.5, 0.7
    a.backlash_deg, a.backlash_seed = bl, seed
    a.cam_follow_yaw = False
    flag = lambda t: 1.0 if t_lie <= t < t_rise else 0.0
    a.cmd_fn = lambda t: {"twist": [flag(t), 0, 0], "head": [0] * 4, "body": [0] * 3}
    a.limp_fn = lambda t: t_limp <= t < t_unlimp
    main = path
    if rise == "stand":
        # the lie network until the rise, then the stand-up network at full gain
        a.pre_policy, a.pre_s, a.pre_kp_ratio = path, t_rise, 1.0
        main = STAND
    log = sh.run(a, a.scene, main, {"twist": [0, 0, 0], "head": [0] * 4, "body": [0] * 3})
    t = log["t"]
    q, g = log["q"], log["g"]
    pose_err = np.abs(q - LIE_14).mean(axis=1)
    g_err = np.degrees(np.arccos(np.clip((g * LIE_G).sum(axis=1) / np.linalg.norm(g, axis=1), -1, 1)))
    there = (pose_err < 0.1) & (g_err < 5.0)
    i0, need = int(t_lie * 50), 25
    arrive = next((i for i in range(i0, int(t_limp * 50) - need) if there[i:i + need].all()), None)
    lie_window = slice(i0, int(t_limp * 50))
    # the trunk's turn while limp: lying orientation at the cut vs at the end of it
    a_cut, a_end = int(t_limp * 50) - 1, int(t_unlimp * 50) - 1
    c = np.dot(g[a_cut], g[a_end]) / (np.linalg.norm(g[a_cut]) * np.linalg.norm(g[a_end]))
    end = len(t) - 1
    stood = bool(log["gz"][end] < -0.94 and log["z"][end] > 0.10)
    rest = slice(int((t_limp - 1.0) * 50), int(t_limp * 50))
    return dict(key=friction, arrive=(t[arrive] - t_lie) if arrive is not None else np.nan,
                lied=arrive is not None, head_peak=float(log["head_f"][lie_window].max()),
                head_rise=float(log["head_f"][int(t_rise * 50):].max()),
                tau_sq=float(np.mean(log["tau_sq"][rest])), limp_turn=float(np.degrees(np.arccos(np.clip(c, -1, 1)))),
                pose_err=float(pose_err[a_cut]), g_err=float(g_err[a_cut]), stood=stood,
                z_lie=float(log["z"][a_cut]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("policy")
    ap.add_argument("--friction", default="1.0,2.0")
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--video", help="also render one run (friction 1.0, lag 20, backlash 0.35) here")
    ap.add_argument("--rise", choices=("stand", "lie"), default="stand")
    ap.add_argument("--out")
    a = ap.parse_args()
    jobs = [(a.policy, float(fr), lag, bl, None, a.rise) for fr in a.friction.split(",") for lag in LAGS for bl in BACKLASH]
    if a.video:
        jobs[0] = jobs[0][:4] + (a.video, a.rise)
    with ProcessPoolExecutor(a.workers) as ex:
        res = list(ex.map(one, jobs))
    n = len(LAGS) * len(BACKLASH)
    lines = [f"`{a.policy}` — stand 2 s, lie 6 s, limp 3 s, brace 1 s, get up 5 s; {n} runs per floor",
             "", f"| floor friction | lay down (of {n}) | time to lie s | head peak N (lying down) | head peak N (getting up) | "
             "torque^2 at rest | pose err rad | trunk err deg | trunk z m | limp turn deg (mean / max) | stood up (of n) |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for key in dict.fromkeys(x["key"] for x in res):
        r = [x for x in res if x["key"] == key]
        m = lambda k: np.nanmean([x[k] for x in r])
        mx = lambda k: np.nanmax([x[k] for x in r])
        lines.append(f"| {key} | {sum(x['lied'] for x in r)} | {m('arrive'):.1f} | {m('head_peak'):.1f} / {mx('head_peak'):.1f} | "
                     f"{m('head_rise'):.1f} | {m('tau_sq'):.3f} | {m('pose_err'):.2f} | {m('g_err'):.1f} | {m('z_lie'):.3f} | "
                     f"{m('limp_turn'):.1f} / {mx('limp_turn'):.1f} | {sum(x['stood'] for x in r)} |")
    table = "\n".join(lines)
    print(table)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(table + "\n")


if __name__ == "__main__":
    main()
