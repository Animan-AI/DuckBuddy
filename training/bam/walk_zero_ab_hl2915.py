# -*- coding: utf-8 -*-
"""Same walking policy, the robot's old vs new hip zeros, in the robot-matched BAM simulation (2026-10-12).

On 2026-10-12 the hip roll / hip yaw zeros were re-done (in-air shank tilt vs the model, toes aligned by
hand). Taking the new zeros as the truth, the old ones left each joint physically past what it read:
    left_hip_yaw -5.0, left_hip_roll +3.4, right_hip_yaw +2.5, right_hip_roll -2.7 deg
(old offsets 168 / -48 / 41 / 260 -> new 111 / -9 / 70 / 229, sign -1, 4096 counts per turn): legs
straighter in front view, toes out. The first walk on the new zeros took smaller steps (0.53 m in 6 s
against 0.78-0.86 m the night before). This runs both under the robot's conditions — IMU 20 / 25 ms
late, 32 ms command delay, robotd low-pass 0.5 / 0.7, gear play, the stand network for 1 s at gain
x0.8, heading hold v2 — on a normal floor and on a grippy one (silicone pads), and reports distance,
yaw, rocking, hip-pitch / knee swing and step length.

    python training/bam/walk_zero_ab_hl2915.py training/policies/hl2915_walk_v6lp_slip_3999.onnx@0.9 \
        --friction 1.0,2.0 --vx 0.3 --out results.md
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
ZEROS = {
    "new": "",
    "old": "left_hip_yaw=-5.0,left_hip_roll=3.43,right_hip_yaw=2.55,right_hip_roll=-2.72",
    "old_roll": "left_hip_roll=3.43,right_hip_roll=-2.72",
    "old_yaw": "left_hip_yaw=-5.0,right_hip_yaw=2.55",
}
BACKLASH = [("0.35", 0)] + [("0.3:1.0", s) for s in range(1, 6)]
LAGS = (20.0, 25.0)
WALK_S, PRE_S = 6.0, 1.0


def one(job):
    import sim_hl2915 as sh
    import sim_walk as sw
    import videos_hl2915 as v
    zeros, path, scale, friction, vx, lag, (bl, seed) = job
    a = v.sh_args(path, vx, PRE_S + WALK_S, "640x480", None, None, 1.0, v.SIDE)
    a.video = None
    a.pre_policy, a.pre_s, a.pre_kp_ratio = STAND, PRE_S, 0.8
    a.delay_ms, a.imu_lag_ms, a.vin, a.action_scale = 32.0, lag, 11.6, scale
    a.kp, a.friction = 16, friction
    a.head_lowpass, a.legs_lowpass = 0.5, 0.7
    a.backlash_deg, a.backlash_seed = bl, seed
    a.heading_hold_gain, a.heading_hold_max, a.heading_hold_tau, a.heading_hold_deadband_deg = 0.5, 0.3, 0.5, 5.0
    a.zero_error_deg = ZEROS[zeros]
    log = sh.run(a, a.scene, path, {"twist": [vx, 0, 0], "head": [0] * 4, "body": [0] * 3})
    w0 = int(PRE_S * 50)
    p, r = np.degrees(log["pitch"]), np.degrees(log["roll"])
    fall = next((i for i in range(w0, len(p)) if np.hypot(p[i], r[i]) > 30), None)
    end = fall if fall is not None else len(p)
    s0 = w0 + 50  # skip the first second of the walk for the gait shape
    q = np.degrees(log["q"][s0:end])
    j = sw.POLICY_JOINTS
    swing = lambda n: float(np.percentile(q[:, j.index(n)], 95) - np.percentile(q[:, j.index(n)], 5)) if len(q) > 10 else np.nan
    hp = q[:, j.index("left_hip_pitch")] - q[:, j.index("left_hip_pitch")].mean() if len(q) > 10 else np.zeros(2)
    fr = np.fft.rfftfreq(len(hp), 0.02)
    f = float(fr[1:][np.abs(np.fft.rfft(hp))[1:].argmax()]) if len(hp) > 10 else np.nan
    yaw = np.degrees(np.unwrap(log["yaw"]))
    yaw -= yaw[w0]
    dist = float(np.hypot(log["x"][end - 1] - log["x"][w0], log["y"][end - 1] - log["y"][w0]))
    d1 = float(np.hypot(log["x"][end - 1] - log["x"][s0], log["y"][end - 1] - log["y"][s0])) if end > s0 else 0.0
    steps = 2 * f * (end - s0) / 50.0
    # net yaw per stride (left hip pitch upward zero crossings): the kicks the heading wanders by
    lhp = q[:, j.index("left_hip_pitch")] - q[:, j.index("left_hip_pitch")].mean() if len(q) > 10 else np.zeros(2)
    cr = [i for i in range(1, len(lhp)) if lhp[i - 1] < 0 <= lhp[i]]
    yw = yaw[s0:end]
    stride_yaw = float(np.std(np.diff(yw[cr]))) if len(cr) > 3 else np.nan
    return dict(key=(zeros, friction), fell=fall is not None, dist=dist, yaw=abs(float(yaw[end - 1])),
                rms=float(np.sqrt((np.degrees(log["gyro"][w0:end, 1]) ** 2).mean())),
                yawrate=float(np.percentile(np.abs(np.degrees(log["gyro"][w0:end, 2])), 95)),
                hip=(swing("left_hip_pitch") + swing("right_hip_pitch")) / 2,
                knee=(swing("left_knee") + swing("right_knee")) / 2, freq=f,
                # the head swung as a counterweight (v9 on the robot 15 deg, v7 25): head_yaw's heat
                head_yaw=swing("head_yaw"),
                # mirror-symmetric gait: mean left hip yaw = -mean right (slip3999 on the robot: -3.5 / 0)
                stride_yaw=stride_yaw,
                yaw_asym=float(q[:, j.index("left_hip_yaw")].mean() + q[:, j.index("right_hip_yaw")].mean()) if len(q) > 10 else np.nan,
                step=d1 / steps * 100 if steps > 0 else np.nan)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("policy", help="path[@action_scale]")
    ap.add_argument("--zeros", default="new,old,old_roll,old_yaw")
    ap.add_argument("--friction", default="1.0,2.0")
    ap.add_argument("--vx", type=float, default=0.3)
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--out")
    a = ap.parse_args()
    path, _, scale = a.policy.partition("@")
    jobs = [(z, path, float(scale or 1.0), float(fr), a.vx, lag, bl)
            for fr in a.friction.split(",") for z in a.zeros.split(",") for lag in LAGS for bl in BACKLASH]
    with ProcessPoolExecutor(a.workers) as ex:
        res = list(ex.map(one, jobs))
    n = len(LAGS) * len(BACKLASH)
    lines = [f"`{os.path.basename(path)}` x{scale or 1.0}, {a.vx} m/s for {WALK_S:.0f} s, heading hold v2, mean of {n} runs",
             "", f"| zeros | floor friction | falls (of {n}) | distance m | yaw end deg | rocking RMS deg/s | yaw rate p95 deg/s | "
             "hip pitch swing deg | knee swing deg | step Hz | step length cm | hip yaw L+R deg | stride yaw std deg | "
             "head yaw swing deg |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for key in dict.fromkeys(x["key"] for x in res):
        r = [x for x in res if x["key"] == key]
        m = lambda k: np.nanmean([x[k] for x in r])
        lines.append(f"| {key[0]} | {key[1]} | {sum(x['fell'] for x in r)} | {m('dist'):.2f} | {m('yaw'):.0f} | {m('rms'):.0f} | "
                     f"{m('yawrate'):.0f} | {m('hip'):.1f} | {m('knee'):.1f} | {m('freq'):.2f} | {m('step'):.1f} | {m('yaw_asym'):+.1f} | {m('stride_yaw'):.1f} | "
                     f"{m('head_yaw'):.1f} |")
    table = "\n".join(lines)
    print(table)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(table + "\n")


if __name__ == "__main__":
    main()
