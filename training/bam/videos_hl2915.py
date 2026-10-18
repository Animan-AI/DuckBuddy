# -*- coding: utf-8 -*-
"""Walking videos from the BAM simulation: plain walking, and pushes while walking and standing.

Every clip runs as the robot would: Kp 16, 30 ms command delay (32 ms measured in the air on
2026-10-09), +-0.35° gear play, robotd's target low-pass (head 0.5 / legs 0.7), 10.7 V. Pushes
are what the training gives (`push_by_setting_velocity`: the trunk's velocity jumps by up to
0.3 m/s, x and y), drawn as a red arrow; the "beyond training" clip uses 0.5 m/s.

    python training/bam/videos_hl2915.py --out training/data/sim-hl2915/videos
    python training/bam/videos_hl2915.py --only walk-pushes --walk training/policies/<other>.onnx
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
from stress_hl2915 import HEAVY, legs  # noqa: E402

WALK = os.path.join(ROOT, "training/policies/hl2915_walk_v5bl_delay50_2000.onnx")
STAND = os.path.join(ROOT, "training/policies/hl2915_standup_v4bl_5999.onnx")
SIDE = dict(cam_azimuth=90.0, cam_elevation=-8.0, cam_distance=0.6)
FRONT = dict(cam_azimuth=150.0, cam_elevation=-12.0, cam_distance=0.65)
# forward, back, left, right, 2 s apart
FOUR = "2:{v},0;4:-{v},0;6:0,{v};8:0,-{v}"

# name: (caption, policy key, vx, extra sim flags, camera, playback speed)
CLIPS = {
    "walk-side": ("正常行走 0.2 m/s（侧面）", "walk", 0.2, [], SIDE, 1.0),
    "walk-front": ("正常行走 0.2 m/s（前侧）", "walk", 0.2, [], FRONT, 1.0),
    "walk-side-slow": ("正常行走 0.2 m/s（侧面，0.5 倍速）", "walk", 0.2, [], SIDE, 0.5),
    "walk-pushes": ("行走中推 0.3 m/s：前 / 后 / 左 / 右", "walk", 0.2, ["--push", FOUR.format(v=0.3)], FRONT, 1.0),
    "walk-pushes-side": ("行走中推 0.3 m/s（侧面）", "walk", 0.2, ["--push", FOUR.format(v=0.3)], SIDE, 1.0),
    "stand-pushes": ("站立（起身策略）推 0.3 m/s", "stand", 0.0, ["--push", FOUR.format(v=0.3)], FRONT, 1.0),
    "walk-pushes-strong": ("行走中推 0.5 m/s（超出训练范围）", "walk", 0.2, ["--push", FOUR.format(v=0.5)], FRONT, 1.0),
    "stress-walk-pushes": ("腿刚度 ×0.5 + 头重 ×1.3、延迟 40 ms，推 0.3 m/s", "walk", 0.2,
                           ["--push", FOUR.format(v=0.3), "--kp-scale", legs(0.5), "--delay-ms", "40"] + HEAVY,
                           FRONT, 1.0),
    # robotd's IMU path as measured on the robot (2026-10-10): median of 3 ticks plus ~10 ms
    "walk-imu-lag": ("机器人的 IMU 路径：中值滤波 + 10 ms", "walk", 0.2, ["--imu-median", "1", "--imu-lag-ms", "10"],
                     FRONT, 1.0),
    "walk-imu-lag-side": ("机器人的 IMU 路径：中值滤波 + 10 ms（侧面）", "walk", 0.2,
                          ["--imu-median", "1", "--imu-lag-ms", "10"], SIDE, 1.0),
}


def render(name, policy, out, size, duration):
    caption, _, vx, extra, camera, speed = CLIPS[name]
    a = sh_args(policy, vx, duration, size, os.path.join(out, name + ".mp4"), caption, speed, camera)
    it = iter(extra)
    for flag in it:
        key = flag.lstrip("-").replace("-", "_")
        setattr(a, key, type(getattr(a, key))(next(it)))
    cmd = {"twist": [vx, 0.0, 0.0], "head": [0.0] * 4, "body": [0.0] * 3}
    log = sh.run(a, a.scene, policy, cmd)
    pitch = np.degrees(log["pitch"])
    yaw = np.degrees(np.unwrap(log["yaw"]) - log["yaw"][0])
    fell = "—" if log["fell"] is None else f"{log['fell']:.1f} s"
    return (f"| {name} | {os.path.basename(policy)} | {fell} | {log['x'][-1] - log['x'][0]:+.2f} | "
            f"{pitch.min():+.0f} / {pitch.max():+.0f} | {yaw[-1]:+.0f} |")


def sh_args(policy, vx, duration, size, video, caption, speed, camera):
    import argparse as ap
    return ap.Namespace(
        scene=os.path.join(ROOT, "hardware/rev-a/mjcf/scene_walk.xml"), policy=policy,
        params=os.path.join(ROOT, "training/bam/params/hl2915/m4.json"), position=False,
        kp=16, kp_head=16, vin=10.7, current_ma=1300.0, delay_ms=30.0, timestep=0.001,
        settle=1.0, settle_kp=32, action_scale=1.0, vx=vx, duration=duration, kp_scale="",
        head_mass_scale=1.0, head_com_forward_mm=0.0, ramp_s=0.3, head_lowpass=0.5, legs_lowpass=0.7,
        backlash_deg="0.35", backlash_seed=0, friction=None, push="", imu_median=False, imu_lag_ms=0.0, video=video, video_size=size,
        caption=caption, video_speed=speed, cam_follow_yaw=True, **camera)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default=os.path.join(ROOT, "training/data/sim-hl2915/videos"))
    p.add_argument("--walk", default=WALK)
    p.add_argument("--stand", default=STAND)
    p.add_argument("--only", nargs="*", choices=list(CLIPS))
    p.add_argument("--size", default="960x540")
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--workers", type=int, default=4)
    a = p.parse_args()
    os.makedirs(a.out, exist_ok=True)
    names = a.only or list(CLIPS)
    policies = {"walk": a.walk, "stand": a.stand}
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(render, names, [policies[CLIPS[n][1]] for n in names], [a.out] * len(names),
                           [a.size] * len(names), [a.duration] * len(names)))
    print("| 片段 | 策略 | 摔倒 | 前进 m | 俯仰 min / max ° | 偏航 ° |\n|---|---|---|---|---|---|")
    print("\n".join(rows))


if __name__ == "__main__":
    main()
