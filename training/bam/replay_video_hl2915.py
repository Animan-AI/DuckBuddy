# -*- coding: utf-8 -*-
"""Render a robot recording (robot.state jsonl) as a video of the MuJoCo model: the measured joint
angles, the trunk attitude from the IMU's gravity and the yaw from odometry, posed kinematically
(no physics). The robot's own motion side by side with the simulation's.

Height, x and y follow robotd's odometry.

    python training/bam/replay_video_hl2915.py training/data/2026-10-09-hl2915-v4test/t_tape_floor_v4lf_a.jsonl \
        --out replay.mp4 --before 1.0 --after 1.5
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "training"))

import sim_hl2915 as sh  # noqa: E402
import sim_walk as sw  # noqa: E402

STATE_TO_POLICY = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14]


def load(path):
    rows = []
    for line in open(path):
        if '"robot.state"' in line:
            try:
                rows.append(json.loads(line)["params"])
            except json.JSONDecodeError:
                pass  # a truncated last line
    return [r for r in rows if "move" in r and "odom" in r]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--out", required=True)
    ap.add_argument("--before", type=float, default=1.0, help="seconds before the walk starts")
    ap.add_argument("--after", type=float, default=5.0, help="seconds after the walk starts")
    ap.add_argument("--caption", default="")
    ap.add_argument("--video-size", default="960x540")
    ap.add_argument("--cam-azimuth", type=float, default=90.0)
    ap.add_argument("--cam-elevation", type=float, default=-8.0)
    ap.add_argument("--cam-distance", type=float, default=0.6)
    ap.add_argument("--speed", type=float, default=1.0)
    a = ap.parse_args()

    import imageio.v2 as iio
    import mujoco
    rows = load(a.recording)
    vx = np.array([r["move"]["applied"][0] for r in rows])
    i0 = int(np.argmax(vx > 0.0))
    lo, hi = max(0, i0 - int(a.before * 50)), min(len(rows), i0 + int(a.after * 50))

    model, data = sh.load_scene(os.path.join(ROOT, "hardware/rev-a/mjcf/scene_walk.xml"), False, 0.001)
    qadr = [model.jnt_qposadr[model.joint(n).id] for n in sw.POLICY_JOINTS]
    w, h = (int(x) for x in a.video_size.split("x"))
    model.vis.global_.offwidth, model.vis.global_.offheight = max(w, 640), max(h, 480)
    renderer = mujoco.Renderer(model, height=h, width=w)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation = a.cam_distance, a.cam_elevation
    frames, cam_yaw = [], None
    yaw0 = rows[lo]["odom"]["yaw"]
    p0 = np.array(rows[lo]["odom"]["position"][:2])
    for i in range(lo, hi):
        r = rows[i]
        g = np.array(r["safety"]["gravity"])
        pitch, roll = np.arctan2(g[0], np.hypot(g[1], g[2])), np.arctan2(-g[1], -g[2])
        yaw = r["odom"]["yaw"] - yaw0
        data.qpos[:] = model.qpos0
        data.qpos[qadr] = np.array(r["joints"])[STATE_TO_POLICY]
        quat = np.zeros(4)
        # yaw first about world z, then pitch and roll in the trunk frame
        qy = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
        qpr = np.zeros(4)
        mujoco.mju_euler2Quat(qpr, [roll, pitch, 0.0], "xyz")
        mujoco.mju_mulQuat(quat, qy, qpr)
        data.qpos[3:7] = quat
        c, s = np.cos(-yaw0), np.sin(-yaw0)
        d = np.array(r["odom"]["position"][:2]) - p0
        data.qpos[0:2] = (c * d[0] - s * d[1], s * d[0] + c * d[1])
        data.qpos[2] = r["odom"]["position"][2]  # robotd's kinematic trunk height
        mujoco.mj_forward(model, data)  # lights and cameras too, not only the bodies
        cam.lookat[:] = (data.qpos[0], data.qpos[1], 0.11)
        yaw_deg = np.degrees(yaw)
        cam_yaw = yaw_deg if cam_yaw is None else cam_yaw + 0.05 * ((yaw_deg - cam_yaw + 180) % 360 - 180)
        cam.azimuth = a.cam_azimuth + cam_yaw
        renderer.update_scene(data, camera=cam)
        frame = renderer.render()
        t = (i - i0) / 50.0
        label = a.caption or os.path.basename(a.recording)
        state = "站立" if t < 0 else ("行走" if r.get("policy") == "walk" else str(r.get("policy")))
        frame = sh.caption_frame(frame, f"{label}  [{state}]", t, pitch, None, None)
        frames.append(frame)
    renderer.close()
    iio.mimsave(a.out, frames, fps=int(50 * a.speed), macro_block_size=1)
    print(a.out, len(frames), "frames")


if __name__ == "__main__":
    main()
