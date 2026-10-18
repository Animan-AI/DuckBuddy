# -*- coding: utf-8 -*-
"""Identify the robot's command-to-motion delay from an in-air `robot.state` recording.

The training models the policy-to-servo delay as an actuator lag (2-6 physics steps, 10-30 ms;
v5 2-10), but the robot's own end-to-end figure — bus read, inference, write, the servo's
response — was never measured. This replays a recording into the BAM simulation with the trunk
held fixed (the robot was held in the air, so no contact to model) and finds the delay that
best reproduces the recorded joint motion.

In tick k robotd reads the joints (`joints`, at the tick's start), runs the policy and writes
`targets` (already through its low-pass). The replay applies targets[k] at t[k] + d and
compares the simulated encoder reading at t[k] with joints[k], so the fitted d is the whole
read -> compute -> write -> servo-moves latency. The low-pass lag comes on top of it (it is in
the targets), and the policy never trains with that either.

Record (next robot session): robot held in the air by the trunk, legs free; the walking policy
at 0.2-0.3 m/s for ~20 s, so the legs swing; `rpc.py record`. Then:

    python training/bam/replay_delay_hl2915.py rec_air.jsonl --kp 16 --kp-head 16 --vin 11.4
"""
import argparse
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)

import sim_hl2915 as sh  # noqa: E402

sw = sh.sw
# robot.state carries 15 joints: the 14 of the policy with the mouth at index 9.
STATE_TO_POLICY = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14]
LEGS = [i for i, n in enumerate(sw.POLICY_JOINTS) if n.startswith(("left_", "right_"))]


def load_recording(path, start=None, end=None):
    rows = [json.loads(line)["params"] for line in open(path, encoding="utf-8") if '"robot.state"' in line]
    rows = [r for r in rows if "targets" in r and "joints" in r and r.get("policy") != "held"
            and not r.get("safety", {}).get("limp", False)]
    t = np.array([r["t"] for r in rows])
    keep = np.ones(len(t), bool)
    if start is not None:
        keep &= t >= start
    if end is not None:
        keep &= t <= end
    rows = [r for r, k in zip(rows, keep) if k]
    t = np.array([r["t"] for r in rows])
    q = np.array([r["joints"] for r in rows])[:, STATE_TO_POLICY]
    target = np.array([r["targets"] for r in rows])[:, STATE_TO_POLICY]
    g = np.array([r["safety"]["gravity"] for r in rows])
    return t - t[0], q, target, g


def trunk_quat(g):
    """World orientation of the trunk that makes its body-frame gravity g (unit, z ~ -1 upright)."""
    g = g / np.linalg.norm(g)
    down = np.array([0.0, 0.0, -1.0])
    axis = np.cross(g, down)
    s, c = np.linalg.norm(axis), float(np.dot(g, down))
    if s < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    angle = np.arctan2(s, c)
    axis = axis / s
    return np.concatenate([[np.cos(angle / 2)], np.sin(angle / 2) * axis])


def simulate(a, t, q, target, quat, delay_ms):
    import mujoco
    import hl2915  # noqa: F401
    from bam.model import load_model
    from bam.mujoco import MujocoController

    def fix_base(spec):
        spec.joint("trunk_base_freejoint").delete()
        for k in list(spec.keys):
            k.delete()
        trunk = spec.body("trunk_base")
        trunk.pos = [0.0, 0.0, 0.5]  # well clear of the floor
        trunk.quat = list(quat)

    backlash = sh.backlash_halves(a.backlash_deg)
    model, data = sh.load_scene(a.scene, True, 0.001, backlash=backlash, edit=fix_base)
    qadr = np.array([model.jnt_qposadr[model.joint(n).id] for n in sw.POLICY_JOINTS])
    bqadr = (np.array([model.jnt_qposadr[model.joint(f"passive_{n}_backlash").id] for n in sw.POLICY_JOINTS])
             if backlash is not None else None)
    bam_model = load_model(a.params)
    act = bam_model.actuator
    act.vin, act.max_current = a.vin, a.current_ma / 1000.0
    act.kp = np.array([a.kp_head if n in sh.HEAD else a.kp for n in sw.POLICY_JOINTS], dtype=float)
    ctrl = MujocoController(bam_model, sw.POLICY_JOINTS, model, data)
    if bqadr is not None:
        control = act.compute_control
        act.compute_control = lambda tg, qq, dq, dt: control(tg, qq + data.qpos[bqadr], dq, dt)
    mujoco.mj_resetData(model, data)
    data.qpos[qadr] = q[0]
    mujoco.mj_forward(model, data)
    ctrl.reset(data.qpos)
    for n, v in zip(sw.POLICY_JOINTS, q[0]):
        ctrl.set_q_target(n, v)

    dt = model.opt.timestep
    steps = int(round(t[-1] / dt)) + 1
    apply_at = t + delay_ms / 1000.0  # targets[k] reaches the servo at t[k] + d
    sim_q = np.zeros_like(q)
    k_out, k_in = 0, -1
    for i in range(steps):
        now = i * dt
        while k_out < len(t) and t[k_out] <= now + 1e-9:  # the reading of tick k_out, taken now
            enc = data.qpos[qadr] + (data.qpos[bqadr] if bqadr is not None else 0.0)
            sim_q[k_out] = enc
            k_out += 1
        while k_in + 1 < len(t) and apply_at[k_in + 1] <= now + 1e-9:
            k_in += 1
            for n, v in zip(sw.POLICY_JOINTS, target[k_in]):
                ctrl.set_q_target(n, v)
        ctrl.update()
        mujoco.mj_step(model, data)
    sim_q[k_out:] = sim_q[k_out - 1]
    return sim_q


def score(args):
    a, t, q, target, quat, d = args
    sim_q = simulate(a, t, q, target, quat, d)
    skip = int(0.5 * 50)  # the first half second settles from rest
    err = np.degrees(sim_q[skip:] - q[skip:])
    return d, float(np.sqrt((err[:, LEGS] ** 2).mean())), np.sqrt((err ** 2).mean(0))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--scene", default=os.path.join(ROOT, "hardware/rev-a/mjcf/scene_walk.xml"))
    ap.add_argument("--params", default=os.path.join(ROOT, "training/bam/params/hl2915/m4.json"))
    ap.add_argument("--kp", type=float, default=16.0, help="legs, register 50 as deployed")
    ap.add_argument("--kp-head", type=float, default=16.0)
    ap.add_argument("--vin", type=float, default=11.4)
    ap.add_argument("--current-ma", type=float, default=1300.0)
    ap.add_argument("--backlash-deg", default="0.35")
    ap.add_argument("--start", type=float, help="recording time window, s (robot.state t)")
    ap.add_argument("--end", type=float)
    ap.add_argument("--delays", default="0:62:2", help="ms, start:stop:step")
    ap.add_argument("--workers", type=int, default=os.cpu_count())
    a = ap.parse_args()
    t, q, target, g = load_recording(a.recording, a.start, a.end)
    print(f"{len(t)} ticks, {t[-1]:.1f} s; trunk tilt {np.degrees(np.arccos(-g.mean(0)[2] / np.linalg.norm(g.mean(0)))):.1f}°, "
          f"gravity spread {np.degrees(g.std(0)).max():.1f}° (held still?)")
    lo, hi, step = (float(x) for x in a.delays.split(":"))
    delays = np.arange(lo, hi, step)
    quat = trunk_quat(g.mean(0))
    with ProcessPoolExecutor(a.workers) as ex:
        results = sorted(ex.map(score, [(a, t, q, target, quat, d) for d in delays]))
    best = min(results, key=lambda r: r[1])
    for d, e, _ in results:
        print(f"  delay {d:5.1f} ms  legs RMS {e:5.2f}°" + ("   <- best" if d == best[0] else ""))
    print(f"best {best[0]:.0f} ms; per joint at best: " +
          " ".join(f"{n.replace('left_', 'L').replace('right_', 'R')}:{e:.1f}" for n, e in zip(sw.POLICY_JOINTS, best[2])))
    print("Add robotd's low-pass group delay ((1-a)/a ticks: legs 0.7 -> ~9 ms, head 0.5 -> 20 ms) for what the "
          "policy sees, and compare with the training's actuator lag (2-6 or 2-10 steps of 5 ms).")


if __name__ == "__main__":
    main()
